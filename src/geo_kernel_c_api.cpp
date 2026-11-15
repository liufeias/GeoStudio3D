#include "geo_kernel/geo_kernel_c_api.h"

#include <cstdio>
#include <cstring>
#include <filesystem>
#include <memory>
#include <mutex>
#include <new>
#include <shared_mutex>
#include <string>
#include <string_view>
#include <vector>

#if defined(_WIN32)
    #ifndef NOMINMAX
        #define NOMINMAX
    #endif
    #ifndef WIN32_LEAN_AND_MEAN
        #define WIN32_LEAN_AND_MEAN
    #endif
    #include <fcntl.h>
    #include <io.h>
    #include <windows.h>
#endif

#include "geo_kernel/core_types.hpp"
#include "geo_kernel/dat_parser.hpp"
#include "geo_kernel/earthwork_calculator.hpp"
#include "geo_kernel/exact_predicates.hpp"
#include "geo_kernel/point_octree.hpp"
#include "geo_kernel/slope_grading.hpp"
#include "geo_kernel/tiled_tin.hpp"
#include "geo_kernel/tin_editor.hpp"

namespace {

constexpr int32_t kMaxSurfaceSlots = 4;

thread_local std::string g_last_error_message;

std::mutex g_log_callback_mutex;
GeoLogCallback g_log_callback = nullptr;
void* g_log_callback_user_data = nullptr;

/**
 * @brief Windows UCRT 标准流文本模式安全守卫
 *
 * 根因防护：
 *   Windows 通用 C 运行时（尤其是 Debug 版 ucrtbased.dll）在任意窄字符 stdio 输出
 *   （printf / fprintf / std::cout / std::cerr）执行时，会强制断言校验：
 *     (_textmode_safe(fn) == __crt_lowio_text_mode::ansi) && !_tm_unicode_safe(fn)
 *   本函数仅在文件描述符有效且关联真实操作系统句柄时，将其安全恢复为 _O_TEXT 模式。
 */
void EnsureSafeWindowsStdioMode() noexcept {
#if defined(_WIN32)
    auto sanitize_stream = [](FILE* stream) noexcept {
        if (stream == nullptr) return;
        const int fd = _fileno(stream);
        if (fd < 0) return;
        const intptr_t os_handle = _get_osfhandle(fd);
        if (os_handle == -1 || os_handle == 0) return;
        (void)_setmode(fd, _O_TEXT);
    };
    sanitize_stream(stdout);
    sanitize_stream(stderr);
#endif
}

/**
 * @brief 将 UTF-8 字符串直接输出到 Windows 控制台或标准错误句柄（完全绕过 UCRT FILE* 断言）
 */
void WriteUtf8ToStderrBypassUcrt(std::string_view utf8_line) noexcept {
#if defined(_WIN32)
    HANDLE hErr = GetStdHandle(STD_ERROR_HANDLE);
    if (hErr == nullptr || hErr == INVALID_HANDLE_VALUE) {
        return;
    }
    DWORD console_mode = 0;
    if (GetConsoleMode(hErr, &console_mode)) {
        const int wlen = MultiByteToWideChar(
            CP_UTF8, 0, utf8_line.data(), static_cast<int>(utf8_line.size()), nullptr, 0);
        if (wlen > 0) {
            std::wstring wbuf(static_cast<size_t>(wlen), L'\0');
            MultiByteToWideChar(
                CP_UTF8, 0, utf8_line.data(), static_cast<int>(utf8_line.size()), wbuf.data(), wlen);
            DWORD written = 0;
            WriteConsoleW(hErr, wbuf.data(), static_cast<DWORD>(wbuf.size()), &written, nullptr);
        }
    } else {
        DWORD written = 0;
        WriteFile(hErr, utf8_line.data(), static_cast<DWORD>(utf8_line.size()), &written, nullptr);
    }
#else
    if (!utf8_line.empty()) {
        (void)std::fwrite(utf8_line.data(), 1, utf8_line.size(), stderr);
        (void)std::fflush(stderr);
    }
#endif
}

void SetLastError(const std::string& msg) {
    g_last_error_message = msg;
    GEO_LOG_ERROR(msg);
}

void ClearLastError() {
    g_last_error_message.clear();
}

/**
 * @brief C++20 标准无警告 UTF-8 字符串转 std::filesystem::path 辅助函数
 */
std::filesystem::path Utf8ToPath(const char* utf8_str) {
    const std::string_view sv(utf8_str);
    return std::filesystem::path(
        reinterpret_cast<const char8_t*>(sv.data()),
        reinterpret_cast<const char8_t*>(sv.data() + sv.size()));
}

struct SurfaceDataset {
    std::vector<geo_kernel::SurveyPoint> points;
    std::vector<std::string> code_dict;
    geo_kernel::BoundingBox3d global_bounds;
    geo_kernel::BoundingBox3d local_bounds;
    geo_kernel::PointCloudOctree octree;
    geo_kernel::TiledTinSurface tin;
    geo_kernel::TinTopologyEditor editor{tin, 64};
};

std::vector<geo_kernel::Vec3d> UnpackXyzArray(const double* xyz_array, uint32_t count) {
    std::vector<geo_kernel::Vec3d> pts;
    if (xyz_array == nullptr || count == 0) {
        return pts;
    }
    pts.reserve(count);
    for (uint32_t i = 0; i < count; ++i) {
        pts.emplace_back(
            xyz_array[static_cast<size_t>(i) * 3 + 0],
            xyz_array[static_cast<size_t>(i) * 3 + 1],
            xyz_array[static_cast<size_t>(i) * 3 + 2]);
    }
    return pts;
}

void FillEarthworkSummary(
    const geo_kernel::EarthworkReport& rep,
    GeoEarthworkSummary& out_sum) noexcept {
    out_sum.cut_volume_m3 = rep.cut_volume_m3;
    out_sum.fill_volume_m3 = rep.fill_volume_m3;
    out_sum.net_volume_m3 = rep.net_volume_m3;
    out_sum.total_overlap_area_m2 = rep.total_overlap_area_m2;
    out_sum.cut_area_m2 = rep.cut_area_m2;
    out_sum.fill_area_m2 = rep.fill_area_m2;
    out_sum.zero_area_m2 = rep.zero_area_m2;
    out_sum.max_cut_depth_m = rep.max_cut_depth_m;
    out_sum.max_fill_height_m = rep.max_fill_height_m;
    out_sum.intersected_prisms = rep.intersected_prisms;
    out_sum.elapsed_seconds = rep.elapsed_seconds;
}

}  // namespace

// ==============================================================================
// 实现 core_types.hpp 声明的跨模块统一日志分发函数 geo_kernel::detail::WriteKernelLog
// ==============================================================================

namespace geo_kernel::detail {

void WriteKernelLog(
    int32_t level,
    const char* file_path,
    int line_no,
    const char* func_name,
    std::string_view message) noexcept {
    try {
        const char* base_name = file_path;
        if (file_path != nullptr) {
            for (const char* p = file_path; *p != '\0'; ++p) {
                if (*p == '/' || *p == '\\') {
                    base_name = p + 1;
                }
            }
        }

        char loc_buf[256];
        std::snprintf(
            loc_buf,
            sizeof(loc_buf),
            "%s:%d %s",
            base_name != nullptr ? base_name : "unknown",
            line_no,
            func_name != nullptr ? func_name : "");

        GeoLogCallback cb = nullptr;
        void* user_data = nullptr;
        {
            std::lock_guard<std::mutex> lock(g_log_callback_mutex);
            cb = g_log_callback;
            user_data = g_log_callback_user_data;
        }

        if (cb != nullptr) {
            std::string msg_copy(message);
            cb(level, "CPP:KERNEL", loc_buf, msg_copy.c_str(), user_data);
            return;
        }

        const char* level_str = "INFO ";
        switch (level) {
            case GEO_LOG_LEVEL_DEBUG: level_str = "DEBUG"; break;
            case GEO_LOG_LEVEL_INFO:  level_str = "INFO "; break;
            case GEO_LOG_LEVEL_WARN:  level_str = "WARN "; break;
            case GEO_LOG_LEVEL_ERROR: level_str = "ERROR"; break;
            default: break;
        }

        std::string line;
        line.reserve(message.size() + 128);
        line.append("[");
        line.append(level_str);
        line.append("] [CPP:KERNEL] (");
        line.append(loc_buf);
        line.append(") ");
        line.append(message.data(), message.size());
        line.append("\n");
        WriteUtf8ToStderrBypassUcrt(line);
    } catch (...) {
        // 日志输出过程严禁向外抛出任何异常
    }
}

}  // namespace geo_kernel::detail

struct GeoProjectOpaque {
    mutable std::shared_mutex rw_mutex;
    geo_kernel::RtcTransform shared_rtc;
    SurfaceDataset surfaces[kMaxSurfaceSlots];
};

namespace {

bool ValidateProjectAndSlot(
    GeoProjectHandle handle,
    int32_t surface_slot,
    GeoStatus& out_err_status) {
    if (handle == nullptr) {
        SetLastError("工程句柄 GeoProjectHandle 为空指针");
        out_err_status = GEO_STATUS_INVALID_ARGUMENT;
        return false;
    }
    if (surface_slot < 0 || surface_slot >= kMaxSurfaceSlots) {
        SetLastError("非法模型槽位编号（有效范围为 0 ~ 3）");
        out_err_status = GEO_STATUS_INVALID_ARGUMENT;
        return false;
    }
    return true;
}

GeoStatus MapStatus(geo_kernel::StatusCode code) noexcept {
    return static_cast<GeoStatus>(static_cast<int32_t>(code));
}

}  // namespace

extern "C" {

const char* geo_get_version(void) {
    return "2.2.1";
}

const char* geo_get_last_error(void) {
    return g_last_error_message.c_str();
}

void geo_set_log_callback(GeoLogCallback callback, void* user_data) {
    EnsureSafeWindowsStdioMode();
    std::lock_guard<std::mutex> lock(g_log_callback_mutex);
    g_log_callback = callback;
    g_log_callback_user_data = user_data;
}

GeoProjectHandle geo_project_create(void) {
    try {
        EnsureSafeWindowsStdioMode();
        geo_kernel::ExactPredicates::Initialize();
        ClearLastError();
        auto* project = new GeoProjectOpaque();
        char ptr_buf[64];
        std::snprintf(ptr_buf, sizeof(ptr_buf), "%p", static_cast<void*>(project));
        GEO_LOG_INFO(std::string("geo_project_create 创建工程上下文成功: ptr=") + ptr_buf);
        return project;
    } catch (...) {
        SetLastError("创建工程实例时内存分配失败");
        return nullptr;
    }
}

void geo_project_destroy(GeoProjectHandle handle) {
    if (handle != nullptr) {
        char ptr_buf[64];
        std::snprintf(ptr_buf, sizeof(ptr_buf), "%p", static_cast<void*>(handle));
        GEO_LOG_INFO(std::string("geo_project_destroy 销毁工程上下文: ptr=") + ptr_buf);
    }
    delete handle;
}

GeoDatLoadParams geo_default_dat_load_params(void) {
    GeoDatLoadParams p{};
    p.coord_order = GEO_COORD_SURVEYING_YXZ;
    p.thread_count = 0;
    p.enable_elevation_filter = true;
    p.min_valid_elevation = -500.0;
    p.max_valid_elevation = 9000.0;
    p.filter_zero_coordinates = true;
    return p;
}

GeoStatus geo_project_load_dat(
    GeoProjectHandle handle,
    int32_t surface_slot,
    const char* utf8_file_path,
    const GeoDatLoadParams* params,
    GeoProgressCallback progress_cb,
    void* user_data,
    GeoDatSummary* out_summary) {
    GeoStatus err = GEO_STATUS_OK;
    if (!ValidateProjectAndSlot(handle, surface_slot, err)) {
        return err;
    }
    if (utf8_file_path == nullptr || utf8_file_path[0] == '\0') {
        SetLastError("点云文件路径不能为空");
        return GEO_STATUS_INVALID_ARGUMENT;
    }

    try {
        std::unique_lock<std::shared_mutex> lock(handle->rw_mutex);

        const GeoDatLoadParams effective_params = (params != nullptr)
            ? *params
            : geo_default_dat_load_params();

        geo_kernel::DatParseOptions parse_opts;
        parse_opts.coord_order = (effective_params.coord_order == GEO_COORD_CARTESIAN_XYZ)
            ? geo_kernel::CoordinateOrder::kCartesianXYZ
            : geo_kernel::CoordinateOrder::kSurveyingYXZ;
        parse_opts.thread_count = effective_params.thread_count;
        parse_opts.enable_elevation_filter = effective_params.enable_elevation_filter;
        parse_opts.min_valid_elevation = effective_params.min_valid_elevation;
        parse_opts.max_valid_elevation = effective_params.max_valid_elevation;
        parse_opts.filter_zero_coordinates = effective_params.filter_zero_coordinates;

        if (progress_cb != nullptr) {
            parse_opts.progress_callback = [progress_cb, user_data](double prog, uint64_t pts) -> bool {
                return progress_cb(prog, pts, user_data);
            };
        }

        SurfaceDataset& target_surface = handle->surfaces[surface_slot];
        target_surface.octree.Clear();
        target_surface.tin.Clear();
        target_surface.editor.ClearHistory();

        geo_kernel::DatPointCloudParser parser;
        geo_kernel::DatParseStats stats;
        const std::filesystem::path fs_path = Utf8ToPath(utf8_file_path);

        geo_kernel::OperationResult res = parser.ParseFile(
            fs_path,
            parse_opts,
            target_surface.points,
            target_surface.code_dict,
            stats);

        if (!res.IsOk()) {
            SetLastError(res.message);
            return MapStatus(res.code);
        }

        if (!handle->shared_rtc.initialized) {
            handle->shared_rtc = stats.rtc_transform;
        } else {
            const geo_kernel::Vec3d delta_origin =
                stats.rtc_transform.origin - handle->shared_rtc.origin;
            if (delta_origin.LengthSquared() > 1e-12) {
                for (auto& pt : target_surface.points) {
                    pt.x += delta_origin.x;
                    pt.y += delta_origin.y;
                    pt.z += delta_origin.z;
                }
                stats.local_bounds.min_pt = stats.global_bounds.min_pt - handle->shared_rtc.origin;
                stats.local_bounds.max_pt = stats.global_bounds.max_pt - handle->shared_rtc.origin;
            }
        }

        target_surface.global_bounds = stats.global_bounds;
        target_surface.local_bounds = stats.local_bounds;

        if (out_summary != nullptr) {
            std::memset(out_summary, 0, sizeof(GeoDatSummary));
            out_summary->file_bytes = stats.file_bytes;
            out_summary->total_lines_scanned = stats.total_lines_scanned;
            out_summary->valid_points = stats.valid_points;
            out_summary->skipped_lines = stats.skipped_empty_or_comment;
            out_summary->malformed_lines = stats.malformed_lines;
            out_summary->filtered_outliers = stats.filtered_outliers;
            out_summary->elapsed_seconds = stats.elapsed_seconds;
            out_summary->throughput_mpts_per_sec = stats.throughput_mpts_per_sec;

            out_summary->rtc_origin[0] = handle->shared_rtc.origin.x;
            out_summary->rtc_origin[1] = handle->shared_rtc.origin.y;
            out_summary->rtc_origin[2] = handle->shared_rtc.origin.z;

            out_summary->global_min[0] = stats.global_bounds.min_pt.x;
            out_summary->global_min[1] = stats.global_bounds.min_pt.y;
            out_summary->global_min[2] = stats.global_bounds.min_pt.z;
            out_summary->global_max[0] = stats.global_bounds.max_pt.x;
            out_summary->global_max[1] = stats.global_bounds.max_pt.y;
            out_summary->global_max[2] = stats.global_bounds.max_pt.z;

            out_summary->local_min[0] = stats.local_bounds.min_pt.x;
            out_summary->local_min[1] = stats.local_bounds.min_pt.y;
            out_summary->local_min[2] = stats.local_bounds.min_pt.z;
            out_summary->local_max[0] = stats.local_bounds.max_pt.x;
            out_summary->local_max[1] = stats.local_bounds.max_pt.y;
            out_summary->local_max[2] = stats.local_bounds.max_pt.z;
        }

        GEO_LOG_INFO(
            "点云加载完成: 槽位=" + std::to_string(surface_slot) +
            ", 有效点数=" + std::to_string(stats.valid_points));
        ClearLastError();
        return GEO_STATUS_OK;
    } catch (const std::bad_alloc&) {
        SetLastError("导入点云数据时内存不足");
        return GEO_STATUS_OUT_OF_MEMORY;
    } catch (...) {
        SetLastError("导入点云数据时发生未预期内部异常");
        return GEO_STATUS_INTERNAL_ERROR;
    }
}

GeoOctreeParams geo_default_octree_params(void) {
    GeoOctreeParams p{};
    p.max_depth = 10;
    p.max_points_per_leaf = 16384;
    p.max_lod_points_per_node = 8192;
    p.thread_count = 0;
    return p;
}

GeoStatus geo_project_build_octree(
    GeoProjectHandle handle,
    int32_t surface_slot,
    const GeoOctreeParams* params,
    GeoOctreeSummary* out_summary) {
    GeoStatus err = GEO_STATUS_OK;
    if (!ValidateProjectAndSlot(handle, surface_slot, err)) {
        return err;
    }

    try {
        std::unique_lock<std::shared_mutex> lock(handle->rw_mutex);
        SurfaceDataset& target_surface = handle->surfaces[surface_slot];
        if (target_surface.points.empty()) {
            SetLastError("目标槽位尚未导入点云数据，无法构建八叉树");
            return GEO_STATUS_EMPTY_DATASET;
        }

        const GeoOctreeParams effective = (params != nullptr) ? *params : geo_default_octree_params();
        geo_kernel::OctreeBuildOptions opts;
        opts.max_depth = effective.max_depth;
        opts.max_points_per_leaf = effective.max_points_per_leaf;
        opts.max_lod_points_per_node = effective.max_lod_points_per_node;
        opts.thread_count = effective.thread_count;

        geo_kernel::OctreeBuildStats stats;
        geo_kernel::OperationResult res = target_surface.octree.Build(
            target_surface.points,
            target_surface.local_bounds,
            opts,
            stats);

        if (!res.IsOk()) {
            SetLastError(res.message);
            return MapStatus(res.code);
        }

        if (out_summary != nullptr) {
            out_summary->total_points = stats.total_points;
            out_summary->total_nodes = stats.total_nodes;
            out_summary->leaf_nodes = stats.leaf_nodes;
            out_summary->max_reached_depth = stats.max_reached_depth;
            out_summary->elapsed_seconds = stats.elapsed_seconds;
        }

        GEO_LOG_INFO(
            "八叉树构建完成: 槽位=" + std::to_string(surface_slot) +
            ", 节点总数=" + std::to_string(stats.total_nodes));
        ClearLastError();
        return GEO_STATUS_OK;
    } catch (...) {
        SetLastError("构建点云八叉树时发生未预期内部异常");
        return GEO_STATUS_INTERNAL_ERROR;
    }
}

GeoStatus geo_project_query_visible_octree_nodes(
    GeoProjectHandle handle,
    int32_t surface_slot,
    const GeoCameraQuery* query,
    uint32_t* out_node_ids,
    uint32_t capacity,
    uint32_t* out_node_count,
    uint64_t* out_estimated_points) {
    GeoStatus err = GEO_STATUS_OK;
    if (!ValidateProjectAndSlot(handle, surface_slot, err)) {
        return err;
    }
    if (query == nullptr || out_node_count == nullptr) {
        SetLastError("查询参数 query 或输出计数指针 out_node_count 不能为空");
        return GEO_STATUS_INVALID_ARGUMENT;
    }

    try {
        std::shared_lock<std::shared_mutex> lock(handle->rw_mutex);
        const SurfaceDataset& target_surface = handle->surfaces[surface_slot];
        if (!target_surface.octree.IsBuilt()) {
            SetLastError("目标槽位尚未构建点云八叉树");
            return GEO_STATUS_EMPTY_DATASET;
        }

        geo_kernel::CameraViewParams view;
        view.camera_local_pos = {
            query->camera_local_pos[0],
            query->camera_local_pos[1],
            query->camera_local_pos[2]
        };
        std::memcpy(view.frustum_planes, query->frustum_planes, sizeof(view.frustum_planes));
        view.viewport_height_px = query->viewport_height_px;
        view.fov_y_radians = query->fov_y_radians;
        view.sse_threshold_px = query->sse_threshold_px;
        view.max_point_budget = query->max_point_budget;
        view.enable_frustum_culling = query->enable_frustum_culling;

        std::vector<uint32_t> visible_ids;
        uint64_t est_pts = 0;
        target_surface.octree.QueryVisibleNodes(view, visible_ids, est_pts);

        *out_node_count = static_cast<uint32_t>(visible_ids.size());
        if (out_estimated_points != nullptr) {
            *out_estimated_points = est_pts;
        }

        if (out_node_ids != nullptr && capacity > 0 && !visible_ids.empty()) {
            const uint32_t copy_count = (static_cast<uint32_t>(visible_ids.size()) < capacity)
                ? static_cast<uint32_t>(visible_ids.size())
                : capacity;
            std::memcpy(out_node_ids, visible_ids.data(), static_cast<size_t>(copy_count) * sizeof(uint32_t));
        }

        ClearLastError();
        return GEO_STATUS_OK;
    } catch (...) {
        SetLastError("查询可视八叉树节点时发生未预期内部异常");
        return GEO_STATUS_INTERNAL_ERROR;
    }
}

GeoStatus geo_project_fetch_octree_node_gpu_packet(
    GeoProjectHandle handle,
    int32_t surface_slot,
    uint32_t node_id,
    uint8_t* out_buffer,
    size_t buffer_capacity,
    size_t* out_byte_size) {
    GeoStatus err = GEO_STATUS_OK;
    if (!ValidateProjectAndSlot(handle, surface_slot, err)) {
        return err;
    }
    if (out_byte_size == nullptr) {
        SetLastError("输出字节大小指针 out_byte_size 不能为空");
        return GEO_STATUS_INVALID_ARGUMENT;
    }

    try {
        std::shared_lock<std::shared_mutex> lock(handle->rw_mutex);
        const SurfaceDataset& target_surface = handle->surfaces[surface_slot];
        if (!target_surface.octree.IsBuilt()) {
            SetLastError("目标槽位尚未构建点云八叉树");
            return GEO_STATUS_EMPTY_DATASET;
        }

        std::vector<uint8_t> packet;
        geo_kernel::OperationResult res = target_surface.octree.SerializeNodeToGpuPacket(
            node_id, target_surface.points, handle->shared_rtc, packet);
        if (!res.IsOk()) {
            SetLastError(res.message);
            return MapStatus(res.code);
        }

        *out_byte_size = packet.size();
        if (out_buffer != nullptr) {
            if (buffer_capacity < packet.size()) {
                SetLastError("输出缓冲区容量不足以容纳八叉树节点 GPU 数据包");
                return GEO_STATUS_INVALID_ARGUMENT;
            }
            if (!packet.empty()) {
                std::memcpy(out_buffer, packet.data(), packet.size());
            }
        }

        ClearLastError();
        return GEO_STATUS_OK;
    } catch (...) {
        SetLastError("提取八叉树节点 GPU 数据包时发生未预期内部异常");
        return GEO_STATUS_INTERNAL_ERROR;
    }
}

GeoTinParams geo_default_tin_params(void) {
    GeoTinParams p{};
    p.decimation_grid_size = 0.0;
    p.tile_size = 128.0;
    p.max_edge_length = 0.0;
    return p;
}

GeoStatus geo_project_build_tin(
    GeoProjectHandle handle,
    int32_t surface_slot,
    const GeoTinParams* params,
    GeoProgressCallback progress_cb,
    void* user_data,
    GeoTinSummary* out_summary) {
    GeoStatus err = GEO_STATUS_OK;
    if (!ValidateProjectAndSlot(handle, surface_slot, err)) {
        return err;
    }

    try {
        std::unique_lock<std::shared_mutex> lock(handle->rw_mutex);
        SurfaceDataset& target_surface = handle->surfaces[surface_slot];
        if (target_surface.points.empty()) {
            SetLastError("目标槽位尚未导入点云数据，无法构建三角网");
            return GEO_STATUS_EMPTY_DATASET;
        }

        const GeoTinParams effective = (params != nullptr) ? *params : geo_default_tin_params();
        geo_kernel::TinBuildOptions opts;
        opts.decimation_grid_size = effective.decimation_grid_size;
        opts.tile_size = effective.tile_size;
        opts.max_edge_length = effective.max_edge_length;

        if (progress_cb != nullptr) {
            opts.progress_callback = [progress_cb, user_data](double prog, uint64_t verts) -> bool {
                return progress_cb(prog, verts, user_data);
            };
        }

        target_surface.editor.ClearHistory();
        geo_kernel::TinBuildStats stats;
        geo_kernel::OperationResult res = target_surface.tin.BuildFromPoints(
            target_surface.points, opts, stats);
        if (!res.IsOk()) {
            SetLastError(res.message);
            return MapStatus(res.code);
        }

        if (out_summary != nullptr) {
            out_summary->input_points = stats.input_points;
            out_summary->deduplicated_points = stats.deduplicated_points;
            out_summary->decimated_points = stats.decimated_points;
            out_summary->tin_vertices = stats.tin_vertices;
            out_summary->tin_triangles = stats.tin_triangles;
            out_summary->total_tiles = stats.total_tiles;
            out_summary->elapsed_seconds = stats.elapsed_seconds;
        }

        GEO_LOG_INFO(
            "三角网构建完成: 槽位=" + std::to_string(surface_slot) +
            ", 三角形数=" + std::to_string(stats.tin_triangles));
        ClearLastError();
        return GEO_STATUS_OK;
    } catch (...) {
        SetLastError("构建三角网时发生未预期内部异常");
        return GEO_STATUS_INTERNAL_ERROR;
    }
}

GeoStatus geo_project_query_elevation(
    GeoProjectHandle handle,
    int32_t surface_slot,
    double local_x,
    double local_y,
    double* out_local_z,
    double* out_global_z) {
    GeoStatus err = GEO_STATUS_OK;
    if (!ValidateProjectAndSlot(handle, surface_slot, err)) {
        return err;
    }
    if (out_local_z == nullptr) {
        SetLastError("输出高程指针 out_local_z 不能为空");
        return GEO_STATUS_INVALID_ARGUMENT;
    }

    try {
        std::shared_lock<std::shared_mutex> lock(handle->rw_mutex);
        const SurfaceDataset& target_surface = handle->surfaces[surface_slot];
        if (!target_surface.tin.IsValid()) {
            SetLastError("目标槽位尚未构建有效三角网");
            return GEO_STATUS_EMPTY_DATASET;
        }

        double lz = 0.0;
        if (!target_surface.tin.QueryElevationAt(local_x, local_y, lz, nullptr)) {
            SetLastError("查询点坐标超出三角网平面覆盖范围");
            return GEO_STATUS_TILE_NOT_FOUND;
        }

        *out_local_z = lz;
        if (out_global_z != nullptr) {
            *out_global_z = lz + handle->shared_rtc.origin.z;
        }

        ClearLastError();
        return GEO_STATUS_OK;
    } catch (...) {
        SetLastError("查询三角网高程时发生未预期内部异常");
        return GEO_STATUS_INTERNAL_ERROR;
    }
}

GeoStatus geo_project_get_dirty_tin_tiles(
    GeoProjectHandle handle,
    int32_t surface_slot,
    uint32_t* out_tile_indices,
    uint32_t capacity,
    uint32_t* out_tile_count) {
    GeoStatus err = GEO_STATUS_OK;
    if (!ValidateProjectAndSlot(handle, surface_slot, err)) {
        return err;
    }
    if (out_tile_count == nullptr) {
        SetLastError("输出瓦片计数指针 out_tile_count 不能为空");
        return GEO_STATUS_INVALID_ARGUMENT;
    }

    try {
        std::shared_lock<std::shared_mutex> lock(handle->rw_mutex);
        const SurfaceDataset& target_surface = handle->surfaces[surface_slot];
        const std::vector<uint32_t> dirty_tiles = target_surface.tin.GetDirtyTileIndices();

        *out_tile_count = static_cast<uint32_t>(dirty_tiles.size());
        if (out_tile_indices != nullptr && capacity > 0 && !dirty_tiles.empty()) {
            const uint32_t copy_count = (static_cast<uint32_t>(dirty_tiles.size()) < capacity)
                ? static_cast<uint32_t>(dirty_tiles.size())
                : capacity;
            std::memcpy(
                out_tile_indices,
                dirty_tiles.data(),
                static_cast<size_t>(copy_count) * sizeof(uint32_t));
        }

        ClearLastError();
        return GEO_STATUS_OK;
    } catch (...) {
        SetLastError("获取脏三角网瓦片列表时发生未预期内部异常");
        return GEO_STATUS_INTERNAL_ERROR;
    }
}

GeoStatus geo_project_fetch_tin_tile_gpu_packet(
    GeoProjectHandle handle,
    int32_t surface_slot,
    uint32_t tile_index,
    bool clear_dirty_flag,
    uint8_t* out_buffer,
    size_t buffer_capacity,
    size_t* out_byte_size) {
    GeoStatus err = GEO_STATUS_OK;
    if (!ValidateProjectAndSlot(handle, surface_slot, err)) {
        return err;
    }
    if (out_byte_size == nullptr) {
        SetLastError("输出字节大小指针 out_byte_size 不能为空");
        return GEO_STATUS_INVALID_ARGUMENT;
    }

    try {
        std::unique_lock<std::shared_mutex> lock(handle->rw_mutex);
        SurfaceDataset& target_surface = handle->surfaces[surface_slot];
        if (!target_surface.tin.IsValid()) {
            SetLastError("目标槽位尚未构建有效三角网");
            return GEO_STATUS_EMPTY_DATASET;
        }

        std::vector<uint8_t> packet;
        geo_kernel::OperationResult res = target_surface.tin.SerializeTileToGpuPacket(
            tile_index, handle->shared_rtc, packet);
        if (!res.IsOk()) {
            SetLastError(res.message);
            return MapStatus(res.code);
        }

        *out_byte_size = packet.size();
        if (out_buffer != nullptr) {
            if (buffer_capacity < packet.size()) {
                SetLastError("输出缓冲区容量不足以容纳三角网瓦片 GPU 数据包");
                return GEO_STATUS_INVALID_ARGUMENT;
            }
            if (!packet.empty()) {
                std::memcpy(out_buffer, packet.data(), packet.size());
            }
            if (clear_dirty_flag) {
                target_surface.tin.ClearTileDirty(tile_index);
            }
        }

        ClearLastError();
        return GEO_STATUS_OK;
    } catch (...) {
        SetLastError("提取三角网瓦片 GPU 数据包时发生未预期内部异常");
        return GEO_STATUS_INTERNAL_ERROR;
    }
}

GeoStatus geo_project_pick_tin_ray(
    GeoProjectHandle handle,
    int32_t surface_slot,
    const double ray_origin[3],
    const double ray_dir[3],
    GeoRayHitResult* out_hit) {
    GeoStatus err = GEO_STATUS_OK;
    if (!ValidateProjectAndSlot(handle, surface_slot, err)) {
        return err;
    }
    if (ray_origin == nullptr || ray_dir == nullptr || out_hit == nullptr) {
        SetLastError("射线拾取参数指针不能为空");
        return GEO_STATUS_INVALID_ARGUMENT;
    }

    try {
        std::shared_lock<std::shared_mutex> lock(handle->rw_mutex);
        const SurfaceDataset& target_surface = handle->surfaces[surface_slot];
        if (!target_surface.tin.IsValid()) {
            SetLastError("目标槽位尚未构建有效三角网");
            return GEO_STATUS_EMPTY_DATASET;
        }

        const geo_kernel::RayHitResult hit = target_surface.tin.PickByRay(
            {ray_origin[0], ray_origin[1], ray_origin[2]},
            {ray_dir[0], ray_dir[1], ray_dir[2]});

        std::memset(out_hit, 0, sizeof(GeoRayHitResult));
        out_hit->hit = hit.hit;
        out_hit->triangle_id = hit.triangle_id;
        out_hit->nearest_vertex_id = hit.nearest_vertex_id;
        out_hit->nearest_edge_slot = hit.nearest_edge_slot;
        out_hit->distance = hit.distance;
        out_hit->hit_local_pos[0] = hit.hit_point.x;
        out_hit->hit_local_pos[1] = hit.hit_point.y;
        out_hit->hit_local_pos[2] = hit.hit_point.z;

        ClearLastError();
        return GEO_STATUS_OK;
    } catch (...) {
        SetLastError("执行三角网射线拾取时发生内部异常");
        return GEO_STATUS_INTERNAL_ERROR;
    }
}

GeoStatus geo_project_tin_flip_edge(
    GeoProjectHandle handle,
    int32_t surface_slot,
    uint32_t triangle_id,
    int32_t edge_slot,
    bool force_flip) {
    GeoStatus err = GEO_STATUS_OK;
    if (!ValidateProjectAndSlot(handle, surface_slot, err)) {
        return err;
    }

    try {
        std::unique_lock<std::shared_mutex> lock(handle->rw_mutex);
        geo_kernel::OperationResult res =
            handle->surfaces[surface_slot].editor.FlipEdge(triangle_id, edge_slot, force_flip);
        if (!res.IsOk()) {
            SetLastError(res.message);
            return MapStatus(res.code);
        }
        ClearLastError();
        return GEO_STATUS_OK;
    } catch (...) {
        SetLastError("执行对角线边翻转时发生内部异常");
        return GEO_STATUS_INTERNAL_ERROR;
    }
}

GeoStatus geo_project_tin_modify_vertex_z(
    GeoProjectHandle handle,
    int32_t surface_slot,
    uint32_t vertex_id,
    double new_local_z) {
    GeoStatus err = GEO_STATUS_OK;
    if (!ValidateProjectAndSlot(handle, surface_slot, err)) {
        return err;
    }

    try {
        std::unique_lock<std::shared_mutex> lock(handle->rw_mutex);
        geo_kernel::OperationResult res =
            handle->surfaces[surface_slot].editor.ModifyVertexElevation(vertex_id, new_local_z);
        if (!res.IsOk()) {
            SetLastError(res.message);
            return MapStatus(res.code);
        }
        ClearLastError();
        return GEO_STATUS_OK;
    } catch (...) {
        SetLastError("修改三角网顶点高程时发生内部异常");
        return GEO_STATUS_INTERNAL_ERROR;
    }
}

GeoStatus geo_project_tin_insert_vertex(
    GeoProjectHandle handle,
    int32_t surface_slot,
    const double local_xyz[3],
    uint16_t code_id,
    bool legalize_delaunay,
    uint32_t* out_vertex_id) {
    GeoStatus err = GEO_STATUS_OK;
    if (!ValidateProjectAndSlot(handle, surface_slot, err)) {
        return err;
    }
    if (local_xyz == nullptr) {
        SetLastError("待插入顶点坐标数组 local_xyz 不能为空");
        return GEO_STATUS_INVALID_ARGUMENT;
    }

    try {
        std::unique_lock<std::shared_mutex> lock(handle->rw_mutex);
        geo_kernel::OperationResult res = handle->surfaces[surface_slot].editor.InsertVertex(
            {local_xyz[0], local_xyz[1], local_xyz[2]},
            0,
            code_id,
            geo_kernel::SurveyPoint::kFlagValid,
            legalize_delaunay,
            out_vertex_id);
        if (!res.IsOk()) {
            SetLastError(res.message);
            return MapStatus(res.code);
        }
        ClearLastError();
        return GEO_STATUS_OK;
    } catch (...) {
        SetLastError("插入三角网顶点时发生内部异常");
        return GEO_STATUS_INTERNAL_ERROR;
    }
}

GeoStatus geo_project_tin_insert_breakline(
    GeoProjectHandle handle,
    int32_t surface_slot,
    const double* xyz_array,
    uint32_t point_count,
    uint16_t code_id,
    bool interpolate_z_from_tin) {
    GeoStatus err = GEO_STATUS_OK;
    if (!ValidateProjectAndSlot(handle, surface_slot, err)) {
        return err;
    }
    if (xyz_array == nullptr || point_count < 2) {
        SetLastError("特征折线顶点数组不能为空且至少包含 2 个点");
        return GEO_STATUS_INVALID_ARGUMENT;
    }

    try {
        std::unique_lock<std::shared_mutex> lock(handle->rw_mutex);
        std::vector<geo_kernel::Vec3d> poly = UnpackXyzArray(xyz_array, point_count);
        geo_kernel::OperationResult res = handle->surfaces[surface_slot].editor.InsertBreakline(
            poly, code_id, interpolate_z_from_tin);
        if (!res.IsOk()) {
            SetLastError(res.message);
            return MapStatus(res.code);
        }
        ClearLastError();
        return GEO_STATUS_OK;
    } catch (...) {
        SetLastError("嵌入特征线时发生内部异常");
        return GEO_STATUS_INTERNAL_ERROR;
    }
}

GeoStatus geo_project_tin_delete_vertex(
    GeoProjectHandle handle,
    int32_t surface_slot,
    uint32_t vertex_id) {
    GeoStatus err = GEO_STATUS_OK;
    if (!ValidateProjectAndSlot(handle, surface_slot, err)) {
        return err;
    }

    try {
        std::unique_lock<std::shared_mutex> lock(handle->rw_mutex);
        geo_kernel::OperationResult res =
            handle->surfaces[surface_slot].editor.DeleteVertex(vertex_id);
        if (!res.IsOk()) {
            SetLastError(res.message);
            return MapStatus(res.code);
        }
        ClearLastError();
        return GEO_STATUS_OK;
    } catch (...) {
        SetLastError("删除三角网顶点时发生内部异常");
        return GEO_STATUS_INTERNAL_ERROR;
    }
}

GeoStatus geo_project_tin_clip_polygon(
    GeoProjectHandle handle,
    int32_t surface_slot,
    const double* xyz_array,
    uint32_t point_count,
    bool keep_inside) {
    GeoStatus err = GEO_STATUS_OK;
    if (!ValidateProjectAndSlot(handle, surface_slot, err)) {
        return err;
    }
    if (xyz_array == nullptr || point_count < 3) {
        SetLastError("裁剪多边形至少需要 3 个顶点");
        return GEO_STATUS_INVALID_ARGUMENT;
    }

    try {
        std::unique_lock<std::shared_mutex> lock(handle->rw_mutex);
        std::vector<geo_kernel::Vec3d> poly = UnpackXyzArray(xyz_array, point_count);
        geo_kernel::OperationResult res =
            handle->surfaces[surface_slot].editor.ClipByPolygon(poly, keep_inside);
        if (!res.IsOk()) {
            SetLastError(res.message);
            return MapStatus(res.code);
        }
        ClearLastError();
        return GEO_STATUS_OK;
    } catch (...) {
        SetLastError("执行三角网多边形裁剪时发生内部异常");
        return GEO_STATUS_INTERNAL_ERROR;
    }
}

GeoStatus geo_project_tin_undo(GeoProjectHandle handle, int32_t surface_slot) {
    GeoStatus err = GEO_STATUS_OK;
    if (!ValidateProjectAndSlot(handle, surface_slot, err)) {
        return err;
    }

    try {
        std::unique_lock<std::shared_mutex> lock(handle->rw_mutex);
        geo_kernel::OperationResult res = handle->surfaces[surface_slot].editor.Undo();
        if (!res.IsOk()) {
            SetLastError(res.message);
            return MapStatus(res.code);
        }
        ClearLastError();
        return GEO_STATUS_OK;
    } catch (...) {
        SetLastError("撤销三角网编辑时发生内部异常");
        return GEO_STATUS_INTERNAL_ERROR;
    }
}

GeoStatus geo_project_tin_redo(GeoProjectHandle handle, int32_t surface_slot) {
    GeoStatus err = GEO_STATUS_OK;
    if (!ValidateProjectAndSlot(handle, surface_slot, err)) {
        return err;
    }

    try {
        std::unique_lock<std::shared_mutex> lock(handle->rw_mutex);
        geo_kernel::OperationResult res = handle->surfaces[surface_slot].editor.Redo();
        if (!res.IsOk()) {
            SetLastError(res.message);
            return MapStatus(res.code);
        }
        ClearLastError();
        return GEO_STATUS_OK;
    } catch (...) {
        SetLastError("重做三角网编辑时发生内部异常");
        return GEO_STATUS_INTERNAL_ERROR;
    }
}

GeoStatus geo_project_tin_get_history_state(
    GeoProjectHandle handle,
    int32_t surface_slot,
    bool* out_can_undo,
    bool* out_can_redo) {
    GeoStatus err = GEO_STATUS_OK;
    if (!ValidateProjectAndSlot(handle, surface_slot, err)) {
        return err;
    }

    std::shared_lock<std::shared_mutex> lock(handle->rw_mutex);
    if (out_can_undo != nullptr) {
        *out_can_undo = handle->surfaces[surface_slot].editor.CanUndo();
    }
    if (out_can_redo != nullptr) {
        *out_can_redo = handle->surfaces[surface_slot].editor.CanRedo();
    }
    ClearLastError();
    return GEO_STATUS_OK;
}

GeoStatus geo_project_compute_earthwork_exact(
    GeoProjectHandle handle,
    int32_t period1_slot,
    int32_t period2_slot,
    const double* roi_xyz_array,
    uint32_t roi_point_count,
    uint32_t thread_count,
    GeoEarthworkSummary* out_summary) {
    GeoStatus err = GEO_STATUS_OK;
    if (!ValidateProjectAndSlot(handle, period1_slot, err) ||
        !ValidateProjectAndSlot(handle, period2_slot, err)) {
        return err;
    }
    if (out_summary == nullptr) {
        SetLastError("输出土方报告指针 out_summary 不能为空");
        return GEO_STATUS_INVALID_ARGUMENT;
    }

    try {
        std::shared_lock<std::shared_mutex> lock(handle->rw_mutex);
        geo_kernel::EarthworkOptions opts;
        opts.thread_count = thread_count;
        if (roi_xyz_array != nullptr && roi_point_count >= 3) {
            opts.roi_polygon = UnpackXyzArray(roi_xyz_array, roi_point_count);
        }

        geo_kernel::EarthworkCalculator calc;
        geo_kernel::EarthworkReport report;
        geo_kernel::OperationResult res = calc.ComputeExactTinToTinVolume(
            handle->surfaces[period1_slot].tin,
            handle->surfaces[period2_slot].tin,
            opts,
            report);

        if (!res.IsOk()) {
            SetLastError(res.message);
            return MapStatus(res.code);
        }

        FillEarthworkSummary(report, *out_summary);
        ClearLastError();
        return GEO_STATUS_OK;
    } catch (...) {
        SetLastError("执行精确三棱柱叠加土方算量时发生内部异常");
        return GEO_STATUS_INTERNAL_ERROR;
    }
}

GeoStatus geo_project_compute_earthwork_grid(
    GeoProjectHandle handle,
    int32_t period1_slot,
    int32_t period2_slot,
    double grid_step,
    const double* roi_xyz_array,
    uint32_t roi_point_count,
    uint32_t thread_count,
    GeoEarthworkSummary* out_summary) {
    GeoStatus err = GEO_STATUS_OK;
    if (!ValidateProjectAndSlot(handle, period1_slot, err) ||
        !ValidateProjectAndSlot(handle, period2_slot, err)) {
        return err;
    }
    if (out_summary == nullptr) {
        SetLastError("输出土方报告指针 out_summary 不能为空");
        return GEO_STATUS_INVALID_ARGUMENT;
    }

    try {
        std::shared_lock<std::shared_mutex> lock(handle->rw_mutex);
        geo_kernel::EarthworkOptions opts;
        opts.grid_step = grid_step;
        opts.thread_count = thread_count;
        if (roi_xyz_array != nullptr && roi_point_count >= 3) {
            opts.roi_polygon = UnpackXyzArray(roi_xyz_array, roi_point_count);
        }

        geo_kernel::EarthworkCalculator calc;
        geo_kernel::EarthworkReport report;
        geo_kernel::OperationResult res = calc.ComputeFastGridDifferential(
            handle->surfaces[period1_slot].tin,
            handle->surfaces[period2_slot].tin,
            opts,
            report,
            nullptr);

        if (!res.IsOk()) {
            SetLastError(res.message);
            return MapStatus(res.code);
        }

        FillEarthworkSummary(report, *out_summary);
        ClearLastError();
        return GEO_STATUS_OK;
    } catch (...) {
        SetLastError("执行并行网格微分土方计算时发生内部异常");
        return GEO_STATUS_INTERNAL_ERROR;
    }
}

GeoGradingParams geo_default_grading_params(void) {
    GeoGradingParams p{};
    p.direction = 0;
    p.vertical_mode = 0;
    p.use_uniform_base_elevation = false;
    p.base_elevation = 0.0;
    p.auto_daylight_to_terrain = true;
    p.cap_inward_bottom = true;
    p.perimeter_sample_step = 1.5;
    p.output_tile_size = 128.0;
    return p;
}

GeoStatus geo_project_compute_slope_grading(
    GeoProjectHandle handle,
    int32_t original_terrain_slot,
    const double* base_xyz_array,
    uint32_t base_point_count,
    const GeoSlopeStageSpec* stages,
    uint32_t stage_count,
    const GeoGradingParams* params,
    bool generate_merged_terrain,
    double* out_daylight_xyz,
    uint32_t daylight_xyz_capacity,
    GeoGradingSummary* out_summary) {
    GeoStatus err = GEO_STATUS_OK;
    if (!ValidateProjectAndSlot(handle, original_terrain_slot, err)) {
        return err;
    }
    if (base_xyz_array == nullptr || base_point_count < 3) {
        SetLastError("放坡基准轮廓多边形至少需要 3 个顶点");
        return GEO_STATUS_INVALID_ARGUMENT;
    }
    if (stages == nullptr || stage_count == 0) {
        SetLastError("多级放坡参数 stages 不能为空且至少包含 1 级");
        return GEO_STATUS_INVALID_ARGUMENT;
    }

    try {
        std::unique_lock<std::shared_mutex> lock(handle->rw_mutex);

        const GeoGradingParams effective = (params != nullptr)
            ? *params
            : geo_default_grading_params();

        geo_kernel::MultiStageGradingOptions opts;
        opts.direction = (effective.direction == 1)
            ? geo_kernel::GradingDirection::kInward
            : geo_kernel::GradingDirection::kOutward;
        if (effective.vertical_mode > 0) {
            opts.vertical_mode = geo_kernel::GradingVerticalMode::kSlopeUp;
        } else if (effective.vertical_mode < 0) {
            opts.vertical_mode = geo_kernel::GradingVerticalMode::kSlopeDown;
        } else {
            opts.vertical_mode = geo_kernel::GradingVerticalMode::kAutoCutFill;
        }

        opts.base_polygon = UnpackXyzArray(base_xyz_array, base_point_count);
        opts.use_uniform_base_elevation = effective.use_uniform_base_elevation;
        opts.base_elevation = effective.base_elevation;
        opts.auto_daylight_to_terrain = effective.auto_daylight_to_terrain;
        opts.cap_inward_bottom = effective.cap_inward_bottom;
        opts.perimeter_sample_step = effective.perimeter_sample_step;
        opts.output_tile_size = effective.output_tile_size;

        opts.stages.reserve(stage_count);
        for (uint32_t i = 0; i < stage_count; ++i) {
            geo_kernel::SlopeStageSpec st{};
            st.stage_height = stages[i].stage_height;
            st.slope_ratio = stages[i].slope_ratio;
            st.berm_width = stages[i].berm_width;
            st.berm_slope_ratio = stages[i].berm_slope_ratio;
            opts.stages.push_back(st);
        }

        SurfaceDataset& grading_only_ds = handle->surfaces[GEO_SURFACE_GRADING_DESIGN_ONLY];
        SurfaceDataset& merged_ds = handle->surfaces[GEO_SURFACE_GRADING_MERGED_TERRAIN];
        grading_only_ds.editor.ClearHistory();
        merged_ds.editor.ClearHistory();

        geo_kernel::SlopeGradingEngine engine;
        geo_kernel::MultiStageGradingResult result;
        geo_kernel::OperationResult res = engine.GenerateMultiStageGrading(
            handle->surfaces[original_terrain_slot].tin,
            opts,
            grading_only_ds.tin,
            generate_merged_terrain ? &merged_ds.tin : nullptr,
            result);

        if (!res.IsOk()) {
            SetLastError(res.message);
            return MapStatus(res.code);
        }

        if (out_daylight_xyz != nullptr && daylight_xyz_capacity > 0 &&
            !result.daylight_polyline.empty()) {
            const uint32_t copy_pts = (static_cast<uint32_t>(result.daylight_polyline.size()) < daylight_xyz_capacity)
                ? static_cast<uint32_t>(result.daylight_polyline.size())
                : daylight_xyz_capacity;
            for (uint32_t i = 0; i < copy_pts; ++i) {
                out_daylight_xyz[static_cast<size_t>(i) * 3 + 0] = result.daylight_polyline[i].x;
                out_daylight_xyz[static_cast<size_t>(i) * 3 + 1] = result.daylight_polyline[i].y;
                out_daylight_xyz[static_cast<size_t>(i) * 3 + 2] = result.daylight_polyline[i].z;
            }
        }

        if (out_summary != nullptr) {
            std::memset(out_summary, 0, sizeof(GeoGradingSummary));
            FillEarthworkSummary(result.earthwork, out_summary->earthwork);
            out_summary->daylight_perimeter_m = result.daylight_perimeter_m;
            out_summary->slope_surface_area_m2 = result.slope_surface_area_m2;
            out_summary->berm_surface_area_m2 = result.berm_surface_area_m2;
            out_summary->base_pad_area_m2 = result.base_pad_area_m2;
            out_summary->actual_stages_used = result.actual_stages_used;
            out_summary->daylight_vertex_count = static_cast<uint32_t>(result.daylight_polyline.size());
            out_summary->daylight_converged = result.daylight_converged;
            out_summary->elapsed_seconds = result.elapsed_seconds;
        }

        ClearLastError();
        return GEO_STATUS_OK;
    } catch (...) {
        SetLastError("执行多级放坡建模与土方计算时发生内部异常");
        return GEO_STATUS_INTERNAL_ERROR;
    }
}

}  // extern "C"

