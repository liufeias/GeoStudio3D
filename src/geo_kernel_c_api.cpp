/**
 * @file geo_kernel_c_api.cpp
 * @brief 空间几何内核跨语言扁平 C ABI 生产级完整实现
 *
 * 企业级健壮性与高性能架构要点：
 *   1. 任意期数动态槽位容器：
 *      - 使用 `std::unordered_map<int32_t, std::unique_ptr<SurfaceSlotData>>` 管理全部测绘期数
 *        与派生放坡面，彻底打破固定 2 期的数量上限；
 *      - 系统派生槽位（2=放坡设计面，3=放坡缝合地形）受写保护，用户期数按 0 -> 1 -> 10 -> 11 -> ...
 *        自动分配或按需创建，互不冲突。
 *   2. 工程级全局统一主 RTC 坐标原点自动对齐：
 *      - 当导入首个有效测绘期数时锁定工程主参考原点 `master_rtc`；
 *      - 后续导入任意第 2、3、……、N 期测绘点云时，自动以多线程并行方式将其相对坐标归算至
 *        同一 `master_rtc` 原点下（消除多期数据因包围盒中心不同产生的平面/高程偏移），保证多期三维
 *        精准叠加、任意两期土方算量零误差以及任意期数原地面放坡无缝衔接。
 *   3. 细粒度读写锁并发安全（std::shared_mutex）：
 *      - 重型文件解析、土方计算与放坡建模尽量在锁外或共享读锁下并行运行，不阻塞三维视口的
 *        高频瓦片拉取、八叉树视锥查询与鼠标射线拾取。
 *   4. 零异常泄漏 C ABI 边界防护：
 *      - 每个 C API 入口均采用 `try / catch` 全量兜底并将详细中文诊断信息写入线程局部
 *        `thread_local` 错误缓存，绝不跨语言边界抛出 C++ 异常。
 *   5. 跨平台安全结构化日志分发：
 *      - 优先路由至宿主通过 `geo_set_log_callback` 注入的日志桥接回调；在无宿主回调时
 *        采用原生操作系统写接口或安全标准输出，彻底杜绝 Windows Debug UCRT 宽窄流模式断言。
 */

#include "geo_kernel/geo_kernel_c_api.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <exception>
#include <filesystem>
#include <memory>
#include <mutex>
#include <new>
#include <shared_mutex>
#include <string>
#include <string_view>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#if defined(_WIN32)
    #include <windows.h>
#else
    #include <unistd.h>
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

using namespace geo_kernel;

constexpr const char* kGeoKernelVersionString = "GeoKernelCore 2.3.0 (C++20 Multi-Period Engine)";

// 线程局部最近一次错误诊断消息（保证多线程并发调用互不覆盖错误信息）
thread_local std::string g_tls_last_error = "操作成功";

// 全局结构化日志回调及其线程安全锁
std::mutex g_log_callback_mutex;
GeoLogCallback g_log_callback = nullptr;
void* g_log_user_data = nullptr;

/**
 * @brief 将内部 StatusCode 映射为 C ABI GeoStatus
 */
inline GeoStatus ToCStatus(StatusCode code) noexcept {
    return static_cast<GeoStatus>(static_cast<int32_t>(code));
}

/**
 * @brief 记录当前线程的错误诊断文本并返回对应状态码
 */
GeoStatus SetLastError(GeoStatus status, std::string msg) noexcept {
    try {
        g_tls_last_error = std::move(msg);
    } catch (...) {
        // 极端内存耗尽情况下静默保护
    }
    return status;
}

GeoStatus SetLastErrorFromResult(const OperationResult& res) noexcept {
    if (res.IsOk()) {
        return SetLastError(GEO_STATUS_OK, "操作成功");
    }
    return SetLastError(ToCStatus(res.code), res.message);
}

/**
 * @brief 将 UTF-8 C 字符串安全转换为跨平台 std::filesystem::path（严格兼容 C++20 无废弃警告）
 */
std::filesystem::path Utf8ToFilesystemPath(const char* utf8_str) {
    if (utf8_str == nullptr) {
        return {};
    }
    const auto* u8_ptr = reinterpret_cast<const char8_t*>(utf8_str);
    return std::filesystem::path(std::u8string_view(u8_ptr));
}

/**
 * @brief 提取源码路径中的纯文件名，简化日志可读性
 */
std::string_view ExtractBaseFileName(const char* file_path) noexcept {
    if (file_path == nullptr) {
        return "unknown";
    }
    std::string_view sv(file_path);
    const size_t pos = sv.find_last_of("/\\");
    return (pos == std::string_view::npos) ? sv : sv.substr(pos + 1);
}

/**
 * @brief 判断槽位编号是否为系统保留的放坡派生槽位（2 或 3）
 */
inline bool IsSystemDerivedSlot(int32_t slot_id) noexcept {
    return slot_id == GEO_SURFACE_GRADING_DESIGN_ONLY ||
           slot_id == GEO_SURFACE_GRADING_MERGED_TERRAIN;
}

/**
 * @brief 将 C ABI 扁平三维坐标数组 (x0,y0,z0,x1,y1,z1,...) 转换为 Vec3d 序列
 */
std::vector<Vec3d> UnpackVec3Array(const double* xyz_array, uint32_t point_count) {
    std::vector<Vec3d> out;
    if (xyz_array == nullptr || point_count == 0) {
        return out;
    }
    out.reserve(point_count);
    for (uint32_t i = 0; i < point_count; ++i) {
        const size_t base = static_cast<size_t>(i) * 3u;
        out.emplace_back(xyz_array[base], xyz_array[base + 1], xyz_array[base + 2]);
    }
    return out;
}

/**
 * @brief 将 C++ EarthworkReport 填充到 C ABI GeoEarthworkSummary
 */
void FillEarthworkSummary(const EarthworkReport& src, GeoEarthworkSummary& dst) noexcept {
    dst.cut_volume_m3 = src.cut_volume_m3;
    dst.fill_volume_m3 = src.fill_volume_m3;
    dst.net_volume_m3 = src.net_volume_m3;
    dst.total_overlap_area_m2 = src.total_overlap_area_m2;
    dst.cut_area_m2 = src.cut_area_m2;
    dst.fill_area_m2 = src.fill_area_m2;
    dst.zero_area_m2 = src.zero_area_m2;
    dst.max_cut_depth_m = src.max_cut_depth_m;
    dst.max_fill_height_m = src.max_fill_height_m;
    dst.intersected_prisms = src.intersected_prisms;
    dst.elapsed_seconds = src.elapsed_seconds;
}

/**
 * @brief 多线程并行平移点云数组至统一工程主 RTC 原点坐标系
 */
void ShiftPointsToMasterRtc(std::vector<SurveyPoint>& points, const Vec3d& delta) {
    if (points.empty()) {
        return;
    }
    if (std::abs(delta.x) <= 1e-12 && std::abs(delta.y) <= 1e-12 && std::abs(delta.z) <= 1e-12) {
        return;
    }
    const size_t n = points.size();
    unsigned int hw = std::thread::hardware_concurrency();
    size_t num_threads = (n >= 200000 && hw > 1) ? static_cast<size_t>(hw) : 1;
    if (num_threads <= 1) {
        for (auto& pt : points) {
            pt.x += delta.x;
            pt.y += delta.y;
            pt.z += delta.z;
        }
        return;
    }
    std::vector<std::thread> workers;
    workers.reserve(num_threads);
    const size_t chunk = (n + num_threads - 1) / num_threads;
    for (size_t t = 0; t < num_threads; ++t) {
        const size_t begin_idx = t * chunk;
        if (begin_idx >= n) break;
        const size_t end_idx = (std::min)(n, begin_idx + chunk);
        workers.emplace_back([&points, begin_idx, end_idx, delta]() {
            for (size_t i = begin_idx; i < end_idx; ++i) {
                points[i].x += delta.x;
                points[i].y += delta.y;
                points[i].z += delta.z;
            }
        });
    }
    for (auto& w : workers) {
        if (w.joinable()) {
            w.join();
        }
    }
}

/**
 * @brief 单个期数或系统派生地形槽位的完整内存上下文
 */
struct SurfaceSlotData {
    std::vector<SurveyPoint> points;
    std::vector<std::string> code_dict;
    DatParseStats dat_stats;
    RtcTransform rtc_transform;
    PointCloudOctree octree;
    TiledTinSurface tin;
    std::unique_ptr<TinTopologyEditor> editor;

    SurfaceSlotData() {
        editor = std::make_unique<TinTopologyEditor>(tin, 64);
    }

    /**
     * @brief 重置八叉树、三角网及撤销历史（重新导入该期点云时调用）
     */
    void ResetDerivedGeometry() {
        octree.Clear();
        tin.Clear();
        if (editor) {
            editor->ClearHistory();
        }
    }
};

}  // namespace

// ==============================================================================
// 0. 工程上下文不透明结构体完整定义（全局作用域，与头文件中前向声明匹配）
// ==============================================================================
struct GeoProjectOpaque {
    mutable std::shared_mutex rw_mutex;
    geo_kernel::RtcTransform master_rtc;
    int32_t next_dynamic_slot_id = GEO_SURFACE_USER_PERIOD_START;
    std::unordered_map<int32_t, std::unique_ptr<SurfaceSlotData>> slots;

    /**
     * @brief 查找指定槽位（只读版本，调用方必须已持有读锁或写锁）
     */
    const SurfaceSlotData* FindSlot(int32_t slot_id) const noexcept {
        auto it = slots.find(slot_id);
        return (it != slots.end()) ? it->second.get() : nullptr;
    }

    /**
     * @brief 查找指定槽位（可写版本，调用方必须已持有写锁）
     */
    SurfaceSlotData* FindSlotMut(int32_t slot_id) noexcept {
        auto it = slots.find(slot_id);
        return (it != slots.end()) ? it->second.get() : nullptr;
    }

    /**
     * @brief 获取或创建指定槽位（调用方必须已持有写锁）
     */
    SurfaceSlotData& GetOrCreateSlot(int32_t slot_id) {
        auto& ptr = slots[slot_id];
        if (!ptr) {
            ptr = std::make_unique<SurfaceSlotData>();
            ptr->rtc_transform = master_rtc;
        }
        return *ptr;
    }

    /**
     * @brief 检测工程中是否还存在包含有效点云或三角网的用户测绘期数
     */
    bool HasAnyLoadedUserPeriod() const noexcept {
        for (const auto& kv : slots) {
            if (IsSystemDerivedSlot(kv.first) || !kv.second) {
                continue;
            }
            if (!kv.second->points.empty() || kv.second->tin.IsValid()) {
                return true;
            }
        }
        return false;
    }
};

// ==============================================================================
// 1. 内核跨模块统一结构化日志实现（供 core_types.hpp 宏调用）
// ==============================================================================
namespace geo_kernel::detail {

void WriteKernelLog(
    int32_t level,
    const char* file_path,
    int line_no,
    const char* func_name,
    std::string_view message) noexcept {
    try {
        GeoLogCallback cb = nullptr;
        void* ud = nullptr;
        {
            std::lock_guard<std::mutex> lock(g_log_callback_mutex);
            cb = g_log_callback;
            ud = g_log_user_data;
        }

        const std::string_view base_file = ExtractBaseFileName(file_path);
        std::string loc;
        loc.reserve(base_file.size() + 48);
        loc.append(base_file);
        loc.push_back(':');
        loc.append(std::to_string(line_no));
        if (func_name != nullptr && func_name[0] != '\0') {
            loc.append(" ");
            loc.append(func_name);
        }

        std::string msg_str(message);

        if (cb != nullptr) {
            cb(level, "GeoKernelCore", loc.c_str(), msg_str.c_str(), ud);
            return;
        }

        // 无宿主日志回调时（如独立命令行压测程序 GeoKernelBench）：
        // 仅输出 WARN / ERROR 级别日志至原生标准错误流，避免污染基准输出并规避 UCRT FILE* 断言
        if (level >= GEO_LOG_LEVEL_WARN) {
            const char* lv_tag = (level >= GEO_LOG_LEVEL_ERROR) ? "ERROR" : "WARN";
            std::string line = std::string("[") + lv_tag + "] [" + loc + "] " + msg_str + "\n";
#if defined(_WIN32)
            HANDLE h_err = ::GetStdHandle(STD_ERROR_HANDLE);
            if (h_err != nullptr && h_err != INVALID_HANDLE_VALUE) {
                DWORD written = 0;
                ::WriteFile(
                    h_err,
                    line.data(),
                    static_cast<DWORD>(line.size()),
                    &written,
                    nullptr);
            }
#else
            const ssize_t ignored = ::write(STDERR_FILENO, line.data(), line.size());
            (void)ignored;
#endif
        }
    } catch (...) {
        // 日志通路绝不允许向上抛出异常
    }
}

}  // namespace geo_kernel::detail

// ==============================================================================
// 2. 生命周期、日志桥接、错误诊断与任意期数动态管理接口实现
// ==============================================================================
extern "C" {

GEO_C_API const char* geo_get_version(void) {
    return kGeoKernelVersionString;
}

GEO_C_API const char* geo_get_last_error(void) {
    return g_tls_last_error.c_str();
}

GEO_C_API void geo_set_log_callback(GeoLogCallback callback, void* user_data) {
    std::lock_guard<std::mutex> lock(g_log_callback_mutex);
    g_log_callback = callback;
    g_log_user_data = user_data;
}

GEO_C_API GeoProjectHandle geo_project_create(void) {
    try {
        ExactPredicates::Initialize();
        auto* project = new GeoProjectOpaque();
        SetLastError(GEO_STATUS_OK, "工程上下文创建成功");
        GEO_LOG_INFO("已创建新的 GeoProjectOpaque 多期工程会话上下文");
        return project;
    } catch (const std::bad_alloc&) {
        SetLastError(GEO_STATUS_OUT_OF_MEMORY, "创建工程会话失败：内存不足");
        return nullptr;
    } catch (const std::exception& ex) {
        SetLastError(GEO_STATUS_INTERNAL_ERROR, std::string("创建工程会话异常: ") + ex.what());
        return nullptr;
    } catch (...) {
        SetLastError(GEO_STATUS_INTERNAL_ERROR, "创建工程会话发生未知异常");
        return nullptr;
    }
}

GEO_C_API void geo_project_destroy(GeoProjectHandle handle) {
    if (handle == nullptr) {
        return;
    }
    try {
        delete handle;
    } catch (...) {
        // 析构阶段静默保护，确保 FFI 边界绝对安全
    }
}

GEO_C_API GeoStatus geo_project_create_period_slot(
    GeoProjectHandle handle,
    int32_t* out_slot_id) {
    if (handle == nullptr || out_slot_id == nullptr) {
        return SetLastError(GEO_STATUS_INVALID_ARGUMENT, "参数非法：工程句柄或输出槽位指针为空");
    }
    try {
        std::unique_lock<std::shared_mutex> lock(handle->rw_mutex);
        int32_t allocated_id = -1;

        // 优先兼容分配槽位 0 与槽位 1，随后从 GEO_SURFACE_USER_PERIOD_START (10) 开始递增分配
        if (handle->slots.find(GEO_SURFACE_PERIOD_1_ORIGINAL) == handle->slots.end()) {
            allocated_id = GEO_SURFACE_PERIOD_1_ORIGINAL;
        } else if (handle->slots.find(GEO_SURFACE_PERIOD_2_TARGET) == handle->slots.end()) {
            allocated_id = GEO_SURFACE_PERIOD_2_TARGET;
        } else {
            int32_t candidate = (std::max)(handle->next_dynamic_slot_id,
                                           static_cast<int32_t>(GEO_SURFACE_USER_PERIOD_START));
            while (IsSystemDerivedSlot(candidate) ||
                   handle->slots.find(candidate) != handle->slots.end()) {
                ++candidate;
                if (candidate >= INT32_MAX - 1) {
                    return SetLastError(GEO_STATUS_INTERNAL_ERROR, "动态期数槽位编号已耗尽");
                }
            }
            allocated_id = candidate;
            handle->next_dynamic_slot_id = candidate + 1;
        }

        handle->GetOrCreateSlot(allocated_id);
        *out_slot_id = allocated_id;
        GEO_LOG_INFO("已动态分配新测绘期数槽位: slot=%d", allocated_id);
        return SetLastError(GEO_STATUS_OK, "分配新期数槽位成功");
    } catch (const std::bad_alloc&) {
        return SetLastError(GEO_STATUS_OUT_OF_MEMORY, "分配新期数槽位失败：内存不足");
    } catch (const std::exception& ex) {
        return SetLastError(GEO_STATUS_INTERNAL_ERROR, std::string("分配新期数槽位异常: ") + ex.what());
    }
}

GEO_C_API GeoStatus geo_project_remove_slot(
    GeoProjectHandle handle,
    int32_t surface_slot) {
    if (handle == nullptr || surface_slot < 0) {
        return SetLastError(GEO_STATUS_INVALID_ARGUMENT, "参数非法：工程句柄为空或槽位编号小于 0");
    }
    try {
        std::unique_lock<std::shared_mutex> lock(handle->rw_mutex);
        auto it = handle->slots.find(surface_slot);
        if (it == handle->slots.end()) {
            return SetLastError(GEO_STATUS_TILE_NOT_FOUND, "指定的期数槽位不存在或已释放");
        }
        handle->slots.erase(it);

        // 若全部用户测绘期数均已移除，自动重置主 RTC 原点并清理已失效的派生放坡槽位
        if (!IsSystemDerivedSlot(surface_slot) && !handle->HasAnyLoadedUserPeriod()) {
            handle->master_rtc = RtcTransform{};
            handle->slots.erase(GEO_SURFACE_GRADING_DESIGN_ONLY);
            handle->slots.erase(GEO_SURFACE_GRADING_MERGED_TERRAIN);
            GEO_LOG_INFO("所有用户测绘期数已清空，已重置工程主 RTC 原点");
        } else {
            GEO_LOG_INFO("已安全卸载并释放槽位内存: slot=%d", surface_slot);
        }
        return SetLastError(GEO_STATUS_OK, "卸载指定槽位成功");
    } catch (const std::exception& ex) {
        return SetLastError(GEO_STATUS_INTERNAL_ERROR, std::string("卸载槽位异常: ") + ex.what());
    }
}

GEO_C_API GeoStatus geo_project_clear_all_periods(GeoProjectHandle handle) {
    if (handle == nullptr) {
        return SetLastError(GEO_STATUS_INVALID_ARGUMENT, "参数非法：工程句柄为空");
    }
    try {
        std::unique_lock<std::shared_mutex> lock(handle->rw_mutex);
        handle->slots.clear();
        handle->master_rtc = RtcTransform{};
        handle->next_dynamic_slot_id = GEO_SURFACE_USER_PERIOD_START;
        GEO_LOG_INFO("已清空全部工程期数与派生数据并重置主 RTC 原点");
        return SetLastError(GEO_STATUS_OK, "清空全部期数成功");
    } catch (const std::exception& ex) {
        return SetLastError(GEO_STATUS_INTERNAL_ERROR, std::string("清空工程期数异常: ") + ex.what());
    }
}

GEO_C_API GeoStatus geo_project_list_slots(
    GeoProjectHandle handle,
    int32_t* out_slot_ids,
    uint32_t capacity,
    uint32_t* out_count) {
    if (handle == nullptr || out_count == nullptr) {
        return SetLastError(GEO_STATUS_INVALID_ARGUMENT, "参数非法：工程句柄或数量输出指针为空");
    }
    try {
        std::shared_lock<std::shared_mutex> lock(handle->rw_mutex);
        std::vector<int32_t> sorted_ids;
        sorted_ids.reserve(handle->slots.size());
        for (const auto& kv : handle->slots) {
            sorted_ids.push_back(kv.first);
        }
        std::sort(sorted_ids.begin(), sorted_ids.end());

        *out_count = static_cast<uint32_t>(sorted_ids.size());
        if (out_slot_ids == nullptr || capacity == 0) {
            return SetLastError(GEO_STATUS_OK, "查询活跃槽位数量成功");
        }
        const uint32_t copy_n = (std::min)(capacity, *out_count);
        for (uint32_t i = 0; i < copy_n; ++i) {
            out_slot_ids[i] = sorted_ids[i];
        }
        return SetLastError(GEO_STATUS_OK, "枚举活跃槽位成功");
    } catch (const std::exception& ex) {
        return SetLastError(GEO_STATUS_INTERNAL_ERROR, std::string("枚举活跃槽位异常: ") + ex.what());
    }
}

GEO_C_API GeoStatus geo_project_get_master_rtc(
    GeoProjectHandle handle,
    double out_rtc_origin[3],
    bool* out_initialized) {
    if (handle == nullptr || out_rtc_origin == nullptr || out_initialized == nullptr) {
        return SetLastError(GEO_STATUS_INVALID_ARGUMENT, "参数非法：输出指针为空");
    }
    try {
        std::shared_lock<std::shared_mutex> lock(handle->rw_mutex);
        *out_initialized = handle->master_rtc.initialized;
        out_rtc_origin[0] = handle->master_rtc.origin.x;
        out_rtc_origin[1] = handle->master_rtc.origin.y;
        out_rtc_origin[2] = handle->master_rtc.origin.z;
        return SetLastError(GEO_STATUS_OK, "获取工程主 RTC 原点成功");
    } catch (const std::exception& ex) {
        return SetLastError(GEO_STATUS_INTERNAL_ERROR, std::string("获取主 RTC 原点异常: ") + ex.what());
    }
}

// ==============================================================================
// 3. 亿级 .dat 点云导入与多级 LOD 八叉树接口实现
// ==============================================================================

GEO_C_API GeoDatLoadParams geo_default_dat_load_params(void) {
    GeoDatLoadParams p{};
    p.coord_order = GEO_COORD_SURVEYING_YXZ;
    p.thread_count = 0;
    p.enable_elevation_filter = true;
    p.min_valid_elevation = -500.0;
    p.max_valid_elevation = 9000.0;
    p.filter_zero_coordinates = true;
    return p;
}

GEO_C_API GeoStatus geo_project_load_dat(
    GeoProjectHandle handle,
    int32_t surface_slot,
    const char* utf8_file_path,
    const GeoDatLoadParams* params,
    GeoProgressCallback progress_cb,
    void* user_data,
    GeoDatSummary* out_summary) {
    if (handle == nullptr || utf8_file_path == nullptr || utf8_file_path[0] == '\0') {
        return SetLastError(GEO_STATUS_INVALID_ARGUMENT, "参数非法：工程句柄为空或文件路径为空");
    }
    if (surface_slot < 0 || IsSystemDerivedSlot(surface_slot)) {
        return SetLastError(
            GEO_STATUS_INVALID_ARGUMENT,
            "非法目标槽位号：不能将原始 .dat 点云导入负槽位或系统放坡保留槽位 (2/3)");
    }

    try {
        const GeoDatLoadParams effective_params =
            (params != nullptr) ? *params : geo_default_dat_load_params();

        DatParseOptions parse_opts;
        parse_opts.coord_order = static_cast<CoordinateOrder>(effective_params.coord_order);
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

        // 在读写锁外执行重型大文件内存映射与多线程解析，保证三维视口其它期数零卡顿
        std::vector<SurveyPoint> parsed_points;
        std::vector<std::string> parsed_code_dict;
        DatParseStats parsed_stats;

        DatPointCloudParser parser;
        const std::filesystem::path fs_path = Utf8ToFilesystemPath(utf8_file_path);
        const OperationResult parse_res = parser.ParseFile(
            fs_path, parse_opts, parsed_points, parsed_code_dict, parsed_stats);
        if (!parse_res.IsOk()) {
            return SetLastErrorFromResult(parse_res);
        }

        // 写锁保护：将新导入期数对齐至工程全局统一主 RTC 原点，消除多期跨坐标中心偏移
        {
            std::unique_lock<std::shared_mutex> lock(handle->rw_mutex);

            if (!handle->master_rtc.initialized) {
                handle->master_rtc = parsed_stats.rtc_transform;
                GEO_LOG_INFO(
                    "已锁定工程全局主 RTC 坐标原点: E=%.1f, N=%.1f, H=%.1f (由槽位 %d 初始化)",
                    handle->master_rtc.origin.x,
                    handle->master_rtc.origin.y,
                    handle->master_rtc.origin.z,
                    surface_slot);
            } else {
                const Vec3d rtc_delta =
                    parsed_stats.rtc_transform.origin - handle->master_rtc.origin;
                if (rtc_delta.LengthSquared() > 1e-12) {
                    GEO_LOG_INFO(
                        "槽位 %d 点云自动归算至工程统一主 RTC 原点: 偏移修正 (dX=%.3f, dY=%.3f, dZ=%.3f)",
                        surface_slot,
                        rtc_delta.x,
                        rtc_delta.y,
                        rtc_delta.z);
                    ShiftPointsToMasterRtc(parsed_points, rtc_delta);
                }
                parsed_stats.rtc_transform = handle->master_rtc;
                parsed_stats.local_bounds.min_pt =
                    parsed_stats.global_bounds.min_pt - handle->master_rtc.origin;
                parsed_stats.local_bounds.max_pt =
                    parsed_stats.global_bounds.max_pt - handle->master_rtc.origin;
            }

            SurfaceSlotData& slot_data = handle->GetOrCreateSlot(surface_slot);
            slot_data.ResetDerivedGeometry();
            slot_data.points = std::move(parsed_points);
            slot_data.code_dict = std::move(parsed_code_dict);
            slot_data.dat_stats = parsed_stats;
            slot_data.rtc_transform = handle->master_rtc;
        }

        if (out_summary != nullptr) {
            out_summary->file_bytes = parsed_stats.file_bytes;
            out_summary->total_lines_scanned = parsed_stats.total_lines_scanned;
            out_summary->valid_points = parsed_stats.valid_points;
            out_summary->skipped_lines = parsed_stats.skipped_empty_or_comment;
            out_summary->malformed_lines = parsed_stats.malformed_lines;
            out_summary->filtered_outliers = parsed_stats.filtered_outliers;
            out_summary->elapsed_seconds = parsed_stats.elapsed_seconds;
            out_summary->throughput_mpts_per_sec = parsed_stats.throughput_mpts_per_sec;

            out_summary->rtc_origin[0] = parsed_stats.rtc_transform.origin.x;
            out_summary->rtc_origin[1] = parsed_stats.rtc_transform.origin.y;
            out_summary->rtc_origin[2] = parsed_stats.rtc_transform.origin.z;

            out_summary->global_min[0] = parsed_stats.global_bounds.min_pt.x;
            out_summary->global_min[1] = parsed_stats.global_bounds.min_pt.y;
            out_summary->global_min[2] = parsed_stats.global_bounds.min_pt.z;
            out_summary->global_max[0] = parsed_stats.global_bounds.max_pt.x;
            out_summary->global_max[1] = parsed_stats.global_bounds.max_pt.y;
            out_summary->global_max[2] = parsed_stats.global_bounds.max_pt.z;

            out_summary->local_min[0] = parsed_stats.local_bounds.min_pt.x;
            out_summary->local_min[1] = parsed_stats.local_bounds.min_pt.y;
            out_summary->local_min[2] = parsed_stats.local_bounds.min_pt.z;
            out_summary->local_max[0] = parsed_stats.local_bounds.max_pt.x;
            out_summary->local_max[1] = parsed_stats.local_bounds.max_pt.y;
            out_summary->local_max[2] = parsed_stats.local_bounds.max_pt.z;
        }

        return SetLastError(GEO_STATUS_OK, "点云导入与统一坐标归化成功");
    } catch (const std::bad_alloc&) {
        return SetLastError(GEO_STATUS_OUT_OF_MEMORY, "点云导入失败：内存分配不足");
    } catch (const std::exception& ex) {
        return SetLastError(GEO_STATUS_INTERNAL_ERROR, std::string("点云导入异常: ") + ex.what());
    }
}

GEO_C_API GeoOctreeParams geo_default_octree_params(void) {
    GeoOctreeParams p{};
    p.max_depth = 10;
    p.max_points_per_leaf = 16384;
    p.max_lod_points_per_node = 8192;
    p.thread_count = 0;
    return p;
}

GEO_C_API GeoStatus geo_project_build_octree(
    GeoProjectHandle handle,
    int32_t surface_slot,
    const GeoOctreeParams* params,
    GeoOctreeSummary* out_summary) {
    if (handle == nullptr) {
        return SetLastError(GEO_STATUS_INVALID_ARGUMENT, "参数非法：工程句柄为空");
    }
    try {
        std::unique_lock<std::shared_mutex> lock(handle->rw_mutex);
        SurfaceSlotData* slot_data = handle->FindSlotMut(surface_slot);
        if (slot_data == nullptr || slot_data->points.empty()) {
            return SetLastError(GEO_STATUS_EMPTY_DATASET, "目标槽位尚未导入有效点云数据，无法构建八叉树");
        }

        const GeoOctreeParams effective = (params != nullptr) ? *params : geo_default_octree_params();
        OctreeBuildOptions opts;
        opts.max_depth = (effective.max_depth > 0) ? effective.max_depth : 10;
        opts.max_points_per_leaf =
            (effective.max_points_per_leaf > 0) ? effective.max_points_per_leaf : 16384;
        opts.max_lod_points_per_node =
            (effective.max_lod_points_per_node > 0) ? effective.max_lod_points_per_node : 8192;
        opts.thread_count = effective.thread_count;

        OctreeBuildStats stats;
        const OperationResult res = slot_data->octree.Build(
            slot_data->points, slot_data->dat_stats.local_bounds, opts, stats);
        if (!res.IsOk()) {
            return SetLastErrorFromResult(res);
        }

        if (out_summary != nullptr) {
            out_summary->total_points = stats.total_points;
            out_summary->total_nodes = stats.total_nodes;
            out_summary->leaf_nodes = stats.leaf_nodes;
            out_summary->max_reached_depth = stats.max_reached_depth;
            out_summary->elapsed_seconds = stats.elapsed_seconds;
        }
        return SetLastError(GEO_STATUS_OK, "点云八叉树构建成功");
    } catch (const std::bad_alloc&) {
        return SetLastError(GEO_STATUS_OUT_OF_MEMORY, "构建八叉树失败：内存分配不足");
    } catch (const std::exception& ex) {
        return SetLastError(GEO_STATUS_INTERNAL_ERROR, std::string("构建八叉树异常: ") + ex.what());
    }
}

GEO_C_API GeoStatus geo_project_query_visible_octree_nodes(
    GeoProjectHandle handle,
    int32_t surface_slot,
    const GeoCameraQuery* query,
    uint32_t* out_node_ids,
    uint32_t capacity,
    uint32_t* out_node_count,
    uint64_t* out_estimated_points) {
    if (handle == nullptr || query == nullptr || out_node_count == nullptr) {
        return SetLastError(GEO_STATUS_INVALID_ARGUMENT, "参数非法：句柄、查询参数或输出计数指针为空");
    }
    try {
        std::shared_lock<std::shared_mutex> lock(handle->rw_mutex);
        const SurfaceSlotData* slot_data = handle->FindSlot(surface_slot);
        if (slot_data == nullptr || !slot_data->octree.IsBuilt()) {
            *out_node_count = 0;
            if (out_estimated_points != nullptr) {
                *out_estimated_points = 0;
            }
            return SetLastError(GEO_STATUS_EMPTY_DATASET, "目标槽位八叉树尚未构建完成");
        }

        CameraViewParams view;
        view.camera_local_pos = {
            query->camera_local_pos[0],
            query->camera_local_pos[1],
            query->camera_local_pos[2]
        };
        for (int i = 0; i < 6; ++i) {
            for (int j = 0; j < 4; ++j) {
                view.frustum_planes[i][j] = query->frustum_planes[i][j];
            }
        }
        view.viewport_height_px = (std::max)(1.0, query->viewport_height_px);
        view.fov_y_radians = (std::max)(0.05, query->fov_y_radians);
        view.sse_threshold_px = (std::max)(0.25, query->sse_threshold_px);
        view.max_point_budget = query->max_point_budget;
        view.enable_frustum_culling = query->enable_frustum_culling;

        std::vector<uint32_t> visible_ids;
        uint64_t est_pts = 0;
        slot_data->octree.QueryVisibleNodes(view, visible_ids, est_pts);

        *out_node_count = static_cast<uint32_t>(visible_ids.size());
        if (out_estimated_points != nullptr) {
            *out_estimated_points = est_pts;
        }
        if (out_node_ids != nullptr && capacity > 0 && !visible_ids.empty()) {
            const uint32_t copy_n = (std::min)(capacity, *out_node_count);
            std::memcpy(out_node_ids, visible_ids.data(), static_cast<size_t>(copy_n) * sizeof(uint32_t));
        }
        return SetLastError(GEO_STATUS_OK, "八叉树视锥体 LOD 查询成功");
    } catch (const std::exception& ex) {
        return SetLastError(GEO_STATUS_INTERNAL_ERROR, std::string("八叉树视锥查询异常: ") + ex.what());
    }
}

GEO_C_API GeoStatus geo_project_fetch_octree_node_gpu_packet(
    GeoProjectHandle handle,
    int32_t surface_slot,
    uint32_t node_id,
    uint8_t* out_buffer,
    size_t buffer_capacity,
    size_t* out_byte_size) {
    if (handle == nullptr || out_byte_size == nullptr) {
        return SetLastError(GEO_STATUS_INVALID_ARGUMENT, "参数非法：工程句柄或字节长度输出指针为空");
    }
    try {
        std::shared_lock<std::shared_mutex> lock(handle->rw_mutex);
        const SurfaceSlotData* slot_data = handle->FindSlot(surface_slot);
        if (slot_data == nullptr || !slot_data->octree.IsBuilt()) {
            return SetLastError(GEO_STATUS_EMPTY_DATASET, "目标槽位八叉树尚未构建完成");
        }

        std::vector<uint8_t> packet;
        const OperationResult res = slot_data->octree.SerializeNodeToGpuPacket(
            node_id, slot_data->points, slot_data->rtc_transform, packet);
        if (!res.IsOk()) {
            return SetLastErrorFromResult(res);
        }

        *out_byte_size = packet.size();
        if (out_buffer == nullptr || buffer_capacity == 0) {
            return SetLastError(GEO_STATUS_OK, "已返回点云数据包所需字节长度");
        }
        if (buffer_capacity < packet.size()) {
            return SetLastError(GEO_STATUS_INVALID_ARGUMENT, "接收点云数据包的缓冲区容量不足");
        }
        if (!packet.empty()) {
            std::memcpy(out_buffer, packet.data(), packet.size());
        }
        return SetLastError(GEO_STATUS_OK, "获取点云八叉树 GPU 数据包成功");
    } catch (const std::bad_alloc&) {
        return SetLastError(GEO_STATUS_OUT_OF_MEMORY, "打包八叉树节点失败：内存不足");
    } catch (const std::exception& ex) {
        return SetLastError(GEO_STATUS_INTERNAL_ERROR, std::string("获取点云数据包异常: ") + ex.what());
    }
}

// ==============================================================================
// 4. 空间分块三角网 (TIN) 构建、高程查询与 GPU 网格包接口实现
// ==============================================================================

GEO_C_API GeoTinParams geo_default_tin_params(void) {
    GeoTinParams p{};
    p.decimation_grid_size = 0.0;
    p.tile_size = 128.0;
    p.max_edge_length = 0.0;
    return p;
}

GEO_C_API GeoStatus geo_project_build_tin(
    GeoProjectHandle handle,
    int32_t surface_slot,
    const GeoTinParams* params,
    GeoProgressCallback progress_cb,
    void* user_data,
    GeoTinSummary* out_summary) {
    if (handle == nullptr) {
        return SetLastError(GEO_STATUS_INVALID_ARGUMENT, "参数非法：工程句柄为空");
    }
    try {
        std::unique_lock<std::shared_mutex> lock(handle->rw_mutex);
        SurfaceSlotData* slot_data = handle->FindSlotMut(surface_slot);
        if (slot_data == nullptr || slot_data->points.empty()) {
            return SetLastError(GEO_STATUS_EMPTY_DATASET, "目标槽位尚未导入有效点云，无法构建三角网");
        }

        const GeoTinParams effective = (params != nullptr) ? *params : geo_default_tin_params();
        TinBuildOptions opts;
        opts.decimation_grid_size = (std::max)(0.0, effective.decimation_grid_size);
        opts.tile_size = (effective.tile_size > 1.0) ? effective.tile_size : 128.0;
        opts.max_edge_length = (std::max)(0.0, effective.max_edge_length);
        if (progress_cb != nullptr) {
            opts.progress_callback = [progress_cb, user_data](double prog, uint64_t done) -> bool {
                return progress_cb(prog, done, user_data);
            };
        }

        TinBuildStats stats;
        const OperationResult res = slot_data->tin.BuildFromPoints(slot_data->points, opts, stats);
        if (!res.IsOk()) {
            return SetLastErrorFromResult(res);
        }
        if (slot_data->editor) {
            slot_data->editor->ClearHistory();
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
        return SetLastError(GEO_STATUS_OK, "分块 Delaunay 三角网构建成功");
    } catch (const std::bad_alloc&) {
        return SetLastError(GEO_STATUS_OUT_OF_MEMORY, "构建三角网失败：内存分配不足");
    } catch (const std::exception& ex) {
        return SetLastError(GEO_STATUS_INTERNAL_ERROR, std::string("构建三角网异常: ") + ex.what());
    }
}

GEO_C_API GeoStatus geo_project_query_elevation(
    GeoProjectHandle handle,
    int32_t surface_slot,
    double local_x,
    double local_y,
    double* out_local_z,
    double* out_global_z) {
    if (handle == nullptr || out_local_z == nullptr) {
        return SetLastError(GEO_STATUS_INVALID_ARGUMENT, "参数非法：工程句柄或高程输出指针为空");
    }
    try {
        std::shared_lock<std::shared_mutex> lock(handle->rw_mutex);
        const SurfaceSlotData* slot_data = handle->FindSlot(surface_slot);
        if (slot_data == nullptr || !slot_data->tin.IsValid()) {
            return SetLastError(GEO_STATUS_EMPTY_DATASET, "目标槽位三角网尚未构建");
        }

        double local_z = 0.0;
        if (!slot_data->tin.QueryElevationAt(local_x, local_y, local_z, nullptr)) {
            return SetLastError(GEO_STATUS_TILE_NOT_FOUND, "查询坐标点位于三角网覆盖范围之外");
        }
        *out_local_z = local_z;
        if (out_global_z != nullptr) {
            *out_global_z = slot_data->rtc_transform.origin.z + local_z;
        }
        return SetLastError(GEO_STATUS_OK, "高程内插查询成功");
    } catch (const std::exception& ex) {
        return SetLastError(GEO_STATUS_INTERNAL_ERROR, std::string("高程查询异常: ") + ex.what());
    }
}

GEO_C_API GeoStatus geo_project_get_dirty_tin_tiles(
    GeoProjectHandle handle,
    int32_t surface_slot,
    uint32_t* out_tile_indices,
    uint32_t capacity,
    uint32_t* out_tile_count) {
    if (handle == nullptr || out_tile_count == nullptr) {
        return SetLastError(GEO_STATUS_INVALID_ARGUMENT, "参数非法：工程句柄或脏瓦片数量输出指针为空");
    }
    try {
        std::shared_lock<std::shared_mutex> lock(handle->rw_mutex);
        const SurfaceSlotData* slot_data = handle->FindSlot(surface_slot);
        if (slot_data == nullptr || !slot_data->tin.IsValid()) {
            *out_tile_count = 0;
            return SetLastError(GEO_STATUS_OK, "目标槽位暂无活跃三角网瓦片");
        }

        const std::vector<uint32_t> dirty_tiles = slot_data->tin.GetDirtyTileIndices();
        *out_tile_count = static_cast<uint32_t>(dirty_tiles.size());
        if (out_tile_indices != nullptr && capacity > 0 && !dirty_tiles.empty()) {
            const uint32_t copy_n = (std::min)(capacity, *out_tile_count);
            std::memcpy(out_tile_indices, dirty_tiles.data(), static_cast<size_t>(copy_n) * sizeof(uint32_t));
        }
        return SetLastError(GEO_STATUS_OK, "获取三角网脏瓦片列表成功");
    } catch (const std::exception& ex) {
        return SetLastError(GEO_STATUS_INTERNAL_ERROR, std::string("获取脏瓦片列表异常: ") + ex.what());
    }
}

GEO_C_API GeoStatus geo_project_fetch_tin_tile_gpu_packet(
    GeoProjectHandle handle,
    int32_t surface_slot,
    uint32_t tile_index,
    bool clear_dirty_flag,
    uint8_t* out_buffer,
    size_t buffer_capacity,
    size_t* out_byte_size) {
    if (handle == nullptr || out_byte_size == nullptr) {
        return SetLastError(GEO_STATUS_INVALID_ARGUMENT, "参数非法：工程句柄或输出字节长度指针为空");
    }
    try {
        const bool will_mutate_dirty = clear_dirty_flag && out_buffer != nullptr && buffer_capacity > 0;
        std::vector<uint8_t> packet;

        if (will_mutate_dirty) {
            std::unique_lock<std::shared_mutex> lock(handle->rw_mutex);
            SurfaceSlotData* slot_data = handle->FindSlotMut(surface_slot);
            if (slot_data == nullptr || !slot_data->tin.IsValid()) {
                return SetLastError(GEO_STATUS_EMPTY_DATASET, "目标槽位三角网尚未构建");
            }
            const OperationResult res = slot_data->tin.SerializeTileToGpuPacket(
                tile_index, slot_data->rtc_transform, packet);
            if (!res.IsOk()) {
                return SetLastErrorFromResult(res);
            }
            *out_byte_size = packet.size();
            if (buffer_capacity < packet.size()) {
                return SetLastError(GEO_STATUS_INVALID_ARGUMENT, "接收三角网瓦片数据包的缓冲区容量不足");
            }
            if (!packet.empty()) {
                std::memcpy(out_buffer, packet.data(), packet.size());
            }
            slot_data->tin.ClearTileDirty(tile_index);
            return SetLastError(GEO_STATUS_OK, "获取三角网 GPU 瓦片包并清除脏标记成功");
        } else {
            std::shared_lock<std::shared_mutex> lock(handle->rw_mutex);
            const SurfaceSlotData* slot_data = handle->FindSlot(surface_slot);
            if (slot_data == nullptr || !slot_data->tin.IsValid()) {
                return SetLastError(GEO_STATUS_EMPTY_DATASET, "目标槽位三角网尚未构建");
            }
            const OperationResult res = slot_data->tin.SerializeTileToGpuPacket(
                tile_index, slot_data->rtc_transform, packet);
            if (!res.IsOk()) {
                return SetLastErrorFromResult(res);
            }
            *out_byte_size = packet.size();
            if (out_buffer == nullptr || buffer_capacity == 0) {
                return SetLastError(GEO_STATUS_OK, "已返回三角网瓦片数据包所需字节长度");
            }
            if (buffer_capacity < packet.size()) {
                return SetLastError(GEO_STATUS_INVALID_ARGUMENT, "接收三角网瓦片数据包的缓冲区容量不足");
            }
            if (!packet.empty()) {
                std::memcpy(out_buffer, packet.data(), packet.size());
            }
            return SetLastError(GEO_STATUS_OK, "获取三角网 GPU 瓦片包成功");
        }
    } catch (const std::bad_alloc&) {
        return SetLastError(GEO_STATUS_OUT_OF_MEMORY, "序列化三角网瓦片失败：内存不足");
    } catch (const std::exception& ex) {
        return SetLastError(GEO_STATUS_INTERNAL_ERROR, std::string("获取三角网瓦片异常: ") + ex.what());
    }
}

// ==============================================================================
// 5. 三角网交互式动态拓扑编辑与 Undo/Redo 接口实现
// ==============================================================================

GEO_C_API GeoStatus geo_project_pick_tin_ray(
    GeoProjectHandle handle,
    int32_t surface_slot,
    const double ray_origin[3],
    const double ray_dir[3],
    GeoRayHitResult* out_hit) {
    if (handle == nullptr || ray_origin == nullptr || ray_dir == nullptr || out_hit == nullptr) {
        return SetLastError(GEO_STATUS_INVALID_ARGUMENT, "参数非法：射线拾取输入或输出指针为空");
    }
    try {
        std::shared_lock<std::shared_mutex> lock(handle->rw_mutex);
        const SurfaceSlotData* slot_data = handle->FindSlot(surface_slot);
        if (slot_data == nullptr || !slot_data->tin.IsValid()) {
            out_hit->hit = false;
            out_hit->triangle_id = UINT32_MAX;
            out_hit->nearest_vertex_id = UINT32_MAX;
            out_hit->nearest_edge_slot = -1;
            out_hit->distance = 0.0;
            out_hit->hit_local_pos[0] = 0.0;
            out_hit->hit_local_pos[1] = 0.0;
            out_hit->hit_local_pos[2] = 0.0;
            return SetLastError(GEO_STATUS_OK, "目标槽位暂无可拾取三角网");
        }

        const Vec3d origin{ray_origin[0], ray_origin[1], ray_origin[2]};
        const Vec3d dir{ray_dir[0], ray_dir[1], ray_dir[2]};
        const RayHitResult hit = slot_data->tin.PickByRay(origin, dir);

        out_hit->hit = hit.hit;
        out_hit->triangle_id = hit.triangle_id;
        out_hit->nearest_vertex_id = hit.nearest_vertex_id;
        out_hit->nearest_edge_slot = hit.nearest_edge_slot;
        out_hit->distance = hit.hit ? hit.distance : 0.0;
        out_hit->hit_local_pos[0] = hit.hit_point.x;
        out_hit->hit_local_pos[1] = hit.hit_point.y;
        out_hit->hit_local_pos[2] = hit.hit_point.z;
        return SetLastError(GEO_STATUS_OK, "三维射线拾取完成");
    } catch (const std::exception& ex) {
        return SetLastError(GEO_STATUS_INTERNAL_ERROR, std::string("三维射线拾取异常: ") + ex.what());
    }
}

GEO_C_API GeoStatus geo_project_tin_flip_edge(
    GeoProjectHandle handle,
    int32_t surface_slot,
    uint32_t triangle_id,
    int32_t edge_slot,
    bool force_flip) {
    if (handle == nullptr) {
        return SetLastError(GEO_STATUS_INVALID_ARGUMENT, "参数非法：工程句柄为空");
    }
    try {
        std::unique_lock<std::shared_mutex> lock(handle->rw_mutex);
        SurfaceSlotData* slot_data = handle->FindSlotMut(surface_slot);
        if (slot_data == nullptr || !slot_data->tin.IsValid() || !slot_data->editor) {
            return SetLastError(GEO_STATUS_EMPTY_DATASET, "目标槽位三角网未初始化，无法执行对角线翻转");
        }
        return SetLastErrorFromResult(
            slot_data->editor->FlipEdge(triangle_id, edge_slot, force_flip));
    } catch (const std::exception& ex) {
        return SetLastError(GEO_STATUS_INTERNAL_ERROR, std::string("边翻转操作异常: ") + ex.what());
    }
}

GEO_C_API GeoStatus geo_project_tin_modify_vertex_z(
    GeoProjectHandle handle,
    int32_t surface_slot,
    uint32_t vertex_id,
    double new_local_z) {
    if (handle == nullptr || !std::isfinite(new_local_z)) {
        return SetLastError(GEO_STATUS_INVALID_ARGUMENT, "参数非法：句柄为空或高程数值非有限实数");
    }
    try {
        std::unique_lock<std::shared_mutex> lock(handle->rw_mutex);
        SurfaceSlotData* slot_data = handle->FindSlotMut(surface_slot);
        if (slot_data == nullptr || !slot_data->tin.IsValid() || !slot_data->editor) {
            return SetLastError(GEO_STATUS_EMPTY_DATASET, "目标槽位三角网未初始化，无法修改顶点高程");
        }
        return SetLastErrorFromResult(
            slot_data->editor->ModifyVertexElevation(vertex_id, new_local_z));
    } catch (const std::exception& ex) {
        return SetLastError(GEO_STATUS_INTERNAL_ERROR, std::string("修改顶点高程异常: ") + ex.what());
    }
}

GEO_C_API GeoStatus geo_project_tin_insert_vertex(
    GeoProjectHandle handle,
    int32_t surface_slot,
    const double local_xyz[3],
    uint16_t code_id,
    bool legalize_delaunay,
    uint32_t* out_vertex_id) {
    if (handle == nullptr || local_xyz == nullptr) {
        return SetLastError(GEO_STATUS_INVALID_ARGUMENT, "参数非法：句柄或顶点坐标指针为空");
    }
    try {
        std::unique_lock<std::shared_mutex> lock(handle->rw_mutex);
        SurfaceSlotData* slot_data = handle->FindSlotMut(surface_slot);
        if (slot_data == nullptr || !slot_data->tin.IsValid() || !slot_data->editor) {
            return SetLastError(GEO_STATUS_EMPTY_DATASET, "目标槽位三角网未初始化，无法插入新顶点");
        }
        const Vec3d pt{local_xyz[0], local_xyz[1], local_xyz[2]};
        const uint16_t flags = (code_id != 0)
            ? static_cast<uint16_t>(SurveyPoint::kFlagValid | SurveyPoint::kFlagFeature)
            : SurveyPoint::kFlagValid;
        return SetLastErrorFromResult(
            slot_data->editor->InsertVertex(pt, 0, code_id, flags, legalize_delaunay, out_vertex_id));
    } catch (const std::exception& ex) {
        return SetLastError(GEO_STATUS_INTERNAL_ERROR, std::string("插入顶点异常: ") + ex.what());
    }
}

GEO_C_API GeoStatus geo_project_tin_insert_breakline(
    GeoProjectHandle handle,
    int32_t surface_slot,
    const double* xyz_array,
    uint32_t point_count,
    uint16_t code_id,
    bool interpolate_z_from_tin) {
    if (handle == nullptr || xyz_array == nullptr || point_count < 2) {
        return SetLastError(GEO_STATUS_INVALID_ARGUMENT, "参数非法：特征折线至少需要 2 个三维顶点");
    }
    try {
        std::unique_lock<std::shared_mutex> lock(handle->rw_mutex);
        SurfaceSlotData* slot_data = handle->FindSlotMut(surface_slot);
        if (slot_data == nullptr || !slot_data->tin.IsValid() || !slot_data->editor) {
            return SetLastError(GEO_STATUS_EMPTY_DATASET, "目标槽位三角网未初始化，无法嵌入特征线");
        }
        const std::vector<Vec3d> polyline = UnpackVec3Array(xyz_array, point_count);
        return SetLastErrorFromResult(
            slot_data->editor->InsertBreakline(polyline, code_id, interpolate_z_from_tin));
    } catch (const std::exception& ex) {
        return SetLastError(GEO_STATUS_INTERNAL_ERROR, std::string("嵌入特征线异常: ") + ex.what());
    }
}

GEO_C_API GeoStatus geo_project_tin_delete_vertex(
    GeoProjectHandle handle,
    int32_t surface_slot,
    uint32_t vertex_id) {
    if (handle == nullptr) {
        return SetLastError(GEO_STATUS_INVALID_ARGUMENT, "参数非法：工程句柄为空");
    }
    try {
        std::unique_lock<std::shared_mutex> lock(handle->rw_mutex);
        SurfaceSlotData* slot_data = handle->FindSlotMut(surface_slot);
        if (slot_data == nullptr || !slot_data->tin.IsValid() || !slot_data->editor) {
            return SetLastError(GEO_STATUS_EMPTY_DATASET, "目标槽位三角网未初始化，无法删除顶点");
        }
        return SetLastErrorFromResult(slot_data->editor->DeleteVertex(vertex_id));
    } catch (const std::exception& ex) {
        return SetLastError(GEO_STATUS_INTERNAL_ERROR, std::string("删除顶点异常: ") + ex.what());
    }
}

GEO_C_API GeoStatus geo_project_tin_clip_polygon(
    GeoProjectHandle handle,
    int32_t surface_slot,
    const double* xyz_array,
    uint32_t point_count,
    bool keep_inside) {
    if (handle == nullptr || xyz_array == nullptr || point_count < 3) {
        return SetLastError(GEO_STATUS_INVALID_ARGUMENT, "参数非法：裁剪多边形至少需要 3 个顶点");
    }
    try {
        std::unique_lock<std::shared_mutex> lock(handle->rw_mutex);
        SurfaceSlotData* slot_data = handle->FindSlotMut(surface_slot);
        if (slot_data == nullptr || !slot_data->tin.IsValid() || !slot_data->editor) {
            return SetLastError(GEO_STATUS_EMPTY_DATASET, "目标槽位三角网未初始化，无法执行多边形裁剪");
        }
        const std::vector<Vec3d> polygon = UnpackVec3Array(xyz_array, point_count);
        return SetLastErrorFromResult(slot_data->editor->ClipByPolygon(polygon, keep_inside));
    } catch (const std::exception& ex) {
        return SetLastError(GEO_STATUS_INTERNAL_ERROR, std::string("多边形裁剪异常: ") + ex.what());
    }
}

GEO_C_API GeoStatus geo_project_tin_undo(GeoProjectHandle handle, int32_t surface_slot) {
    if (handle == nullptr) {
        return SetLastError(GEO_STATUS_INVALID_ARGUMENT, "参数非法：工程句柄为空");
    }
    try {
        std::unique_lock<std::shared_mutex> lock(handle->rw_mutex);
        SurfaceSlotData* slot_data = handle->FindSlotMut(surface_slot);
        if (slot_data == nullptr || !slot_data->editor) {
            return SetLastError(GEO_STATUS_EMPTY_DATASET, "目标槽位未初始化，无法撤销");
        }
        return SetLastErrorFromResult(slot_data->editor->Undo());
    } catch (const std::exception& ex) {
        return SetLastError(GEO_STATUS_INTERNAL_ERROR, std::string("撤销操作异常: ") + ex.what());
    }
}

GEO_C_API GeoStatus geo_project_tin_redo(GeoProjectHandle handle, int32_t surface_slot) {
    if (handle == nullptr) {
        return SetLastError(GEO_STATUS_INVALID_ARGUMENT, "参数非法：工程句柄为空");
    }
    try {
        std::unique_lock<std::shared_mutex> lock(handle->rw_mutex);
        SurfaceSlotData* slot_data = handle->FindSlotMut(surface_slot);
        if (slot_data == nullptr || !slot_data->editor) {
            return SetLastError(GEO_STATUS_EMPTY_DATASET, "目标槽位未初始化，无法重做");
        }
        return SetLastErrorFromResult(slot_data->editor->Redo());
    } catch (const std::exception& ex) {
        return SetLastError(GEO_STATUS_INTERNAL_ERROR, std::string("重做操作异常: ") + ex.what());
    }
}

GEO_C_API GeoStatus geo_project_tin_get_history_state(
    GeoProjectHandle handle,
    int32_t surface_slot,
    bool* out_can_undo,
    bool* out_can_redo) {
    if (handle == nullptr || out_can_undo == nullptr || out_can_redo == nullptr) {
        return SetLastError(GEO_STATUS_INVALID_ARGUMENT, "参数非法：历史状态输出指针为空");
    }
    try {
        std::shared_lock<std::shared_mutex> lock(handle->rw_mutex);
        const SurfaceSlotData* slot_data = handle->FindSlot(surface_slot);
        if (slot_data == nullptr || !slot_data->editor) {
            *out_can_undo = false;
            *out_can_redo = false;
            return SetLastError(GEO_STATUS_OK, "目标槽位暂无编辑历史");
        }
        *out_can_undo = slot_data->editor->CanUndo();
        *out_can_redo = slot_data->editor->CanRedo();
        return SetLastError(GEO_STATUS_OK, "查询编辑历史状态成功");
    } catch (const std::exception& ex) {
        return SetLastError(GEO_STATUS_INTERNAL_ERROR, std::string("查询历史状态异常: ") + ex.what());
    }
}

// ==============================================================================
// 6. 任意两期三角网高精度土方算量接口实现
// ==============================================================================

GEO_C_API GeoStatus geo_project_compute_earthwork_exact(
    GeoProjectHandle handle,
    int32_t period1_slot,
    int32_t period2_slot,
    const double* roi_xyz_array,
    uint32_t roi_point_count,
    uint32_t thread_count,
    GeoEarthworkSummary* out_summary) {
    if (handle == nullptr || out_summary == nullptr) {
        return SetLastError(GEO_STATUS_INVALID_ARGUMENT, "参数非法：句柄或土方结果输出指针为空");
    }
    if (period1_slot == period2_slot) {
        return SetLastError(GEO_STATUS_INVALID_ARGUMENT, "基准期与对比期不能选择同一期数槽位");
    }
    try {
        std::shared_lock<std::shared_mutex> lock(handle->rw_mutex);
        const SurfaceSlotData* slot1 = handle->FindSlot(period1_slot);
        const SurfaceSlotData* slot2 = handle->FindSlot(period2_slot);
        if (slot1 == nullptr || !slot1->tin.IsValid()) {
            return SetLastError(GEO_STATUS_EMPTY_DATASET, "选定的基准期尚未构建有效三角网");
        }
        if (slot2 == nullptr || !slot2->tin.IsValid()) {
            return SetLastError(GEO_STATUS_EMPTY_DATASET, "选定的对比期尚未构建有效三角网");
        }

        EarthworkOptions opts;
        opts.thread_count = thread_count;
        if (roi_xyz_array != nullptr && roi_point_count >= 3) {
            opts.roi_polygon = UnpackVec3Array(roi_xyz_array, roi_point_count);
        }

        EarthworkCalculator calc;
        EarthworkReport report;
        const OperationResult res = calc.ComputeExactTinToTinVolume(
            slot1->tin, slot2->tin, opts, report);
        if (!res.IsOk()) {
            return SetLastErrorFromResult(res);
        }

        FillEarthworkSummary(report, *out_summary);
        return SetLastError(GEO_STATUS_OK, "精确三棱柱叠加土方算量完成");
    } catch (const std::exception& ex) {
        return SetLastError(GEO_STATUS_INTERNAL_ERROR, std::string("精确土方算量异常: ") + ex.what());
    }
}

GEO_C_API GeoStatus geo_project_compute_earthwork_grid(
    GeoProjectHandle handle,
    int32_t period1_slot,
    int32_t period2_slot,
    double grid_step,
    const double* roi_xyz_array,
    uint32_t roi_point_count,
    uint32_t thread_count,
    GeoEarthworkSummary* out_summary) {
    if (handle == nullptr || out_summary == nullptr) {
        return SetLastError(GEO_STATUS_INVALID_ARGUMENT, "参数非法：句柄或土方结果输出指针为空");
    }
    if (period1_slot == period2_slot) {
        return SetLastError(GEO_STATUS_INVALID_ARGUMENT, "基准期与对比期不能选择同一期数槽位");
    }
    try {
        std::shared_lock<std::shared_mutex> lock(handle->rw_mutex);
        const SurfaceSlotData* slot1 = handle->FindSlot(period1_slot);
        const SurfaceSlotData* slot2 = handle->FindSlot(period2_slot);
        if (slot1 == nullptr || !slot1->tin.IsValid()) {
            return SetLastError(GEO_STATUS_EMPTY_DATASET, "选定的基准期尚未构建有效三角网");
        }
        if (slot2 == nullptr || !slot2->tin.IsValid()) {
            return SetLastError(GEO_STATUS_EMPTY_DATASET, "选定的对比期尚未构建有效三角网");
        }

        EarthworkOptions opts;
        opts.grid_step = (grid_step > 0.05) ? grid_step : 1.0;
        opts.thread_count = thread_count;
        if (roi_xyz_array != nullptr && roi_point_count >= 3) {
            opts.roi_polygon = UnpackVec3Array(roi_xyz_array, roi_point_count);
        }

        EarthworkCalculator calc;
        EarthworkReport report;
        const OperationResult res = calc.ComputeFastGridDifferential(
            slot1->tin, slot2->tin, opts, report, nullptr);
        if (!res.IsOk()) {
            return SetLastErrorFromResult(res);
        }

        FillEarthworkSummary(report, *out_summary);
        return SetLastError(GEO_STATUS_OK, "并行网格微分土方估算完成");
    } catch (const std::exception& ex) {
        return SetLastError(GEO_STATUS_INTERNAL_ERROR, std::string("网格微分土方估算异常: ") + ex.what());
    }
}

// ==============================================================================
// 7. 向内与向外多级放坡建模与土方计算接口实现（支持基于任意期数原地面放坡）
// ==============================================================================

GEO_C_API GeoGradingParams geo_default_grading_params(void) {
    GeoGradingParams p{};
    p.direction = static_cast<int32_t>(GradingDirection::kOutward);
    p.vertical_mode = static_cast<int32_t>(GradingVerticalMode::kAutoCutFill);
    p.use_uniform_base_elevation = false;
    p.base_elevation = 0.0;
    p.auto_daylight_to_terrain = true;
    p.cap_inward_bottom = true;
    p.perimeter_sample_step = 1.5;
    p.output_tile_size = 128.0;
    return p;
}

GEO_C_API GeoStatus geo_project_compute_slope_grading(
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
    if (handle == nullptr || base_xyz_array == nullptr || base_point_count < 3 ||
        stages == nullptr || stage_count == 0 || out_summary == nullptr) {
        return SetLastError(
            GEO_STATUS_INVALID_ARGUMENT,
            "放坡参数非法：请提供至少 3 个轮廓顶点与至少 1 级边坡参数");
    }
    try {
        std::unique_lock<std::shared_mutex> lock(handle->rw_mutex);
        const SurfaceSlotData* orig_slot = handle->FindSlot(original_terrain_slot);
        if (orig_slot == nullptr || !orig_slot->tin.IsValid()) {
            return SetLastError(
                GEO_STATUS_EMPTY_DATASET,
                "指定的放坡基准原地面期数尚未构建有效三角网");
        }

        const GeoGradingParams effective = (params != nullptr) ? *params : geo_default_grading_params();
        MultiStageGradingOptions opts;
        opts.direction = (effective.direction == 1)
            ? GradingDirection::kInward
            : GradingDirection::kOutward;
        if (effective.vertical_mode < 0) {
            opts.vertical_mode = GradingVerticalMode::kSlopeDown;
        } else if (effective.vertical_mode > 0) {
            opts.vertical_mode = GradingVerticalMode::kSlopeUp;
        } else {
            opts.vertical_mode = GradingVerticalMode::kAutoCutFill;
        }
        opts.base_polygon = UnpackVec3Array(base_xyz_array, base_point_count);
        opts.use_uniform_base_elevation = effective.use_uniform_base_elevation;
        opts.base_elevation = effective.base_elevation;
        opts.auto_daylight_to_terrain = effective.auto_daylight_to_terrain;
        opts.cap_inward_bottom = effective.cap_inward_bottom;
        opts.perimeter_sample_step = (effective.perimeter_sample_step > 0.1)
            ? effective.perimeter_sample_step
            : 1.5;
        opts.output_tile_size = (effective.output_tile_size > 1.0)
            ? effective.output_tile_size
            : 128.0;

        opts.stages.reserve(stage_count);
        for (uint32_t i = 0; i < stage_count; ++i) {
            SlopeStageSpec st;
            st.stage_height = stages[i].stage_height;
            st.slope_ratio = stages[i].slope_ratio;
            st.berm_width = stages[i].berm_width;
            st.berm_slope_ratio = stages[i].berm_slope_ratio;
            opts.stages.push_back(st);
        }

        // 先在临时三角网中生成放坡成果，避免失败时破坏已有系统放坡槽位数据
        TiledTinSurface new_design_tin;
        TiledTinSurface new_merged_tin;
        MultiStageGradingResult grading_res;

        SlopeGradingEngine engine;
        const OperationResult op_res = engine.GenerateMultiStageGrading(
            orig_slot->tin,
            opts,
            new_design_tin,
            generate_merged_terrain ? &new_merged_tin : nullptr,
            grading_res);
        if (!op_res.IsOk()) {
            return SetLastErrorFromResult(op_res);
        }

        // 写回系统保留槽位 2（纯放坡设计面）与槽位 3（缝合后完整地形）
        SurfaceSlotData& design_slot = handle->GetOrCreateSlot(GEO_SURFACE_GRADING_DESIGN_ONLY);
        design_slot.rtc_transform = handle->master_rtc;
        design_slot.tin = std::move(new_design_tin);
        if (design_slot.editor) {
            design_slot.editor->ClearHistory();
        }

        if (generate_merged_terrain) {
            SurfaceSlotData& merged_slot = handle->GetOrCreateSlot(GEO_SURFACE_GRADING_MERGED_TERRAIN);
            merged_slot.rtc_transform = handle->master_rtc;
            merged_slot.tin = std::move(new_merged_tin);
            if (merged_slot.editor) {
                merged_slot.editor->ClearHistory();
            }
        }

        // 输出封闭坡脚线三维坐标序列
        const uint32_t dl_count = static_cast<uint32_t>(grading_res.daylight_polyline.size());
        if (out_daylight_xyz != nullptr && daylight_xyz_capacity > 0 && dl_count > 0) {
            const uint32_t copy_n = (std::min)(dl_count, daylight_xyz_capacity);
            for (uint32_t i = 0; i < copy_n; ++i) {
                const size_t base = static_cast<size_t>(i) * 3u;
                out_daylight_xyz[base] = grading_res.daylight_polyline[i].x;
                out_daylight_xyz[base + 1] = grading_res.daylight_polyline[i].y;
                out_daylight_xyz[base + 2] = grading_res.daylight_polyline[i].z;
            }
        }

        FillEarthworkSummary(grading_res.earthwork, out_summary->earthwork);
        out_summary->daylight_perimeter_m = grading_res.daylight_perimeter_m;
        out_summary->slope_surface_area_m2 = grading_res.slope_surface_area_m2;
        out_summary->berm_surface_area_m2 = grading_res.berm_surface_area_m2;
        out_summary->base_pad_area_m2 = grading_res.base_pad_area_m2;
        out_summary->actual_stages_used = grading_res.actual_stages_used;
        out_summary->daylight_vertex_count = dl_count;
        out_summary->daylight_converged = grading_res.daylight_converged;
        out_summary->elapsed_seconds = grading_res.elapsed_seconds;

        return SetLastError(GEO_STATUS_OK, "多级放坡建模与土方计算完成");
    } catch (const std::bad_alloc&) {
        return SetLastError(GEO_STATUS_OUT_OF_MEMORY, "多级放坡建模失败：内存分配不足");
    } catch (const std::exception& ex) {
        return SetLastError(GEO_STATUS_INTERNAL_ERROR, std::string("多级放坡建模异常: ") + ex.what());
    }
}

}  // extern "C"

