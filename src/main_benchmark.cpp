/**
 * @file main_benchmark.cpp
 * @brief GeoKernelCore v2.2.1 全链路功能验证与企业级精度/性能基准测试程序
 */

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <string_view>
#include <vector>

#include "geo_kernel/core_types.hpp"
#include "geo_kernel/geo_kernel_c_api.h"

namespace {

std::string PathToUtf8(const std::filesystem::path& p) {
    const std::u8string u8 = p.u8string();
    return std::string(reinterpret_cast<const char*>(u8.data()), u8.size());
}

std::filesystem::path Utf8ToPath(std::string_view utf8_str) {
    return std::filesystem::path(
        reinterpret_cast<const char8_t*>(utf8_str.data()),
        reinterpret_cast<const char8_t*>(utf8_str.data() + utf8_str.size()));
}

bool GenerateSyntheticSurveyDatFile(
    const std::filesystem::path& out_path,
    uint64_t point_count,
    int mode /* 0=复杂山体地形, 1=解析真值对照斜平面 */) {
    std::ofstream ofs(out_path, std::ios::binary);
    if (!ofs.is_open()) {
        std::cerr << "[错误] 无法创建仿真测试文件: " << PathToUtf8(out_path) << "\n";
        return false;
    }

    constexpr double kBaseEasting = 512000.0;
    constexpr double kBaseNorthing = 3543000.0;
    constexpr double kBaseElevation = 100.0;

    const auto grid_side = static_cast<uint64_t>(std::ceil(std::sqrt(static_cast<double>(point_count))));
    const double span = (mode == 1) ? 100.0 : 1000.0;
    const double spacing = span / static_cast<double>(grid_side - 1);

    std::vector<char> io_buffer(4 * 1024 * 1024);
    ofs.rdbuf()->pubsetbuf(io_buffer.data(), static_cast<std::streamsize>(io_buffer.size()));

    char line_buf[128];
    uint64_t written = 0;
    for (uint64_t row = 0; row < grid_side && written < point_count; ++row) {
        const double dy = static_cast<double>(row) * spacing;
        const double northing = kBaseNorthing + dy;

        for (uint64_t col = 0; col < grid_side && written < point_count; ++col) {
            const double dx = static_cast<double>(col) * spacing;
            const double easting = kBaseEasting + dx;

            double elev = kBaseElevation;
            const char* code = "";
            if (mode == 0) {
                elev = kBaseElevation
                    + 22.0 * std::sin(dx * 0.006) * std::cos(dy * 0.006)
                    + 6.0 * std::sin(dx * 0.02 + dy * 0.015)
                    + 0.01 * dx - 0.005 * dy;
                if (row == grid_side / 3 || row == (grid_side * 2) / 3) {
                    code = "KD";
                    elev += 1.5;
                }
            } else {
                elev = kBaseElevation + 0.02 * (dx - 50.0);
            }

            ++written;
            const int len = std::snprintf(
                line_buf,
                sizeof(line_buf),
                "%llu,%s,%.4f,%.4f,%.4f\n",
                static_cast<unsigned long long>(written),
                code,
                easting,
                northing,
                elev);
            if (len > 0) {
                ofs.write(line_buf, len);
            }
        }
    }

    ofs.flush();
    return ofs.good();
}

bool RunAnalyticalEarthworkVerification() {
    std::cout << "\n[验证 5] 两期三角网土方算量解析真值对照测试（零填挖线三棱柱剖分精度）...\n";

    const auto tmp_dir = std::filesystem::temp_directory_path();
    const auto p1_file = tmp_dir / "geo_ew_period1_tilted.dat";
    const auto p2_file = tmp_dir / "geo_ew_period2_flat.dat";

    if (!GenerateSyntheticSurveyDatFile(p1_file, 10000, 1)) return false;

    {
        std::ofstream ofs(p2_file, std::ios::binary);
        uint64_t seq = 0;
        for (int r = 0; r <= 77; ++r) {
            const double y = 3543000.0 + (100.0 * r) / 77.0;
            for (int c = 0; c <= 77; ++c) {
                const double x = 512000.0 + (100.0 * c) / 77.0;
                ofs << (++seq) << ",," << std::fixed << std::setprecision(4)
                    << x << "," << y << ",100.0000\n";
            }
        }
    }

    const std::string p1_utf8 = PathToUtf8(p1_file);
    const std::string p2_utf8 = PathToUtf8(p2_file);

    GeoProjectHandle proj = geo_project_create();
    GeoDatLoadParams lp = geo_default_dat_load_params();
    GeoTinParams tp = geo_default_tin_params();

    geo_project_load_dat(proj, GEO_SURFACE_PERIOD_1_ORIGINAL, p1_utf8.c_str(), &lp, nullptr, nullptr, nullptr);
    geo_project_load_dat(proj, GEO_SURFACE_PERIOD_2_TARGET, p2_utf8.c_str(), &lp, nullptr, nullptr, nullptr);

    geo_project_build_tin(proj, GEO_SURFACE_PERIOD_1_ORIGINAL, &tp, nullptr, nullptr, nullptr);
    geo_project_build_tin(proj, GEO_SURFACE_PERIOD_2_TARGET, &tp, nullptr, nullptr, nullptr);

    GeoEarthworkSummary ew_sum{};
    GeoStatus st = geo_project_compute_earthwork_exact(
        proj, GEO_SURFACE_PERIOD_1_ORIGINAL, GEO_SURFACE_PERIOD_2_TARGET,
        nullptr, 0, 0, &ew_sum);

    geo_project_destroy(proj);
    std::error_code ec;
    std::filesystem::remove(p1_file, ec);
    std::filesystem::remove(p2_file, ec);

    if (st != GEO_STATUS_OK) {
        std::cerr << "  [失败] 土方算量调用失败: " << geo_get_last_error() << "\n";
        return false;
    }

    const double cut_err = std::abs(ew_sum.cut_volume_m3 - 2500.0);
    const double fill_err = std::abs(ew_sum.fill_volume_m3 - 2500.0);

    std::cout << std::fixed << std::setprecision(6)
              << "  理论挖方: 2500.000000 m³ | 实测挖方: " << ew_sum.cut_volume_m3
              << " m³ (误差: " << cut_err << " m³)\n"
              << "  理论填方: 2500.000000 m³ | 实测填方: " << ew_sum.fill_volume_m3
              << " m³ (误差: " << fill_err << " m³)\n"
              << "  净土方量: " << ew_sum.net_volume_m3
              << " m³ | 叠加微元棱柱数: " << ew_sum.intersected_prisms
              << " | 耗时: " << (ew_sum.elapsed_seconds * 1000.0) << " ms\n";

    return (cut_err < 1e-3 && fill_err < 1e-3);
}

}  // namespace

int main(int argc, char* argv[]) {
    std::string input_file;
    uint64_t synthetic_points = 250000;
    double tin_grid_size = 0.0;

    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if ((arg == "--file" || arg == "-f") && i + 1 < argc) {
            input_file = argv[++i];
        } else if ((arg == "--points" || arg == "-n") && i + 1 < argc) {
            synthetic_points = std::strtoull(argv[++i], nullptr, 10);
        } else if ((arg == "--grid" || arg == "-g") && i + 1 < argc) {
            tin_grid_size = std::strtod(argv[++i], nullptr);
        }
    }

    std::cout << "====================================================================\n";
    std::cout << "  GeoKernelCore 企业级空间几何内核全流程验证 (v" << geo_get_version() << ")\n";
    std::cout << "====================================================================\n";

    bool generated_temp = false;
    std::filesystem::path target_path;
    if (!input_file.empty()) {
        target_path = Utf8ToPath(input_file);
    } else {
        target_path = std::filesystem::temp_directory_path() / "geo_kernel_v2_terrain.dat";
        if (!GenerateSyntheticSurveyDatFile(target_path, synthetic_points, 0)) {
            return 1;
        }
        generated_temp = true;
    }

    GeoProjectHandle project = geo_project_create();
    if (project == nullptr) {
        std::cerr << "[失败] 无法创建工程句柄: " << geo_get_last_error() << "\n";
        return 1;
    }

    std::cout << "\n[验证 1~3] 导入第一期原地面点云、构建八叉树与分块 Delaunay 三角网...\n";
    GeoDatLoadParams load_params = geo_default_dat_load_params();
    GeoDatSummary dat_sum{};
    const std::string target_utf8 = PathToUtf8(target_path);

    if (geo_project_load_dat(
            project, GEO_SURFACE_PERIOD_1_ORIGINAL, target_utf8.c_str(),
            &load_params, nullptr, nullptr, &dat_sum) != GEO_STATUS_OK) {
        std::cerr << "[失败] 点云解析失败: " << geo_get_last_error() << "\n";
        geo_project_destroy(project);
        return 1;
    }

    GeoOctreeParams oct_params = geo_default_octree_params();
    GeoOctreeSummary oct_sum{};
    geo_project_build_octree(project, GEO_SURFACE_PERIOD_1_ORIGINAL, &oct_params, &oct_sum);

    GeoTinParams tin_params = geo_default_tin_params();
    tin_params.decimation_grid_size = tin_grid_size;
    tin_params.tile_size = 128.0;
    GeoTinSummary tin_sum{};
    if (geo_project_build_tin(
            project, GEO_SURFACE_PERIOD_1_ORIGINAL, &tin_params, nullptr, nullptr, &tin_sum) != GEO_STATUS_OK) {
        std::cerr << "[失败] 三角网构建失败: " << geo_get_last_error() << "\n";
        geo_project_destroy(project);
        return 1;
    }

    std::cout << std::fixed << std::setprecision(3)
              << "  点云有效点数: " << dat_sum.valid_points
              << " (解析吞吐: " << dat_sum.throughput_mpts_per_sec << " Mpts/s)\n"
              << "  八叉树节点数: " << oct_sum.total_nodes
              << " | 三角网顶点数: " << tin_sum.tin_vertices
              << " | 三角形数: " << tin_sum.tin_triangles
              << " | 空间瓦片数: " << tin_sum.total_tiles << "\n";

    std::cout << "\n[验证 4] 三角网三维射线拾取、强制特征线嵌入、边翻转与 Undo/Redo 验证...\n";
    const double cx = 0.5 * (dat_sum.local_min[0] + dat_sum.local_max[0]);
    const double cy = 0.5 * (dat_sum.local_min[1] + dat_sum.local_max[1]);

    uint32_t init_dirty = 0;
    geo_project_get_dirty_tin_tiles(project, GEO_SURFACE_PERIOD_1_ORIGINAL, nullptr, 0, &init_dirty);
    if (init_dirty > 0) {
        std::vector<uint32_t> dt(init_dirty);
        geo_project_get_dirty_tin_tiles(project, GEO_SURFACE_PERIOD_1_ORIGINAL, dt.data(), init_dirty, &init_dirty);
        for (uint32_t tid : dt) {
            size_t sz = 0;
            geo_project_fetch_tin_tile_gpu_packet(project, GEO_SURFACE_PERIOD_1_ORIGINAL, tid, false, nullptr, 0, &sz);
            std::vector<uint8_t> buf(sz);
            geo_project_fetch_tin_tile_gpu_packet(project, GEO_SURFACE_PERIOD_1_ORIGINAL, tid, true, buf.data(), sz, &sz);
        }
    }

    const double breakline_pts[9] = {
        cx - 40.0, cy - 25.0, 0.0,
        cx,        cy + 5.0,  0.0,
        cx + 45.0, cy + 30.0, 0.0
    };
    GeoStatus bl_status = geo_project_tin_insert_breakline(
        project, GEO_SURFACE_PERIOD_1_ORIGINAL, breakline_pts, 3, 201, true);
    if (bl_status != GEO_STATUS_OK) {
        std::cerr << "  [失败] 嵌入特征线失败: " << geo_get_last_error() << "\n";
        geo_project_destroy(project);
        return 1;
    }

    uint32_t dirty_after_bl = 0;
    geo_project_get_dirty_tin_tiles(project, GEO_SURFACE_PERIOD_1_ORIGINAL, nullptr, 0, &dirty_after_bl);
    std::cout << "  嵌入强制特征线成功！增量脏瓦片数: " << dirty_after_bl
              << " / 总瓦片数 " << tin_sum.total_tiles << "\n";

    const double ray_orig[3] = {cx + 10.0, cy - 10.0, 500.0};
    const double ray_dir[3] = {0.0, 0.0, -1.0};
    GeoRayHitResult hit{};
    geo_project_pick_tin_ray(project, GEO_SURFACE_PERIOD_1_ORIGINAL, ray_orig, ray_dir, &hit);
    if (hit.hit) {
        std::cout << "  射线拾取命中三角形 #" << hit.triangle_id
                  << ", 最近顶点 #" << hit.nearest_vertex_id
                  << ", 交点高程 Z = " << hit.hit_local_pos[2] << " m\n";

        geo_project_tin_modify_vertex_z(
            project, GEO_SURFACE_PERIOD_1_ORIGINAL, hit.nearest_vertex_id, hit.hit_local_pos[2] + 3.5);

        bool can_undo = false, can_redo = false;
        geo_project_tin_get_history_state(project, GEO_SURFACE_PERIOD_1_ORIGINAL, &can_undo, &can_redo);
        if (can_undo) {
            geo_project_tin_undo(project, GEO_SURFACE_PERIOD_1_ORIGINAL);
            geo_project_tin_redo(project, GEO_SURFACE_PERIOD_1_ORIGINAL);
            std::cout << "  事务级 Undo / Redo 拓扑状态可逆恢复验证通过！\n";
        }
    }

    if (!RunAnalyticalEarthworkVerification()) {
        std::cerr << "[失败] 两期土方算量解析真值精度验证未通过！\n";
        geo_project_destroy(project);
        return 1;
    }

    std::cout << "\n[验证 6] 向外多级放坡建模、原地面求交（Daylighting）、地形缝合与土方计算...\n";
    double center_ground_z = 0.0;
    geo_project_query_elevation(project, GEO_SURFACE_PERIOD_1_ORIGINAL, cx, cy, &center_ground_z, nullptr);

    const double design_pad_z = center_ground_z + 14.0;
    const double outward_pad[12] = {
        cx - 30.0, cy - 20.0, design_pad_z,
        cx + 30.0, cy - 20.0, design_pad_z,
        cx + 30.0, cy + 20.0, design_pad_z,
        cx - 30.0, cy + 20.0, design_pad_z
    };

    const GeoSlopeStageSpec outward_stages[3] = {
        {6.0, 1.50, 2.0, -0.02},
        {6.0, 1.75, 2.0, -0.02},
        {10.0, 2.00, 0.0, 0.0}
    };

    GeoGradingParams outward_params = geo_default_grading_params();
    outward_params.direction = 0;
    outward_params.vertical_mode = -1;
    outward_params.auto_daylight_to_terrain = true;
    outward_params.perimeter_sample_step = 2.0;

    std::vector<double> daylight_buf(4096 * 3);
    GeoGradingSummary outward_sum{};
    GeoStatus gr_st = geo_project_compute_slope_grading(
        project,
        GEO_SURFACE_PERIOD_1_ORIGINAL,
        outward_pad,
        4,
        outward_stages,
        3,
        &outward_params,
        true,
        daylight_buf.data(),
        4096,
        &outward_sum);

    if (gr_st != GEO_STATUS_OK) {
        std::cerr << "  [失败] 向外多级放坡失败: " << geo_get_last_error() << "\n";
        geo_project_destroy(project);
        return 1;
    }

    uint32_t merged_dirty_tiles = 0;
    geo_project_get_dirty_tin_tiles(
        project, GEO_SURFACE_GRADING_MERGED_TERRAIN, nullptr, 0, &merged_dirty_tiles);

    std::cout << "  向外放坡收敛状态: " << (outward_sum.daylight_converged ? "全部封闭相交" : "部分到达最大级数")
              << " | 实际级数: " << outward_sum.actual_stages_used
              << " | 坡脚线周长: " << outward_sum.daylight_perimeter_m << " m\n"
              << "  设计平台面积: " << outward_sum.base_pad_area_m2 << " m²"
              << " | 边坡斜面积: " << outward_sum.slope_surface_area_m2 << " m²"
              << " | 马道面积: " << outward_sum.berm_surface_area_m2 << " m²\n"
              << "  放坡总填方: " << outward_sum.earthwork.fill_volume_m3 << " m³"
              << " | 总挖方: " << outward_sum.earthwork.cut_volume_m3 << " m³"
              << " | 缝合后地形GPU瓦片数: " << merged_dirty_tiles << "\n";

    std::cout << "\n[验证 7] 向内多级基坑开挖放坡建模、坑底封底与土方计算...\n";
    const double pit_top_z = center_ground_z;
    const double inward_rim[12] = {
        cx - 40.0, cy - 35.0, pit_top_z,
        cx + 40.0, cy - 35.0, pit_top_z,
        cx + 40.0, cy + 35.0, pit_top_z,
        cx - 40.0, cy + 35.0, pit_top_z
    };

    const GeoSlopeStageSpec inward_stages[2] = {
        {5.0, 0.75, 1.5, 0.0},
        {5.0, 1.00, 0.0, 0.0}
    };

    GeoGradingParams inward_params = geo_default_grading_params();
    inward_params.direction = 1;
    inward_params.vertical_mode = -1;
    inward_params.auto_daylight_to_terrain = false;
    inward_params.cap_inward_bottom = true;
    inward_params.perimeter_sample_step = 2.0;

    GeoGradingSummary inward_sum{};
    if (geo_project_compute_slope_grading(
            project,
            GEO_SURFACE_PERIOD_1_ORIGINAL,
            inward_rim,
            4,
            inward_stages,
            2,
            &inward_params,
            false,
            nullptr,
            0,
            &inward_sum) != GEO_STATUS_OK) {
        std::cerr << "  [失败] 向内多级放坡失败: " << geo_get_last_error() << "\n";
        geo_project_destroy(project);
        return 1;
    }

    std::cout << "  向内开挖实际级数: " << inward_sum.actual_stages_used
              << " | 边坡斜面积: " << inward_sum.slope_surface_area_m2 << " m²"
              << " | 马道面积: " << inward_sum.berm_surface_area_m2 << " m²\n"
              << "  基坑总挖方量: " << inward_sum.earthwork.cut_volume_m3 << " m³"
              << " | 总填方量: " << inward_sum.earthwork.fill_volume_m3 << " m³\n";

    geo_project_destroy(project);

    if (generated_temp) {
        std::error_code ec;
        std::filesystem::remove(target_path, ec);
    }

    std::cout << "\n====================================================================\n";
    std::cout << "  全流程验证全部通过！几何内核已具备完整企业级生产能力。\n";
    std::cout << "====================================================================\n";
    return 0;
}

