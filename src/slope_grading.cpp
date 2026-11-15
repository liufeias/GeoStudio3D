#include "geo_kernel/slope_grading.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <unordered_map>
#include <utility>
#include <vector>
#include "geo_kernel/exact_predicates.hpp"
#include "geo_kernel/tin_editor.hpp"

namespace geo_kernel {
namespace {
constexpr double kMinSegLen = 1e-5;
constexpr double kMiterClampFactor = 2.5;

/**
 * @brief 计算二维简单多边形的有向面积（正值表示逆时针 CCW，负值表示顺时针 CW）
 */
double ComputeSignedArea2D(const std::vector<Vec3d>& poly) noexcept {
    const size_t n = poly.size();
    if (n < 3) return 0.0;
    double area2 = 0.0;
    for (size_t i = 0; i < n; ++i) {
        const Vec3d& p0 = poly[i];
        const Vec3d& p1 = poly[(i + 1) % n];
        area2 += (p0.x * p1.y - p1.x * p0.y);
    }
    return 0.5 * area2;
}

/**
 * @brief 清理多边形相邻重复顶点与零长度边，并统一规范为逆时针 (CCW) 方向
 */
std::vector<Vec3d> NormalizePolygonCCW(const std::vector<Vec3d>& input_poly) {
    std::vector<Vec3d> cleaned;
    cleaned.reserve(input_poly.size());
    for (const Vec3d& pt : input_poly) {
        if (cleaned.empty() || cleaned.back().DistanceSquared2D(pt) > kMinSegLen * kMinSegLen) {
            cleaned.push_back(pt);
        }
    }
    if (cleaned.size() >= 2 && cleaned.front().DistanceSquared2D(cleaned.back()) <= kMinSegLen * kMinSegLen) {
        cleaned.pop_back();
    }
    if (cleaned.size() < 3) {
        return {};
    }
    if (ComputeSignedArea2D(cleaned) < 0.0) {
        std::reverse(cleaned.begin(), cleaned.end());
    }
    return cleaned;
}

struct DenseStation {
    Vec3d base_pt;              ///< 基准轮廓上的三维坐标
    Vec3d outward_dir;          ///< 该采样点在基准多边形上的水平外法向
    double miter_scale = 1.0;   ///< 斜接长度放大系数
    bool is_corner = false;     ///< 是否为原始多边形角点
};

/**
 * @brief 构建基准多边形的加密剖面站点序列（含平滑斜接方向向量）
 */
std::vector<DenseStation> BuildDenseProfileStations(
    const std::vector<Vec3d>& ccw_poly,
    double sample_step) {
    const size_t n = ccw_poly.size();
    std::vector<DenseStation> stations;
    if (n < 3) return stations;
    const double step = std::max(0.25, sample_step);

    std::vector<Vec3d> edge_out_normals(n);
    std::vector<double> edge_lengths(n);
    for (size_t i = 0; i < n; ++i) {
        const Vec3d& a = ccw_poly[i];
        const Vec3d& b = ccw_poly[(i + 1) % n];
        const double dx = b.x - a.x;
        const double dy = b.y - a.y;
        const double len = std::hypot(dx, dy);
        edge_lengths[i] = len;
        if (len > 1e-12) {
            edge_out_normals[i] = {dy / len, -dx / len, 0.0};
        } else {
            edge_out_normals[i] = {0.0, 1.0, 0.0};
        }
    }

    std::vector<Vec3d> corner_dirs(n);
    std::vector<double> corner_scales(n, 1.0);
    for (size_t i = 0; i < n; ++i) {
        const size_t prev_edge = (i + n - 1) % n;
        const size_t next_edge = i;
        const Vec3d& n0 = edge_out_normals[prev_edge];
        const Vec3d& n1 = edge_out_normals[next_edge];
        Vec3d bisector{n0.x + n1.x, n0.y + n1.y, 0.0};
        const double b_len = std::hypot(bisector.x, bisector.y);
        if (b_len > 1e-9) {
            bisector.x /= b_len;
            bisector.y /= b_len;
            const double cos_half = std::max(0.2, bisector.x * n1.x + bisector.y * n1.y);
            corner_dirs[i] = bisector;
            corner_scales[i] = std::min(kMiterClampFactor, 1.0 / cos_half);
        } else {
            corner_dirs[i] = n1;
            corner_scales[i] = 1.0;
        }
    }

    for (size_t i = 0; i < n; ++i) {
        const Vec3d& a = ccw_poly[i];
        const Vec3d& b = ccw_poly[(i + 1) % n];
        const double len = edge_lengths[i];
        const int subdivs = std::max(1, static_cast<int>(std::ceil(len / step)));
        for (int s = 0; s < subdivs; ++s) {
            const double t = static_cast<double>(s) / static_cast<double>(subdivs);
            DenseStation st{};
            st.base_pt = {
                a.x + t * (b.x - a.x),
                a.y + t * (b.y - a.y),
                a.z + t * (b.z - a.z)
            };
            if (s == 0) {
                st.outward_dir = corner_dirs[i];
                st.miter_scale = corner_scales[i];
                st.is_corner = true;
            } else {
                st.outward_dir = edge_out_normals[i];
                st.miter_scale = 1.0;
                st.is_corner = false;
            }
            stations.push_back(st);
        }
    }
    return stations;
}

/**
 * @brief 计算多边形环的水平偏置并消解局部自相交折叠（适用于向内收缩与凹角外扩）
 */
std::vector<Vec3d> ComputeCleanOffsetRing(
    const std::vector<Vec3d>& ring,
    const std::vector<Vec3d>& station_dirs,
    const std::vector<double>& station_scales,
    double signed_offset_dist,
    double delta_z,
    bool& out_collapsed) {
    out_collapsed = false;
    const size_t n = ring.size();
    std::vector<Vec3d> next_ring(n);
    const double abs_dist = std::abs(signed_offset_dist);
    const double dir_sign = (signed_offset_dist >= 0.0) ? 1.0 : -1.0;

    for (size_t i = 0; i < n; ++i) {
        const double move = dir_sign * abs_dist * station_scales[i];
        next_ring[i] = {
            ring[i].x + station_dirs[i].x * move,
            ring[i].y + station_dirs[i].y * move,
            ring[i].z + delta_z
        };
    }

    if (n >= 4 && abs_dist > 1e-6) {
        const double orig_area = ComputeSignedArea2D(ring);
        const double new_area = ComputeSignedArea2D(next_ring);
        if (signed_offset_dist < 0.0 && (new_area <= 1e-3 || new_area < orig_area * 0.01)) {
            out_collapsed = true;
            Vec3d centroid{0.0, 0.0, 0.0};
            for (const auto& p : ring) {
                centroid += p;
            }
            centroid = centroid / static_cast<double>(n);
            centroid.z += delta_z;
            for (size_t i = 0; i < n; ++i) {
                next_ring[i] = centroid;
            }
            return next_ring;
        }

        for (int iter = 0; iter < 4; ++iter) {
            bool fixed_any = false;
            for (size_t i = 0; i < n; ++i) {
                const size_t j = (i + 1) % n;
                const Vec3d orig_e{ring[j].x - ring[i].x, ring[j].y - ring[i].y, 0.0};
                const Vec3d new_e{next_ring[j].x - next_ring[i].x, next_ring[j].y - next_ring[i].y, 0.0};
                if (orig_e.x * new_e.x + orig_e.y * new_e.y < -0.5 * orig_e.LengthSquared()) {
                    const Vec3d mid{
                        0.5 * (next_ring[i].x + next_ring[j].x),
                        0.5 * (next_ring[i].y + next_ring[j].y),
                        0.5 * (next_ring[i].z + next_ring[j].z)
                    };
                    next_ring[i] = mid;
                    next_ring[j] = mid;
                    fixed_any = true;
                }
            }
            if (!fixed_any) break;
        }
    }
    return next_ring;
}

/**
 * @brief 简单多边形耳切三角化（用于基底平台与向内放坡封底平面生成）
 */
void TriangulateFlatPolygon(
    const std::vector<uint32_t>& ring_vertex_ids,
    const std::vector<TinVertex>& all_vertices,
    uint16_t region_tag,
    std::vector<TinTriangle>& out_triangles) {
    const size_t n = ring_vertex_ids.size();
    if (n < 3) return;
    std::vector<uint32_t> poly;
    poly.reserve(n);
    for (uint32_t vid : ring_vertex_ids) {
        if (poly.empty()) {
            poly.push_back(vid);
        } else {
            const Vec3d& prev = all_vertices[poly.back()].Position();
            const Vec3d& curr = all_vertices[vid].Position();
            if (prev.DistanceSquared2D(curr) > 1e-10) {
                poly.push_back(vid);
            }
        }
    }
    if (poly.size() >= 2) {
        const Vec3d& first = all_vertices[poly.front()].Position();
        const Vec3d& last = all_vertices[poly.back()].Position();
        if (first.DistanceSquared2D(last) <= 1e-10) {
            poly.pop_back();
        }
    }
    if (poly.size() < 3) return;

    double area2 = 0.0;
    for (size_t i = 0; i < poly.size(); ++i) {
        const Vec3d& p0 = all_vertices[poly[i]].Position();
        const Vec3d& p1 = all_vertices[poly[(i + 1) % poly.size()]].Position();
        area2 += (p0.x * p1.y - p1.x * p0.y);
    }
    if (area2 < 0.0) {
        std::reverse(poly.begin(), poly.end());
    }

    size_t guard = 0;
    const size_t max_steps = poly.size() * poly.size() + 16;
    while (poly.size() > 3 && guard++ < max_steps) {
        bool clipped = false;
        const size_t m = poly.size();
        for (size_t i = 0; i < m; ++i) {
            const size_t prev_i = (i + m - 1) % m;
            const size_t next_i = (i + 1) % m;
            const Vec3d& pa = all_vertices[poly[prev_i]].Position();
            const Vec3d& pb = all_vertices[poly[i]].Position();
            const Vec3d& pc = all_vertices[poly[next_i]].Position();
            if (ExactPredicates::Orient2D(pa, pb, pc) <= 1e-10) {
                continue;
            }
            bool has_inside = false;
            for (size_t k = 0; k < m; ++k) {
                if (k == prev_i || k == i || k == next_i) continue;
                const uint32_t vk = poly[k];
                if (vk == poly[prev_i] || vk == poly[i] || vk == poly[next_i]) continue;
                if (ExactPredicates::PointInTriangle2D(pa, pb, pc, all_vertices[vk].Position()) != 0) {
                    has_inside = true;
                    break;
                }
            }
            if (has_inside) continue;

            TinTriangle tri{};
            tri.v[0] = poly[prev_i];
            tri.v[1] = poly[i];
            tri.v[2] = poly[next_i];
            tri.region_tag = region_tag;
            tri.is_active = 1;
            out_triangles.push_back(tri);
            poly.erase(poly.begin() + static_cast<std::ptrdiff_t>(i));
            clipped = true;
            break;
        }
        if (!clipped) {
            for (size_t k = 1; k + 1 < poly.size(); ++k) {
                const Vec3d& p0 = all_vertices[poly[0]].Position();
                const Vec3d& p1 = all_vertices[poly[k]].Position();
                const Vec3d& p2 = all_vertices[poly[k + 1]].Position();
                if (std::abs(ExactPredicates::Orient2D(p0, p1, p2)) > 1e-10) {
                    TinTriangle tri{};
                    tri.v[0] = poly[0];
                    tri.v[1] = poly[k];
                    tri.v[2] = poly[k + 1];
                    tri.region_tag = region_tag;
                    tri.is_active = 1;
                    out_triangles.push_back(tri);
                }
            }
            return;
        }
    }
    if (poly.size() == 3) {
        const Vec3d& p0 = all_vertices[poly[0]].Position();
        const Vec3d& p1 = all_vertices[poly[1]].Position();
        const Vec3d& p2 = all_vertices[poly[2]].Position();
        if (std::abs(ExactPredicates::Orient2D(p0, p1, p2)) > 1e-10) {
            TinTriangle tri{};
            tri.v[0] = poly[0];
            tri.v[1] = poly[1];
            tri.v[2] = poly[2];
            tri.region_tag = region_tag;
            tri.is_active = 1;
            out_triangles.push_back(tri);
        }
    }
}

double TriangleArea3D(const Vec3d& a, const Vec3d& b, const Vec3d& c) noexcept {
    return 0.5 * (b - a).Cross(c - a).Length();
}

/**
 * @brief 沿三维线段 (p_from -> p_to) 检测是否与原地面 TIN 相交，若相交则二分锁定毫米级精确交点
 */
bool IntersectSegmentWithTerrain(
    const TiledTinSurface& terrain,
    const Vec3d& p_from,
    const Vec3d& p_to,
    double initial_sign,
    Vec3d& out_hit_pt) noexcept {
    const double seg_len = (p_to - p_from).Length();
    if (seg_len <= 1e-9) {
        return false;
    }
    const int steps = std::max(4, static_cast<int>(std::ceil(seg_len / 0.25)));
    double prev_t = 0.0;
    double prev_diff = 0.0;
    {
        double z_g = 0.0;
        if (terrain.QueryElevationAt(p_from.x, p_from.y, z_g, nullptr)) {
            prev_diff = p_from.z - z_g;
        } else {
            prev_diff = initial_sign;
        }
    }

    for (int s = 1; s <= steps; ++s) {
        const double curr_t = static_cast<double>(s) / static_cast<double>(steps);
        const Vec3d curr_p{
            p_from.x + curr_t * (p_to.x - p_from.x),
            p_from.y + curr_t * (p_to.y - p_from.y),
            p_from.z + curr_t * (p_to.z - p_from.z)
        };
        double z_ground = 0.0;
        if (!terrain.QueryElevationAt(curr_p.x, curr_p.y, z_ground, nullptr)) {
            out_hit_pt = {
                p_from.x + prev_t * (p_to.x - p_from.x),
                p_from.y + prev_t * (p_to.y - p_from.y),
                p_from.z + prev_t * (p_to.z - p_from.z)
            };
            return true;
        }
        const double curr_diff = curr_p.z - z_ground;
        if ((prev_diff > 0.0 && curr_diff <= 0.0) ||
            (prev_diff < 0.0 && curr_diff >= 0.0) ||
            std::abs(curr_diff) <= 1e-4) {
            double lo = prev_t;
            double hi = curr_t;
            double lo_diff = prev_diff;
            for (int iter = 0; iter < 18; ++iter) {
                const double mid = 0.5 * (lo + hi);
                const double mx = p_from.x + mid * (p_to.x - p_from.x);
                const double my = p_from.y + mid * (p_to.y - p_from.y);
                const double mz = p_from.z + mid * (p_to.z - p_from.z);
                double gz = 0.0;
                if (!terrain.QueryElevationAt(mx, my, gz, nullptr)) {
                    hi = mid;
                    continue;
                }
                const double mid_diff = mz - gz;
                if ((lo_diff > 0.0 && mid_diff > 0.0) || (lo_diff < 0.0 && mid_diff < 0.0)) {
                    lo = mid;
                    lo_diff = mid_diff;
                } else {
                    hi = mid;
                }
            }
            const double final_t = 0.5 * (lo + hi);
            const double fx = p_from.x + final_t * (p_to.x - p_from.x);
            const double fy = p_from.y + final_t * (p_to.y - p_from.y);
            double fz = p_from.z + final_t * (p_to.z - p_from.z);
            double exact_gz = fz;
            if (terrain.QueryElevationAt(fx, fy, exact_gz, nullptr)) {
                fz = exact_gz;
            }
            out_hit_pt = {fx, fy, fz};
            return true;
        }
        prev_t = curr_t;
        prev_diff = curr_diff;
    }
    return false;
}
}  // namespace

// ==============================================================================
// SlopeGradingEngine 完整多级放坡建模与地形缝合实现
// ==============================================================================
OperationResult SlopeGradingEngine::GenerateMultiStageGrading(
    const TiledTinSurface& original_terrain,
    const MultiStageGradingOptions& options,
    TiledTinSurface& out_grading_surface,
    TiledTinSurface* out_merged_terrain,
    MultiStageGradingResult& out_result) const {
    const auto start_tp = std::chrono::steady_clock::now();
    out_grading_surface.Clear();
    if (out_merged_terrain != nullptr) {
        out_merged_terrain->Clear();
    }
    out_result = MultiStageGradingResult{};

    // 1. 输入参数完整性校验
    if (options.base_polygon.size() < 3) {
        return OperationResult::Fail(
            StatusCode::kInvalidArgument,
            "放坡基准轮廓多边形至少需要包含 3 个顶点");
    }
    if (options.stages.empty()) {
        return OperationResult::Fail(
            StatusCode::kInvalidArgument,
            "多级放坡参数 stages 不能为空，至少需要配置 1 级边坡参数");
    }
    for (size_t s = 0; s < options.stages.size(); ++s) {
        const auto& st = options.stages[s];
        if (st.stage_height <= 1e-4 || st.slope_ratio < 0.0 || st.berm_width < 0.0) {
            return OperationResult::Fail(
                StatusCode::kInvalidArgument,
                "第 " + std::to_string(s + 1) + " 级放坡参数非法（坡高必须 > 0，坡比和马道宽度不能为负）");
        }
    }
    if ((options.auto_daylight_to_terrain || out_merged_terrain != nullptr) &&
        !original_terrain.IsValid()) {
        return OperationResult::Fail(
            StatusCode::kEmptyDataset,
            "启用原地面求交或地形缝合时，第一期原地面三角网不能为空");
    }

    GEO_LOG_INFO(
        "GRADING",
        "启动多级放坡建模: dir=%d, vmode=%d, base_pts=%zu, stages=%zu, auto_daylight=%d, stitch=%d, step=%.2fm",
        static_cast<int>(options.direction),
        static_cast<int>(options.vertical_mode),
        options.base_polygon.size(),
        options.stages.size(),
        options.auto_daylight_to_terrain ? 1 : 0,
        out_merged_terrain != nullptr ? 1 : 0,
        options.perimeter_sample_step);

    ExactPredicates::Initialize();

    // 2. 规范化基准多边形为逆时针 CCW 顺序并应用统一高程（若启用）
    std::vector<Vec3d> ccw_base = NormalizePolygonCCW(options.base_polygon);
    if (ccw_base.size() < 3) {
        return OperationResult::Fail(
            StatusCode::kInvalidArgument,
            "基准轮廓多边形去重后有效顶点数不足 3 个");
    }
    if (options.use_uniform_base_elevation) {
        for (auto& pt : ccw_base) {
            pt.z = options.base_elevation;
        }
    }
    out_result.base_pad_area_m2 = std::abs(ComputeSignedArea2D(ccw_base));

    // 3. 沿基准轮廓加密剖面站点
    std::vector<DenseStation> stations = BuildDenseProfileStations(ccw_base, options.perimeter_sample_step);
    const size_t num_stations = stations.size();
    if (num_stations < 3) {
        return OperationResult::Fail(StatusCode::kInvalidArgument, "生成基准轮廓剖面站点失败");
    }

    // 确定每个剖面站点的垂直放坡方向 (+1 向上挖方坡 / -1 向下填方坡)
    std::vector<double> station_v_signs(num_stations, -1.0);
    for (size_t i = 0; i < num_stations; ++i) {
        if (options.vertical_mode == GradingVerticalMode::kSlopeUp) {
            station_v_signs[i] = 1.0;
        } else if (options.vertical_mode == GradingVerticalMode::kSlopeDown) {
            station_v_signs[i] = -1.0;
        } else {
            double z_ground = stations[i].base_pt.z;
            if (original_terrain.IsValid() &&
                original_terrain.QueryElevationAt(stations[i].base_pt.x, stations[i].base_pt.y, z_ground, nullptr)) {
                station_v_signs[i] = (stations[i].base_pt.z >= z_ground) ? -1.0 : 1.0;
            } else {
                station_v_signs[i] = -1.0;
            }
        }
    }
    const double h_dir_sign = (options.direction == GradingDirection::kOutward) ? 1.0 : -1.0;

    // 4. 逐级构建放坡轮廓环序列
    struct ProfileRing {
        std::vector<Vec3d> pts;
        uint16_t region_tag = 0;
        bool is_berm_ring = false;
    };
    std::vector<ProfileRing> rings;
    rings.reserve(options.stages.size() * 2 + 1);

    ProfileRing base_ring{};
    base_ring.pts.reserve(num_stations);
    for (const auto& st : stations) {
        base_ring.pts.push_back(st.base_pt);
    }
    base_ring.region_tag = 4;
    rings.push_back(std::move(base_ring));

    std::vector<Vec3d> station_dirs(num_stations);
    std::vector<double> station_scales(num_stations);
    for (size_t i = 0; i < num_stations; ++i) {
        station_dirs[i] = stations[i].outward_dir;
        station_scales[i] = stations[i].miter_scale;
    }

    std::vector<bool> station_daylighted(num_stations, false);
    std::vector<Vec3d> station_daylight_pt(num_stations);
    for (size_t i = 0; i < num_stations; ++i) {
        station_daylight_pt[i] = stations[i].base_pt;
    }

    bool inward_collapsed = false;
    uint32_t stages_executed = 0;
    for (size_t s_idx = 0; s_idx < options.stages.size(); ++s_idx) {
        if (inward_collapsed) break;
        bool all_hit = true;
        for (size_t i = 0; i < num_stations; ++i) {
            if (!station_daylighted[i]) {
                all_hit = false;
                break;
            }
        }
        if (options.auto_daylight_to_terrain && all_hit) {
            GEO_LOG_DEBUG("GRADING", "全部 %zu 个站点已提前与原地面求交收敛，在第 %zu 级提前结束", num_stations, s_idx);
            break;
        }
        const SlopeStageSpec& spec = options.stages[s_idx];
        ++stages_executed;

        // 4.1 生成本级边坡斜面终环
        const double slope_horiz = h_dir_sign * (spec.stage_height * spec.slope_ratio);
        const double avg_v_sign = station_v_signs[0];
        bool collapsed_now = false;
        std::vector<Vec3d> raw_slope_ring = ComputeCleanOffsetRing(
            rings.back().pts,
            station_dirs,
            station_scales,
            slope_horiz,
            avg_v_sign * spec.stage_height,
            collapsed_now);

        for (size_t i = 0; i < num_stations; ++i) {
            raw_slope_ring[i].z = rings.back().pts[i].z + station_v_signs[i] * spec.stage_height;
        }

        ProfileRing slope_ring{};
        slope_ring.pts.resize(num_stations);
        slope_ring.region_tag = (avg_v_sign < 0.0) ? 1 : 2;
        slope_ring.is_berm_ring = false;
        for (size_t i = 0; i < num_stations; ++i) {
            if (station_daylighted[i]) {
                slope_ring.pts[i] = station_daylight_pt[i];
                continue;
            }
            const Vec3d& p_prev = rings.back().pts[i];
            const Vec3d& p_cand = raw_slope_ring[i];
            if (options.auto_daylight_to_terrain) {
                Vec3d hit_pt{};
                if (IntersectSegmentWithTerrain(
                        original_terrain, p_prev, p_cand, -station_v_signs[i], hit_pt)) {
                    station_daylighted[i] = true;
                    station_daylight_pt[i] = hit_pt;
                    slope_ring.pts[i] = hit_pt;
                    continue;
                }
            }
            slope_ring.pts[i] = p_cand;
            station_daylight_pt[i] = p_cand;
        }
        rings.push_back(std::move(slope_ring));
        if (collapsed_now) {
            GEO_LOG_WARN("GRADING", "向内放坡在第 %zu 级边坡处中心收缩坍缩，停止继续内收", s_idx + 1);
            inward_collapsed = true;
            break;
        }

        // 4.2 若配置了马道平台宽度（berm_width > 0），生成马道外环
        if (spec.berm_width > 1e-4) {
            const double berm_horiz = h_dir_sign * spec.berm_width;
            const double berm_dz = spec.berm_width * spec.berm_slope_ratio;
            bool berm_collapsed = false;
            std::vector<Vec3d> raw_berm_ring = ComputeCleanOffsetRing(
                rings.back().pts,
                station_dirs,
                station_scales,
                berm_horiz,
                berm_dz,
                berm_collapsed);

            ProfileRing berm_ring{};
            berm_ring.pts.resize(num_stations);
            berm_ring.region_tag = 3;
            berm_ring.is_berm_ring = true;
            for (size_t i = 0; i < num_stations; ++i) {
                if (station_daylighted[i]) {
                    berm_ring.pts[i] = station_daylight_pt[i];
                    continue;
                }
                const Vec3d& p_prev = rings.back().pts[i];
                const Vec3d& p_cand = raw_berm_ring[i];
                if (options.auto_daylight_to_terrain) {
                    Vec3d hit_pt{};
                    if (IntersectSegmentWithTerrain(
                            original_terrain, p_prev, p_cand, -station_v_signs[i], hit_pt)) {
                        station_daylighted[i] = true;
                        station_daylight_pt[i] = hit_pt;
                        berm_ring.pts[i] = hit_pt;
                        continue;
                    }
                }
                berm_ring.pts[i] = p_cand;
                station_daylight_pt[i] = p_cand;
            }
            rings.push_back(std::move(berm_ring));
            if (berm_collapsed) {
                GEO_LOG_WARN("GRADING", "向内放坡在第 %zu 级马道处中心收缩坍缩，停止继续内收", s_idx + 1);
                inward_collapsed = true;
                break;
            }
        }
    }

    out_result.actual_stages_used = stages_executed;
    size_t daylighted_count = 0;
    for (size_t i = 0; i < num_stations; ++i) {
        if (station_daylighted[i]) ++daylighted_count;
    }
    out_result.daylight_converged = (daylighted_count == num_stations);

    const std::vector<Vec3d>& outer_ring_pts = rings.back().pts;
    out_result.daylight_polyline.clear();
    out_result.daylight_polyline.reserve(num_stations + 1);
    for (size_t i = 0; i < num_stations; ++i) {
        if (out_result.daylight_polyline.empty() ||
            out_result.daylight_polyline.back().DistanceSquared2D(outer_ring_pts[i]) > 1e-8) {
            out_result.daylight_polyline.push_back(outer_ring_pts[i]);
        }
    }
    if (!out_result.daylight_polyline.empty()) {
        out_result.daylight_polyline.push_back(out_result.daylight_polyline.front());
        for (size_t i = 0; i + 1 < out_result.daylight_polyline.size(); ++i) {
            out_result.daylight_perimeter_m +=
                (out_result.daylight_polyline[i + 1] - out_result.daylight_polyline[i]).Length();
        }
    }

    // 5. 构建纯放坡设计面三维网格
    std::vector<TinVertex> mesh_vertices;
    std::vector<TinTriangle> mesh_triangles;
    mesh_vertices.reserve(rings.size() * num_stations + 16);
    mesh_triangles.reserve(rings.size() * num_stations * 2 + num_stations);

    std::vector<std::vector<uint32_t>> ring_vids(rings.size(), std::vector<uint32_t>(num_stations));
    for (size_t r = 0; r < rings.size(); ++r) {
        for (size_t i = 0; i < num_stations; ++i) {
            const auto vid = static_cast<uint32_t>(mesh_vertices.size());
            TinVertex v{};
            v.x = rings[r].pts[i].x;
            v.y = rings[r].pts[i].y;
            v.z = rings[r].pts[i].z;
            v.source_point_id = vid + 1;
            v.code_id = (r == rings.size() - 1) ? 102 : 101;
            v.flags = SurveyPoint::kFlagValid | SurveyPoint::kFlagFeature;
            mesh_vertices.push_back(v);
            ring_vids[r][i] = vid;
        }
    }

    if (options.direction == GradingDirection::kOutward) {
        TriangulateFlatPolygon(ring_vids[0], mesh_vertices, 4, mesh_triangles);
    }

    double total_slope_area_3d = 0.0;
    double total_berm_area_3d = 0.0;
    for (size_t r = 0; r + 1 < rings.size(); ++r) {
        const uint16_t strip_tag = rings[r + 1].region_tag;
        const bool is_berm = rings[r + 1].is_berm_ring;
        for (size_t i = 0; i < num_stations; ++i) {
            const size_t next_i = (i + 1) % num_stations;
            const uint32_t v00 = ring_vids[r][i];
            const uint32_t v01 = ring_vids[r][next_i];
            const uint32_t v11 = ring_vids[r + 1][next_i];
            const uint32_t v10 = ring_vids[r + 1][i];
            const Vec3d& p00 = mesh_vertices[v00].Position();
            const Vec3d& p01 = mesh_vertices[v01].Position();
            const Vec3d& p11 = mesh_vertices[v11].Position();
            const Vec3d& p10 = mesh_vertices[v10].Position();

            const bool edge0_degen = (p00.DistanceSquared2D(p10) <= 1e-10);
            const bool edge1_degen = (p01.DistanceSquared2D(p11) <= 1e-10);
            if (edge0_degen && edge1_degen) {
                continue;
            }
            if (edge0_degen) {
                if (std::abs(ExactPredicates::Orient2D(p00, p01, p11)) > 1e-10) {
                    TinTriangle t{};
                    t.v[0] = v00; t.v[1] = v01; t.v[2] = v11;
                    t.region_tag = strip_tag;
                    t.is_active = 1;
                    mesh_triangles.push_back(t);
                    const double a3d = TriangleArea3D(p00, p01, p11);
                    if (is_berm) total_berm_area_3d += a3d;
                    else total_slope_area_3d += a3d;
                }
            } else if (edge1_degen) {
                if (std::abs(ExactPredicates::Orient2D(p00, p01, p10)) > 1e-10) {
                    TinTriangle t{};
                    t.v[0] = v00; t.v[1] = v01; t.v[2] = v10;
                    t.region_tag = strip_tag;
                    t.is_active = 1;
                    mesh_triangles.push_back(t);
                    const double a3d = TriangleArea3D(p00, p01, p10);
                    if (is_berm) total_berm_area_3d += a3d;
                    else total_slope_area_3d += a3d;
                }
            } else {
                if (std::abs(ExactPredicates::Orient2D(p00, p01, p11)) > 1e-10) {
                    TinTriangle t1{};
                    t1.v[0] = v00; t1.v[1] = v01; t1.v[2] = v11;
                    t1.region_tag = strip_tag;
                    t1.is_active = 1;
                    mesh_triangles.push_back(t1);
                    const double a1 = TriangleArea3D(p00, p01, p11);
                    if (is_berm) total_berm_area_3d += a1;
                    else total_slope_area_3d += a1;
                }
                if (std::abs(ExactPredicates::Orient2D(p00, p11, p10)) > 1e-10) {
                    TinTriangle t2{};
                    t2.v[0] = v00; t2.v[1] = v11; t2.v[2] = v10;
                    t2.region_tag = strip_tag;
                    t2.is_active = 1;
                    mesh_triangles.push_back(t2);
                    const double a2 = TriangleArea3D(p00, p11, p10);
                    if (is_berm) total_berm_area_3d += a2;
                    else total_slope_area_3d += a2;
                }
            }
        }
    }

    if (options.direction == GradingDirection::kInward &&
        options.cap_inward_bottom &&
        !inward_collapsed &&
        !rings.empty()) {
        TriangulateFlatPolygon(ring_vids.back(), mesh_vertices, 4, mesh_triangles);
    }

    out_result.slope_surface_area_m2 = total_slope_area_3d;
    out_result.berm_surface_area_m2 = total_berm_area_3d;

    std::vector<TinVertex> grading_verts_copy = mesh_vertices;
    std::vector<TinTriangle> grading_tris_copy = mesh_triangles;

    OperationResult build_res = out_grading_surface.BuildFromCustomMesh(
        std::move(mesh_vertices),
        std::move(mesh_triangles),
        options.output_tile_size);
    if (!build_res.IsOk()) {
        return build_res;
    }

    // 6. 生成与原地面无缝缝合的完整设计地形（out_merged_terrain）
    if (out_merged_terrain != nullptr && original_terrain.IsValid()) {
        const std::vector<Vec3d>& outer_seam_poly =
            (options.direction == GradingDirection::kOutward) ? outer_ring_pts : ccw_base;
        *out_merged_terrain = original_terrain;
        TinTopologyEditor clipper(*out_merged_terrain);
        OperationResult clip_res = clipper.ClipByPolygon(outer_seam_poly, /*keep_inside=*/false);
        if (clip_res.IsOk()) {
            std::vector<TinVertex> merged_verts = out_merged_terrain->Vertices();
            std::vector<TinTriangle> merged_tris;
            merged_tris.reserve(
                out_merged_terrain->ActiveTriangleCount() + grading_tris_copy.size());
            for (const auto& t : out_merged_terrain->Triangles()) {
                if (t.is_active) {
                    merged_tris.push_back(t);
                }
            }

            auto snap_key = [](double x, double y) noexcept -> uint64_t {
                const auto ix = static_cast<int64_t>(std::llround(x * 10000.0));
                const auto iy = static_cast<int64_t>(std::llround(y * 10000.0));
                return (static_cast<uint64_t>(ix) << 32) ^ static_cast<uint64_t>(iy);
            };
            std::unordered_map<uint64_t, uint32_t> seam_vertex_map;
            seam_vertex_map.reserve(merged_verts.size());
            for (uint32_t vid = 0; vid < merged_verts.size(); ++vid) {
                if (merged_verts[vid].IsActive()) {
                    seam_vertex_map[snap_key(merged_verts[vid].x, merged_verts[vid].y)] = vid;
                }
            }

            std::vector<uint32_t> g_to_m_vid(grading_verts_copy.size(), UINT32_MAX);
            for (uint32_t g_vid = 0; g_vid < grading_verts_copy.size(); ++g_vid) {
                const TinVertex& gv = grading_verts_copy[g_vid];
                const uint64_t k = snap_key(gv.x, gv.y);
                auto it = seam_vertex_map.find(k);
                if (it != seam_vertex_map.end() &&
                    merged_verts[it->second].Position().DistanceSquared2D(gv.Position()) <= 1e-6) {
                    g_to_m_vid[g_vid] = it->second;
                } else {
                    const auto new_m_vid = static_cast<uint32_t>(merged_verts.size());
                    merged_verts.push_back(gv);
                    seam_vertex_map[k] = new_m_vid;
                    g_to_m_vid[g_vid] = new_m_vid;
                }
            }
            for (const auto& gt : grading_tris_copy) {
                if (!gt.is_active) continue;
                TinTriangle mt = gt;
                mt.v[0] = g_to_m_vid[gt.v[0]];
                mt.v[1] = g_to_m_vid[gt.v[1]];
                mt.v[2] = g_to_m_vid[gt.v[2]];
                merged_tris.push_back(mt);
            }
            OperationResult stitch_res = out_merged_terrain->BuildFromCustomMesh(
                std::move(merged_verts),
                std::move(merged_tris),
                options.output_tile_size);
            if (!stitch_res.IsOk()) {
                return stitch_res;
            }
            GEO_LOG_INFO(
                "GRADING",
                "放坡与原地面无缝缝合成功: merged_tris=%llu",
                static_cast<unsigned long long>(out_merged_terrain->ActiveTriangleCount()));
        } else {
            GEO_LOG_WARN(
                "GRADING",
                "原地面按坡脚线裁剪失败 (已跳过缝合输出): %s",
                clip_res.message.c_str());
        }
    }

    // 7. 计算放坡设计面与原地面之间的精确土方量
    if (original_terrain.IsValid() && out_grading_surface.IsValid()) {
        EarthworkCalculator ew_calc;
        EarthworkOptions ew_opts;
        ew_opts.roi_polygon = (options.direction == GradingDirection::kOutward)
            ? outer_ring_pts
            : ccw_base;
        OperationResult ew_res = ew_calc.ComputeExactTinToTinVolume(
            original_terrain, out_grading_surface, ew_opts, out_result.earthwork);
        if (!ew_res.IsOk()) {
            GEO_LOG_WARN("GRADING", "放坡面与原地面无有效水平投影重叠，土方量报告置零: %s", ew_res.message.c_str());
            out_result.earthwork = EarthworkReport{};
        }
    }

    const auto end_tp = std::chrono::steady_clock::now();
    out_result.elapsed_seconds = std::chrono::duration<double>(end_tp - start_tp).count();

    GEO_LOG_INFO(
        "GRADING",
        "多级放坡建模汇总: stations=%zu (daylighted=%zu, converged=%d), stages=%u, perim=%.2fm, slope_area=%.2fm2, berm_area=%.2fm2, fill=%.2fm3, cut=%.2fm3, elapsed=%.4fs",
        num_stations,
        daylighted_count,
        out_result.daylight_converged ? 1 : 0,
        out_result.actual_stages_used,
        out_result.daylight_perimeter_m,
        out_result.slope_surface_area_m2,
        out_result.berm_surface_area_m2,
        out_result.earthwork.fill_volume_m3,
        out_result.earthwork.cut_volume_m3,
        out_result.elapsed_seconds);
    return OperationResult::Ok();
}

}  // namespace geo_kernel
