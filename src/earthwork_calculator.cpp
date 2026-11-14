#include "geo_kernel/earthwork_calculator.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <thread>
#include <unordered_map>
#include <vector>

#include "geo_kernel/exact_predicates.hpp"

namespace geo_kernel {

namespace {

/**
 * @brief Kahan 补偿高精度浮点数累加器（消除千万级微元体积求和时的舍入误差）
 */
struct KahanAccumulator {
    double sum = 0.0;
    double compensation = 0.0;

    void Add(double value) noexcept {
        const double y = value - compensation;
        const double t = sum + y;
        compensation = (t - sum) - y;
        sum = t;
    }
};

struct Point2D {
    double x = 0.0;
    double y = 0.0;
};

/**
 * @brief 静态容量凸多边形缓冲区（栈上分配，零堆内存开销，最多容纳 16 个顶点）
 */
struct SmallConvexPoly {
    std::array<Point2D, 16> pts{};
    size_t count = 0;

    void Clear() noexcept { count = 0; }

    void Push(double x, double y) noexcept {
        if (count > 0) {
            const double dx = pts[count - 1].x - x;
            const double dy = pts[count - 1].y - y;
            if (dx * dx + dy * dy <= 1e-16) {
                return;  // 过滤相邻重复顶点
            }
        }
        if (count < pts.size()) {
            pts[count++] = {x, y};
        }
    }

    void Finalize() noexcept {
        if (count >= 2) {
            const double dx = pts[0].x - pts[count - 1].x;
            const double dy = pts[0].y - pts[count - 1].y;
            if (dx * dx + dy * dy <= 1e-16) {
                --count;
            }
        }
    }
};

/**
 * @brief Sutherland-Hodgman 凸多边形单条有向半平面裁剪
 *        保留位于有向边 (edge_a -> edge_b) 左侧（Orient2D >= 0）的顶点部分
 */
void ClipPolyByHalfPlane(
    const SmallConvexPoly& in_poly,
    const Point2D& edge_a,
    const Point2D& edge_b,
    SmallConvexPoly& out_poly) noexcept {
    out_poly.Clear();
    if (in_poly.count < 3) return;

    Point2D prev = in_poly.pts[in_poly.count - 1];
    double prev_side = ExactPredicates::Orient2D(edge_a.x, edge_a.y, edge_b.x, edge_b.y, prev.x, prev.y);

    for (size_t i = 0; i < in_poly.count; ++i) {
        const Point2D curr = in_poly.pts[i];
        const double curr_side = ExactPredicates::Orient2D(edge_a.x, edge_a.y, edge_b.x, edge_b.y, curr.x, curr.y);

        if (curr_side >= 0.0) {
            if (prev_side < 0.0) {
                // 线段从外部穿入内部：计算精确交点
                const double denom = prev_side - curr_side;
                const double t = (std::abs(denom) > 1e-24) ? (prev_side / denom) : 0.5;
                out_poly.Push(
                    prev.x + t * (curr.x - prev.x),
                    prev.y + t * (curr.y - prev.y));
            }
            out_poly.Push(curr.x, curr.y);
        } else if (prev_side >= 0.0) {
            // 线段从内部穿出外部：计算精确交点
            const double denom = prev_side - curr_side;
            const double t = (std::abs(denom) > 1e-24) ? (prev_side / denom) : 0.5;
            out_poly.Push(
                prev.x + t * (curr.x - prev.x),
                prev.y + t * (curr.y - prev.y));
        }

        prev = curr;
        prev_side = curr_side;
    }

    out_poly.Finalize();
}

/**
 * @brief 计算二维三角形的有向面积绝对值
 */
inline double TriangleArea2D(const Point2D& a, const Point2D& b, const Point2D& c) noexcept {
    return 0.5 * std::abs((b.x - a.x) * (c.y - a.y) - (c.x - a.x) * (b.y - a.y));
}

/**
 * @brief 单个微元三棱柱的精确积分器（自动处理横跨零填挖线的三棱柱剖分）
 */
void IntegrateMicroPrism(
    const Point2D& p0, const Point2D& p1, const Point2D& p2,
    double dh0, double dh1, double dh2,
    KahanAccumulator& cut_vol,
    KahanAccumulator& fill_vol,
    KahanAccumulator& cut_area,
    KahanAccumulator& fill_area,
    KahanAccumulator& zero_area,
    double& max_cut,
    double& max_fill) noexcept {
    const double area = TriangleArea2D(p0, p1, p2);
    if (area <= 1e-14) {
        return;
    }

    constexpr double kZeroEps = 1e-9;
    const double dhs[3] = {dh0, dh1, dh2};
    const Point2D pts[3] = {p0, p1, p2};

    int pos_cnt = 0;
    int neg_cnt = 0;
    for (int i = 0; i < 3; ++i) {
        if (dhs[i] > kZeroEps) {
            ++pos_cnt;
            if (dhs[i] > max_fill) max_fill = dhs[i];
        } else if (dhs[i] < -kZeroEps) {
            ++neg_cnt;
            if (-dhs[i] > max_cut) max_cut = -dhs[i];
        }
    }

    // 情况 1：纯零填挖平面
    if (pos_cnt == 0 && neg_cnt == 0) {
        zero_area.Add(area);
        return;
    }

    // 情况 2：纯填方三棱柱（所有角点 dh >= 0）
    if (neg_cnt == 0) {
        const double avg_h = (std::max(0.0, dh0) + std::max(0.0, dh1) + std::max(0.0, dh2)) / 3.0;
        fill_vol.Add(area * avg_h);
        fill_area.Add(area);
        return;
    }

    // 情况 3：纯挖方三棱柱（所有角点 dh <= 0）
    if (pos_cnt == 0) {
        const double avg_d = (std::max(0.0, -dh0) + std::max(0.0, -dh1) + std::max(0.0, -dh2)) / 3.0;
        cut_vol.Add(area * avg_d);
        cut_area.Add(area);
        return;
    }

    // 情况 4：混合符号三棱柱（横跨零填挖等值线 dh = 0）
    int lone_idx = 0;
    if ((dhs[0] > 0.0 && dhs[1] <= 0.0 && dhs[2] <= 0.0) ||
        (dhs[0] < 0.0 && dhs[1] >= 0.0 && dhs[2] >= 0.0)) {
        lone_idx = 0;
    } else if ((dhs[1] > 0.0 && dhs[0] <= 0.0 && dhs[2] <= 0.0) ||
               (dhs[1] < 0.0 && dhs[0] >= 0.0 && dhs[2] >= 0.0)) {
        lone_idx = 1;
    } else {
        lone_idx = 2;
    }

    const int i1 = (lone_idx + 1) % 3;
    const int i2 = (lone_idx + 2) % 3;

    const double h_lone = dhs[lone_idx];
    const double h_1 = dhs[i1];
    const double h_2 = dhs[i2];

    const double t1 = h_lone / (h_lone - h_1);
    const double t2 = h_lone / (h_lone - h_2);

    const Point2D q1{
        pts[lone_idx].x + t1 * (pts[i1].x - pts[lone_idx].x),
        pts[lone_idx].y + t1 * (pts[i1].y - pts[lone_idx].y)
    };
    const Point2D q2{
        pts[lone_idx].x + t2 * (pts[i2].x - pts[lone_idx].x),
        pts[lone_idx].y + t2 * (pts[i2].y - pts[lone_idx].y)
    };

    const double area_tip = TriangleArea2D(pts[lone_idx], q1, q2);
    if (h_lone > 0.0) {
        fill_vol.Add(area_tip * (h_lone / 3.0));
        fill_area.Add(area_tip);
    } else {
        cut_vol.Add(area_tip * (-h_lone / 3.0));
        cut_area.Add(area_tip);
    }

    const double area_quad1 = TriangleArea2D(q1, pts[i1], pts[i2]);
    const double area_quad2 = TriangleArea2D(q1, pts[i2], q2);

    if (h_lone > 0.0) {
        cut_vol.Add(area_quad1 * ((-h_1 - h_2) / 3.0));
        cut_vol.Add(area_quad2 * (-h_2 / 3.0));
        cut_area.Add(area_quad1 + area_quad2);
    } else {
        fill_vol.Add(area_quad1 * ((h_1 + h_2) / 3.0));
        fill_vol.Add(area_quad2 * (h_2 / 3.0));
        fill_area.Add(area_quad1 + area_quad2);
    }
}

bool PointInPoly2D(double x, double y, const std::vector<Vec3d>& poly) noexcept {
    const size_t n = poly.size();
    if (n < 3) return true;
    bool inside = false;
    for (size_t i = 0, j = n - 1; i < n; j = i++) {
        const double xi = poly[i].x, yi = poly[i].y;
        const double xj = poly[j].x, yj = poly[j].y;
        if (((yi > y) != (yj > y)) &&
            (x < (xj - xi) * (y - yi) / ((yj - yi) + 1e-30) + xi)) {
            inside = !inside;
        }
    }
    return inside;
}

}  // namespace

// ==============================================================================
// 高精度三角网布尔叠加三棱柱法完整实现
// ==============================================================================

OperationResult EarthworkCalculator::ComputeExactTinToTinVolume(
    const TiledTinSurface& period1_original,
    const TiledTinSurface& period2_target,
    const EarthworkOptions& options,
    EarthworkReport& out_report) const {
    const auto start_tp = std::chrono::steady_clock::now();
    out_report = EarthworkReport{};

    if (!period1_original.IsValid() || !period2_target.IsValid()) {
        return OperationResult::Fail(
            StatusCode::kEmptyDataset,
            "两期三角网中至少有一期为空，无法执行土方量计算");
    }

    ExactPredicates::Initialize();

    BoundingBox3d overlap_box = period1_original.Bounds();
    const BoundingBox3d& b2 = period2_target.Bounds();

    overlap_box.min_pt.x = std::max(overlap_box.min_pt.x, b2.min_pt.x);
    overlap_box.min_pt.y = std::max(overlap_box.min_pt.y, b2.min_pt.y);
    overlap_box.max_pt.x = std::min(overlap_box.max_pt.x, b2.max_pt.x);
    overlap_box.max_pt.y = std::min(overlap_box.max_pt.y, b2.max_pt.y);

    const bool has_roi = options.roi_polygon.size() >= 3;
    if (has_roi) {
        BoundingBox3d roi_box;
        for (const auto& p : options.roi_polygon) {
            roi_box.Expand(p);
        }
        overlap_box.min_pt.x = std::max(overlap_box.min_pt.x, roi_box.min_pt.x);
        overlap_box.min_pt.y = std::max(overlap_box.min_pt.y, roi_box.min_pt.y);
        overlap_box.max_pt.x = std::min(overlap_box.max_pt.x, roi_box.max_pt.x);
        overlap_box.max_pt.y = std::min(overlap_box.max_pt.y, roi_box.max_pt.y);
    }

    if (overlap_box.min_pt.x >= overlap_box.max_pt.x ||
        overlap_box.min_pt.y >= overlap_box.max_pt.y) {
        return OperationResult::Fail(
            StatusCode::kTopologyDegeneracy,
            "两期三角网与算量边界在平面投影上无重叠区域");
    }

    const auto& verts2 = period2_target.Vertices();
    const auto& tris2 = period2_target.Triangles();

    // 根据目标三角形平均尺寸自适应估算最优哈希网格边长
    double avg_edge_len = 16.0;
    {
        double sum_span = 0.0;
        size_t sample_cnt = 0;
        const size_t step_stride = std::max<size_t>(1, tris2.size() / 256);
        for (size_t i = 0; i < tris2.size() && sample_cnt < 256; i += step_stride) {
            if (!tris2[i].is_active) continue;
            const Vec3d& a = verts2[tris2[i].v[0]].Position();
            const Vec3d& b = verts2[tris2[i].v[1]].Position();
            const Vec3d& c = verts2[tris2[i].v[2]].Position();
            sum_span += std::max({a.x, b.x, c.x}) - std::min({a.x, b.x, c.x});
            ++sample_cnt;
        }
        if (sample_cnt > 0) {
            avg_edge_len = std::clamp(sum_span / static_cast<double>(sample_cnt) * 2.0, 2.0, 256.0);
        }
    }

    const double cell_size = avg_edge_len;
    const double inv_cell = 1.0 / cell_size;

    auto pack_cell = [](int32_t gx, int32_t gy) noexcept -> uint64_t {
        return (static_cast<uint64_t>(static_cast<uint32_t>(gx)) << 32) |
               static_cast<uint64_t>(static_cast<uint32_t>(gy));
    };

    struct Tri2Cache {
        Vec3d a, b, c;
        double min_x, max_x, min_y, max_y;
        uint32_t orig_tid;
    };

    std::vector<Tri2Cache> target_tris_cache;
    target_tris_cache.reserve(period2_target.ActiveTriangleCount());
    std::unordered_map<uint64_t, std::vector<uint32_t>> target_grid;
    target_grid.reserve(period2_target.ActiveTriangleCount());

    for (uint32_t tid2 = 0; tid2 < tris2.size(); ++tid2) {
        const TinTriangle& t2 = tris2[tid2];
        if (!t2.is_active) continue;

        const Vec3d& a = verts2[t2.v[0]].Position();
        const Vec3d& b = verts2[t2.v[1]].Position();
        const Vec3d& c = verts2[t2.v[2]].Position();

        const double min_x = std::min({a.x, b.x, c.x});
        const double max_x = std::max({a.x, b.x, c.x});
        const double min_y = std::min({a.y, b.y, c.y});
        const double max_y = std::max({a.y, b.y, c.y});

        if (max_x < overlap_box.min_pt.x || min_x > overlap_box.max_pt.x ||
            max_y < overlap_box.min_pt.y || min_y > overlap_box.max_pt.y) {
            continue;
        }

        const auto cache_idx = static_cast<uint32_t>(target_tris_cache.size());
        target_tris_cache.push_back({a, b, c, min_x, max_x, min_y, max_y, tid2});

        const auto gx0 = static_cast<int32_t>(std::floor(min_x * inv_cell));
        const auto gx1 = static_cast<int32_t>(std::floor(max_x * inv_cell));
        const auto gy0 = static_cast<int32_t>(std::floor(min_y * inv_cell));
        const auto gy1 = static_cast<int32_t>(std::floor(max_y * inv_cell));

        for (int32_t gx = gx0; gx <= gx1; ++gx) {
            for (int32_t gy = gy0; gy <= gy1; ++gy) {
                target_grid[pack_cell(gx, gy)].push_back(cache_idx);
            }
        }
    }

    if (target_tris_cache.empty()) {
        return OperationResult::Fail(
            StatusCode::kTopologyDegeneracy,
            "重叠区域内不存在有效的设计面三角形");
    }

    const auto& verts1 = period1_original.Vertices();
    const auto& tris1 = period1_original.Triangles();
    std::vector<uint32_t> candidate_orig_tris;
    candidate_orig_tris.reserve(period1_original.ActiveTriangleCount());

    for (uint32_t tid1 = 0; tid1 < tris1.size(); ++tid1) {
        const TinTriangle& t1 = tris1[tid1];
        if (!t1.is_active) continue;
        const Vec3d& a = verts1[t1.v[0]].Position();
        const Vec3d& b = verts1[t1.v[1]].Position();
        const Vec3d& c = verts1[t1.v[2]].Position();

        const double min_x = std::min({a.x, b.x, c.x});
        const double max_x = std::max({a.x, b.x, c.x});
        const double min_y = std::min({a.y, b.y, c.y});
        const double max_y = std::max({a.y, b.y, c.y});

        if (max_x < overlap_box.min_pt.x || min_x > overlap_box.max_pt.x ||
            max_y < overlap_box.min_pt.y || min_y > overlap_box.max_pt.y) {
            continue;
        }
        candidate_orig_tris.push_back(tid1);
    }

    if (candidate_orig_tris.empty()) {
        return OperationResult::Fail(
            StatusCode::kTopologyDegeneracy,
            "重叠区域内不存在有效的原地面三角形");
    }

    size_t num_threads = options.thread_count;
    if (num_threads == 0) {
        const unsigned int hw = std::thread::hardware_concurrency();
        num_threads = hw > 0 ? static_cast<size_t>(hw) : 4;
    }
    if (candidate_orig_tris.size() < 2048) {
        num_threads = 1;
    }

    struct WorkerAccum {
        KahanAccumulator cut_vol;
        KahanAccumulator fill_vol;
        KahanAccumulator cut_area;
        KahanAccumulator fill_area;
        KahanAccumulator zero_area;
        double max_cut = 0.0;
        double max_fill = 0.0;
        uint64_t prism_count = 0;
    };

    std::vector<WorkerAccum> worker_accums(num_threads);
    const size_t total_orig = candidate_orig_tris.size();

    std::vector<std::thread> workers;
    workers.reserve(num_threads);

    for (size_t t = 0; t < num_threads; ++t) {
        const size_t begin = (total_orig * t) / num_threads;
        const size_t end = (total_orig * (t + 1)) / num_threads;

        workers.emplace_back([&, t, begin, end]() {
            WorkerAccum& acc = worker_accums[t];
            std::vector<uint32_t> seen_target_stamp(target_tris_cache.size(), 0);
            uint32_t stamp = 1;

            SmallConvexPoly poly_buf1, poly_buf2;

            for (size_t idx = begin; idx < end; ++idx) {
                const TinTriangle& t1 = tris1[candidate_orig_tris[idx]];
                const Vec3d& a1 = verts1[t1.v[0]].Position();
                const Vec3d& b1 = verts1[t1.v[1]].Position();
                const Vec3d& c1 = verts1[t1.v[2]].Position();

                const double min_x1 = std::min({a1.x, b1.x, c1.x});
                const double max_x1 = std::max({a1.x, b1.x, c1.x});
                const double min_y1 = std::min({a1.y, b1.y, c1.y});
                const double max_y1 = std::max({a1.y, b1.y, c1.y});

                ++stamp;
                if (stamp == 0) {
                    std::fill(seen_target_stamp.begin(), seen_target_stamp.end(), 0);
                    stamp = 1;
                }

                const auto gx0 = static_cast<int32_t>(std::floor(min_x1 * inv_cell));
                const auto gx1 = static_cast<int32_t>(std::floor(max_x1 * inv_cell));
                const auto gy0 = static_cast<int32_t>(std::floor(min_y1 * inv_cell));
                const auto gy1 = static_cast<int32_t>(std::floor(max_y1 * inv_cell));

                for (int32_t gx = gx0; gx <= gx1; ++gx) {
                    for (int32_t gy = gy0; gy <= gy1; ++gy) {
                        auto it_cell = target_grid.find(pack_cell(gx, gy));
                        if (it_cell == target_grid.end()) continue;

                        for (uint32_t c_idx : it_cell->second) {
                            if (seen_target_stamp[c_idx] == stamp) continue;
                            seen_target_stamp[c_idx] = stamp;

                            const Tri2Cache& t2 = target_tris_cache[c_idx];
                            if (max_x1 <= t2.min_x || min_x1 >= t2.max_x ||
                                max_y1 <= t2.min_y || min_y1 >= t2.max_y) {
                                continue;
                            }

                            poly_buf1.Clear();
                            poly_buf1.Push(a1.x, a1.y);
                            poly_buf1.Push(b1.x, b1.y);
                            poly_buf1.Push(c1.x, c1.y);
                            poly_buf1.Finalize();

                            ClipPolyByHalfPlane(poly_buf1, {t2.a.x, t2.a.y}, {t2.b.x, t2.b.y}, poly_buf2);
                            if (poly_buf2.count < 3) continue;

                            ClipPolyByHalfPlane(poly_buf2, {t2.b.x, t2.b.y}, {t2.c.x, t2.c.y}, poly_buf1);
                            if (poly_buf1.count < 3) continue;

                            ClipPolyByHalfPlane(poly_buf1, {t2.c.x, t2.c.y}, {t2.a.x, t2.a.y}, poly_buf2);
                            if (poly_buf2.count < 3) continue;

                            const Point2D& p0 = poly_buf2.pts[0];
                            const double z1_0 = ExactPredicates::InterpolateZOnTriangle(a1, b1, c1, p0.x, p0.y);
                            const double z2_0 = ExactPredicates::InterpolateZOnTriangle(t2.a, t2.b, t2.c, p0.x, p0.y);
                            const double dh0 = z2_0 - z1_0;

                            for (size_t k = 1; k + 1 < poly_buf2.count; ++k) {
                                const Point2D& p1 = poly_buf2.pts[k];
                                const Point2D& p2 = poly_buf2.pts[k + 1];

                                if (has_roi) {
                                    const double cx = (p0.x + p1.x + p2.x) / 3.0;
                                    const double cy = (p0.y + p1.y + p2.y) / 3.0;
                                    if (!PointInPoly2D(cx, cy, options.roi_polygon)) {
                                        continue;
                                    }
                                }

                                const double z1_1 = ExactPredicates::InterpolateZOnTriangle(a1, b1, c1, p1.x, p1.y);
                                const double z2_1 = ExactPredicates::InterpolateZOnTriangle(t2.a, t2.b, t2.c, p1.x, p1.y);
                                const double dh1 = z2_1 - z1_1;

                                const double z1_2 = ExactPredicates::InterpolateZOnTriangle(a1, b1, c1, p2.x, p2.y);
                                const double z2_2 = ExactPredicates::InterpolateZOnTriangle(t2.a, t2.b, t2.c, p2.x, p2.y);
                                const double dh2 = z2_2 - z1_2;

                                IntegrateMicroPrism(
                                    p0, p1, p2,
                                    dh0, dh1, dh2,
                                    acc.cut_vol,
                                    acc.fill_vol,
                                    acc.cut_area,
                                    acc.fill_area,
                                    acc.zero_area,
                                    acc.max_cut,
                                    acc.max_fill);
                                ++acc.prism_count;
                            }
                        }
                    }
                }
            }
        });
    }

    for (auto& w : workers) {
        if (w.joinable()) w.join();
    }

    KahanAccumulator total_cut_vol, total_fill_vol;
    KahanAccumulator total_cut_area, total_fill_area, total_zero_area;

    for (const auto& acc : worker_accums) {
        total_cut_vol.Add(acc.cut_vol.sum);
        total_fill_vol.Add(acc.fill_vol.sum);
        total_cut_area.Add(acc.cut_area.sum);
        total_fill_area.Add(acc.fill_area.sum);
        total_zero_area.Add(acc.zero_area.sum);
        out_report.max_cut_depth_m = std::max(out_report.max_cut_depth_m, acc.max_cut);
        out_report.max_fill_height_m = std::max(out_report.max_fill_height_m, acc.max_fill);
        out_report.intersected_prisms += acc.prism_count;
    }

    out_report.cut_volume_m3 = total_cut_vol.sum;
    out_report.fill_volume_m3 = total_fill_vol.sum;
    out_report.net_volume_m3 = out_report.fill_volume_m3 - out_report.cut_volume_m3;
    out_report.cut_area_m2 = total_cut_area.sum;
    out_report.fill_area_m2 = total_fill_area.sum;
    out_report.zero_area_m2 = total_zero_area.sum;
    out_report.total_overlap_area_m2 =
        out_report.cut_area_m2 + out_report.fill_area_m2 + out_report.zero_area_m2;

    if (out_report.total_overlap_area_m2 < options.min_overlap_area) {
        return OperationResult::Fail(
            StatusCode::kTopologyDegeneracy,
            "两期三角网实际有效重叠面积过小或为零，请检查数据范围");
    }

    const auto end_tp = std::chrono::steady_clock::now();
    out_report.elapsed_seconds = std::chrono::duration<double>(end_tp - start_tp).count();
    return OperationResult::Ok();
}

// ==============================================================================
// 并行网格微分法（实时预览 + 填挖方热力栅格生成）
// ==============================================================================

OperationResult EarthworkCalculator::ComputeFastGridDifferential(
    const TiledTinSurface& period1_original,
    const TiledTinSurface& period2_target,
    const EarthworkOptions& options,
    EarthworkReport& out_report,
    CutFillHeatmapGrid* out_heatmap) const {
    const auto start_tp = std::chrono::steady_clock::now();
    out_report = EarthworkReport{};

    if (!period1_original.IsValid() || !period2_target.IsValid()) {
        return OperationResult::Fail(
            StatusCode::kEmptyDataset,
            "两期三角网中至少有一期为空，无法执行网格微分计算");
    }

    const double step = (options.grid_step > 0.05) ? options.grid_step : 1.0;
    const double cell_area = step * step;

    BoundingBox3d box = period1_original.Bounds();
    const BoundingBox3d& b2 = period2_target.Bounds();
    box.min_pt.x = std::max(box.min_pt.x, b2.min_pt.x);
    box.min_pt.y = std::max(box.min_pt.y, b2.min_pt.y);
    box.max_pt.x = std::min(box.max_pt.x, b2.max_pt.x);
    box.max_pt.y = std::min(box.max_pt.y, b2.max_pt.y);

    const bool has_roi = options.roi_polygon.size() >= 3;
    if (has_roi) {
        BoundingBox3d roi_box;
        for (const auto& p : options.roi_polygon) roi_box.Expand(p);
        box.min_pt.x = std::max(box.min_pt.x, roi_box.min_pt.x);
        box.min_pt.y = std::max(box.min_pt.y, roi_box.min_pt.y);
        box.max_pt.x = std::min(box.max_pt.x, roi_box.max_pt.x);
        box.max_pt.y = std::min(box.max_pt.y, roi_box.max_pt.y);
    }

    if (box.min_pt.x >= box.max_pt.x || box.min_pt.y >= box.max_pt.y) {
        return OperationResult::Fail(
            StatusCode::kTopologyDegeneracy,
            "两期三角网与算量边界无有效重叠区域");
    }

    const auto cols = static_cast<uint32_t>(std::ceil((box.max_pt.x - box.min_pt.x) / step));
    const auto rows = static_cast<uint32_t>(std::ceil((box.max_pt.y - box.min_pt.y) / step));
    if (cols == 0 || rows == 0 || static_cast<uint64_t>(cols) * rows > 250000000ull) {
        return OperationResult::Fail(
            StatusCode::kInvalidArgument,
            "网格采样步长过小导致像元总数超过 2.5 亿上限，请增大 grid_step");
    }

    if (out_heatmap != nullptr) {
        out_heatmap->width = cols;
        out_heatmap->height = rows;
        out_heatmap->origin_x = box.min_pt.x;
        out_heatmap->origin_y = box.min_pt.y;
        out_heatmap->cell_size = step;
        out_heatmap->min_delta_z = 0.0f;
        out_heatmap->max_delta_z = 0.0f;
        out_heatmap->delta_z_matrix.assign(
            static_cast<size_t>(cols) * rows,
            std::numeric_limits<float>::quiet_NaN());
    }

    size_t num_threads = options.thread_count;
    if (num_threads == 0) {
        const unsigned int hw = std::thread::hardware_concurrency();
        num_threads = hw > 0 ? static_cast<size_t>(hw) : 4;
    }
    if (rows < 64) num_threads = 1;

    struct GridWorkerAccum {
        KahanAccumulator cut_vol;
        KahanAccumulator fill_vol;
        uint64_t cut_cells = 0;
        uint64_t fill_cells = 0;
        uint64_t zero_cells = 0;
        double max_cut = 0.0;
        double max_fill = 0.0;
    };

    std::vector<GridWorkerAccum> accums(num_threads);
    std::vector<std::thread> workers;
    workers.reserve(num_threads);

    for (size_t t = 0; t < num_threads; ++t) {
        const uint32_t r_begin = (rows * static_cast<uint32_t>(t)) / static_cast<uint32_t>(num_threads);
        const uint32_t r_end = (rows * static_cast<uint32_t>(t + 1)) / static_cast<uint32_t>(num_threads);

        workers.emplace_back([&, t, r_begin, r_end]() {
            GridWorkerAccum& acc = accums[t];
            for (uint32_t r = r_begin; r < r_end; ++r) {
                const double py = box.min_pt.y + (static_cast<double>(r) + 0.5) * step;
                for (uint32_t c = 0; c < cols; ++c) {
                    const double px = box.min_pt.x + (static_cast<double>(c) + 0.5) * step;
                    if (has_roi && !PointInPoly2D(px, py, options.roi_polygon)) {
                        continue;
                    }

                    double z1 = 0.0, z2 = 0.0;
                    if (!period1_original.QueryElevationAt(px, py, z1, nullptr) ||
                        !period2_target.QueryElevationAt(px, py, z2, nullptr)) {
                        continue;
                    }

                    const double dh = z2 - z1;
                    if (out_heatmap != nullptr) {
                        out_heatmap->delta_z_matrix[static_cast<size_t>(r) * cols + c] =
                            static_cast<float>(dh);
                    }

                    if (dh > 1e-6) {
                        acc.fill_vol.Add(dh * cell_area);
                        ++acc.fill_cells;
                        if (dh > acc.max_fill) acc.max_fill = dh;
                    } else if (dh < -1e-6) {
                        acc.cut_vol.Add((-dh) * cell_area);
                        ++acc.cut_cells;
                        if (-dh > acc.max_cut) acc.max_cut = -dh;
                    } else {
                        ++acc.zero_cells;
                    }
                }
            }
        });
    }

    for (auto& w : workers) {
        if (w.joinable()) w.join();
    }

    KahanAccumulator total_cut_vol, total_fill_vol;
    uint64_t total_cut_cells = 0, total_fill_cells = 0, total_zero_cells = 0;

    for (const auto& a : accums) {
        total_cut_vol.Add(a.cut_vol.sum);
        total_fill_vol.Add(a.fill_vol.sum);
        total_cut_cells += a.cut_cells;
        total_fill_cells += a.fill_cells;
        total_zero_cells += a.zero_cells;
        out_report.max_cut_depth_m = std::max(out_report.max_cut_depth_m, a.max_cut);
        out_report.max_fill_height_m = std::max(out_report.max_fill_height_m, a.max_fill);
    }

    out_report.cut_volume_m3 = total_cut_vol.sum;
    out_report.fill_volume_m3 = total_fill_vol.sum;
    out_report.net_volume_m3 = out_report.fill_volume_m3 - out_report.cut_volume_m3;
    out_report.cut_area_m2 = static_cast<double>(total_cut_cells) * cell_area;
    out_report.fill_area_m2 = static_cast<double>(total_fill_cells) * cell_area;
    out_report.zero_area_m2 = static_cast<double>(total_zero_cells) * cell_area;
    out_report.total_overlap_area_m2 =
        out_report.cut_area_m2 + out_report.fill_area_m2 + out_report.zero_area_m2;
    out_report.intersected_prisms = total_cut_cells + total_fill_cells + total_zero_cells;

    if (out_heatmap != nullptr) {
        out_heatmap->min_delta_z = static_cast<float>(-out_report.max_cut_depth_m);
        out_heatmap->max_delta_z = static_cast<float>(out_report.max_fill_height_m);
    }

    const auto end_tp = std::chrono::steady_clock::now();
    out_report.elapsed_seconds = std::chrono::duration<double>(end_tp - start_tp).count();
    return OperationResult::Ok();
}

}  // namespace geo_kernel

