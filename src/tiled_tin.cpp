#include "geo_kernel/tiled_tin.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <limits>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include "geo_kernel/exact_predicates.hpp"

namespace geo_kernel {
namespace {
constexpr double kDuplicateXyEpsSq = 1e-14;  // 平面坐标重复判定平方阈值（0.1 微米）

/**
 * @brief Bowyer-Watson 构网阶段内部三角单元结构
 */
struct BwTriangle {
    uint32_t v[3]{UINT32_MAX, UINT32_MAX, UINT32_MAX};
    uint32_t adj[3]{UINT32_MAX, UINT32_MAX, UINT32_MAX};
    bool deleted = false;
    int IndexOfNeighbor(uint32_t tid) const noexcept {
        if (adj[0] == tid) return 0;
        if (adj[1] == tid) return 1;
        if (adj[2] == tid) return 2;
        return -1;
    }
};

/**
 * @brief 空腔边界有向边描述符（用于重新缝合新生成的三角形）
 */
struct CavityEdge {
    uint32_t v0 = UINT32_MAX;           ///< 逆时针空腔边界起点
    uint32_t v1 = UINT32_MAX;           ///< 逆时针空腔边界终点
    uint32_t outer_tri = UINT32_MAX;    ///< 空腔外侧保留的邻接三角形 ID
    int outer_edge_slot = -1;           ///< 邻接边在 outer_tri 中的槽位 [0..2]
};

/**
 * @brief Möller–Trumbore 三维射线与空间三角形精确相交算法
 */
bool RayIntersectsTriangle3D(
    const Vec3d& orig,
    const Vec3d& dir,
    const Vec3d& v0,
    const Vec3d& v1,
    const Vec3d& v2,
    double& out_t) noexcept {
    constexpr double kEps = 1e-12;
    const Vec3d e1 = v1 - v0;
    const Vec3d e2 = v2 - v0;
    const Vec3d h = dir.Cross(e2);
    const double a = e1.Dot(h);
    if (a > -kEps && a < kEps) {
        return false; // 射线与三角形平面平行
    }
    const double f = 1.0 / a;
    const Vec3d s = orig - v0;
    const double u = f * s.Dot(h);
    if (u < -1e-9 || u > 1.0 + 1e-9) {
        return false;
    }
    const Vec3d q = s.Cross(e1);
    const double v = f * dir.Dot(q);
    if (v < -1e-9 || u + v > 1.0 + 1e-9) {
        return false;
    }
    const double t = f * e2.Dot(q);
    if (t >= 0.0) {
        out_t = t;
        return true;
    }
    return false;
}

/**
 * @brief 射线与三维轴对齐包围盒 (AABB) 相交测试（Slab Method）
 */
bool RayIntersectsAABB(
    const Vec3d& orig,
    const Vec3d& inv_dir,
    const BoundingBox3d& box) noexcept {
    if (!box.IsValid()) return false;
    const double tx1 = (box.min_pt.x - orig.x) * inv_dir.x;
    const double tx2 = (box.max_pt.x - orig.x) * inv_dir.x;
    double tmin = std::min(tx1, tx2);
    double tmax = std::max(tx1, tx2);
    const double ty1 = (box.min_pt.y - orig.y) * inv_dir.y;
    const double ty2 = (box.max_pt.y - orig.y) * inv_dir.y;
    tmin = std::max(tmin, std::min(ty1, ty2));
    tmax = std::min(tmax, std::max(ty1, ty2));
    const double tz1 = (box.min_pt.z - orig.z) * inv_dir.z;
    const double tz2 = (box.max_pt.z - orig.z) * inv_dir.z;
    tmin = std::max(tmin, std::min(tz1, tz2));
    tmax = std::min(tmax, std::max(tz1, tz2));
    return tmax >= std::max(0.0, tmin);
}

/**
 * @brief 计算二维点 P 到二维线段 AB 的最短平方距离
 */
double PointToSegmentDistanceSq2D(
    double px, double py,
    double ax, double ay,
    double bx, double by) noexcept {
    const double vx = bx - ax;
    const double vy = by - ay;
    const double wx = px - ax;
    const double wy = py - ay;
    const double len_sq = vx * vx + vy * vy;
    if (len_sq <= 1e-18) {
        return wx * wx + wy * wy;
    }
    const double t = std::clamp((wx * vx + wy * vy) / len_sq, 0.0, 1.0);
    const double proj_x = ax + t * vx;
    const double proj_y = ay + t * vy;
    const double dx = px - proj_x;
    const double dy = py - proj_y;
    return dx * dx + dy * dy;
}
}  // namespace

void TiledTinSurface::Clear() noexcept {
    if (!triangles_.empty()) {
        GEO_LOG_DEBUG("TIN", "清空三角网释放 %zu 顶点, %zu 三角形", vertices_.size(), triangles_.size());
    }
    vertices_.clear();
    vertices_.shrink_to_fit();
    triangles_.clear();
    triangles_.shrink_to_fit();
    tiles_.clear();
    tiles_.shrink_to_fit();
    tile_uid_to_index_.clear();
    tile_overlapping_triangles_.clear();
    active_triangle_count_ = 0;
    mesh_bounds_ = BoundingBox3d{};
}

void TiledTinSurface::PrepareVertices(
    const std::vector<SurveyPoint>& points,
    const TinBuildOptions& options,
    TinBuildStats& stats) {
    vertices_.clear();
    stats.input_points = static_cast<uint64_t>(points.size());

    // 1. 若启用了自适应网格抽稀，按平面网格保留极值高程点（最高/最低）和全部特征点
    std::vector<const SurveyPoint*> candidate_pts;
    if (options.decimation_grid_size > 1e-6) {
        const double inv_cell = 1.0 / options.decimation_grid_size;
        struct CellBucket {
            const SurveyPoint* min_z_pt = nullptr;
            const SurveyPoint* max_z_pt = nullptr;
        };
        std::unordered_map<uint64_t, CellBucket> grid_buckets;
        grid_buckets.reserve(points.size() / 4 + 16);
        candidate_pts.reserve(points.size() / 2 + 16);

        for (const auto& pt : points) {
            if (!pt.IsValid()) continue;
            if (pt.IsFeature()) {
                candidate_pts.push_back(&pt);
                continue;
            }
            const auto gx = static_cast<int32_t>(std::floor(pt.x * inv_cell));
            const auto gy = static_cast<int32_t>(std::floor(pt.y * inv_cell));
            const uint64_t cell_key = PackTileUid(gx, gy);
            auto& bucket = grid_buckets[cell_key];
            if (bucket.min_z_pt == nullptr || pt.z < bucket.min_z_pt->z) {
                bucket.min_z_pt = &pt;
            }
            if (bucket.max_z_pt == nullptr || pt.z > bucket.max_z_pt->z) {
                bucket.max_z_pt = &pt;
            }
        }
        for (const auto& kv : grid_buckets) {
            const CellBucket& b = kv.second;
            if (b.min_z_pt != nullptr) {
                candidate_pts.push_back(b.min_z_pt);
            }
            if (b.max_z_pt != nullptr && b.max_z_pt != b.min_z_pt) {
                candidate_pts.push_back(b.max_z_pt);
            }
        }
        stats.decimated_points = stats.input_points - static_cast<uint64_t>(candidate_pts.size());
    } else {
        candidate_pts.reserve(points.size());
        for (const auto& pt : points) {
            if (pt.IsValid()) {
                candidate_pts.push_back(&pt);
            }
        }
    }
    if (candidate_pts.empty()) {
        return;
    }

    // 2. 计算候选点整体二维包围盒
    BoundingBox3d raw_bounds;
    for (const SurveyPoint* p : candidate_pts) {
        raw_bounds.Expand(p->Position());
    }
    const double span_x = std::max(raw_bounds.max_pt.x - raw_bounds.min_pt.x, 1.0);
    const double span_y = std::max(raw_bounds.max_pt.y - raw_bounds.min_pt.y, 1.0);
    const double inv_span_x = 1.0 / span_x;
    const double inv_span_y = 1.0 / span_y;

    // 3. 为每个点计算二维莫顿码，用于空间局部性排序与相邻重复点过滤
    struct SortItem {
        uint64_t morton2d = 0;
        const SurveyPoint* ptr = nullptr;
    };
    std::vector<SortItem> sorted_items(candidate_pts.size());
    for (size_t i = 0; i < candidate_pts.size(); ++i) {
        const SurveyPoint* p = candidate_pts[i];
        const double nx = (p->x - raw_bounds.min_pt.x) * inv_span_x;
        const double ny = (p->y - raw_bounds.min_pt.y) * inv_span_y;
        sorted_items[i].morton2d = Morton3D::EncodeNormalized(nx, ny, 0.0);
        sorted_items[i].ptr = p;
    }
    std::sort(sorted_items.begin(), sorted_items.end(), [](const SortItem& a, const SortItem& b) noexcept {
        if (a.morton2d != b.morton2d) {
            return a.morton2d < b.morton2d;
        }
        if (a.ptr->x != b.ptr->x) return a.ptr->x < b.ptr->x;
        return a.ptr->y < b.ptr->y;
    });

    // 4. 严格过滤平面坐标完全重合的冗余点（避免构网出现零长边退化）
    vertices_.reserve(sorted_items.size());
    uint64_t dup_count = 0;
    for (size_t i = 0; i < sorted_items.size(); ++i) {
        const SurveyPoint* p = sorted_items[i].ptr;
        if (!vertices_.empty()) {
            const TinVertex& prev = vertices_.back();
            const double dx = p->x - prev.x;
            const double dy = p->y - prev.y;
            if (dx * dx + dy * dy <= kDuplicateXyEpsSq) {
                ++dup_count;
                if (p->IsFeature() && (prev.flags & SurveyPoint::kFlagFeature) == 0) {
                    vertices_.back().z = p->z;
                    vertices_.back().source_point_id = p->point_id;
                    vertices_.back().code_id = p->code_id;
                    vertices_.back().flags = p->flags | TinVertex::kFlagActive;
                }
                continue;
            }
        }
        TinVertex v{};
        v.x = p->x;
        v.y = p->y;
        v.z = p->z;
        v.source_point_id = p->point_id;
        v.code_id = p->code_id;
        v.flags = p->flags | TinVertex::kFlagActive;
        vertices_.push_back(v);
        mesh_bounds_.Expand(v.Position());
    }
    stats.deduplicated_points = dup_count;
    stats.tin_vertices = static_cast<uint64_t>(vertices_.size());
    stats.bounds = mesh_bounds_;

    GEO_LOG_DEBUG(
        "TIN",
        "构网顶点预处理完成: input=%llu, decimated=%llu, deduplicated=%llu, final_verts=%llu",
        static_cast<unsigned long long>(stats.input_points),
        static_cast<unsigned long long>(stats.decimated_points),
        static_cast<unsigned long long>(stats.deduplicated_points),
        static_cast<unsigned long long>(stats.tin_vertices));
}

OperationResult TiledTinSurface::RunBowyerWatsonDelaunay(const TinBuildOptions& options) {
    ExactPredicates::Initialize();
    const uint32_t n_pts = static_cast<uint32_t>(vertices_.size());
    if (n_pts < 3) {
        return OperationResult::Fail(
            StatusCode::kTopologyDegeneracy,
            "有效不重合顶点数量少于 3 个，无法构建三角网");
    }

    const Vec3d center = mesh_bounds_.Center();
    const Vec3d extent = mesh_bounds_.Extent();
    const double delta = std::max({extent.x, extent.y, 10.0}) * 20.0;
    const uint32_t sv0 = n_pts;
    const uint32_t sv1 = n_pts + 1;
    const uint32_t sv2 = n_pts + 2;

    std::vector<Vec3d> pts_2d;
    pts_2d.reserve(static_cast<size_t>(n_pts) + 3);
    for (uint32_t i = 0; i < n_pts; ++i) {
        pts_2d.push_back(vertices_[i].Position());
    }
    pts_2d.emplace_back(center.x - 2.0 * delta, center.y - delta, center.z);
    pts_2d.emplace_back(center.x + 2.0 * delta, center.y - delta, center.z);
    pts_2d.emplace_back(center.x, center.y + 2.0 * delta, center.z);

    std::vector<BwTriangle> bw_tris;
    bw_tris.reserve(static_cast<size_t>(n_pts) * 2 + 16);
    BwTriangle root_tri{};
    root_tri.v[0] = sv0;
    root_tri.v[1] = sv1;
    root_tri.v[2] = sv2;
    bw_tris.push_back(root_tri);

    uint32_t last_created_tri = 0;
    std::vector<uint32_t> cavity_tris;
    std::vector<CavityEdge> boundary_edges;
    std::vector<uint32_t> search_stack;
    std::vector<uint32_t> visited_stamp(bw_tris.capacity(), 0);
    uint32_t current_stamp = 1;
    cavity_tris.reserve(64);
    boundary_edges.reserve(64);
    search_stack.reserve(64);

#if GEO_DEBUG_ENABLED
    uint64_t walk_fallback_count = 0;
    uint64_t unlocated_skip_count = 0;
#endif

    for (uint32_t pid = 0; pid < n_pts; ++pid) {
        if ((pid & 0xFFFFu) == 0 && options.progress_callback) {
            const double progress = 0.1 + 0.8 * (static_cast<double>(pid) / static_cast<double>(n_pts));
            if (!options.progress_callback(progress, pid)) {
                return OperationResult::Fail(StatusCode::kCancelled, "用户已取消三角网构建任务");
            }
        }
        const Vec3d& p = pts_2d[pid];
        uint32_t cur_tid = last_created_tri;
        if (cur_tid >= bw_tris.size() || bw_tris[cur_tid].deleted) {
            cur_tid = 0;
            while (cur_tid < bw_tris.size() && bw_tris[cur_tid].deleted) {
                ++cur_tid;
            }
        }

        uint32_t guard_steps = 0;
        const uint32_t max_walk_steps = static_cast<uint32_t>(bw_tris.size()) + 16;
        bool located = false;
        while (guard_steps++ < max_walk_steps) {
            const BwTriangle& t = bw_tris[cur_tid];
            const double o0 = ExactPredicates::Orient2D(pts_2d[t.v[1]], pts_2d[t.v[2]], p);
            if (o0 < 0.0 && t.adj[0] != UINT32_MAX) {
                cur_tid = t.adj[0];
                continue;
            }
            const double o1 = ExactPredicates::Orient2D(pts_2d[t.v[2]], pts_2d[t.v[0]], p);
            if (o1 < 0.0 && t.adj[1] != UINT32_MAX) {
                cur_tid = t.adj[1];
                continue;
            }
            const double o2 = ExactPredicates::Orient2D(pts_2d[t.v[0]], pts_2d[t.v[1]], p);
            if (o2 < 0.0 && t.adj[2] != UINT32_MAX) {
                cur_tid = t.adj[2];
                continue;
            }
            located = true;
            break;
        }
        if (!located) {
#if GEO_DEBUG_ENABLED
            ++walk_fallback_count;
#endif
            for (uint32_t tid = 0; tid < bw_tris.size(); ++tid) {
                if (bw_tris[tid].deleted) continue;
                const BwTriangle& t = bw_tris[tid];
                if (ExactPredicates::PointInTriangle2D(pts_2d[t.v[0]], pts_2d[t.v[1]], pts_2d[t.v[2]], p) != 0) {
                    cur_tid = tid;
                    located = true;
                    break;
                }
            }
        }
        if (!located) {
#if GEO_DEBUG_ENABLED
            ++unlocated_skip_count;
#endif
            continue;
        }

        ++current_stamp;
        if (current_stamp == 0) {
            std::fill(visited_stamp.begin(), visited_stamp.end(), 0);
            current_stamp = 1;
        }
        if (visited_stamp.size() < bw_tris.size() + 64) {
            visited_stamp.resize(bw_tris.size() * 2 + 64, 0);
        }

        cavity_tris.clear();
        search_stack.clear();
        visited_stamp[cur_tid] = current_stamp;
        search_stack.push_back(cur_tid);
        while (!search_stack.empty()) {
            const uint32_t tid = search_stack.back();
            search_stack.pop_back();
            cavity_tris.push_back(tid);
            const BwTriangle& t = bw_tris[tid];
            for (int e = 0; e < 3; ++e) {
                const uint32_t nb = t.adj[e];
                if (nb == UINT32_MAX || bw_tris[nb].deleted || visited_stamp[nb] == current_stamp) {
                    continue;
                }
                const BwTriangle& nb_tri = bw_tris[nb];
                const double inc = ExactPredicates::InCircle(
                    pts_2d[nb_tri.v[0]], pts_2d[nb_tri.v[1]], pts_2d[nb_tri.v[2]], p);
                if (inc > 0.0) {
                    visited_stamp[nb] = current_stamp;
                    search_stack.push_back(nb);
                }
            }
        }

        boundary_edges.clear();
        for (uint32_t tid : cavity_tris) {
            const BwTriangle& t = bw_tris[tid];
            for (int e = 0; e < 3; ++e) {
                const uint32_t nb = t.adj[e];
                if (nb == UINT32_MAX || visited_stamp[nb] != current_stamp) {
                    const uint32_t ev0 = t.v[(e + 1) % 3];
                    const uint32_t ev1 = t.v[(e + 2) % 3];
                    int outer_slot = -1;
                    if (nb != UINT32_MAX) {
                        outer_slot = bw_tris[nb].IndexOfNeighbor(tid);
                    }
                    boundary_edges.push_back({ev0, ev1, nb, outer_slot});
                }
            }
        }
        for (uint32_t tid : cavity_tris) {
            bw_tris[tid].deleted = true;
        }

        const size_t num_edges = boundary_edges.size();
        if (num_edges < 3) {
            continue;
        }
        std::vector<uint32_t> new_tri_ids(num_edges);
        for (size_t i = 0; i < num_edges; ++i) {
            if (i < cavity_tris.size()) {
                new_tri_ids[i] = cavity_tris[i];
            } else {
                new_tri_ids[i] = static_cast<uint32_t>(bw_tris.size());
                bw_tris.emplace_back();
            }
        }
        std::unordered_map<uint32_t, uint32_t> edge_start_to_new_tri;
        edge_start_to_new_tri.reserve(num_edges);
        for (size_t i = 0; i < num_edges; ++i) {
            edge_start_to_new_tri[boundary_edges[i].v0] = new_tri_ids[i];
        }
        for (size_t i = 0; i < num_edges; ++i) {
            const CavityEdge& edge = boundary_edges[i];
            const uint32_t tid = new_tri_ids[i];
            if (ExactPredicates::Orient2D(pts_2d[pid], pts_2d[edge.v0], pts_2d[edge.v1]) <= 0.0) {
                bw_tris[tid].deleted = true;
                continue;
            }
            BwTriangle& nt = bw_tris[tid];
            nt.deleted = false;
            nt.v[0] = pid;
            nt.v[1] = edge.v0;
            nt.v[2] = edge.v1;
            nt.adj[0] = edge.outer_tri;
            if (edge.outer_tri != UINT32_MAX && edge.outer_edge_slot >= 0) {
                bw_tris[edge.outer_tri].adj[edge.outer_edge_slot] = tid;
            }
            auto it_next = edge_start_to_new_tri.find(edge.v1);
            nt.adj[1] = (it_next != edge_start_to_new_tri.end()) ? it_next->second : UINT32_MAX;
            nt.adj[2] = UINT32_MAX;
            last_created_tri = tid;
        }
        for (size_t i = 0; i < num_edges; ++i) {
            const uint32_t tid = new_tri_ids[i];
            if (bw_tris[tid].deleted) continue;
            const uint32_t next_tid = bw_tris[tid].adj[1];
            if (next_tid != UINT32_MAX && !bw_tris[next_tid].deleted) {
                bw_tris[next_tid].adj[2] = tid;
            }
        }
    }

#if GEO_DEBUG_ENABLED
    if (walk_fallback_count > 0 || unlocated_skip_count > 0) {
        GEO_LOG_WARN(
            "TIN",
            "Bowyer-Watson 构网诊断: walk_fallback=%llu, unlocated_skipped=%llu",
            static_cast<unsigned long long>(walk_fallback_count),
            static_cast<unsigned long long>(unlocated_skip_count));
    }
#endif

    const double max_edge_sq = (options.max_edge_length > 0.0)
        ? (options.max_edge_length * options.max_edge_length)
        : std::numeric_limits<double>::max();
    std::vector<uint32_t> old_to_new_tid(bw_tris.size(), UINT32_MAX);
    triangles_.clear();
    triangles_.reserve(bw_tris.size());

    for (uint32_t tid = 0; tid < bw_tris.size(); ++tid) {
        const BwTriangle& t = bw_tris[tid];
        if (t.deleted) continue;
        if (t.v[0] >= n_pts || t.v[1] >= n_pts || t.v[2] >= n_pts) {
            continue;
        }
        if (options.max_edge_length > 0.0) {
            const double d01 = pts_2d[t.v[0]].DistanceSquared2D(pts_2d[t.v[1]]);
            const double d12 = pts_2d[t.v[1]].DistanceSquared2D(pts_2d[t.v[2]]);
            const double d20 = pts_2d[t.v[2]].DistanceSquared2D(pts_2d[t.v[0]]);
            if (d01 > max_edge_sq || d12 > max_edge_sq || d20 > max_edge_sq) {
                continue;
            }
        }
        const uint32_t final_tid = static_cast<uint32_t>(triangles_.size());
        old_to_new_tid[tid] = final_tid;
        TinTriangle out_tri{};
        out_tri.v[0] = t.v[0];
        out_tri.v[1] = t.v[1];
        out_tri.v[2] = t.v[2];
        out_tri.adj[0] = t.adj[0];
        out_tri.adj[1] = t.adj[1];
        out_tri.adj[2] = t.adj[2];
        out_tri.region_tag = 0;
        out_tri.is_active = 1;
        out_tri.constrained_mask = 0;
        triangles_.push_back(out_tri);
    }

    for (auto& tri : triangles_) {
        for (int e = 0; e < 3; ++e) {
            const uint32_t old_nb = tri.adj[e];
            tri.adj[e] = (old_nb != UINT32_MAX && old_nb < old_to_new_tid.size())
                ? old_to_new_tid[old_nb]
                : UINT32_MAX;
        }
    }

    active_triangle_count_ = static_cast<uint64_t>(triangles_.size());
    if (active_triangle_count_ == 0) {
        return OperationResult::Fail(
            StatusCode::kTopologyDegeneracy,
            "所有输入点共线或最大边长过滤阈值过小，未能生成任何有效三角形");
    }
    return OperationResult::Ok();
}

void TiledTinSurface::RecomputeAllTriangleAdjacency() {
    struct HalfEdgeRef {
        uint32_t tri_id = UINT32_MAX;
        int edge_slot = -1;
    };
    auto pack_edge = [](uint32_t u, uint32_t w) noexcept -> uint64_t {
        return (static_cast<uint64_t>(u) << 32) | static_cast<uint64_t>(w);
    };
    std::unordered_map<uint64_t, HalfEdgeRef> half_edge_map;
    half_edge_map.reserve(triangles_.size() * 3);

    for (uint32_t tid = 0; tid < triangles_.size(); ++tid) {
        TinTriangle& tri = triangles_[tid];
        tri.adj[0] = UINT32_MAX;
        tri.adj[1] = UINT32_MAX;
        tri.adj[2] = UINT32_MAX;
        if (!tri.is_active) continue;
        if (ExactPredicates::Orient2D(
                vertices_[tri.v[0]].Position(),
                vertices_[tri.v[1]].Position(),
                vertices_[tri.v[2]].Position()) < 0.0) {
            std::swap(tri.v[1], tri.v[2]);
        }
        for (int e = 0; e < 3; ++e) {
            const uint32_t ev0 = tri.v[(e + 1) % 3];
            const uint32_t ev1 = tri.v[(e + 2) % 3];
            half_edge_map[pack_edge(ev0, ev1)] = {tid, e};
        }
    }
    for (uint32_t tid = 0; tid < triangles_.size(); ++tid) {
        TinTriangle& tri = triangles_[tid];
        if (!tri.is_active) continue;
        for (int e = 0; e < 3; ++e) {
            if (tri.adj[e] != UINT32_MAX) continue;
            const uint32_t ev0 = tri.v[(e + 1) % 3];
            const uint32_t ev1 = tri.v[(e + 2) % 3];
            auto it_twin = half_edge_map.find(pack_edge(ev1, ev0));
            if (it_twin != half_edge_map.end()) {
                const uint32_t nb_tid = it_twin->second.tri_id;
                const int nb_slot = it_twin->second.edge_slot;
                tri.adj[e] = nb_tid;
                triangles_[nb_tid].adj[nb_slot] = tid;
            }
        }
    }
}

void TiledTinSurface::BuildSpatialTilesAndLookupGrid(double tile_size) {
    tiles_.clear();
    tile_uid_to_index_.clear();
    tile_overlapping_triangles_.clear();
    active_triangle_count_ = 0;
    mesh_bounds_ = BoundingBox3d{};
    lookup_tile_size_ = (tile_size > 1.0) ? tile_size : 128.0;
    const double inv_tile = 1.0 / lookup_tile_size_;

    for (uint32_t tid = 0; tid < triangles_.size(); ++tid) {
        TinTriangle& tri = triangles_[tid];
        if (!tri.is_active) continue;
        ++active_triangle_count_;
        const Vec3d& p0 = vertices_[tri.v[0]].Position();
        const Vec3d& p1 = vertices_[tri.v[1]].Position();
        const Vec3d& p2 = vertices_[tri.v[2]].Position();
        mesh_bounds_.Expand(p0);
        mesh_bounds_.Expand(p1);
        mesh_bounds_.Expand(p2);

        const double cx = (p0.x + p1.x + p2.x) / 3.0;
        const double cy = (p0.y + p1.y + p2.y) / 3.0;
        const auto owner_tx = static_cast<int32_t>(std::floor(cx * inv_tile));
        const auto owner_ty = static_cast<int32_t>(std::floor(cy * inv_tile));
        const uint64_t owner_uid = PackTileUid(owner_tx, owner_ty);
        uint32_t tile_idx = 0;
        auto it = tile_uid_to_index_.find(owner_uid);
        if (it == tile_uid_to_index_.end()) {
            tile_idx = static_cast<uint32_t>(tiles_.size());
            tile_uid_to_index_.emplace(owner_uid, tile_idx);
            TinSpatialTile new_tile{};
            new_tile.tile_index = tile_idx;
            new_tile.tile_x = owner_tx;
            new_tile.tile_y = owner_ty;
            new_tile.tile_uid = owner_uid;
            new_tile.revision = 1;
            new_tile.is_dirty = true;
            tiles_.push_back(std::move(new_tile));
        } else {
            tile_idx = it->second;
        }
        tri.tile_index = tile_idx;
        TinSpatialTile& tile = tiles_[tile_idx];
        tile.triangle_indices.push_back(tid);
        tile.bounds.Expand(p0);
        tile.bounds.Expand(p1);
        tile.bounds.Expand(p2);

        const double min_x = std::min({p0.x, p1.x, p2.x});
        const double max_x = std::max({p0.x, p1.x, p2.x});
        const double min_y = std::min({p0.y, p1.y, p2.y});
        const double max_y = std::max({p0.y, p1.y, p2.y});
        const auto gx0 = static_cast<int32_t>(std::floor(min_x * inv_tile));
        const auto gx1 = static_cast<int32_t>(std::floor(max_x * inv_tile));
        const auto gy0 = static_cast<int32_t>(std::floor(min_y * inv_tile));
        const auto gy1 = static_cast<int32_t>(std::floor(max_y * inv_tile));
        for (int32_t gx = gx0; gx <= gx1; ++gx) {
            for (int32_t gy = gy0; gy <= gy1; ++gy) {
                tile_overlapping_triangles_[PackTileUid(gx, gy)].push_back(tid);
            }
        }
    }
}

void TiledTinSurface::RebuildSpatialTilesAfterEdit(const BoundingBox3d& dirty_box) {
    for (auto& tile : tiles_) {
        tile.triangle_indices.clear();
        tile.bounds = BoundingBox3d{};
    }
    tile_overlapping_triangles_.clear();
    active_triangle_count_ = 0;
    mesh_bounds_ = BoundingBox3d{};
    const double inv_tile = 1.0 / lookup_tile_size_;

    for (uint32_t tid = 0; tid < triangles_.size(); ++tid) {
        TinTriangle& tri = triangles_[tid];
        if (!tri.is_active) continue;
        ++active_triangle_count_;
        const Vec3d& p0 = vertices_[tri.v[0]].Position();
        const Vec3d& p1 = vertices_[tri.v[1]].Position();
        const Vec3d& p2 = vertices_[tri.v[2]].Position();
        mesh_bounds_.Expand(p0);
        mesh_bounds_.Expand(p1);
        mesh_bounds_.Expand(p2);

        const double cx = (p0.x + p1.x + p2.x) / 3.0;
        const double cy = (p0.y + p1.y + p2.y) / 3.0;
        const auto owner_tx = static_cast<int32_t>(std::floor(cx * inv_tile));
        const auto owner_ty = static_cast<int32_t>(std::floor(cy * inv_tile));
        const uint64_t owner_uid = PackTileUid(owner_tx, owner_ty);
        uint32_t tile_idx = 0;
        auto it = tile_uid_to_index_.find(owner_uid);
        if (it == tile_uid_to_index_.end()) {
            tile_idx = static_cast<uint32_t>(tiles_.size());
            tile_uid_to_index_.emplace(owner_uid, tile_idx);
            TinSpatialTile new_tile{};
            new_tile.tile_index = tile_idx;
            new_tile.tile_x = owner_tx;
            new_tile.tile_y = owner_ty;
            new_tile.tile_uid = owner_uid;
            new_tile.revision = 1;
            new_tile.is_dirty = true;
            tiles_.push_back(std::move(new_tile));
        } else {
            tile_idx = it->second;
        }
        tri.tile_index = tile_idx;
        TinSpatialTile& tile = tiles_[tile_idx];
        tile.triangle_indices.push_back(tid);
        tile.bounds.Expand(p0);
        tile.bounds.Expand(p1);
        tile.bounds.Expand(p2);

        const double min_x = std::min({p0.x, p1.x, p2.x});
        const double max_x = std::max({p0.x, p1.x, p2.x});
        const double min_y = std::min({p0.y, p1.y, p2.y});
        const double max_y = std::max({p0.y, p1.y, p2.y});
        const auto gx0 = static_cast<int32_t>(std::floor(min_x * inv_tile));
        const auto gx1 = static_cast<int32_t>(std::floor(max_x * inv_tile));
        const auto gy0 = static_cast<int32_t>(std::floor(min_y * inv_tile));
        const auto gy1 = static_cast<int32_t>(std::floor(max_y * inv_tile));
        for (int32_t gx = gx0; gx <= gx1; ++gx) {
            for (int32_t gy = gy0; gy <= gy1; ++gy) {
                tile_overlapping_triangles_[PackTileUid(gx, gy)].push_back(tid);
            }
        }
    }

    if (dirty_box.IsValid()) {
        MarkTilesDirtyInBox2D(
            dirty_box.min_pt.x, dirty_box.min_pt.y,
            dirty_box.max_pt.x, dirty_box.max_pt.y);
    } else {
        for (auto& tile : tiles_) {
            tile.is_dirty = true;
            ++tile.revision;
        }
    }
    GEO_LOG_DEBUG(
        "TIN",
        "编辑后局部瓦片索引重建完毕: active_tris=%llu, total_tiles=%zu",
        static_cast<unsigned long long>(active_triangle_count_),
        tiles_.size());
}

OperationResult TiledTinSurface::BuildFromPoints(
    const std::vector<SurveyPoint>& points,
    const TinBuildOptions& options,
    TinBuildStats& out_stats) {
    const auto start_tp = std::chrono::steady_clock::now();
    Clear();
    out_stats = TinBuildStats{};
    if (points.empty()) {
        return OperationResult::Fail(StatusCode::kEmptyDataset, "输入点云为空，无法生成三角网");
    }

    GEO_LOG_INFO(
        "TIN",
        "开始从点云构建分块 Delaunay 三角网: points=%zu, decimate=%.3fm, tile=%.1fm, max_edge=%.2fm",
        points.size(),
        options.decimation_grid_size,
        options.tile_size,
        options.max_edge_length);

    try {
        if (options.progress_callback) {
            if (!options.progress_callback(0.02, 0)) {
                return OperationResult::Fail(StatusCode::kCancelled, "用户已取消三角网构建任务");
            }
        }
        PrepareVertices(points, options, out_stats);
        if (vertices_.size() < 3) {
            Clear();
            return OperationResult::Fail(
                StatusCode::kTopologyDegeneracy,
                "去重后有效顶点不足 3 个，无法构建三角网");
        }
        OperationResult tin_res = RunBowyerWatsonDelaunay(options);
        if (!tin_res.IsOk()) {
            Clear();
            return tin_res;
        }
        BuildSpatialTilesAndLookupGrid(options.tile_size);
        if (options.progress_callback) {
            options.progress_callback(1.0, out_stats.tin_vertices);
        }
        const auto end_tp = std::chrono::steady_clock::now();
        out_stats.tin_triangles = active_triangle_count_;
        out_stats.total_tiles = static_cast<uint32_t>(tiles_.size());
        out_stats.elapsed_seconds = std::chrono::duration<double>(end_tp - start_tp).count();

        GEO_LOG_INFO(
            "TIN",
            "分块 Delaunay 三角网构建成功: verts=%llu, tris=%llu, tiles=%u, elapsed=%.3fs",
            static_cast<unsigned long long>(out_stats.tin_vertices),
            static_cast<unsigned long long>(out_stats.tin_triangles),
            out_stats.total_tiles,
            out_stats.elapsed_seconds);
        return OperationResult::Ok();
    } catch (const std::bad_alloc&) {
        Clear();
        return OperationResult::Fail(StatusCode::kOutOfMemory, "构建三角网时内存不足");
    } catch (const std::exception& ex) {
        Clear();
        return OperationResult::Fail(
            StatusCode::kInternalError,
            std::string("构建三角网时捕获异常: ") + ex.what());
    } catch (...) {
        Clear();
        return OperationResult::Fail(StatusCode::kInternalError, "构建三角网时发生未预期内部异常");
    }
}

OperationResult TiledTinSurface::BuildFromCustomMesh(
    std::vector<TinVertex>&& vertices,
    std::vector<TinTriangle>&& triangles,
    double tile_size) {
    Clear();
    if (vertices.size() < 3 || triangles.empty()) {
        return OperationResult::Fail(
            StatusCode::kEmptyDataset,
            "自定义网格顶点数不足 3 个或三角形列表为空");
    }
    try {
        ExactPredicates::Initialize();
        vertices_ = std::move(vertices);
        triangles_ = std::move(triangles);
        RecomputeAllTriangleAdjacency();
        BuildSpatialTilesAndLookupGrid(tile_size);
        if (active_triangle_count_ == 0) {
            Clear();
            return OperationResult::Fail(
                StatusCode::kTopologyDegeneracy,
                "自定义网格中无有效活跃三角形");
        }
        GEO_LOG_INFO(
            "TIN",
            "自定义网格导入完成: verts=%zu, active_tris=%llu, tiles=%zu",
            vertices_.size(),
            static_cast<unsigned long long>(active_triangle_count_),
            tiles_.size());
        return OperationResult::Ok();
    } catch (const std::bad_alloc&) {
        Clear();
        return OperationResult::Fail(StatusCode::kOutOfMemory, "构建自定义网格索引时内存不足");
    } catch (...) {
        Clear();
        return OperationResult::Fail(StatusCode::kInternalError, "构建自定义网格索引时发生内部异常");
    }
}

bool TiledTinSurface::QueryElevationAt(
    double local_x,
    double local_y,
    double& out_z,
    uint32_t* out_triangle_id) const noexcept {
    if (!IsValid() || !mesh_bounds_.Contains2D(local_x, local_y)) {
        return false;
    }
    const double inv_tile = 1.0 / lookup_tile_size_;
    const auto gx = static_cast<int32_t>(std::floor(local_x * inv_tile));
    const auto gy = static_cast<int32_t>(std::floor(local_y * inv_tile));
    const uint64_t cell_uid = PackTileUid(gx, gy);
    auto it = tile_overlapping_triangles_.find(cell_uid);
    if (it == tile_overlapping_triangles_.end()) {
        return false;
    }
    const Vec3d query_pt{local_x, local_y, 0.0};
    for (uint32_t tid : it->second) {
        const TinTriangle& tri = triangles_[tid];
        if (!tri.is_active) continue;
        const Vec3d& a = vertices_[tri.v[0]].Position();
        const Vec3d& b = vertices_[tri.v[1]].Position();
        const Vec3d& c = vertices_[tri.v[2]].Position();
        if (local_x < std::min({a.x, b.x, c.x}) - 1e-9 ||
            local_x > std::max({a.x, b.x, c.x}) + 1e-9 ||
            local_y < std::min({a.y, b.y, c.y}) - 1e-9 ||
            local_y > std::max({a.y, b.y, c.y}) + 1e-9) {
            continue;
        }
        if (ExactPredicates::PointInTriangle2D(a, b, c, query_pt) != 0) {
            out_z = ExactPredicates::InterpolateZOnTriangle(a, b, c, local_x, local_y);
            if (out_triangle_id != nullptr) {
                *out_triangle_id = tid;
            }
            return true;
        }
    }
    return false;
}

RayHitResult TiledTinSurface::PickByRay(const Vec3d& ray_origin, const Vec3d& ray_dir) const noexcept {
    RayHitResult best_hit;
    if (!IsValid() || ray_dir.LengthSquared() <= 1e-18) {
        return best_hit;
    }
    const Vec3d dir = ray_dir.Normalized();
    const Vec3d inv_dir{
        std::abs(dir.x) > 1e-12 ? (1.0 / dir.x) : 1e12,
        std::abs(dir.y) > 1e-12 ? (1.0 / dir.y) : 1e12,
        std::abs(dir.z) > 1e-12 ? (1.0 / dir.z) : 1e12
    };
    if (!RayIntersectsAABB(ray_origin, inv_dir, mesh_bounds_)) {
        return best_hit;
    }
    for (const auto& tile : tiles_) {
        if (tile.triangle_indices.empty()) continue;
        if (!RayIntersectsAABB(ray_origin, inv_dir, tile.bounds)) {
            continue;
        }
        for (uint32_t tid : tile.triangle_indices) {
            const TinTriangle& tri = triangles_[tid];
            if (!tri.is_active) continue;
            const Vec3d& v0 = vertices_[tri.v[0]].Position();
            const Vec3d& v1 = vertices_[tri.v[1]].Position();
            const Vec3d& v2 = vertices_[tri.v[2]].Position();
            double t = 0.0;
            if (RayIntersectsTriangle3D(ray_origin, dir, v0, v1, v2, t) && t < best_hit.distance) {
                best_hit.hit = true;
                best_hit.triangle_id = tid;
                best_hit.distance = t;
                best_hit.hit_point = ray_origin + dir * t;
            }
        }
    }
    if (best_hit.hit) {
        const TinTriangle& tri = triangles_[best_hit.triangle_id];
        const Vec3d& hp = best_hit.hit_point;
        double min_v_dist = std::numeric_limits<double>::max();
        for (int i = 0; i < 3; ++i) {
            const double d_sq = (vertices_[tri.v[i]].Position() - hp).LengthSquared();
            if (d_sq < min_v_dist) {
                min_v_dist = d_sq;
                best_hit.nearest_vertex_id = tri.v[i];
            }
        }
        double min_e_dist = std::numeric_limits<double>::max();
        for (int e = 0; e < 3; ++e) {
            const Vec3d& a = vertices_[tri.v[(e + 1) % 3]].Position();
            const Vec3d& b = vertices_[tri.v[(e + 2) % 3]].Position();
            const double d_sq = PointToSegmentDistanceSq2D(hp.x, hp.y, a.x, a.y, b.x, b.y);
            if (d_sq < min_e_dist) {
                min_e_dist = d_sq;
                best_hit.nearest_edge_slot = e;
            }
        }
    }
    return best_hit;
}

bool TiledTinSurface::FindNearestVertex2D(
    double local_x,
    double local_y,
    double max_radius,
    uint32_t& out_vertex_id) const noexcept {
    if (!IsValid() || max_radius <= 0.0) return false;
    const double max_r_sq = max_radius * max_radius;
    double best_d_sq = max_r_sq;
    bool found = false;
    const double inv_tile = 1.0 / lookup_tile_size_;
    const auto gx0 = static_cast<int32_t>(std::floor((local_x - max_radius) * inv_tile));
    const auto gx1 = static_cast<int32_t>(std::floor((local_x + max_radius) * inv_tile));
    const auto gy0 = static_cast<int32_t>(std::floor((local_y - max_radius) * inv_tile));
    const auto gy1 = static_cast<int32_t>(std::floor((local_y + max_radius) * inv_tile));
    for (int32_t gx = gx0; gx <= gx1; ++gx) {
        for (int32_t gy = gy0; gy <= gy1; ++gy) {
            auto it = tile_overlapping_triangles_.find(PackTileUid(gx, gy));
            if (it == tile_overlapping_triangles_.end()) continue;
            for (uint32_t tid : it->second) {
                const TinTriangle& tri = triangles_[tid];
                if (!tri.is_active) continue;
                for (int i = 0; i < 3; ++i) {
                    const uint32_t vid = tri.v[i];
                    const TinVertex& v = vertices_[vid];
                    if (!v.IsActive()) continue;
                    const double dx = v.x - local_x;
                    const double dy = v.y - local_y;
                    const double d_sq = dx * dx + dy * dy;
                    if (d_sq <= best_d_sq) {
                        best_d_sq = d_sq;
                        out_vertex_id = vid;
                        found = true;
                    }
                }
            }
        }
    }
    return found;
}

bool TiledTinSurface::FindNearestEdge2D(
    double local_x,
    double local_y,
    double max_distance,
    uint32_t& out_triangle_id,
    int& out_edge_slot) const noexcept {
    if (!IsValid() || max_distance <= 0.0) return false;
    double best_d_sq = max_distance * max_distance;
    bool found = false;
    const double inv_tile = 1.0 / lookup_tile_size_;
    const auto gx0 = static_cast<int32_t>(std::floor((local_x - max_distance) * inv_tile));
    const auto gx1 = static_cast<int32_t>(std::floor((local_x + max_distance) * inv_tile));
    const auto gy0 = static_cast<int32_t>(std::floor((local_y - max_distance) * inv_tile));
    const auto gy1 = static_cast<int32_t>(std::floor((local_y + max_distance) * inv_tile));
    for (int32_t gx = gx0; gx <= gx1; ++gx) {
        for (int32_t gy = gy0; gy <= gy1; ++gy) {
            auto it = tile_overlapping_triangles_.find(PackTileUid(gx, gy));
            if (it == tile_overlapping_triangles_.end()) continue;
            for (uint32_t tid : it->second) {
                const TinTriangle& tri = triangles_[tid];
                if (!tri.is_active) continue;
                for (int e = 0; e < 3; ++e) {
                    const TinVertex& a = vertices_[tri.v[(e + 1) % 3]];
                    const TinVertex& b = vertices_[tri.v[(e + 2) % 3]];
                    const double d_sq = PointToSegmentDistanceSq2D(local_x, local_y, a.x, a.y, b.x, b.y);
                    if (d_sq <= best_d_sq) {
                        best_d_sq = d_sq;
                        out_triangle_id = tid;
                        out_edge_slot = e;
                        found = true;
                    }
                }
            }
        }
    }
    return found;
}

std::vector<uint32_t> TiledTinSurface::GetDirtyTileIndices() const {
    std::vector<uint32_t> dirty_list;
    for (const auto& tile : tiles_) {
        if (tile.is_dirty) {
            dirty_list.push_back(tile.tile_index);
        }
    }
    return dirty_list;
}

void TiledTinSurface::ClearTileDirty(uint32_t tile_index) noexcept {
    if (tile_index < tiles_.size()) {
        tiles_[tile_index].is_dirty = false;
    }
}

void TiledTinSurface::MarkTilesDirtyInBox2D(
    double min_x, double min_y, double max_x, double max_y) noexcept {
    if (tiles_.empty()) return;
    const double inv_tile = 1.0 / lookup_tile_size_;
    const auto gx0 = static_cast<int32_t>(std::floor(min_x * inv_tile)) - 1;
    const auto gx1 = static_cast<int32_t>(std::floor(max_x * inv_tile)) + 1;
    const auto gy0 = static_cast<int32_t>(std::floor(min_y * inv_tile)) - 1;
    const auto gy1 = static_cast<int32_t>(std::floor(max_y * inv_tile)) + 1;
    for (int32_t gx = gx0; gx <= gx1; ++gx) {
        for (int32_t gy = gy0; gy <= gy1; ++gy) {
            auto it = tile_uid_to_index_.find(PackTileUid(gx, gy));
            if (it != tile_uid_to_index_.end()) {
                TinSpatialTile& tile = tiles_[it->second];
                tile.is_dirty = true;
                ++tile.revision;
            }
        }
    }
}

OperationResult TiledTinSurface::SerializeTileToGpuPacket(
    uint32_t tile_index,
    const RtcTransform& rtc,
    std::vector<uint8_t>& out_packet) const {
    out_packet.clear();
    if (tile_index >= tiles_.size()) {
        return OperationResult::Fail(
            StatusCode::kTileNotFound,
            "请求的三角网瓦片编号越界: " + std::to_string(tile_index));
    }
    const TinSpatialTile& tile = tiles_[tile_index];
    std::unordered_map<uint32_t, uint32_t> global_to_local_vid;
    global_to_local_vid.reserve(tile.triangle_indices.size());
    std::vector<uint32_t> local_global_vids;
    std::vector<Vec3d> accumulated_normals;
    std::vector<uint32_t> local_indices;
    local_global_vids.reserve(tile.triangle_indices.size());
    accumulated_normals.reserve(tile.triangle_indices.size());
    local_indices.reserve(tile.triangle_indices.size() * 3);

    auto get_or_add_local_vertex = [&](uint32_t global_vid) -> uint32_t {
        auto it = global_to_local_vid.find(global_vid);
        if (it != global_to_local_vid.end()) {
            return it->second;
        }
        const auto local_vid = static_cast<uint32_t>(local_global_vids.size());
        global_to_local_vid.emplace(global_vid, local_vid);
        local_global_vids.push_back(global_vid);
        accumulated_normals.emplace_back(0.0, 0.0, 0.0);
        return local_vid;
    };

    for (uint32_t tid : tile.triangle_indices) {
        const TinTriangle& tri = triangles_[tid];
        if (!tri.is_active) continue;
        const Vec3d& p0 = vertices_[tri.v[0]].Position();
        const Vec3d& p1 = vertices_[tri.v[1]].Position();
        const Vec3d& p2 = vertices_[tri.v[2]].Position();
        Vec3d face_normal = (p1 - p0).Cross(p2 - p0);
        if (face_normal.z < 0.0) {
            face_normal = face_normal * -1.0;
        }
        const uint32_t lv0 = get_or_add_local_vertex(tri.v[0]);
        const uint32_t lv1 = get_or_add_local_vertex(tri.v[1]);
        const uint32_t lv2 = get_or_add_local_vertex(tri.v[2]);
        accumulated_normals[lv0] += face_normal;
        accumulated_normals[lv1] += face_normal;
        accumulated_normals[lv2] += face_normal;
        local_indices.push_back(lv0);
        local_indices.push_back(lv1);
        local_indices.push_back(lv2);
    }

    const uint32_t vertex_count = static_cast<uint32_t>(local_global_vids.size());
    const uint32_t index_count = static_cast<uint32_t>(local_indices.size());
    const size_t header_bytes = sizeof(GpuTilePacketHeader);
    const size_t vertex_bytes = static_cast<size_t>(vertex_count) * sizeof(GpuMeshVertex);
    const size_t index_bytes = static_cast<size_t>(index_count) * sizeof(uint32_t);
    out_packet.resize(header_bytes + vertex_bytes + index_bytes);

    GpuTilePacketHeader header{};
    header.magic = 0x544F4547u; // "GEOT"
    header.version = 1u;
    header.payload_type = static_cast<uint32_t>(GpuTilePayloadType::kTinSurfaceMesh);
    header.flags = 1u;
    header.tile_x = tile.tile_x;
    header.tile_y = tile.tile_y;
    header.tile_uid = tile.tile_uid;
    header.revision = tile.revision;
    header.vertex_count = vertex_count;
    header.index_count = index_count;
    header.vertex_stride = static_cast<uint32_t>(sizeof(GpuMeshVertex));
    header.rtc_origin[0] = rtc.origin.x;
    header.rtc_origin[1] = rtc.origin.y;
    header.rtc_origin[2] = rtc.origin.z;
    header.bbox_min[0] = static_cast<float>(tile.bounds.min_pt.x);
    header.bbox_min[1] = static_cast<float>(tile.bounds.min_pt.y);
    header.bbox_min[2] = static_cast<float>(tile.bounds.min_pt.z);
    header.bbox_max[0] = static_cast<float>(tile.bounds.max_pt.x);
    header.bbox_max[1] = static_cast<float>(tile.bounds.max_pt.y);
    header.bbox_max[2] = static_cast<float>(tile.bounds.max_pt.z);
    std::memcpy(out_packet.data(), &header, header_bytes);

    if (vertex_count > 0) {
        auto* dst_vertices = reinterpret_cast<GpuMeshVertex*>(out_packet.data() + header_bytes);
        for (uint32_t i = 0; i < vertex_count; ++i) {
            const TinVertex& v = vertices_[local_global_vids[i]];
            const Vec3d n = accumulated_normals[i].Normalized();
            dst_vertices[i].x = static_cast<float>(v.x);
            dst_vertices[i].y = static_cast<float>(v.y);
            dst_vertices[i].z = static_cast<float>(v.z);
            dst_vertices[i].nx = static_cast<float>(n.x);
            dst_vertices[i].ny = static_cast<float>(n.y);
            dst_vertices[i].nz = static_cast<float>(n.z);
            dst_vertices[i].packed_attr =
                (static_cast<uint32_t>(v.flags) << 16) | static_cast<uint32_t>(v.code_id);
        }
    }
    if (index_count > 0) {
        std::memcpy(
            out_packet.data() + header_bytes + vertex_bytes,
            local_indices.data(),
            index_bytes);
    }
    return OperationResult::Ok();
}

}  // namespace geo_kernel
