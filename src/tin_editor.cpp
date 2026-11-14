#include "geo_kernel/tin_editor.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "geo_kernel/exact_predicates.hpp"

namespace geo_kernel {

namespace {

constexpr double kVertexSnapEps = 1e-6;         // 顶点平面吸附阈值（1 微米）
constexpr double kVertexSnapEpsSq = 1e-12;
constexpr double kCollinearEps = 1e-9;

/**
 * @brief 计算二维线段 AB 与线段 CD 的精确交点（已知两线段在二维平面上跨立相交）
 */
Vec3d ComputeSegmentIntersection2D(
    const Vec3d& a, const Vec3d& b,
    const Vec3d& c, const Vec3d& d) noexcept {
    const double dx1 = b.x - a.x;
    const double dy1 = b.y - a.y;
    const double dx2 = d.x - c.x;
    const double dy2 = d.y - c.y;
    const double denom = dx1 * dy2 - dy1 * dx2;
    if (std::abs(denom) <= 1e-18) {
        return {(a.x + b.x) * 0.5, (a.y + b.y) * 0.5, (a.z + b.z) * 0.5};
    }
    const double t = ((c.x - a.x) * dy2 - (c.y - a.y) * dx2) / denom;
    const double tc = std::clamp(t, 0.0, 1.0);
    return {
        a.x + tc * dx1,
        a.y + tc * dy1,
        a.z + tc * (b.z - a.z)
    };
}

/**
 * @brief 射线法精确判断二维点 (px, py) 是否位于简单多边形内部
 */
bool IsPointInPolygon2D(double px, double py, const std::vector<Vec3d>& poly) noexcept {
    const size_t n = poly.size();
    if (n < 3) return false;
    bool inside = false;
    for (size_t i = 0, j = n - 1; i < n; j = i++) {
        const double xi = poly[i].x, yi = poly[i].y;
        const double xj = poly[j].x, yj = poly[j].y;
        const bool intersect = ((yi > py) != (yj > py)) &&
                               (px < (xj - xi) * (py - yi) / ((yj - yi) + 1e-30) + xi);
        if (intersect) {
            inside = !inside;
        }
    }
    return inside;
}

}  // namespace

// ==============================================================================
// 事务增量录制器（确保同一顶点/三角形在单次复合操作中只记录原始 before 状态与最终 after 状态）
// ==============================================================================

class TinTopologyEditor::TransactionRecorder {
public:
    explicit TransactionRecorder(TiledTinSurface& surface, std::string desc)
        : surface_(surface) {
        tx_.description = std::move(desc);
    }

    void RecordVertexBefore(uint32_t vid) {
        if (vid >= surface_.vertices_.size()) return;
        if (vertex_delta_index_.find(vid) == vertex_delta_index_.end()) {
            const size_t idx = tx_.vertex_deltas.size();
            vertex_delta_index_.emplace(vid, idx);
            VertexDelta d{};
            d.vertex_id = vid;
            d.existed_before = true;
            d.before_state = surface_.vertices_[vid];
            tx_.vertex_deltas.push_back(d);
            tx_.affected_bounds.Expand(d.before_state.Position());
        }
    }

    uint32_t AddNewVertex(const TinVertex& v) {
        const auto vid = static_cast<uint32_t>(surface_.vertices_.size());
        surface_.vertices_.push_back(v);
        const size_t idx = tx_.vertex_deltas.size();
        vertex_delta_index_.emplace(vid, idx);
        VertexDelta d{};
        d.vertex_id = vid;
        d.existed_before = false;
        d.after_state = v;
        tx_.vertex_deltas.push_back(d);
        tx_.affected_bounds.Expand(v.Position());
        return vid;
    }

    void RecordTriangleBefore(uint32_t tid) {
        if (tid == UINT32_MAX || tid >= surface_.triangles_.size()) return;
        if (tri_delta_index_.find(tid) == tri_delta_index_.end()) {
            const size_t idx = tx_.triangle_deltas.size();
            tri_delta_index_.emplace(tid, idx);
            TriangleDelta d{};
            d.triangle_id = tid;
            d.existed_before = true;
            d.before_state = surface_.triangles_[tid];
            tx_.triangle_deltas.push_back(d);

            if (d.before_state.is_active) {
                tx_.affected_bounds.Expand(surface_.vertices_[d.before_state.v[0]].Position());
                tx_.affected_bounds.Expand(surface_.vertices_[d.before_state.v[1]].Position());
                tx_.affected_bounds.Expand(surface_.vertices_[d.before_state.v[2]].Position());
            }
        }
    }

    uint32_t AllocateTriangle(const TinTriangle& tri) {
        const auto tid = static_cast<uint32_t>(surface_.triangles_.size());
        surface_.triangles_.push_back(tri);
        const size_t idx = tx_.triangle_deltas.size();
        tri_delta_index_.emplace(tid, idx);
        TriangleDelta d{};
        d.triangle_id = tid;
        d.existed_before = false;
        d.after_state = tri;
        tx_.triangle_deltas.push_back(d);

        if (tri.is_active) {
            tx_.affected_bounds.Expand(surface_.vertices_[tri.v[0]].Position());
            tx_.affected_bounds.Expand(surface_.vertices_[tri.v[1]].Position());
            tx_.affected_bounds.Expand(surface_.vertices_[tri.v[2]].Position());
        }
        return tid;
    }

    TinEditTransaction Finalize() {
        for (auto& vd : tx_.vertex_deltas) {
            if (vd.vertex_id < surface_.vertices_.size()) {
                vd.after_state = surface_.vertices_[vd.vertex_id];
                tx_.affected_bounds.Expand(vd.after_state.Position());
            }
        }
        for (auto& td : tx_.triangle_deltas) {
            if (td.triangle_id < surface_.triangles_.size()) {
                td.after_state = surface_.triangles_[td.triangle_id];
                if (td.after_state.is_active) {
                    tx_.affected_bounds.Expand(surface_.vertices_[td.after_state.v[0]].Position());
                    tx_.affected_bounds.Expand(surface_.vertices_[td.after_state.v[1]].Position());
                    tx_.affected_bounds.Expand(surface_.vertices_[td.after_state.v[2]].Position());
                }
            }
        }
        return std::move(tx_);
    }

    void Rollback() {
        for (auto it = tx_.triangle_deltas.rbegin(); it != tx_.triangle_deltas.rend(); ++it) {
            if (it->existed_before) {
                surface_.triangles_[it->triangle_id] = it->before_state;
            } else {
                surface_.triangles_[it->triangle_id].is_active = 0;
            }
        }
        while (!surface_.triangles_.empty() &&
               !tx_.triangle_deltas.empty() &&
               !tx_.triangle_deltas.back().existed_before &&
               tx_.triangle_deltas.back().triangle_id == surface_.triangles_.size() - 1) {
            surface_.triangles_.pop_back();
            tx_.triangle_deltas.pop_back();
        }

        for (auto it = tx_.vertex_deltas.rbegin(); it != tx_.vertex_deltas.rend(); ++it) {
            if (it->existed_before) {
                surface_.vertices_[it->vertex_id] = it->before_state;
            }
        }
        while (!surface_.vertices_.empty() &&
               !tx_.vertex_deltas.empty() &&
               !tx_.vertex_deltas.back().existed_before &&
               tx_.vertex_deltas.back().vertex_id == surface_.vertices_.size() - 1) {
            surface_.vertices_.pop_back();
            tx_.vertex_deltas.pop_back();
        }
    }

private:
    TiledTinSurface& surface_;
    TinEditTransaction tx_;
    std::unordered_map<uint32_t, size_t> vertex_delta_index_;
    std::unordered_map<uint32_t, size_t> tri_delta_index_;
};

// ==============================================================================
// TinTopologyEditor 公开接口与拓扑算法实现
// ==============================================================================

TinTopologyEditor::TinTopologyEditor(TiledTinSurface& surface, size_t max_undo_levels) noexcept
    : surface_(surface),
      max_undo_levels_(max_undo_levels > 0 ? max_undo_levels : 64) {
    ExactPredicates::Initialize();
}

void TinTopologyEditor::ClearHistory() noexcept {
    undo_stack_.clear();
    redo_stack_.clear();
}

void TinTopologyEditor::PushTransaction(TinEditTransaction&& tx) {
    redo_stack_.clear();
    if (undo_stack_.size() >= max_undo_levels_) {
        undo_stack_.erase(undo_stack_.begin());
    }
    undo_stack_.push_back(std::move(tx));
}

OperationResult TinTopologyEditor::FlipEdge(
    uint32_t tri_id,
    int edge_slot,
    bool force_flip) {
    if (!surface_.IsValid()) {
        return OperationResult::Fail(StatusCode::kEmptyDataset, "当前三角网为空，无法执行边翻转");
    }
    if (tri_id >= surface_.triangles_.size() || edge_slot < 0 || edge_slot > 2) {
        return OperationResult::Fail(StatusCode::kInvalidArgument, "非法的三角形编号或边槽位索引");
    }

    const TinTriangle& t_a = surface_.triangles_[tri_id];
    if (!t_a.is_active) {
        return OperationResult::Fail(StatusCode::kInvalidArgument, "目标三角形已被删除或裁剪");
    }

    const uint32_t adj_id = t_a.adj[edge_slot];
    if (adj_id == UINT32_MAX || adj_id >= surface_.triangles_.size() || !surface_.triangles_[adj_id].is_active) {
        return OperationResult::Fail(
            StatusCode::kTopologyDegeneracy,
            "目标边为三角网外边界边，不存在相邻三角形，无法执行对角线翻转");
    }

    if (!force_flip && t_a.IsEdgeConstrained(edge_slot)) {
        return OperationResult::Fail(
            StatusCode::kTopologyDegeneracy,
            "目标边为已锁定的特征线/断裂线约束边，禁止普通边翻转");
    }

    const TinTriangle& t_b = surface_.triangles_[adj_id];
    const int adj_slot = t_b.IndexOfNeighbor(tri_id);
    if (adj_slot < 0) {
        return OperationResult::Fail(StatusCode::kInternalError, "三角形双向邻接指针不一致");
    }

    const uint32_t opp_a = t_a.v[edge_slot];
    const uint32_t s0 = t_a.v[(edge_slot + 1) % 3];
    const uint32_t s1 = t_a.v[(edge_slot + 2) % 3];
    const uint32_t opp_b = t_b.v[adj_slot];

    const Vec3d& p_opp_a = surface_.vertices_[opp_a].Position();
    const Vec3d& p_s0 = surface_.vertices_[s0].Position();
    const Vec3d& p_opp_b = surface_.vertices_[opp_b].Position();
    const Vec3d& p_s1 = surface_.vertices_[s1].Position();

    // 严格凸性检验：四边形 (opp_a -> s0 -> opp_b -> s1) 的两条新对角线内角必须严格小于 180 度
    if (ExactPredicates::Orient2D(p_opp_a, p_s0, p_opp_b) <= 0.0 ||
        ExactPredicates::Orient2D(p_opp_b, p_s1, p_opp_a) <= 0.0) {
        return OperationResult::Fail(
            StatusCode::kTopologyDegeneracy,
            "目标共享边所在的四边形为非凸多边形，翻转对角线会导致三角形重叠退化");
    }

    TransactionRecorder recorder(surface_, "对角线边翻转");
    recorder.RecordTriangleBefore(tri_id);
    recorder.RecordTriangleBefore(adj_id);

    const int slot_a_s1_oppa = (edge_slot + 1) % 3;
    const int slot_a_oppa_s0 = (edge_slot + 2) % 3;
    const int slot_b_s0_oppb = (adj_slot + 1) % 3;
    const int slot_b_oppb_s1 = (adj_slot + 2) % 3;

    const uint32_t nb_s1_oppa = t_a.adj[slot_a_s1_oppa];
    const uint32_t nb_oppa_s0 = t_a.adj[slot_a_oppa_s0];
    const uint32_t nb_s0_oppb = t_b.adj[slot_b_s0_oppb];
    const uint32_t nb_oppb_s1 = t_b.adj[slot_b_oppb_s1];

    const bool c_s1_oppa = t_a.IsEdgeConstrained(slot_a_s1_oppa);
    const bool c_oppa_s0 = t_a.IsEdgeConstrained(slot_a_oppa_s0);
    const bool c_s0_oppb = t_b.IsEdgeConstrained(slot_b_s0_oppb);
    const bool c_oppb_s1 = t_b.IsEdgeConstrained(slot_b_oppb_s1);

    recorder.RecordTriangleBefore(nb_s1_oppa);
    recorder.RecordTriangleBefore(nb_oppa_s0);
    recorder.RecordTriangleBefore(nb_s0_oppb);
    recorder.RecordTriangleBefore(nb_oppb_s1);

    TinTriangle new_a = t_a;
    new_a.v[0] = opp_a;
    new_a.v[1] = s0;
    new_a.v[2] = opp_b;
    new_a.adj[0] = nb_s0_oppb;
    new_a.adj[1] = adj_id;
    new_a.adj[2] = nb_oppa_s0;
    new_a.constrained_mask = 0;
    new_a.SetEdgeConstrained(0, c_s0_oppb);
    new_a.SetEdgeConstrained(2, c_oppa_s0);

    TinTriangle new_b = t_b;
    new_b.v[0] = opp_b;
    new_b.v[1] = s1;
    new_b.v[2] = opp_a;
    new_b.adj[0] = nb_s1_oppa;
    new_b.adj[1] = tri_id;
    new_b.adj[2] = nb_oppb_s1;
    new_b.constrained_mask = 0;
    new_b.SetEdgeConstrained(0, c_s1_oppa);
    new_b.SetEdgeConstrained(2, c_oppb_s1);

    surface_.triangles_[tri_id] = new_a;
    surface_.triangles_[adj_id] = new_b;

    auto relink_outer = [&](uint32_t outer_tid, uint32_t old_target, uint32_t new_target, int new_slot) {
        if (outer_tid == UINT32_MAX || outer_tid >= surface_.triangles_.size()) return;
        TinTriangle& ot = surface_.triangles_[outer_tid];
        const int s = ot.IndexOfNeighbor(old_target);
        if (s >= 0) {
            ot.adj[s] = new_target;
            const bool is_c = surface_.triangles_[new_target].IsEdgeConstrained(new_slot);
            ot.SetEdgeConstrained(s, is_c);
        }
    };

    relink_outer(nb_s0_oppb, adj_id, tri_id, 0);
    relink_outer(nb_oppa_s0, tri_id, tri_id, 2);
    relink_outer(nb_s1_oppa, tri_id, adj_id, 0);
    relink_outer(nb_oppb_s1, adj_id, adj_id, 2);

    TinEditTransaction tx = recorder.Finalize();
    surface_.RebuildSpatialTilesAfterEdit(tx.affected_bounds);
    PushTransaction(std::move(tx));
    return OperationResult::Ok();
}

OperationResult TinTopologyEditor::ModifyVertexElevation(uint32_t vertex_id, double new_local_z) {
    if (!surface_.IsValid()) {
        return OperationResult::Fail(StatusCode::kEmptyDataset, "当前三角网为空，无法修改顶点高程");
    }
    if (vertex_id >= surface_.vertices_.size() || !surface_.vertices_[vertex_id].IsActive()) {
        return OperationResult::Fail(StatusCode::kInvalidArgument, "指定的顶点编号无效或已被删除");
    }
    if (!std::isfinite(new_local_z)) {
        return OperationResult::Fail(StatusCode::kInvalidArgument, "目标高程数值非法 (NaN/Inf)");
    }

    TransactionRecorder recorder(surface_, "修改顶点高程");
    recorder.RecordVertexBefore(vertex_id);
    surface_.vertices_[vertex_id].z = new_local_z;

    TinEditTransaction tx = recorder.Finalize();
    const double pad = surface_.TileSize();
    tx.affected_bounds.Expand({
        surface_.vertices_[vertex_id].x - pad,
        surface_.vertices_[vertex_id].y - pad,
        new_local_z
    });
    tx.affected_bounds.Expand({
        surface_.vertices_[vertex_id].x + pad,
        surface_.vertices_[vertex_id].y + pad,
        new_local_z
    });

    surface_.RebuildSpatialTilesAfterEdit(tx.affected_bounds);
    PushTransaction(std::move(tx));
    return OperationResult::Ok();
}

void TinTopologyEditor::LegalizeEdgesInternal(
    std::vector<std::pair<uint32_t, int>>& edge_stack,
    TransactionRecorder& recorder) {
    size_t guard = 0;
    const size_t max_flips = surface_.triangles_.size() * 4 + 256;

    while (!edge_stack.empty() && guard++ < max_flips) {
        const auto [tid, slot] = edge_stack.back();
        edge_stack.pop_back();

        if (tid >= surface_.triangles_.size() || slot < 0 || slot > 2) continue;
        const TinTriangle& t_a = surface_.triangles_[tid];
        if (!t_a.is_active || t_a.IsEdgeConstrained(slot)) continue;

        const uint32_t adj_id = t_a.adj[slot];
        if (adj_id == UINT32_MAX || adj_id >= surface_.triangles_.size()) continue;
        const TinTriangle& t_b = surface_.triangles_[adj_id];
        if (!t_b.is_active) continue;

        const int adj_slot = t_b.IndexOfNeighbor(tid);
        if (adj_slot < 0) continue;

        const uint32_t opp_a = t_a.v[slot];
        const uint32_t s0 = t_a.v[(slot + 1) % 3];
        const uint32_t s1 = t_a.v[(slot + 2) % 3];
        const uint32_t opp_b = t_b.v[adj_slot];

        const Vec3d& p_opp_a = surface_.vertices_[opp_a].Position();
        const Vec3d& p_s0 = surface_.vertices_[s0].Position();
        const Vec3d& p_s1 = surface_.vertices_[s1].Position();
        const Vec3d& p_opp_b = surface_.vertices_[opp_b].Position();

        if (ExactPredicates::InCircle(p_opp_b, p_s1, p_s0, p_opp_a) > 0.0) {
            if (ExactPredicates::Orient2D(p_opp_a, p_s0, p_opp_b) <= 0.0 ||
                ExactPredicates::Orient2D(p_opp_b, p_s1, p_opp_a) <= 0.0) {
                continue;
            }

            recorder.RecordTriangleBefore(tid);
            recorder.RecordTriangleBefore(adj_id);

            const int slot_a_s1_oppa = (slot + 1) % 3;
            const int slot_a_oppa_s0 = (slot + 2) % 3;
            const int slot_b_s0_oppb = (adj_slot + 1) % 3;
            const int slot_b_oppb_s1 = (adj_slot + 2) % 3;

            const uint32_t nb_s1_oppa = t_a.adj[slot_a_s1_oppa];
            const uint32_t nb_oppa_s0 = t_a.adj[slot_a_oppa_s0];
            const uint32_t nb_s0_oppb = t_b.adj[slot_b_s0_oppb];
            const uint32_t nb_oppb_s1 = t_b.adj[slot_b_oppb_s1];

            const bool c_s1_oppa = t_a.IsEdgeConstrained(slot_a_s1_oppa);
            const bool c_oppa_s0 = t_a.IsEdgeConstrained(slot_a_oppa_s0);
            const bool c_s0_oppb = t_b.IsEdgeConstrained(slot_b_s0_oppb);
            const bool c_oppb_s1 = t_b.IsEdgeConstrained(slot_b_oppb_s1);

            recorder.RecordTriangleBefore(nb_s1_oppa);
            recorder.RecordTriangleBefore(nb_oppa_s0);
            recorder.RecordTriangleBefore(nb_s0_oppb);
            recorder.RecordTriangleBefore(nb_oppb_s1);

            TinTriangle new_a = t_a;
            new_a.v[0] = opp_a;
            new_a.v[1] = s0;
            new_a.v[2] = opp_b;
            new_a.adj[0] = nb_s0_oppb;
            new_a.adj[1] = adj_id;
            new_a.adj[2] = nb_oppa_s0;
            new_a.constrained_mask = 0;
            new_a.SetEdgeConstrained(0, c_s0_oppb);
            new_a.SetEdgeConstrained(2, c_oppa_s0);

            TinTriangle new_b = t_b;
            new_b.v[0] = opp_b;
            new_b.v[1] = s1;
            new_b.v[2] = opp_a;
            new_b.adj[0] = nb_s1_oppa;
            new_b.adj[1] = tid;
            new_b.adj[2] = nb_oppb_s1;
            new_b.constrained_mask = 0;
            new_b.SetEdgeConstrained(0, c_s1_oppa);
            new_b.SetEdgeConstrained(2, c_oppb_s1);

            surface_.triangles_[tid] = new_a;
            surface_.triangles_[adj_id] = new_b;

            auto relink = [&](uint32_t outer_tid, uint32_t old_t, uint32_t new_t) {
                if (outer_tid == UINT32_MAX || outer_tid >= surface_.triangles_.size()) return;
                TinTriangle& ot = surface_.triangles_[outer_tid];
                const int s = ot.IndexOfNeighbor(old_t);
                if (s >= 0) ot.adj[s] = new_t;
            };
            relink(nb_s0_oppb, adj_id, tid);
            relink(nb_oppa_s0, tid, tid);
            relink(nb_s1_oppa, tid, adj_id);
            relink(nb_oppb_s1, adj_id, adj_id);

            edge_stack.emplace_back(tid, 0);
            edge_stack.emplace_back(adj_id, 2);
        }
    }
}

OperationResult TinTopologyEditor::InsertVertexInternal(
    const Vec3d& local_pt,
    uint32_t source_pt_id,
    uint16_t code_id,
    uint16_t flags,
    bool legalize_delaunay,
    TransactionRecorder& recorder,
    uint32_t& out_vertex_id) {
    uint32_t existing_vid = UINT32_MAX;
    if (surface_.FindNearestVertex2D(local_pt.x, local_pt.y, kVertexSnapEps, existing_vid)) {
        out_vertex_id = existing_vid;
        return OperationResult::Ok();
    }

    double interp_z = 0.0;
    uint32_t hit_tid = UINT32_MAX;
    if (!surface_.QueryElevationAt(local_pt.x, local_pt.y, interp_z, &hit_tid) ||
        hit_tid == UINT32_MAX) {
        return OperationResult::Fail(
            StatusCode::kInvalidArgument,
            "待插入顶点位于当前三角网平面覆盖范围之外");
    }

    TinVertex new_v{};
    new_v.x = local_pt.x;
    new_v.y = local_pt.y;
    new_v.z = local_pt.z;
    new_v.source_point_id = source_pt_id;
    new_v.code_id = code_id;
    new_v.flags = flags | TinVertex::kFlagActive;
    const uint32_t new_vid = recorder.AddNewVertex(new_v);
    out_vertex_id = new_vid;

    const TinTriangle orig_t = surface_.triangles_[hit_tid];
    const Vec3d& p0 = surface_.vertices_[orig_t.v[0]].Position();
    const Vec3d& p1 = surface_.vertices_[orig_t.v[1]].Position();
    const Vec3d& p2 = surface_.vertices_[orig_t.v[2]].Position();

    int on_edge_slot = -1;
    const double area2 = std::abs(ExactPredicates::Orient2D(p0, p1, p2));
    const double len_scale = std::sqrt(std::max(area2, 1e-12));
    for (int e = 0; e < 3; ++e) {
        const Vec3d& ea = surface_.vertices_[orig_t.v[(e + 1) % 3]].Position();
        const Vec3d& eb = surface_.vertices_[orig_t.v[(e + 2) % 3]].Position();
        const double cross_val = std::abs(ExactPredicates::Orient2D(ea, eb, local_pt));
        if (cross_val <= kCollinearEps * len_scale) {
            on_edge_slot = e;
            break;
        }
    }

    std::vector<std::pair<uint32_t, int>> legalize_stack;

    if (on_edge_slot >= 0 && orig_t.adj[on_edge_slot] != UINT32_MAX &&
        surface_.triangles_[orig_t.adj[on_edge_slot]].is_active) {
        const uint32_t t1_id = hit_tid;
        const uint32_t t2_id = orig_t.adj[on_edge_slot];
        const TinTriangle t1 = surface_.triangles_[t1_id];
        const TinTriangle t2 = surface_.triangles_[t2_id];
        const int s2 = t2.IndexOfNeighbor(t1_id);
        if (s2 < 0) {
            return OperationResult::Fail(StatusCode::kInternalError, "边上插点时邻接拓扑异常");
        }

        const bool edge_was_constrained = t1.IsEdgeConstrained(on_edge_slot);

        const uint32_t a = t1.v[on_edge_slot];
        const uint32_t b = t1.v[(on_edge_slot + 1) % 3];
        const uint32_t c = t1.v[(on_edge_slot + 2) % 3];
        const uint32_t d = t2.v[s2];

        const uint32_t nb_ca = t1.adj[(on_edge_slot + 1) % 3];
        const uint32_t nb_ab = t1.adj[(on_edge_slot + 2) % 3];
        const uint32_t nb_bd = t2.adj[(s2 + 1) % 3];
        const uint32_t nb_dc = t2.adj[(s2 + 2) % 3];

        const bool c_ca = t1.IsEdgeConstrained((on_edge_slot + 1) % 3);
        const bool c_ab = t1.IsEdgeConstrained((on_edge_slot + 2) % 3);
        const bool c_bd = t2.IsEdgeConstrained((s2 + 1) % 3);
        const bool c_dc = t2.IsEdgeConstrained((s2 + 2) % 3);

        recorder.RecordTriangleBefore(t1_id);
        recorder.RecordTriangleBefore(t2_id);
        recorder.RecordTriangleBefore(nb_ca);
        recorder.RecordTriangleBefore(nb_ab);
        recorder.RecordTriangleBefore(nb_bd);
        recorder.RecordTriangleBefore(nb_dc);

        TinTriangle nt_ab{}, nt_ca{}, nt_bd{}, nt_dc{};
        nt_ab.region_tag = t1.region_tag;
        nt_ca.region_tag = t1.region_tag;
        nt_bd.region_tag = t2.region_tag;
        nt_dc.region_tag = t2.region_tag;

        const uint32_t id_ab = t1_id;
        const uint32_t id_bd = t2_id;
        const uint32_t id_ca = recorder.AllocateTriangle(nt_ca);
        const uint32_t id_dc = recorder.AllocateTriangle(nt_dc);

        nt_ab.v[0] = new_vid; nt_ab.v[1] = a; nt_ab.v[2] = b;
        nt_ab.adj[0] = nb_ab; nt_ab.adj[1] = id_bd; nt_ab.adj[2] = id_ca;
        nt_ab.SetEdgeConstrained(0, c_ab);
        nt_ab.SetEdgeConstrained(1, edge_was_constrained);

        nt_ca.v[0] = new_vid; nt_ca.v[1] = c; nt_ca.v[2] = a;
        nt_ca.adj[0] = nb_ca; nt_ca.adj[1] = id_ab; nt_ca.adj[2] = id_dc;
        nt_ca.SetEdgeConstrained(0, c_ca);
        nt_ca.SetEdgeConstrained(2, edge_was_constrained);

        nt_bd.v[0] = new_vid; nt_bd.v[1] = b; nt_bd.v[2] = d;
        nt_bd.adj[0] = nb_bd; nt_bd.adj[1] = id_dc; nt_bd.adj[2] = id_ab;
        nt_bd.SetEdgeConstrained(0, c_bd);
        nt_bd.SetEdgeConstrained(2, edge_was_constrained);

        nt_dc.v[0] = new_vid; nt_dc.v[1] = d; nt_dc.v[2] = c;
        nt_dc.adj[0] = nb_dc; nt_dc.adj[1] = id_ca; nt_dc.adj[2] = id_bd;
        nt_dc.SetEdgeConstrained(0, c_dc);
        nt_dc.SetEdgeConstrained(1, edge_was_constrained);

        surface_.triangles_[id_ab] = nt_ab;
        surface_.triangles_[id_bd] = nt_bd;
        surface_.triangles_[id_ca] = nt_ca;
        surface_.triangles_[id_dc] = nt_dc;

        auto relink = [&](uint32_t outer_tid, uint32_t old_t, uint32_t new_t) {
            if (outer_tid == UINT32_MAX || outer_tid >= surface_.triangles_.size()) return;
            TinTriangle& ot = surface_.triangles_[outer_tid];
            const int s = ot.IndexOfNeighbor(old_t);
            if (s >= 0) ot.adj[s] = new_t;
        };
        relink(nb_ab, t1_id, id_ab);
        relink(nb_ca, t1_id, id_ca);
        relink(nb_bd, t2_id, id_bd);
        relink(nb_dc, t2_id, id_dc);

        legalize_stack.emplace_back(id_ab, 0);
        legalize_stack.emplace_back(id_ca, 0);
        legalize_stack.emplace_back(id_bd, 0);
        legalize_stack.emplace_back(id_dc, 0);
    } else {
        recorder.RecordTriangleBefore(hit_tid);
        const uint32_t nb0 = orig_t.adj[0];
        const uint32_t nb1 = orig_t.adj[1];
        const uint32_t nb2 = orig_t.adj[2];
        recorder.RecordTriangleBefore(nb0);
        recorder.RecordTriangleBefore(nb1);
        recorder.RecordTriangleBefore(nb2);

        TinTriangle nt0{}, nt1{}, nt2{};
        nt0.region_tag = orig_t.region_tag;
        nt1.region_tag = orig_t.region_tag;
        nt2.region_tag = orig_t.region_tag;

        const uint32_t id0 = hit_tid;
        const uint32_t id1 = recorder.AllocateTriangle(nt1);
        const uint32_t id2 = recorder.AllocateTriangle(nt2);

        nt0.v[0] = new_vid; nt0.v[1] = orig_t.v[1]; nt0.v[2] = orig_t.v[2];
        nt0.adj[0] = nb0; nt0.adj[1] = id1; nt0.adj[2] = id2;
        nt0.SetEdgeConstrained(0, orig_t.IsEdgeConstrained(0));

        nt1.v[0] = new_vid; nt1.v[1] = orig_t.v[2]; nt1.v[2] = orig_t.v[0];
        nt1.adj[0] = nb1; nt1.adj[1] = id2; nt1.adj[2] = id0;
        nt1.SetEdgeConstrained(0, orig_t.IsEdgeConstrained(1));

        nt2.v[0] = new_vid; nt2.v[1] = orig_t.v[0]; nt2.v[2] = orig_t.v[1];
        nt2.adj[0] = nb2; nt2.adj[1] = id0; nt2.adj[2] = id1;
        nt2.SetEdgeConstrained(0, orig_t.IsEdgeConstrained(2));

        surface_.triangles_[id0] = nt0;
        surface_.triangles_[id1] = nt1;
        surface_.triangles_[id2] = nt2;

        auto relink = [&](uint32_t outer_tid, uint32_t old_t, uint32_t new_t) {
            if (outer_tid == UINT32_MAX || outer_tid >= surface_.triangles_.size()) return;
            TinTriangle& ot = surface_.triangles_[outer_tid];
            const int s = ot.IndexOfNeighbor(old_t);
            if (s >= 0) ot.adj[s] = new_t;
        };
        relink(nb1, hit_tid, id1);
        relink(nb2, hit_tid, id2);

        legalize_stack.emplace_back(id0, 0);
        legalize_stack.emplace_back(id1, 0);
        legalize_stack.emplace_back(id2, 0);
    }

    if (legalize_delaunay) {
        LegalizeEdgesInternal(legalize_stack, recorder);
    }
    return OperationResult::Ok();
}

OperationResult TinTopologyEditor::InsertVertex(
    const Vec3d& local_pt,
    uint32_t source_pt_id,
    uint16_t code_id,
    uint16_t flags,
    bool legalize_delaunay,
    uint32_t* out_vertex_id) {
    if (!surface_.IsValid()) {
        return OperationResult::Fail(StatusCode::kEmptyDataset, "当前三角网为空，无法插入顶点");
    }

    TransactionRecorder recorder(surface_, "插入三角网顶点");
    uint32_t vid = UINT32_MAX;
    OperationResult res = InsertVertexInternal(
        local_pt, source_pt_id, code_id, flags, legalize_delaunay, recorder, vid);
    if (!res.IsOk()) {
        recorder.Rollback();
        return res;
    }

    if (out_vertex_id != nullptr) {
        *out_vertex_id = vid;
    }

    TinEditTransaction tx = recorder.Finalize();
    surface_.RebuildSpatialTilesAfterEdit(tx.affected_bounds);
    PushTransaction(std::move(tx));
    return OperationResult::Ok();
}

void TinTopologyEditor::TriangulateSimpleCavityPolygon(
    const std::vector<uint32_t>& poly_verts,
    const std::vector<uint32_t>& outer_tris,
    const std::vector<int>& outer_slots,
    const std::vector<uint8_t>& outer_constrained,
    uint32_t constrained_base_v0,
    uint32_t constrained_base_v1,
    uint32_t& out_base_tri_id,
    int& out_base_edge_slot,
    TransactionRecorder& recorder) {
    out_base_tri_id = UINT32_MAX;
    out_base_edge_slot = -1;

    const size_t n = poly_verts.size();
    if (n < 3) return;

    struct PolyNode {
        uint32_t vid = UINT32_MAX;
        uint32_t edge_adj_tri = UINT32_MAX;
        int edge_adj_slot = -1;
        uint8_t edge_constrained = 0;
    };

    std::vector<PolyNode> ring(n);
    for (size_t i = 0; i < n; ++i) {
        ring[i].vid = poly_verts[i];
        ring[i].edge_adj_tri = outer_tris[i];
        ring[i].edge_adj_slot = outer_slots[i];
        ring[i].edge_constrained = outer_constrained[i];
    }

    std::vector<uint32_t> created_tris;
    created_tris.reserve(n - 2);
    std::vector<std::pair<uint32_t, int>> internal_legalize_edges;

    size_t guard = 0;
    while (ring.size() > 3 && guard++ < n * n + 16) {
        bool ear_found = false;
        const size_t m = ring.size();

        for (size_t i = 0; i < m; ++i) {
            const size_t prev_idx = (i + m - 1) % m;
            const size_t next_idx = (i + 1) % m;

            const uint32_t va = ring[prev_idx].vid;
            const uint32_t vb = ring[i].vid;
            const uint32_t vc = ring[next_idx].vid;

            const Vec3d& pa = surface_.vertices_[va].Position();
            const Vec3d& pb = surface_.vertices_[vb].Position();
            const Vec3d& pc = surface_.vertices_[vc].Position();

            if (ExactPredicates::Orient2D(pa, pb, pc) <= 0.0) {
                continue;
            }

            bool contains_other_vertex = false;
            for (size_t k = 0; k < m; ++k) {
                if (k == prev_idx || k == i || k == next_idx) continue;
                const uint32_t vk = ring[k].vid;
                if (vk == va || vk == vb || vk == vc) continue;
                if (ExactPredicates::PointInTriangle2D(
                        pa, pb, pc, surface_.vertices_[vk].Position()) != 0) {
                    contains_other_vertex = true;
                    break;
                }
            }
            if (contains_other_vertex) {
                continue;
            }

            TinTriangle nt{};
            nt.v[0] = va;
            nt.v[1] = vb;
            nt.v[2] = vc;
            nt.is_active = 1;

            nt.adj[0] = ring[i].edge_adj_tri;
            nt.adj[2] = ring[prev_idx].edge_adj_tri;
            nt.SetEdgeConstrained(0, ring[i].edge_constrained != 0);
            nt.SetEdgeConstrained(2, ring[prev_idx].edge_constrained != 0);

            const uint32_t new_tid = recorder.AllocateTriangle(nt);
            created_tris.push_back(new_tid);

            if (nt.adj[0] != UINT32_MAX && ring[i].edge_adj_slot >= 0) {
                recorder.RecordTriangleBefore(nt.adj[0]);
                surface_.triangles_[nt.adj[0]].adj[ring[i].edge_adj_slot] = new_tid;
            }
            if (nt.adj[2] != UINT32_MAX && ring[prev_idx].edge_adj_slot >= 0) {
                recorder.RecordTriangleBefore(nt.adj[2]);
                surface_.triangles_[nt.adj[2]].adj[ring[prev_idx].edge_adj_slot] = new_tid;
            }

            ring[prev_idx].edge_adj_tri = new_tid;
            ring[prev_idx].edge_adj_slot = 1;
            ring[prev_idx].edge_constrained = 0;

            internal_legalize_edges.emplace_back(new_tid, 1);

            ring.erase(ring.begin() + static_cast<std::ptrdiff_t>(i));
            ear_found = true;
            break;
        }

        if (!ear_found) {
            break;
        }
    }

    if (ring.size() == 3) {
        TinTriangle last_t{};
        last_t.v[0] = ring[0].vid;
        last_t.v[1] = ring[1].vid;
        last_t.v[2] = ring[2].vid;
        last_t.is_active = 1;

        last_t.adj[0] = ring[1].edge_adj_tri;
        last_t.adj[1] = ring[2].edge_adj_tri;
        last_t.adj[2] = ring[0].edge_adj_tri;
        last_t.SetEdgeConstrained(0, ring[1].edge_constrained != 0);
        last_t.SetEdgeConstrained(1, ring[2].edge_constrained != 0);
        last_t.SetEdgeConstrained(2, ring[0].edge_constrained != 0);

        const uint32_t last_tid = recorder.AllocateTriangle(last_t);
        created_tris.push_back(last_tid);

        for (int e = 0; e < 3; ++e) {
            const size_t r_idx = (e + 1) % 3;
            const uint32_t nb = ring[r_idx].edge_adj_tri;
            const int nb_slot = ring[r_idx].edge_adj_slot;
            if (nb != UINT32_MAX && nb_slot >= 0) {
                recorder.RecordTriangleBefore(nb);
                surface_.triangles_[nb].adj[nb_slot] = last_tid;
            }
        }
    }

    LegalizeEdgesInternal(internal_legalize_edges, recorder);

    for (uint32_t tid : created_tris) {
        const TinTriangle& t = surface_.triangles_[tid];
        if (!t.is_active) continue;
        for (int e = 0; e < 3; ++e) {
            if (t.v[(e + 1) % 3] == constrained_base_v0 &&
                t.v[(e + 2) % 3] == constrained_base_v1) {
                out_base_tri_id = tid;
                out_base_edge_slot = e;
                return;
            }
        }
    }
}

OperationResult TinTopologyEditor::InsertConstrainedSegmentInternal(
    uint32_t start_vid,
    uint32_t end_vid,
    uint16_t code_id,
    TransactionRecorder& recorder,
    int recursion_depth) {
    if (start_vid == end_vid || recursion_depth > 64) {
        return OperationResult::Ok();
    }

    const Vec3d& p_start = surface_.vertices_[start_vid].Position();
    const Vec3d& p_end = surface_.vertices_[end_vid].Position();
    if (p_start.DistanceSquared2D(p_end) <= kVertexSnapEpsSq) {
        return OperationResult::Ok();
    }

    uint32_t start_fan_tri = UINT32_MAX;
    for (uint32_t tid = 0; tid < surface_.triangles_.size(); ++tid) {
        TinTriangle& t = surface_.triangles_[tid];
        if (!t.is_active) continue;
        const int idx_s = t.IndexOfVertex(start_vid);
        if (idx_s < 0) continue;

        start_fan_tri = tid;
        const int idx_e = t.IndexOfVertex(end_vid);
        if (idx_e >= 0) {
            const int shared_slot = 3 - idx_s - idx_e;
            recorder.RecordTriangleBefore(tid);
            t.SetEdgeConstrained(shared_slot, true);
            if (code_id != 0) {
                recorder.RecordVertexBefore(start_vid);
                recorder.RecordVertexBefore(end_vid);
                surface_.vertices_[start_vid].code_id = code_id;
                surface_.vertices_[end_vid].code_id = code_id;
            }

            const uint32_t nb_tid = t.adj[shared_slot];
            if (nb_tid != UINT32_MAX && surface_.triangles_[nb_tid].is_active) {
                const int nb_slot = surface_.triangles_[nb_tid].IndexOfNeighbor(tid);
                if (nb_slot >= 0) {
                    recorder.RecordTriangleBefore(nb_tid);
                    surface_.triangles_[nb_tid].SetEdgeConstrained(nb_slot, true);
                }
            }
            return OperationResult::Ok();
        }
    }

    if (start_fan_tri == UINT32_MAX) {
        return OperationResult::Fail(StatusCode::kInternalError, "无法定位特征线起点的关联三角形");
    }

    uint32_t first_crossed_tri = UINT32_MAX;
    int first_exit_slot = -1;

    for (uint32_t tid = 0; tid < surface_.triangles_.size(); ++tid) {
        const TinTriangle& t = surface_.triangles_[tid];
        if (!t.is_active) continue;
        const int idx_s = t.IndexOfVertex(start_vid);
        if (idx_s < 0) continue;

        const uint32_t v_left = t.v[(idx_s + 1) % 3];
        const uint32_t v_right = t.v[(idx_s + 2) % 3];
        const Vec3d& p_left = surface_.vertices_[v_left].Position();
        const Vec3d& p_right = surface_.vertices_[v_right].Position();

        const double o_left = ExactPredicates::Orient2D(p_start, p_end, p_left);
        const double o_right = ExactPredicates::Orient2D(p_start, p_end, p_right);

        if (std::abs(o_left) <= kCollinearEps &&
            (p_left - p_start).Dot(p_end - p_start) > 0.0 &&
            (p_left - p_end).Dot(p_start - p_end) > 0.0) {
            OperationResult r1 = InsertConstrainedSegmentInternal(
                start_vid, v_left, code_id, recorder, recursion_depth + 1);
            if (!r1.IsOk()) return r1;
            return InsertConstrainedSegmentInternal(
                v_left, end_vid, code_id, recorder, recursion_depth + 1);
        }

        if (o_left > 0.0 && o_right < 0.0) {
            first_crossed_tri = tid;
            first_exit_slot = idx_s;
            break;
        }
    }

    if (first_crossed_tri == UINT32_MAX) {
        return OperationResult::Fail(
            StatusCode::kTopologyDegeneracy,
            "特征线段跨越了三角网外部非凸空洞区域，无法穿透");
    }

    std::vector<uint32_t> crossed_tris;
    std::vector<uint32_t> upper_chain;
    std::vector<uint32_t> upper_outer_tris;
    std::vector<int> upper_outer_slots;
    std::vector<uint8_t> upper_outer_constrained;

    std::vector<uint32_t> lower_chain;
    std::vector<uint32_t> lower_outer_tris;
    std::vector<int> lower_outer_slots;
    std::vector<uint8_t> lower_outer_constrained;

    upper_chain.push_back(start_vid);
    lower_chain.push_back(start_vid);

    uint32_t cur_tid = first_crossed_tri;
    int exit_slot = first_exit_slot;

    {
        const TinTriangle& t0 = surface_.triangles_[cur_tid];
        const int idx_s = exit_slot;
        const uint32_t v_left = t0.v[(idx_s + 1) % 3];
        const uint32_t v_right = t0.v[(idx_s + 2) % 3];

        const int left_slot = (idx_s + 2) % 3;
        const uint32_t nb_left = t0.adj[left_slot];
        upper_outer_tris.push_back(nb_left);
        upper_outer_slots.push_back(
            (nb_left != UINT32_MAX) ? surface_.triangles_[nb_left].IndexOfNeighbor(cur_tid) : -1);
        upper_outer_constrained.push_back(t0.IsEdgeConstrained(left_slot) ? 1 : 0);
        upper_chain.push_back(v_left);

        const int right_slot = (idx_s + 1) % 3;
        const uint32_t nb_right = t0.adj[right_slot];
        lower_outer_tris.push_back(nb_right);
        lower_outer_slots.push_back(
            (nb_right != UINT32_MAX) ? surface_.triangles_[nb_right].IndexOfNeighbor(cur_tid) : -1);
        lower_outer_constrained.push_back(t0.IsEdgeConstrained(right_slot) ? 1 : 0);
        lower_chain.push_back(v_right);
    }

    crossed_tris.push_back(cur_tid);

    size_t walk_guard = 0;
    const size_t max_walk = surface_.triangles_.size() + 16;

    while (walk_guard++ < max_walk) {
        const TinTriangle& cur_t = surface_.triangles_[cur_tid];
        const uint32_t next_tid = cur_t.adj[exit_slot];

        if (cur_t.IsEdgeConstrained(exit_slot)) {
            const Vec3d& ea = surface_.vertices_[cur_t.v[(exit_slot + 1) % 3]].Position();
            const Vec3d& eb = surface_.vertices_[cur_t.v[(exit_slot + 2) % 3]].Position();
            const Vec3d inter_pt = ComputeSegmentIntersection2D(p_start, p_end, ea, eb);

            uint32_t split_vid = UINT32_MAX;
            OperationResult ins_res = InsertVertexInternal(
                inter_pt, 0, code_id, SurveyPoint::kFlagFeature, false, recorder, split_vid);
            if (!ins_res.IsOk()) return ins_res;

            OperationResult r1 = InsertConstrainedSegmentInternal(
                start_vid, split_vid, code_id, recorder, recursion_depth + 1);
            if (!r1.IsOk()) return r1;
            return InsertConstrainedSegmentInternal(
                split_vid, end_vid, code_id, recorder, recursion_depth + 1);
        }

        if (next_tid == UINT32_MAX || !surface_.triangles_[next_tid].is_active) {
            return OperationResult::Fail(
                StatusCode::kTopologyDegeneracy,
                "特征线段中途触及三角网外边界空洞，请确保特征线完全位于测区内部");
        }

        const TinTriangle& next_t = surface_.triangles_[next_tid];
        const int entry_slot = next_t.IndexOfNeighbor(cur_tid);
        if (entry_slot < 0) {
            return OperationResult::Fail(StatusCode::kInternalError, "特征线穿越行走时邻接关系断裂");
        }

        crossed_tris.push_back(next_tid);
        const uint32_t opp_v = next_t.v[entry_slot];

        if (opp_v == end_vid) {
            const int last_left_slot = (entry_slot + 1) % 3;
            const uint32_t nb_last_left = next_t.adj[last_left_slot];
            upper_outer_tris.push_back(nb_last_left);
            upper_outer_slots.push_back(
                (nb_last_left != UINT32_MAX) ? surface_.triangles_[nb_last_left].IndexOfNeighbor(next_tid) : -1);
            upper_outer_constrained.push_back(next_t.IsEdgeConstrained(last_left_slot) ? 1 : 0);
            upper_chain.push_back(end_vid);

            const int last_right_slot = (entry_slot + 2) % 3;
            const uint32_t nb_last_right = next_t.adj[last_right_slot];
            lower_outer_tris.push_back(nb_last_right);
            lower_outer_slots.push_back(
                (nb_last_right != UINT32_MAX) ? surface_.triangles_[nb_last_right].IndexOfNeighbor(next_tid) : -1);
            lower_outer_constrained.push_back(next_t.IsEdgeConstrained(last_right_slot) ? 1 : 0);
            lower_chain.push_back(end_vid);
            break;
        }

        const Vec3d& p_opp = surface_.vertices_[opp_v].Position();
        const double orient = ExactPredicates::Orient2D(p_start, p_end, p_opp);

        if (std::abs(orient) <= kCollinearEps) {
            OperationResult r1 = InsertConstrainedSegmentInternal(
                start_vid, opp_v, code_id, recorder, recursion_depth + 1);
            if (!r1.IsOk()) return r1;
            return InsertConstrainedSegmentInternal(
                opp_v, end_vid, code_id, recorder, recursion_depth + 1);
        }

        if (orient > 0.0) {
            const int left_edge_slot = (entry_slot + 1) % 3;
            const uint32_t nb_left = next_t.adj[left_edge_slot];
            upper_outer_tris.push_back(nb_left);
            upper_outer_slots.push_back(
                (nb_left != UINT32_MAX) ? surface_.triangles_[nb_left].IndexOfNeighbor(next_tid) : -1);
            upper_outer_constrained.push_back(next_t.IsEdgeConstrained(left_edge_slot) ? 1 : 0);
            upper_chain.push_back(opp_v);

            cur_tid = next_tid;
            exit_slot = (entry_slot + 2) % 3;
        } else {
            const int right_edge_slot = (entry_slot + 2) % 3;
            const uint32_t nb_right = next_t.adj[right_edge_slot];
            lower_outer_tris.push_back(nb_right);
            lower_outer_slots.push_back(
                (nb_right != UINT32_MAX) ? surface_.triangles_[nb_right].IndexOfNeighbor(next_tid) : -1);
            lower_outer_constrained.push_back(next_t.IsEdgeConstrained(right_edge_slot) ? 1 : 0);
            lower_chain.push_back(opp_v);

            cur_tid = next_tid;
            exit_slot = (entry_slot + 1) % 3;
        }
    }

    for (uint32_t tid : crossed_tris) {
        recorder.RecordTriangleBefore(tid);
        surface_.triangles_[tid].is_active = 0;
    }

    std::vector<uint32_t> upper_poly = upper_chain;
    upper_outer_tris.push_back(UINT32_MAX);
    upper_outer_slots.push_back(-1);
    upper_outer_constrained.push_back(1);

    std::vector<uint32_t> lower_poly(lower_chain.rbegin(), lower_chain.rend());
    std::vector<uint32_t> lower_poly_outer_tris(lower_outer_tris.rbegin(), lower_outer_tris.rend());
    std::vector<int> lower_poly_outer_slots(lower_outer_slots.rbegin(), lower_outer_slots.rend());
    std::vector<uint8_t> lower_poly_outer_constrained(lower_outer_constrained.rbegin(), lower_outer_constrained.rend());
    lower_poly_outer_tris.push_back(UINT32_MAX);
    lower_poly_outer_slots.push_back(-1);
    lower_poly_outer_constrained.push_back(1);

    uint32_t upper_base_tid = UINT32_MAX;
    int upper_base_slot = -1;
    TriangulateSimpleCavityPolygon(
        upper_poly, upper_outer_tris, upper_outer_slots, upper_outer_constrained,
        end_vid, start_vid, upper_base_tid, upper_base_slot, recorder);

    uint32_t lower_base_tid = UINT32_MAX;
    int lower_base_slot = -1;
    TriangulateSimpleCavityPolygon(
        lower_poly, lower_poly_outer_tris, lower_poly_outer_slots, lower_poly_outer_constrained,
        start_vid, end_vid, lower_base_tid, lower_base_slot, recorder);

    if (upper_base_tid != UINT32_MAX && upper_base_slot >= 0) {
        recorder.RecordTriangleBefore(upper_base_tid);
        surface_.triangles_[upper_base_tid].adj[upper_base_slot] = lower_base_tid;
        surface_.triangles_[upper_base_tid].SetEdgeConstrained(upper_base_slot, true);
    }
    if (lower_base_tid != UINT32_MAX && lower_base_slot >= 0) {
        recorder.RecordTriangleBefore(lower_base_tid);
        surface_.triangles_[lower_base_tid].adj[lower_base_slot] = upper_base_tid;
        surface_.triangles_[lower_base_tid].SetEdgeConstrained(lower_base_slot, true);
    }

    if (code_id != 0) {
        recorder.RecordVertexBefore(start_vid);
        recorder.RecordVertexBefore(end_vid);
        surface_.vertices_[start_vid].code_id = code_id;
        surface_.vertices_[end_vid].code_id = code_id;
    }

    return OperationResult::Ok();
}

OperationResult TinTopologyEditor::InsertBreakline(
    const std::vector<Vec3d>& polyline,
    uint16_t code_id,
    bool interpolate_z_from_tin) {
    if (!surface_.IsValid()) {
        return OperationResult::Fail(StatusCode::kEmptyDataset, "当前三角网为空，无法插入特征线");
    }
    if (polyline.size() < 2) {
        return OperationResult::Fail(StatusCode::kInvalidArgument, "特征折线至少需要包含 2 个顶点");
    }

    TransactionRecorder recorder(surface_, "嵌入约束特征线");

    std::vector<uint32_t> node_vids;
    node_vids.reserve(polyline.size());

    for (const Vec3d& raw_pt : polyline) {
        Vec3d pt = raw_pt;
        if (interpolate_z_from_tin) {
            double z_tin = 0.0;
            if (!surface_.QueryElevationAt(pt.x, pt.y, z_tin, nullptr)) {
                recorder.Rollback();
                return OperationResult::Fail(
                    StatusCode::kInvalidArgument,
                    "特征折线存在超出三角网边界的节点，无法内插高程");
            }
            pt.z = z_tin;
        }

        uint32_t vid = UINT32_MAX;
        OperationResult ins_res = InsertVertexInternal(
            pt, 0, code_id, SurveyPoint::kFlagFeature, true, recorder, vid);
        if (!ins_res.IsOk()) {
            recorder.Rollback();
            return ins_res;
        }
        if (node_vids.empty() || node_vids.back() != vid) {
            node_vids.push_back(vid);
        }
    }

    for (size_t i = 0; i + 1 < node_vids.size(); ++i) {
        OperationResult seg_res = InsertConstrainedSegmentInternal(
            node_vids[i], node_vids[i + 1], code_id, recorder, 0);
        if (!seg_res.IsOk()) {
            recorder.Rollback();
            return seg_res;
        }
    }

    TinEditTransaction tx = recorder.Finalize();
    surface_.RebuildSpatialTilesAfterEdit(tx.affected_bounds);
    PushTransaction(std::move(tx));
    return OperationResult::Ok();
}

OperationResult TinTopologyEditor::DeleteVertex(uint32_t vertex_id) {
    if (!surface_.IsValid()) {
        return OperationResult::Fail(StatusCode::kEmptyDataset, "当前三角网为空，无法删除顶点");
    }
    if (vertex_id >= surface_.vertices_.size() || !surface_.vertices_[vertex_id].IsActive()) {
        return OperationResult::Fail(StatusCode::kInvalidArgument, "待删除顶点编号无效或已删除");
    }

    std::vector<uint32_t> star_tris;
    for (uint32_t tid = 0; tid < surface_.triangles_.size(); ++tid) {
        const TinTriangle& t = surface_.triangles_[tid];
        if (!t.is_active) continue;
        const int slot = t.IndexOfVertex(vertex_id);
        if (slot >= 0) {
            if (t.IsEdgeConstrained((slot + 1) % 3) || t.IsEdgeConstrained((slot + 2) % 3)) {
                return OperationResult::Fail(
                    StatusCode::kTopologyDegeneracy,
                    "待删除顶点连接有已锁定的特征线/断裂线，请先解除特征线后再删除");
            }
            star_tris.push_back(tid);
        }
    }

    if (star_tris.size() < 3) {
        return OperationResult::Fail(
            StatusCode::kTopologyDegeneracy,
            "待删除顶点位于外边界或邻接三角形不足 3 个，无法构成封闭重剖分空腔");
    }

    std::unordered_set<uint32_t> star_set(star_tris.begin(), star_tris.end());
    std::vector<uint32_t> poly_verts;
    std::vector<uint32_t> outer_tris;
    std::vector<int> outer_slots;
    std::vector<uint8_t> outer_constrained;

    uint32_t cur_tid = star_tris[0];
    for (size_t step = 0; step < star_tris.size(); ++step) {
        const TinTriangle& t = surface_.triangles_[cur_tid];
        const int slot = t.IndexOfVertex(vertex_id);
        const uint32_t v_next = t.v[(slot + 1) % 3];
        const uint32_t nb_outer = t.adj[slot];

        poly_verts.push_back(v_next);
        outer_tris.push_back(nb_outer);
        outer_slots.push_back(
            (nb_outer != UINT32_MAX) ? surface_.triangles_[nb_outer].IndexOfNeighbor(cur_tid) : -1);
        outer_constrained.push_back(t.IsEdgeConstrained(slot) ? 1 : 0);

        const uint32_t next_tid = t.adj[(slot + 2) % 3];
        if (next_tid == UINT32_MAX || star_set.find(next_tid) == star_set.end()) {
            return OperationResult::Fail(
                StatusCode::kTopologyDegeneracy,
                "待删除顶点位于非封闭外边界上，禁止直接删除边界顶点");
        }
        cur_tid = next_tid;
    }

    if (cur_tid != star_tris[0] || poly_verts.size() != star_tris.size()) {
        return OperationResult::Fail(
            StatusCode::kTopologyDegeneracy,
            "待删除顶点邻域拓扑不封闭，无法安全重剖分");
    }

    TransactionRecorder recorder(surface_, "删除三角网顶点");
    recorder.RecordVertexBefore(vertex_id);
    surface_.vertices_[vertex_id].flags &= static_cast<uint16_t>(~TinVertex::kFlagActive);

    for (uint32_t tid : star_tris) {
        recorder.RecordTriangleBefore(tid);
        surface_.triangles_[tid].is_active = 0;
    }

    uint32_t dummy_tid = UINT32_MAX;
    int dummy_slot = -1;
    TriangulateSimpleCavityPolygon(
        poly_verts, outer_tris, outer_slots, outer_constrained,
        UINT32_MAX, UINT32_MAX, dummy_tid, dummy_slot, recorder);

    TinEditTransaction tx = recorder.Finalize();
    surface_.RebuildSpatialTilesAfterEdit(tx.affected_bounds);
    PushTransaction(std::move(tx));
    return OperationResult::Ok();
}

OperationResult TinTopologyEditor::ClipByPolygon(
    const std::vector<Vec3d>& boundary_polygon,
    bool keep_inside) {
    if (!surface_.IsValid()) {
        return OperationResult::Fail(StatusCode::kEmptyDataset, "当前三角网为空，无法执行边界裁剪");
    }
    if (boundary_polygon.size() < 3) {
        return OperationResult::Fail(StatusCode::kInvalidArgument, "裁剪多边形至少需要 3 个顶点");
    }

    std::vector<Vec3d> closed_ring = boundary_polygon;
    if (closed_ring.front().DistanceSquared2D(closed_ring.back()) > kVertexSnapEpsSq) {
        closed_ring.push_back(closed_ring.front());
    }

    TransactionRecorder recorder(surface_, keep_inside ? "多边形外边界裁剪" : "多边形内部挖空裁剪");

    std::vector<uint32_t> ring_vids;
    ring_vids.reserve(closed_ring.size());
    for (size_t i = 0; i + 1 < closed_ring.size(); ++i) {
        Vec3d pt = closed_ring[i];
        double z_tin = 0.0;
        if (!surface_.QueryElevationAt(pt.x, pt.y, z_tin, nullptr)) {
            recorder.Rollback();
            return OperationResult::Fail(
                StatusCode::kInvalidArgument,
                "裁剪多边形存在超出当前三角网覆盖范围的顶点");
        }
        pt.z = z_tin;

        uint32_t vid = UINT32_MAX;
        OperationResult res = InsertVertexInternal(
            pt, 0, 0, SurveyPoint::kFlagFeature, true, recorder, vid);
        if (!res.IsOk()) {
            recorder.Rollback();
            return res;
        }
        if (ring_vids.empty() || ring_vids.back() != vid) {
            ring_vids.push_back(vid);
        }
    }

    if (ring_vids.size() < 3) {
        recorder.Rollback();
        return OperationResult::Fail(StatusCode::kInvalidArgument, "去重后裁剪多边形顶点数不足 3 个");
    }

    for (size_t i = 0; i < ring_vids.size(); ++i) {
        const uint32_t v0 = ring_vids[i];
        const uint32_t v1 = ring_vids[(i + 1) % ring_vids.size()];
        OperationResult seg_res = InsertConstrainedSegmentInternal(v0, v1, 0, recorder, 0);
        if (!seg_res.IsOk()) {
            recorder.Rollback();
            return seg_res;
        }
    }

    for (uint32_t tid = 0; tid < surface_.triangles_.size(); ++tid) {
        TinTriangle& t = surface_.triangles_[tid];
        if (!t.is_active) continue;

        const Vec3d& p0 = surface_.vertices_[t.v[0]].Position();
        const Vec3d& p1 = surface_.vertices_[t.v[1]].Position();
        const Vec3d& p2 = surface_.vertices_[t.v[2]].Position();
        const double cx = (p0.x + p1.x + p2.x) / 3.0;
        const double cy = (p0.y + p1.y + p2.y) / 3.0;

        const bool inside = IsPointInPolygon2D(cx, cy, boundary_polygon);
        const bool should_remove = keep_inside ? (!inside) : inside;

        if (should_remove) {
            recorder.RecordTriangleBefore(tid);
            t.is_active = 0;
            for (int e = 0; e < 3; ++e) {
                const uint32_t nb = t.adj[e];
                if (nb != UINT32_MAX && surface_.triangles_[nb].is_active) {
                    const int nb_slot = surface_.triangles_[nb].IndexOfNeighbor(tid);
                    if (nb_slot >= 0) {
                        recorder.RecordTriangleBefore(nb);
                        surface_.triangles_[nb].adj[nb_slot] = UINT32_MAX;
                    }
                }
            }
        }
    }

    TinEditTransaction tx = recorder.Finalize();
    surface_.RebuildSpatialTilesAfterEdit(BoundingBox3d{});
    PushTransaction(std::move(tx));
    return OperationResult::Ok();
}

OperationResult TinTopologyEditor::Undo() {
    if (undo_stack_.empty()) {
        return OperationResult::Fail(StatusCode::kEmptyDataset, "当前没有可撤销的三角网编辑操作");
    }

    TinEditTransaction tx = std::move(undo_stack_.back());
    undo_stack_.pop_back();

    for (auto it = tx.triangle_deltas.rbegin(); it != tx.triangle_deltas.rend(); ++it) {
        if (it->triangle_id < surface_.triangles_.size()) {
            if (it->existed_before) {
                surface_.triangles_[it->triangle_id] = it->before_state;
            } else {
                surface_.triangles_[it->triangle_id].is_active = 0;
            }
        }
    }
    for (auto it = tx.vertex_deltas.rbegin(); it != tx.vertex_deltas.rend(); ++it) {
        if (it->vertex_id < surface_.vertices_.size() && it->existed_before) {
            surface_.vertices_[it->vertex_id] = it->before_state;
        }
    }

    surface_.RebuildSpatialTilesAfterEdit(tx.affected_bounds);
    redo_stack_.push_back(std::move(tx));
    return OperationResult::Ok();
}

OperationResult TinTopologyEditor::Redo() {
    if (redo_stack_.empty()) {
        return OperationResult::Fail(StatusCode::kEmptyDataset, "当前没有可重做的三角网编辑操作");
    }

    TinEditTransaction tx = std::move(redo_stack_.back());
    redo_stack_.pop_back();

    for (const auto& vd : tx.vertex_deltas) {
        if (vd.vertex_id >= surface_.vertices_.size()) {
            surface_.vertices_.resize(static_cast<size_t>(vd.vertex_id) + 1);
        }
        surface_.vertices_[vd.vertex_id] = vd.after_state;
    }
    for (const auto& td : tx.triangle_deltas) {
        if (td.triangle_id >= surface_.triangles_.size()) {
            surface_.triangles_.resize(static_cast<size_t>(td.triangle_id) + 1);
        }
        surface_.triangles_[td.triangle_id] = td.after_state;
    }

    surface_.RebuildSpatialTilesAfterEdit(tx.affected_bounds);
    undo_stack_.push_back(std::move(tx));
    return OperationResult::Ok();
}

}  // namespace geo_kernel

