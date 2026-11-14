#pragma once

/**
 * @file tin_editor.hpp
 * @brief 交互式三角网动态拓扑编辑与可逆增量事务 (Undo/Redo) 引擎
 *
 * 支持的企业级交互式三角网编辑能力：
 *   1. 对角线边翻转（FlipEdge）：
 *      自动检验共享边所属四边形是否严格凸（防止产生重叠翻转三角形），默认保护断裂线
 *      约束边不被误翻转（支持 force_flip 强制覆写），实时双向同步 4 条外围邻接指针。
 *   2. 强制特征线/断裂线嵌入（InsertBreakline - Constrained Delaunay Edge Recovery）：
 *      支持任意多段折线（坎顶线、坎脚线、道路边线、水沟中心线）。自动处理端点插入、
 *      共线中间点分段、穿越三角形带提取、交叉已有断裂线自动求交生成节点（PSLG）、
 *      上下侧多边形耳切三角化与约束局部 Lawson 优化，并将整条折线锁定为不可随意翻转的约束边。
 *   3. 局部增删改顶点：
 *      - InsertVertex：自动定位三角形内部或共享边，剖分为 3 或 4 个新三角形并执行局部 Lawson 合法化。
 *      - ModifyVertexElevation：修改单个测点高程并刷新关联瓦片法线与包围盒。
 *      - DeleteVertex：自动提取顶点 1-邻域星形多边形空腔并执行局部 Delaunay 重剖分。
 *   4. 测区多边形边界裁剪（ClipByPolygon）：
 *      先将裁剪多边形作为强制特征线嵌入三角网，再精确剔除多边形外部（或内部）三角形，
 *      保证裁剪边界呈平滑直线而非锯齿状。
 *   5. 轻量级增量撤销/重做（Delta-Based Undo/Redo）：
 *      仅保存每次编辑操作前后发生变化的局部顶点与三角形差异快照（非全网拷贝），
 *      即使上千万三角形规模也能实现毫秒级撤销与重做，并自动标记脏瓦片通知 GPU 局部刷新。
 */

#include <cstdint>
#include <string>
#include <vector>

#include "geo_kernel/core_types.hpp"
#include "geo_kernel/tiled_tin.hpp"

namespace geo_kernel {

/**
 * @brief 单个顶点状态快照（用于 Undo/Redo 增量恢复）
 */
struct VertexDelta {
    uint32_t vertex_id = UINT32_MAX;
    bool existed_before = false;
    TinVertex before_state{};
    TinVertex after_state{};
};

/**
 * @brief 单个三角形状态快照（用于 Undo/Redo 增量恢复）
 */
struct TriangleDelta {
    uint32_t triangle_id = UINT32_MAX;
    bool existed_before = false;
    TinTriangle before_state{};
    TinTriangle after_state{};
};

/**
 * @brief 一次原子编辑操作的可逆事务记录
 */
struct TinEditTransaction {
    std::string description;                        ///< 操作中文名称（如 "边翻转"、"插入特征线"）
    BoundingBox3d affected_bounds;                  ///< 本次修改覆盖的二维包围盒
    std::vector<VertexDelta> vertex_deltas;         ///< 发生增删改的顶点差分列表
    std::vector<TriangleDelta> triangle_deltas;     ///< 发生增删改的三角形差分列表
};

/**
 * @brief 三角网动态拓扑编辑控制器（绑定到一个 TiledTinSurface 实例）
 */
class GEO_API TinTopologyEditor {
public:
    explicit TinTopologyEditor(TiledTinSurface& surface, size_t max_undo_levels = 64) noexcept;
    ~TinTopologyEditor() = default;

    /**
     * @brief 翻转指定三角形与对应边槽位邻居之间的共享对角线
     *
     * @param tri_id 目标三角形索引
     * @param edge_slot 边槽位 [0..2]（0=边(v1,v2), 1=边(v2,v0), 2=边(v0,v1)）
     * @param force_flip 是否允许强制翻转已标记为断裂线的约束边（默认 false）
     * @return 成功返回 kOk；若为边界边、约束边或非凸四边形返回 kTopologyDegeneracy
     */
    [[nodiscard]] OperationResult FlipEdge(
        uint32_t tri_id,
        int edge_slot,
        bool force_flip = false);

    /**
     * @brief 修改指定顶点的高程 Z
     */
    [[nodiscard]] OperationResult ModifyVertexElevation(
        uint32_t vertex_id,
        double new_local_z);

    /**
     * @brief 向三角网中插入一个新顶点并局部恢复 Delaunay 性质
     *
     * @param local_pt 新顶点 RTC 局部坐标
     * @param source_pt_id 关联原始测点编号
     * @param code_id 简码 ID
     * @param flags 顶点属性标志
     * @param legalize_delaunay 是否对周围非约束边执行 Lawson 合法化翻转（默认 true）
     * @param out_vertex_id 可选输出新插入（或已重合复用）的顶点索引
     */
    [[nodiscard]] OperationResult InsertVertex(
        const Vec3d& local_pt,
        uint32_t source_pt_id = 0,
        uint16_t code_id = 0,
        uint16_t flags = SurveyPoint::kFlagValid,
        bool legalize_delaunay = true,
        uint32_t* out_vertex_id = nullptr);

    /**
     * @brief 插入强制特征线/断裂线（Constrained Delaunay Breakline Insertion）
     *
     * @param polyline 折线顶点序列（至少包含 2 个点，局部坐标系）
     * @param code_id 特征线简码 ID
     * @param interpolate_z_from_tin 若为 true，则忽略折线自带的 Z 值，直接从当前 TIN 表面内插高程
     */
    [[nodiscard]] OperationResult InsertBreakline(
        const std::vector<Vec3d>& polyline,
        uint16_t code_id = 0,
        bool interpolate_z_from_tin = false);

    /**
     * @brief 删除指定内部顶点并对其 1-邻域星形多边形空腔执行局部重三角化
     */
    [[nodiscard]] OperationResult DeleteVertex(uint32_t vertex_id);

    /**
     * @brief 按给定二维封闭多边形边界裁剪三角网
     *
     * @param boundary_polygon 多边形顶点序列（首尾无需重复闭合，至少 3 个顶点）
     * @param keep_inside true = 保留多边形内部三角形（裁掉外部）；false = 挖除多边形内部（如建筑物基坑挖空）
     */
    [[nodiscard]] OperationResult ClipByPolygon(
        const std::vector<Vec3d>& boundary_polygon,
        bool keep_inside = true);

    /**
     * @brief 撤销最近一次拓扑编辑操作
     */
    [[nodiscard]] OperationResult Undo();

    /**
     * @brief 重做最近一次被撤销的拓扑编辑操作
     */
    [[nodiscard]] OperationResult Redo();

    /**
     * @brief 清空撤销与重做历史栈
     */
    void ClearHistory() noexcept;

    [[nodiscard]] bool CanUndo() const noexcept { return !undo_stack_.empty(); }
    [[nodiscard]] bool CanRedo() const noexcept { return !redo_stack_.empty(); }
    [[nodiscard]] size_t UndoCount() const noexcept { return undo_stack_.size(); }
    [[nodiscard]] size_t RedoCount() const noexcept { return redo_stack_.size(); }

private:
    class TransactionRecorder;

    [[nodiscard]] OperationResult InsertVertexInternal(
        const Vec3d& local_pt,
        uint32_t source_pt_id,
        uint16_t code_id,
        uint16_t flags,
        bool legalize_delaunay,
        TransactionRecorder& recorder,
        uint32_t& out_vertex_id);

    void LegalizeEdgesInternal(
        std::vector<std::pair<uint32_t, int>>& edge_stack,
        TransactionRecorder& recorder);

    [[nodiscard]] OperationResult InsertConstrainedSegmentInternal(
        uint32_t start_vid,
        uint32_t end_vid,
        uint16_t code_id,
        TransactionRecorder& recorder,
        int recursion_depth);

    void TriangulateSimpleCavityPolygon(
        const std::vector<uint32_t>& poly_verts,
        const std::vector<uint32_t>& outer_tris,
        const std::vector<int>& outer_slots,
        const std::vector<uint8_t>& outer_constrained,
        uint32_t constrained_base_v0,
        uint32_t constrained_base_v1,
        uint32_t& out_base_tri_id,
        int& out_base_edge_slot,
        TransactionRecorder& recorder);

    void PushTransaction(TinEditTransaction&& tx);

    TiledTinSurface& surface_;
    size_t max_undo_levels_ = 64;
    std::vector<TinEditTransaction> undo_stack_;
    std::vector<TinEditTransaction> redo_stack_;
};

}  // namespace geo_kernel

