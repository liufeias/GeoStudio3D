#pragma once

/**
 * @file point_octree.hpp
 * @brief 亿级点云并行线性八叉树 (Linear Octree) 与多级 LOD 视锥体调度器
 *
 * 核心架构与性能设计：
 *   1. 莫顿码物理重排（Morton Z-Order Physical Reordering）：
 *      构建阶段先对全部点云并行计算 64 位三维莫顿码，随后采用 O(N) 线性时间复杂度的
 *      并行 8-bit 基数排序（Parallel Radix Sort）重排点云数组。重排后空间相邻的点在
 *      物理内存中严格连续，八叉树叶子节点无需存储海量点索引指针，仅需保存 [start_idx, end_idx)
 *      区间偏移，1 亿个点的八叉树结构自身额外内存开销不足 40 MB。
 *   2. 特征优先的多级 LOD 抽稀（Feature-Preserving Bottom-Up LOD）：
 *      自底向上为每个内部节点生成代表性 LOD 点集，优先保留带有地形特征标志（如坎顶、
 *      坎脚、道路边线）的点云，其余点采用三维体素网格均匀抽稀，确保远景缩放时地形骨架不失真。
 *   3. 屏幕空间误差驱动的视锥体调度（Frustum Culling + SSE）：
 *      根据当前相机视锥体六平面与屏幕像素投影误差（Screen-Space Error），在 1 毫秒内
 *      从数万个八叉树节点中筛选出最优可视节点集合，并将单帧送入 GPU 的点数严格控制在预算内。
 */

#include <array>
#include <cstdint>
#include <vector>

#include "geo_kernel/core_types.hpp"

namespace geo_kernel {

/**
 * @brief 八叉树构建配置参数
 */
struct OctreeBuildOptions {
    uint32_t max_depth = 10;                    ///< 最大八叉树划分深度（默认 10 层，对应 1024^3 空间分辨率）
    uint32_t max_points_per_leaf = 16384;       ///< 叶子节点最大容纳点数（触发细分阈值）
    uint32_t max_lod_points_per_node = 8192;    ///< 内部节点存储的 LOD 抽稀点数上限
    size_t thread_count = 0;                    ///< 并行构建线程数（0 表示自动匹配 CPU 核心数）
};

/**
 * @brief 八叉树单节点元数据（紧凑缓存友好结构）
 */
struct OctreeNode {
    uint32_t node_id = 0;                       ///< 节点在 nodes_ 数组中的索引
    uint32_t parent_id = UINT32_MAX;            ///< 父节点索引（根节点为 UINT32_MAX）
    uint32_t children[8]{
        UINT32_MAX, UINT32_MAX, UINT32_MAX, UINT32_MAX,
        UINT32_MAX, UINT32_MAX, UINT32_MAX, UINT32_MAX
    };                                          ///< 8 个子节点索引（UINT32_MAX 表示空子象限）
    uint8_t depth = 0;                          ///< 节点所处深度（根节点为 0）
    bool is_leaf = true;                        ///< 是否为叶子节点

    BoundingBox3d cubic_bounds;                 ///< 节点严格立方体空间包围盒（用于八叉树八分划分）
    BoundingBox3d tight_bounds;                 ///< 节点内部实际点云的紧凑包围盒（用于精确视锥剔除）

    uint64_t point_start = 0;                   ///< 叶子节点在全局重排点云数组中的起始下标
    uint64_t point_count = 0;                   ///< 该节点覆盖的原始点云总数（含所有子树）
    std::vector<uint32_t> lod_point_indices;    ///< 内部节点专用：抽稀后的全局点云下标列表
};

/**
 * @brief 三维视锥体六平面与相机投影参数（用于 LOD 调度查询）
 */
struct CameraViewParams {
    Vec3d camera_local_pos{0.0, 0.0, 500.0};    ///< 相机在 RTC 局部坐标系下的三维位置
    /**
     * @brief 视锥体 6 个裁剪平面方程：ax + by + cz + d >= 0 表示位于视锥体内部
     *        顺序依次为：左(Left)、右(Right)、下(Bottom)、上(Top)、近(Near)、远(Far)
     *        注意：法向量 (a, b, c) 必须已单位化。
     */
    double frustum_planes[6][4]{
        { 1.0,  0.0,  0.0, 1e9},
        {-1.0,  0.0,  0.0, 1e9},
        { 0.0,  1.0,  0.0, 1e9},
        { 0.0, -1.0,  0.0, 1e9},
        { 0.0,  0.0,  1.0, 1e9},
        { 0.0,  0.0, -1.0, 1e9}
    };
    double viewport_height_px = 1080.0;         ///< 渲染视口高度（像素）
    double fov_y_radians = 0.7853981633974483;  ///< 垂直视场角（弧度，默认 45 度）
    double sse_threshold_px = 2.5;              ///< 屏幕空间误差细分阈值（像素，越小越精细）
    uint64_t max_point_budget = 8000000;        ///< 单帧最大渲染点数预算（默认 800 万点）
    bool enable_frustum_culling = true;         ///< 是否启用视锥体裁剪
};

/**
 * @brief 八叉树构建统计报告
 */
struct OctreeBuildStats {
    uint64_t total_points = 0;                  ///< 纳入索引的点云总数
    uint32_t total_nodes = 0;                   ///< 八叉树节点总数
    uint32_t leaf_nodes = 0;                    ///< 叶子节点数量
    uint32_t max_reached_depth = 0;             ///< 实际达到的最大树深度
    double elapsed_seconds = 0.0;               ///< 构建总耗时（秒）
};

/**
 * @brief 亿级点云空间八叉树与 LOD 渲染包生成器
 */
class GEO_API PointCloudOctree {
public:
    PointCloudOctree() = default;
    ~PointCloudOctree() = default;

    /**
     * @brief 基于已归化的点云数组构建多级 LOD 八叉树
     *
     * 注意：为实现极致内存局部性，本函数会对传入的 points 数组按三维莫顿码进行原地重排。
     *
     * @param points 输入/输出：RTC 局部坐标点云数组（构建完成后将按空间 Z-order 连续排列）
     * @param local_bounds 局部点云包围盒
     * @param options 八叉树构建参数
     * @param out_stats 输出构建统计报告
     * @return 操作结果状态
     */
    [[nodiscard]] OperationResult Build(
        std::vector<SurveyPoint>& points,
        const BoundingBox3d& local_bounds,
        const OctreeBuildOptions& options,
        OctreeBuildStats& out_stats);

    /**
     * @brief 清空八叉树释放内存
     */
    void Clear() noexcept;

    /**
     * @brief 判断八叉树是否已构建就绪
     */
    [[nodiscard]] bool IsBuilt() const noexcept { return !nodes_.empty(); }

    /**
     * @brief 获取只读节点数组
     */
    [[nodiscard]] const std::vector<OctreeNode>& Nodes() const noexcept { return nodes_; }

    /**
     * @brief 根据相机视锥体与屏幕空间误差（SSE）筛选当前帧需要渲染的八叉树节点列表
     *
     * @param view 相机位置、视锥体平面及渲染预算参数
     * @param out_visible_node_ids 输出：按由近及远排序的可视节点 ID 列表
     * @param out_estimated_points 输出：所选节点预计渲染的总点数
     */
    void QueryVisibleNodes(
        const CameraViewParams& view,
        std::vector<uint32_t>& out_visible_node_ids,
        uint64_t& out_estimated_points) const;

    /**
     * @brief 将指定八叉树节点打包为可直接上传 GPU 的二进制瓦片数据包
     *
     * 数据布局：[GpuTilePacketHeader (96B)] + [GpuPointVertex (16B) * vertex_count]
     *
     * @param node_id 目标八叉树节点 ID
     * @param points 全局重排后的点云数组（需与 Build 时传入的数组一致）
     * @param rtc RTC 全局参考原点信息（写入包头供前端大坐标还原）
     * @param out_packet 输出二进制字节流缓冲
     * @return 成功返回 kOk，节点不存在返回 kTileNotFound
     */
    [[nodiscard]] OperationResult SerializeNodeToGpuPacket(
        uint32_t node_id,
        const std::vector<SurveyPoint>& points,
        const RtcTransform& rtc,
        std::vector<uint8_t>& out_packet) const;

private:
    struct BuildContext;

    uint32_t BuildRecursive(
        BuildContext& ctx,
        uint32_t parent_id,
        uint8_t depth,
        const BoundingBox3d& cubic_box,
        uint64_t start_idx,
        uint64_t end_idx);

    void GenerateInternalNodeLod(
        BuildContext& ctx,
        OctreeNode& node);

    [[nodiscard]] bool IsBoxOutsideFrustum(
        const BoundingBox3d& box,
        const double frustum_planes[6][4]) const noexcept;

    std::vector<OctreeNode> nodes_;
    BoundingBox3d root_cubic_bounds_;
    OctreeBuildOptions options_;
};

}  // namespace geo_kernel

