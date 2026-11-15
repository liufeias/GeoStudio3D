#pragma once

/**
 * @file tiled_tin.hpp
 * @brief 空间分块邻接 Delaunay 三角网 (TIN) 构建器、射线拾取与脏瓦片增量更新引擎
 *
 * 第二阶段增强特性：
 *   1. 约束边位掩码支持（constrained_mask）：
 *      每个三角形的 3 条边均独立支持断裂线/特征线约束标记，后续交互编辑或局部重三角化时
 *      严格保护特征线不被非法翻转，确保坎顶、坎脚、道路边线拓扑绝对保真。
 *   2. 交互式三维射线拾取与平面最近要素查询：
 *      内置基于空间瓦片包围盒加速的 Möller–Trumbore 三维射线-三角形精确相交检测，
 *      返回命中三角形 ID、三维交点坐标、最近顶点 ID 及最近边槽位，支撑前端鼠标交互编辑。
 *   3. 动态拓扑编辑后的局部空间瓦片增量同步（RebuildSpatialTilesAfterEdit）：
 *      编辑操作修改三角形列表或顶点高程后，自动重算受影响瓦片的局部三角形列表与包围盒，
 *      递增瓦片 revision 并置位 is_dirty，前端仅需增量更新脏瓦片显存。
 *   4. 自定义缝合网格导入（BuildFromCustomMesh）：
 *      支持由多级放坡引擎或外部布尔裁剪生成的顶点与三角形直接构建完整的邻接与空间瓦片索引。
 */

#include <array>
#include <cstdint>
#include <functional>
#include <limits>
#include <unordered_map>
#include <vector>

#include "geo_kernel/core_types.hpp"

namespace geo_kernel {

// 前向声明友元拓扑编辑类，允许其直接执行零拷贝底层邻接表事务修改
class TinTopologyEditor;

/**
 * @brief 三角网构建配置选项
 */
struct TinBuildOptions {
    /**
     * @brief 构网前自适应网格抽稀步长（米）
     *        - <= 0.0 : 禁用抽稀，保留全部非重复原始点云构网
     *        - > 0.0  : 在每个 grid_cell_size x grid_cell_size 平面网格内保留高程极值点与特征点
     */
    double decimation_grid_size = 0.0;

    /**
     * @brief 空间渲染与编辑瓦片边长（米，默认 128.0m x 128.0m）
     */
    double tile_size = 128.0;

    /**
     * @brief 最大允许边长过滤阈值（米，<= 0.0 表示不裁剪外围狭长边缘三角形）
     */
    double max_edge_length = 0.0;

    /**
     * @brief 可选进度回调函数（返回 false 可中止构网）
     */
    std::function<bool(double progress, uint64_t processed_vertices)> progress_callback = nullptr;
};

/**
 * @brief 三角网顶点结构（局部双精度坐标 + 原始属性溯源）
 */
struct TinVertex {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    uint32_t source_point_id = 0;   ///< 关联原始测点编号
    uint16_t code_id = 0;           ///< 简码 ID
    uint16_t flags = 0x0001;        ///< 属性标志位（bit0=有效顶点）

    static constexpr uint16_t kFlagActive = 0x0001;

    [[nodiscard]] constexpr Vec3d Position() const noexcept { return {x, y, z}; }
    [[nodiscard]] constexpr bool IsActive() const noexcept { return (flags & kFlagActive) != 0; }
};

/**
 * @brief 拓扑三角形单元（严格 32 字节对齐，逆时针 CCW 顶点序 + O(1) 邻接关系 + 约束边标记）
 *
 * 约定规范：
 *   - 顶点 (v[0], v[1], v[2]) 严格满足 Orient2D(v[0], v[1], v[2]) > 0（逆时针）。
 *   - 邻接三角形 adj[i] 对应与第 i 个顶点相对的那条边：
 *       adj[0] 共享边 (v[1], v[2])，对应 constrained_mask 的 bit0 (0x01)
 *       adj[1] 共享边 (v[2], v[0])，对应 constrained_mask 的 bit1 (0x02)
 *       adj[2] 共享边 (v[0], v[1])，对应 constrained_mask 的 bit2 (0x04)
 *     若某条边位于三角网边界或已裁剪，对应 adj[i] 为 UINT32_MAX。
 */
struct TinTriangle {
    uint32_t v[3]{UINT32_MAX, UINT32_MAX, UINT32_MAX};
    uint32_t adj[3]{UINT32_MAX, UINT32_MAX, UINT32_MAX};
    uint32_t tile_index = 0;        ///< 所属空间瓦片在 tiles_ 数组中的下标
    uint16_t region_tag = 0;        ///< 区域标识（0=原地面，1=填方坡面，2=挖方坡面，3=马道平台，4=基底平台）
    uint8_t is_active = 1;          ///< 1=有效三角形，0=已删除/已裁剪
    uint8_t constrained_mask = 0;   ///< 约束特征边位掩码（bit0/1/2 分别对应边槽位 0/1/2）

    [[nodiscard]] constexpr int IndexOfVertex(uint32_t vid) const noexcept {
        if (v[0] == vid) return 0;
        if (v[1] == vid) return 1;
        if (v[2] == vid) return 2;
        return -1;
    }

    [[nodiscard]] constexpr int IndexOfNeighbor(uint32_t tid) const noexcept {
        if (adj[0] == tid) return 0;
        if (adj[1] == tid) return 1;
        if (adj[2] == tid) return 2;
        return -1;
    }

    [[nodiscard]] constexpr bool IsEdgeConstrained(int edge_slot) const noexcept {
        return (edge_slot >= 0 && edge_slot < 3) &&
               ((constrained_mask & (1u << edge_slot)) != 0);
    }

    constexpr void SetEdgeConstrained(int edge_slot, bool constrained) noexcept {
        if (edge_slot < 0 || edge_slot >= 3) return;
        const auto bit = static_cast<uint8_t>(1u << edge_slot);
        if (constrained) {
            constrained_mask |= bit;
        } else {
            constrained_mask &= static_cast<uint8_t>(~bit);
        }
    }
};
static_assert(sizeof(TinTriangle) == 32, "TinTriangle 必须保持 32 字节紧凑对齐");

/**
 * @brief 空间网格瓦片描述符（支持独立渲染更新与脏标记追踪）
 */
struct TinSpatialTile {
    uint32_t tile_index = 0;        ///< 瓦片数组下标
    int32_t tile_x = 0;             ///< 瓦片平面 X 网格索引
    int32_t tile_y = 0;             ///< 瓦片平面 Y 网格索引
    uint64_t tile_uid = 0;          ///< 全局唯一瓦片键值（由 tile_x 与 tile_y 打包）
    uint32_t revision = 1;          ///< 瓦片拓扑修改版本号（每次局部修改自增）
    bool is_dirty = true;           ///< 脏标记：为 true 时表示需要重新上传显存缓冲区
    BoundingBox3d bounds;           ///< 瓦片内所有三角形的精确三维包围盒
    std::vector<uint32_t> triangle_indices;  ///< 归属于本瓦片的有效三角形全局下标列表
};

/**
 * @brief 三维射线拾取结果结构体（用于交互式选择三角网顶点、边或面片）
 */
struct RayHitResult {
    bool hit = false;                                       ///< 是否命中有效三角形
    uint32_t triangle_id = UINT32_MAX;                      ///< 命中的三角形全局索引
    uint32_t nearest_vertex_id = UINT32_MAX;                ///< 距离交点最近的三角形顶点索引
    int nearest_edge_slot = -1;                             ///< 距离交点最近的边槽位 [0..2]
    double distance = (std::numeric_limits<double>::max)(); ///< 射线原点到交点的欧氏距离（米）
    Vec3d hit_point{0.0, 0.0, 0.0};                         ///< 交点在 RTC 局部坐标系下的三维坐标
};

/**
 * @brief 三角网构建统计报告
 */
struct TinBuildStats {
    uint64_t input_points = 0;          ///< 输入原始点云总数
    uint64_t deduplicated_points = 0;   ///< 平面坐标重复被合并的点数
    uint64_t decimated_points = 0;      ///< 网格抽稀掉的冗余点数
    uint64_t tin_vertices = 0;          ///< 最终纳入三角网的顶点数
    uint64_t tin_triangles = 0;         ///< 生成的有效三角形总数
    uint32_t total_tiles = 0;           ///< 划分的空间瓦片总数
    BoundingBox3d bounds;               ///< 三角网整体局部包围盒
    double elapsed_seconds = 0.0;       ///< 总耗时（秒）
};

/**
 * @brief 空间分块邻接 Delaunay 三角网核心引擎
 */
class GEO_API TiledTinSurface {
public:
    TiledTinSurface() = default;
    ~TiledTinSurface() = default;

    /**
     * @brief 从局部坐标点云数组构建空间分块 Delaunay 三角网
     */
    [[nodiscard]] OperationResult BuildFromPoints(
        const std::vector<SurveyPoint>& points,
        const TinBuildOptions& options,
        TinBuildStats& out_stats);

    /**
     * @brief 从外部已生成的顶点与三角形列表构建完整三角网（自动推算邻接关系与空间瓦片）
     *        主要用于多级放坡后与原地面缝合生成完整设计地形模型。
     */
    [[nodiscard]] OperationResult BuildFromCustomMesh(
        std::vector<TinVertex>&& vertices,
        std::vector<TinTriangle>&& triangles,
        double tile_size = 128.0);

    /**
     * @brief 清空三角网数据释放内存
     */
    void Clear() noexcept;

    /**
     * @brief 当前三角网是否包含有效三角形
     */
    [[nodiscard]] bool IsValid() const noexcept { return !triangles_.empty() && active_triangle_count_ > 0; }

    [[nodiscard]] const std::vector<TinVertex>& Vertices() const noexcept { return vertices_; }
    [[nodiscard]] const std::vector<TinTriangle>& Triangles() const noexcept { return triangles_; }
    [[nodiscard]] const std::vector<TinSpatialTile>& Tiles() const noexcept { return tiles_; }
    [[nodiscard]] uint64_t ActiveTriangleCount() const noexcept { return active_triangle_count_; }
    [[nodiscard]] const BoundingBox3d& Bounds() const noexcept { return mesh_bounds_; }
    [[nodiscard]] double TileSize() const noexcept { return lookup_tile_size_; }

    /**
     * @brief 查询指定平面坐标 (local_x, local_y) 在三角网上的精确线性内插高程 Z
     */
    [[nodiscard]] bool QueryElevationAt(
        double local_x,
        double local_y,
        double& out_z,
        uint32_t* out_triangle_id = nullptr) const noexcept;

    /**
     * @brief 执行三维射线与三角网的精确相交拾取（支持前端视口鼠标点击拾取顶点/边/三角形）
     * @param ray_origin 射线起点（RTC 局部坐标系）
     * @param ray_dir 射线方向向量（无需预先单位化，内部自动处理）
     */
    [[nodiscard]] RayHitResult PickByRay(const Vec3d& ray_origin, const Vec3d& ray_dir) const noexcept;

    /**
     * @brief 在给定二维搜索半径内查找距离 (local_x, local_y) 最近的活跃顶点
     */
    [[nodiscard]] bool FindNearestVertex2D(
        double local_x,
        double local_y,
        double max_radius,
        uint32_t& out_vertex_id) const noexcept;

    /**
     * @brief 在给定二维搜索距离内查找距离 (local_x, local_y) 最近的活跃三角形边
     */
    [[nodiscard]] bool FindNearestEdge2D(
        double local_x,
        double local_y,
        double max_distance,
        uint32_t& out_triangle_id,
        int& out_edge_slot) const noexcept;

    /**
     * @brief 获取所有当前处于脏状态（需要前端重新拉取 GPU 网格）的瓦片下标列表
     */
    [[nodiscard]] std::vector<uint32_t> GetDirtyTileIndices() const;

    /**
     * @brief 清除指定瓦片的脏标记（前端完成显存上传后调用）
     */
    void ClearTileDirty(uint32_t tile_index) noexcept;

    /**
     * @brief 标记与给定二维包围盒相交的所有空间瓦片为脏（局部拓扑编辑后调用）
     */
    void MarkTilesDirtyInBox2D(double min_x, double min_y, double max_x, double max_y) noexcept;

    /**
     * @brief 拓扑编辑（增删三角形、修改高程、裁剪等）后，增量同步空间瓦片三角形归属与二维查找网格
     * @param dirty_box 发生修改的二维包围盒范围（传入无效包围盒则全量同步所有瓦片）
     */
    void RebuildSpatialTilesAfterEdit(const BoundingBox3d& dirty_box);

    /**
     * @brief 将指定空间瓦片序列化为可直接上传 GPU 的三角形网格二进制包
     */
    [[nodiscard]] OperationResult SerializeTileToGpuPacket(
        uint32_t tile_index,
        const RtcTransform& rtc,
        std::vector<uint8_t>& out_packet) const;

private:
    friend class TinTopologyEditor;

    static uint64_t PackTileUid(int32_t tx, int32_t ty) noexcept {
        return (static_cast<uint64_t>(static_cast<uint32_t>(tx)) << 32) |
               static_cast<uint64_t>(static_cast<uint32_t>(ty));
    }

    void PrepareVertices(
        const std::vector<SurveyPoint>& points,
        const TinBuildOptions& options,
        TinBuildStats& stats);

    [[nodiscard]] OperationResult RunBowyerWatsonDelaunay(const TinBuildOptions& options);

    void BuildSpatialTilesAndLookupGrid(double tile_size);

    void RecomputeAllTriangleAdjacency();

    std::vector<TinVertex> vertices_;
    std::vector<TinTriangle> triangles_;
    uint64_t active_triangle_count_ = 0;

    std::vector<TinSpatialTile> tiles_;
    std::unordered_map<uint64_t, uint32_t> tile_uid_to_index_;

    // 二维空间查询加速网格（用于毫秒级点高程内插与射线拾取定位）
    double lookup_tile_size_ = 128.0;
    BoundingBox3d mesh_bounds_;
    std::unordered_map<uint64_t, std::vector<uint32_t>> tile_overlapping_triangles_;
};

}  // namespace geo_kernel

