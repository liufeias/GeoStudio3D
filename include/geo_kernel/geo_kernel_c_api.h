#ifndef GEO_KERNEL_C_API_H_
#define GEO_KERNEL_C_API_H_

/**
 * @file geo_kernel_c_api.h
 * @brief 空间几何内核跨语言扁平 C ABI 接口契约 (Flat C ABI, v2.0.0)
 *
 * 完整覆盖第一阶段与第二阶段全部核心能力：
 *   1. 亿级 .dat 点云内存映射并发解析与 RTC 大坐标自动归化
 *   2. 并行莫顿码多级 LOD 点云八叉树构建、视锥体/SSE 调度与 GPU 二进制打包
 *   3. 空间分块邻接 Delaunay 三角网构建、高程精确内插、脏瓦片追踪与 GPU 网格导出
 *   4. 三角网交互式动态拓扑编辑（三维射线拾取、边翻转、强制特征线嵌入、顶点增删改、
 *      多边形边界裁剪、多步增量撤销 Undo / 重做 Redo）
 *   5. 两期三角网高精度三棱柱叠加土方算量与实时网格微分热力图生成
 *   6. 向内/向外多级放坡建模、坡脚线/开挖边线提取（Daylighting）、原地面无缝缝合及放坡土方算量
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#if defined(GEO_KERNEL_STATIC)
    #define GEO_C_API
#elif defined(_WIN32) || defined(__CYGWIN__)
    #if defined(GEO_KERNEL_EXPORTS)
        #define GEO_C_API __declspec(dllexport)
    #else
        #define GEO_C_API __declspec(dllimport)
    #endif
#else
    #if __GNUC__ >= 4
        #define GEO_C_API __attribute__((visibility("default")))
    #else
        #define GEO_C_API
    #endif
#endif

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 不透明工程会话句柄（一个句柄对应一个独立工程上下文，支持多工程并发隔离）
 */
typedef struct GeoProjectOpaque* GeoProjectHandle;

/**
 * @brief 内核状态返回码（与 geo_kernel::StatusCode 一一对应）
 */
typedef enum GeoStatus {
    GEO_STATUS_OK = 0,
    GEO_STATUS_INVALID_ARGUMENT = 1,
    GEO_STATUS_FILE_NOT_FOUND = 2,
    GEO_STATUS_FILE_OPEN_FAILED = 3,
    GEO_STATUS_EMPTY_DATASET = 4,
    GEO_STATUS_PARSE_FORMAT_ERROR = 5,
    GEO_STATUS_OUT_OF_MEMORY = 6,
    GEO_STATUS_TOPOLOGY_DEGENERACY = 7,
    GEO_STATUS_CANCELLED = 8,
    GEO_STATUS_TILE_NOT_FOUND = 9,
    GEO_STATUS_INTERNAL_ERROR = 99
} GeoStatus;

/**
 * @brief 原始文件坐标轴向排列规范
 */
typedef enum GeoCoordOrder {
    GEO_COORD_SURVEYING_YXZ = 0,  /**< 测绘规范：第3列 Y(东->X), 第4列 X(北->Y), 第5列 H(高程->Z) */
    GEO_COORD_CARTESIAN_XYZ = 1   /**< 笛卡尔规范：依次为 X, Y, Z */
} GeoCoordOrder;

/**
 * @brief 地形模型期数与派生设计面槽位（支持多表面同屏叠加对比与独立 GPU 瓦片拉取）
 */
typedef enum GeoSurfaceSlot {
    GEO_SURFACE_PERIOD_1_ORIGINAL = 0,      /**< 第一期：原始地面点云与三角网 */
    GEO_SURFACE_PERIOD_2_TARGET = 1,        /**< 第二期：竣工/目标/开挖后点云与三角网 */
    GEO_SURFACE_GRADING_DESIGN_ONLY = 2,    /**< 多级放坡纯设计面三角网（基底+坡面+马道） */
    GEO_SURFACE_GRADING_MERGED_TERRAIN = 3  /**< 多级放坡与原地面沿坡脚线缝合后的完整地形网 */
} GeoSurfaceSlot;

/**
 * @brief C ABI 进度与取消回调函数指针
 */
typedef bool (*GeoProgressCallback)(double progress, uint64_t processed_items, void* user_data);

/**
 * @brief 点云导入解析参数结构体
 */
typedef struct GeoDatLoadParams {
    int32_t coord_order;            /**< 见 GeoCoordOrder */
    uint32_t thread_count;          /**< 工作线程数（0 表示自动匹配硬件核心数） */
    bool enable_elevation_filter;   /**< 是否启用高程飞点过滤 */
    double min_valid_elevation;     /**< 有效高程下限（米） */
    double max_valid_elevation;     /**< 有效高程上限（米） */
    bool filter_zero_coordinates;   /**< 是否过滤 (0, 0) 无效测点 */
} GeoDatLoadParams;

/**
 * @brief 点云导入统计与工程坐标范围摘要
 */
typedef struct GeoDatSummary {
    uint64_t file_bytes;
    uint64_t total_lines_scanned;
    uint64_t valid_points;
    uint64_t skipped_lines;
    uint64_t malformed_lines;
    uint64_t filtered_outliers;
    double elapsed_seconds;
    double throughput_mpts_per_sec;
    double rtc_origin[3];           /**< 全局大地参考原点 (X, Y, Z) */
    double global_min[3];           /**< 全局大地包围盒最小值 */
    double global_max[3];           /**< 全局大地包围盒最大值 */
    double local_min[3];            /**< 相对 RTC 原点的局部包围盒最小值 */
    double local_max[3];            /**< 相对 RTC 原点的局部包围盒最大值 */
} GeoDatSummary;

/**
 * @brief 八叉树构建参数
 */
typedef struct GeoOctreeParams {
    uint32_t max_depth;
    uint32_t max_points_per_leaf;
    uint32_t max_lod_points_per_node;
    uint32_t thread_count;
} GeoOctreeParams;

/**
 * @brief 八叉树构建摘要
 */
typedef struct GeoOctreeSummary {
    uint64_t total_points;
    uint32_t total_nodes;
    uint32_t leaf_nodes;
    uint32_t max_reached_depth;
    double elapsed_seconds;
} GeoOctreeSummary;

/**
 * @brief 三角网构建参数
 */
typedef struct GeoTinParams {
    double decimation_grid_size;    /**< 构网前特征保留抽稀网格边长（米，0 表示不抽稀） */
    double tile_size;               /**< 空间渲染与编辑瓦片边长（米，默认 128.0） */
    double max_edge_length;         /**< 最大允许边长裁剪阈值（米，0 表示不裁剪） */
} GeoTinParams;

/**
 * @brief 三角网构建摘要
 */
typedef struct GeoTinSummary {
    uint64_t input_points;
    uint64_t deduplicated_points;
    uint64_t decimated_points;
    uint64_t tin_vertices;
    uint64_t tin_triangles;
    uint32_t total_tiles;
    double elapsed_seconds;
} GeoTinSummary;

/**
 * @brief 视锥体 LOD 查询相机参数
 */
typedef struct GeoCameraQuery {
    double camera_local_pos[3];     /**< 相机在 RTC 局部坐标系下的坐标 */
    double frustum_planes[6][4];    /**< 6 个单位化视锥体裁剪平面 (a, b, c, d) */
    double viewport_height_px;      /**< 视口高度（像素） */
    double fov_y_radians;           /**< 垂直视场角（弧度） */
    double sse_threshold_px;        /**< 屏幕空间误差阈值（像素） */
    uint64_t max_point_budget;      /**< 单帧渲染点数预算上限 */
    bool enable_frustum_culling;    /**< 是否启用视锥体裁剪 */
} GeoCameraQuery;

/**
 * @brief 三维射线拾取结果（供前端鼠标点击三角网顶点/边/面片）
 */
typedef struct GeoRayHitResult {
    bool hit;                       /**< 是否命中三角形 */
    uint32_t triangle_id;           /**< 命中三角形索引 */
    uint32_t nearest_vertex_id;     /**< 最近顶点索引 */
    int32_t nearest_edge_slot;      /**< 最近边槽位 [0..2] */
    double distance;                /**< 射线起点到交点距离（米） */
    double hit_local_pos[3];        /**< 交点局部三维坐标 */
} GeoRayHitResult;

/**
 * @brief 两期土方算量报告结构体
 */
typedef struct GeoEarthworkSummary {
    double cut_volume_m3;           /**< 总挖方体积（立方米） */
    double fill_volume_m3;          /**< 总填方体积（立方米） */
    double net_volume_m3;           /**< 净土方量 = 填方 - 挖方（立方米） */
    double total_overlap_area_m2;   /**< 重叠总面积（平方米） */
    double cut_area_m2;             /**< 挖方区面积（平方米） */
    double fill_area_m2;            /**< 填方区面积（平方米） */
    double zero_area_m2;            /**< 零填挖区面积（平方米） */
    double max_cut_depth_m;         /**< 最大挖深（米） */
    double max_fill_height_m;       /**< 最大填高（米） */
    uint64_t intersected_prisms;    /**< 参与积分的微元三棱柱或网格数 */
    double elapsed_seconds;         /**< 计算总耗时（秒） */
} GeoEarthworkSummary;

/**
 * @brief 单级放坡参数结构体（C ABI 兼容）
 */
typedef struct GeoSlopeStageSpec {
    double stage_height;            /**< 本级边坡最大垂直高度（米） */
    double slope_ratio;             /**< 坡比 1:m 中的 m 值 */
    double berm_width;              /**< 马道平台宽度（米） */
    double berm_slope_ratio;        /**< 马道排水坡度 */
} GeoSlopeStageSpec;

/**
 * @brief 多级放坡整体参数结构体
 */
typedef struct GeoGradingParams {
    int32_t direction;              /**< 0 = 向外放坡 (Outward), 1 = 向内放坡 (Inward) */
    int32_t vertical_mode;          /**< 0 = 自动挖填, -1 = 强制向下放坡, 1 = 强制向上放坡 */
    bool use_uniform_base_elevation;/**< 是否统一基准高程 */
    double base_elevation;          /**< 统一基准高程（局部 Z，米） */
    bool auto_daylight_to_terrain;  /**< 是否自动与第一期原地面求交（坡脚线/开挖线） */
    bool cap_inward_bottom;         /**< 向内放坡时是否自动封底 */
    double perimeter_sample_step;   /**< 轮廓加密步长（米，默认 1.5m） */
    double output_tile_size;        /**< 输出网格瓦片大小（米，默认 128.0m） */
} GeoGradingParams;

/**
 * @brief 多级放坡工程计算摘要报告
 */
typedef struct GeoGradingSummary {
    GeoEarthworkSummary earthwork;  /**< 放坡挖填土方量报告 */
    double daylight_perimeter_m;    /**< 坡脚线/开挖边线总周长（米） */
    double slope_surface_area_m2;   /**< 边坡斜面展开总面积（平方米） */
    double berm_surface_area_m2;    /**< 马道平台总面积（平方米） */
    double base_pad_area_m2;        /**< 基准平台/坑底面积（平方米） */
    uint32_t actual_stages_used;    /**< 实际使用的最大级数 */
    uint32_t daylight_vertex_count; /**< 坡脚线闭合折线顶点数 */
    bool daylight_converged;        /**< 是否全部轮廓与原地面成功相交闭合 */
    double elapsed_seconds;         /**< 计算总耗时（秒） */
} GeoGradingSummary;

// ==============================================================================
// 1. 生命周期与错误诊断接口
// ==============================================================================

GEO_C_API const char* geo_get_version(void);
GEO_C_API const char* geo_get_last_error(void);
GEO_C_API GeoProjectHandle geo_project_create(void);
GEO_C_API void geo_project_destroy(GeoProjectHandle handle);

// ==============================================================================
// 2. 亿级 .dat 点云导入与多级 LOD 八叉树接口
// ==============================================================================

GEO_C_API GeoDatLoadParams geo_default_dat_load_params(void);

GEO_C_API GeoStatus geo_project_load_dat(
    GeoProjectHandle handle,
    int32_t surface_slot,
    const char* utf8_file_path,
    const GeoDatLoadParams* params,
    GeoProgressCallback progress_cb,
    void* user_data,
    GeoDatSummary* out_summary);

GEO_C_API GeoOctreeParams geo_default_octree_params(void);

GEO_C_API GeoStatus geo_project_build_octree(
    GeoProjectHandle handle,
    int32_t surface_slot,
    const GeoOctreeParams* params,
    GeoOctreeSummary* out_summary);

GEO_C_API GeoStatus geo_project_query_visible_octree_nodes(
    GeoProjectHandle handle,
    int32_t surface_slot,
    const GeoCameraQuery* query,
    uint32_t* out_node_ids,
    uint32_t capacity,
    uint32_t* out_node_count,
    uint64_t* out_estimated_points);

GEO_C_API GeoStatus geo_project_fetch_octree_node_gpu_packet(
    GeoProjectHandle handle,
    int32_t surface_slot,
    uint32_t node_id,
    uint8_t* out_buffer,
    size_t buffer_capacity,
    size_t* out_byte_size);

// ==============================================================================
// 3. 空间分块三角网 (TIN) 构建、高程查询与 GPU 网格包接口
// ==============================================================================

GEO_C_API GeoTinParams geo_default_tin_params(void);

GEO_C_API GeoStatus geo_project_build_tin(
    GeoProjectHandle handle,
    int32_t surface_slot,
    const GeoTinParams* params,
    GeoProgressCallback progress_cb,
    void* user_data,
    GeoTinSummary* out_summary);

GEO_C_API GeoStatus geo_project_query_elevation(
    GeoProjectHandle handle,
    int32_t surface_slot,
    double local_x,
    double local_y,
    double* out_local_z,
    double* out_global_z);

GEO_C_API GeoStatus geo_project_get_dirty_tin_tiles(
    GeoProjectHandle handle,
    int32_t surface_slot,
    uint32_t* out_tile_indices,
    uint32_t capacity,
    uint32_t* out_tile_count);

GEO_C_API GeoStatus geo_project_fetch_tin_tile_gpu_packet(
    GeoProjectHandle handle,
    int32_t surface_slot,
    uint32_t tile_index,
    bool clear_dirty_flag,
    uint8_t* out_buffer,
    size_t buffer_capacity,
    size_t* out_byte_size);

// ==============================================================================
// 4. 三角网交互式动态拓扑编辑与 Undo/Redo 接口（第二阶段新增）
// ==============================================================================

/**
 * @brief 三维射线拾取三角网顶点、边与面片
 */
GEO_C_API GeoStatus geo_project_pick_tin_ray(
    GeoProjectHandle handle,
    int32_t surface_slot,
    const double ray_origin[3],
    const double ray_dir[3],
    GeoRayHitResult* out_hit);

/**
 * @brief 翻转三角网对角线边（受断裂线约束保护，force_flip 为 true 时可强制翻转）
 */
GEO_C_API GeoStatus geo_project_tin_flip_edge(
    GeoProjectHandle handle,
    int32_t surface_slot,
    uint32_t triangle_id,
    int32_t edge_slot,
    bool force_flip);

/**
 * @brief 修改指定三角网顶点的局部高程 Z
 */
GEO_C_API GeoStatus geo_project_tin_modify_vertex_z(
    GeoProjectHandle handle,
    int32_t surface_slot,
    uint32_t vertex_id,
    double new_local_z);

/**
 * @brief 向三角网插入新顶点并局部恢复 Delaunay 性质
 */
GEO_C_API GeoStatus geo_project_tin_insert_vertex(
    GeoProjectHandle handle,
    int32_t surface_slot,
    const double local_xyz[3],
    uint16_t code_id,
    bool legalize_delaunay,
    uint32_t* out_vertex_id);

/**
 * @brief 插入强制特征线/断裂线（xyz_array 为连续的 [x0,y0,z0, x1,y1,z1, ...] 双精度数组）
 */
GEO_C_API GeoStatus geo_project_tin_insert_breakline(
    GeoProjectHandle handle,
    int32_t surface_slot,
    const double* xyz_array,
    uint32_t point_count,
    uint16_t code_id,
    bool interpolate_z_from_tin);

/**
 * @brief 删除指定三角网内部顶点并重剖分星形空腔
 */
GEO_C_API GeoStatus geo_project_tin_delete_vertex(
    GeoProjectHandle handle,
    int32_t surface_slot,
    uint32_t vertex_id);

/**
 * @brief 按封闭多边形裁剪三角网（keep_inside=true 保留内部，false 挖除内部）
 */
GEO_C_API GeoStatus geo_project_tin_clip_polygon(
    GeoProjectHandle handle,
    int32_t surface_slot,
    const double* xyz_array,
    uint32_t point_count,
    bool keep_inside);

/**
 * @brief 撤销最近一次三角网编辑操作
 */
GEO_C_API GeoStatus geo_project_tin_undo(GeoProjectHandle handle, int32_t surface_slot);

/**
 * @brief 重做最近一次被撤销的三角网编辑操作
 */
GEO_C_API GeoStatus geo_project_tin_redo(GeoProjectHandle handle, int32_t surface_slot);

/**
 * @brief 查询当前是否可执行撤销或重做
 */
GEO_C_API GeoStatus geo_project_tin_get_history_state(
    GeoProjectHandle handle,
    int32_t surface_slot,
    bool* out_can_undo,
    bool* out_can_redo);

// ==============================================================================
// 5. 两期三角网高精度土方算量接口（第二阶段新增）
// ==============================================================================

/**
 * @brief 计算两期三角网之间的精确三棱柱叠加土方量（支持零填挖等值线自动剖分）
 * @param roi_xyz_array 可选算量边界多边形顶点数组（传 NULL 表示计算全重叠区域）
 * @param roi_point_count 边界多边形顶点数
 */
GEO_C_API GeoStatus geo_project_compute_earthwork_exact(
    GeoProjectHandle handle,
    int32_t period1_slot,
    int32_t period2_slot,
    const double* roi_xyz_array,
    uint32_t roi_point_count,
    uint32_t thread_count,
    GeoEarthworkSummary* out_summary);

/**
 * @brief 执行并行网格微分法快速土方估算（用于交互预览）
 */
GEO_C_API GeoStatus geo_project_compute_earthwork_grid(
    GeoProjectHandle handle,
    int32_t period1_slot,
    int32_t period2_slot,
    double grid_step,
    const double* roi_xyz_array,
    uint32_t roi_point_count,
    uint32_t thread_count,
    GeoEarthworkSummary* out_summary);

// ==============================================================================
// 6. 向内与向外多级放坡建模与土方计算接口（第二阶段新增）
// ==============================================================================

/**
 * @brief 获取默认多级放坡参数
 */
GEO_C_API GeoGradingParams geo_default_grading_params(void);

/**
 * @brief 执行向内或向外多级放坡建模、坡脚线求交提取、原地面缝合及放坡土方计算
 *
 * 计算完成后：
 *   - 纯放坡设计面自动写入 GEO_SURFACE_GRADING_DESIGN_ONLY (槽位 2)；
 *   - 若 generate_merged_terrain 为 true，缝合后的完整地形自动写入 GEO_SURFACE_GRADING_MERGED_TERRAIN (槽位 3)；
 *   - 前端可直接通过 geo_project_get_dirty_tin_tiles 与 geo_project_fetch_tin_tile_gpu_packet 拉取三维网格渲染。
 *
 * @param base_xyz_array 基准设计轮廓多边形顶点数组 [x0,y0,z0, x1,y1,z1, ...]
 * @param base_point_count 基准轮廓顶点数（至少 3 个）
 * @param stages 多级放坡配置数组
 * @param stage_count 放坡级数（至少 1 级）
 * @param params 放坡整体配置参数
 * @param generate_merged_terrain 是否同时生成与原地面缝合后的完整地形模型
 * @param out_daylight_xyz 可选输出缓冲区：接收坡脚线/开挖边线三维坐标 [x,y,z,...]（传 NULL 则跳过拷贝）
 * @param daylight_xyz_capacity 缓冲区可容纳的三维点数量上限
 * @param out_summary 输出完整放坡工程报告
 */
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
    GeoGradingSummary* out_summary);

#ifdef __cplusplus
}
#endif

#endif  // GEO_KERNEL_C_API_H_

