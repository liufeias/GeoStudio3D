#ifndef GEO_KERNEL_C_API_H_
#define GEO_KERNEL_C_API_H_

/**
 * @file geo_kernel_c_api.h
 * @brief 空间几何内核跨语言扁平 C ABI 接口契约 (Flat C ABI)
 *
 * 版本：2.3.0（任意期数动态土方数据管理 + 全局统一 RTC 坐标对齐 + 零拷贝 GPU 瓦片流）
 *
 * 核心能力覆盖：
 *   1. 任意期数工程数据动态生命周期管理：
 *      - 向后兼容第 1 期 (slot=0)、第 2 期 (slot=1) 以及系统派生放坡槽位 (slot=2, 3)；
 *      - 支持动态创建、枚举与安全销毁任意数量的测绘土方期数槽位 (slot >= 10，如第 3 期、第 4 期……第 N 期)；
 *      - 内核自动维护工程级全局统一主 RTC 参考原点，确保任意期点云与三角网在三维叠加显示、
 *        任意两期土方算量以及多级放坡建模时始终处于同一局部毫米级精确坐标系，彻底杜绝跨期坐标漂移。
 *   2. 亿级 .dat 点云内存映射并发解析与异常容错过滤。
 *   3. 并行莫顿码多级 LOD 点云八叉树构建、视锥体/SSE 调度与 GPU 二进制打包。
 *   4. 空间分块邻接 Delaunay 三角网构建（含最大边长外围狭长三角形自动裁剪）、高程精确内插、
 *      脏瓦片追踪与 GPU 网格导出。
 *   5. 三角网交互式动态拓扑编辑（三维射线拾取、对角线边翻转、强制特征线嵌入、顶点增删改、
 *      多边形边界裁剪、多步增量撤销 Undo / 重做 Redo），支持对任意选定期数执行编辑。
 *   6. 任意两期三角网高精度三棱柱布尔叠加土方算量与实时网格微分估算。
 *   7. 基于任意指定期数原地面的向内/向外多级放坡建模、坡脚线提取、原地面无缝缝合及放坡土方算量。
 *   8. 跨语言结构化诊断日志回调接口（彻底隔离 C++ DLL 与宿主进程之间的 CRT stdio 模式冲突）。
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
 * @brief 不透明工程会话句柄（每个句柄对应一个独立的工程上下文，支持多工程并发隔离）
 */
typedef struct GeoProjectOpaque* GeoProjectHandle;

/**
 * @brief 内核状态返回码（与 C++ 侧 geo_kernel::StatusCode 枚举值严格一一对应）
 */
typedef enum GeoStatus {
    GEO_STATUS_OK = 0,                      /**< 执行成功 */
    GEO_STATUS_INVALID_ARGUMENT = 1,        /**< 参数非法（空指针、槽位号非法或缓冲区不足） */
    GEO_STATUS_FILE_NOT_FOUND = 2,          /**< 目标文件不存在或无法访问 */
    GEO_STATUS_FILE_OPEN_FAILED = 3,        /**< 文件打开或内存映射失败 */
    GEO_STATUS_EMPTY_DATASET = 4,           /**< 数据集为空（未导入点云或无有效三角形） */
    GEO_STATUS_PARSE_FORMAT_ERROR = 5,      /**< 文本解析格式错误或无有效坐标记录 */
    GEO_STATUS_OUT_OF_MEMORY = 6,           /**< 内存分配失败 */
    GEO_STATUS_TOPOLOGY_DEGENERACY = 7,     /**< 几何拓扑退化（如共线点、非凸四边形或受保护约束边） */
    GEO_STATUS_CANCELLED = 8,               /**< 用户通过进度回调主动取消操作 */
    GEO_STATUS_TILE_NOT_FOUND = 9,          /**< 请求的空间瓦片、节点或查询点不在有效范围内 */
    GEO_STATUS_INTERNAL_ERROR = 99          /**< 未预期的内部逻辑或运行时异常 */
} GeoStatus;

/**
 * @brief 内核结构化日志级别枚举（使用 GEO_LOG_LEVEL_* 前缀避免与 C++ 日志宏重名）
 */
typedef enum GeoLogLevel {
    GEO_LOG_LEVEL_DEBUG = 0,
    GEO_LOG_LEVEL_INFO = 1,
    GEO_LOG_LEVEL_WARN = 2,
    GEO_LOG_LEVEL_ERROR = 3
} GeoLogLevel;

/**
 * @brief 原始测绘文件坐标轴向排列规范
 */
typedef enum GeoCoordOrder {
    GEO_COORD_SURVEYING_YXZ = 0,            /**< 测绘规范：第3列 Y(东->X), 第4列 X(北->Y), 第5列 H(高程->Z) */
    GEO_COORD_CARTESIAN_XYZ = 1             /**< 笛卡尔规范：依次为 X, Y, Z */
} GeoCoordOrder;

/**
 * @brief 地形模型期数与系统派生面预定义槽位枚举
 *
 * 槽位编号规划规范：
 *   - 槽位 0：默认第 1 期原始地面（向后兼容旧版固定调用）
 *   - 槽位 1：默认第 2 期对比/竣工面（向后兼容旧版固定调用）
 *   - 槽位 2：系统保留 —— 多级放坡纯设计面三角网（不可直接导入 .dat）
 *   - 槽位 3：系统保留 —— 多级放坡与原地面沿坡脚线缝合后的完整地形（不可直接导入 .dat）
 *   - 槽位 >= 10：动态分配的第 3 期及后续任意期土方测绘数据槽位（由 geo_project_create_period_slot 分配）
 */
typedef enum GeoSurfaceSlot {
    GEO_SURFACE_PERIOD_1_ORIGINAL = 0,      /**< 第 1 期默认槽位：原始地面点云与三角网 */
    GEO_SURFACE_PERIOD_2_TARGET = 1,        /**< 第 2 期默认槽位：竣工/目标/开挖后点云与三角网 */
    GEO_SURFACE_GRADING_DESIGN_ONLY = 2,    /**< 系统派生槽位：多级放坡纯设计面三角网（基底+坡面+马道） */
    GEO_SURFACE_GRADING_MERGED_TERRAIN = 3, /**< 系统派生槽位：多级放坡与原地面无缝缝合完整地形网 */
    GEO_SURFACE_USER_PERIOD_START = 10      /**< 第 3 期及后续任意期动态分配起始槽位号 */
} GeoSurfaceSlot;

/**
 * @brief C ABI 进度与取消回调函数指针（返回 false 可立即安全中止耗时计算任务）
 */
typedef bool (*GeoProgressCallback)(double progress, uint64_t processed_items, void* user_data);

/**
 * @brief C ABI 结构化日志回调函数指针（全部字符串均为 UTF-8 编码）
 */
typedef void (*GeoLogCallback)(
    int32_t level,
    const char* module_name,
    const char* source_location,
    const char* message,
    void* user_data);

/**
 * @brief 点云导入解析参数结构体
 */
typedef struct GeoDatLoadParams {
    int32_t coord_order;                    /**< 见 GeoCoordOrder */
    uint32_t thread_count;                  /**< 工作线程数（0 表示自动匹配硬件逻辑核心数） */
    bool enable_elevation_filter;           /**< 是否启用高程飞点过滤 */
    double min_valid_elevation;             /**< 有效高程下限（米） */
    double max_valid_elevation;             /**< 有效高程上限（米） */
    bool filter_zero_coordinates;           /**< 是否过滤 (0, 0) 无效测点 */
} GeoDatLoadParams;

/**
 * @brief 点云导入统计与工程坐标范围摘要
 */
typedef struct GeoDatSummary {
    uint64_t file_bytes;                    /**< 文件原始字节数 */
    uint64_t total_lines_scanned;           /**< 扫描总行数 */
    uint64_t valid_points;                  /**< 成功解析的有效测点数 */
    uint64_t skipped_lines;                 /**< 跳过的空行、表头与注释行数 */
    uint64_t malformed_lines;               /**< 格式异常行数 */
    uint64_t filtered_outliers;             /**< 被飞点过滤器剔除的异常点数 */
    double elapsed_seconds;                 /**< 解析总耗时（秒） */
    double throughput_mpts_per_sec;         /**< 解析吞吐率（百万点/秒） */
    double rtc_origin[3];                   /**< 工程统一全局大地参考原点 (X, Y, Z) */
    double global_min[3];                   /**< 本期数据全局大地包围盒最小值 */
    double global_max[3];                   /**< 本期数据全局大地包围盒最大值 */
    double local_min[3];                    /**< 相对工程统一 RTC 原点的局部包围盒最小值 */
    double local_max[3];                    /**< 相对工程统一 RTC 原点的局部包围盒最大值 */
} GeoDatSummary;

/**
 * @brief 八叉树构建参数结构体
 */
typedef struct GeoOctreeParams {
    uint32_t max_depth;                     /**< 最大树深度（默认 10） */
    uint32_t max_points_per_leaf;           /**< 叶子节点最大点数（默认 16384） */
    uint32_t max_lod_points_per_node;       /**< 内部节点抽稀点数上限（默认 8192） */
    uint32_t thread_count;                  /**< 并行构建线程数（0 表示自动匹配） */
} GeoOctreeParams;

/**
 * @brief 八叉树构建结果摘要
 */
typedef struct GeoOctreeSummary {
    uint64_t total_points;                  /**< 纳入八叉树索引的点云总数 */
    uint32_t total_nodes;                   /**< 生成的八叉树总节点数 */
    uint32_t leaf_nodes;                    /**< 叶子节点总数 */
    uint32_t max_reached_depth;             /**< 实际达到的最大树深度 */
    double elapsed_seconds;                 /**< 构建耗时（秒） */
} GeoOctreeSummary;

/**
 * @brief 空间分块三角网构建参数结构体
 */
typedef struct GeoTinParams {
    double decimation_grid_size;            /**< 构网前特征保留抽稀网格步长（米，<=0 表示全量构网） */
    double tile_size;                       /**< 空间渲染与编辑瓦片边长（米，默认 128.0） */
    double max_edge_length;                 /**< 外围最大允许边长裁剪阈值（米，<=0 表示不裁剪） */
} GeoTinParams;

/**
 * @brief 三角网构建结果摘要
 */
typedef struct GeoTinSummary {
    uint64_t input_points;                  /**< 输入原始点数 */
    uint64_t deduplicated_points;           /**< 平面重合去重点数 */
    uint64_t decimated_points;              /**< 网格抽稀滤除点数 */
    uint64_t tin_vertices;                  /**< 最终三角网有效顶点数 */
    uint64_t tin_triangles;                 /**< 最终三角网有效三角形数 */
    uint32_t total_tiles;                   /**< 划分的空间渲染瓦片数 */
    double elapsed_seconds;                 /**< 构网总耗时（秒） */
} GeoTinSummary;

/**
 * @brief 视锥体 LOD 查询相机参数结构体
 */
typedef struct GeoCameraQuery {
    double camera_local_pos[3];             /**< 相机在统一 RTC 局部坐标系下的三维位置 */
    double frustum_planes[6][4];            /**< 6 个单位化视锥体裁剪平面方程 (a, b, c, d) */
    double viewport_height_px;              /**< 渲染视口高度（像素） */
    double fov_y_radians;                   /**< 垂直视场角（弧度） */
    double sse_threshold_px;                /**< 屏幕空间误差阈值（像素） */
    uint64_t max_point_budget;              /**< 本次查询允许的最大渲染点数预算 */
    bool enable_frustum_culling;            /**< 是否开启视锥体剔除 */
} GeoCameraQuery;

/**
 * @brief 三维射线拾取结果结构体（用于前端交互式选择任意期三角网顶点/边/面片）
 */
typedef struct GeoRayHitResult {
    bool hit;                               /**< 是否命中有效三角形 */
    uint32_t triangle_id;                   /**< 命中三角形全局索引 */
    uint32_t nearest_vertex_id;             /**< 距交点最近的三角形顶点索引 */
    int32_t nearest_edge_slot;              /**< 距交点最近的边槽位 [0..2] */
    double distance;                        /**< 射线起点至交点的欧氏距离（米） */
    double hit_local_pos[3];                /**< 交点在统一 RTC 局部坐标系下的三维坐标 */
} GeoRayHitResult;

/**
 * @brief 两期三角网土方算量结果摘要结构体
 */
typedef struct GeoEarthworkSummary {
    double cut_volume_m3;                   /**< 总挖方体积（立方米，正值） */
    double fill_volume_m3;                  /**< 总填方体积（立方米，正值） */
    double net_volume_m3;                   /**< 净土方量 = 填方体积 - 挖方体积（立方米） */
    double total_overlap_area_m2;           /**< 两期重叠有效平面面积（平方米） */
    double cut_area_m2;                     /**< 挖方区平面投影面积（平方米） */
    double fill_area_m2;                    /**< 填方区平面投影面积（平方米） */
    double zero_area_m2;                    /**< 零填挖区平面面积（平方米） */
    double max_cut_depth_m;                 /**< 最大挖方深度（正值，米） */
    double max_fill_height_m;               /**< 最大填方高度（正值，米） */
    uint64_t intersected_prisms;            /**< 参与积分的微元三棱柱或有效采样网格总数 */
    double elapsed_seconds;                 /**< 计算总耗时（秒） */
} GeoEarthworkSummary;

/**
 * @brief 单级放坡参数结构体
 */
typedef struct GeoSlopeStageSpec {
    double stage_height;                    /**< 本级边坡最大垂直高差（米） */
    double slope_ratio;                     /**< 坡比 1:m 中的 m 值 */
    double berm_width;                      /**< 马道平台水平宽度（米） */
    double berm_slope_ratio;                /**< 马道排水横坡 */
} GeoSlopeStageSpec;

/**
 * @brief 多级放坡整体设计参数结构体
 */
typedef struct GeoGradingParams {
    int32_t direction;                      /**< 0 = 向外放坡 (Outward), 1 = 向内放坡 (Inward) */
    int32_t vertical_mode;                  /**< 0 = 自动挖填, -1 = 强制向下放坡, 1 = 强制向上放坡 */
    bool use_uniform_base_elevation;        /**< 是否统一设计基准平台高程 */
    double base_elevation;                  /**< 统一基准高程（局部 Z，米） */
    bool auto_daylight_to_terrain;          /**< 是否自动与原地面求交并生成坡脚线/开挖边线 */
    bool cap_inward_bottom;                 /**< 向内放坡时是否自动封底 */
    double perimeter_sample_step;           /**< 基准轮廓加密采样步长（米，默认 1.5m） */
    double output_tile_size;                /**< 输出放坡三角网瓦片尺寸（米，默认 128.0m） */
} GeoGradingParams;

/**
 * @brief 多级放坡工程计算摘要报告结构体
 */
typedef struct GeoGradingSummary {
    GeoEarthworkSummary earthwork;          /**< 放坡挖填土方量统计 */
    double daylight_perimeter_m;            /**< 坡脚线/开挖边线总周长（米） */
    double slope_surface_area_m2;           /**< 边坡斜面三维展开总面积（平方米） */
    double berm_surface_area_m2;            /**< 马道平台三维总面积（平方米） */
    double base_pad_area_m2;                /**< 基准设计平台/坑底平面面积（平方米） */
    uint32_t actual_stages_used;            /**< 实际达到的最大放坡级数 */
    uint32_t daylight_vertex_count;         /**< 封闭坡脚线三维顶点数 */
    bool daylight_converged;                /**< 是否全周向成功与原地面相交闭合 */
    double elapsed_seconds;                 /**< 计算总耗时（秒） */
} GeoGradingSummary;

// ==============================================================================
// 1. 生命周期、日志桥接、错误诊断与任意期数槽位管理接口
// ==============================================================================

GEO_C_API const char* geo_get_version(void);
GEO_C_API const char* geo_get_last_error(void);
GEO_C_API void geo_set_log_callback(GeoLogCallback callback, void* user_data);

GEO_C_API GeoProjectHandle geo_project_create(void);
GEO_C_API void geo_project_destroy(GeoProjectHandle handle);

/**
 * @brief 动态分配一个新的用户期数槽位 ID（优先分配 0 -> 1 -> 10 -> 11 -> ...，永不与系统槽位 2/3 冲突）
 */
GEO_C_API GeoStatus geo_project_create_period_slot(
    GeoProjectHandle handle,
    int32_t* out_slot_id);

/**
 * @brief 安全卸载并释放指定期数槽位的全部点云、八叉树、三角网及撤销栈内存
 *        若全部用户期数均已被清空，自动重置工程主 RTC 原点以供新工程重新归化。
 */
GEO_C_API GeoStatus geo_project_remove_slot(
    GeoProjectHandle handle,
    int32_t surface_slot);

/**
 * @brief 清空工程内全部用户期数与派生放坡面并重置全局主 RTC 坐标原点
 */
GEO_C_API GeoStatus geo_project_clear_all_periods(GeoProjectHandle handle);

/**
 * @brief 枚举当前工程中所有已激活的用户期数与派生槽位编号（支持两段式缓冲区大小探测）
 */
GEO_C_API GeoStatus geo_project_list_slots(
    GeoProjectHandle handle,
    int32_t* out_slot_ids,
    uint32_t capacity,
    uint32_t* out_count);

/**
 * @brief 获取工程当前统一的全局主 RTC 坐标原点
 */
GEO_C_API GeoStatus geo_project_get_master_rtc(
    GeoProjectHandle handle,
    double out_rtc_origin[3],
    bool* out_initialized);

// ==============================================================================
// 2. 亿级 .dat 点云导入与多级 LOD 八叉树接口（支持任意期数槽位）
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
// 3. 空间分块三角网 (TIN) 构建、高程查询与 GPU 网格包接口（支持任意期数槽位）
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
// 4. 三角网交互式动态拓扑编辑与 Undo/Redo 接口（支持任意期数槽位）
// ==============================================================================

GEO_C_API GeoStatus geo_project_pick_tin_ray(
    GeoProjectHandle handle,
    int32_t surface_slot,
    const double ray_origin[3],
    const double ray_dir[3],
    GeoRayHitResult* out_hit);

GEO_C_API GeoStatus geo_project_tin_flip_edge(
    GeoProjectHandle handle,
    int32_t surface_slot,
    uint32_t triangle_id,
    int32_t edge_slot,
    bool force_flip);

GEO_C_API GeoStatus geo_project_tin_modify_vertex_z(
    GeoProjectHandle handle,
    int32_t surface_slot,
    uint32_t vertex_id,
    double new_local_z);

GEO_C_API GeoStatus geo_project_tin_insert_vertex(
    GeoProjectHandle handle,
    int32_t surface_slot,
    const double local_xyz[3],
    uint16_t code_id,
    bool legalize_delaunay,
    uint32_t* out_vertex_id);

GEO_C_API GeoStatus geo_project_tin_insert_breakline(
    GeoProjectHandle handle,
    int32_t surface_slot,
    const double* xyz_array,
    uint32_t point_count,
    uint16_t code_id,
    bool interpolate_z_from_tin);

GEO_C_API GeoStatus geo_project_tin_delete_vertex(
    GeoProjectHandle handle,
    int32_t surface_slot,
    uint32_t vertex_id);

GEO_C_API GeoStatus geo_project_tin_clip_polygon(
    GeoProjectHandle handle,
    int32_t surface_slot,
    const double* xyz_array,
    uint32_t point_count,
    bool keep_inside);

GEO_C_API GeoStatus geo_project_tin_undo(GeoProjectHandle handle, int32_t surface_slot);
GEO_C_API GeoStatus geo_project_tin_redo(GeoProjectHandle handle, int32_t surface_slot);

GEO_C_API GeoStatus geo_project_tin_get_history_state(
    GeoProjectHandle handle,
    int32_t surface_slot,
    bool* out_can_undo,
    bool* out_can_redo);

// ==============================================================================
// 5. 任意两期三角网高精度土方算量接口
// ==============================================================================

GEO_C_API GeoStatus geo_project_compute_earthwork_exact(
    GeoProjectHandle handle,
    int32_t period1_slot,
    int32_t period2_slot,
    const double* roi_xyz_array,
    uint32_t roi_point_count,
    uint32_t thread_count,
    GeoEarthworkSummary* out_summary);

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
// 6. 向内与向外多级放坡建模与土方计算接口（支持基于任意期数原地面放坡）
// ==============================================================================

GEO_C_API GeoGradingParams geo_default_grading_params(void);

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

