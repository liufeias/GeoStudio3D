#pragma once

/**
 * @file slope_grading.hpp
 * @brief 向内与向外多级放坡建模、原地面动态求交（Daylighting）、地形缝合与放坡土方算量引擎
 *
 * 核心工程能力：
 *   1. 双向多级放坡支持（GradingDirection）：
 *      - kOutward（向外放坡）：适用于填方路基、场地平整垫高、高填方边坡及外扩式露天边坡。
 *        支持每级独立配置坡高、坡比 (1:m)、马道/平台宽度及马道排水横坡。
 *      - kInward（向内放坡）：适用于深基坑分级开挖、水库/蓄水池库盆开挖、露天矿坑阶梯边坡。
 *        支持凹角收缩自相交消解、窄腰多环分裂处理以及闭合基底平台自动封底。
 *   2. 自动挖填判定与原地面动态求交（Auto Cut/Fill & Daylighting）：
 *      - 支持显式指定向下放坡 (-1)、向上放坡 (+1) 或自动按设计高程与原地面高差判定挖填方向 (0)。
 *      - 沿基准多边形轮廓按设定步长加密剖面线，逐级向空间延伸并与第一期原地面 TIN 实时求交；
 *        一旦坡面穿透原地面，立即通过二分迭代精确锁定毫米级**坡脚线/开挖边线 (Daylight Line)**。
 *   3. 三维坡体网格构建与原地面无缝缝合（Seamless Terrain Stitching）：
 *      - 自动生成包含基底/顶面平台、各级边坡面（按挖/填自动着色标签）及各级马道平台的
 *        水密三维三角网（`out_grading_surface`）。
 *      - 同时将坡脚线作为约束边界裁剪原地面 TIN 内部区域，并与放坡设计面沿坡脚线精确缝合，
 *        输出完整的“放坡设计后整体地形模型”（`out_merged_terrain`）。
 *   4. 放坡工程量精确统计：
 *      - 调用精确三棱柱叠加积分器计算放坡挖方量、填方量、净土方量、坡面展开面积及马道面积。
 */

#include <cstdint>
#include <vector>

#include "geo_kernel/core_types.hpp"
#include "geo_kernel/earthwork_calculator.hpp"
#include "geo_kernel/tiled_tin.hpp"

namespace geo_kernel {

/**
 * @brief 放坡水平延伸方向枚举
 */
enum class GradingDirection : int32_t {
    kOutward = 0,   ///< 向基准多边形外侧放坡（如填方路基、场地垫高、外扩边坡）
    kInward = 1     ///< 向基准多边形内侧放坡（如基坑开挖、水库库盆、矿坑边坡）
};

/**
 * @brief 放坡垂直升降方向枚举
 */
enum class GradingVerticalMode : int32_t {
    kAutoCutFill = 0,   ///< 自动判定：设计面高于原地面则向下放填方坡，低于原地面则向上放挖方坡
    kSlopeDown = -1,    ///< 强制向下放坡（高程递减，如填方边坡或基坑向内开挖）
    kSlopeUp = 1        ///< 强制向上放坡（高程递增，如挖方边坡或堆体向上收分）
};

/**
 * @brief 单级放坡与马道参数配置
 */
struct SlopeStageSpec {
    double stage_height = 8.0;          ///< 本级边坡最大垂直高差（米，必须 > 0）
    double slope_ratio = 1.5;           ///< 边坡坡比 1:m 中的 m 值（水平宽度 / 垂直高度，如 1.5 表示 1:1.5）
    double berm_width = 2.0;            ///< 本级坡顶/坡底马道（平台）水平宽度（米，0 表示无马道）
    double berm_slope_ratio = 0.0;      ///< 马道排水坡度（垂直高差 / 水平宽度，如 -0.02 表示向外排水 2%）
};

/**
 * @brief 多级放坡整体设计参数
 */
struct MultiStageGradingOptions {
    GradingDirection direction = GradingDirection::kOutward;
    GradingVerticalMode vertical_mode = GradingVerticalMode::kAutoCutFill;

    /**
     * @brief 基准设计轮廓多边形（RTC 局部坐标系，至少 3 个顶点，自动闭合）
     *        若 use_uniform_base_elevation 为 true，则所有顶点高程统一采用 base_elevation。
     */
    std::vector<Vec3d> base_polygon;
    bool use_uniform_base_elevation = false;
    double base_elevation = 0.0;

    /**
     * @brief 多级边坡参数序列（至少包含 1 级；若设有多级则按顺序逐级外扩或内收）
     */
    std::vector<SlopeStageSpec> stages;

    /**
     * @brief 是否自动与原地面三角网求交并在坡脚线/开挖边线处终止放坡（Daylighting）
     *        - true : 遇到原地面立即终止，生成自然贴合地形的封闭坡脚线/开挖线
     *        - false: 严格按所有 stages 规定的总高差完整放坡（适用于固定底标高基坑/水池设计）
     */
    bool auto_daylight_to_terrain = true;

    /**
     * @brief 向内放坡（kInward）且未触及地形时，是否自动封底生成基坑/库盆底板平面（默认 true）
     */
    bool cap_inward_bottom = true;

    /**
     * @brief 基准轮廓沿边加密采样步长（米，默认 1.5m，越小则坡脚线贴合起伏地形越精细）
     */
    double perimeter_sample_step = 1.5;

    /**
     * @brief 输出三角网的空间瓦片尺寸（米，默认 128.0m）
     */
    double output_tile_size = 128.0;
};

/**
 * @brief 多级放坡计算结果与工程量报告
 */
struct MultiStageGradingResult {
    EarthworkReport earthwork;              ///< 放坡区域与原地面之间的精确挖填土方量报告
    std::vector<Vec3d> daylight_polyline;   ///< 提取出的封闭坡脚线/开挖边线三维坐标序列（局部坐标）
    double daylight_perimeter_m = 0.0;      ///< 坡脚线/开挖边线总周长（米）
    double slope_surface_area_m2 = 0.0;     ///< 各级边坡斜面三维展开总面积（平方米）
    double berm_surface_area_m2 = 0.0;      ///< 各级马道平台三维总面积（平方米）
    double base_pad_area_m2 = 0.0;          ///< 基准设计平台/坑底平面面积（平方米）
    uint32_t actual_stages_used = 0;        ///< 实际参与放坡的最大级数
    bool daylight_converged = false;        ///< 是否全部轮廓均成功与原地面相交闭合
    double elapsed_seconds = 0.0;           ///< 放坡建模与算量总耗时（秒）
};

/**
 * @brief 多级放坡三维建模与地形缝合核心引擎
 */
class GEO_API SlopeGradingEngine {
public:
    SlopeGradingEngine() = default;
    ~SlopeGradingEngine() = default;

    /**
     * @brief 执行多级放坡建模、坡脚线求交提取、原地面缝合及放坡土方算量
     *
     * @param original_terrain 第一期原始地面三角网（当 auto_daylight_to_terrain 为 true 或需缝合/算量时必须有效）
     * @param options 多级放坡设计参数
     * @param out_grading_surface 输出：纯放坡设计面三角网（含基准面、各级坡面及马道，region_tag 区分区域）
     * @param out_merged_terrain 可选输出：与原地面沿坡脚线裁剪缝合后的完整设计地形三角网（传 nullptr 则跳过缝合）
     * @param out_result 输出：完整放坡统计与土方工程量报告
     * @return 操作结果状态
     */
    [[nodiscard]] OperationResult GenerateMultiStageGrading(
        const TiledTinSurface& original_terrain,
        const MultiStageGradingOptions& options,
        TiledTinSurface& out_grading_surface,
        TiledTinSurface* out_merged_terrain,
        MultiStageGradingResult& out_result) const;
};

}  // namespace geo_kernel

