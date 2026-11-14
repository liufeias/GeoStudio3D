#pragma once

/**
 * @file earthwork_calculator.hpp
 * @brief 两期三角网高精度土方算量与填挖热力图分析引擎
 *
 * 双引擎设计理念（兼顾竣工审计绝对精度与实时参数拖拽流畅交互）：
 *   1. 精确基准算法 —— 三角网布尔叠加三棱柱法（Exact TIN-to-TIN Prism Overlay）：
 *      - 对第一期（原地面）与第二期（设计/竣工面）三角网在 XY 平面执行空间网格过滤与
 *        Sutherland-Hodgman 精确凸多边形交集裁剪，将交集多边形剖分为互不重叠的微元三角形。
 *      - **零填挖等值线自动剖分（Zero-Contour Prism Splitting）**：当单个微元三棱柱的三个
 *        角点高差 $\Delta h = Z_{\text{target}} - Z_{\text{original}}$ 符号不一致（即横跨零填挖分界线）
 *        时，沿 $\Delta h = 0$ 直线将三棱柱精确切分为纯填方子棱柱与纯挖方子棱柱，彻底消除
 *        传统算法在同一棱柱内正负体积相互抵消导致的算量偏低误差。
 *      - 全流程采用多线程数据并行与 Kahan 补偿高精度浮点数累加，亿级微元积分无精度损失。
 *   2. 实时预览算法 —— 自适应并行网格微分法（Fast Parallel Grid Differential）：
 *      - 在用户交互拖动平场高程或放坡参数时，按指定采样步长并行采样两期 TIN 高程差，
 *        几十毫秒内返回挖方量、填方量及二维高差热力栅格，供前端即时渲染红蓝填挖色带。
 */

#include <cstdint>
#include <vector>

#include "geo_kernel/core_types.hpp"
#include "geo_kernel/tiled_tin.hpp"

namespace geo_kernel {

/**
 * @brief 两期土方算量配置选项
 */
struct EarthworkOptions {
    /**
     * @brief 可选算量边界多边形（RTC 局部坐标系，空表示计算两期三角网全部重叠覆盖区域）
     */
    std::vector<Vec3d> roi_polygon;

    /**
     * @brief 并行计算线程数（0 表示自动匹配硬件逻辑核心数）
     */
    size_t thread_count = 0;

    /**
     * @brief 网格微分法专用采样间距（米，默认 1.0m；精确三棱柱法忽略此项）
     */
    double grid_step = 1.0;

    /**
     * @brief 有效算量平面投影面积下限（平方米），低于此值判定为两期表面无重叠区域
     */
    double min_overlap_area = 1e-4;
};

/**
 * @brief 填挖方二维热力栅格数据（供前端渲染挖填深度色带与等高线）
 */
struct CutFillHeatmapGrid {
    uint32_t width = 0;                     ///< 栅格列数 (X 方向)
    uint32_t height = 0;                    ///< 栅格行数 (Y 方向)
    double origin_x = 0.0;                  ///< 左下角局部 X 坐标（米）
    double origin_y = 0.0;                  ///< 左下角局部 Y 坐标（米）
    double cell_size = 1.0;                 ///< 像元边长（米）
    float min_delta_z = 0.0f;               ///< 最大挖深（负值，米）
    float max_delta_z = 0.0f;               ///< 最大填高（正值，米）
    /**
     * @brief 行主序高差矩阵 (Z_target - Z_original)，长度 = width * height
     *        - 正值 (> 0) 表示填方高度（米）
     *        - 负值 (< 0) 表示挖方深度（米）
     *        - NaN 表示该像元位于两期三角网重叠区域或 ROI 之外
     */
    std::vector<float> delta_z_matrix;
};

/**
 * @brief 土方工程算量完整报告
 */
struct EarthworkReport {
    double cut_volume_m3 = 0.0;             ///< 总挖方体积（立方米，正值，Z_target < Z_original）
    double fill_volume_m3 = 0.0;            ///< 总填方体积（立方米，正值，Z_target > Z_original）
    double net_volume_m3 = 0.0;             ///< 净土方量 = 填方体积 - 挖方体积（立方米）
    double total_overlap_area_m2 = 0.0;     ///< 两期三角网有效重叠总平面面积（平方米）
    double cut_area_m2 = 0.0;               ///< 挖方区平面投影面积（平方米）
    double fill_area_m2 = 0.0;              ///< 填方区平面投影面积（平方米）
    double zero_area_m2 = 0.0;              ///< 零填挖平衡区面积（平方米）
    double max_cut_depth_m = 0.0;           ///< 最大挖方深度（正值，米）
    double max_fill_height_m = 0.0;         ///< 最大填方高度（正值，米）
    uint64_t intersected_prisms = 0;        ///< 参与积分的微元三棱柱（或有效网格）总数
    double elapsed_seconds = 0.0;           ///< 计算总耗时（秒）
};

/**
 * @brief 两期三角网土方算量核心引擎
 */
class GEO_API EarthworkCalculator {
public:
    EarthworkCalculator() = default;
    ~EarthworkCalculator() = default;

    /**
     * @brief 执行高精度三角网布尔叠加三棱柱法土方量计算
     *
     * @param period1_original 第一期原始地面三角网
     * @param period2_target 第二期设计/竣工/开挖后三角网
     * @param options 算量配置参数（支持可选 ROI 边界多边形裁剪）
     * @param out_report 输出完整土方工程量报告
     * @return 操作结果状态
     */
    [[nodiscard]] OperationResult ComputeExactTinToTinVolume(
        const TiledTinSurface& period1_original,
        const TiledTinSurface& period2_target,
        const EarthworkOptions& options,
        EarthworkReport& out_report) const;

    /**
     * @brief 执行并行网格微分法快速土方估算，并可选生成填挖方二维热力栅格
     */
    [[nodiscard]] OperationResult ComputeFastGridDifferential(
        const TiledTinSurface& period1_original,
        const TiledTinSurface& period2_target,
        const EarthworkOptions& options,
        EarthworkReport& out_report,
        CutFillHeatmapGrid* out_heatmap = nullptr) const;
};

}  // namespace geo_kernel

