#pragma once

/**
 * @file exact_predicates.hpp
 * @brief 自适应高精度浮点扩展算术与鲁棒二维几何谓词
 *
 * 算法原理（Jonathan Richard Shewchuk 自适应精度算术）：
 *   1. 快速过滤阶段（Fast Filter）：首先使用常规硬件双精度计算行列式，
 *      若其绝对值大于严格推导出的静态浮点误差上界，直接返回符号（覆盖 99.5% 以上的点）。
 *   2. 精确扩展阶段（Exact Expansion）：仅当点接近共线或共圆（行列式落入误差带）时，
 *      自动升级为无舍入误差的浮点扩展序列进行精确求值，保证符号 100% 数学精确。
 */

#include "geo_kernel/core_types.hpp"

namespace geo_kernel {

class GEO_API ExactPredicates {
public:
    /**
     * @brief 初始化浮点处理器分离常数与误差上界（线程安全，支持重复调用）
     */
    static void Initialize() noexcept;

    /**
     * @brief 鲁棒二维方向判定（2D Orientation Test）
     * @return > 0.0 逆时针；< 0.0 顺时针；= 0.0 严格共线
     */
    [[nodiscard]] static double Orient2D(
        double ax, double ay,
        double bx, double by,
        double cx, double cy) noexcept;

    [[nodiscard]] static double Orient2D(
        const Vec3d& a, const Vec3d& b, const Vec3d& c) noexcept {
        return Orient2D(a.x, a.y, b.x, b.y, c.x, c.y);
    }

    /**
     * @brief 鲁棒二维外接圆包含判定（前提：abc 为逆时针顺序）
     * @return > 0.0 点 d 在外接圆内；< 0.0 在外接圆外；= 0.0 严格共圆
     */
    [[nodiscard]] static double InCircle(
        double ax, double ay,
        double bx, double by,
        double cx, double cy,
        double dx, double dy) noexcept;

    [[nodiscard]] static double InCircle(
        const Vec3d& a, const Vec3d& b, const Vec3d& c, const Vec3d& d) noexcept {
        return InCircle(a.x, a.y, b.x, b.y, c.x, c.y, d.x, d.y);
    }

    /**
     * @brief 判断二维点 p 是否位于三角形 (a, b, c) 内部或边界上
     * @return 0 = 外部；1 = 严格内部；2 = 边或顶点上
     */
    [[nodiscard]] static int PointInTriangle2D(
        const Vec3d& a, const Vec3d& b, const Vec3d& c, const Vec3d& p) noexcept;

    /**
     * @brief 判断二维线段 AB 与线段 CD 是否发生严格相交（不含端点接触）
     */
    [[nodiscard]] static bool SegmentsProperlyIntersect2D(
        const Vec3d& a, const Vec3d& b,
        const Vec3d& c, const Vec3d& d) noexcept;

    /**
     * @brief 计算点 p 在三角形 (a, b, c) 上的重心插值高程 Z
     */
    [[nodiscard]] static double InterpolateZOnTriangle(
        const Vec3d& a, const Vec3d& b, const Vec3d& c,
        double px, double py) noexcept;
};

}  // namespace geo_kernel

