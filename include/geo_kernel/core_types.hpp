#pragma once

/**
 * @file core_types.hpp
 * @brief 空间几何内核基础数据类型、RTC大坐标归化器、莫顿码空间索引与GPU二进制传输包定义
 *
 * 设计说明：
 *   1. 测绘领域工程坐标（如 CGCS2000 3度带投影坐标）通常带有 7~8 位整数部分。
 *      内核内部统一采用 double (64位双精度) 进行计算，并在导入阶段自动建立
 *      RTC (Relative-To-Center) 局部工程原点，向前端 GPU 传输时输出 float32 相对坐标，
 *      彻底消除大坐标下显卡渲染抖动与三角网破面问题。
 *   2. 所有跨模块/跨语言二进制数据包均采用 1 字节紧凑对齐 (#pragma pack(push, 1))，
 *      确保与 WebGPU / Three.js TypedArray 零开销直接映射。
 */

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

// 跨平台动态库符号导出宏定义
#if defined(GEO_KERNEL_STATIC)
    #define GEO_API
#elif defined(_WIN32) || defined(__CYGWIN__)
    #if defined(GEO_KERNEL_EXPORTS)
        #define GEO_API __declspec(dllexport)
    #else
        #define GEO_API __declspec(dllimport)
    #endif
#else
    #if __GNUC__ >= 4
        #define GEO_API __attribute__((visibility("default")))
    #else
        #define GEO_API
    #endif
#endif

namespace geo_kernel {

// ==============================================================================
// 1. 统一状态码与错误诊断结构
// ==============================================================================

/**
 * @brief 内核执行状态码枚举（与 C ABI 严格对齐）
 */
enum class StatusCode : int32_t {
    kOk = 0,                    ///< 执行成功
    kInvalidArgument = 1,       ///< 参数非法（空指针、越界或非法参数组合）
    kFileNotFound = 2,          ///< 目标文件不存在或无法访问
    kFileOpenFailed = 3,        ///< 文件打开或内存映射失败
    kEmptyDataset = 4,          ///< 数据集为空（无有效点云或三角形）
    kParseFormatError = 5,      ///< 文件格式错误或有效数据比例过低
    kOutOfMemory = 6,           ///< 内存分配失败
    kTopologyDegeneracy = 7,    ///< 几何拓扑退化（如全部点共线无法构成有效平面）
    kCancelled = 8,             ///< 用户通过进度回调主动取消操作
    kTileNotFound = 9,          ///< 请求的空间瓦片编号不存在
    kInternalError = 99         ///< 未预期的内部逻辑异常
};

/**
 * @brief 带有详细中文诊断信息的通用操作结果包装器
 */
struct OperationResult {
    StatusCode code = StatusCode::kOk;
    std::string message;

    [[nodiscard]] bool IsOk() const noexcept { return code == StatusCode::kOk; }

    static OperationResult Ok() { return {StatusCode::kOk, "操作成功"}; }
    static OperationResult Fail(StatusCode c, std::string msg) {
        return {c, std::move(msg)};
    }
};

// ==============================================================================
// 2. 基础三维/二维向量与包围盒（双精度内核计算 + 单精度GPU渲染）
// ==============================================================================

/**
 * @brief 单精度三维向量（用于 GPU 顶点缓冲区上传，节省显存带宽）
 */
struct Vec3f {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;

    constexpr Vec3f() noexcept = default;
    constexpr Vec3f(float x_val, float y_val, float z_val) noexcept
        : x(x_val), y(y_val), z(z_val) {}
};

/**
 * @brief 双精度三维向量（用于内核全流程几何计算与拓扑构建）
 */
struct Vec3d {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;

    constexpr Vec3d() noexcept = default;
    constexpr Vec3d(double x_val, double y_val, double z_val) noexcept
        : x(x_val), y(y_val), z(z_val) {}

    [[nodiscard]] constexpr Vec3d operator+(const Vec3d& rhs) const noexcept {
        return {x + rhs.x, y + rhs.y, z + rhs.z};
    }

    [[nodiscard]] constexpr Vec3d operator-(const Vec3d& rhs) const noexcept {
        return {x - rhs.x, y - rhs.y, z - rhs.z};
    }

    [[nodiscard]] constexpr Vec3d operator*(double scalar) const noexcept {
        return {x * scalar, y * scalar, z * scalar};
    }

    [[nodiscard]] constexpr Vec3d operator/(double scalar) const noexcept {
        const double inv = 1.0 / scalar;
        return {x * inv, y * inv, z * inv};
    }

    constexpr Vec3d& operator+=(const Vec3d& rhs) noexcept {
        x += rhs.x;
        y += rhs.y;
        z += rhs.z;
        return *this;
    }

    constexpr Vec3d& operator-=(const Vec3d& rhs) noexcept {
        x -= rhs.x;
        y -= rhs.y;
        z -= rhs.z;
        return *this;
    }

    [[nodiscard]] constexpr double Dot(const Vec3d& rhs) const noexcept {
        return x * rhs.x + y * rhs.y + z * rhs.z;
    }

    [[nodiscard]] constexpr Vec3d Cross(const Vec3d& rhs) const noexcept {
        return {
            y * rhs.z - z * rhs.y,
            z * rhs.x - x * rhs.z,
            x * rhs.y - y * rhs.x
        };
    }

    [[nodiscard]] constexpr double LengthSquared() const noexcept {
        return x * x + y * y + z * z;
    }

    [[nodiscard]] double Length() const noexcept {
        return std::sqrt(LengthSquared());
    }

    [[nodiscard]] constexpr double DistanceSquared2D(const Vec3d& rhs) const noexcept {
        const double dx = x - rhs.x;
        const double dy = y - rhs.y;
        return dx * dx + dy * dy;
    }

    [[nodiscard]] Vec3d Normalized() const noexcept {
        const double len = Length();
        if (len <= 1e-15) {
            return {0.0, 0.0, 1.0};
        }
        return *this / len;
    }

    [[nodiscard]] constexpr Vec3f ToFloat32() const noexcept {
        return {static_cast<float>(x), static_cast<float>(y), static_cast<float>(z)};
    }
};

/**
 * @brief 双精度三维轴对齐包围盒 (AABB)
 */
struct BoundingBox3d {
    Vec3d min_pt{
        std::numeric_limits<double>::max(),
        std::numeric_limits<double>::max(),
        std::numeric_limits<double>::max()
    };
    Vec3d max_pt{
        std::numeric_limits<double>::lowest(),
        std::numeric_limits<double>::lowest(),
        std::numeric_limits<double>::lowest()
    };

    [[nodiscard]] constexpr bool IsValid() const noexcept {
        return min_pt.x <= max_pt.x && min_pt.y <= max_pt.y && min_pt.z <= max_pt.z;
    }

    constexpr void Expand(const Vec3d& p) noexcept {
        if (p.x < min_pt.x) min_pt.x = p.x;
        if (p.y < min_pt.y) min_pt.y = p.y;
        if (p.z < min_pt.z) min_pt.z = p.z;
        if (p.x > max_pt.x) max_pt.x = p.x;
        if (p.y > max_pt.y) max_pt.y = p.y;
        if (p.z > max_pt.z) max_pt.z = p.z;
    }

    constexpr void Expand(const BoundingBox3d& other) noexcept {
        if (!other.IsValid()) return;
        Expand(other.min_pt);
        Expand(other.max_pt);
    }

    [[nodiscard]] constexpr Vec3d Center() const noexcept {
        return {
            0.5 * (min_pt.x + max_pt.x),
            0.5 * (min_pt.y + max_pt.y),
            0.5 * (min_pt.z + max_pt.z)
        };
    }

    [[nodiscard]] constexpr Vec3d Extent() const noexcept {
        return max_pt - min_pt;
    }

    [[nodiscard]] double Diagonal() const noexcept {
        return IsValid() ? Extent().Length() : 0.0;
    }

    [[nodiscard]] constexpr bool Contains2D(double x, double y) const noexcept {
        return x >= min_pt.x && x <= max_pt.x && y >= min_pt.y && y <= max_pt.y;
    }

    [[nodiscard]] constexpr bool Intersects(const BoundingBox3d& other) const noexcept {
        return (min_pt.x <= other.max_pt.x && max_pt.x >= other.min_pt.x) &&
               (min_pt.y <= other.max_pt.y && max_pt.y >= other.min_pt.y) &&
               (min_pt.z <= other.max_pt.z && max_pt.z >= other.min_pt.z);
    }
};

// ==============================================================================
// 3. 测绘坐标系映射与 RTC (Relative-To-Center) 大坐标归化器
// ==============================================================================

/**
 * @brief 原始文件坐标轴向排列规范
 */
enum class CoordinateOrder : int32_t {
    /**
     * @brief 工程测绘规范（如南方 CASS .dat 文件）：
     *   文件第3列为 Y（东向坐标 Easting -> 映射为内部笛卡尔 X）
     *   文件第4列为 X（北向坐标 Northing -> 映射为内部笛卡尔 Y）
     *   文件第5列为 H（高程 Elevation -> 映射为内部笛卡尔 Z）
     */
    kSurveyingYXZ = 0,

    /**
     * @brief 标准三维笛卡尔规范：
     *   依次为 X（东向）、Y（北向）、Z（高程）
     */
    kCartesianXYZ = 1
};

/**
 * @brief 全局-局部坐标归化变换器 (Relative-To-Center, RTC)
 */
struct RtcTransform {
    Vec3d origin{0.0, 0.0, 0.0};  ///< 全局大地坐标参考原点（已对齐至整米）
    bool initialized = false;

    /**
     * @brief 根据全局包围盒自动初始化取整后的参考原点
     */
    void InitializeFromBounds(const BoundingBox3d& global_bounds) noexcept {
        if (!global_bounds.IsValid()) {
            origin = {0.0, 0.0, 0.0};
            initialized = false;
            return;
        }
        const Vec3d c = global_bounds.Center();
        origin.x = std::floor(c.x);
        origin.y = std::floor(c.y);
        origin.z = std::floor(c.z);
        initialized = true;
    }

    [[nodiscard]] constexpr Vec3d ToLocal(const Vec3d& global_pt) const noexcept {
        return global_pt - origin;
    }

    [[nodiscard]] constexpr Vec3d ToGlobal(const Vec3d& local_pt) const noexcept {
        return local_pt + origin;
    }

    [[nodiscard]] constexpr Vec3f ToGpuFloat32(const Vec3d& local_pt) const noexcept {
        return local_pt.ToFloat32();
    }
};

// ==============================================================================
// 4. 亿级点云紧凑内存结构（严格 32 字节缓存行友好对齐）
// ==============================================================================

/**
 * @brief 紧凑型测绘点云节点（32 字节，单 Cache Line 可容纳 2 个完整点）
 */
struct alignas(32) SurveyPoint {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    uint32_t point_id = 0;
    uint16_t code_id = 0;
    uint16_t flags = 0x0001;

    static constexpr uint16_t kFlagValid = 0x0001;
    static constexpr uint16_t kFlagFeature = 0x0002;
    static constexpr uint16_t kFlagBoundary = 0x0004;

    [[nodiscard]] constexpr Vec3d Position() const noexcept { return {x, y, z}; }
    [[nodiscard]] constexpr bool IsValid() const noexcept { return (flags & kFlagValid) != 0; }
    [[nodiscard]] constexpr bool IsFeature() const noexcept { return (flags & kFlagFeature) != 0; }
};
static_assert(sizeof(SurveyPoint) == 32, "SurveyPoint 必须严格保持 32 字节对齐");

// ==============================================================================
// 5. 64位三维莫顿码 (Morton Z-Order Curve) 空间局部性编码器
// ==============================================================================

/**
 * @brief 三维莫顿码编码器（将 21 位 X/Y/Z 整数网格坐标交错映射为 63 位无符号整数）
 */
class Morton3D {
public:
    static constexpr uint32_t kMaxCoordBits = 21;
    static constexpr uint32_t kMaxGridCoord = (1u << kMaxCoordBits) - 1u;

    [[nodiscard]] static constexpr uint64_t SplitBy3(uint32_t a) noexcept {
        uint64_t x = static_cast<uint64_t>(a) & 0x1FFFFFull;
        x = (x | (x << 32)) & 0x1F00000000FFFFull;
        x = (x | (x << 16)) & 0x1F0000FF0000FFull;
        x = (x | (x << 8))  & 0x100F00F00F00F00Full;
        x = (x | (x << 4))  & 0x10C30C30C30C30C3ull;
        x = (x | (x << 2))  & 0x1249249249249249ull;
        return x;
    }

    [[nodiscard]] static constexpr uint64_t EncodeGrid(uint32_t ix, uint32_t iy, uint32_t iz) noexcept {
        return SplitBy3(ix) | (SplitBy3(iy) << 1) | (SplitBy3(iz) << 2);
    }

    [[nodiscard]] static constexpr uint64_t EncodeNormalized(double nx, double ny, double nz) noexcept {
        const double cx = std::clamp(nx, 0.0, 1.0);
        const double cy = std::clamp(ny, 0.0, 1.0);
        const double cz = std::clamp(nz, 0.0, 1.0);
        const auto ix = static_cast<uint32_t>(cx * static_cast<double>(kMaxGridCoord));
        const auto iy = static_cast<uint32_t>(cy * static_cast<double>(kMaxGridCoord));
        const auto iz = static_cast<uint32_t>(cz * static_cast<double>(kMaxGridCoord));
        return EncodeGrid(ix, iy, iz);
    }
};

// ==============================================================================
// 6. 前后端解耦：标准 GPU 瓦片二进制传输协议（供 WebGPU / Three.js 直接消费）
// ==============================================================================

enum class GpuTilePayloadType : uint32_t {
    kPointCloudOctreeNode = 1,  ///< 八叉树点云 LOD 节点数据包
    kTinSurfaceMesh = 2,        ///< 分块三角网网格数据包（含顶点、法线与三角形索引）
    kGradingDesignMesh = 3      ///< 多级放坡设计面网格数据包
};

#pragma pack(push, 1)

/**
 * @brief 跨进程 / WebView2 / WASM 二进制几何瓦片统一包头（固定 96 字节）
 */
struct GpuTilePacketHeader {
    uint32_t magic = 0x544F4547u;   ///< 小端序 "GEOT" 魔数校验
    uint32_t version = 1u;          ///< 协议版本号
    uint32_t payload_type = 0u;     ///< 见 GpuTilePayloadType
    uint32_t flags = 0u;            ///< 保留标志位（bit0: 是否包含索引缓冲）

    int32_t tile_x = 0;             ///< 空间瓦片 X 网格编号（或八叉树层级）
    int32_t tile_y = 0;             ///< 空间瓦片 Y 网格编号（或八叉树节点哈希）
    uint64_t tile_uid = 0;          ///< 全局唯一瓦片/节点标识符
    uint32_t revision = 0;          ///< 瓦片拓扑修改版本号（用于增量脏更新对比）

    uint32_t vertex_count = 0;      ///< 顶点数量
    uint32_t index_count = 0;       ///< 索引数量（三角网模式下为三角形数 * 3）
    uint32_t vertex_stride = 0;     ///< 单个顶点字节步长

    double rtc_origin[3]{0.0, 0.0, 0.0};  ///< 全局参考原点（前端用于大坐标恢复与拾取）
    float bbox_min[3]{0.0f, 0.0f, 0.0f};  ///< 瓦片局部包围盒最小值（用于视锥体剔除）
    float bbox_max[3]{0.0f, 0.0f, 0.0f};  ///< 瓦片局部包围盒最大值
};
static_assert(sizeof(GpuTilePacketHeader) == 96, "GpuTilePacketHeader 必须严格为 96 字节");

/**
 * @brief GPU 点云顶点结构（16 字节，紧跟 GpuTilePacketHeader 之后）
 */
struct GpuPointVertex {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    uint32_t packed_attr = 0u;
};
static_assert(sizeof(GpuPointVertex) == 16, "GpuPointVertex 必须严格为 16 字节");

/**
 * @brief GPU 三角网顶点结构（28 字节，包含法线与高程属性，支持光照与坡度渲染）
 */
struct GpuMeshVertex {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    float nx = 0.0f;
    float ny = 0.0f;
    float nz = 1.0f;
    uint32_t packed_attr = 0u;
};
static_assert(sizeof(GpuMeshVertex) == 28, "GpuMeshVertex 必须严格为 28 字节");

#pragma pack(pop)

}  // namespace geo_kernel

