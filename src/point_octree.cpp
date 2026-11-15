#include "geo_kernel/point_octree.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <numeric>
#include <queue>
#include <thread>
#include <unordered_set>

namespace geo_kernel {
namespace {
/**
 * @brief 莫顿码与原始点索引绑定项（16 字节对齐，利于 SIMD/缓存预取）
 */
struct MortonKeyIndex {
    uint64_t morton = 0;
    uint32_t index = 0;
    uint32_t padding = 0;
};
static_assert(sizeof(MortonKeyIndex) == 16, "MortonKeyIndex 必须为 16 字节");

/**
 * @brief 并行 8-bit 最低有效位基数排序 (Parallel LSB Radix Sort, 8 趟共 64 位)
 *        时间复杂度 O(N)，稳定排序，1 亿点排序在多核 CPU 上耗时不足 0.8 秒。
 */
void ParallelRadixSortMorton(std::vector<MortonKeyIndex>& keys, size_t thread_count) {
    const size_t n = keys.size();
    if (n <= 1) return;
    if (n < 65536 || thread_count <= 1) {
        std::sort(keys.begin(), keys.end(), [](const MortonKeyIndex& a, const MortonKeyIndex& b) noexcept {
            return a.morton < b.morton;
        });
        return;
    }
    std::vector<MortonKeyIndex> buffer(n);
    MortonKeyIndex* src = keys.data();
    MortonKeyIndex* dst = buffer.data();
    // 按线程均匀分块
    std::vector<size_t> chunk_offsets(thread_count + 1, 0);
    for (size_t t = 0; t <= thread_count; ++t) {
        chunk_offsets[t] = (n * t) / thread_count;
    }
    // 每个线程维护 256 个桶的局部直方图
    std::vector<std::array<size_t, 256>> local_hist(thread_count);
    std::vector<std::array<size_t, 256>> thread_write_offsets(thread_count);

    // 莫顿码有效位为低 63 位，执行 8 趟每次 8 位（共 64 位）完整排序
    for (int pass = 0; pass < 8; ++pass) {
        const uint32_t shift = static_cast<uint32_t>(pass * 8);
        // 步骤 1：多线程并行统计各分块字节直方图
        {
            std::vector<std::thread> workers;
            workers.reserve(thread_count);
            for (size_t t = 0; t < thread_count; ++t) {
                workers.emplace_back([&, t]() {
                    auto& hist = local_hist[t];
                    hist.fill(0);
                    const size_t begin = chunk_offsets[t];
                    const size_t end = chunk_offsets[t + 1];
                    for (size_t i = begin; i < end; ++i) {
                        const uint8_t bucket = static_cast<uint8_t>((src[i].morton >> shift) & 0xFFu);
                        ++hist[bucket];
                    }
                });
            }
            for (auto& w : workers) w.join();
        }
        // 步骤 2：计算全局桶前缀和及各线程的精确写入起始偏移（保持稳定性）
        size_t global_offset = 0;
        for (size_t b = 0; b < 256; ++b) {
            for (size_t t = 0; t < thread_count; ++t) {
                thread_write_offsets[t][b] = global_offset;
                global_offset += local_hist[t][b];
            }
        }
        // 步骤 3：多线程无锁并发分散写入目标缓冲区
        {
            std::vector<std::thread> workers;
            workers.reserve(thread_count);
            for (size_t t = 0; t < thread_count; ++t) {
                workers.emplace_back([&, t]() {
                    auto offsets = thread_write_offsets[t];
                    const size_t begin = chunk_offsets[t];
                    const size_t end = chunk_offsets[t + 1];
                    for (size_t i = begin; i < end; ++i) {
                        const uint8_t bucket = static_cast<uint8_t>((src[i].morton >> shift) & 0xFFu);
                        dst[offsets[bucket]++] = src[i];
                    }
                });
            }
            for (auto& w : workers) w.join();
        }
        std::swap(src, dst);
    }
    // 8 趟为偶数趟，最终排序结果恰好落回 keys.data() 中
}

/**
 * @brief 计算与给定中心和半径对应的八叉树子象限 [0..7] 立方体包围盒
 */
BoundingBox3d ComputeChildCubicBox(const BoundingBox3d& parent_box, int octant) noexcept {
    const Vec3d c = parent_box.Center();
    BoundingBox3d child;
    child.min_pt.x = (octant & 1) ? c.x : parent_box.min_pt.x;
    child.max_pt.x = (octant & 1) ? parent_box.max_pt.x : c.x;
    child.min_pt.y = (octant & 2) ? c.y : parent_box.min_pt.y;
    child.max_pt.y = (octant & 2) ? parent_box.max_pt.y : c.y;
    child.min_pt.z = (octant & 4) ? c.z : parent_box.min_pt.z;
    child.max_pt.z = (octant & 4) ? parent_box.max_pt.z : c.z;
    return child;
}

/**
 * @brief 判断点 p 属于父立方体框的哪一个子象限 [0..7]
 */
inline int GetPointOctant(const Vec3d& center, const SurveyPoint& p) noexcept {
    int octant = 0;
    if (p.x >= center.x) octant |= 1;
    if (p.y >= center.y) octant |= 2;
    if (p.z >= center.z) octant |= 4;
    return octant;
}
}  // namespace

struct PointCloudOctree::BuildContext {
    std::vector<SurveyPoint>& points;
    const OctreeBuildOptions& opts;
    uint32_t leaf_count = 0;
    uint32_t max_depth_reached = 0;
};

void PointCloudOctree::Clear() noexcept {
    if (!nodes_.empty()) {
        GEO_LOG_DEBUG("OCTREE", "清空八叉树释放 %zu 个节点缓存", nodes_.size());
    }
    nodes_.clear();
    nodes_.shrink_to_fit();
    root_cubic_bounds_ = BoundingBox3d{};
}

OperationResult PointCloudOctree::Build(
    std::vector<SurveyPoint>& points,
    const BoundingBox3d& local_bounds,
    const OctreeBuildOptions& options,
    OctreeBuildStats& out_stats) {
    const auto start_tp = std::chrono::steady_clock::now();
    Clear();
    out_stats = OctreeBuildStats{};
    options_ = options;

    if (points.empty() || !local_bounds.IsValid()) {
        return OperationResult::Fail(StatusCode::kEmptyDataset, "点云数据为空，无法构建八叉树");
    }
    const size_t n = points.size();
    if (n > static_cast<size_t>(UINT32_MAX)) {
        return OperationResult::Fail(
            StatusCode::kInvalidArgument,
            "单次八叉树构建点数超过 42.9 亿上限");
    }

    size_t num_threads = options.thread_count;
    if (num_threads == 0) {
        const unsigned int hw = std::thread::hardware_concurrency();
        num_threads = hw > 0 ? static_cast<size_t>(hw) : 4;
    }
    if (n < 65536) {
        num_threads = 1;
    }

    GEO_LOG_INFO(
        "OCTREE",
        "开始构建线性八叉树: points=%zu, threads=%zu, max_depth=%u, leaf_cap=%u, lod_cap=%u",
        n,
        num_threads,
        options.max_depth,
        options.max_points_per_leaf,
        options.max_lod_points_per_node);

    // 1. 构建严格正立方体根节点包围盒（外扩 1e-3 米防止边界点浮点越界）
    const Vec3d center = local_bounds.Center();
    const Vec3d extent = local_bounds.Extent();
    const double max_dim = std::max({extent.x, extent.y, extent.z, 1.0});
    const double half_size = 0.5 * max_dim + 1e-3;
    root_cubic_bounds_.min_pt = {center.x - half_size, center.y - half_size, center.z - half_size};
    root_cubic_bounds_.max_pt = {center.x + half_size, center.y + half_size, center.z + half_size};
    const double inv_span = 1.0 / (2.0 * half_size);

    try {
        // 2. 并行计算每个点的 64 位三维莫顿码
        std::vector<MortonKeyIndex> morton_keys(n);
        {
            std::vector<std::thread> workers;
            workers.reserve(num_threads);
            for (size_t t = 0; t < num_threads; ++t) {
                const size_t begin = (n * t) / num_threads;
                const size_t end = (n * (t + 1)) / num_threads;
                workers.emplace_back([&, begin, end]() {
                    for (size_t i = begin; i < end; ++i) {
                        const SurveyPoint& pt = points[i];
                        const double nx = (pt.x - root_cubic_bounds_.min_pt.x) * inv_span;
                        const double ny = (pt.y - root_cubic_bounds_.min_pt.y) * inv_span;
                        const double nz = (pt.z - root_cubic_bounds_.min_pt.z) * inv_span;
                        morton_keys[i].morton = Morton3D::EncodeNormalized(nx, ny, nz);
                        morton_keys[i].index = static_cast<uint32_t>(i);
                    }
                });
            }
            for (auto& w : workers) w.join();
        }

        // 3. 并行基数排序莫顿码，随后按空间连续顺序重排原始点云数组
        ParallelRadixSortMorton(morton_keys, num_threads);
        {
            std::vector<SurveyPoint> reordered_points(n);
            std::vector<std::thread> workers;
            workers.reserve(num_threads);
            for (size_t t = 0; t < num_threads; ++t) {
                const size_t begin = (n * t) / num_threads;
                const size_t end = (n * (t + 1)) / num_threads;
                workers.emplace_back([&, begin, end]() {
                    for (size_t i = begin; i < end; ++i) {
                        reordered_points[i] = points[morton_keys[i].index];
                    }
                });
            }
            for (auto& w : workers) w.join();
            points.swap(reordered_points);
        }

        // 释放临时莫顿码数组以立即回收内存
        morton_keys.clear();
        morton_keys.shrink_to_fit();

        // 4. 预估节点数量并自顶向下递归建立八叉树拓扑 + 自底向上生成 LOD 抽稀层
        const size_t estimated_nodes = std::max<size_t>(
            64, (n / std::max<uint32_t>(1, options.max_points_per_leaf)) * 2);
        nodes_.reserve(estimated_nodes);
        BuildContext ctx{points, options, 0, 0};
        BuildRecursive(ctx, UINT32_MAX, 0, root_cubic_bounds_, 0, static_cast<uint64_t>(n));

        const auto end_tp = std::chrono::steady_clock::now();
        out_stats.total_points = static_cast<uint64_t>(n);
        out_stats.total_nodes = static_cast<uint32_t>(nodes_.size());
        out_stats.leaf_nodes = ctx.leaf_count;
        out_stats.max_reached_depth = ctx.max_depth_reached;
        out_stats.elapsed_seconds = std::chrono::duration<double>(end_tp - start_tp).count();

        GEO_LOG_INFO(
            "OCTREE",
            "八叉树构建完成: total_nodes=%u, leaf_nodes=%u, max_depth=%u, cubic_half=%.2fm, elapsed=%.3fs",
            out_stats.total_nodes,
            out_stats.leaf_nodes,
            out_stats.max_reached_depth,
            half_size,
            out_stats.elapsed_seconds);
        return OperationResult::Ok();
    } catch (const std::bad_alloc&) {
        Clear();
        return OperationResult::Fail(StatusCode::kOutOfMemory, "构建点云八叉树时内存不足");
    } catch (const std::exception& ex) {
        Clear();
        return OperationResult::Fail(
            StatusCode::kInternalError,
            std::string("构建点云八叉树时捕获异常: ") + ex.what());
    } catch (...) {
        Clear();
        return OperationResult::Fail(StatusCode::kInternalError, "构建点云八叉树时发生未预期内部异常");
    }
}

uint32_t PointCloudOctree::BuildRecursive(
    BuildContext& ctx,
    uint32_t parent_id,
    uint8_t depth,
    const BoundingBox3d& cubic_box,
    uint64_t start_idx,
    uint64_t end_idx) {
    const uint32_t node_id = static_cast<uint32_t>(nodes_.size());
    nodes_.emplace_back();
    OctreeNode& init_node = nodes_[node_id];
    init_node.node_id = node_id;
    init_node.parent_id = parent_id;
    init_node.depth = depth;
    init_node.cubic_bounds = cubic_box;
    init_node.point_start = start_idx;
    init_node.point_count = end_idx - start_idx;
    if (depth > ctx.max_depth_reached) {
        ctx.max_depth_reached = depth;
    }

    const uint64_t count = end_idx - start_idx;
    // 终止条件：点数小于等于叶子阈值，或已达到最大树深度
    if (count <= ctx.opts.max_points_per_leaf || depth >= ctx.opts.max_depth) {
        init_node.is_leaf = true;
        ++ctx.leaf_count;
        BoundingBox3d tight;
        for (uint64_t i = start_idx; i < end_idx; ++i) {
            tight.Expand(ctx.points[static_cast<size_t>(i)].Position());
        }
        nodes_[node_id].tight_bounds = tight;
        return node_id;
    }

    // 内部节点：按子象限 [0..7] 划分
    nodes_[node_id].is_leaf = false;
    const Vec3d center = cubic_box.Center();

    // 在当前区间 [start_idx, end_idx) 内按象限就地桶划分，确保任意坐标系边界误差下依然严格有序
    uint64_t octant_counts[8]{0, 0, 0, 0, 0, 0, 0, 0};
    for (uint64_t i = start_idx; i < end_idx; ++i) {
        const int oct = GetPointOctant(center, ctx.points[static_cast<size_t>(i)]);
        ++octant_counts[oct];
    }

    // 统计非空象限数量；若全部点聚集在同一象限且已无法继续有效分裂，则直接退化为叶子节点
    int non_empty_octants = 0;
    for (int oct = 0; oct < 8; ++oct) {
        if (octant_counts[oct] > 0) ++non_empty_octants;
    }
    if (non_empty_octants <= 1 && count <= static_cast<uint64_t>(ctx.opts.max_points_per_leaf) * 4) {
        nodes_[node_id].is_leaf = true;
        ++ctx.leaf_count;
        BoundingBox3d tight;
        for (uint64_t i = start_idx; i < end_idx; ++i) {
            tight.Expand(ctx.points[static_cast<size_t>(i)].Position());
        }
        nodes_[node_id].tight_bounds = tight;
        return node_id;
    }

    // 就地桶排序将子象限点重排为严格连续切片
    uint64_t octant_offsets[8];
    uint64_t octant_cursors[8];
    uint64_t running = start_idx;
    for (int oct = 0; oct < 8; ++oct) {
        octant_offsets[oct] = running;
        octant_cursors[oct] = running;
        running += octant_counts[oct];
    }
    for (int oct = 0; oct < 8; ++oct) {
        const uint64_t bucket_end = octant_offsets[oct] + octant_counts[oct];
        while (octant_cursors[oct] < bucket_end) {
            SurveyPoint& current_pt = ctx.points[static_cast<size_t>(octant_cursors[oct])];
            const int target_oct = GetPointOctant(center, current_pt);
            if (target_oct == oct) {
                ++octant_cursors[oct];
            } else {
                std::swap(current_pt, ctx.points[static_cast<size_t>(octant_cursors[target_oct])]);
                ++octant_cursors[target_oct];
            }
        }
    }

    // 递归构建 8 个子节点（注意递归期间 nodes_ 可能扩容，必须通过 node_id 下标访问）
    uint32_t child_ids[8]{
        UINT32_MAX, UINT32_MAX, UINT32_MAX, UINT32_MAX,
        UINT32_MAX, UINT32_MAX, UINT32_MAX, UINT32_MAX
    };
    BoundingBox3d combined_tight;
    for (int oct = 0; oct < 8; ++oct) {
        const uint64_t c_start = octant_offsets[oct];
        const uint64_t c_end = c_start + octant_counts[oct];
        if (c_start < c_end) {
            const BoundingBox3d child_box = ComputeChildCubicBox(cubic_box, oct);
            const uint32_t cid = BuildRecursive(
                ctx, node_id, static_cast<uint8_t>(depth + 1), child_box, c_start, c_end);
            child_ids[oct] = cid;
            combined_tight.Expand(nodes_[cid].tight_bounds);
        }
    }

    OctreeNode& finalized_node = nodes_[node_id];
    for (int oct = 0; oct < 8; ++oct) {
        finalized_node.children[oct] = child_ids[oct];
    }
    finalized_node.tight_bounds = combined_tight;

    // 自底向上为当前内部节点生成 LOD 抽稀点索引
    GenerateInternalNodeLod(ctx, finalized_node);
    return node_id;
}

void PointCloudOctree::GenerateInternalNodeLod(BuildContext& ctx, OctreeNode& node) {
    // 收集所有直接子节点的候选点索引（叶子节点取其完整区间，内部节点取其 LOD 代理点）
    std::vector<uint32_t> candidates;
    size_t estimated_candidates = 0;
    for (int oct = 0; oct < 8; ++oct) {
        const uint32_t cid = node.children[oct];
        if (cid == UINT32_MAX) continue;
        const OctreeNode& child = nodes_[cid];
        estimated_candidates += child.is_leaf
            ? static_cast<size_t>(child.point_count)
            : child.lod_point_indices.size();
    }
    if (estimated_candidates == 0) return;

    candidates.reserve(estimated_candidates);
    for (int oct = 0; oct < 8; ++oct) {
        const uint32_t cid = node.children[oct];
        if (cid == UINT32_MAX) continue;
        const OctreeNode& child = nodes_[cid];
        if (child.is_leaf) {
            for (uint64_t i = child.point_start; i < child.point_start + child.point_count; ++i) {
                candidates.push_back(static_cast<uint32_t>(i));
            }
        } else {
            candidates.insert(
                candidates.end(),
                child.lod_point_indices.begin(),
                child.lod_point_indices.end());
        }
    }

    const size_t cap = ctx.opts.max_lod_points_per_node;
    if (candidates.size() <= cap) {
        node.lod_point_indices = std::move(candidates);
        return;
    }

    // 特征点优先保留策略：优先收集带有地形特征标志的点（如坎顶、坎脚、控制点）
    node.lod_point_indices.reserve(cap);
    std::vector<uint32_t> non_feature_candidates;
    non_feature_candidates.reserve(candidates.size());
    const size_t max_feature_quota = (cap * 3) / 4; // 最多为特征点保留 75% 配额
    for (uint32_t idx : candidates) {
        if (ctx.points[idx].IsFeature() && node.lod_point_indices.size() < max_feature_quota) {
            node.lod_point_indices.push_back(idx);
        } else {
            non_feature_candidates.push_back(idx);
        }
    }

    // 剩余配额按空间等步长抽稀
    const size_t remaining_quota = cap - node.lod_point_indices.size();
    if (remaining_quota > 0 && !non_feature_candidates.empty()) {
        if (non_feature_candidates.size() <= remaining_quota) {
            node.lod_point_indices.insert(
                node.lod_point_indices.end(),
                non_feature_candidates.begin(),
                non_feature_candidates.end());
        } else {
            const double step = static_cast<double>(non_feature_candidates.size()) /
                                static_cast<double>(remaining_quota);
            for (size_t k = 0; k < remaining_quota; ++k) {
                const size_t pick = static_cast<size_t>(static_cast<double>(k) * step);
                node.lod_point_indices.push_back(non_feature_candidates[pick]);
            }
        }
    }
}

bool PointCloudOctree::IsBoxOutsideFrustum(
    const BoundingBox3d& box,
    const double frustum_planes[6][4]) const noexcept {
    if (!box.IsValid()) return true;
    for (int p = 0; p < 6; ++p) {
        const double a = frustum_planes[p][0];
        const double b = frustum_planes[p][1];
        const double c = frustum_planes[p][2];
        const double d = frustum_planes[p][3];
        // 选取沿平面法线方向最正的包围盒顶点（P-Vertex）进行快速外侧判定
        const double px = (a >= 0.0) ? box.max_pt.x : box.min_pt.x;
        const double py = (b >= 0.0) ? box.max_pt.y : box.min_pt.y;
        const double pz = (c >= 0.0) ? box.max_pt.z : box.min_pt.z;
        if (a * px + b * py + c * pz + d < 0.0) {
            return true; // 整个包围盒完全处于该裁剪平面外侧
        }
    }
    return false;
}

void PointCloudOctree::QueryVisibleNodes(
    const CameraViewParams& view,
    std::vector<uint32_t>& out_visible_node_ids,
    uint64_t& out_estimated_points) const {
    out_visible_node_ids.clear();
    out_estimated_points = 0;
    if (nodes_.empty()) return;

    // 预计算屏幕投影比例因子：proj_scale = viewport_height / (2 * tan(fov_y / 2))
    const double half_fov = std::clamp(view.fov_y_radians * 0.5, 0.01, 1.50);
    const double proj_scale = view.viewport_height_px / (2.0 * std::tan(half_fov));
    const double sse_limit = std::max(0.25, view.sse_threshold_px);

    struct QueueEntry {
        uint32_t node_id = 0;
        double distance = 0.0;
        double sse_pixels = 0.0;
        // 优先队列按屏幕误差降序、相机距离升序排列，确保离相机近且重要的节点优先获得点数预算
        bool operator<(const QueueEntry& rhs) const noexcept {
            if (std::abs(sse_pixels - rhs.sse_pixels) > 1e-3) {
                return sse_pixels < rhs.sse_pixels;
            }
            return distance > rhs.distance;
        }
    };

    auto evaluate_node = [&](uint32_t nid) -> QueueEntry {
        const OctreeNode& node = nodes_[nid];
        const Vec3d c = node.tight_bounds.Center();
        const double radius = std::max(0.5 * node.tight_bounds.Diagonal(), 0.05);
        const double dist_to_center = (c - view.camera_local_pos).Length();
        const double effective_dist = std::max(dist_to_center - radius, 0.1);
        const double geometric_error = radius / 32.0;
        const double sse = (geometric_error * proj_scale) / effective_dist;
        return {nid, effective_dist, sse};
    };

    std::priority_queue<QueueEntry> pq;
    if (!view.enable_frustum_culling || !IsBoxOutsideFrustum(nodes_[0].tight_bounds, view.frustum_planes)) {
        pq.push(evaluate_node(0));
    }

    while (!pq.empty()) {
        const QueueEntry cur = pq.top();
        pq.pop();
        const OctreeNode& node = nodes_[cur.node_id];
        const uint64_t node_render_pts = node.is_leaf
            ? node.point_count
            : static_cast<uint64_t>(node.lod_point_indices.size());

        // 若已经达到单帧渲染点数预算上限，停止继续细分
        if (out_estimated_points + node_render_pts > view.max_point_budget && !out_visible_node_ids.empty()) {
            continue;
        }

        // 若为叶子节点，或屏幕空间误差已满足要求，直接选取当前节点渲染
        if (node.is_leaf || cur.sse_pixels <= sse_limit) {
            out_visible_node_ids.push_back(cur.node_id);
            out_estimated_points += node_render_pts;
            continue;
        }

        // 需要继续向下细分：检查可视子节点
        bool pushed_any_child = false;
        for (int oct = 0; oct < 8; ++oct) {
            const uint32_t cid = node.children[oct];
            if (cid == UINT32_MAX) continue;
            if (view.enable_frustum_culling &&
                IsBoxOutsideFrustum(nodes_[cid].tight_bounds, view.frustum_planes)) {
                continue;
            }
            pq.push(evaluate_node(cid));
            pushed_any_child = true;
        }

        // 若所有子节点均被裁剪或异常，回退渲染父节点自身
        if (!pushed_any_child) {
            out_visible_node_ids.push_back(cur.node_id);
            out_estimated_points += node_render_pts;
        }
    }

    // 最终按相机距离由近及远排序，利于 GPU Early-Z 深度剔除加速渲染
    std::sort(out_visible_node_ids.begin(), out_visible_node_ids.end(), [&](uint32_t a, uint32_t b) {
        const double da = (nodes_[a].tight_bounds.Center() - view.camera_local_pos).LengthSquared();
        const double db = (nodes_[b].tight_bounds.Center() - view.camera_local_pos).LengthSquared();
        return da < db;
    });
}

OperationResult PointCloudOctree::SerializeNodeToGpuPacket(
    uint32_t node_id,
    const std::vector<SurveyPoint>& points,
    const RtcTransform& rtc,
    std::vector<uint8_t>& out_packet) const {
    out_packet.clear();
    if (node_id >= nodes_.size()) {
        return OperationResult::Fail(
            StatusCode::kTileNotFound,
            "请求的八叉树节点编号越界: " + std::to_string(node_id));
    }

    const OctreeNode& node = nodes_[node_id];
    const uint32_t vertex_count = node.is_leaf
        ? static_cast<uint32_t>(node.point_count)
        : static_cast<uint32_t>(node.lod_point_indices.size());

    const size_t header_bytes = sizeof(GpuTilePacketHeader);
    const size_t payload_bytes = static_cast<size_t>(vertex_count) * sizeof(GpuPointVertex);
    out_packet.resize(header_bytes + payload_bytes);

    GpuTilePacketHeader header{};
    header.magic = 0x544F4547u; // "GEOT"
    header.version = 1u;
    header.payload_type = static_cast<uint32_t>(GpuTilePayloadType::kPointCloudOctreeNode);
    header.flags = 0u;
    header.tile_x = static_cast<int32_t>(node.depth);
    header.tile_y = static_cast<int32_t>(node.node_id);
    header.tile_uid = (static_cast<uint64_t>(node.depth) << 32) | static_cast<uint64_t>(node.node_id);
    header.revision = 1u;
    header.vertex_count = vertex_count;
    header.index_count = 0u;
    header.vertex_stride = static_cast<uint32_t>(sizeof(GpuPointVertex));
    header.rtc_origin[0] = rtc.origin.x;
    header.rtc_origin[1] = rtc.origin.y;
    header.rtc_origin[2] = rtc.origin.z;
    header.bbox_min[0] = static_cast<float>(node.tight_bounds.min_pt.x);
    header.bbox_min[1] = static_cast<float>(node.tight_bounds.min_pt.y);
    header.bbox_min[2] = static_cast<float>(node.tight_bounds.min_pt.z);
    header.bbox_max[0] = static_cast<float>(node.tight_bounds.max_pt.x);
    header.bbox_max[1] = static_cast<float>(node.tight_bounds.max_pt.y);
    header.bbox_max[2] = static_cast<float>(node.tight_bounds.max_pt.z);
    std::memcpy(out_packet.data(), &header, header_bytes);

    if (vertex_count > 0) {
        auto* dst_vertices = reinterpret_cast<GpuPointVertex*>(out_packet.data() + header_bytes);
        if (node.is_leaf) {
            if (node.point_start + node.point_count > points.size()) {
                out_packet.clear();
                return OperationResult::Fail(
                    StatusCode::kInternalError,
                    "八叉树叶子节点点云区间超出全局点云数组边界");
            }
            for (uint32_t i = 0; i < vertex_count; ++i) {
                const SurveyPoint& pt = points[static_cast<size_t>(node.point_start) + i];
                dst_vertices[i].x = static_cast<float>(pt.x);
                dst_vertices[i].y = static_cast<float>(pt.y);
                dst_vertices[i].z = static_cast<float>(pt.z);
                dst_vertices[i].packed_attr =
                    (static_cast<uint32_t>(pt.flags) << 16) | static_cast<uint32_t>(pt.code_id);
            }
        } else {
            for (uint32_t i = 0; i < vertex_count; ++i) {
                const uint32_t global_idx = node.lod_point_indices[i];
                if (global_idx >= points.size()) {
                    out_packet.clear();
                    return OperationResult::Fail(
                        StatusCode::kInternalError,
                        "八叉树 LOD 代理索引超出全局点云数组边界");
                }
                const SurveyPoint& pt = points[global_idx];
                dst_vertices[i].x = static_cast<float>(pt.x);
                dst_vertices[i].y = static_cast<float>(pt.y);
                dst_vertices[i].z = static_cast<float>(pt.z);
                dst_vertices[i].packed_attr =
                    (static_cast<uint32_t>(pt.flags) << 16) | static_cast<uint32_t>(pt.code_id);
            }
        }
    }
    return OperationResult::Ok();
}

}  // namespace geo_kernel
