#include "geo_kernel/dat_parser.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstring>
#include <string_view>
#include <thread>
#include <unordered_map>
#include <vector>

#include "geo_kernel/mmap_reader.hpp"

namespace geo_kernel {

namespace {

// 预计算 10 的正负幂次查找表，加速 FastParseDouble 小数与科学计数法解析
constexpr double kPow10Pos[] = {
    1e0,  1e1,  1e2,  1e3,  1e4,  1e5,  1e6,  1e7,  1e8,  1e9,
    1e10, 1e11, 1e12, 1e13, 1e14, 1e15, 1e16, 1e17, 1e18, 1e19,
    1e20, 1e21, 1e22
};

constexpr double kPow10Neg[] = {
    1e0,   1e-1,  1e-2,  1e-3,  1e-4,  1e-5,  1e-6,  1e-7,  1e-8,  1e-9,
    1e-10, 1e-11, 1e-12, 1e-13, 1e-14, 1e-15, 1e-16, 1e-17, 1e-18, 1e-19,
    1e-20, 1e-21, 1e-22
};

inline bool IsHorizontalSpace(char c) noexcept {
    return c == ' ' || c == '\t' || c == '\r';
}

inline const char* TrimLeft(const char* p, const char* end) noexcept {
    while (p < end && IsHorizontalSpace(*p)) {
        ++p;
    }
    return p;
}

inline const char* TrimRight(const char* begin, const char* p) noexcept {
    while (p > begin && IsHorizontalSpace(*(p - 1))) {
        --p;
    }
    return p;
}

/**
 * @brief 单行字段零拷贝切片描述符（直接指向内存映射视图）
 */
struct TokenSlice {
    const char* begin = nullptr;
    const char* end = nullptr;

    [[nodiscard]] size_t Length() const noexcept {
        return static_cast<size_t>(end - begin);
    }

    [[nodiscard]] std::string_view View() const noexcept {
        return {begin, Length()};
    }
};

/**
 * @brief 对单行文本执行快速字段切分（自动兼容逗号、制表符、分号及连续空格分隔）
 * @return 返回实际切分出的有效列数（最多采集前 6 列）
 */
int SplitLineTokens(const char* line_begin, const char* line_end, TokenSlice out_tokens[6]) noexcept {
    const char* p = TrimLeft(line_begin, line_end);
    const char* end = TrimRight(p, line_end);
    if (p >= end) {
        return 0;
    }

    bool has_explicit_delim = false;
    for (const char* scan = p; scan < end; ++scan) {
        if (*scan == ',' || *scan == '\t' || *scan == ';') {
            has_explicit_delim = true;
            break;
        }
    }

    int count = 0;
    if (has_explicit_delim) {
        const char* field_start = p;
        for (const char* scan = p; scan <= end; ++scan) {
            if (scan == end || *scan == ',' || *scan == '\t' || *scan == ';') {
                if (count < 6) {
                    const char* tb = TrimLeft(field_start, scan);
                    const char* te = TrimRight(tb, scan);
                    out_tokens[count++] = {tb, te};
                }
                field_start = scan + 1;
            }
        }
    } else {
        while (p < end && count < 6) {
            p = TrimLeft(p, end);
            if (p >= end) break;
            const char* word_start = p;
            while (p < end && !IsHorizontalSpace(*p)) {
                ++p;
            }
            out_tokens[count++] = {word_start, p};
        }
    }

    return count;
}

/**
 * @brief 解析测点点号（纯数字直接转 uint32，含字母点号如 "GP01" 采用 FNV-1a 哈希保留唯一标识）
 */
uint32_t ParsePointId(std::string_view sv, uint32_t fallback_seq) noexcept {
    if (sv.empty()) {
        return fallback_seq;
    }
    uint64_t val = 0;
    bool all_digits = true;
    for (char c : sv) {
        if (c >= '0' && c <= '9') {
            val = val * 10 + static_cast<uint64_t>(c - '0');
            if (val > 0xFFFFFFFFull) {
                all_digits = false;
                break;
            }
        } else {
            all_digits = false;
            break;
        }
    }
    if (all_digits) {
        return static_cast<uint32_t>(val);
    }

    uint32_t hash = 2166136261u;
    for (unsigned char c : sv) {
        hash ^= c;
        hash *= 16777619u;
    }
    return hash;
}

/**
 * @brief 单个工作线程独立局部解析缓冲区（无锁并行，最后统一归并）
 */
struct ThreadWorkerResult {
    std::vector<SurveyPoint> raw_global_points;
    BoundingBox3d bounds;
    uint64_t lines_scanned = 0;
    uint64_t skipped_empty_or_comment = 0;
    uint64_t malformed_lines = 0;
    uint64_t filtered_outliers = 0;
    std::unordered_map<std::string, uint16_t> local_code_map;
    std::vector<std::string> local_code_list;
    std::vector<std::string> error_samples;
};

}  // namespace

// ==============================================================================
// 高性能无锁双精度解析器实现
// ==============================================================================

bool FastParseDouble(const char* begin, const char* end, double& out_value) noexcept {
    const char* p = TrimLeft(begin, end);
    const char* stop = TrimRight(p, end);
    if (p >= stop) {
        return false;
    }

    bool negative = false;
    if (*p == '-') {
        negative = true;
        ++p;
    } else if (*p == '+') {
        ++p;
    }
    if (p >= stop) {
        return false;
    }

    uint64_t mantissa = 0;
    int exponent_adjust = 0;
    int significant_digits = 0;
    bool has_any_digit = false;

    while (p < stop && *p >= '0' && *p <= '9') {
        has_any_digit = true;
        if (significant_digits < 19) {
            mantissa = mantissa * 10 + static_cast<uint64_t>(*p - '0');
            if (mantissa > 0) {
                ++significant_digits;
            }
        } else {
            ++exponent_adjust;
        }
        ++p;
    }

    if (p < stop && *p == '.') {
        ++p;
        while (p < stop && *p >= '0' && *p <= '9') {
            has_any_digit = true;
            if (significant_digits < 19) {
                mantissa = mantissa * 10 + static_cast<uint64_t>(*p - '0');
                if (mantissa > 0) {
                    ++significant_digits;
                }
                --exponent_adjust;
            }
            ++p;
        }
    }

    if (!has_any_digit) {
        return false;
    }

    int explicit_exp = 0;
    if (p < stop && (*p == 'e' || *p == 'E')) {
        ++p;
        bool exp_neg = false;
        if (p < stop && *p == '-') {
            exp_neg = true;
            ++p;
        } else if (p < stop && *p == '+') {
            ++p;
        }
        if (p >= stop || *p < '0' || *p > '9') {
            return false;
        }
        while (p < stop && *p >= '0' && *p <= '9') {
            explicit_exp = explicit_exp * 10 + (*p - '0');
            if (explicit_exp > 400) {
                return false;
            }
            ++p;
        }
        if (exp_neg) {
            explicit_exp = -explicit_exp;
        }
    }

    if (p != stop) {
        return false;
    }

    double val = static_cast<double>(mantissa);
    const int total_exp = exponent_adjust + explicit_exp;

    if (total_exp > 0) {
        if (total_exp <= 22) {
            val *= kPow10Pos[total_exp];
        } else {
            val *= std::pow(10.0, static_cast<double>(total_exp));
        }
    } else if (total_exp < 0) {
        const int neg_exp = -total_exp;
        if (neg_exp <= 22) {
            val *= kPow10Neg[neg_exp];
        } else {
            val *= std::pow(10.0, static_cast<double>(total_exp));
        }
    }

    if (!std::isfinite(val)) {
        return false;
    }

    out_value = negative ? -val : val;
    return true;
}

// ==============================================================================
// 多线程 .dat 点云主解析流程
// ==============================================================================

OperationResult DatPointCloudParser::ParseFile(
    const std::filesystem::path& file_path,
    const DatParseOptions& options,
    std::vector<SurveyPoint>& out_points,
    std::vector<std::string>& out_code_dict,
    DatParseStats& out_stats) const {
    const auto start_tp = std::chrono::steady_clock::now();

    out_points.clear();
    out_code_dict.clear();
    out_code_dict.emplace_back("");
    out_stats = DatParseStats{};

    MappedFileReader reader;
    OperationResult open_res = reader.Open(file_path);
    if (!open_res.IsOk()) {
        return open_res;
    }

    const char* const data = reader.Data();
    const uint64_t total_size = reader.Size();
    out_stats.file_bytes = reader.RawFileSize();

    if (total_size == 0) {
        return OperationResult::Fail(StatusCode::kEmptyDataset, "文件有效文本内容为空");
    }

    size_t num_threads = options.thread_count;
    if (num_threads == 0) {
        const unsigned int hw = std::thread::hardware_concurrency();
        num_threads = hw > 0 ? static_cast<size_t>(hw) : 4;
    }
    if (total_size < 256ull * 1024ull) {
        num_threads = 1;
    }

    struct ByteRange {
        uint64_t start = 0;
        uint64_t end = 0;
    };
    std::vector<ByteRange> chunks;
    chunks.reserve(num_threads);

    uint64_t current_start = 0;
    const uint64_t avg_chunk = total_size / num_threads;
    for (size_t i = 0; i < num_threads && current_start < total_size; ++i) {
        uint64_t target_end = (i == num_threads - 1)
            ? total_size
            : std::min<uint64_t>(total_size, current_start + avg_chunk);

        while (target_end < total_size && data[target_end] != '\n') {
            ++target_end;
        }
        if (target_end < total_size && data[target_end] == '\n') {
            ++target_end;
        }

        if (target_end > current_start) {
            chunks.push_back({current_start, target_end});
            current_start = target_end;
        }
    }

    const size_t actual_threads = chunks.size();
    std::vector<ThreadWorkerResult> worker_results(actual_threads);
    std::atomic<uint64_t> processed_bytes{0};
    std::atomic<uint64_t> atomic_valid_points{0};
    std::atomic<bool> cancel_requested{false};

    auto worker_fn = [&](size_t worker_idx) {
        const ByteRange range = chunks[worker_idx];
        ThreadWorkerResult& res = worker_results[worker_idx];

        const uint64_t chunk_bytes = range.end - range.start;
        res.raw_global_points.reserve(static_cast<size_t>(chunk_bytes / 36 + 64));

        const char* p = data + range.start;
        const char* const chunk_end = data + range.end;
        TokenSlice tokens[6];

        uint64_t batch_bytes = 0;
        uint64_t batch_pts = 0;
        uint32_t local_seq = static_cast<uint32_t>(worker_idx * 10000000u + 1u);

        while (p < chunk_end) {
            if ((res.lines_scanned & 0x3FFFull) == 0 && cancel_requested.load(std::memory_order_relaxed)) {
                return;
            }

            const void* nl_ptr = std::memchr(p, '\n', static_cast<size_t>(chunk_end - p));
            const char* line_end = (nl_ptr != nullptr)
                ? static_cast<const char*>(nl_ptr)
                : chunk_end;
            const char* next_line = (nl_ptr != nullptr) ? (line_end + 1) : chunk_end;

            batch_bytes += static_cast<uint64_t>(next_line - p);
            ++res.lines_scanned;

            const char* trimmed = TrimLeft(p, line_end);
            if (trimmed >= line_end || *trimmed == '#' || *trimmed == ';' ||
                (trimmed + 1 < line_end && trimmed[0] == '/' && trimmed[1] == '/')) {
                ++res.skipped_empty_or_comment;
                p = next_line;
                continue;
            }

            const int col_count = SplitLineTokens(trimmed, line_end, tokens);
            if (col_count < 3) {
                ++res.malformed_lines;
                if (res.error_samples.size() < options.max_error_samples) {
                    res.error_samples.emplace_back(
                        "分块[" + std::to_string(worker_idx) + "]第" +
                        std::to_string(res.lines_scanned) + "行列数不足(<3): " +
                        std::string(trimmed, std::min<size_t>(64, static_cast<size_t>(line_end - trimmed))));
                }
                p = next_line;
                continue;
            }

            double c1 = 0.0, c2 = 0.0, c3 = 0.0;
            uint32_t pt_id = local_seq++;
            std::string_view code_sv;
            bool parsed_ok = false;

            if (col_count >= 5) {
                // 标准 CASS 5 列格式：点号, 编码, Y(东), X(北), H(高程)
                parsed_ok = FastParseDouble(tokens[2].begin, tokens[2].end, c1) &&
                            FastParseDouble(tokens[3].begin, tokens[3].end, c2) &&
                            FastParseDouble(tokens[4].begin, tokens[4].end, c3);
                if (parsed_ok) {
                    pt_id = ParsePointId(tokens[0].View(), pt_id);
                    code_sv = tokens[1].View();
                }
            } else if (col_count == 4) {
                // 4 列简码省略格式：点号, Y(东), X(北), H(高程)
                parsed_ok = FastParseDouble(tokens[1].begin, tokens[1].end, c1) &&
                            FastParseDouble(tokens[2].begin, tokens[2].end, c2) &&
                            FastParseDouble(tokens[3].begin, tokens[3].end, c3);
                if (parsed_ok) {
                    pt_id = ParsePointId(tokens[0].View(), pt_id);
                }
            } else {
                // 3 列纯坐标格式：Easting, Northing, Elevation
                parsed_ok = FastParseDouble(tokens[0].begin, tokens[0].end, c1) &&
                            FastParseDouble(tokens[1].begin, tokens[1].end, c2) &&
                            FastParseDouble(tokens[2].begin, tokens[2].end, c3);
            }

            if (!parsed_ok) {
                if (worker_idx == 0 && res.lines_scanned <= 3) {
                    ++res.skipped_empty_or_comment;
                } else {
                    ++res.malformed_lines;
                    if (res.error_samples.size() < options.max_error_samples) {
                        res.error_samples.emplace_back(
                            "分块[" + std::to_string(worker_idx) + "]第" +
                            std::to_string(res.lines_scanned) + "行数值解析失败: " +
                            std::string(trimmed, std::min<size_t>(64, static_cast<size_t>(line_end - trimmed))));
                    }
                }
                p = next_line;
                continue;
            }

            // 测绘高斯坐标系与标准三维笛卡尔坐标系轴向统一映射：
            //   - 测绘 Y 轴（东向 Easting）  -> 内部笛卡尔 X 轴
            //   - 测绘 X 轴（北向 Northing） -> 内部笛卡尔 Y 轴
            //   - 测绘 H 轴（高程 Elevation）-> 内部笛卡尔 Z 轴
            // 因此在 kSurveyingYXZ（列序 Y, X, H）和 kCartesianXYZ（列序 X, Y, Z）两种规范下，
            // 第一坐标分量 c1 均对应东向 X，第二坐标分量 c2 均对应北向 Y。
            (void)options.coord_order;
            const double gx = c1;
            const double gy = c2;
            const double gz = c3;

            if (options.filter_zero_coordinates && std::abs(gx) < 1e-6 && std::abs(gy) < 1e-6) {
                ++res.filtered_outliers;
                p = next_line;
                continue;
            }
            if (options.enable_elevation_filter &&
                (gz < options.min_valid_elevation || gz > options.max_valid_elevation)) {
                ++res.filtered_outliers;
                p = next_line;
                continue;
            }

            uint16_t code_id = 0;
            if (!code_sv.empty()) {
                std::string code_str(code_sv);
                auto it = res.local_code_map.find(code_str);
                if (it != res.local_code_map.end()) {
                    code_id = it->second;
                } else if (res.local_code_list.size() < 65000) {
                    code_id = static_cast<uint16_t>(res.local_code_list.size() + 1);
                    res.local_code_map.emplace(code_str, code_id);
                    res.local_code_list.emplace_back(std::move(code_str));
                }
            }

            SurveyPoint pt{};
            pt.x = gx;
            pt.y = gy;
            pt.z = gz;
            pt.point_id = pt_id;
            pt.code_id = code_id;
            pt.flags = SurveyPoint::kFlagValid;
            if (code_id != 0) {
                pt.flags |= SurveyPoint::kFlagFeature;
            }

            res.raw_global_points.push_back(pt);
            res.bounds.Expand({gx, gy, gz});
            ++batch_pts;

            if ((batch_pts & 0xFFFFull) == 0) {
                processed_bytes.fetch_add(batch_bytes, std::memory_order_relaxed);
                atomic_valid_points.fetch_add(batch_pts, std::memory_order_relaxed);
                batch_bytes = 0;
                batch_pts = 0;
            }

            p = next_line;
        }

        processed_bytes.fetch_add(batch_bytes, std::memory_order_relaxed);
        atomic_valid_points.fetch_add(batch_pts, std::memory_order_relaxed);
    };

    std::vector<std::thread> threads;
    threads.reserve(actual_threads);
    for (size_t i = 0; i < actual_threads; ++i) {
        threads.emplace_back(worker_fn, i);
    }

    if (options.progress_callback) {
        while (true) {
            const uint64_t done = processed_bytes.load(std::memory_order_relaxed);
            const uint64_t pts = atomic_valid_points.load(std::memory_order_relaxed);
            const double ratio = std::clamp(
                static_cast<double>(done) / static_cast<double>(total_size), 0.0, 0.92);

            if (!options.progress_callback(ratio, pts)) {
                cancel_requested.store(true, std::memory_order_relaxed);
                break;
            }
            if (done >= total_size || cancel_requested.load(std::memory_order_relaxed)) {
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
    }

    for (auto& t : threads) {
        if (t.joinable()) {
            t.join();
        }
    }

    if (cancel_requested.load(std::memory_order_relaxed)) {
        return OperationResult::Fail(StatusCode::kCancelled, "用户已取消点云导入解析任务");
    }

    uint64_t total_valid = 0;
    std::unordered_map<std::string, uint16_t> global_code_map;
    std::vector<std::vector<uint16_t>> code_remap(actual_threads);

    for (size_t i = 0; i < actual_threads; ++i) {
        const auto& r = worker_results[i];
        total_valid += static_cast<uint64_t>(r.raw_global_points.size());
        out_stats.total_lines_scanned += r.lines_scanned;
        out_stats.skipped_empty_or_comment += r.skipped_empty_or_comment;
        out_stats.malformed_lines += r.malformed_lines;
        out_stats.filtered_outliers += r.filtered_outliers;
        out_stats.global_bounds.Expand(r.bounds);

        for (const auto& err_msg : r.error_samples) {
            if (out_stats.error_samples.size() < options.max_error_samples) {
                out_stats.error_samples.push_back(err_msg);
            }
        }

        code_remap[i].resize(r.local_code_list.size() + 1, 0);
        for (size_t c_idx = 0; c_idx < r.local_code_list.size(); ++c_idx) {
            const std::string& code_str = r.local_code_list[c_idx];
            auto it = global_code_map.find(code_str);
            if (it != global_code_map.end()) {
                code_remap[i][c_idx + 1] = it->second;
            } else if (out_code_dict.size() < 65535) {
                const auto global_id = static_cast<uint16_t>(out_code_dict.size());
                global_code_map.emplace(code_str, global_id);
                out_code_dict.push_back(code_str);
                code_remap[i][c_idx + 1] = global_id;
            }
        }
    }

    if (total_valid == 0) {
        return OperationResult::Fail(
            StatusCode::kParseFormatError,
            "未从文件中解析出任何有效三维坐标点，请检查坐标列顺序或分隔符格式");
    }

    out_stats.valid_points = total_valid;
    out_stats.rtc_transform.InitializeFromBounds(out_stats.global_bounds);
    const Vec3d rtc_origin = out_stats.rtc_transform.origin;

    out_stats.local_bounds.min_pt = out_stats.global_bounds.min_pt - rtc_origin;
    out_stats.local_bounds.max_pt = out_stats.global_bounds.max_pt - rtc_origin;

    try {
        out_points.resize(static_cast<size_t>(total_valid));
    } catch (const std::bad_alloc&) {
        return OperationResult::Fail(
            StatusCode::kOutOfMemory,
            "分配点云内存失败，当前有效点数: " + std::to_string(total_valid));
    }

    std::vector<size_t> write_offsets(actual_threads, 0);
    size_t running_offset = 0;
    for (size_t i = 0; i < actual_threads; ++i) {
        write_offsets[i] = running_offset;
        running_offset += worker_results[i].raw_global_points.size();
    }

    std::vector<std::thread> copy_threads;
    copy_threads.reserve(actual_threads);
    for (size_t i = 0; i < actual_threads; ++i) {
        copy_threads.emplace_back([&, i]() {
            const auto& src = worker_results[i].raw_global_points;
            const auto& remap = code_remap[i];
            SurveyPoint* dst = out_points.data() + write_offsets[i];

            for (size_t k = 0; k < src.size(); ++k) {
                SurveyPoint pt = src[k];
                pt.x -= rtc_origin.x;
                pt.y -= rtc_origin.y;
                pt.z -= rtc_origin.z;
                if (pt.code_id < remap.size()) {
                    pt.code_id = remap[pt.code_id];
                }
                dst[k] = pt;
            }
        });
    }

    for (auto& t : copy_threads) {
        if (t.joinable()) {
            t.join();
        }
    }

    if (options.progress_callback) {
        options.progress_callback(1.0, total_valid);
    }

    const auto end_tp = std::chrono::steady_clock::now();
    out_stats.elapsed_seconds = std::chrono::duration<double>(end_tp - start_tp).count();
    if (out_stats.elapsed_seconds > 1e-9) {
        out_stats.throughput_mpts_per_sec =
            (static_cast<double>(total_valid) / 1e6) / out_stats.elapsed_seconds;
    }

    return OperationResult::Ok();
}

}  // namespace geo_kernel

