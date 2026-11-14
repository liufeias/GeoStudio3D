#pragma once

/**
 * @file dat_parser.hpp
 * @brief 亿级测绘 .dat 点云数据多线程高吞吐解析器
 */

#include <cstdint>
#include <filesystem>
#include <functional>
#include <string>
#include <vector>

#include "geo_kernel/core_types.hpp"

namespace geo_kernel {

using ParseProgressCallback = std::function<bool(double progress, uint64_t parsed_points)>;

struct DatParseOptions {
    CoordinateOrder coord_order = CoordinateOrder::kSurveyingYXZ;
    size_t thread_count = 0;
    bool enable_elevation_filter = true;
    double min_valid_elevation = -500.0;
    double max_valid_elevation = 9000.0;
    bool filter_zero_coordinates = true;
    size_t max_error_samples = 10;
    ParseProgressCallback progress_callback = nullptr;
};

struct DatParseStats {
    uint64_t file_bytes = 0;
    uint64_t total_lines_scanned = 0;
    uint64_t valid_points = 0;
    uint64_t skipped_empty_or_comment = 0;
    uint64_t malformed_lines = 0;
    uint64_t filtered_outliers = 0;
    double elapsed_seconds = 0.0;
    double throughput_mpts_per_sec = 0.0;
    BoundingBox3d global_bounds;
    BoundingBox3d local_bounds;
    RtcTransform rtc_transform;
    std::vector<std::string> error_samples;
};

[[nodiscard]] GEO_API bool FastParseDouble(const char* begin, const char* end, double& out_value) noexcept;

class GEO_API DatPointCloudParser {
public:
    DatPointCloudParser() = default;
    ~DatPointCloudParser() = default;

    [[nodiscard]] OperationResult ParseFile(
        const std::filesystem::path& file_path,
        const DatParseOptions& options,
        std::vector<SurveyPoint>& out_points,
        std::vector<std::string>& out_code_dict,
        DatParseStats& out_stats) const;
};

}  // namespace geo_kernel

