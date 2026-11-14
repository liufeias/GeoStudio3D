#pragma once

/**
 * @file mmap_reader.hpp
 * @brief 跨平台高性能大文件内存映射 (Memory-Mapped File) 读取器
 */

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <string>

#include "geo_kernel/core_types.hpp"

namespace geo_kernel {

class GEO_API MappedFileReader {
public:
    MappedFileReader() noexcept = default;
    ~MappedFileReader() noexcept;

    MappedFileReader(const MappedFileReader&) = delete;
    MappedFileReader& operator=(const MappedFileReader&) = delete;
    MappedFileReader(MappedFileReader&& other) noexcept;
    MappedFileReader& operator=(MappedFileReader&& other) noexcept;

    /**
     * @brief 以只读顺序访问模式映射指定路径的文件（支持 >4GB 超大 .dat 文件）
     */
    [[nodiscard]] OperationResult Open(const std::filesystem::path& file_path) noexcept;

    /**
     * @brief 显式释放内存映射与底层文件句柄
     */
    void Close() noexcept;

    [[nodiscard]] const char* Data() const noexcept { return effective_data_; }
    [[nodiscard]] uint64_t Size() const noexcept { return effective_size_; }
    [[nodiscard]] uint64_t RawFileSize() const noexcept { return raw_size_; }
    [[nodiscard]] bool IsOpen() const noexcept { return mapped_base_ != nullptr; }

private:
    const char* mapped_base_ = nullptr;
    const char* effective_data_ = nullptr;
    uint64_t raw_size_ = 0;
    uint64_t effective_size_ = 0;

#if defined(_WIN32)
    void* file_handle_ = nullptr;
    void* mapping_handle_ = nullptr;
#else
    int fd_ = -1;
#endif
};

}  // namespace geo_kernel

