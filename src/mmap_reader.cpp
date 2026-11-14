#include "geo_kernel/mmap_reader.hpp"

#include <cstring>
#include <utility>

#if defined(_WIN32)
    #ifndef WIN32_LEAN_AND_MEAN
        #define WIN32_LEAN_AND_MEAN
    #endif
    #include <windows.h>
#else
    #include <cerrno>
    #include <fcntl.h>
    #include <sys/mman.h>
    #include <sys/stat.h>
    #include <unistd.h>
#endif

namespace geo_kernel {

namespace {

/**
 * @brief C++20 兼容的路径转 UTF-8 std::string 辅助函数（解决 MSVC /std:c++20 下 u8string 返回 char8_t 的类型差异）
 */
std::string PathToUtf8String(const std::filesystem::path& p) {
    const std::u8string u8 = p.u8string();
    return std::string(reinterpret_cast<const char*>(u8.data()), u8.size());
}

#if defined(_WIN32)
std::string FormatWindowsError(DWORD err_code) {
    if (err_code == 0) return "未知系统错误";
    LPWSTR buffer = nullptr;
    const DWORD size = FormatMessageW(
        FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS,
        nullptr,
        err_code,
        MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT),
        reinterpret_cast<LPWSTR>(&buffer),
        0,
        nullptr);
    std::string result;
    if (size > 0 && buffer != nullptr) {
        const int utf8_len = WideCharToMultiByte(CP_UTF8, 0, buffer, static_cast<int>(size), nullptr, 0, nullptr, nullptr);
        if (utf8_len > 0) {
            result.resize(static_cast<size_t>(utf8_len));
            WideCharToMultiByte(CP_UTF8, 0, buffer, static_cast<int>(size), result.data(), utf8_len, nullptr, nullptr);
        }
        LocalFree(buffer);
    }
    while (!result.empty() && (result.back() == '\r' || result.back() == '\n' || result.back() == ' ')) {
        result.pop_back();
    }
    return result + " (错误码: " + std::to_string(err_code) + ")";
}
#endif

}  // namespace

MappedFileReader::~MappedFileReader() noexcept {
    Close();
}

MappedFileReader::MappedFileReader(MappedFileReader&& other) noexcept
    : mapped_base_(std::exchange(other.mapped_base_, nullptr)),
      effective_data_(std::exchange(other.effective_data_, nullptr)),
      raw_size_(std::exchange(other.raw_size_, 0)),
      effective_size_(std::exchange(other.effective_size_, 0))
#if defined(_WIN32)
      , file_handle_(std::exchange(other.file_handle_, nullptr)),
      mapping_handle_(std::exchange(other.mapping_handle_, nullptr))
#else
      , fd_(std::exchange(other.fd_, -1))
#endif
{}

MappedFileReader& MappedFileReader::operator=(MappedFileReader&& other) noexcept {
    if (this != &other) {
        Close();
        mapped_base_ = std::exchange(other.mapped_base_, nullptr);
        effective_data_ = std::exchange(other.effective_data_, nullptr);
        raw_size_ = std::exchange(other.raw_size_, 0);
        effective_size_ = std::exchange(other.effective_size_, 0);
#if defined(_WIN32)
        file_handle_ = std::exchange(other.file_handle_, nullptr);
        mapping_handle_ = std::exchange(other.mapping_handle_, nullptr);
#else
        fd_ = std::exchange(other.fd_, -1);
#endif
    }
    return *this;
}

OperationResult MappedFileReader::Open(const std::filesystem::path& file_path) noexcept {
    Close();

    try {
        if (file_path.empty()) {
            return OperationResult::Fail(StatusCode::kInvalidArgument, "文件路径不能为空");
        }

        std::error_code ec;
        if (!std::filesystem::exists(file_path, ec) || ec) {
            return OperationResult::Fail(
                StatusCode::kFileNotFound,
                "找不到指定点云文件: " + PathToUtf8String(file_path));
        }

        if (std::filesystem::is_directory(file_path, ec)) {
            return OperationResult::Fail(
                StatusCode::kInvalidArgument,
                "指定路径是一个目录而非文件: " + PathToUtf8String(file_path));
        }

#if defined(_WIN32)
        HANDLE hFile = CreateFileW(
            file_path.c_str(),
            GENERIC_READ,
            FILE_SHARE_READ | FILE_SHARE_WRITE,
            nullptr,
            OPEN_EXISTING,
            FILE_ATTRIBUTE_NORMAL | FILE_FLAG_SEQUENTIAL_SCAN,
            nullptr);

        if (hFile == INVALID_HANDLE_VALUE) {
            const DWORD err = GetLastError();
            return OperationResult::Fail(
                StatusCode::kFileOpenFailed,
                "打开点云文件失败: " + FormatWindowsError(err));
        }

        LARGE_INTEGER file_size_li{};
        if (!GetFileSizeEx(hFile, &file_size_li)) {
            const DWORD err = GetLastError();
            CloseHandle(hFile);
            return OperationResult::Fail(
                StatusCode::kFileOpenFailed,
                "获取文件大小失败: " + FormatWindowsError(err));
        }

        if (file_size_li.QuadPart <= 0) {
            CloseHandle(hFile);
            return OperationResult::Fail(StatusCode::kEmptyDataset, "点云文件内容为空 (0 字节)");
        }

        raw_size_ = static_cast<uint64_t>(file_size_li.QuadPart);

        HANDLE hMapping = CreateFileMappingW(
            hFile,
            nullptr,
            PAGE_READONLY,
            0,
            0,
            nullptr);

        if (hMapping == nullptr) {
            const DWORD err = GetLastError();
            CloseHandle(hFile);
            raw_size_ = 0;
            return OperationResult::Fail(
                StatusCode::kFileOpenFailed,
                "创建内存映射对象失败: " + FormatWindowsError(err));
        }

        void* view = MapViewOfFile(hMapping, FILE_MAP_READ, 0, 0, 0);
        if (view == nullptr) {
            const DWORD err = GetLastError();
            CloseHandle(hMapping);
            CloseHandle(hFile);
            raw_size_ = 0;
            return OperationResult::Fail(
                StatusCode::kFileOpenFailed,
                "映射文件视图到虚拟内存失败: " + FormatWindowsError(err));
        }

        file_handle_ = hFile;
        mapping_handle_ = hMapping;
        mapped_base_ = static_cast<const char*>(view);
#else
        const int fd = ::open(file_path.c_str(), O_RDONLY);
        if (fd < 0) {
            return OperationResult::Fail(
                StatusCode::kFileOpenFailed,
                std::string("打开点云文件失败: ") + std::strerror(errno));
        }

        struct stat st {};
        if (::fstat(fd, &st) != 0) {
            const int err = errno;
            ::close(fd);
            return OperationResult::Fail(
                StatusCode::kFileOpenFailed,
                std::string("获取文件元数据失败: ") + std::strerror(err));
        }

        if (st.st_size <= 0) {
            ::close(fd);
            return OperationResult::Fail(StatusCode::kEmptyDataset, "点云文件内容为空 (0 字节)");
        }

        raw_size_ = static_cast<uint64_t>(st.st_size);
        void* view = ::mmap(nullptr, static_cast<size_t>(raw_size_), PROT_READ, MAP_PRIVATE, fd, 0);
        if (view == MAP_FAILED) {
            const int err = errno;
            ::close(fd);
            raw_size_ = 0;
            return OperationResult::Fail(
                StatusCode::kFileOpenFailed,
                std::string("mmap 映射文件失败: ") + std::strerror(err));
        }

        ::posix_madvise(view, static_cast<size_t>(raw_size_), POSIX_MADV_SEQUENTIAL);

        fd_ = fd;
        mapped_base_ = static_cast<const char*>(view);
#endif

        effective_data_ = mapped_base_;
        effective_size_ = raw_size_;

        if (raw_size_ >= 3 &&
            static_cast<uint8_t>(mapped_base_[0]) == 0xEF &&
            static_cast<uint8_t>(mapped_base_[1]) == 0xBB &&
            static_cast<uint8_t>(mapped_base_[2]) == 0xBF) {
            effective_data_ += 3;
            effective_size_ -= 3;
        } else if (raw_size_ >= 2) {
            const auto b0 = static_cast<uint8_t>(mapped_base_[0]);
            const auto b1 = static_cast<uint8_t>(mapped_base_[1]);
            if ((b0 == 0xFF && b1 == 0xFE) || (b0 == 0xFE && b1 == 0xFF)) {
                Close();
                return OperationResult::Fail(
                    StatusCode::kParseFormatError,
                    "检测到 UTF-16 编码文件，请先转换为 UTF-8 或 ANSI/GBK 编码后再导入");
            }
        }

        return OperationResult::Ok();
    } catch (...) {
        Close();
        return OperationResult::Fail(StatusCode::kInternalError, "文件映射过程中发生未预期内部异常");
    }
}

void MappedFileReader::Close() noexcept {
#if defined(_WIN32)
    if (mapped_base_ != nullptr) {
        UnmapViewOfFile(mapped_base_);
        mapped_base_ = nullptr;
    }
    if (mapping_handle_ != nullptr) {
        CloseHandle(static_cast<HANDLE>(mapping_handle_));
        mapping_handle_ = nullptr;
    }
    if (file_handle_ != nullptr) {
        CloseHandle(static_cast<HANDLE>(file_handle_));
        file_handle_ = nullptr;
    }
#else
    if (mapped_base_ != nullptr && raw_size_ > 0) {
        ::munmap(const_cast<char*>(mapped_base_), static_cast<size_t>(raw_size_));
        mapped_base_ = nullptr;
    }
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
#endif
    effective_data_ = nullptr;
    raw_size_ = 0;
    effective_size_ = 0;
}

}  // namespace geo_kernel

