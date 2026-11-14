cmake_minimum_required(VERSION 3.20)

# ==============================================================================
# 项目名称：GeoKernelCore（三维测绘点云与三角网土方放坡空间几何内核）
# 版本：2.2.1（严格路径匹配 + 跨平台 C++20 生产级构建脚本）
#
# 设计规范：
#   1. 严格遵循工程标准目录结构：
#      - 公共与模块头文件严格位于：include/geo_kernel/
#      - 实现源文件与压测入口严格位于：src/
#   2. 配置阶段执行严格文件存在性预检：若预期路径下缺少任意文件，立即输出清晰的
#      中文缺失路径清单并中止配置。
#   3. 严格保持 IEEE 754 浮点精度语义（禁用 -ffast-math / 启用 /fp:precise），
#      确保 Shewchuk 自适应精确几何谓词在任意优化级别下符号 100% 数学精确。
# ==============================================================================
project(GeoKernelCore
    VERSION 2.2.1
    DESCRIPTION "Enterprise C++20 Geospatial Point Cloud, TIN Editing, Earthwork & Grading Kernel"
    LANGUAGES CXX C
)

# 强制启用 C++20 标准，禁止编译器非标准扩展以保证 Windows / Linux / macOS / WASM 行为一致
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# 默认生成位置无关代码 (PIC)，支持编译为动态库或被其他语言 FFI / WASM 链接
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# 构建配置开关
option(GEO_BUILD_SHARED "构建为动态链接库 (.dll/.so/.dylib) 以供 C ABI 调用" ON)
option(GEO_BUILD_BENCHMARK "构建命令行亿级点云与土方放坡性能基准测试程序" ON)
option(GEO_ENABLE_ARCH_OPT "开启针对当前架构的 SIMD (AVX2/NEON) 向量化优化" ON)

# 统一归集静态库、动态库与可执行程序的输出目录
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")

# ------------------------------------------------------------------------------
# 平台与编译器精细化调优
# ------------------------------------------------------------------------------
if(MSVC)
    # 启用 UTF-8 源码与执行字符集，消除中文注释与路径编码警告
    # /wd4251: 抑制导出类中私有 STL 容器成员的 DLL 接口冗余警告（跨语言调用统一走扁平 C ABI）
    add_compile_options(/utf-8 /W4 /wd4251 /permissive- /EHsc)
    # 保持 IEEE 754 严格浮点语义 (/fp:precise)，确保精确几何谓词不被编译器重排破坏
    add_compile_options(/fp:precise)
    if(GEO_ENABLE_ARCH_OPT AND CMAKE_SYSTEM_PROCESSOR MATCHES "AMD64|x86_64")
        add_compile_options(
            $<$<CONFIG:Release>:/O2>
            $<$<CONFIG:Release>:/arch:AVX2>
            $<$<CONFIG:RelWithDebInfo>:/O2>
            $<$<CONFIG:RelWithDebInfo>:/arch:AVX2>
        )
    else()
        add_compile_options(
            $<$<CONFIG:Release>:/O2>
            $<$<CONFIG:RelWithDebInfo>:/O2>
        )
    endif()
    add_compile_definitions(_CRT_SECURE_NO_WARNINGS NOMINMAX WIN32_LEAN_AND_MEAN)
else()
    add_compile_options(-Wall -Wextra -Wpedantic)
    # 严禁开启 -ffast-math，否则会破坏 Shewchuk 浮点误差扩展算法
    add_compile_options(-fno-fast-math)
    if(EMSCRIPTEN)
        # WebAssembly 目标支持多线程 SharedArrayBuffer 与 SIMD128（32位 WASM 内存上限 2GB）
        add_compile_options(-O3 -msimd128 -pthread)
        add_link_options(-pthread -sALLOW_MEMORY_GROWTH=1 -sMAXIMUM_MEMORY=2GB)
    else()
        if(GEO_ENABLE_ARCH_OPT)
            add_compile_options(
                $<$<CONFIG:Release>:-O3>
                $<$<CONFIG:Release>:-march=native>
                $<$<CONFIG:RelWithDebInfo>:-O3>
                $<$<CONFIG:RelWithDebInfo>:-march=native>
            )
        else()
            add_compile_options(
                $<$<CONFIG:Release>:-O3>
                $<$<CONFIG:RelWithDebInfo>:-O3>
            )
        endif()
    endif()
endif()

# 查找跨平台多线程支持库（Windows 原生线程 / POSIX pthread）
find_package(Threads REQUIRED)

# ------------------------------------------------------------------------------
# 严格声明核心几何内核头文件与源文件路径（与项目目录结构一一对应）
# ------------------------------------------------------------------------------
set(GEO_KERNEL_HEADERS
    include/geo_kernel/core_types.hpp
    include/geo_kernel/exact_predicates.hpp
    include/geo_kernel/mmap_reader.hpp
    include/geo_kernel/dat_parser.hpp
    include/geo_kernel/point_octree.hpp
    include/geo_kernel/tiled_tin.hpp
    include/geo_kernel/tin_editor.hpp
    include/geo_kernel/earthwork_calculator.hpp
    include/geo_kernel/slope_grading.hpp
    include/geo_kernel/geo_kernel_c_api.h
)

set(GEO_KERNEL_SOURCES
    src/exact_predicates.cpp
    src/mmap_reader.cpp
    src/dat_parser.cpp
    src/point_octree.cpp
    src/tiled_tin.cpp
    src/tin_editor.cpp
    src/earthwork_calculator.cpp
    src/slope_grading.cpp
    src/geo_kernel_c_api.cpp
)

set(GEO_BENCHMARK_SOURCES
    src/main_benchmark.cpp
)

# ------------------------------------------------------------------------------
# 严格路径完整性预检：逐一验证每个声明路径是否存在，绝不推测或搜索其他路径
# ------------------------------------------------------------------------------
set(GEO_REQUIRED_FILES ${GEO_KERNEL_HEADERS} ${GEO_KERNEL_SOURCES})
if(GEO_BUILD_BENCHMARK)
    list(APPEND GEO_REQUIRED_FILES ${GEO_BENCHMARK_SOURCES})
endif()

set(GEO_MISSING_FILES "")
foreach(_REL_PATH IN LISTS GEO_REQUIRED_FILES)
    if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${_REL_PATH}")
        list(APPEND GEO_MISSING_FILES "${_REL_PATH}")
    endif()
endforeach()

if(NOT "${GEO_MISSING_FILES}" STREQUAL "")
    string(REPLACE ";" "\n  - " _MISSING_LIST_STR "${GEO_MISSING_FILES}")
    message(FATAL_ERROR
        "\n======================================================================\n"
        "[CMake 严格路径校验失败] 以下项目文件在预期路径下不存在：\n"
        "  - ${_MISSING_LIST_STR}\n\n"
        "请确认上述文件已按完整相对路径提交并推送至当前 Git 仓库分支。\n"
        "======================================================================\n"
    )
endif()

# ------------------------------------------------------------------------------
# 核心几何内核库目标定义
# ------------------------------------------------------------------------------
if(GEO_BUILD_SHARED)
    add_library(GeoKernelCore SHARED ${GEO_KERNEL_HEADERS} ${GEO_KERNEL_SOURCES})
    target_compile_definitions(GeoKernelCore PRIVATE GEO_KERNEL_BUILD_SHARED GEO_KERNEL_EXPORTS)
else()
    add_library(GeoKernelCore STATIC ${GEO_KERNEL_HEADERS} ${GEO_KERNEL_SOURCES})
    target_compile_definitions(GeoKernelCore PUBLIC GEO_KERNEL_STATIC)
endif()

target_include_directories(GeoKernelCore
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(GeoKernelCore PUBLIC Threads::Threads)

# ------------------------------------------------------------------------------
# 命令行亿级点云与土方放坡基准验证程序
# ------------------------------------------------------------------------------
if(GEO_BUILD_BENCHMARK)
    add_executable(GeoKernelBench ${GEO_BENCHMARK_SOURCES})
    target_link_libraries(GeoKernelBench PRIVATE GeoKernelCore)
endif()

