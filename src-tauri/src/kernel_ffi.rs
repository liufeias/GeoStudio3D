//! 空间几何内核 `GeoKernelCore` 动态库 FFI 安全封装层
//!
//! 设计要点：
//!   1. 所有 `#[repr(C)]` 结构体与 `include/geo_kernel/geo_kernel_c_api.h` 严格逐字节对齐；
//!   2. 通过 `libloading` 运行时动态加载 `GeoKernelCore.dll` / `libGeoKernelCore.so` / `libGeoKernelCore.dylib`，
//!      支持自动探测可执行文件同级目录及构建输出目录，部署零配置；
//!   3. 暴露线程安全的 `SafeGeoProject` 句柄包装，析构时自动调用 `geo_project_destroy` 释放全部 C++ 内存。

use libloading::{Library, Symbol};
use serde::{Deserialize, Serialize};
use std::ffi::{CStr, CString};
use std::os::raw::{c_char, c_void};
use std::path::PathBuf;
use std::ptr;
use std::sync::Arc;

#[repr(C)]
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum GeoStatus {
    Ok = 0,
    InvalidArgument = 1,
    FileNotFound = 2,
    FileOpenFailed = 3,
    EmptyDataset = 4,
    ParseFormatError = 5,
    OutOfMemory = 6,
    TopologyDegeneracy = 7,
    Cancelled = 8,
    TileNotFound = 9,
    InternalError = 99,
}

#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct GeoDatLoadParams {
    pub coord_order: i32,
    pub thread_count: u32,
    pub enable_elevation_filter: bool,
    pub min_valid_elevation: f64,
    pub max_valid_elevation: f64,
    pub filter_zero_coordinates: bool,
}

#[repr(C)]
#[derive(Debug, Copy, Clone, Default, Serialize)]
#[serde(rename_all = "camelCase")]
pub struct GeoDatSummary {
    pub file_bytes: u64,
    pub total_lines_scanned: u64,
    pub valid_points: u64,
    pub skipped_lines: u64,
    pub malformed_lines: u64,
    pub filtered_outliers: u64,
    pub elapsed_seconds: f64,
    pub throughput_mpts_per_sec: f64,
    pub rtc_origin: [f64; 3],
    pub global_min: [f64; 3],
    pub global_max: [f64; 3],
    pub local_min: [f64; 3],
    pub local_max: [f64; 3],
}

#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct GeoOctreeParams {
    pub max_depth: u32,
    pub max_points_per_leaf: u32,
    pub max_lod_points_per_node: u32,
    pub thread_count: u32,
}

#[repr(C)]
#[derive(Debug, Copy, Clone, Default, Serialize)]
#[serde(rename_all = "camelCase")]
pub struct GeoOctreeSummary {
    pub total_points: u64,
    pub total_nodes: u32,
    pub leaf_nodes: u32,
    pub max_reached_depth: u32,
    pub elapsed_seconds: f64,
}

#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct GeoTinParams {
    pub decimation_grid_size: f64,
    pub tile_size: f64,
    pub max_edge_length: f64,
}

#[repr(C)]
#[derive(Debug, Copy, Clone, Default, Serialize)]
#[serde(rename_all = "camelCase")]
pub struct GeoTinSummary {
    pub input_points: u64,
    pub deduplicated_points: u64,
    pub decimated_points: u64,
    pub tin_vertices: u64,
    pub tin_triangles: u64,
    pub total_tiles: u32,
    pub elapsed_seconds: f64,
}

#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct GeoCameraQuery {
    pub camera_local_pos: [f64; 3],
    pub frustum_planes: [[f64; 4]; 6],
    pub viewport_height_px: f64,
    pub fov_y_radians: f64,
    pub sse_threshold_px: f64,
    pub max_point_budget: u64,
    pub enable_frustum_culling: bool,
}

#[repr(C)]
#[derive(Debug, Copy, Clone, Default, Serialize)]
#[serde(rename_all = "camelCase")]
pub struct GeoRayHitResult {
    pub hit: bool,
    pub triangle_id: u32,
    pub nearest_vertex_id: u32,
    pub nearest_edge_slot: i32,
    pub distance: f64,
    pub hit_local_pos: [f64; 3],
}

#[repr(C)]
#[derive(Debug, Copy, Clone, Default, Serialize)]
#[serde(rename_all = "camelCase")]
pub struct GeoEarthworkSummary {
    pub cut_volume_m3: f64,
    pub fill_volume_m3: f64,
    pub net_volume_m3: f64,
    pub total_overlap_area_m2: f64,
    pub cut_area_m2: f64,
    pub fill_area_m2: f64,
    pub zero_area_m2: f64,
    pub max_cut_depth_m: f64,
    pub max_fill_height_m: f64,
    pub intersected_prisms: u64,
    pub elapsed_seconds: f64,
}

#[repr(C)]
#[derive(Debug, Copy, Clone, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct GeoSlopeStageSpec {
    pub stage_height: f64,
    pub slope_ratio: f64,
    pub berm_width: f64,
    pub berm_slope_ratio: f64,
}

#[repr(C)]
#[derive(Debug, Copy, Clone, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct GeoGradingParams {
    pub direction: i32,
    pub vertical_mode: i32,
    pub use_uniform_base_elevation: bool,
    pub base_elevation: f64,
    pub auto_daylight_to_terrain: bool,
    pub cap_inward_bottom: bool,
    pub perimeter_sample_step: f64,
    pub output_tile_size: f64,
}

#[repr(C)]
#[derive(Debug, Copy, Clone, Default, Serialize)]
#[serde(rename_all = "camelCase")]
pub struct GeoGradingSummary {
    pub earthwork: GeoEarthworkSummary,
    pub daylight_perimeter_m: f64,
    pub slope_surface_area_m2: f64,
    pub berm_surface_area_m2: f64,
    pub base_pad_area_m2: f64,
    pub actual_stages_used: u32,
    pub daylight_vertex_count: u32,
    pub daylight_converged: bool,
    pub elapsed_seconds: f64,
}

pub type GeoProjectHandle = *mut c_void;
pub type GeoProgressCallback = Option<extern "C" fn(f64, u64, *mut c_void) -> bool>;

/// 动态加载的 C++20 几何内核函数表
pub struct GeoKernelLib {
    _lib: Library,
    pub geo_get_last_error: unsafe extern "C" fn() -> *const c_char,
    pub geo_project_create: unsafe extern "C" fn() -> GeoProjectHandle,
    pub geo_project_destroy: unsafe extern "C" fn(GeoProjectHandle),
    pub geo_default_dat_load_params: unsafe extern "C" fn() -> GeoDatLoadParams,
    pub geo_project_load_dat: unsafe extern "C" fn(
        GeoProjectHandle,
        i32,
        *const c_char,
        *const GeoDatLoadParams,
        GeoProgressCallback,
        *mut c_void,
        *mut GeoDatSummary,
    ) -> GeoStatus,
    pub geo_default_octree_params: unsafe extern "C" fn() -> GeoOctreeParams,
    pub geo_project_build_octree: unsafe extern "C" fn(
        GeoProjectHandle,
        i32,
        *const GeoOctreeParams,
        *mut GeoOctreeSummary,
    ) -> GeoStatus,
    pub geo_project_query_visible_octree_nodes: unsafe extern "C" fn(
        GeoProjectHandle,
        i32,
        *const GeoCameraQuery,
        *mut u32,
        u32,
        *mut u32,
        *mut u64,
    ) -> GeoStatus,
    pub geo_project_fetch_octree_node_gpu_packet: unsafe extern "C" fn(
        GeoProjectHandle,
        i32,
        u32,
        *mut u8,
        usize,
        *mut usize,
    ) -> GeoStatus,
    pub geo_default_tin_params: unsafe extern "C" fn() -> GeoTinParams,
    pub geo_project_build_tin: unsafe extern "C" fn(
        GeoProjectHandle,
        i32,
        *const GeoTinParams,
        GeoProgressCallback,
        *mut c_void,
        *mut GeoTinSummary,
    ) -> GeoStatus,
    pub geo_project_query_elevation:
        unsafe extern "C" fn(GeoProjectHandle, i32, f64, f64, *mut f64, *mut f64) -> GeoStatus,
    pub geo_project_get_dirty_tin_tiles:
        unsafe extern "C" fn(GeoProjectHandle, i32, *mut u32, u32, *mut u32) -> GeoStatus,
    pub geo_project_fetch_tin_tile_gpu_packet: unsafe extern "C" fn(
        GeoProjectHandle,
        i32,
        u32,
        bool,
        *mut u8,
        usize,
        *mut usize,
    ) -> GeoStatus,
    pub geo_project_pick_tin_ray: unsafe extern "C" fn(
        GeoProjectHandle,
        i32,
        *const f64,
        *const f64,
        *mut GeoRayHitResult,
    ) -> GeoStatus,
    pub geo_project_tin_flip_edge:
        unsafe extern "C" fn(GeoProjectHandle, i32, u32, i32, bool) -> GeoStatus,
    pub geo_project_tin_modify_vertex_z:
        unsafe extern "C" fn(GeoProjectHandle, i32, u32, f64) -> GeoStatus,
    pub geo_project_tin_insert_vertex:
        unsafe extern "C" fn(GeoProjectHandle, i32, *const f64, u16, bool, *mut u32) -> GeoStatus,
    pub geo_project_tin_insert_breakline:
        unsafe extern "C" fn(GeoProjectHandle, i32, *const f64, u32, u16, bool) -> GeoStatus,
    pub geo_project_tin_clip_polygon:
        unsafe extern "C" fn(GeoProjectHandle, i32, *const f64, u32, bool) -> GeoStatus,
    pub geo_project_tin_undo: unsafe extern "C" fn(GeoProjectHandle, i32) -> GeoStatus,
    pub geo_project_tin_redo: unsafe extern "C" fn(GeoProjectHandle, i32) -> GeoStatus,
    pub geo_project_tin_get_history_state:
        unsafe extern "C" fn(GeoProjectHandle, i32, *mut bool, *mut bool) -> GeoStatus,
    pub geo_project_compute_earthwork_exact: unsafe extern "C" fn(
        GeoProjectHandle,
        i32,
        i32,
        *const f64,
        u32,
        u32,
        *mut GeoEarthworkSummary,
    ) -> GeoStatus,
    pub geo_project_compute_earthwork_grid: unsafe extern "C" fn(
        GeoProjectHandle,
        i32,
        i32,
        f64,
        *const f64,
        u32,
        u32,
        *mut GeoEarthworkSummary,
    ) -> GeoStatus,
    pub geo_project_compute_slope_grading: unsafe extern "C" fn(
        GeoProjectHandle,
        i32,
        *const f64,
        u32,
        *const GeoSlopeStageSpec,
        u32,
        *const GeoGradingParams,
        bool,
        *mut f64,
        u32,
        *mut GeoGradingSummary,
    ) -> GeoStatus,
}

impl GeoKernelLib {
    /// 自动从多个候选路径加载 `GeoKernelCore` 动态链接库
    pub fn load_auto() -> Result<Self, String> {
        #[cfg(target_os = "windows")]
        let lib_name = "GeoKernelCore.dll";
        #[cfg(target_os = "macos")]
        let lib_name = "libGeoKernelCore.dylib";
        #[cfg(all(not(target_os = "windows"), not(target_os = "macos")))]
        let lib_name = "libGeoKernelCore.so";

        let mut candidates: Vec<PathBuf> = vec![PathBuf::from(lib_name)];
        if let Ok(exe_path) = std::env::current_exe() {
            if let Some(exe_dir) = exe_path.parent() {
                candidates.push(exe_dir.join(lib_name));
                candidates.push(exe_dir.join("../lib").join(lib_name));
                candidates.push(exe_dir.join("../../build/lib").join(lib_name));
                candidates.push(exe_dir.join("../../build/bin/Release").join(lib_name));
            }
        }

        let mut last_err = String::new();
        for cand in candidates {
            if let Ok(lib) = unsafe { Library::new(&cand) } {
                return unsafe { Self::bind_symbols(lib) };
            } else if let Err(e) = unsafe { Library::new(&cand) } {
                last_err = format!("尝试加载 {:?} 失败: {}", cand, e);
            }
        }

        Err(format!(
            "未能找到或加载几何内核动态库 {} ({})",
            lib_name, last_err
        ))
    }

    unsafe fn bind_symbols(lib: Library) -> Result<Self, String> {
        macro_rules! load_sym {
            ($name:ident, $ty:ty) => {{
                let sym: Symbol<$ty> = lib
                    .get(concat!(stringify!($name), "\0").as_bytes())
                    .map_err(|e| format!("绑定内核符号 {} 失败: {}", stringify!($name), e))?;
                *sym
            }};
        }

        Ok(Self {
            geo_get_last_error: load_sym!(geo_get_last_error, unsafe extern "C" fn() -> *const c_char),
            geo_project_create: load_sym!(geo_project_create, unsafe extern "C" fn() -> GeoProjectHandle),
            geo_project_destroy: load_sym!(geo_project_destroy, unsafe extern "C" fn(GeoProjectHandle)),
            geo_default_dat_load_params: load_sym!(geo_default_dat_load_params, unsafe extern "C" fn() -> GeoDatLoadParams),
            geo_project_load_dat: load_sym!(
                geo_project_load_dat,
                unsafe extern "C" fn(GeoProjectHandle, i32, *const c_char, *const GeoDatLoadParams, GeoProgressCallback, *mut c_void, *mut GeoDatSummary) -> GeoStatus
            ),
            geo_default_octree_params: load_sym!(geo_default_octree_params, unsafe extern "C" fn() -> GeoOctreeParams),
            geo_project_build_octree: load_sym!(
                geo_project_build_octree,
                unsafe extern "C" fn(GeoProjectHandle, i32, *const GeoOctreeParams, *mut GeoOctreeSummary) -> GeoStatus
            ),
            geo_project_query_visible_octree_nodes: load_sym!(
                geo_project_query_visible_octree_nodes,
                unsafe extern "C" fn(GeoProjectHandle, i32, *const GeoCameraQuery, *mut u32, u32, *mut u32, *mut u64) -> GeoStatus
            ),
            geo_project_fetch_octree_node_gpu_packet: load_sym!(
                geo_project_fetch_octree_node_gpu_packet,
                unsafe extern "C" fn(GeoProjectHandle, i32, u32, *mut u8, usize, *mut usize) -> GeoStatus
            ),
            geo_default_tin_params: load_sym!(geo_default_tin_params, unsafe extern "C" fn() -> GeoTinParams),
            geo_project_build_tin: load_sym!(
                geo_project_build_tin,
                unsafe extern "C" fn(GeoProjectHandle, i32, *const GeoTinParams, GeoProgressCallback, *mut c_void, *mut GeoTinSummary) -> GeoStatus
            ),
            geo_project_query_elevation: load_sym!(
                geo_project_query_elevation,
                unsafe extern "C" fn(GeoProjectHandle, i32, f64, f64, *mut f64, *mut f64) -> GeoStatus
            ),
            geo_project_get_dirty_tin_tiles: load_sym!(
                geo_project_get_dirty_tin_tiles,
                unsafe extern "C" fn(GeoProjectHandle, i32, *mut u32, u32, *mut u32) -> GeoStatus
            ),
            geo_project_fetch_tin_tile_gpu_packet: load_sym!(
                geo_project_fetch_tin_tile_gpu_packet,
                unsafe extern "C" fn(GeoProjectHandle, i32, u32, bool, *mut u8, usize, *mut usize) -> GeoStatus
            ),
            geo_project_pick_tin_ray: load_sym!(
                geo_project_pick_tin_ray,
                unsafe extern "C" fn(GeoProjectHandle, i32, *const f64, *const f64, *mut GeoRayHitResult) -> GeoStatus
            ),
            geo_project_tin_flip_edge: load_sym!(
                geo_project_tin_flip_edge,
                unsafe extern "C" fn(GeoProjectHandle, i32, u32, i32, bool) -> GeoStatus
            ),
            geo_project_tin_modify_vertex_z: load_sym!(
                geo_project_tin_modify_vertex_z,
                unsafe extern "C" fn(GeoProjectHandle, i32, u32, f64) -> GeoStatus
            ),
            geo_project_tin_insert_vertex: load_sym!(
                geo_project_tin_insert_vertex,
                unsafe extern "C" fn(GeoProjectHandle, i32, *const f64, u16, bool, *mut u32) -> GeoStatus
            ),
            geo_project_tin_insert_breakline: load_sym!(
                geo_project_tin_insert_breakline,
                unsafe extern "C" fn(GeoProjectHandle, i32, *const f64, u32, u16, bool) -> GeoStatus
            ),
            geo_project_tin_clip_polygon: load_sym!(
                geo_project_tin_clip_polygon,
                unsafe extern "C" fn(GeoProjectHandle, i32, *const f64, u32, bool) -> GeoStatus
            ),
            geo_project_tin_undo: load_sym!(
                geo_project_tin_undo,
                unsafe extern "C" fn(GeoProjectHandle, i32) -> GeoStatus
            ),
            geo_project_tin_redo: load_sym!(
                geo_project_tin_redo,
                unsafe extern "C" fn(GeoProjectHandle, i32) -> GeoStatus
            ),
            geo_project_tin_get_history_state: load_sym!(
                geo_project_tin_get_history_state,
                unsafe extern "C" fn(GeoProjectHandle, i32, *mut bool, *mut bool) -> GeoStatus
            ),
            geo_project_compute_earthwork_exact: load_sym!(
                geo_project_compute_earthwork_exact,
                unsafe extern "C" fn(GeoProjectHandle, i32, i32, *const f64, u32, u32, *mut GeoEarthworkSummary) -> GeoStatus
            ),
            geo_project_compute_earthwork_grid: load_sym!(
                geo_project_compute_earthwork_grid,
                unsafe extern "C" fn(GeoProjectHandle, i32, i32, f64, *const f64, u32, u32, *mut GeoEarthworkSummary) -> GeoStatus
            ),
            geo_project_compute_slope_grading: load_sym!(
                geo_project_compute_slope_grading,
                unsafe extern "C" fn(GeoProjectHandle, i32, *const f64, u32, *const GeoSlopeStageSpec, u32, *const GeoGradingParams, bool, *mut f64, u32, *mut GeoGradingSummary) -> GeoStatus
            ),
            _lib: lib,
        })
    }

    pub fn last_error_string(&self) -> String {
        unsafe {
            let ptr = (self.geo_get_last_error)();
            if ptr.is_null() {
                return "未知几何内核错误".to_string();
            }
            CStr::from_ptr(ptr).to_string_lossy().into_owned()
        }
    }

    pub fn check_status(&self, st: GeoStatus) -> Result<(), String> {
        if st == GeoStatus::Ok {
            Ok(())
        } else {
            Err(format!("{} (状态码: {:?})", self.last_error_string(), st))
        }
    }
}

/// 线程安全的工程句柄 RAII 包装器（C++ 内核内部已使用 std::shared_mutex 保护并发访问）
pub struct SafeGeoProject {
    pub lib: Arc<GeoKernelLib>,
    pub handle: GeoProjectHandle,
}

unsafe impl Send for SafeGeoProject {}
unsafe impl Sync for SafeGeoProject {}

impl SafeGeoProject {
    pub fn new(lib: Arc<GeoKernelLib>) -> Result<Self, String> {
        let handle = unsafe { (lib.geo_project_create)() };
        if handle.is_null() {
            return Err(lib.last_error_string());
        }
        Ok(Self { lib, handle })
    }

    pub fn load_dat(
        &self,
        slot: i32,
        file_path: &str,
        coord_order: i32,
    ) -> Result<GeoDatSummary, String> {
        let c_path = CString::new(file_path).map_err(|_| "文件路径包含非法空字符".to_string())?;
        let mut params = unsafe { (self.lib.geo_default_dat_load_params)() };
        params.coord_order = coord_order;
        let mut summary = GeoDatSummary::default();

        let st = unsafe {
            (self.lib.geo_project_load_dat)(
                self.handle,
                slot,
                c_path.as_ptr(),
                &params,
                None,
                ptr::null_mut(),
                &mut summary,
            )
        };
        self.lib.check_status(st)?;
        Ok(summary)
    }

    pub fn fetch_octree_packet(&self, slot: i32, node_id: u32) -> Result<Vec<u8>, String> {
        let mut byte_size: usize = 0;
        let st1 = unsafe {
            (self.lib.geo_project_fetch_octree_node_gpu_packet)(
                self.handle,
                slot,
                node_id,
                ptr::null_mut(),
                0,
                &mut byte_size,
            )
        };
        self.lib.check_status(st1)?;

        let mut buf = vec![0u8; byte_size];
        if byte_size > 0 {
            let st2 = unsafe {
                (self.lib.geo_project_fetch_octree_node_gpu_packet)(
                    self.handle,
                    slot,
                    node_id,
                    buf.as_mut_ptr(),
                    buf.len(),
                    &mut byte_size,
                )
            };
            self.lib.check_status(st2)?;
        }
        Ok(buf)
    }

    pub fn fetch_tin_tile_packet(
        &self,
        slot: i32,
        tile_index: u32,
        clear_dirty: bool,
    ) -> Result<Vec<u8>, String> {
        let mut byte_size: usize = 0;
        let st1 = unsafe {
            (self.lib.geo_project_fetch_tin_tile_gpu_packet)(
                self.handle,
                slot,
                tile_index,
                false,
                ptr::null_mut(),
                0,
                &mut byte_size,
            )
        };
        self.lib.check_status(st1)?;

        let mut buf = vec![0u8; byte_size];
        if byte_size > 0 {
            let st2 = unsafe {
                (self.lib.geo_project_fetch_tin_tile_gpu_packet)(
                    self.handle,
                    slot,
                    tile_index,
                    clear_dirty,
                    buf.as_mut_ptr(),
                    buf.len(),
                    &mut byte_size,
                )
            };
            self.lib.check_status(st2)?;
        }
        Ok(buf)
    }
}

impl Drop for SafeGeoProject {
    fn drop(&mut self) {
        if !self.handle.is_null() {
            unsafe {
                (self.lib.geo_project_destroy)(self.handle);
            }
            self.handle = ptr::null_mut();
        }
    }
}

