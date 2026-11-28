//! 空间几何内核 `GeoKernelCore` 动态库 FFI 安全封装层
//!
//! 设计与企业级健壮性规范：
//!   1. 所有 `#[repr(C)]` 结构体与 `include/geo_kernel/geo_kernel_c_api.h` 严格逐字节对齐；
//!   2. 严格同级目录加载策略：仅从当前桌面主程序（`geo-studio-3d.exe`）所在同级目录加载
//!      `GeoKernelCore.dll`（Linux 为 `libGeoKernelCore.so`，macOS 为 `libGeoKernelCore.dylib`），
//!      绝不跨目录盲目搜寻；
//!   3. 完整绑定任意期数动态槽位管理接口（创建期数槽位、移除期数槽位、清空全部期数、枚举活跃槽位、
//!      获取统一工程主 RTC 原点）以及全套点云、八叉树、三角网、拓扑编辑、土方与放坡 C ABI；
//!   4. 暴露线程安全的 `SafeGeoProject` RAII 句柄包装，析构时自动调用 `geo_project_destroy`
//!      回收全部 C++20 原生内存。

use libloading::{Library, Symbol};
use serde::{Deserialize, Serialize};
use std::ffi::{CStr, CString};
use std::os::raw::{c_char, c_void};
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
pub type GeoLogCallback = Option<
    extern "C" fn(
        level: i32,
        module_name: *const c_char,
        source_location: *const c_char,
        message: *const c_char,
        user_data: *mut c_void,
    ),
>;

/// 动态加载的 C++20 空间几何内核完整函数指针表
pub struct GeoKernelLib {
    _lib: Library,
    pub geo_get_last_error: unsafe extern "C" fn() -> *const c_char,
    pub geo_set_log_callback: Option<unsafe extern "C" fn(GeoLogCallback, *mut c_void)>,
    pub geo_project_create: unsafe extern "C" fn() -> GeoProjectHandle,
    pub geo_project_destroy: unsafe extern "C" fn(GeoProjectHandle),
    pub geo_project_create_period_slot:
        unsafe extern "C" fn(GeoProjectHandle, *mut i32) -> GeoStatus,
    pub geo_project_remove_slot: unsafe extern "C" fn(GeoProjectHandle, i32) -> GeoStatus,
    pub geo_project_clear_all_periods: unsafe extern "C" fn(GeoProjectHandle) -> GeoStatus,
    pub geo_project_list_slots:
        unsafe extern "C" fn(GeoProjectHandle, *mut i32, u32, *mut u32) -> GeoStatus,
    pub geo_project_get_master_rtc:
        unsafe extern "C" fn(GeoProjectHandle, *mut f64, *mut bool) -> GeoStatus,
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
    pub geo_project_tin_delete_vertex:
        unsafe extern "C" fn(GeoProjectHandle, i32, u32) -> GeoStatus,
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
    /// 严格从当前主程序所在同级目录加载 `GeoKernelCore` 核心动态链接库
    pub fn load_auto<F>(mut log_fn: F) -> Result<Self, String>
    where
        F: FnMut(&str),
    {
        #[cfg(target_os = "windows")]
        let lib_name = "GeoKernelCore.dll";
        #[cfg(target_os = "macos")]
        let lib_name = "libGeoKernelCore.dylib";
        #[cfg(all(not(target_os = "windows"), not(target_os = "macos")))]
        let lib_name = "libGeoKernelCore.so";

        let exe_path = std::env::current_exe()
            .map_err(|e| format!("无法获取当前主程序可执行文件路径: {}", e))?;
        let exe_dir = exe_path
            .parent()
            .ok_or_else(|| "无法解析当前主程序所在父目录".to_string())?;

        let exact_dll_path = exe_dir.join(lib_name);
        log_fn(&format!(
            "正在检测核心几何内核动态库: {}",
            exact_dll_path.display()
        ));

        if !exact_dll_path.is_file() {
            return Err(format!(
                "同级目录下缺少核心几何内核动态库 {}。\n期望精确路径: {}\n请确保 {} 与主程序放在同一文件夹内。",
                lib_name,
                exact_dll_path.display(),
                lib_name
            ));
        }

        let lib = unsafe { Library::new(&exact_dll_path) }.map_err(|err| {
            format!(
                "加载同级目录核心动态库失败。\n目标路径: {}\n系统错误详情: {}",
                exact_dll_path.display(),
                err
            )
        })?;

        log_fn("核心动态库装载成功，开始绑定全部多期 C ABI 符号...");
        let bound = unsafe { Self::bind_symbols(lib)? };
        log_fn("C++20 核心动态库全部 ABI 符号绑定完成");
        Ok(bound)
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

        let log_cb_sym: Option<unsafe extern "C" fn(GeoLogCallback, *mut c_void)> = lib
            .get::<unsafe extern "C" fn(GeoLogCallback, *mut c_void)>(b"geo_set_log_callback\0")
            .ok()
            .map(|s| *s);

        Ok(Self {
            geo_get_last_error: load_sym!(
                geo_get_last_error,
                unsafe extern "C" fn() -> *const c_char
            ),
            geo_set_log_callback: log_cb_sym,
            geo_project_create: load_sym!(
                geo_project_create,
                unsafe extern "C" fn() -> GeoProjectHandle
            ),
            geo_project_destroy: load_sym!(
                geo_project_destroy,
                unsafe extern "C" fn(GeoProjectHandle)
            ),
            geo_project_create_period_slot: load_sym!(
                geo_project_create_period_slot,
                unsafe extern "C" fn(GeoProjectHandle, *mut i32) -> GeoStatus
            ),
            geo_project_remove_slot: load_sym!(
                geo_project_remove_slot,
                unsafe extern "C" fn(GeoProjectHandle, i32) -> GeoStatus
            ),
            geo_project_clear_all_periods: load_sym!(
                geo_project_clear_all_periods,
                unsafe extern "C" fn(GeoProjectHandle) -> GeoStatus
            ),
            geo_project_list_slots: load_sym!(
                geo_project_list_slots,
                unsafe extern "C" fn(GeoProjectHandle, *mut i32, u32, *mut u32) -> GeoStatus
            ),
            geo_project_get_master_rtc: load_sym!(
                geo_project_get_master_rtc,
                unsafe extern "C" fn(GeoProjectHandle, *mut f64, *mut bool) -> GeoStatus
            ),
            geo_default_dat_load_params: load_sym!(
                geo_default_dat_load_params,
                unsafe extern "C" fn() -> GeoDatLoadParams
            ),
            geo_project_load_dat: load_sym!(
                geo_project_load_dat,
                unsafe extern "C" fn(
                    GeoProjectHandle,
                    i32,
                    *const c_char,
                    *const GeoDatLoadParams,
                    GeoProgressCallback,
                    *mut c_void,
                    *mut GeoDatSummary,
                ) -> GeoStatus
            ),
            geo_default_octree_params: load_sym!(
                geo_default_octree_params,
                unsafe extern "C" fn() -> GeoOctreeParams
            ),
            geo_project_build_octree: load_sym!(
                geo_project_build_octree,
                unsafe extern "C" fn(
                    GeoProjectHandle,
                    i32,
                    *const GeoOctreeParams,
                    *mut GeoOctreeSummary,
                ) -> GeoStatus
            ),
            geo_project_query_visible_octree_nodes: load_sym!(
                geo_project_query_visible_octree_nodes,
                unsafe extern "C" fn(
                    GeoProjectHandle,
                    i32,
                    *const GeoCameraQuery,
                    *mut u32,
                    u32,
                    *mut u32,
                    *mut u64,
                ) -> GeoStatus
            ),
            geo_project_fetch_octree_node_gpu_packet: load_sym!(
                geo_project_fetch_octree_node_gpu_packet,
                unsafe extern "C" fn(
                    GeoProjectHandle,
                    i32,
                    u32,
                    *mut u8,
                    usize,
                    *mut usize,
                ) -> GeoStatus
            ),
            geo_default_tin_params: load_sym!(
                geo_default_tin_params,
                unsafe extern "C" fn() -> GeoTinParams
            ),
            geo_project_build_tin: load_sym!(
                geo_project_build_tin,
                unsafe extern "C" fn(
                    GeoProjectHandle,
                    i32,
                    *const GeoTinParams,
                    GeoProgressCallback,
                    *mut c_void,
                    *mut GeoTinSummary,
                ) -> GeoStatus
            ),
            geo_project_query_elevation: load_sym!(
                geo_project_query_elevation,
                unsafe extern "C" fn(
                    GeoProjectHandle,
                    i32,
                    f64,
                    f64,
                    *mut f64,
                    *mut f64,
                ) -> GeoStatus
            ),
            geo_project_get_dirty_tin_tiles: load_sym!(
                geo_project_get_dirty_tin_tiles,
                unsafe extern "C" fn(GeoProjectHandle, i32, *mut u32, u32, *mut u32) -> GeoStatus
            ),
            geo_project_fetch_tin_tile_gpu_packet: load_sym!(
                geo_project_fetch_tin_tile_gpu_packet,
                unsafe extern "C" fn(
                    GeoProjectHandle,
                    i32,
                    u32,
                    bool,
                    *mut u8,
                    usize,
                    *mut usize,
                ) -> GeoStatus
            ),
            geo_project_pick_tin_ray: load_sym!(
                geo_project_pick_tin_ray,
                unsafe extern "C" fn(
                    GeoProjectHandle,
                    i32,
                    *const f64,
                    *const f64,
                    *mut GeoRayHitResult,
                ) -> GeoStatus
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
                unsafe extern "C" fn(
                    GeoProjectHandle,
                    i32,
                    *const f64,
                    u16,
                    bool,
                    *mut u32,
                ) -> GeoStatus
            ),
            geo_project_tin_insert_breakline: load_sym!(
                geo_project_tin_insert_breakline,
                unsafe extern "C" fn(
                    GeoProjectHandle,
                    i32,
                    *const f64,
                    u32,
                    u16,
                    bool,
                ) -> GeoStatus
            ),
            geo_project_tin_delete_vertex: load_sym!(
                geo_project_tin_delete_vertex,
                unsafe extern "C" fn(GeoProjectHandle, i32, u32) -> GeoStatus
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
                unsafe extern "C" fn(
                    GeoProjectHandle,
                    i32,
                    i32,
                    *const f64,
                    u32,
                    u32,
                    *mut GeoEarthworkSummary,
                ) -> GeoStatus
            ),
            geo_project_compute_earthwork_grid: load_sym!(
                geo_project_compute_earthwork_grid,
                unsafe extern "C" fn(
                    GeoProjectHandle,
                    i32,
                    i32,
                    f64,
                    *const f64,
                    u32,
                    u32,
                    *mut GeoEarthworkSummary,
                ) -> GeoStatus
            ),
            geo_project_compute_slope_grading: load_sym!(
                geo_project_compute_slope_grading,
                unsafe extern "C" fn(
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
                ) -> GeoStatus
            ),
            _lib: lib,
        })
    }

    pub fn set_log_callback(&self, cb: GeoLogCallback) {
        if let Some(func) = self.geo_set_log_callback {
            unsafe {
                func(cb, ptr::null_mut());
            }
        }
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

/// 线程安全的工程句柄 RAII 包装器
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

    /// 申请一个新的动态测绘期数槽位号
    pub fn create_period_slot(&self) -> Result<i32, String> {
        let mut slot_id: i32 = -1;
        let st = unsafe { (self.lib.geo_project_create_period_slot)(self.handle, &mut slot_id) };
        self.lib.check_status(st)?;
        Ok(slot_id)
    }

    /// 卸载并释放指定期数槽位的全部内存资源
    pub fn remove_slot(&self, slot: i32) -> Result<(), String> {
        let st = unsafe { (self.lib.geo_project_remove_slot)(self.handle, slot) };
        self.lib.check_status(st)
    }

    /// 清空全部期数与派生数据
    pub fn clear_all_periods(&self) -> Result<(), String> {
        let st = unsafe { (self.lib.geo_project_clear_all_periods)(self.handle) };
        self.lib.check_status(st)
    }

    /// 枚举当前所有活跃槽位号
    pub fn list_slots(&self) -> Result<Vec<i32>, String> {
        let mut count: u32 = 0;
        let st1 = unsafe {
            (self.lib.geo_project_list_slots)(self.handle, ptr::null_mut(), 0, &mut count)
        };
        self.lib.check_status(st1)?;
        let mut ids = vec![0i32; count as usize];
        if count > 0 {
            let st2 = unsafe {
                (self.lib.geo_project_list_slots)(
                    self.handle,
                    ids.as_mut_ptr(),
                    count,
                    &mut count,
                )
            };
            self.lib.check_status(st2)?;
        }
        Ok(ids)
    }

    /// 导入指定期数的 .dat 点云文件并自动对齐至工程主 RTC 原点
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

    /// 零拷贝两段式拉取八叉树点云节点 GPU 二进制数据包
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

    /// 零拷贝两段式拉取三角网空间瓦片 GPU 二进制网格包
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

