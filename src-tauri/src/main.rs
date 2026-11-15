//! GeoStudio 3D Tauri v2 桌面端后端宿主入口
//!
//! 健壮性与诊断设计：
//!   1. Windows UCRT 宽窄流模式断言彻底防护（修复 corecrt_internal_stdio.h:495 崩溃）：
//!      - 启动第一时间将有效的标准流描述符强制锁定为 `_O_TEXT`（字节透传模式），绝不设置
//!        `_O_U8TEXT` / `_O_U16TEXT`；
//!      - 所有控制台中文日志均通过 Win32 原生 `WriteConsoleW` / `WriteFile` 输出，文件日志均以
//!        二进制字节追加模式写入 `logs/geostudio3d.log`，100% 绕开 UCRT `FILE*` 宽窄字符断言；
//!      - 统一接管 C++20 几何内核的 `geo_set_log_callback` 回调，前后端日志汇聚至同一文件与终端。
//!   2. 启动失败原生弹窗兜底：无控制台窗口模式下双击运行时若发生异常，弹出模态错误对话框。

#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")]

mod kernel_ffi;

use kernel_ffi::{
    GeoCameraQuery, GeoDatSummary, GeoEarthworkSummary, GeoGradingParams, GeoGradingSummary,
    GeoKernelLib, GeoOctreeSummary, GeoRayHitResult, GeoSlopeStageSpec, GeoTinSummary,
    SafeGeoProject,
};
use serde::{Deserialize, Serialize};
use std::ffi::CStr;
use std::fs::{self, File, OpenOptions};
use std::io::Write;
use std::os::raw::{c_char, c_int, c_void};
use std::ptr;
use std::sync::{Arc, Mutex, OnceLock};
use std::time::{SystemTime, UNIX_EPOCH};
use tauri::http::{header::CONTENT_TYPE, Response, StatusCode};
use tauri::State;

// ==============================================================================
// 1. 跨平台企业级双通道 UTF-8 日志系统（绕过 UCRT FILE* 宽窄字符断言）
// ==============================================================================

static GLOBAL_LOG_FILE: OnceLock<Mutex<Option<File>>> = OnceLock::new();

/// Windows 启动期标准流与 UTF-8 控制台初始化守卫
#[cfg(target_os = "windows")]
fn init_windows_stdio_safety() {
    const _O_TEXT: c_int = 0x4000;
    const STD_OUTPUT_HANDLE: u32 = 0xFFFF_FFF5;
    const STD_ERROR_HANDLE: u32 = 0xFFFF_FFF4;
    const CP_UTF8: u32 = 65001;

    extern "C" {
        fn __acrt_iob_func(ix: u32) -> *mut c_void;
        fn _fileno(stream: *mut c_void) -> c_int;
        fn _get_osfhandle(fd: c_int) -> isize;
        fn _setmode(fd: c_int, mode: c_int) -> c_int;
    }

    #[link(name = "kernel32")]
    extern "system" {
        fn SetConsoleOutputCP(wCodePageID: u32) -> i32;
        fn SetConsoleCP(wCodePageID: u32) -> i32;
        fn GetStdHandle(nStdHandle: u32) -> isize;
    }

    unsafe {
        // 设置控制台代码页为 UTF-8，确保中文字符正常显示
        let _ = SetConsoleOutputCP(CP_UTF8);
        let _ = SetConsoleCP(CP_UTF8);

        // 仅当标准输出/标准错误拥有有效操作系统句柄时，将其锁定为 _O_TEXT（防止 ucrtbased.dll 断言）
        for (idx, std_id) in [(1u32, STD_OUTPUT_HANDLE), (2u32, STD_ERROR_HANDLE)] {
            let h = GetStdHandle(std_id);
            if h != 0 && h != -1 {
                let stream = __acrt_iob_func(idx);
                if !stream.is_null() {
                    let fd = _fileno(stream);
                    if fd >= 0 {
                        let osf = _get_osfhandle(fd);
                        if osf != 0 && osf != -1 {
                            let _ = _setmode(fd, _O_TEXT);
                        }
                    }
                }
            }
        }
    }
}

#[cfg(not(target_os = "windows"))]
fn init_windows_stdio_safety() {}

/// 格式化当前 UTC/本地时间戳字符串 `YYYY-MM-DD HH:MM:SS.mmm`
fn format_timestamp_now() -> String {
    let now = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default();
    let total_secs = now.as_secs();
    let millis = now.subsec_millis();

    let days = (total_secs / 86400) as i64;
    let day_secs = total_secs % 86400;
    let hour = day_secs / 3600;
    let minute = (day_secs % 3600) / 60;
    let second = day_secs % 60;

    // Howard Hinnant 民用历法转换算法（无外部时间库依赖）
    let z = days + 719468;
    let era = if z >= 0 { z } else { z - 146096 } / 146097;
    let doe = (z - era * 146097) as u64;
    let yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
    let y = (yoe as i64) + era * 400;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let d = doy - (153 * mp + 2) / 5 + 1;
    let m = if mp < 10 { mp + 3 } else { mp - 9 };
    let year = if m <= 2 { y + 1 } else { y };

    format!(
        "{:04}-{:02}-{:02} {:02}:{:02}:{:02}.{:03}",
        year, m, d, hour, minute, second, millis
    )
}

/// 通过 Win32 原生 API 输出 UTF-8 文本到标准输出/标准错误（完全绕过 UCRT FILE* 断言）
fn write_console_utf8(line: &str) {
    #[cfg(target_os = "windows")]
    {
        use std::ffi::OsStr;
        use std::os::windows::ffi::OsStrExt;

        const STD_OUTPUT_HANDLE: u32 = 0xFFFF_FFF5;

        #[link(name = "kernel32")]
        extern "system" {
            fn GetStdHandle(nStdHandle: u32) -> isize;
            fn GetConsoleMode(hConsoleHandle: isize, lpMode: *mut u32) -> i32;
            fn WriteConsoleW(
                hConsoleOutput: isize,
                lpBuffer: *const u16,
                nNumberOfCharsToWrite: u32,
                lpNumberOfCharsWritten: *mut u32,
                lpReserved: *mut c_void,
            ) -> i32;
            fn WriteFile(
                hFile: isize,
                lpBuffer: *const u8,
                nNumberOfBytesToWrite: u32,
                lpNumberOfBytesWritten: *mut u32,
                lpOverlapped: *mut c_void,
            ) -> i32;
        }

        unsafe {
            let h_out = GetStdHandle(STD_OUTPUT_HANDLE);
            if h_out != 0 && h_out != -1 {
                let mut mode: u32 = 0;
                if GetConsoleMode(h_out, &mut mode) != 0 {
                    let wide: Vec<u16> = OsStr::new(line).encode_wide().collect();
                    let mut written: u32 = 0;
                    let _ = WriteConsoleW(
                        h_out,
                        wide.as_ptr(),
                        wide.len() as u32,
                        &mut written,
                        ptr::null_mut(),
                    );
                } else {
                    let bytes = line.as_bytes();
                    let mut written: u32 = 0;
                    let _ = WriteFile(
                        h_out,
                        bytes.as_ptr(),
                        bytes.len() as u32,
                        &mut written,
                        ptr::null_mut(),
                    );
                }
            }
        }
    }

    #[cfg(not(target_os = "windows"))]
    {
        let _ = std::io::stdout().write_all(line.as_bytes());
        let _ = std::io::stdout().flush();
    }
}

fn init_app_logger() {
    init_windows_stdio_safety();

    let log_file = std::env::current_exe()
        .ok()
        .and_then(|p| p.parent().map(|d| d.join("logs")))
        .and_then(|log_dir| {
            let _ = fs::create_dir_all(&log_dir);
            OpenOptions::new()
                .create(true)
                .append(true)
                .open(log_dir.join("geostudio3d.log"))
                .ok()
        });

    let _ = GLOBAL_LOG_FILE.set(Mutex::new(log_file));
}

fn app_log(level: &str, module: &str, msg: &str) {
    let ts = format_timestamp_now();
    let tid = std::thread::current().id();
    let formatted = format!("[{}] [{:<5}] [{:?}] [{}] {}\n", ts, level, tid, module, msg);

    write_console_utf8(&formatted);

    if let Some(lock) = GLOBAL_LOG_FILE.get() {
        if let Ok(mut guard) = lock.lock() {
            if let Some(ref mut file) = *guard {
                let _ = file.write_all(formatted.as_bytes());
                let _ = file.flush();
            }
        }
    }
}

/// C++20 几何内核日志回调桥接函数
extern "C" fn cpp_kernel_log_bridge(
    level: i32,
    module_name: *const c_char,
    source_location: *const c_char,
    message: *const c_char,
    _user_data: *mut c_void,
) {
    let level_str = match level {
        0 => "DEBUG",
        1 => "INFO",
        2 => "WARN",
        _ => "ERROR",
    };
    let mod_str = if module_name.is_null() {
        "CPP"
    } else {
        unsafe { CStr::from_ptr(module_name).to_str().unwrap_or("CPP") }
    };
    let loc_str = if source_location.is_null() {
        ""
    } else {
        unsafe { CStr::from_ptr(source_location).to_str().unwrap_or("") }
    };
    let msg_str = if message.is_null() {
        ""
    } else {
        unsafe { CStr::from_ptr(message).to_str().unwrap_or("") }
    };

    let full_msg = if loc_str.is_empty() {
        msg_str.to_string()
    } else {
        format!("({}) {}", loc_str, msg_str)
    };
    app_log(level_str, mod_str, &full_msg);
}

/// 跨平台原生启动错误提示弹窗（确保双击启动失败时用户可见明确诊断原因）
fn show_native_fatal_dialog(title: &str, message: &str) {
    app_log("ERROR", "FATAL", &format!("{}: {}", title, message));

    #[cfg(target_os = "windows")]
    {
        use std::ffi::OsStr;
        use std::os::windows::ffi::OsStrExt;

        #[link(name = "user32")]
        extern "system" {
            fn MessageBoxW(
                hwnd: *mut c_void,
                text: *const u16,
                caption: *const u16,
                utype: u32,
            ) -> i32;
        }

        let wide_msg: Vec<u16> = OsStr::new(message)
            .encode_wide()
            .chain(std::iter::once(0))
            .collect();
        let wide_title: Vec<u16> = OsStr::new(title)
            .encode_wide()
            .chain(std::iter::once(0))
            .collect();

        // MB_OK (0x0) | MB_ICONERROR (0x10) | MB_SYSTEMMODAL (0x1000)
        unsafe {
            MessageBoxW(
                ptr::null_mut(),
                wide_msg.as_ptr(),
                wide_title.as_ptr(),
                0x0000_1010,
            );
        }
    }

    #[cfg(target_os = "macos")]
    {
        let escaped = message.replace('"', "\\\"");
        let script = format!(
            "display dialog \"{}\" with title \"{}\" buttons {{\"确定\"}} default button \"确定\" with icon stop",
            escaped, title
        );
        let _ = std::process::Command::new("osascript")
            .args(["-e", &script])
            .status();
    }

    #[cfg(all(not(target_os = "windows"), not(target_os = "macos")))]
    {
        let _ = std::process::Command::new("zenity")
            .args(["--error", "--title", title, "--text", message])
            .status()
            .or_else(|_| {
                std::process::Command::new("kdialog")
                    .args(["--error", message, "--title", title])
                    .status()
            });
    }
}

// ==============================================================================
// 2. Tauri 应用状态与数据传输对象 (DTO)
// ==============================================================================

struct AppKernelState {
    project: Arc<SafeGeoProject>,
}

#[derive(Debug, Deserialize)]
#[serde(rename_all = "camelCase")]
struct CameraQueryDto {
    camera_local_pos: [f64; 3],
    frustum_planes: Vec<[f64; 4]>,
    viewport_height_px: f64,
    fov_y_radians: f64,
    sse_threshold_px: f64,
    max_point_budget: u64,
    enable_frustum_culling: bool,
}

#[derive(Debug, Serialize)]
#[serde(rename_all = "camelCase")]
struct VisibleOctreeNodesDto {
    node_ids: Vec<u32>,
    estimated_points: u64,
}

#[derive(Debug, Serialize)]
#[serde(rename_all = "camelCase")]
struct HistoryStateDto {
    can_undo: bool,
    can_redo: bool,
}

#[derive(Debug, Serialize)]
#[serde(rename_all = "camelCase")]
struct GradingSummaryDto {
    #[serde(flatten)]
    base: GeoGradingSummary,
    daylight_polyline: Vec<[f64; 3]>,
}

// ==============================================================================
// 3. Tauri 异步业务命令集
// ==============================================================================

#[tauri::command]
async fn cmd_load_dat(
    state: State<'_, AppKernelState>,
    slot: i32,
    file_path: String,
    coord_order: i32,
) -> Result<GeoDatSummary, String> {
    app_log("INFO", "IPC", &format!("请求导入点云: slot={}, path={}", slot, file_path));
    let proj = Arc::clone(&state.project);
    tauri::async_runtime::spawn_blocking(move || proj.load_dat(slot, &file_path, coord_order))
        .await
        .map_err(|e| format!("后台任务调度异常: {}", e))?
}

#[tauri::command]
async fn cmd_build_octree(
    state: State<'_, AppKernelState>,
    slot: i32,
) -> Result<GeoOctreeSummary, String> {
    let proj = Arc::clone(&state.project);
    tauri::async_runtime::spawn_blocking(move || {
        let params = unsafe { (proj.lib.geo_default_octree_params)() };
        let mut summary = GeoOctreeSummary::default();
        let st = unsafe {
            (proj.lib.geo_project_build_octree)(proj.handle, slot, &params, &mut summary)
        };
        proj.lib.check_status(st)?;
        Ok(summary)
    })
    .await
    .map_err(|e| format!("后台任务调度异常: {}", e))?
}

#[tauri::command]
async fn cmd_build_tin(
    state: State<'_, AppKernelState>,
    slot: i32,
    decimation_grid_size: f64,
    tile_size: f64,
) -> Result<GeoTinSummary, String> {
    let proj = Arc::clone(&state.project);
    tauri::async_runtime::spawn_blocking(move || {
        let mut params = unsafe { (proj.lib.geo_default_tin_params)() };
        params.decimation_grid_size = decimation_grid_size;
        params.tile_size = tile_size;
        let mut summary = GeoTinSummary::default();
        let st = unsafe {
            (proj.lib.geo_project_build_tin)(
                proj.handle,
                slot,
                &params,
                None,
                ptr::null_mut(),
                &mut summary,
            )
        };
        proj.lib.check_status(st)?;
        Ok(summary)
    })
    .await
    .map_err(|e| format!("后台任务调度异常: {}", e))?
}

#[tauri::command]
fn cmd_query_visible_octree_nodes(
    state: State<'_, AppKernelState>,
    slot: i32,
    query: CameraQueryDto,
) -> Result<VisibleOctreeNodesDto, String> {
    if query.frustum_planes.len() != 6 {
        return Err("视锥体裁剪平面数量必须严格为 6 个".to_string());
    }
    let mut planes = [[0.0f64; 4]; 6];
    for i in 0..6 {
        planes[i] = query.frustum_planes[i];
    }

    let c_query = GeoCameraQuery {
        camera_local_pos: query.camera_local_pos,
        frustum_planes: planes,
        viewport_height_px: query.viewport_height_px,
        fov_y_radians: query.fov_y_radians,
        sse_threshold_px: query.sse_threshold_px,
        max_point_budget: query.max_point_budget,
        enable_frustum_culling: query.enable_frustum_culling,
    };

    let proj = &state.project;
    let mut count: u32 = 0;
    let mut est_pts: u64 = 0;

    let st1 = unsafe {
        (proj.lib.geo_project_query_visible_octree_nodes)(
            proj.handle,
            slot,
            &c_query,
            ptr::null_mut(),
            0,
            &mut count,
            &mut est_pts,
        )
    };
    proj.lib.check_status(st1)?;

    let mut node_ids = vec![0u32; count as usize];
    if count > 0 {
        let st2 = unsafe {
            (proj.lib.geo_project_query_visible_octree_nodes)(
                proj.handle,
                slot,
                &c_query,
                node_ids.as_mut_ptr(),
                count,
                &mut count,
                &mut est_pts,
            )
        };
        proj.lib.check_status(st2)?;
    }

    Ok(VisibleOctreeNodesDto {
        node_ids,
        estimated_points: est_pts,
    })
}

#[tauri::command]
fn cmd_get_dirty_tin_tiles(
    state: State<'_, AppKernelState>,
    slot: i32,
) -> Result<Vec<u32>, String> {
    let proj = &state.project;
    let mut count: u32 = 0;
    let st1 = unsafe {
        (proj.lib.geo_project_get_dirty_tin_tiles)(
            proj.handle,
            slot,
            ptr::null_mut(),
            0,
            &mut count,
        )
    };
    proj.lib.check_status(st1)?;

    let mut tiles = vec![0u32; count as usize];
    if count > 0 {
        let st2 = unsafe {
            (proj.lib.geo_project_get_dirty_tin_tiles)(
                proj.handle,
                slot,
                tiles.as_mut_ptr(),
                count,
                &mut count,
            )
        };
        proj.lib.check_status(st2)?;
    }
    Ok(tiles)
}

#[tauri::command]
fn cmd_pick_tin_ray(
    state: State<'_, AppKernelState>,
    slot: i32,
    ray_origin: [f64; 3],
    ray_dir: [f64; 3],
) -> Result<GeoRayHitResult, String> {
    let proj = &state.project;
    let mut hit = GeoRayHitResult::default();
    let st = unsafe {
        (proj.lib.geo_project_pick_tin_ray)(
            proj.handle,
            slot,
            ray_origin.as_ptr(),
            ray_dir.as_ptr(),
            &mut hit,
        )
    };
    proj.lib.check_status(st)?;
    Ok(hit)
}

#[tauri::command]
fn cmd_tin_flip_edge(
    state: State<'_, AppKernelState>,
    slot: i32,
    triangle_id: u32,
    edge_slot: i32,
    force_flip: bool,
) -> Result<(), String> {
    let proj = &state.project;
    let st = unsafe {
        (proj.lib.geo_project_tin_flip_edge)(
            proj.handle,
            slot,
            triangle_id,
            edge_slot,
            force_flip,
        )
    };
    proj.lib.check_status(st)
}

#[tauri::command]
fn cmd_tin_modify_vertex_z(
    state: State<'_, AppKernelState>,
    slot: i32,
    vertex_id: u32,
    new_local_z: f64,
) -> Result<(), String> {
    let proj = &state.project;
    let st = unsafe {
        (proj.lib.geo_project_tin_modify_vertex_z)(proj.handle, slot, vertex_id, new_local_z)
    };
    proj.lib.check_status(st)
}

#[tauri::command]
fn cmd_tin_insert_vertex(
    state: State<'_, AppKernelState>,
    slot: i32,
    local_xyz: [f64; 3],
    code_id: u16,
) -> Result<u32, String> {
    let proj = &state.project;
    let mut out_vid: u32 = 0;
    let st = unsafe {
        (proj.lib.geo_project_tin_insert_vertex)(
            proj.handle,
            slot,
            local_xyz.as_ptr(),
            code_id,
            true,
            &mut out_vid,
        )
    };
    proj.lib.check_status(st)?;
    Ok(out_vid)
}

#[tauri::command]
fn cmd_tin_insert_breakline(
    state: State<'_, AppKernelState>,
    slot: i32,
    polyline: Vec<[f64; 3]>,
    code_id: u16,
    interpolate_z_from_tin: bool,
) -> Result<(), String> {
    let flat: Vec<f64> = polyline.iter().flat_map(|p| p.iter().copied()).collect();
    let proj = &state.project;
    let st = unsafe {
        (proj.lib.geo_project_tin_insert_breakline)(
            proj.handle,
            slot,
            flat.as_ptr(),
            polyline.len() as u32,
            code_id,
            interpolate_z_from_tin,
        )
    };
    proj.lib.check_status(st)
}

#[tauri::command]
fn cmd_tin_clip_polygon(
    state: State<'_, AppKernelState>,
    slot: i32,
    polygon: Vec<[f64; 3]>,
    keep_inside: bool,
) -> Result<(), String> {
    let flat: Vec<f64> = polygon.iter().flat_map(|p| p.iter().copied()).collect();
    let proj = &state.project;
    let st = unsafe {
        (proj.lib.geo_project_tin_clip_polygon)(
            proj.handle,
            slot,
            flat.as_ptr(),
            polygon.len() as u32,
            keep_inside,
        )
    };
    proj.lib.check_status(st)
}

#[tauri::command]
fn cmd_tin_undo(state: State<'_, AppKernelState>, slot: i32) -> Result<(), String> {
    let proj = &state.project;
    let st = unsafe { (proj.lib.geo_project_tin_undo)(proj.handle, slot) };
    proj.lib.check_status(st)
}

#[tauri::command]
fn cmd_tin_redo(state: State<'_, AppKernelState>, slot: i32) -> Result<(), String> {
    let proj = &state.project;
    let st = unsafe { (proj.lib.geo_project_tin_redo)(proj.handle, slot) };
    proj.lib.check_status(st)
}

#[tauri::command]
fn cmd_tin_get_history_state(
    state: State<'_, AppKernelState>,
    slot: i32,
) -> Result<HistoryStateDto, String> {
    let proj = &state.project;
    let mut can_undo = false;
    let mut can_redo = false;
    let st = unsafe {
        (proj.lib.geo_project_tin_get_history_state)(
            proj.handle,
            slot,
            &mut can_undo,
            &mut can_redo,
        )
    };
    proj.lib.check_status(st)?;
    Ok(HistoryStateDto { can_undo, can_redo })
}

#[tauri::command]
async fn cmd_compute_earthwork_exact(
    state: State<'_, AppKernelState>,
    period1_slot: i32,
    period2_slot: i32,
    roi_polygon: Vec<[f64; 3]>,
) -> Result<GeoEarthworkSummary, String> {
    let proj = Arc::clone(&state.project);
    tauri::async_runtime::spawn_blocking(move || {
        let flat: Vec<f64> = roi_polygon.iter().flat_map(|p| p.iter().copied()).collect();
        let roi_ptr = if flat.is_empty() { ptr::null() } else { flat.as_ptr() };
        let mut summary = GeoEarthworkSummary::default();
        let st = unsafe {
            (proj.lib.geo_project_compute_earthwork_exact)(
                proj.handle,
                period1_slot,
                period2_slot,
                roi_ptr,
                roi_polygon.len() as u32,
                0,
                &mut summary,
            )
        };
        proj.lib.check_status(st)?;
        Ok(summary)
    })
    .await
    .map_err(|e| format!("后台任务调度异常: {}", e))?
}

#[tauri::command]
async fn cmd_compute_earthwork_grid(
    state: State<'_, AppKernelState>,
    period1_slot: i32,
    period2_slot: i32,
    grid_step: f64,
    roi_polygon: Vec<[f64; 3]>,
) -> Result<GeoEarthworkSummary, String> {
    let proj = Arc::clone(&state.project);
    tauri::async_runtime::spawn_blocking(move || {
        let flat: Vec<f64> = roi_polygon.iter().flat_map(|p| p.iter().copied()).collect();
        let roi_ptr = if flat.is_empty() { ptr::null() } else { flat.as_ptr() };
        let mut summary = GeoEarthworkSummary::default();
        let st = unsafe {
            (proj.lib.geo_project_compute_earthwork_grid)(
                proj.handle,
                period1_slot,
                period2_slot,
                grid_step,
                roi_ptr,
                roi_polygon.len() as u32,
                0,
                &mut summary,
            )
        };
        proj.lib.check_status(st)?;
        Ok(summary)
    })
    .await
    .map_err(|e| format!("后台任务调度异常: {}", e))?
}

#[tauri::command]
async fn cmd_compute_slope_grading(
    state: State<'_, AppKernelState>,
    original_slot: i32,
    base_polygon: Vec<[f64; 3]>,
    stages: Vec<GeoSlopeStageSpec>,
    params: GeoGradingParams,
    generate_merged_terrain: bool,
) -> Result<GradingSummaryDto, String> {
    let proj = Arc::clone(&state.project);
    tauri::async_runtime::spawn_blocking(move || {
        let flat_base: Vec<f64> = base_polygon.iter().flat_map(|p| p.iter().copied()).collect();
        let max_daylight_pts: u32 = 8192;
        let mut daylight_flat = vec![0.0f64; (max_daylight_pts as usize) * 3];
        let mut summary = GeoGradingSummary::default();

        let st = unsafe {
            (proj.lib.geo_project_compute_slope_grading)(
                proj.handle,
                original_slot,
                flat_base.as_ptr(),
                base_polygon.len() as u32,
                stages.as_ptr(),
                stages.len() as u32,
                &params,
                generate_merged_terrain,
                daylight_flat.as_mut_ptr(),
                max_daylight_pts,
                &mut summary,
            )
        };
        proj.lib.check_status(st)?;

        let valid_dl_count = (summary.daylight_vertex_count.min(max_daylight_pts)) as usize;
        let mut daylight_polyline = Vec::with_capacity(valid_dl_count);
        for i in 0..valid_dl_count {
            daylight_polyline.push([
                daylight_flat[i * 3],
                daylight_flat[i * 3 + 1],
                daylight_flat[i * 3 + 2],
            ]);
        }

        Ok(GradingSummaryDto {
            base: summary,
            daylight_polyline,
        })
    })
    .await
    .map_err(|e| format!("后台任务调度异常: {}", e))?
}

// ==============================================================================
// 4. Tauri 主应用启动入口（带异常保护与原生错误提示弹窗）
// ==============================================================================

fn try_run_app() -> Result<(), String> {
    let mode_label = if cfg!(debug_assertions) {
        "DEBUG 模式"
    } else {
        "RELEASE 模式"
    };
    app_log(
        "INFO",
        "BOOT",
        &format!("=== GeoStudio 3D ({}) 启动初始化 ===", mode_label),
    );

    let kernel_lib = Arc::new(GeoKernelLib::load_auto(|msg| {
        app_log("INFO", "FFI", msg);
    })?);
    kernel_lib.set_log_callback(Some(cpp_kernel_log_bridge));

    let project = Arc::new(SafeGeoProject::new(kernel_lib)?);
    let protocol_project = Arc::clone(&project);

    app_log("INFO", "TAURI", "正在初始化 Tauri v2 桌面窗口与 WebView2 渲染引擎...");

    tauri::Builder::default()
        .plugin(tauri_plugin_dialog::init())
        .manage(AppKernelState { project })
        .setup(|_app| {
            app_log("INFO", "TAURI", "主窗口与自定义协议 geotile:// 就绪");
            Ok(())
        })
        .register_uri_scheme_protocol("geotile", move |_ctx, request| {
            let uri = request.uri();
            let path = uri.path();
            let query = uri.query().unwrap_or("");
            let clear_dirty = query.contains("clear=1");

            let segments: Vec<&str> = path.trim_start_matches('/').split('/').collect();
            let result: Result<Vec<u8>, String> = match segments.as_slice() {
                ["octree", slot_str, node_str] => {
                    let slot = slot_str.parse::<i32>().map_err(|_| "非法槽位编号".to_string());
                    let node_id = node_str
                        .parse::<u32>()
                        .map_err(|_| "非法八叉树节点编号".to_string());
                    match (slot, node_id) {
                        (Ok(s), Ok(nid)) => protocol_project.fetch_octree_packet(s, nid),
                        (Err(e), _) | (_, Err(e)) => Err(e),
                    }
                }
                ["tin", slot_str, tile_str] => {
                    let slot = slot_str.parse::<i32>().map_err(|_| "非法槽位编号".to_string());
                    let tile_idx = tile_str
                        .parse::<u32>()
                        .map_err(|_| "非法三角网瓦片编号".to_string());
                    match (slot, tile_idx) {
                        (Ok(s), Ok(tidx)) => {
                            protocol_project.fetch_tin_tile_packet(s, tidx, clear_dirty)
                        }
                        (Err(e), _) | (_, Err(e)) => Err(e),
                    }
                }
                _ => Err(format!("未知几何瓦片协议路径: {}", path)),
            };

            match result {
                Ok(binary_bytes) => Response::builder()
                    .status(StatusCode::OK)
                    .header(CONTENT_TYPE, "application/octet-stream")
                    .header("Access-Control-Allow-Origin", "*")
                    .body(binary_bytes)
                    .unwrap(),
                Err(err_msg) => Response::builder()
                    .status(StatusCode::BAD_REQUEST)
                    .header(CONTENT_TYPE, "text/plain; charset=utf-8")
                    .header("Access-Control-Allow-Origin", "*")
                    .body(err_msg.into_bytes())
                    .unwrap(),
            }
        })
        .invoke_handler(tauri::generate_handler![
            cmd_load_dat,
            cmd_build_octree,
            cmd_build_tin,
            cmd_query_visible_octree_nodes,
            cmd_get_dirty_tin_tiles,
            cmd_pick_tin_ray,
            cmd_tin_flip_edge,
            cmd_tin_modify_vertex_z,
            cmd_tin_insert_vertex,
            cmd_tin_insert_breakline,
            cmd_tin_clip_polygon,
            cmd_tin_undo,
            cmd_tin_redo,
            cmd_tin_get_history_state,
            cmd_compute_earthwork_exact,
            cmd_compute_earthwork_grid,
            cmd_compute_slope_grading,
        ])
        .run(tauri::generate_context!())
        .map_err(|e| format!("Tauri 桌面窗口引擎启动失败: {}", e))
}

fn main() {
    init_app_logger();
    if let Err(err_msg) = try_run_app() {
        show_native_fatal_dialog("GeoStudio 3D 启动错误", &err_msg);
        std::process::exit(1);
    }
}

