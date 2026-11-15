//! GeoStudio 3D Tauri v2 桌面端后端宿主入口
//!
//! 健壮性与诊断规范：
//!   1. Release 模式（`not(debug_assertions)`）：
//!      - 启用 Windows 无控制台 GUI 子系统 (`windows_subsystem = "windows"`)；
//!      - 所有调试日志宏编译期完全展开为空语句，零锁竞争、零文件 I/O、零性能损耗；
//!      - 若启动期初始化异常，弹出原生系统错误模态框。
//!   2. Debug 模式（`debug_assertions`）：
//!      - 初始化带精确本地日期时间 (`YYYY-MM-DD HH:MM:SS.mmm`)、线程 ID 与级别标签的
//!        线程安全持久化日志引擎（同时输出至标准错误流与 `<exe_dir>/logs/GeoStudio3D_rust_debug.log`）；
//!      - 安装全局 Panic 捕获钩子，记录 Panic 源码位置与完整错误负载；
//!      - 接收前端转发的错误/调试日志，记录所有 Tauri IPC 命令与 `geotile://` 二进制请求及异常详情。

#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")]

// ==============================================================================
// Debug 专属高精度日期时间日志模块（Release 模式下编译期零指令剔除）
// ==============================================================================
#[cfg(debug_assertions)]
mod debug_logger {
    use std::fs::{self, File, OpenOptions};
    use std::io::{BufWriter, Write};
    use std::sync::{Mutex, OnceLock};
    use std::time::{SystemTime, UNIX_EPOCH};

    static LOG_WRITER: OnceLock<Mutex<Option<BufWriter<File>>>> = OnceLock::new();

    #[cfg(target_os = "windows")]
    fn current_local_datetime_string() -> String {
        #[repr(C)]
        struct SystemTimeWin {
            w_year: u16,
            w_month: u16,
            w_day_of_week: u16,
            w_day: u16,
            w_hour: u16,
            w_minute: u16,
            w_second: u16,
            w_milliseconds: u16,
        }
        #[link(name = "kernel32")]
        extern "system" {
            fn GetLocalTime(lp_system_time: *mut SystemTimeWin);
        }
        let mut st = SystemTimeWin {
            w_year: 0,
            w_month: 0,
            w_day_of_week: 0,
            w_day: 0,
            w_hour: 0,
            w_minute: 0,
            w_second: 0,
            w_milliseconds: 0,
        };
        unsafe {
            GetLocalTime(&mut st);
        }
        format!(
            "{:04}-{:02}-{:02} {:02}:{:02}:{:02}.{:03}",
            st.w_year,
            st.w_month,
            st.w_day,
            st.w_hour,
            st.w_minute,
            st.w_second,
            st.w_milliseconds
        )
    }

    #[cfg(not(target_os = "windows"))]
    fn current_local_datetime_string() -> String {
        use std::os::raw::c_int;
        #[repr(C)]
        struct TmPosix {
            tm_sec: c_int,
            tm_min: c_int,
            tm_hour: c_int,
            tm_mday: c_int,
            tm_mon: c_int,
            tm_year: c_int,
            tm_wday: c_int,
            tm_yday: c_int,
            tm_isdst: c_int,
            _pad: [usize; 8],
        }
        extern "C" {
            fn localtime_r(timep: *const i64, result: *mut TmPosix) -> *mut TmPosix;
        }
        let dur = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap_or_default();
        let secs = dur.as_secs() as i64;
        let millis = dur.subsec_millis();
        let mut tm = TmPosix {
            tm_sec: 0,
            tm_min: 0,
            tm_hour: 0,
            tm_mday: 0,
            tm_mon: 0,
            tm_year: 0,
            tm_wday: 0,
            tm_yday: 0,
            tm_isdst: 0,
            _pad: [0; 8],
        };
        let ptr = unsafe { localtime_r(&secs, &mut tm) };
        if ptr.is_null() {
            return format!("1970-01-01 00:00:00.{:03}", millis);
        }
        format!(
            "{:04}-{:02}-{:02} {:02}:{:02}:{:02}.{:03}",
            tm.tm_year + 1900,
            tm.tm_mon + 1,
            tm.tm_mday,
            tm.tm_hour,
            tm.tm_min,
            tm.tm_sec,
            millis
        )
    }

    fn open_debug_log_file() -> Option<BufWriter<File>> {
        let candidate_dir = std::env::current_exe()
            .ok()
            .and_then(|p| p.parent().map(|d| d.join("logs")))
            .unwrap_or_else(|| std::env::temp_dir().join("GeoStudio3D_logs"));

        let log_dir = if fs::create_dir_all(&candidate_dir).is_ok() {
            candidate_dir
        } else {
            let fallback = std::env::temp_dir().join("GeoStudio3D_logs");
            let _ = fs::create_dir_all(&fallback);
            fallback
        };

        let log_path = log_dir.join("GeoStudio3D_host_debug.log");
        OpenOptions::new()
            .create(true)
            .append(true)
            .open(&log_path)
            .ok()
            .map(BufWriter::new)
    }

    pub fn write_log(level: &str, module: &str, msg: &str) {
        let ts = current_local_datetime_string();
        let tid = format!("{:?}", std::thread::current().id());
        let line = format!("[{}] [{}] [{}] [{}] {}", ts, level, tid, module, msg);
        eprintln!("{}", line);

        let mutex = LOG_WRITER.get_or_init(|| Mutex::new(open_debug_log_file()));
        if let Ok(mut guard) = mutex.lock() {
            if let Some(writer) = guard.as_mut() {
                let _ = writeln!(writer, "{}", line);
                let _ = writer.flush();
            }
        }
    }

    pub fn install_panic_hook() {
        let prev_hook = std::panic::take_hook();
        std::panic::set_hook(Box::new(move |info| {
            let location = info
                .location()
                .map(|l| format!("{}:{}:{}", l.file(), l.line(), l.column()))
                .unwrap_or_else(|| "未知位置".to_string());
            let payload = if let Some(s) = info.payload().downcast_ref::<&str>() {
                (*s).to_string()
            } else if let Some(s) = info.payload().downcast_ref::<String>() {
                s.clone()
            } else {
                "非字符串 Panic 负载".to_string()
            };
            write_log(
                "FATAL",
                "PANIC",
                &format!("Rust 线程发生致命 Panic @ {} -> {}", location, payload),
            );
            prev_hook(info);
        }));
    }
}

#[cfg(debug_assertions)]
#[macro_export]
macro_rules! app_log_debug {
    ($mod:expr, $($arg:tt)*) => {
        $crate::debug_logger::write_log("DEBUG", $mod, &format!($($arg)*))
    };
}
#[cfg(not(debug_assertions))]
#[macro_export]
macro_rules! app_log_debug {
    ($mod:expr, $($arg:tt)*) => {};
}

#[cfg(debug_assertions)]
#[macro_export]
macro_rules! app_log_info {
    ($mod:expr, $($arg:tt)*) => {
        $crate::debug_logger::write_log("INFO ", $mod, &format!($($arg)*))
    };
}
#[cfg(not(debug_assertions))]
#[macro_export]
macro_rules! app_log_info {
    ($mod:expr, $($arg:tt)*) => {};
}

#[cfg(debug_assertions)]
#[macro_export]
macro_rules! app_log_warn {
    ($mod:expr, $($arg:tt)*) => {
        $crate::debug_logger::write_log("WARN ", $mod, &format!($($arg)*))
    };
}
#[cfg(not(debug_assertions))]
#[macro_export]
macro_rules! app_log_warn {
    ($mod:expr, $($arg:tt)*) => {};
}

#[cfg(debug_assertions)]
#[macro_export]
macro_rules! app_log_error {
    ($mod:expr, $($arg:tt)*) => {
        $crate::debug_logger::write_log("ERROR", $mod, &format!($($arg)*))
    };
}
#[cfg(not(debug_assertions))]
#[macro_export]
macro_rules! app_log_error {
    ($mod:expr, $($arg:tt)*) => {};
}

mod kernel_ffi;

use kernel_ffi::{
    GeoCameraQuery, GeoDatSummary, GeoEarthworkSummary, GeoGradingParams, GeoGradingSummary,
    GeoKernelLib, GeoOctreeSummary, GeoRayHitResult, GeoSlopeStageSpec, GeoTinSummary,
    SafeGeoProject,
};
use serde::{Deserialize, Serialize};
use std::ptr;
use std::sync::Arc;
use tauri::http::{header::CONTENT_TYPE, Response, StatusCode};
use tauri::State;

#[cfg(debug_assertions)]
use std::time::Instant;

/// 跨平台原生启动错误提示弹窗（确保双击启动失败时用户可见明确诊断原因）
fn show_native_fatal_dialog(title: &str, message: &str) {
    app_log_error!("FATAL", "[{}] {}", title, message);
    eprintln!("[{}] {}", title, message);
    #[cfg(target_os = "windows")]
    {
        use std::ffi::OsStr;
        use std::os::raw::c_void;
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
// Tauri 调试诊断辅助命令（Release 模式返回 false 且零开销）
// ==============================================================================
#[tauri::command]
fn cmd_is_debug_mode() -> bool {
    cfg!(debug_assertions)
}

#[tauri::command]
fn cmd_write_frontend_log(level: String, module: String, message: String) {
    #[cfg(debug_assertions)]
    {
        let normalized_level = match level.as_str() {
            "DEBUG" => "DEBUG",
            "INFO" => "INFO ",
            "WARN" => "WARN ",
            "ERROR" => "ERROR",
            _ => "INFO ",
        };
        let mod_tag = format!("FE:{}", module);
        debug_logger::write_log(normalized_level, &mod_tag, &message);
    }
    #[cfg(not(debug_assertions))]
    {
        let _ = (level, module, message);
    }
}

// ==============================================================================
// Tauri 异步业务命令集
// ==============================================================================
#[tauri::command]
async fn cmd_load_dat(
    state: State<'_, AppKernelState>,
    slot: i32,
    file_path: String,
    coord_order: i32,
) -> Result<GeoDatSummary, String> {
    app_log_info!("CMD", "cmd_load_dat(slot={}, order={}, file={})", slot, coord_order, file_path);
    let proj = Arc::clone(&state.project);
    tauri::async_runtime::spawn_blocking(move || proj.load_dat(slot, &file_path, coord_order))
        .await
        .map_err(|e| {
            let msg = format!("后台任务调度异常: {}", e);
            app_log_error!("CMD", "cmd_load_dat {}", msg);
            msg
        })?
}

#[tauri::command]
async fn cmd_build_octree(
    state: State<'_, AppKernelState>,
    slot: i32,
) -> Result<GeoOctreeSummary, String> {
    app_log_info!("CMD", "cmd_build_octree(slot={})", slot);
    let proj = Arc::clone(&state.project);
    tauri::async_runtime::spawn_blocking(move || {
        #[cfg(debug_assertions)]
        let t0 = Instant::now();
        let params = unsafe { (proj.lib.geo_default_octree_params)() };
        let mut summary = GeoOctreeSummary::default();
        let st = unsafe {
            (proj.lib.geo_project_build_octree)(proj.handle, slot, &params, &mut summary)
        };
        proj.lib.check_status(st)?;
        #[cfg(debug_assertions)]
        app_log_info!(
            "CMD",
            "cmd_build_octree(slot={}) 完成 ({:.2} ms): nodes={}, leaves={}, depth={}",
            slot,
            t0.elapsed().as_secs_f64() * 1000.0,
            summary.total_nodes,
            summary.leaf_nodes,
            summary.max_reached_depth
        );
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
    app_log_info!(
        "CMD",
        "cmd_build_tin(slot={}, decimate={}, tile_size={})",
        slot,
        decimation_grid_size,
        tile_size
    );
    let proj = Arc::clone(&state.project);
    tauri::async_runtime::spawn_blocking(move || {
        #[cfg(debug_assertions)]
        let t0 = Instant::now();
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
        #[cfg(debug_assertions)]
        app_log_info!(
            "CMD",
            "cmd_build_tin(slot={}) 完成 ({:.2} ms): verts={}, tris={}, tiles={}",
            slot,
            t0.elapsed().as_secs_f64() * 1000.0,
            summary.tin_vertices,
            summary.tin_triangles,
            summary.total_tiles
        );
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
        let msg = format!(
            "视锥体裁剪平面数量必须严格为 6 个，实际收到 {} 个",
            query.frustum_planes.len()
        );
        app_log_error!("CMD", "cmd_query_visible_octree_nodes: {}", msg);
        return Err(msg);
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
    app_log_debug!("CMD", "cmd_get_dirty_tin_tiles(slot={}) -> {} 个脏瓦片", slot, tiles.len());
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
    app_log_info!(
        "CMD",
        "cmd_tin_flip_edge(slot={}, tri={}, edge={}, force={})",
        slot,
        triangle_id,
        edge_slot,
        force_flip
    );
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
    app_log_info!(
        "CMD",
        "cmd_tin_modify_vertex_z(slot={}, vid={}, new_z={:.4})",
        slot,
        vertex_id,
        new_local_z
    );
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
    app_log_info!(
        "CMD",
        "cmd_tin_insert_vertex(slot={}, xyz=[{:.3},{:.3},{:.3}], code={})",
        slot,
        local_xyz[0],
        local_xyz[1],
        local_xyz[2],
        code_id
    );
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
    app_log_info!("CMD", "cmd_tin_insert_vertex 生成/复用顶点 ID={}", out_vid);
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
    app_log_info!(
        "CMD",
        "cmd_tin_insert_breakline(slot={}, pts={}, code={}, interpZ={})",
        slot,
        polyline.len(),
        code_id,
        interpolate_z_from_tin
    );
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
    app_log_info!(
        "CMD",
        "cmd_tin_clip_polygon(slot={}, pts={}, keep_inside={})",
        slot,
        polygon.len(),
        keep_inside
    );
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
    app_log_info!("CMD", "cmd_tin_undo(slot={})", slot);
    let proj = &state.project;
    let st = unsafe { (proj.lib.geo_project_tin_undo)(proj.handle, slot) };
    proj.lib.check_status(st)
}

#[tauri::command]
fn cmd_tin_redo(state: State<'_, AppKernelState>, slot: i32) -> Result<(), String> {
    app_log_info!("CMD", "cmd_tin_redo(slot={})", slot);
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
    app_log_info!(
        "CMD",
        "cmd_compute_earthwork_exact(p1={}, p2={}, roi_pts={})",
        period1_slot,
        period2_slot,
        roi_polygon.len()
    );
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
        app_log_info!(
            "CMD",
            "cmd_compute_earthwork_exact 完成: cut={:.2} m3, fill={:.2} m3, prisms={}",
            summary.cut_volume_m3,
            summary.fill_volume_m3,
            summary.intersected_prisms
        );
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
    app_log_info!(
        "CMD",
        "cmd_compute_earthwork_grid(p1={}, p2={}, step={}, roi_pts={})",
        period1_slot,
        period2_slot,
        grid_step,
        roi_polygon.len()
    );
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
    app_log_info!(
        "CMD",
        "cmd_compute_slope_grading(orig={}, base_pts={}, stages={}, dir={}, vmode={}, merged={})",
        original_slot,
        base_polygon.len(),
        stages.len(),
        params.direction,
        params.vertical_mode,
        generate_merged_terrain
    );
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
        app_log_info!(
            "CMD",
            "cmd_compute_slope_grading 完成: converged={}, dl_pts={}, cut={:.2} m3, fill={:.2} m3",
            summary.daylight_converged,
            valid_dl_count,
            summary.earthwork.cut_volume_m3,
            summary.earthwork.fill_volume_m3
        );
        Ok(GradingSummaryDto {
            base: summary,
            daylight_polyline,
        })
    })
    .await
    .map_err(|e| format!("后台任务调度异常: {}", e))?
}

// ==============================================================================
// Tauri 主应用启动入口（带异常保护与原生错误提示弹窗）
// ==============================================================================
fn try_run_app() -> Result<(), String> {
    #[cfg(debug_assertions)]
    {
        debug_logger::install_panic_hook();
        app_log_info!("BOOT", "=== GeoStudio 3D (DEBUG 模式) 启动初始化 ===");
    }

    let kernel_lib = Arc::new(GeoKernelLib::load_auto()?);
    let project = Arc::new(SafeGeoProject::new(kernel_lib)?);
    let protocol_project = Arc::clone(&project);

    app_log_info!("BOOT", "正在初始化 Tauri v2 窗口引擎并注册 geotile:// 二进制流协议...");
    tauri::Builder::default()
        .plugin(tauri_plugin_dialog::init())
        .manage(AppKernelState { project })
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
                Err(err_msg) => {
                    app_log_error!("PROTOCOL", "geotile:// 请求失败 ({}): {}", path, err_msg);
                    Response::builder()
                        .status(StatusCode::BAD_REQUEST)
                        .header(CONTENT_TYPE, "text/plain; charset=utf-8")
                        .header("Access-Control-Allow-Origin", "*")
                        .body(err_msg.into_bytes())
                        .unwrap()
                }
            }
        })
        .invoke_handler(tauri::generate_handler![
            cmd_is_debug_mode,
            cmd_write_frontend_log,
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
        .map_err(|e| {
            let msg = format!("Tauri 桌面窗口引擎启动失败: {}", e);
            app_log_error!("BOOT", "{}", msg);
            msg
        })
}

fn main() {
    if let Err(err_msg) = try_run_app() {
        show_native_fatal_dialog("GeoStudio 3D 启动错误", &err_msg);
        std::process::exit(1);
    }
}
