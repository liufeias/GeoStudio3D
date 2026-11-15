/**
 * @file kernel_bridge.ts
 * @brief 多平台解耦内核通信抽象层与前端调试级带时间戳日志器
 *
 * 解耦与诊断设计规范：
 *   1. 渲染器与所有业务面板仅依赖 IGeoKernelBridge 接口，完全屏蔽底层是 Tauri 本地动态库、
 *      还是远程计算节点。
 *   2. 桌面端模式下，几何数据瓦片通过自定义协议 `geotile://localhost/...` 直接返回原生
 *      ArrayBuffer 字节流（零 JSON/Base64 膨胀开销）。
 *   3. 调试日志规范（Debug 专属，Release 零性能损耗）：
 *      - `FrontendDebugLogger` 在应用启动时向宿主查询是否为 Debug 构建（`cmd_is_debug_mode`）。
 *      - 在 Debug 构建下：所有日志均携带精确到毫秒的本地日期时间 `[YYYY-MM-DD HH:MM:SS.mmm]`，
 *        同时输出到浏览器 DevTools 控制台与 Tauri 后端持久化调试日志文件，便于完整追踪全链路异常。
 *      - 在 Release 构建下：`isEnabled()` 恒为 `false`，所有日志调用采用惰性函数求值，直接短路返回，
 *        不执行任何字符串格式化、不产生内存分配、不发起任何日志 IPC 通信。
 */
import {
  CameraQueryInput,
  DatSummary,
  DecodedPointCloudPacket,
  DecodedTinMeshPacket,
  EarthworkSummary,
  GradingParams,
  GradingSummary,
  OctreeSummary,
  RayHitResult,
  SlopeStageSpec,
  SurfaceSlot,
  TinSummary,
  VisibleOctreeNodesResult,
  decodePointCloudPacket,
  decodeTinMeshPacket,
} from '../types/geo_protocol';

/**
 * 统一日志级别定义
 */
export type LogLevel = 'DEBUG' | 'INFO' | 'WARN' | 'ERROR';

/**
 * 惰性日志消息构造器类型（避免 Release 模式下提前拼接日志字符串消耗性能）
 */
export type LazyLogMessage = string | (() => string);

interface PendingBootLog {
  level: LogLevel;
  module: string;
  message: string;
  timestamp: string;
}

/**
 * 格式化当前本地日期时间为 `YYYY-MM-DD HH:MM:SS.mmm` 高精度字符串
 */
export function formatLocalTimestamp(date = new Date()): string {
  const yyyy = date.getFullYear();
  const mm = String(date.getMonth() + 1).padStart(2, '0');
  const dd = String(date.getDate()).padStart(2, '0');
  const hh = String(date.getHours()).padStart(2, '0');
  const mi = String(date.getMinutes()).padStart(2, '0');
  const ss = String(date.getSeconds()).padStart(2, '0');
  const ms = String(date.getMilliseconds()).padStart(3, '0');
  return `${yyyy}-${mm}-${dd} ${hh}:${mi}:${ss}.${ms}`;
}

/**
 * 提取任意异常对象的完整诊断文本（含错误名称、消息文本、堆栈及嵌套 cause）
 */
export function formatErrorDetails(err: unknown): string {
  if (err instanceof Error) {
    const parts: string[] = [`${err.name}: ${err.message}`];
    if (err.stack) {
      parts.push(`堆栈信息:\n${err.stack}`);
    }
    const errWithCause = err as Error & { cause?: unknown };
    if (typeof errWithCause.cause !== 'undefined') {
      parts.push(`底层原因(cause): ${formatErrorDetails(errWithCause.cause)}`);
    }
    return parts.join(' | ');
  }
  if (typeof err === 'string') {
    return err;
  }
  try {
    return JSON.stringify(err);
  } catch {
    return String(err);
  }
}

/**
 * 前端全局统一日志控制器（仅 Debug 构建生效，Release 构建零开销静默）
 */
export class FrontendDebugLogger {
  private bridge: IGeoKernelBridge | null = null;
  private initialized = false;
  private debugEnabled = false;
  private isForwarding = false;
  private readonly bootQueue: PendingBootLog[] = [];
  private readonly maxBootQueueSize = 128;

  constructor() {
    // 在尚未完成后端调试标志握手前，仅当 Vite 开发服务器明确处于 DEV 环境时预开启
    const importMeta = import.meta as unknown as { env?: { DEV?: boolean } };
    this.debugEnabled = importMeta.env?.DEV === true;
  }

  /**
   * 判定当前是否处于启用详细调试日志模式
   */
  isEnabled(): boolean {
    return this.debugEnabled;
  }

  /**
   * 绑定内核通信桥接器并向原生宿主查询真实编译模式（Debug / Release）
   */
  async initWithBridge(bridge: IGeoKernelBridge): Promise<void> {
    this.bridge = bridge;
    try {
      this.debugEnabled = await bridge.isDebugMode();
    } catch {
      this.debugEnabled = false;
    }
    this.initialized = true;

    if (!this.debugEnabled) {
      // Release 模式下立即清空并释放启动缓冲队列
      this.bootQueue.length = 0;
      return;
    }

    // Debug 模式下回放启动阶段缓存的早期日志至持久化日志文件
    while (this.bootQueue.length > 0) {
      const item = this.bootQueue.shift();
      if (item) {
        this.emitToConsole(item.timestamp, item.level, item.module, item.message);
        void this.forwardToBackend(item.level, item.module, `[启动缓冲 @ ${item.timestamp}] ${item.message}`);
      }
    }

    this.info(
      'LOGGER',
      () => `前端高精度调试日志器初始化完成 | 通信后端: ${bridge.backendName} | 调试模式: 已开启`
    );
  }

  debug(module: string, msgOrFn: LazyLogMessage): void {
    this.log('DEBUG', module, msgOrFn);
  }

  info(module: string, msgOrFn: LazyLogMessage): void {
    this.log('INFO', module, msgOrFn);
  }

  warn(module: string, msgOrFn: LazyLogMessage, err?: unknown): void {
    if (!this.shouldCapture()) return;
    const base = typeof msgOrFn === 'function' ? msgOrFn() : msgOrFn;
    const fullMsg = typeof err !== 'undefined' ? `${base} | 详情: ${formatErrorDetails(err)}` : base;
    this.dispatch('WARN', module, fullMsg);
  }

  error(module: string, msgOrFn: LazyLogMessage, err?: unknown): void {
    if (!this.shouldCapture()) return;
    const base = typeof msgOrFn === 'function' ? msgOrFn() : msgOrFn;
    const fullMsg = typeof err !== 'undefined' ? `${base} | 异常详情: ${formatErrorDetails(err)}` : base;
    this.dispatch('ERROR', module, fullMsg);
  }

  private log(level: LogLevel, module: string, msgOrFn: LazyLogMessage): void {
    if (!this.shouldCapture()) return;
    const message = typeof msgOrFn === 'function' ? msgOrFn() : msgOrFn;
    this.dispatch(level, module, message);
  }

  private shouldCapture(): boolean {
    return this.debugEnabled || !this.initialized;
  }

  private dispatch(level: LogLevel, module: string, message: string): void {
    const ts = formatLocalTimestamp();
    if (!this.initialized) {
      if (this.bootQueue.length < this.maxBootQueueSize) {
        this.bootQueue.push({ level, module, message, timestamp: ts });
      }
      if (this.debugEnabled) {
        this.emitToConsole(ts, level, module, message);
      }
      return;
    }
    if (!this.debugEnabled) return;
    this.emitToConsole(ts, level, module, message);
    void this.forwardToBackend(level, module, message);
  }

  private emitToConsole(ts: string, level: LogLevel, module: string, message: string): void {
    const line = `[${ts}] [${level}] [FE:${module}] ${message}`;
    switch (level) {
      case 'DEBUG':
        console.debug(line);
        break;
      case 'INFO':
        console.info(line);
        break;
      case 'WARN':
        console.warn(line);
        break;
      case 'ERROR':
        console.error(line);
        break;
    }
  }

  private async forwardToBackend(level: LogLevel, module: string, message: string): Promise<void> {
    if (!this.bridge || !this.debugEnabled || this.isForwarding) {
      return;
    }
    this.isForwarding = true;
    try {
      await this.bridge.writeDebugLog(level, module, message);
    } catch {
      // 日志转发通道本身发生异常时静默保护，严禁引发无限递归异常
    } finally {
      this.isForwarding = false;
    }
  }
}

/**
 * 全局共享前端日志单例
 */
export const frontendLogger = new FrontendDebugLogger();

/**
 * 几何内核统一异步通信契约接口
 */
export interface IGeoKernelBridge {
  readonly backendName: string;

  /** 查询原生宿主是否处于 Debug 调试编译模式 */
  isDebugMode(): Promise<boolean>;

  /** 将前端带级别与模块名的诊断日志写入原生调试日志通道（Release 下空实现） */
  writeDebugLog(level: LogLevel, module: string, message: string): Promise<void>;

  /** 打开原生文件对话框选择 .dat 文件（浏览器模式下自动回退为路径输入/上传） */
  pickDatFileDialog(title: string): Promise<string | null>;

  /** 导入 .dat 点云文件并构建八叉树与初始三角网 */
  loadDatFile(
    slot: SurfaceSlot,
    filePath: string,
    coordOrder: 0 | 1
  ): Promise<DatSummary>;

  buildOctree(slot: SurfaceSlot): Promise<OctreeSummary>;

  buildTin(
    slot: SurfaceSlot,
    decimationGridSize: number,
    tileSize: number
  ): Promise<TinSummary>;

  queryVisibleOctreeNodes(
    slot: SurfaceSlot,
    query: CameraQueryInput
  ): Promise<VisibleOctreeNodesResult>;

  fetchOctreeNodePacket(
    slot: SurfaceSlot,
    nodeId: number
  ): Promise<DecodedPointCloudPacket>;

  getDirtyTinTiles(slot: SurfaceSlot): Promise<number[]>;

  fetchTinTilePacket(
    slot: SurfaceSlot,
    tileIndex: number,
    clearDirtyFlag: boolean
  ): Promise<DecodedTinMeshPacket>;

  queryElevationAt(
    slot: SurfaceSlot,
    localX: number,
    localY: number
  ): Promise<{ localZ: number; globalZ: number } | null>;

  pickTinByRay(
    slot: SurfaceSlot,
    rayOrigin: [number, number, number],
    rayDir: [number, number, number]
  ): Promise<RayHitResult>;

  flipTinEdge(
    slot: SurfaceSlot,
    triangleId: number,
    edgeSlot: number,
    forceFlip: boolean
  ): Promise<void>;

  modifyVertexElevation(
    slot: SurfaceSlot,
    vertexId: number,
    newLocalZ: number
  ): Promise<void>;

  insertVertex(
    slot: SurfaceSlot,
    localXyz: [number, number, number],
    codeId: number
  ): Promise<number>;

  insertBreakline(
    slot: SurfaceSlot,
    polyline: Array<[number, number, number]>,
    codeId: number,
    interpolateZFromTin: boolean
  ): Promise<void>;

  clipByPolygon(
    slot: SurfaceSlot,
    polygon: Array<[number, number, number]>,
    keepInside: boolean
  ): Promise<void>;

  undoTin(slot: SurfaceSlot): Promise<void>;
  redoTin(slot: SurfaceSlot): Promise<void>;
  getHistoryState(slot: SurfaceSlot): Promise<{ canUndo: boolean; canRedo: boolean }>;

  computeEarthworkExact(
    period1Slot: SurfaceSlot,
    period2Slot: SurfaceSlot,
    roiPolygon?: Array<[number, number, number]>
  ): Promise<EarthworkSummary>;

  computeEarthworkGrid(
    period1Slot: SurfaceSlot,
    period2Slot: SurfaceSlot,
    gridStep: number,
    roiPolygon?: Array<[number, number, number]>
  ): Promise<EarthworkSummary>;

  computeSlopeGrading(
    originalSlot: SurfaceSlot,
    basePolygon: Array<[number, number, number]>,
    stages: SlopeStageSpec[],
    params: GradingParams,
    generateMergedTerrain: boolean
  ): Promise<GradingSummary>;
}

// ==============================================================================
// 实现 1：Tauri v2 原生桌面桥接器（Rust FFI + geotile:// 零拷贝二进制流）
// ==============================================================================
export class TauriNativeKernelBridge implements IGeoKernelBridge {
  readonly backendName = 'Tauri v2 Native C++20 Kernel';

  private async invokeCmd<T>(
    cmd: string,
    args?: Record<string, unknown>,
    skipLog = false
  ): Promise<T> {
    const logOn = !skipLog && frontendLogger.isEnabled();
    const startMs = logOn ? performance.now() : 0;
    if (logOn) {
      frontendLogger.debug('IPC', () => `调用原生命令 ${cmd} | 参数: ${JSON.stringify(args ?? {})}`);
    }
    try {
      const { invoke } = await import('@tauri-apps/api/core');
      const res = await invoke<T>(cmd, args);
      if (logOn) {
        const costMs = (performance.now() - startMs).toFixed(2);
        frontendLogger.debug('IPC', () => `命令 ${cmd} 执行成功 (耗时 ${costMs} ms)`);
      }
      return res;
    } catch (err) {
      if (!skipLog) {
        const costMs = logOn ? (performance.now() - startMs).toFixed(2) : '0.00';
        frontendLogger.error('IPC', () => `原生命令 ${cmd} 执行失败 (耗时 ${costMs} ms)`, err);
      }
      throw err;
    }
  }

  async isDebugMode(): Promise<boolean> {
    try {
      return await this.invokeCmd<boolean>('cmd_is_debug_mode', undefined, true);
    } catch {
      return false;
    }
  }

  async writeDebugLog(level: LogLevel, module: string, message: string): Promise<void> {
    if (!frontendLogger.isEnabled()) return;
    await this.invokeCmd<void>('cmd_write_frontend_log', { level, module, message }, true);
  }

  async pickDatFileDialog(title: string): Promise<string | null> {
    frontendLogger.debug('DIALOG', () => `打开原生文件选择框: "${title}"`);
    try {
      const { open } = await import('@tauri-apps/plugin-dialog');
      const selected = await open({
        title,
        multiple: false,
        filters: [
          { name: '测绘点云数据 (*.dat, *.txt, *.csv)', extensions: ['dat', 'txt', 'csv'] },
          { name: '所有文件', extensions: ['*'] },
        ],
      });
      if (typeof selected === 'string') {
        frontendLogger.info('DIALOG', () => `用户选中文件: ${selected}`);
        return selected;
      }
      frontendLogger.debug('DIALOG', '用户取消了文件选择');
      return null;
    } catch (err) {
      frontendLogger.error('DIALOG', () => `文件选择对话框调用异常 (${title})`, err);
      throw err;
    }
  }

  async loadDatFile(
    slot: SurfaceSlot,
    filePath: string,
    coordOrder: 0 | 1
  ): Promise<DatSummary> {
    return this.invokeCmd<DatSummary>('cmd_load_dat', {
      slot,
      filePath,
      coordOrder,
    });
  }

  async buildOctree(slot: SurfaceSlot): Promise<OctreeSummary> {
    return this.invokeCmd<OctreeSummary>('cmd_build_octree', { slot });
  }

  async buildTin(
    slot: SurfaceSlot,
    decimationGridSize: number,
    tileSize: number
  ): Promise<TinSummary> {
    return this.invokeCmd<TinSummary>('cmd_build_tin', {
      slot,
      decimationGridSize,
      tileSize,
    });
  }

  async queryVisibleOctreeNodes(
    slot: SurfaceSlot,
    query: CameraQueryInput
  ): Promise<VisibleOctreeNodesResult> {
    // 高频帧级视锥查询使用 skipLog=true 避免与上层重复刷屏，由上层 OctreeLodManager 汇总输出精简日志
    return this.invokeCmd<VisibleOctreeNodesResult>(
      'cmd_query_visible_octree_nodes',
      { slot, query },
      true
    );
  }

  async fetchOctreeNodePacket(
    slot: SurfaceSlot,
    nodeId: number
  ): Promise<DecodedPointCloudPacket> {
    const url = `http://geotile.localhost/octree/${slot}/${nodeId}`;
    const logOn = frontendLogger.isEnabled();
    const startMs = logOn ? performance.now() : 0;
    const response = await fetch(url);
    if (!response.ok) {
      const errText = await response.text();
      const msg = `拉取八叉树节点 #${nodeId} 失败 (HTTP ${response.status}): ${errText}`;
      frontendLogger.error('GEOTILE', msg);
      throw new Error(msg);
    }
    const buffer = await response.arrayBuffer();
    const pkt = decodePointCloudPacket(buffer);
    if (logOn) {
      const costMs = (performance.now() - startMs).toFixed(2);
      frontendLogger.debug(
        'GEOTILE',
        () => `拉取八叉树节点 #${nodeId} (槽位 ${slot}) 成功: ${pkt.header.vertexCount} 点, ${buffer.byteLength} 字节, 耗时 ${costMs} ms`
      );
    }
    return pkt;
  }

  async getDirtyTinTiles(slot: SurfaceSlot): Promise<number[]> {
    return this.invokeCmd<number[]>('cmd_get_dirty_tin_tiles', { slot });
  }

  async fetchTinTilePacket(
    slot: SurfaceSlot,
    tileIndex: number,
    clearDirtyFlag: boolean
  ): Promise<DecodedTinMeshPacket> {
    const clearFlag = clearDirtyFlag ? 1 : 0;
    const url = `http://geotile.localhost/tin/${slot}/${tileIndex}?clear=${clearFlag}`;
    const logOn = frontendLogger.isEnabled();
    const startMs = logOn ? performance.now() : 0;
    const response = await fetch(url);
    if (!response.ok) {
      const errText = await response.text();
      const msg = `拉取三角网瓦片 #${tileIndex} 失败 (HTTP ${response.status}): ${errText}`;
      frontendLogger.error('GEOTILE', msg);
      throw new Error(msg);
    }
    const buffer = await response.arrayBuffer();
    const pkt = decodeTinMeshPacket(buffer);
    if (logOn) {
      const costMs = (performance.now() - startMs).toFixed(2);
      frontendLogger.debug(
        'GEOTILE',
        () => `拉取三角网瓦片 #${tileIndex} (槽位 ${slot}, rev=${pkt.header.revision}) 成功: ${pkt.header.vertexCount} 顶点, ${Math.floor(pkt.header.indexCount / 3)} 三角形, ${buffer.byteLength} 字节, 耗时 ${costMs} ms`
      );
    }
    return pkt;
  }

  async queryElevationAt(
    slot: SurfaceSlot,
    localX: number,
    localY: number
  ): Promise<{ localZ: number; globalZ: number } | null> {
    return this.invokeCmd<{ localZ: number; globalZ: number } | null>('cmd_query_elevation', {
      slot,
      localX,
      localY,
    });
  }

  async pickTinByRay(
    slot: SurfaceSlot,
    rayOrigin: [number, number, number],
    rayDir: [number, number, number]
  ): Promise<RayHitResult> {
    // 鼠标移动期间 35ms 高频触发，跳过高频成功日志，仅在发生异常时记录 ERROR
    try {
      return await this.invokeCmd<RayHitResult>(
        'cmd_pick_tin_ray',
        { slot, rayOrigin, rayDir },
        true
      );
    } catch (err) {
      frontendLogger.error('PICK', () => `射线拾取命令执行失败 (槽位 ${slot})`, err);
      throw err;
    }
  }

  async flipTinEdge(
    slot: SurfaceSlot,
    triangleId: number,
    edgeSlot: number,
    forceFlip: boolean
  ): Promise<void> {
    await this.invokeCmd<void>('cmd_tin_flip_edge', {
      slot,
      triangleId,
      edgeSlot,
      forceFlip,
    });
  }

  async modifyVertexElevation(
    slot: SurfaceSlot,
    vertexId: number,
    newLocalZ: number
  ): Promise<void> {
    await this.invokeCmd<void>('cmd_tin_modify_vertex_z', {
      slot,
      vertexId,
      newLocalZ,
    });
  }

  async insertVertex(
    slot: SurfaceSlot,
    localXyz: [number, number, number],
    codeId: number
  ): Promise<number> {
    return this.invokeCmd<number>('cmd_tin_insert_vertex', {
      slot,
      localXyz,
      codeId,
    });
  }

  async insertBreakline(
    slot: SurfaceSlot,
    polyline: Array<[number, number, number]>,
    codeId: number,
    interpolateZFromTin: boolean
  ): Promise<void> {
    await this.invokeCmd<void>('cmd_tin_insert_breakline', {
      slot,
      polyline,
      codeId,
      interpolateZFromTin,
    });
  }

  async clipByPolygon(
    slot: SurfaceSlot,
    polygon: Array<[number, number, number]>,
    keepInside: boolean
  ): Promise<void> {
    await this.invokeCmd<void>('cmd_tin_clip_polygon', {
      slot,
      polygon,
      keepInside,
    });
  }

  async undoTin(slot: SurfaceSlot): Promise<void> {
    await this.invokeCmd<void>('cmd_tin_undo', { slot });
  }

  async redoTin(slot: SurfaceSlot): Promise<void> {
    await this.invokeCmd<void>('cmd_tin_redo', { slot });
  }

  async getHistoryState(slot: SurfaceSlot): Promise<{ canUndo: boolean; canRedo: boolean }> {
    return this.invokeCmd<{ canUndo: boolean; canRedo: boolean }>('cmd_tin_get_history_state', {
      slot,
    });
  }

  async computeEarthworkExact(
    period1Slot: SurfaceSlot,
    period2Slot: SurfaceSlot,
    roiPolygon?: Array<[number, number, number]>
  ): Promise<EarthworkSummary> {
    return this.invokeCmd<EarthworkSummary>('cmd_compute_earthwork_exact', {
      period1Slot,
      period2Slot,
      roiPolygon: roiPolygon ?? [],
    });
  }

  async computeEarthworkGrid(
    period1Slot: SurfaceSlot,
    period2Slot: SurfaceSlot,
    gridStep: number,
    roiPolygon?: Array<[number, number, number]>
  ): Promise<EarthworkSummary> {
    return this.invokeCmd<EarthworkSummary>('cmd_compute_earthwork_grid', {
      period1Slot,
      period2Slot,
      gridStep,
      roiPolygon: roiPolygon ?? [],
    });
  }

  async computeSlopeGrading(
    originalSlot: SurfaceSlot,
    basePolygon: Array<[number, number, number]>,
    stages: SlopeStageSpec[],
    params: GradingParams,
    generateMergedTerrain: boolean
  ): Promise<GradingSummary> {
    return this.invokeCmd<GradingSummary>('cmd_compute_slope_grading', {
      originalSlot,
      basePolygon,
      stages,
      params,
      generateMergedTerrain,
    });
  }
}

// ==============================================================================
// 实现 2：Web HTTP/Binary 远程计算服务器桥接器（便于无缝迁移至云端 SaaS 模式）
// ==============================================================================
export class HttpBinaryKernelBridge implements IGeoKernelBridge {
  readonly backendName = 'Web Remote C++20 Compute Server';
  private readonly baseUrl: string;

  constructor(baseUrl = '/api/geokernel') {
    this.baseUrl = baseUrl;
  }

  async isDebugMode(): Promise<boolean> {
    const importMeta = import.meta as unknown as { env?: { DEV?: boolean } };
    return importMeta.env?.DEV === true;
  }

  async writeDebugLog(_level: LogLevel, _module: string, _message: string): Promise<void> {
    // 浏览器远程模式已由 FrontendDebugLogger 输出至控制台，此处无需额外远程请求
  }

  private async postJson<T>(endpoint: string, body: unknown): Promise<T> {
    const logOn = frontendLogger.isEnabled();
    const startMs = logOn ? performance.now() : 0;
    const resp = await fetch(`${this.baseUrl}${endpoint}`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(body),
    });
    if (!resp.ok) {
      const errText = await resp.text();
      frontendLogger.error('HTTP', () => `POST ${endpoint} 返回异常状态 ${resp.status}: ${errText}`);
      throw new Error(errText);
    }
    const data = (await resp.json()) as T;
    if (logOn) {
      const costMs = (performance.now() - startMs).toFixed(2);
      frontendLogger.debug('HTTP', () => `POST ${endpoint} 完成 (耗时 ${costMs} ms)`);
    }
    return data;
  }

  async pickDatFileDialog(title: string): Promise<string | null> {
    const path = window.prompt(`${title} — 请输入服务器侧 .dat 文件路径:`);
    return path && path.trim().length > 0 ? path.trim() : null;
  }

  async loadDatFile(slot: SurfaceSlot, filePath: string, coordOrder: 0 | 1): Promise<DatSummary> {
    return this.postJson<DatSummary>('/load_dat', { slot, filePath, coordOrder });
  }

  async buildOctree(slot: SurfaceSlot): Promise<OctreeSummary> {
    return this.postJson<OctreeSummary>('/build_octree', { slot });
  }

  async buildTin(slot: SurfaceSlot, decimationGridSize: number, tileSize: number): Promise<TinSummary> {
    return this.postJson<TinSummary>('/build_tin', { slot, decimationGridSize, tileSize });
  }

  async queryVisibleOctreeNodes(slot: SurfaceSlot, query: CameraQueryInput): Promise<VisibleOctreeNodesResult> {
    return this.postJson<VisibleOctreeNodesResult>('/octree/visible', { slot, query });
  }

  async fetchOctreeNodePacket(slot: SurfaceSlot, nodeId: number): Promise<DecodedPointCloudPacket> {
    const resp = await fetch(`${this.baseUrl}/octree/packet/${slot}/${nodeId}`);
    if (!resp.ok) {
      const errText = await resp.text();
      frontendLogger.error('HTTP', () => `拉取八叉树节点 #${nodeId} 失败: ${errText}`);
      throw new Error(errText);
    }
    return decodePointCloudPacket(await resp.arrayBuffer());
  }

  async getDirtyTinTiles(slot: SurfaceSlot): Promise<number[]> {
    return this.postJson<number[]>('/tin/dirty_tiles', { slot });
  }

  async fetchTinTilePacket(slot: SurfaceSlot, tileIndex: number, clearDirtyFlag: boolean): Promise<DecodedTinMeshPacket> {
    const resp = await fetch(`${this.baseUrl}/tin/packet/${slot}/${tileIndex}?clear=${clearDirtyFlag ? 1 : 0}`);
    if (!resp.ok) {
      const errText = await resp.text();
      frontendLogger.error('HTTP', () => `拉取三角网瓦片 #${tileIndex} 失败: ${errText}`);
      throw new Error(errText);
    }
    return decodeTinMeshPacket(await resp.arrayBuffer());
  }

  async queryElevationAt(slot: SurfaceSlot, localX: number, localY: number): Promise<{ localZ: number; globalZ: number } | null> {
    return this.postJson<{ localZ: number; globalZ: number } | null>('/tin/elevation', { slot, localX, localY });
  }

  async pickTinByRay(slot: SurfaceSlot, rayOrigin: [number, number, number], rayDir: [number, number, number]): Promise<RayHitResult> {
    return this.postJson<RayHitResult>('/tin/pick', { slot, rayOrigin, rayDir });
  }

  async flipTinEdge(slot: SurfaceSlot, triangleId: number, edgeSlot: number, forceFlip: boolean): Promise<void> {
    await this.postJson<void>('/tin/flip_edge', { slot, triangleId, edgeSlot, forceFlip });
  }

  async modifyVertexElevation(slot: SurfaceSlot, vertexId: number, newLocalZ: number): Promise<void> {
    await this.postJson<void>('/tin/modify_z', { slot, vertexId, newLocalZ });
  }

  async insertVertex(slot: SurfaceSlot, localXyz: [number, number, number], codeId: number): Promise<number> {
    return this.postJson<number>('/tin/insert_vertex', { slot, localXyz, codeId });
  }

  async insertBreakline(slot: SurfaceSlot, polyline: Array<[number, number, number]>, codeId: number, interpolateZFromTin: boolean): Promise<void> {
    await this.postJson<void>('/tin/insert_breakline', { slot, polyline, codeId, interpolateZFromTin });
  }

  async clipByPolygon(slot: SurfaceSlot, polygon: Array<[number, number, number]>, keepInside: boolean): Promise<void> {
    await this.postJson<void>('/tin/clip_polygon', { slot, polygon, keepInside });
  }

  async undoTin(slot: SurfaceSlot): Promise<void> {
    await this.postJson<void>('/tin/undo', { slot });
  }

  async redoTin(slot: SurfaceSlot): Promise<void> {
    await this.postJson<void>('/tin/redo', { slot });
  }

  async getHistoryState(slot: SurfaceSlot): Promise<{ canUndo: boolean; canRedo: boolean }> {
    return this.postJson<{ canUndo: boolean; canRedo: boolean }>('/tin/history', { slot });
  }

  async computeEarthworkExact(period1Slot: SurfaceSlot, period2Slot: SurfaceSlot, roiPolygon?: Array<[number, number, number]>): Promise<EarthworkSummary> {
    return this.postJson<EarthworkSummary>('/earthwork/exact', { period1Slot, period2Slot, roiPolygon: roiPolygon ?? [] });
  }

  async computeEarthworkGrid(period1Slot: SurfaceSlot, period2Slot: SurfaceSlot, gridStep: number, roiPolygon?: Array<[number, number, number]>): Promise<EarthworkSummary> {
    return this.postJson<EarthworkSummary>('/earthwork/grid', { period1Slot, period2Slot, gridStep, roiPolygon: roiPolygon ?? [] });
  }

  async computeSlopeGrading(originalSlot: SurfaceSlot, basePolygon: Array<[number, number, number]>, stages: SlopeStageSpec[], params: GradingParams, generateMergedTerrain: boolean): Promise<GradingSummary> {
    return this.postJson<GradingSummary>('/grading/compute', { originalSlot, basePolygon, stages, params, generateMergedTerrain });
  }
}

/**
 * 自动探测运行环境并实例化最优通信桥接器
 */
export function createAutoKernelBridge(): IGeoKernelBridge {
  const win = window as unknown as Record<string, unknown>;
  if (typeof win.__TAURI_INTERNALS__ !== 'undefined' || typeof win.__TAURI__ !== 'undefined') {
    return new TauriNativeKernelBridge();
  }
  return new HttpBinaryKernelBridge('/api/geokernel');
}
