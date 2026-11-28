/**
 * @file kernel_bridge.ts
 * @brief 多平台解耦几何内核通信抽象层与前端高精度调试日志器
 *
 * 企业级架构特性：
 *   1. 全功能任意期数动态管理：
 *      - 支持动态创建新期数槽位 (`createPeriodSlot`)、安全卸载单期 (`removePeriodSlot`)、
 *        清空工程全部期数 (`clearAllPeriods`) 及批量多选 `.dat` 点云文件导入 (`pickDatFilesDialog`)。
 *   2. 跨平台零拷贝二进制瓦片流网关：
 *      - 自动适配 Windows WebView2 (`http://geotile.localhost/...`) 与 macOS/Linux (`geotile://localhost/...`)；
 *      - 强制 `cache: 'no-store'` 并携带递增版本号参数，杜绝编辑或重载后读取 WebView 陈旧瓦片缓存。
 *   3. Debug 专属带毫秒时间戳日志器（Release 零开销短路）：
 *      - Debug 模式下同步输出至 DevTools 与原生后端持久化日志文件；
 *      - Release 模式下惰性日志回调直接短路，零字符串拼接与零 IPC 开销。
 */

import {
  CameraQueryInput,
  DatSummary,
  DecodedPointCloudPacket,
  DecodedTinMeshPacket,
  EarthworkSummary,
  ElevationQueryResult,
  GradingParams,
  GradingSummary,
  OctreeSummary,
  RayHitResult,
  SlopeStageSpec,
  SlotId,
  TinSummary,
  VisibleOctreeNodesResult,
  decodePointCloudPacket,
  decodeTinMeshPacket,
} from '../types/geo_protocol';

export type LogLevel = 'DEBUG' | 'INFO' | 'WARN' | 'ERROR';
export type LazyLogMessage = string | (() => string);

interface PendingBootLog {
  level: LogLevel;
  module: string;
  message: string;
  timestamp: string;
}

/**
 * 格式化本地高精度时间戳 `YYYY-MM-DD HH:MM:SS.mmm`
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
 * 提取任意异常对象的完整中文诊断文本（含错误堆栈与嵌套 cause）
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
    const importMeta = import.meta as unknown as { env?: { DEV?: boolean } };
    this.debugEnabled = importMeta.env?.DEV === true;
  }

  isEnabled(): boolean {
    return this.debugEnabled;
  }

  async initWithBridge(bridge: IGeoKernelBridge): Promise<void> {
    this.bridge = bridge;
    try {
      this.debugEnabled = await bridge.isDebugMode();
    } catch {
      this.debugEnabled = false;
    }
    this.initialized = true;

    if (!this.debugEnabled) {
      this.bootQueue.length = 0;
      return;
    }

    while (this.bootQueue.length > 0) {
      const item = this.bootQueue.shift();
      if (item) {
        this.emitToConsole(item.timestamp, item.level, item.module, item.message);
        void this.forwardToBackend(
          item.level,
          item.module,
          `[启动缓冲 @ ${item.timestamp}] ${item.message}`
        );
      }
    }

    this.info(
      'LOGGER',
      () =>
        `前端高精度调试日志器初始化完成 | 通信后端: ${bridge.backendName} | 调试模式: 已开启`
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
    const fullMsg =
      typeof err !== 'undefined' ? `${base} | 详情: ${formatErrorDetails(err)}` : base;
    this.dispatch('WARN', module, fullMsg);
  }

  error(module: string, msgOrFn: LazyLogMessage, err?: unknown): void {
    if (!this.shouldCapture()) return;
    const base = typeof msgOrFn === 'function' ? msgOrFn() : msgOrFn;
    const fullMsg =
      typeof err !== 'undefined' ? `${base} | 异常详情: ${formatErrorDetails(err)}` : base;
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

  private emitToConsole(
    ts: string,
    level: LogLevel,
    module: string,
    message: string
  ): void {
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

  private async forwardToBackend(
    level: LogLevel,
    module: string,
    message: string
  ): Promise<void> {
    if (!this.bridge || !this.debugEnabled || this.isForwarding) {
      return;
    }
    this.isForwarding = true;
    try {
      await this.bridge.writeDebugLog(level, module, message);
    } catch {
      // 防止日志通道异常触发递归错误
    } finally {
      this.isForwarding = false;
    }
  }
}

export const frontendLogger = new FrontendDebugLogger();

/**
 * 空间几何内核跨平台异步通信统一契约接口
 */
export interface IGeoKernelBridge {
  readonly backendName: string;

  isDebugMode(): Promise<boolean>;
  writeDebugLog(level: LogLevel, module: string, message: string): Promise<void>;

  /** 打开原生文件选择对话框，支持单选或批量多选 .dat/.txt/.csv 测绘点云文件 */
  pickDatFilesDialog(title: string, allowMultiple?: boolean): Promise<string[]>;
  /** 便捷单文件选择接口 */
  pickDatFileDialog(title: string): Promise<string | null>;

  /** 动态期数槽位生命周期管理 */
  createPeriodSlot(): Promise<SlotId>;
  removePeriodSlot(slot: SlotId): Promise<void>;
  clearAllPeriods(): Promise<void>;
  listPeriodSlots(): Promise<SlotId[]>;

  /** 点云导入、八叉树与分块三角网构建 */
  loadDatFile(slot: SlotId, filePath: string, coordOrder: 0 | 1): Promise<DatSummary>;
  buildOctree(slot: SlotId): Promise<OctreeSummary>;
  buildTin(
    slot: SlotId,
    decimationGridSize: number,
    tileSize: number,
    maxEdgeLength?: number
  ): Promise<TinSummary>;

  /** 视锥体 LOD 八叉树与脏瓦片 GPU 数据流 */
  queryVisibleOctreeNodes(
    slot: SlotId,
    query: CameraQueryInput
  ): Promise<VisibleOctreeNodesResult>;
  fetchOctreeNodePacket(slot: SlotId, nodeId: number): Promise<DecodedPointCloudPacket>;
  getDirtyTinTiles(slot: SlotId): Promise<number[]>;
  fetchTinTilePacket(
    slot: SlotId,
    tileIndex: number,
    clearDirtyFlag: boolean
  ): Promise<DecodedTinMeshPacket>;

  /** 高程内插查询与三维射线拾取 */
  queryElevationAt(
    slot: SlotId,
    localX: number,
    localY: number
  ): Promise<ElevationQueryResult | null>;
  pickTinByRay(
    slot: SlotId,
    rayOrigin: [number, number, number],
    rayDir: [number, number, number]
  ): Promise<RayHitResult>;

  /** 交互式三角网拓扑编辑与多步增量 Undo/Redo */
  flipTinEdge(
    slot: SlotId,
    triangleId: number,
    edgeSlot: number,
    forceFlip: boolean
  ): Promise<void>;
  modifyVertexElevation(
    slot: SlotId,
    vertexId: number,
    newLocalZ: number
  ): Promise<void>;
  insertVertex(
    slot: SlotId,
    localXyz: [number, number, number],
    codeId: number
  ): Promise<number>;
  insertBreakline(
    slot: SlotId,
    polyline: Array<[number, number, number]>,
    codeId: number,
    interpolateZFromTin: boolean
  ): Promise<void>;
  deleteVertex(slot: SlotId, vertexId: number): Promise<void>;
  clipByPolygon(
    slot: SlotId,
    polygon: Array<[number, number, number]>,
    keepInside: boolean
  ): Promise<void>;
  undoTin(slot: SlotId): Promise<void>;
  redoTin(slot: SlotId): Promise<void>;
  getHistoryState(slot: SlotId): Promise<{ canUndo: boolean; canRedo: boolean }>;

  /** 任意两期土方算量与多级放坡建模 */
  computeEarthworkExact(
    period1Slot: SlotId,
    period2Slot: SlotId,
    roiPolygon?: Array<[number, number, number]>
  ): Promise<EarthworkSummary>;
  computeEarthworkGrid(
    period1Slot: SlotId,
    period2Slot: SlotId,
    gridStep: number,
    roiPolygon?: Array<[number, number, number]>
  ): Promise<EarthworkSummary>;
  computeSlopeGrading(
    originalSlot: SlotId,
    basePolygon: Array<[number, number, number]>,
    stages: SlopeStageSpec[],
    params: GradingParams,
    generateMergedTerrain: boolean
  ): Promise<GradingSummary>;
}

// ==============================================================================
// 实现 1：Tauri v2 原生桌面桥接器（Rust FFI + geotile:// 零拷贝二进制瓦片网关）
// ==============================================================================
export class TauriNativeKernelBridge implements IGeoKernelBridge {
  readonly backendName = 'Tauri v2 Native C++20 Multi-Period Kernel';
  private fetchSeq = 0;

  /**
   * 构造跨平台兼容且防缓存污染的 geotile 自定义协议 URL
   * - Windows (WebView2): `http://geotile.localhost/...`
   * - macOS / Linux (WKWebView / WebKitGTK): `geotile://localhost/...`
   */
  private buildGeoTileUrl(path: string, queryParams?: Record<string, string | number>): string {
    const isWindows = /windows/i.test(navigator.userAgent);
    const origin = isWindows ? 'http://geotile.localhost' : 'geotile://localhost';
    const params = new URLSearchParams();
    if (queryParams) {
      for (const [k, v] of Object.entries(queryParams)) {
        params.set(k, String(v));
      }
    }
    params.set('_seq', String(++this.fetchSeq));
    return `${origin}${path}?${params.toString()}`;
  }

  private async invokeCmd<T>(
    cmd: string,
    args?: Record<string, unknown>,
    skipLog = false
  ): Promise<T> {
    const logOn = !skipLog && frontendLogger.isEnabled();
    const startMs = logOn ? performance.now() : 0;
    if (logOn) {
      frontendLogger.debug(
        'IPC',
        () => `调用原生命令 ${cmd} | 参数: ${JSON.stringify(args ?? {})}`
      );
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

  async pickDatFilesDialog(title: string, allowMultiple = true): Promise<string[]> {
    frontendLogger.debug('DIALOG', () => `打开原生文件选择框: "${title}", 多选=${allowMultiple}`);
    try {
      const { open } = await import('@tauri-apps/plugin-dialog');
      const selected = await open({
        title,
        multiple: allowMultiple,
        filters: [
          { name: '测绘点云数据 (*.dat, *.txt, *.csv)', extensions: ['dat', 'txt', 'csv'] },
          { name: '所有文件', extensions: ['*'] },
        ],
      });
      if (Array.isArray(selected)) {
        const list = selected.filter((p): p is string => typeof p === 'string' && p.length > 0);
        frontendLogger.info('DIALOG', () => `用户批量选中 ${list.length} 个点云文件`);
        return list;
      }
      if (typeof selected === 'string' && selected.length > 0) {
        frontendLogger.info('DIALOG', () => `用户选中单个点云文件: ${selected}`);
        return [selected];
      }
      frontendLogger.debug('DIALOG', '用户取消了文件选择');
      return [];
    } catch (err) {
      frontendLogger.error('DIALOG', () => `文件选择对话框调用异常 (${title})`, err);
      throw err;
    }
  }

  async pickDatFileDialog(title: string): Promise<string | null> {
    const files = await this.pickDatFilesDialog(title, false);
    return files.length > 0 ? files[0] : null;
  }

  async createPeriodSlot(): Promise<SlotId> {
    return this.invokeCmd<SlotId>('cmd_create_period_slot');
  }

  async removePeriodSlot(slot: SlotId): Promise<void> {
    await this.invokeCmd<void>('cmd_remove_period_slot', { slot });
  }

  async clearAllPeriods(): Promise<void> {
    await this.invokeCmd<void>('cmd_clear_all_periods');
  }

  async listPeriodSlots(): Promise<SlotId[]> {
    return this.invokeCmd<SlotId[]>('cmd_list_period_slots');
  }

  async loadDatFile(slot: SlotId, filePath: string, coordOrder: 0 | 1): Promise<DatSummary> {
    return this.invokeCmd<DatSummary>('cmd_load_dat', {
      slot,
      filePath,
      coordOrder,
    });
  }

  async buildOctree(slot: SlotId): Promise<OctreeSummary> {
    return this.invokeCmd<OctreeSummary>('cmd_build_octree', { slot });
  }

  async buildTin(
    slot: SlotId,
    decimationGridSize: number,
    tileSize: number,
    maxEdgeLength = 0
  ): Promise<TinSummary> {
    return this.invokeCmd<TinSummary>('cmd_build_tin', {
      slot,
      decimationGridSize,
      tileSize,
      maxEdgeLength,
    });
  }

  async queryVisibleOctreeNodes(
    slot: SlotId,
    query: CameraQueryInput
  ): Promise<VisibleOctreeNodesResult> {
    return this.invokeCmd<VisibleOctreeNodesResult>(
      'cmd_query_visible_octree_nodes',
      { slot, query },
      true
    );
  }

  async fetchOctreeNodePacket(
    slot: SlotId,
    nodeId: number
  ): Promise<DecodedPointCloudPacket> {
    const url = this.buildGeoTileUrl(`/octree/${slot}/${nodeId}`);
    const logOn = frontendLogger.isEnabled();
    const startMs = logOn ? performance.now() : 0;
    const response = await fetch(url, { cache: 'no-store' });
    if (!response.ok) {
      const errText = await response.text();
      const msg = `拉取八叉树节点 #${nodeId} (槽位 ${slot}) 失败 (HTTP ${response.status}): ${errText}`;
      frontendLogger.error('GEOTILE', msg);
      throw new Error(msg);
    }
    const buffer = await response.arrayBuffer();
    const pkt = decodePointCloudPacket(buffer);
    if (logOn) {
      const costMs = (performance.now() - startMs).toFixed(2);
      frontendLogger.debug(
        'GEOTILE',
        () =>
          `拉取八叉树节点 #${nodeId} (槽位 ${slot}) 成功: ${pkt.header.vertexCount} 点, ${buffer.byteLength} 字节, 耗时 ${costMs} ms`
      );
    }
    return pkt;
  }

  async getDirtyTinTiles(slot: SlotId): Promise<number[]> {
    return this.invokeCmd<number[]>('cmd_get_dirty_tin_tiles', { slot });
  }

  async fetchTinTilePacket(
    slot: SlotId,
    tileIndex: number,
    clearDirtyFlag: boolean
  ): Promise<DecodedTinMeshPacket> {
    const url = this.buildGeoTileUrl(`/tin/${slot}/${tileIndex}`, {
      clear: clearDirtyFlag ? 1 : 0,
    });
    const logOn = frontendLogger.isEnabled();
    const startMs = logOn ? performance.now() : 0;
    const response = await fetch(url, { cache: 'no-store' });
    if (!response.ok) {
      const errText = await response.text();
      const msg = `拉取三角网瓦片 #${tileIndex} (槽位 ${slot}) 失败 (HTTP ${response.status}): ${errText}`;
      frontendLogger.error('GEOTILE', msg);
      throw new Error(msg);
    }
    const buffer = await response.arrayBuffer();
    const pkt = decodeTinMeshPacket(buffer);
    if (logOn) {
      const costMs = (performance.now() - startMs).toFixed(2);
      frontendLogger.debug(
        'GEOTILE',
        () =>
          `拉取三角网瓦片 #${tileIndex} (槽位 ${slot}, rev=${pkt.header.revision}) 成功: ${pkt.header.vertexCount} 顶点, ${Math.floor(pkt.header.indexCount / 3)} 三角形, ${buffer.byteLength} 字节, 耗时 ${costMs} ms`
      );
    }
    return pkt;
  }

  async queryElevationAt(
    slot: SlotId,
    localX: number,
    localY: number
  ): Promise<ElevationQueryResult | null> {
    return this.invokeCmd<ElevationQueryResult | null>('cmd_query_elevation', {
      slot,
      localX,
      localY,
    });
  }

  async pickTinByRay(
    slot: SlotId,
    rayOrigin: [number, number, number],
    rayDir: [number, number, number]
  ): Promise<RayHitResult> {
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
    slot: SlotId,
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
    slot: SlotId,
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
    slot: SlotId,
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
    slot: SlotId,
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

  async deleteVertex(slot: SlotId, vertexId: number): Promise<void> {
    await this.invokeCmd<void>('cmd_tin_delete_vertex', { slot, vertexId });
  }

  async clipByPolygon(
    slot: SlotId,
    polygon: Array<[number, number, number]>,
    keepInside: boolean
  ): Promise<void> {
    await this.invokeCmd<void>('cmd_tin_clip_polygon', {
      slot,
      polygon,
      keepInside,
    });
  }

  async undoTin(slot: SlotId): Promise<void> {
    await this.invokeCmd<void>('cmd_tin_undo', { slot });
  }

  async redoTin(slot: SlotId): Promise<void> {
    await this.invokeCmd<void>('cmd_tin_redo', { slot });
  }

  async getHistoryState(slot: SlotId): Promise<{ canUndo: boolean; canRedo: boolean }> {
    return this.invokeCmd<{ canUndo: boolean; canRedo: boolean }>(
      'cmd_tin_get_history_state',
      { slot }
    );
  }

  async computeEarthworkExact(
    period1Slot: SlotId,
    period2Slot: SlotId,
    roiPolygon?: Array<[number, number, number]>
  ): Promise<EarthworkSummary> {
    return this.invokeCmd<EarthworkSummary>('cmd_compute_earthwork_exact', {
      period1Slot,
      period2Slot,
      roiPolygon: roiPolygon ?? [],
    });
  }

  async computeEarthworkGrid(
    period1Slot: SlotId,
    period2Slot: SlotId,
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
    originalSlot: SlotId,
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
  readonly backendName = 'Web Remote C++20 Multi-Period Compute Server';
  private readonly baseUrl: string;

  constructor(baseUrl = '/api/geokernel') {
    this.baseUrl = baseUrl;
  }

  async isDebugMode(): Promise<boolean> {
    const importMeta = import.meta as unknown as { env?: { DEV?: boolean } };
    return importMeta.env?.DEV === true;
  }

  async writeDebugLog(_level: LogLevel, _module: string, _message: string): Promise<void> {
    // 浏览器模式下已输出至开发者控制台，此处无需额外远程请求
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
      frontendLogger.error(
        'HTTP',
        () => `POST ${endpoint} 返回异常状态 ${resp.status}: ${errText}`
      );
      throw new Error(errText);
    }
    const data = (await resp.json()) as T;
    if (logOn) {
      const costMs = (performance.now() - startMs).toFixed(2);
      frontendLogger.debug('HTTP', () => `POST ${endpoint} 完成 (耗时 ${costMs} ms)`);
    }
    return data;
  }

  async pickDatFilesDialog(title: string, allowMultiple = true): Promise<string[]> {
    const hint = allowMultiple
      ? `${title} — 请输入服务器侧 .dat 文件路径（多个路径用分号 ; 分隔）:`
      : `${title} — 请输入服务器侧 .dat 文件路径:`;
    const raw = window.prompt(hint);
    if (!raw) return [];
    return raw
      .split(';')
      .map((s) => s.trim())
      .filter((s) => s.length > 0);
  }

  async pickDatFileDialog(title: string): Promise<string | null> {
    const list = await this.pickDatFilesDialog(title, false);
    return list.length > 0 ? list[0] : null;
  }

  async createPeriodSlot(): Promise<SlotId> {
    return this.postJson<SlotId>('/periods/create', {});
  }

  async removePeriodSlot(slot: SlotId): Promise<void> {
    await this.postJson<void>('/periods/remove', { slot });
  }

  async clearAllPeriods(): Promise<void> {
    await this.postJson<void>('/periods/clear', {});
  }

  async listPeriodSlots(): Promise<SlotId[]> {
    return this.postJson<SlotId[]>('/periods/list', {});
  }

  async loadDatFile(slot: SlotId, filePath: string, coordOrder: 0 | 1): Promise<DatSummary> {
    return this.postJson<DatSummary>('/load_dat', { slot, filePath, coordOrder });
  }

  async buildOctree(slot: SlotId): Promise<OctreeSummary> {
    return this.postJson<OctreeSummary>('/build_octree', { slot });
  }

  async buildTin(
    slot: SlotId,
    decimationGridSize: number,
    tileSize: number,
    maxEdgeLength = 0
  ): Promise<TinSummary> {
    return this.postJson<TinSummary>('/build_tin', {
      slot,
      decimationGridSize,
      tileSize,
      maxEdgeLength,
    });
  }

  async queryVisibleOctreeNodes(
    slot: SlotId,
    query: CameraQueryInput
  ): Promise<VisibleOctreeNodesResult> {
    return this.postJson<VisibleOctreeNodesResult>('/octree/visible', { slot, query });
  }

  async fetchOctreeNodePacket(
    slot: SlotId,
    nodeId: number
  ): Promise<DecodedPointCloudPacket> {
    const resp = await fetch(`${this.baseUrl}/octree/packet/${slot}/${nodeId}`, {
      cache: 'no-store',
    });
    if (!resp.ok) {
      const errText = await resp.text();
      frontendLogger.error('HTTP', () => `拉取八叉树节点 #${nodeId} 失败: ${errText}`);
      throw new Error(errText);
    }
    return decodePointCloudPacket(await resp.arrayBuffer());
  }

  async getDirtyTinTiles(slot: SlotId): Promise<number[]> {
    return this.postJson<number[]>('/tin/dirty_tiles', { slot });
  }

  async fetchTinTilePacket(
    slot: SlotId,
    tileIndex: number,
    clearDirtyFlag: boolean
  ): Promise<DecodedTinMeshPacket> {
    const resp = await fetch(
      `${this.baseUrl}/tin/packet/${slot}/${tileIndex}?clear=${clearDirtyFlag ? 1 : 0}`,
      { cache: 'no-store' }
    );
    if (!resp.ok) {
      const errText = await resp.text();
      frontendLogger.error('HTTP', () => `拉取三角网瓦片 #${tileIndex} 失败: ${errText}`);
      throw new Error(errText);
    }
    return decodeTinMeshPacket(await resp.arrayBuffer());
  }

  async queryElevationAt(
    slot: SlotId,
    localX: number,
    localY: number
  ): Promise<ElevationQueryResult | null> {
    return this.postJson<ElevationQueryResult | null>('/tin/elevation', {
      slot,
      localX,
      localY,
    });
  }

  async pickTinByRay(
    slot: SlotId,
    rayOrigin: [number, number, number],
    rayDir: [number, number, number]
  ): Promise<RayHitResult> {
    return this.postJson<RayHitResult>('/tin/pick', { slot, rayOrigin, rayDir });
  }

  async flipTinEdge(
    slot: SlotId,
    triangleId: number,
    edgeSlot: number,
    forceFlip: boolean
  ): Promise<void> {
    await this.postJson<void>('/tin/flip_edge', {
      slot,
      triangleId,
      edgeSlot,
      forceFlip,
    });
  }

  async modifyVertexElevation(
    slot: SlotId,
    vertexId: number,
    newLocalZ: number
  ): Promise<void> {
    await this.postJson<void>('/tin/modify_z', { slot, vertexId, newLocalZ });
  }

  async insertVertex(
    slot: SlotId,
    localXyz: [number, number, number],
    codeId: number
  ): Promise<number> {
    return this.postJson<number>('/tin/insert_vertex', { slot, localXyz, codeId });
  }

  async insertBreakline(
    slot: SlotId,
    polyline: Array<[number, number, number]>,
    codeId: number,
    interpolateZFromTin: boolean
  ): Promise<void> {
    await this.postJson<void>('/tin/insert_breakline', {
      slot,
      polyline,
      codeId,
      interpolateZFromTin,
    });
  }

  async deleteVertex(slot: SlotId, vertexId: number): Promise<void> {
    await this.postJson<void>('/tin/delete_vertex', { slot, vertexId });
  }

  async clipByPolygon(
    slot: SlotId,
    polygon: Array<[number, number, number]>,
    keepInside: boolean
  ): Promise<void> {
    await this.postJson<void>('/tin/clip_polygon', { slot, polygon, keepInside });
  }

  async undoTin(slot: SlotId): Promise<void> {
    await this.postJson<void>('/tin/undo', { slot });
  }

  async redoTin(slot: SlotId): Promise<void> {
    await this.postJson<void>('/tin/redo', { slot });
  }

  async getHistoryState(slot: SlotId): Promise<{ canUndo: boolean; canRedo: boolean }> {
    return this.postJson<{ canUndo: boolean; canRedo: boolean }>('/tin/history', { slot });
  }

  async computeEarthworkExact(
    period1Slot: SlotId,
    period2Slot: SlotId,
    roiPolygon?: Array<[number, number, number]>
  ): Promise<EarthworkSummary> {
    return this.postJson<EarthworkSummary>('/earthwork/exact', {
      period1Slot,
      period2Slot,
      roiPolygon: roiPolygon ?? [],
    });
  }

  async computeEarthworkGrid(
    period1Slot: SlotId,
    period2Slot: SlotId,
    gridStep: number,
    roiPolygon?: Array<[number, number, number]>
  ): Promise<EarthworkSummary> {
    return this.postJson<EarthworkSummary>('/earthwork/grid', {
      period1Slot,
      period2Slot,
      gridStep,
      roiPolygon: roiPolygon ?? [],
    });
  }

  async computeSlopeGrading(
    originalSlot: SlotId,
    basePolygon: Array<[number, number, number]>,
    stages: SlopeStageSpec[],
    params: GradingParams,
    generateMergedTerrain: boolean
  ): Promise<GradingSummary> {
    return this.postJson<GradingSummary>('/grading/compute', {
      originalSlot,
      basePolygon,
      stages,
      params,
      generateMergedTerrain,
    });
  }
}

/**
 * 自动探测运行环境并实例化最优内核通信桥接器
 */
export function createAutoKernelBridge(): IGeoKernelBridge {
  const win = window as unknown as Record<string, unknown>;
  if (
    typeof win.__TAURI_INTERNALS__ !== 'undefined' ||
    typeof win.__TAURI__ !== 'undefined'
  ) {
    return new TauriNativeKernelBridge();
  }
  return new HttpBinaryKernelBridge('/api/geokernel');
}

