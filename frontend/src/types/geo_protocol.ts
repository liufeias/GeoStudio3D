/**
 * @file geo_protocol.ts
 * @brief 空间几何内核前后端二进制通信协议、多期槽位契约与零拷贝解包工具
 *
 * 严格逐字节对齐 C++20 端 `include/geo_kernel/core_types.hpp` 与 `geo_kernel_c_api.h`：
 *   - GpuTilePacketHeader: 固定 96 字节紧凑小端序包头 (Magic = 0x544F4547, "GEOT")
 *   - GpuPointVertex:      固定 16 字节 (float32 x, y, z + uint16 code_id + uint16 flags)
 *   - GpuMeshVertex:       固定 28 字节 (float32 x, y, z, nx, ny, nz + uint16 code_id + uint16 flags)
 *   - IndexBuffer:         紧跟顶点缓冲区之后的 Uint32Array（每 3 个索引组成一个逆时针 CCW 三角形）
 *
 * 零拷贝解包与跨 GPU 精度保护设计：
 *   1. 直接在接收到的底层 `ArrayBuffer` 切片上创建 `Float32Array`、`Uint32Array` 与 `Uint16Array` 视图，
 *      不产生任何中间 JavaScript 对象数组分配，支撑亿级点云与千万级三角网的毫秒级显存上传。
 *   2. 针对 `packed_attr`（低 16 位 `code_id`，高 16 位 `flags`），同步提供同缓冲区的 `Uint16Array` 视图，
 *      使 WebGL2 通过 `UNSIGNED_SHORT` 硬件直接解包为精确 `vec2(codeId, flags)`，彻底杜绝 Direct3D11/ANGLE
 *      将小整数位模式误当作次正规浮点数清零（FTZ）的跨平台着色缺陷。
 */

export const GPU_PACKET_MAGIC = 0x544f4547; // 小端序 ASCII "GEOT"
export const GPU_PACKET_HEADER_BYTES = 96;
export const GPU_POINT_VERTEX_STRIDE = 16;
export const GPU_MESH_VERTEX_STRIDE = 28;

/**
 * 工程槽位编号类型（支持 0、1 以及动态分配的 10、11、12……任意期数）
 */
export type SlotId = number;

/**
 * 地形模型预定义槽位常量（与 C ABI GeoSurfaceSlot 严格对齐）
 */
export enum SurfaceSlot {
  /** 第 1 期默认槽位：原始地面点云与三角网 */
  Period1Original = 0,
  /** 第 2 期默认槽位：对比/竣工/开挖后点云与三角网 */
  Period2Target = 1,
  /** 系统保留槽位：多级放坡纯设计面三角网（基底+各级边坡+马道） */
  GradingDesignOnly = 2,
  /** 系统保留槽位：多级放坡与原地面沿坡脚线缝合后的完整设计地形 */
  GradingMergedTerrain = 3,
  /** 动态分配的第 3 期及后续任意用户期数起始槽位编号 */
  UserPeriodStart = 10,
}

/**
 * 判断指定槽位编号是否为系统保留的放坡派生槽位（2 或 3）
 */
export function isSystemDerivedSlot(slot: SlotId): boolean {
  return (
    slot === SurfaceSlot.GradingDesignOnly ||
    slot === SurfaceSlot.GradingMergedTerrain
  );
}

/**
 * 判断指定槽位编号是否为合法的用户测绘土方期数槽位
 */
export function isUserPeriodSlot(slot: SlotId): boolean {
  return Number.isInteger(slot) && slot >= 0 && !isSystemDerivedSlot(slot);
}

/**
 * GPU 瓦片二进制数据包载荷类型（与 C++ GpuTilePayloadType 枚举严格一致）
 */
export enum GpuTilePayloadType {
  PointCloudOctreeNode = 1,
  TinSurfaceMesh = 2,
  GradingDesignMesh = 3,
}

/**
 * 96 字节固定二进制包头解析视图
 */
export interface ParsedGpuTileHeader {
  magic: number;
  version: number;
  payloadType: GpuTilePayloadType;
  flags: number;
  tileX: number;
  tileY: number;
  tileUid: bigint;
  revision: number;
  vertexCount: number;
  indexCount: number;
  vertexStride: number;
  rtcOrigin: [number, number, number];
  bboxMin: [number, number, number];
  bboxMax: [number, number, number];
}

/**
 * 零拷贝解包后的点云八叉树节点 GPU 数据包
 */
export interface DecodedPointCloudPacket {
  header: ParsedGpuTileHeader;
  /** 16 字节步长原始交错顶点字节缓冲 */
  vertexBuffer: ArrayBuffer;
  /** 坐标视图（stride = 4 float32：x, y, z, packedAttrRaw） */
  floatView: Float32Array;
  /** 32 位整数字段视图 */
  uint32View: Uint32Array;
  /** 16 位无符号整数字段视图（stride = 8 uint16：偏移 6 为 code_id，偏移 7 为 flags） */
  uint16View: Uint16Array;
}

/**
 * 零拷贝解包后的三角网空间瓦片 GPU 网格数据包
 */
export interface DecodedTinMeshPacket {
  header: ParsedGpuTileHeader;
  /** 28 字节步长原始交错顶点字节缓冲（pos[3] + normal[3] + packed_attr[1]） */
  vertexBuffer: ArrayBuffer;
  /** 浮点视图（stride = 7 float32：x, y, z, nx, ny, nz, packedAttrRaw） */
  vertexFloatView: Float32Array;
  /** 32 位整数字段视图 */
  vertexUint32View: Uint32Array;
  /** 16 位无符号整数字段视图（stride = 14 uint16：偏移 12 为 code_id，偏移 13 为 flags） */
  vertexUint16View: Uint16Array;
  /** 三角形索引缓冲（长度 = indexCount，每 3 个构成 1 个逆时针三角形） */
  indices: Uint32Array;
}

/**
 * 解析 96 字节紧凑二进制包头，并执行魔数与字节边界完整性校验
 */
export function parseGpuTileHeader(buffer: ArrayBuffer): ParsedGpuTileHeader {
  if (buffer.byteLength < GPU_PACKET_HEADER_BYTES) {
    throw new Error(
      `GPU 二进制数据包头长度不足 96 字节: 实际仅收到 ${buffer.byteLength} 字节`
    );
  }
  const view = new DataView(buffer);
  const magic = view.getUint32(0, true);
  if (magic !== GPU_PACKET_MAGIC) {
    throw new Error(
      `非法 GPU 数据包魔数: 0x${magic.toString(16).toUpperCase()} (期望 0x544F4547 "GEOT")`
    );
  }
  return {
    magic,
    version: view.getUint32(4, true),
    payloadType: view.getUint32(8, true) as GpuTilePayloadType,
    flags: view.getUint32(12, true),
    tileX: view.getInt32(16, true),
    tileY: view.getInt32(20, true),
    tileUid: view.getBigUint64(24, true),
    revision: view.getUint32(32, true),
    vertexCount: view.getUint32(36, true),
    indexCount: view.getUint32(40, true),
    vertexStride: view.getUint32(44, true),
    rtcOrigin: [
      view.getFloat64(48, true),
      view.getFloat64(56, true),
      view.getFloat64(64, true),
    ],
    bboxMin: [
      view.getFloat32(72, true),
      view.getFloat32(76, true),
      view.getFloat32(80, true),
    ],
    bboxMax: [
      view.getFloat32(84, true),
      view.getFloat32(88, true),
      view.getFloat32(92, true),
    ],
  };
}

/**
 * 零拷贝解析点云八叉树节点 GPU 数据包
 */
export function decodePointCloudPacket(buffer: ArrayBuffer): DecodedPointCloudPacket {
  const header = parseGpuTileHeader(buffer);
  const vertexBytes = header.vertexCount * GPU_POINT_VERTEX_STRIDE;
  const expectedBytes = GPU_PACKET_HEADER_BYTES + vertexBytes;
  if (buffer.byteLength < expectedBytes) {
    throw new Error(
      `点云八叉树数据包被截断: 期望至少 ${expectedBytes} 字节, 实际收到 ${buffer.byteLength} 字节`
    );
  }
  const vertexBuffer = buffer.slice(
    GPU_PACKET_HEADER_BYTES,
    GPU_PACKET_HEADER_BYTES + vertexBytes
  );
  return {
    header,
    vertexBuffer,
    floatView: new Float32Array(vertexBuffer),
    uint32View: new Uint32Array(vertexBuffer),
    uint16View: new Uint16Array(vertexBuffer),
  };
}

/**
 * 零拷贝解析三角网空间瓦片 GPU 网格数据包
 */
export function decodeTinMeshPacket(buffer: ArrayBuffer): DecodedTinMeshPacket {
  const header = parseGpuTileHeader(buffer);
  const vertexBytes = header.vertexCount * GPU_MESH_VERTEX_STRIDE;
  const indexBytes = header.indexCount * 4;
  const expectedBytes = GPU_PACKET_HEADER_BYTES + vertexBytes + indexBytes;
  if (buffer.byteLength < expectedBytes) {
    throw new Error(
      `三角网瓦片数据包被截断: 期望至少 ${expectedBytes} 字节, 实际收到 ${buffer.byteLength} 字节`
    );
  }
  if (header.indexCount % 3 !== 0) {
    throw new Error(
      `三角网瓦片索引数量不是 3 的整数倍 (indexCount=${header.indexCount})，数据可能已损坏`
    );
  }
  const vertexBuffer = buffer.slice(
    GPU_PACKET_HEADER_BYTES,
    GPU_PACKET_HEADER_BYTES + vertexBytes
  );
  const indexBuffer = buffer.slice(
    GPU_PACKET_HEADER_BYTES + vertexBytes,
    GPU_PACKET_HEADER_BYTES + vertexBytes + indexBytes
  );
  return {
    header,
    vertexBuffer,
    vertexFloatView: new Float32Array(vertexBuffer),
    vertexUint32View: new Uint32Array(vertexBuffer),
    vertexUint16View: new Uint16Array(vertexBuffer),
    indices: new Uint32Array(indexBuffer),
  };
}

// ==============================================================================
// 业务摘要与多期工程元数据契约（与 C ABI / Rust DTO 字段严格一一对应）
// ==============================================================================

export interface DatSummary {
  fileBytes: number;
  totalLinesScanned: number;
  validPoints: number;
  skippedLines: number;
  malformedLines: number;
  filteredOutliers: number;
  elapsedSeconds: number;
  throughputMptsPerSec: number;
  rtcOrigin: [number, number, number];
  globalMin: [number, number, number];
  globalMax: [number, number, number];
  localMin: [number, number, number];
  localMax: [number, number, number];
}

export interface OctreeSummary {
  totalPoints: number;
  totalNodes: number;
  leafNodes: number;
  maxReachedDepth: number;
  elapsedSeconds: number;
}

export interface TinSummary {
  inputPoints: number;
  deduplicatedPoints: number;
  decimatedPoints: number;
  tinVertices: number;
  tinTriangles: number;
  totalTiles: number;
  elapsedSeconds: number;
}

export interface ElevationQueryResult {
  localZ: number;
  globalZ: number;
}

export interface CameraQueryInput {
  cameraLocalPos: [number, number, number];
  frustumPlanes: number[][]; // 6 x 4
  viewportHeightPx: number;
  fovYRadians: number;
  sseThresholdPx: number;
  maxPointBudget: number;
  enableFrustumCulling: boolean;
}

export interface VisibleOctreeNodesResult {
  nodeIds: number[];
  estimatedPoints: number;
}

export interface RayHitResult {
  hit: boolean;
  triangleId: number;
  nearestVertexId: number;
  nearestEdgeSlot: number;
  distance: number;
  hitLocalPos: [number, number, number];
}

export interface EarthworkSummary {
  cutVolumeM3: number;
  fillVolumeM3: number;
  netVolumeM3: number;
  totalOverlapAreaM2: number;
  cutAreaM2: number;
  fillAreaM2: number;
  zeroAreaM2: number;
  maxCutDepthM: number;
  maxFillHeightM: number;
  intersectedPrisms: number;
  elapsedSeconds: number;
}

export interface SlopeStageSpec {
  stageHeight: number;
  slopeRatio: number;
  bermWidth: number;
  bermSlopeRatio: number;
}

export interface GradingParams {
  direction: 0 | 1; // 0=向外放坡, 1=向内放坡
  verticalMode: -1 | 0 | 1; // 0=自动挖填, -1=强制向下, 1=强制向上
  useUniformBaseElevation: boolean;
  baseElevation: number;
  autoDaylightToTerrain: boolean;
  capInwardBottom: boolean;
  perimeterSampleStep: number;
  outputTileSize: number;
}

export interface GradingSummary {
  earthwork: EarthworkSummary;
  daylightPerimeterM: number;
  slopeSurfaceAreaM2: number;
  bermSurfaceAreaM2: number;
  basePadAreaM2: number;
  actualStagesUsed: number;
  daylightVertexCount: number;
  daylightConverged: boolean;
  elapsedSeconds: number;
  daylightPolyline: Array<[number, number, number]>;
}

/**
 * 前端任意期数土方数据图层元数据模型
 */
export interface PeriodRecord {
  /** 对应 C++ 内核中的唯一动态槽位号 (0, 1, 10, 11, ...) */
  slot: SlotId;
  /** 期数序号（第 1 期、第 2 期……第 N 期，便于界面友好展示） */
  periodIndex: number;
  /** 用户可自定义重命名的期数显示名称 */
  name: string;
  /** 原始 .dat 文件完整路径 */
  filePath: string;
  /** 原始文件短名称 */
  fileName: string;
  /** 期数主题标识色（HEX，如 #38bdf8，用于多期叠加区分与图例） */
  colorHex: string;
  /** 是否显示该期的点云八叉树 */
  showPointCloud: boolean;
  /** 是否显示该期的三角网表面 */
  showTin: boolean;
  /** 该期三角网不透明度 [0.15 .. 1.0] */
  tinOpacity: number;
  /** 该期三角网构网抽稀网格步长（米） */
  decimationStep: number;
  /** 该期三角网外围最大允许边长裁剪阈值（米，0=不裁剪） */
  maxEdgeLength: number;
  /** 点云导入统计与坐标范围摘要 */
  datSummary: DatSummary;
  /** 八叉树构建摘要 */
  octreeSummary: OctreeSummary | null;
  /** 三角网构建摘要 */
  tinSummary: TinSummary | null;
}

/**
 * 预置高区分度工程期数配色循环色板（覆盖 12 种视觉差异显著且适配暗色工程界面的配色）
 */
export const PERIOD_COLOR_PALETTE: readonly string[] = [
  '#22c55e', // 第 1 期：翠绿（默认原地面）
  '#38bdf8', // 第 2 期：天蓝（默认第二期）
  '#f59e0b', // 第 3 期：琥珀橙
  '#a855f7', // 第 4 期：紫罗兰
  '#ec4899', // 第 5 期：玫瑰红
  '#14b8a6', // 第 6 期：青碧色
  '#eab308', // 第 7 期：明黄
  '#6366f1', // 第 8 期：靛蓝
  '#fb7185', // 第 9 期：珊瑚粉
  '#84cc16', // 第 10 期：青柠绿
  '#06b6d4', // 第 11 期：湖蓝
  '#f97316', // 第 12 期：深橙
] as const;

