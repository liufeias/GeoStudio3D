/**
 * @file geo_protocol.ts
 * @brief 空间几何内核前后端二进制通信协议与类型契约
 *
 * 严格对齐 C++ 端 `include/geo_kernel/core_types.hpp` 与 `geo_kernel_c_api.h`：
 *   - GpuTilePacketHeader: 固定 96 字节紧凑包头（小端序 Little-Endian）
 *   - GpuPointVertex: 固定 16 字节（float32 x, y, z + uint32 packed_attr）
 *   - GpuMeshVertex: 固定 28 字节（float32 x, y, z, nx, ny, nz + uint32 packed_attr）
 *   - IndexBuffer: 紧跟顶点缓冲区之后的 Uint32Array（三角形列表，长度 = index_count）
 *
 * 零拷贝解析说明：
 *   直接基于网络/IPC 传回的底层 ArrayBuffer 构造 TypedArray 视图并送入 Three.js
 *   InterleavedBuffer / BufferAttribute，不产生任何中间 JS 对象数组分配。
 */

export const GPU_PACKET_MAGIC = 0x544f4547; // 小端序 "GEOT"
export const GPU_PACKET_HEADER_BYTES = 96;
export const GPU_POINT_VERTEX_STRIDE = 16;
export const GPU_MESH_VERTEX_STRIDE = 28;

/**
 * 地形模型期数与派生面槽位枚举（与 GeoSurfaceSlot 严格一致）
 */
export enum SurfaceSlot {
  Period1Original = 0,       // 第一期：原始地面点云与三角网
  Period2Target = 1,         // 第二期：竣工/目标/开挖后点云与三角网
  GradingDesignOnly = 2,     // 多级放坡纯设计面三角网（基底+坡面+马道）
  GradingMergedTerrain = 3,  // 多级放坡与原地面沿坡脚线缝合后的完整地形网
}

/**
 * GPU 瓦片载荷类型（与 GpuTilePayloadType 严格一致）
 */
export enum GpuTilePayloadType {
  PointCloudOctreeNode = 1,
  TinSurfaceMesh = 2,
  GradingDesignMesh = 3,
}

/**
 * 96 字节二进制包头解析视图
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
 * 零拷贝解包后的点云八叉树节点数据包
 */
export interface DecodedPointCloudPacket {
  header: ParsedGpuTileHeader;
  /** 原始字节缓冲（可直接绑定到 Three.js InterleavedBuffer，stride = 16 字节） */
  vertexBuffer: ArrayBuffer;
  floatView: Float32Array;
  uint32View: Uint32Array;
}

/**
 * 零拷贝解包后的三角网瓦片网格数据包
 */
export interface DecodedTinMeshPacket {
  header: ParsedGpuTileHeader;
  /** 顶点交错缓冲（stride = 28 字节：pos[3] + normal[3] + packed_attr[1]） */
  vertexBuffer: ArrayBuffer;
  vertexFloatView: Float32Array;
  vertexUint32View: Uint32Array;
  /** 三角形索引缓冲（长度 = indexCount） */
  indices: Uint32Array;
}

/**
 * 解析 96 字节固定包头并校验魔数与缓冲区边界
 */
export function parseGpuTileHeader(buffer: ArrayBuffer): ParsedGpuTileHeader {
  if (buffer.byteLength < GPU_PACKET_HEADER_BYTES) {
    throw new Error(`GPU 数据包长度不足 96 字节: 实际仅 ${buffer.byteLength} 字节`);
  }

  const view = new DataView(buffer);
  const magic = view.getUint32(0, true);
  if (magic !== GPU_PACKET_MAGIC) {
    throw new Error(`非法 GPU 数据包魔数: 0x${magic.toString(16)} (期望 0x544f4547)`);
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
  const expectedBytes =
    GPU_PACKET_HEADER_BYTES + header.vertexCount * GPU_POINT_VERTEX_STRIDE;
  if (buffer.byteLength < expectedBytes) {
    throw new Error(
      `点云数据包截断: 期望 ${expectedBytes} 字节, 实际 ${buffer.byteLength} 字节`
    );
  }

  // slice 仅拷贝有效负载字节并保证 4 字节对齐，供 InterleavedBuffer 直接消费
  const vertexBuffer = buffer.slice(
    GPU_PACKET_HEADER_BYTES,
    GPU_PACKET_HEADER_BYTES + header.vertexCount * GPU_POINT_VERTEX_STRIDE
  );
  return {
    header,
    vertexBuffer,
    floatView: new Float32Array(vertexBuffer),
    uint32View: new Uint32Array(vertexBuffer),
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
      `三角网网格数据包截断: 期望 ${expectedBytes} 字节, 实际 ${buffer.byteLength} 字节`
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
    indices: new Uint32Array(indexBuffer),
  };
}

// ==============================================================================
// 业务摘要与参数接口定义（与 C ABI 结构体字段一一对应）
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
  direction: 0 | 1;         // 0=向外放坡, 1=向内放坡
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

