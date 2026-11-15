/**
 * @file octree_lod_manager.ts
 * @brief 亿级点云八叉树视锥体 SSE LOD 调度器与异步 LRU 显存瓦片缓存池
 *
 * 性能工程设计：
 *   1. 视锥体六平面提取：直接从 Three.js Z-up 透视相机提取 ax + by + cz + d >= 0 六平面方程，
 *      与 C++ 后端 `PointCloudOctree::IsBoxOutsideFrustum` 数学定义 100% 一致。
 *   2. 并发限流与增量切换（防闪烁双缓冲策略）：
 *      - 相机移动时异步向后端请求新的可视八叉树节点列表；
 *      - 仅当新节点集（或足够比例的父/子节点）完成显存上传后才隐藏旧节点，彻底杜绝镜头快速
 *        推拉时画面出现“空洞闪烁”；
 *      - 限制同时在途的二进制 fetch 请求数量（默认最多 8 路并发），避免主线程阻塞掉帧。
 *   3. 零拷贝 InterleavedBuffer 绑定：
 *      - 直接将 16 字节步长的 `ArrayBuffer` 包装为 `THREE.InterleavedBuffer`，分别绑定 `position`
 *        (offset 0, itemSize 3) 与 `aPackedAttr` (offset 3, itemSize 1)，显存零冗余拷贝。
 *   4. Debug 模式详细日志追踪：
 *      - 记录节点调度、并发拉取、GPU 缓冲上传及 LRU 淘汰详情；Release 模式零开销跳过。
 */
import * as THREE from 'three';
import { IGeoKernelBridge, frontendLogger } from '../bridge/kernel_bridge';
import {
  CameraQueryInput,
  DecodedPointCloudPacket,
  GPU_POINT_VERTEX_STRIDE,
  SurfaceSlot,
} from '../types/geo_protocol';

interface CachedOctreeNodeGpu {
  nodeId: number;
  pointCount: number;
  pointsObj: THREE.Points;
  geometry: THREE.BufferGeometry;
  lastUsedFrame: number;
}

export class OctreeLodManager {
  private readonly bridge: IGeoKernelBridge;
  private readonly slot: SurfaceSlot;
  private readonly group: THREE.Group;
  private readonly material: THREE.ShaderMaterial;
  private readonly cache = new Map<number, CachedOctreeNodeGpu>();
  private readonly pendingLoads = new Set<number>();
  private activeNodeIds = new Set<number>();
  private frameCounter = 0;
  private maxCacheNodes = 512;
  private maxConcurrentFetches = 8;
  private pointBudget = 8_000_000;
  private sseThresholdPx = 2.5;
  private isEnabled = true;
  private isBuilt = false;
  private isQueryInFlight = false;
  private lastCameraSignature = '';
  private currentVisiblePoints = 0;

  constructor(
    bridge: IGeoKernelBridge,
    slot: SurfaceSlot,
    parentScene: THREE.Scene,
    material: THREE.ShaderMaterial
  ) {
    this.bridge = bridge;
    this.slot = slot;
    this.material = material;
    this.group = new THREE.Group();
    this.group.name = `PointCloudOctree_Slot_${slot}`;
    parentScene.add(this.group);
    frontendLogger.debug('OCTREE', () => `初始化点云八叉树渲染管理器 (槽位 ${slot})`);
  }

  setBuilt(built: boolean): void {
    this.isBuilt = built;
    this.lastCameraSignature = '';
    frontendLogger.info('OCTREE', () => `槽位 ${this.slot} 八叉树状态更新为: ${built ? '已就绪' : '未构建'}`);
  }

  setVisible(visible: boolean): void {
    this.isEnabled = visible;
    this.group.visible = visible;
    frontendLogger.debug('OCTREE', () => `槽位 ${this.slot} 点云可见性切换为: ${visible}`);
  }

  setPointBudget(budget: number): void {
    this.pointBudget = Math.max(500_000, budget);
    this.lastCameraSignature = '';
    frontendLogger.debug('OCTREE', () => `槽位 ${this.slot} 单帧点云预算调整为: ${this.pointBudget}`);
  }

  getVisiblePointCount(): number {
    return this.isEnabled && this.isBuilt ? this.currentVisiblePoints : 0;
  }

  getCachedTileCount(): number {
    return this.cache.size;
  }

  /**
   * 清空全部显存几何体缓存（重新导入点云时调用）
   */
  clearAll(): void {
    const prevCount = this.cache.size;
    for (const entry of this.cache.values()) {
      this.group.remove(entry.pointsObj);
      entry.geometry.dispose();
    }
    this.cache.clear();
    this.pendingLoads.clear();
    this.activeNodeIds.clear();
    this.currentVisiblePoints = 0;
    this.lastCameraSignature = '';
    this.isBuilt = false;
    frontendLogger.debug('OCTREE', () => `已清理槽位 ${this.slot} 显存缓存节点 (${prevCount} 个)`);
  }

  /**
   * 每帧渲染前调用：检测相机变化并触发异步八叉树 LOD 调度与 LRU 显存回收
   */
  update(camera: THREE.PerspectiveCamera, viewportHeightPx: number): void {
    ++this.frameCounter;
    if (!this.isEnabled || !this.isBuilt) {
      return;
    }
    const sig = this.computeCameraSignature(camera, viewportHeightPx);
    if (sig !== this.lastCameraSignature && !this.isQueryInFlight) {
      this.lastCameraSignature = sig;
      void this.scheduleLodQuery(camera, viewportHeightPx);
    }
    this.evictStaleCacheEntries();
  }

  private computeCameraSignature(camera: THREE.PerspectiveCamera, viewportH: number): string {
    const p = camera.position;
    const q = camera.quaternion;
    return [
      p.x.toFixed(2),
      p.y.toFixed(2),
      p.z.toFixed(2),
      q.x.toFixed(4),
      q.y.toFixed(4),
      q.z.toFixed(4),
      q.w.toFixed(4),
      viewportH.toFixed(0),
      this.pointBudget,
    ].join('|');
  }

  private extractFrustumPlanes(camera: THREE.PerspectiveCamera): number[][] {
    camera.updateMatrixWorld();
    const projView = new THREE.Matrix4().multiplyMatrices(
      camera.projectionMatrix,
      camera.matrixWorldInverse
    );
    const frustum = new THREE.Frustum().setFromProjectionMatrix(projView);
    // Three.js Frustum 平面顺序为: 0=Right, 1=Left, 2=Bottom, 3=Top, 4=Far, 5=Near
    // 调整为与 C++ 端严格一致的 [Left, Right, Bottom, Top, Near, Far] 顺序
    const orderedPlanes = [
      frustum.planes[1],
      frustum.planes[0],
      frustum.planes[2],
      frustum.planes[3],
      frustum.planes[5],
      frustum.planes[4],
    ];
    return orderedPlanes.map((pl) => [
      pl.normal.x,
      pl.normal.y,
      pl.normal.z,
      pl.constant,
    ]);
  }

  private async scheduleLodQuery(
    camera: THREE.PerspectiveCamera,
    viewportHeightPx: number
  ): Promise<void> {
    this.isQueryInFlight = true;
    try {
      const query: CameraQueryInput = {
        cameraLocalPos: [camera.position.x, camera.position.y, camera.position.z],
        frustumPlanes: this.extractFrustumPlanes(camera),
        viewportHeightPx,
        fovYRadians: THREE.MathUtils.degToRad(camera.fov),
        sseThresholdPx: this.sseThresholdPx,
        maxPointBudget: this.pointBudget,
        enableFrustumCulling: true,
      };
      const res = await this.bridge.queryVisibleOctreeNodes(this.slot, query);
      if (!this.isBuilt) return;

      const targetIds = new Set<number>(res.nodeIds);
      this.activeNodeIds = targetIds;

      // 1. 立即激活已在显存缓存中的目标节点，并统计已就绪点数
      let readyPoints = 0;
      const missingIds: number[] = [];
      for (const nid of res.nodeIds) {
        const cached = this.cache.get(nid);
        if (cached) {
          cached.lastUsedFrame = this.frameCounter;
          cached.pointsObj.visible = true;
          readyPoints += cached.pointCount;
        } else if (!this.pendingLoads.has(nid)) {
          missingIds.push(nid);
        }
      }

      // 2. 隐藏不再处于目标集合中的旧缓存节点（仅当新集合已加载大部分时立即隐藏，防止切换闪烁）
      if (missingIds.length === 0 || readyPoints > res.estimatedPoints * 0.5) {
        for (const [nid, entry] of this.cache.entries()) {
          if (!targetIds.has(nid)) {
            entry.pointsObj.visible = false;
          }
        }
      }
      this.currentVisiblePoints = readyPoints;

      frontendLogger.debug(
        'OCTREE',
        () => `视锥LOD查询: 命中节点 ${res.nodeIds.length} 个, 缓存命中点数 ${readyPoints}, 待拉取节点 ${missingIds.length} 个, 预估总点数 ${res.estimatedPoints}`
      );

      // 3. 并发限流拉取缺失的八叉树节点 GPU 二进制数据包
      const availableSlots = Math.max(0, this.maxConcurrentFetches - this.pendingLoads.size);
      const batchToFetch = missingIds.slice(0, availableSlots);
      for (const nid of batchToFetch) {
        this.pendingLoads.add(nid);
        void this.fetchAndUploadNode(nid);
      }
    } catch (err) {
      frontendLogger.error('OCTREE', () => `八叉树可视节点 LOD 调度查询失败 (槽位 ${this.slot})`, err);
    } finally {
      this.isQueryInFlight = false;
    }
  }

  private async fetchAndUploadNode(nodeId: number): Promise<void> {
    try {
      const pkt: DecodedPointCloudPacket = await this.bridge.fetchOctreeNodePacket(
        this.slot,
        nodeId
      );
      if (!this.isBuilt) return;

      const geometry = new THREE.BufferGeometry();
      if (pkt.header.vertexCount > 0) {
        // 16 字节步长（4 个 32 位元素）零拷贝交错缓冲区绑定
        const interleaved = new THREE.InterleavedBuffer(
          pkt.floatView,
          GPU_POINT_VERTEX_STRIDE / 4
        );
        geometry.setAttribute(
          'position',
          new THREE.InterleavedBufferAttribute(interleaved, 3, 0, false)
        );
        geometry.setAttribute(
          'aPackedAttr',
          new THREE.InterleavedBufferAttribute(interleaved, 1, 3, false)
        );
        geometry.boundingBox = new THREE.Box3(
          new THREE.Vector3(...pkt.header.bboxMin),
          new THREE.Vector3(...pkt.header.bboxMax)
        );
        geometry.boundingSphere = new THREE.Sphere();
        geometry.boundingBox.getBoundingSphere(geometry.boundingSphere);
      }

      const pointsObj = new THREE.Points(geometry, this.material);
      pointsObj.frustumCulled = false; // 已由内核八叉树完成精确视锥体裁剪
      pointsObj.visible = this.activeNodeIds.has(nodeId);
      this.group.add(pointsObj);

      this.cache.set(nodeId, {
        nodeId,
        pointCount: pkt.header.vertexCount,
        pointsObj,
        geometry,
        lastUsedFrame: this.frameCounter,
      });

      if (pointsObj.visible) {
        this.currentVisiblePoints += pkt.header.vertexCount;
      }

      // 若仍有未加载的当前可视节点，触发下一轮补齐
      if (this.activeNodeIds.has(nodeId)) {
        this.lastCameraSignature = '';
      }
    } catch (err) {
      frontendLogger.error('OCTREE', () => `加载并上传八叉树节点 #${nodeId} GPU 数据包失败`, err);
    } finally {
      this.pendingLoads.delete(nodeId);
    }
  }

  private evictStaleCacheEntries(): void {
    if (this.cache.size <= this.maxCacheNodes) {
      return;
    }
    // 收集所有非当前激活的缓存节点并按最近使用帧号升序淘汰
    const inactive: CachedOctreeNodeGpu[] = [];
    for (const entry of this.cache.values()) {
      if (!this.activeNodeIds.has(entry.nodeId)) {
        inactive.push(entry);
      }
    }
    inactive.sort((a, b) => a.lastUsedFrame - b.lastUsedFrame);

    let toRemove = this.cache.size - this.maxCacheNodes;
    let removedCount = 0;
    for (const item of inactive) {
      if (toRemove <= 0) break;
      this.group.remove(item.pointsObj);
      item.geometry.dispose();
      this.cache.delete(item.nodeId);
      --toRemove;
      ++removedCount;
    }
    if (removedCount > 0) {
      frontendLogger.debug(
        'OCTREE',
        () => `LRU 显存淘汰完成: 释放 ${removedCount} 个非活跃节点, 当前缓存节点数 ${this.cache.size}`
      );
    }
  }
}
