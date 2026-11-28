/**
 * @file octree_lod_manager.ts
 * @brief 亿级多期点云八叉树视锥体 SSE LOD 调度器与异步 LRU 显存缓存池
 *
 * 性能与企业级健壮性设计：
 *   1. 支持任意期数动态实例化与安全销毁 (`dispose`)：
 *      - 每个导入期数拥有独立的 `OctreeLodManager` 与专属 `ShaderMaterial` 实例，
 *        卸载某期点云时彻底释放显存 `BufferGeometry`、材质及场景组，严防内存/显存泄漏；
 *      - 内置 `disposed` 生命周期栅栏，防止异步拉取的二进制节点数据包在期数销毁后误回写场景。
 *   2. 零拷贝 InterleavedBuffer + 跨平台精确 `Uint16` 属性绑定：
 *      - 16 字节步长（4 个 float32 或 8 个 uint16）直接零拷贝映射至 WebGL2 VBO；
 *      - 通过 `Uint16Array` 交错视图绑定 `aAttrPair`（offset 6, itemSize 2），精准还原
 *        `code_id` 与 `flags`，彻底规避 Windows Direct3D11/ANGLE 浮点次正规数清零缺陷。
 *   3. 视锥体六平面严格对齐 + 防闪烁双缓冲切换：
 *      - 提取 Three.js Z-up 透视相机六平面并排序为 `[Left, Right, Bottom, Top, Near, Far]`，
 *        与 C++ 内核 `PointCloudOctree::IsBoxOutsideFrustum` 数学定义 100% 一致；
 *      - 仅当新可视节点集合完成足够比例加载后才隐藏旧节点，相机快速推拉零空洞闪烁；
 *      - 并发限流（每实例最多 8 路在途 fetch）与按帧号 LRU 淘汰保护显存稳定不爆栈。
 */

import * as THREE from 'three';
import { IGeoKernelBridge, frontendLogger } from '../bridge/kernel_bridge';
import {
  CameraQueryInput,
  DecodedPointCloudPacket,
  GPU_POINT_VERTEX_STRIDE,
  SlotId,
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
  private readonly slot: SlotId;
  private readonly parentScene: THREE.Scene;
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
  private disposed = false;
  private lastCameraSignature = '';
  private currentVisiblePoints = 0;

  constructor(
    bridge: IGeoKernelBridge,
    slot: SlotId,
    parentScene: THREE.Scene,
    material: THREE.ShaderMaterial
  ) {
    this.bridge = bridge;
    this.slot = slot;
    this.parentScene = parentScene;
    this.material = material;

    this.group = new THREE.Group();
    this.group.name = `PointCloudOctree_Slot_${slot}`;
    this.parentScene.add(this.group);

    frontendLogger.debug('OCTREE', () => `初始化点云八叉树渲染管理器 (槽位 ${slot})`);
  }

  getSlot(): SlotId {
    return this.slot;
  }

  setBuilt(built: boolean): void {
    this.isBuilt = built;
    this.lastCameraSignature = '';
    frontendLogger.info(
      'OCTREE',
      () => `槽位 ${this.slot} 八叉树状态更新为: ${built ? '已就绪' : '未构建'}`
    );
  }

  isOctreeBuilt(): boolean {
    return this.isBuilt;
  }

  setVisible(visible: boolean): void {
    this.isEnabled = visible;
    this.group.visible = visible;
    if (visible) {
      this.lastCameraSignature = '';
    }
    frontendLogger.debug('OCTREE', () => `槽位 ${this.slot} 点云可见性设为: ${visible}`);
  }

  isLayerVisible(): boolean {
    return this.isEnabled;
  }

  setPointBudget(budget: number): void {
    const normalized = Math.max(250_000, Math.floor(budget));
    if (this.pointBudget !== normalized) {
      this.pointBudget = normalized;
      this.lastCameraSignature = '';
    }
  }

  setTintColor(colorHex: string | number): void {
    if (this.material.uniforms.uTintColor) {
      (this.material.uniforms.uTintColor.value as THREE.Color).set(colorHex);
    }
  }

  setTintMix(mix: number): void {
    if (this.material.uniforms.uTintMix) {
      this.material.uniforms.uTintMix.value = Math.max(0.0, Math.min(1.0, mix));
    }
  }

  updateElevationRange(minZ: number, maxZ: number): void {
    if (this.material.uniforms.uMinZ) {
      this.material.uniforms.uMinZ.value = minZ;
    }
    if (this.material.uniforms.uMaxZ) {
      this.material.uniforms.uMaxZ.value = maxZ;
    }
  }

  updateViewportHeight(rtHeightPx: number): void {
    if (this.material.uniforms.uViewportHeight) {
      this.material.uniforms.uViewportHeight.value = Math.max(1.0, rtHeightPx);
    }
  }

  getVisiblePointCount(): number {
    return this.isEnabled && this.isBuilt ? this.currentVisiblePoints : 0;
  }

  getCachedTileCount(): number {
    return this.cache.size;
  }

  /**
   * 清空本槽位已上传的全部八叉树节点显存几何体（重新导入同槽位数据时调用）
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
    frontendLogger.debug(
      'OCTREE',
      () => `已清理槽位 ${this.slot} 显存缓存节点 (${prevCount} 个)`
    );
  }

  /**
   * 销毁本槽位八叉树管理器并彻底释放显存几何体、着色器材质与场景节点
   */
  dispose(): void {
    if (this.disposed) return;
    this.disposed = true;
    this.clearAll();
    this.parentScene.remove(this.group);
    this.material.dispose();
    frontendLogger.debug('OCTREE', () => `已彻底销毁槽位 ${this.slot} 八叉树管理器`);
  }

  /**
   * 每帧渲染前调用：检测相机姿态变化并触发异步八叉树视锥体 LOD 调度与 LRU 缓存回收
   */
  update(camera: THREE.PerspectiveCamera, viewportHeightPx: number): void {
    if (this.disposed) return;
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

  private computeCameraSignature(
    camera: THREE.PerspectiveCamera,
    viewportH: number
  ): string {
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
    // 严格按 [Left, Right, Bottom, Top, Near, Far] 顺序与 C++ 内核对齐
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
      if (this.disposed || !this.isBuilt) return;

      const targetIds = new Set<number>(res.nodeIds);
      this.activeNodeIds = targetIds;

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

      // 防闪烁策略：仅当新可视集合已有超过半数点数在显存就绪或无缺失节点时，隐藏旧节点
      if (missingIds.length === 0 || readyPoints >= res.estimatedPoints * 0.5) {
        for (const [nid, entry] of this.cache.entries()) {
          if (!targetIds.has(nid)) {
            entry.pointsObj.visible = false;
          }
        }
      }

      this.currentVisiblePoints = readyPoints;

      const availableSlots = Math.max(0, this.maxConcurrentFetches - this.pendingLoads.size);
      const batchToFetch = missingIds.slice(0, availableSlots);
      for (const nid of batchToFetch) {
        this.pendingLoads.add(nid);
        void this.fetchAndUploadNode(nid);
      }
    } catch (err) {
      if (!this.disposed) {
        frontendLogger.error(
          'OCTREE',
          () => `八叉树可视节点 LOD 调度查询失败 (槽位 ${this.slot})`,
          err
        );
      }
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
      if (this.disposed || !this.isBuilt) return;

      const geometry = new THREE.BufferGeometry();
      if (pkt.header.vertexCount > 0) {
        // 16 字节步长（4 个 float32 或 8 个 uint16）零拷贝双视图交错缓冲绑定
        const floatInterleaved = new THREE.InterleavedBuffer(
          pkt.floatView,
          GPU_POINT_VERTEX_STRIDE / 4
        );
        const attrUint16Interleaved = new THREE.InterleavedBuffer(
          pkt.uint16View,
          GPU_POINT_VERTEX_STRIDE / 2
        );

        geometry.setAttribute(
          'position',
          new THREE.InterleavedBufferAttribute(floatInterleaved, 3, 0, false)
        );
        // 字节偏移 12 = uint16 偏移 6：2 个无符号 16 位整数分别为 code_id 与 flags
        geometry.setAttribute(
          'aAttrPair',
          new THREE.InterleavedBufferAttribute(attrUint16Interleaved, 2, 6, false)
        );

        geometry.boundingBox = new THREE.Box3(
          new THREE.Vector3(...pkt.header.bboxMin),
          new THREE.Vector3(...pkt.header.bboxMax)
        );
        geometry.boundingSphere = new THREE.Sphere();
        geometry.boundingBox.getBoundingSphere(geometry.boundingSphere);
      }

      const pointsObj = new THREE.Points(geometry, this.material);
      pointsObj.frustumCulled = false; // 精确视锥剔除已由内核八叉树完成
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
        // 若仍有待补齐的激活节点，重置相机签名触发下一轮并发补齐
        this.lastCameraSignature = '';
      }
    } catch (err) {
      if (!this.disposed) {
        frontendLogger.error(
          'OCTREE',
          () => `加载并上传八叉树节点 #${nodeId} (槽位 ${this.slot}) 失败`,
          err
        );
      }
    } finally {
      this.pendingLoads.delete(nodeId);
    }
  }

  private evictStaleCacheEntries(): void {
    if (this.cache.size <= this.maxCacheNodes) {
      return;
    }
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
        () =>
          `槽位 ${this.slot} LRU 显存淘汰完成: 释放 ${removedCount} 个非活跃节点, 剩余缓存 ${this.cache.size}`
      );
    }
  }
}

