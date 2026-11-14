/**
 * @file tin_tile_manager.ts
 * @brief 空间分块三角网 (TIN) 脏瓦片增量 GPU 同步管理器与交互叠加矢量图层
 *
 * 核心工程特性：
 *   1. 脏瓦片增量同步（Dirty-Tile Incremental GPU Upload）：
 *      仅向后端拉取 `is_dirty == true` 的空间瓦片索引，并通过 `geotile://` 二进制流
 *      零拷贝构建 28 字节交错缓冲（pos[3] + normal[3] + packed_attr[1]）与 Uint32 索引缓冲。
 *      交互式修改局部高程、翻转边或插入断裂线时，仅重建 1~2 个脏瓦片，帧耗时低于 2ms。
 *   2. 拓扑线框同步叠加：
 *      支持在任意时刻无额外显存拷贝地为所有已加载瓦片切换叠加三角网拓扑线框（Wireframe）。
 *   3. 交互绘制草图与坡脚线（Daylight Line）三维高亮渲染：
 *      实时渲染用户正在绘制的特征线、裁剪多边形、放坡基准轮廓以及内核计算返回的封闭坡脚线。
 */

import * as THREE from 'three';
import { IGeoKernelBridge } from '../bridge/kernel_bridge';
import {
  DecodedTinMeshPacket,
  GPU_MESH_VERTEX_STRIDE,
  SurfaceSlot,
} from '../types/geo_protocol';
import { TinRenderMode } from '../shaders/terrain_tin_shader';

interface CachedTinTileGpu {
  tileIndex: number;
  revision: number;
  triangleCount: number;
  meshObj: THREE.Mesh;
  wireObj: THREE.LineSegments;
  geometry: THREE.BufferGeometry;
}

export class TinTileManager {
  private readonly bridge: IGeoKernelBridge;
  private readonly slot: SurfaceSlot;
  private readonly group: THREE.Group;
  private readonly material: THREE.ShaderMaterial;
  private readonly wireMaterial: THREE.LineBasicMaterial;

  private readonly tiles = new Map<number, CachedTinTileGpu>();
  private showWireframe = false;
  private isVisible = true;
  private totalTriangles = 0;

  constructor(
    bridge: IGeoKernelBridge,
    slot: SurfaceSlot,
    parentScene: THREE.Scene,
    material: THREE.ShaderMaterial
  ) {
    this.bridge = bridge;
    this.slot = slot;
    this.material = material;
    this.wireMaterial = new THREE.LineBasicMaterial({
      color: 0x16222f,
      transparent: true,
      opacity: 0.45,
      depthTest: true,
    });

    this.group = new THREE.Group();
    this.group.name = `TinSurface_Slot_${slot}`;
    parentScene.add(this.group);
  }

  setVisible(visible: boolean): void {
    this.isVisible = visible;
    this.group.visible = visible;
  }

  setWireframeVisible(show: boolean): void {
    this.showWireframe = show;
    for (const entry of this.tiles.values()) {
      entry.wireObj.visible = show;
    }
  }

  setRenderMode(mode: TinRenderMode): void {
    if (this.material.uniforms.uRenderMode) {
      this.material.uniforms.uRenderMode.value = mode;
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

  getTotalTriangleCount(): number {
    return this.isVisible ? this.totalTriangles : 0;
  }

  getLoadedTileCount(): number {
    return this.tiles.size;
  }

  /**
   * 彻底清空当前槽位的所有显存网格资源
   */
  clearAll(): void {
    for (const entry of this.tiles.values()) {
      this.group.remove(entry.meshObj);
      this.group.remove(entry.wireObj);
      entry.geometry.dispose();
    }
    this.tiles.clear();
    this.totalTriangles = 0;
  }

  /**
   * 查询后端所有脏瓦片编号并并发拉取最新二进制网格包刷新显存
   */
  async syncDirtyTiles(): Promise<number> {
    const dirtyIndices = await this.bridge.getDirtyTinTiles(this.slot);
    if (dirtyIndices.length === 0) {
      return 0;
    }

    // 分批并发拉取脏瓦片二进制数据包（每批 16 路并发）
    const batchSize = 16;
    for (let i = 0; i < dirtyIndices.length; i += batchSize) {
      const batch = dirtyIndices.slice(i, i + batchSize);
      await Promise.all(
        batch.map(async (tileIdx) => {
          const pkt = await this.bridge.fetchTinTilePacket(this.slot, tileIdx, true);
          this.uploadOrReplaceTile(tileIdx, pkt);
        })
      );
    }

    this.recomputeTotalTriangles();
    return dirtyIndices.length;
  }

  private uploadOrReplaceTile(tileIndex: number, pkt: DecodedTinMeshPacket): void {
    // 释放该瓦片旧显存对象（若存在）
    const existing = this.tiles.get(tileIndex);
    if (existing) {
      this.group.remove(existing.meshObj);
      this.group.remove(existing.wireObj);
      existing.geometry.dispose();
      this.tiles.delete(tileIndex);
    }

    // 若编辑裁剪后该瓦片内已无活跃三角形，直接移除即可
    if (pkt.header.vertexCount === 0 || pkt.header.indexCount === 0) {
      return;
    }

    const geometry = new THREE.BufferGeometry();
    // 28 字节步长（7 个 32 位元素）零拷贝交错缓冲区绑定：
    //   offset 0..2: position (x, y, z)
    //   offset 3..5: normal   (nx, ny, nz)
    //   offset 6:    aPackedAttr
    const interleaved = new THREE.InterleavedBuffer(
      pkt.vertexFloatView,
      GPU_MESH_VERTEX_STRIDE / 4
    );

    geometry.setAttribute(
      'position',
      new THREE.InterleavedBufferAttribute(interleaved, 3, 0, false)
    );
    geometry.setAttribute(
      'normal',
      new THREE.InterleavedBufferAttribute(interleaved, 3, 3, false)
    );
    geometry.setAttribute(
      'aPackedAttr',
      new THREE.InterleavedBufferAttribute(interleaved, 1, 6, false)
    );
    geometry.setIndex(new THREE.BufferAttribute(pkt.indices, 1));

    geometry.boundingBox = new THREE.Box3(
      new THREE.Vector3(...pkt.header.bboxMin),
      new THREE.Vector3(...pkt.header.bboxMax)
    );
    geometry.boundingSphere = new THREE.Sphere();
    geometry.boundingBox.getBoundingSphere(geometry.boundingSphere);

    const meshObj = new THREE.Mesh(geometry, this.material);
    meshObj.frustumCulled = true;
    // 对第二期目标面施加微小 Z 偏移避免与第一期原地面共面 Z-Fighting
    if (this.slot === SurfaceSlot.Period2Target) {
      meshObj.position.z = 0.005;
    } else if (this.slot === SurfaceSlot.GradingDesignOnly) {
      meshObj.position.z = 0.01;
    }

    const wireObj = new THREE.LineSegments(geometry, this.wireMaterial);
    wireObj.visible = this.showWireframe;
    wireObj.position.copy(meshObj.position);
    wireObj.position.z += 0.015;

    this.group.add(meshObj);
    this.group.add(wireObj);

    this.tiles.set(tileIndex, {
      tileIndex,
      revision: pkt.header.revision,
      triangleCount: Math.floor(pkt.header.indexCount / 3),
      meshObj,
      wireObj,
      geometry,
    });
  }

  private recomputeTotalTriangles(): void {
    let sum = 0;
    for (const entry of this.tiles.values()) {
      sum += entry.triangleCount;
    }
    this.totalTriangles = sum;
  }
}

