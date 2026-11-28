/**
 * @file geo_viewport.ts
 * @brief Z-up 多期测绘三维交互主视口（任意期数动态点云 EDL + 真实拓扑三角网 + 射线拾取）
 *
 * 企业级架构升级要点：
 *   1. 任意期数动态图层池：
 *      - 通过 `userOctreeManagers` 与 `tinManagers` 动态管理任意数量的用户测绘期数（第 1 期、
 *        第 2 期……第 N 期）及系统派生放坡面（槽位 2/3），支持单期独立显隐、不透明度、主题色、
 *        局部居中与彻底销毁释放显存。
 *   2. 智能多期单帧点云预算自动均衡：
 *      - 当同时开启多期点云显示时，自动将全局单帧点云预算均分至各可见八叉树调度器，
 *        确保多期亿级点云叠加浏览依然保持 60FPS 流畅不掉帧。
 *   3. 任意选定期数交互式三维射线拾取：
 *      - 支持动态切换当前“交互拾取与编辑目标期数” (`setActiveInteractionSlot`)，鼠标实时高程读数
 *        与全部 7 项三角网拓扑编辑工具均可无缝作用于任意期数三角网。
 *   4. 双通道混合深度合成与 WebGL 上下文自愈保护：
 *      - 通道 1 渲染全部可见期数点云至离屏深度缓冲并执行 EDL 眼罩光照合成；
 *      - 通道 2 叠加绘制全部可见期数三角网实体、无重复真实拓扑格网线与三维矢量标注。
 */

import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import { IGeoKernelBridge, frontendLogger } from '../bridge/kernel_bridge';
import {
  RayHitResult,
  SlotId,
  SurfaceSlot,
  isSystemDerivedSlot,
} from '../types/geo_protocol';
import {
  createEdlPostProcessMaterial,
  createPointCloudMaterial,
} from '../shaders/point_cloud_edl';
import {
  TinRenderMode,
  createTerrainTinMaterial,
} from '../shaders/terrain_tin_shader';
import { OctreeLodManager } from './octree_lod_manager';
import { TinTileManager } from './tin_tile_manager';

export interface ViewportPickEvent {
  slot: SlotId;
  button: number;
  rayOrigin: [number, number, number];
  rayDir: [number, number, number];
  hit: RayHitResult;
}

export interface ViewportStatsSummary {
  fps: number;
  visiblePoints: number;
  tinTriangles: number;
  cachedTiles: number;
}

export class GeoViewport3D {
  private readonly canvas: HTMLCanvasElement;
  private readonly bridge: IGeoKernelBridge;
  private readonly renderer: THREE.WebGLRenderer;
  private readonly camera: THREE.PerspectiveCamera;
  private readonly controls: OrbitControls;

  // 双通道场景：点云场景（走离屏 EDL 眼罩光照）+ 三角网与矢量标注场景
  private readonly pointCloudScene: THREE.Scene;
  private readonly meshOverlayScene: THREE.Scene;

  // EDL 离屏渲染目标与全屏合成 Quad
  private edlRenderTarget: THREE.WebGLRenderTarget;
  private readonly edlQuadScene: THREE.Scene;
  private readonly edlQuadCamera: THREE.OrthographicCamera;
  private readonly edlMaterial: THREE.ShaderMaterial;

  // 任意期数动态点云八叉树管理器池 & 全量（用户期数 + 系统放坡槽位）三角网管理器池
  private readonly octreeManagers = new Map<SlotId, OctreeLodManager>();
  private readonly tinManagers = new Map<SlotId, TinTileManager>();

  // 全局渲染共享状态（新注册期数自动继承当前用户界面配置）
  private globalMinZ = -20.0;
  private globalMaxZ = 120.0;
  private globalPointBudget = 8_000_000;
  private globalPointTintMix = 0.0;
  private globalTinRenderMode: TinRenderMode = TinRenderMode.ElevationAndContours;
  private globalShowWireframe = false;
  private activeInteractionSlot: SlotId | null = null;

  // 交互矢量叠加组（特征线草图、放坡基准轮廓、封闭坡脚线、实时拾取光标球）
  private readonly vectorOverlayGroup: THREE.Group;
  private draftLineObj: THREE.Line | null = null;
  private daylightLineObj: THREE.Line | null = null;
  private gradingBaseLineObj: THREE.Line | null = null;
  private cursorMarkerObj: THREE.Mesh;

  // 对外回调
  public onCursorTerrainMove?: (
    localPos: [number, number, number] | null,
    slot: SlotId | null
  ) => void;
  public onViewportClickPick?: (event: ViewportPickEvent) => void;

  private animFrameId = 0;
  private lastFpsTime = performance.now();
  private frameCountSinceLastFps = 0;
  private currentFps = 60;
  private pointerDownPos = new THREE.Vector2();
  private lastRayPickTime = 0;
  private lastLoggedMoveErrorTime = 0;
  private currentRtHeight = 900;

  constructor(canvas: HTMLCanvasElement, bridge: IGeoKernelBridge) {
    this.canvas = canvas;
    this.bridge = bridge;
    frontendLogger.info('VIEWPORT', '正在初始化 WebGL2 Z-up 多期三维工程主视口...');

    // 1. 初始化 WebGL2 渲染器（开启抗锯齿、高精深度缓冲与高性能显卡偏好）
    this.renderer = new THREE.WebGLRenderer({
      canvas: this.canvas,
      antialias: true,
      powerPreference: 'high-performance',
    });
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    this.renderer.autoClear = false;
    this.renderer.setClearColor(0x0d131a, 1.0);

    // 2. 初始化测绘工程标准 Z-up 透视相机与阻尼轨道控制器
    this.camera = new THREE.PerspectiveCamera(45, 16 / 9, 0.5, 50000);
    this.camera.up.set(0, 0, 1);
    this.camera.position.set(0, -350, 320);

    this.controls = new OrbitControls(this.camera, this.canvas);
    this.controls.enableDamping = true;
    this.controls.dampingFactor = 0.08;
    this.controls.screenSpacePanning = false;
    this.controls.maxPolarAngle = Math.PI * 0.495;
    this.controls.target.set(0, 0, 0);

    // 3. 初始化双渲染场景
    this.pointCloudScene = new THREE.Scene();
    this.meshOverlayScene = new THREE.Scene();

    // 4. 初始化 EDL 离屏深度渲染目标与全屏后处理平面
    const initW = Math.max(1, this.canvas.clientWidth);
    const initH = Math.max(1, this.canvas.clientHeight);
    this.edlRenderTarget = this.createEdlTarget(initW, initH);
    this.edlMaterial = createEdlPostProcessMaterial();
    this.edlQuadScene = new THREE.Scene();
    this.edlQuadCamera = new THREE.OrthographicCamera(-1, 1, 1, -1, 0, 1);
    const quadGeo = new THREE.PlaneGeometry(2, 2);
    const quadMesh = new THREE.Mesh(quadGeo, this.edlMaterial);
    quadMesh.frustumCulled = false;
    this.edlQuadScene.add(quadMesh);

    // 5. 初始化系统保留的 2 个多级放坡派生三角网槽位（槽位 2 = 放坡设计面，槽位 3 = 缝合后地形）
    this.tinManagers.set(
      SurfaceSlot.GradingDesignOnly,
      new TinTileManager(
        this.bridge,
        SurfaceSlot.GradingDesignOnly,
        this.meshOverlayScene,
        createTerrainTinMaterial({
          minZ: this.globalMinZ,
          maxZ: this.globalMaxZ,
          renderMode: TinRenderMode.GradingRegionClass,
          isGradingSurface: true,
          opacity: 0.94,
        }),
        0x1f2937,
        0.02
      )
    );

    const mergedMgr = new TinTileManager(
      this.bridge,
      SurfaceSlot.GradingMergedTerrain,
      this.meshOverlayScene,
      createTerrainTinMaterial({
        minZ: this.globalMinZ,
        maxZ: this.globalMaxZ,
        renderMode: TinRenderMode.ElevationAndContours,
        isGradingSurface: true,
        opacity: 1.0,
      }),
      0x101c28,
      0.0
    );
    mergedMgr.setVisible(false);
    this.tinManagers.set(SurfaceSlot.GradingMergedTerrain, mergedMgr);

    // 6. 初始化交互矢量叠加组与光标拾取球体
    this.vectorOverlayGroup = new THREE.Group();
    this.vectorOverlayGroup.name = 'InteractiveVectorOverlays';
    this.meshOverlayScene.add(this.vectorOverlayGroup);

    const markerGeo = new THREE.SphereGeometry(0.8, 16, 12);
    const markerMat = new THREE.MeshBasicMaterial({ color: 0xffd43b, depthTest: false });
    this.cursorMarkerObj = new THREE.Mesh(markerGeo, markerMat);
    this.cursorMarkerObj.visible = false;
    this.cursorMarkerObj.renderOrder = 999;
    this.vectorOverlayGroup.add(this.cursorMarkerObj);

    // 7. 绑定视口自适应、WebGL 上下文恢复、右键防护与鼠标射线拾取
    this.handleResize();
    window.addEventListener('resize', this.handleResize);
    this.canvas.addEventListener('contextmenu', this.handleCanvasContextMenu);
    this.canvas.addEventListener('webglcontextlost', this.handleContextLost);
    this.canvas.addEventListener('webglcontextrestored', this.handleContextRestored);
    this.canvas.addEventListener('pointerdown', this.handlePointerDown);
    this.canvas.addEventListener('pointermove', this.handlePointerMove);
    this.canvas.addEventListener('pointerup', this.handlePointerUp);

    frontendLogger.info(
      'VIEWPORT',
      () =>
        `WebGL2 多期主视口初始化完毕 | 尺寸: ${initW}x${initH}, DPR: ${this.renderer.getPixelRatio()}`
    );

    this.startRenderLoop();
  }

  private handleCanvasContextMenu = (e: MouseEvent): void => {
    e.preventDefault();
  };

  private handleContextLost = (e: Event): void => {
    e.preventDefault();
    frontendLogger.error('VIEWPORT', '检测到 WebGL 上下文丢失 (webglcontextlost)！渲染等待恢复');
  };

  private handleContextRestored = (): void => {
    frontendLogger.info('VIEWPORT', 'WebGL 上下文已自动恢复 (webglcontextrestored)');
  };

  private createEdlTarget(width: number, height: number): THREE.WebGLRenderTarget {
    const target = new THREE.WebGLRenderTarget(width, height, {
      minFilter: THREE.NearestFilter,
      magFilter: THREE.NearestFilter,
      format: THREE.RGBAFormat,
      type: THREE.UnsignedByteType,
    });
    target.depthTexture = new THREE.DepthTexture(width, height);
    target.depthTexture.type = THREE.UnsignedIntType;
    return target;
  }

  private handleResize = (): void => {
    const parent = this.canvas.parentElement;
    const width = Math.max(1, parent ? parent.clientWidth : window.innerWidth);
    const height = Math.max(1, parent ? parent.clientHeight : window.innerHeight);

    this.camera.aspect = width / height;
    this.camera.updateProjectionMatrix();
    this.renderer.setSize(width, height, false);

    const dpr = this.renderer.getPixelRatio();
    const rtW = Math.max(1, Math.floor(width * dpr));
    const rtH = Math.max(1, Math.floor(height * dpr));
    this.currentRtHeight = rtH;

    this.edlRenderTarget.dispose();
    this.edlRenderTarget = this.createEdlTarget(rtW, rtH);
    this.edlMaterial.uniforms.uResolution.value.set(rtW, rtH);

    for (const oct of this.octreeManagers.values()) {
      oct.updateViewportHeight(rtH);
    }
  };

  // ============================================================================
  // 1. 任意期数动态渲染图层生命周期管理
  // ============================================================================

  /**
   * 为指定用户期数槽位注册（或更新）独立的点云八叉树管理器与三角网瓦片管理器
   */
  registerPeriodLayer(
    slot: SlotId,
    colorHex: string,
    tinOpacity = 1.0,
    zBias = 0.0
  ): { octree: OctreeLodManager; tin: TinTileManager } {
    let octree = this.octreeManagers.get(slot);
    if (!octree) {
      const pcMat = createPointCloudMaterial({
        minZ: this.globalMinZ,
        maxZ: this.globalMaxZ,
        pointSize: 2.2,
        tintColor: new THREE.Color(colorHex),
        tintMix: this.globalPointTintMix,
      });
      octree = new OctreeLodManager(this.bridge, slot, this.pointCloudScene, pcMat);
      octree.updateViewportHeight(this.currentRtHeight);
      this.octreeManagers.set(slot, octree);
    } else {
      octree.setTintColor(colorHex);
      octree.setTintMix(this.globalPointTintMix);
    }

    let tin = this.tinManagers.get(slot);
    if (!tin) {
      const tinMat = createTerrainTinMaterial({
        minZ: this.globalMinZ,
        maxZ: this.globalMaxZ,
        renderMode: this.globalTinRenderMode,
        tintColor: new THREE.Color(colorHex),
        opacity: tinOpacity,
      });
      tin = new TinTileManager(
        this.bridge,
        slot,
        this.meshOverlayScene,
        tinMat,
        0x101c28,
        zBias
      );
      tin.setWireframeVisible(this.globalShowWireframe);
      this.tinManagers.set(slot, tin);
    } else {
      tin.setTintColor(colorHex);
      tin.setOpacity(tinOpacity);
      tin.setZBias(zBias);
    }

    this.rebalancePointBudgets();
    frontendLogger.info(
      'VIEWPORT',
      () => `已注册期数渲染图层: slot=${slot}, color=${colorHex}, opacity=${tinOpacity.toFixed(2)}`
    );
    return { octree, tin };
  }

  /**
   * 卸载并彻底销毁指定用户期数槽位的全部 GPU 显存资源
   */
  removePeriodLayer(slot: SlotId): void {
    if (isSystemDerivedSlot(slot)) {
      this.tinManagers.get(slot)?.clearAll();
      return;
    }

    const oct = this.octreeManagers.get(slot);
    if (oct) {
      oct.dispose();
      this.octreeManagers.delete(slot);
    }

    const tin = this.tinManagers.get(slot);
    if (tin) {
      tin.dispose();
      this.tinManagers.delete(slot);
    }

    if (this.activeInteractionSlot === slot) {
      this.activeInteractionSlot = null;
      this.cursorMarkerObj.visible = false;
    }

    this.rebalancePointBudgets();
    frontendLogger.info('VIEWPORT', () => `已彻底销毁期数渲染图层: slot=${slot}`);
  }

  /**
   * 清空全部用户期数图层及放坡派生展示结果
   */
  clearAllUserPeriodLayers(): void {
    for (const [slot, oct] of this.octreeManagers.entries()) {
      oct.dispose();
      this.octreeManagers.delete(slot);
    }
    for (const [slot, tin] of this.tinManagers.entries()) {
      if (isSystemDerivedSlot(slot)) {
        tin.clearAll();
      } else {
        tin.dispose();
        this.tinManagers.delete(slot);
      }
    }
    this.activeInteractionSlot = null;
    this.cursorMarkerObj.visible = false;
    this.updateDraftPolyline([], false);
    this.updateGradingOverlayLines([], []);
  }

  getOctreeManager(slot: SlotId): OctreeLodManager | undefined {
    return this.octreeManagers.get(slot);
  }

  getTinManager(slot: SlotId): TinTileManager | undefined {
    return this.tinManagers.get(slot);
  }

  setActiveInteractionSlot(slot: SlotId | null): void {
    this.activeInteractionSlot = slot;
    if (slot === null) {
      this.cursorMarkerObj.visible = false;
    }
    frontendLogger.debug('VIEWPORT', () => `当前交互拾取与编辑目标期数槽位切换为: ${String(slot)}`);
  }

  getActiveInteractionSlot(): SlotId | null {
    return this.activeInteractionSlot;
  }

  // ============================================================================
  // 2. 全局渲染参数、预算均衡与着色控制
  // ============================================================================

  setGlobalPointBudget(totalBudget: number): void {
    this.globalPointBudget = Math.max(500_000, Math.floor(totalBudget));
    this.rebalancePointBudgets();
  }

  /**
   * 在全部可见且已构建的点云八叉树图层间公平分配总点数预算，防止多期叠加超预算掉帧
   */
  rebalancePointBudgets(): void {
    const activeOctrees: OctreeLodManager[] = [];
    for (const oct of this.octreeManagers.values()) {
      if (oct.isLayerVisible() && oct.isOctreeBuilt()) {
        activeOctrees.push(oct);
      }
    }
    const activeCount = Math.max(1, activeOctrees.length);
    const perSlotBudget = Math.max(250_000, Math.floor(this.globalPointBudget / activeCount));
    for (const oct of this.octreeManagers.values()) {
      oct.setPointBudget(perSlotBudget);
    }
  }

  setPointCloudTintMix(mix: number): void {
    this.globalPointTintMix = Math.max(0.0, Math.min(1.0, mix));
    for (const oct of this.octreeManagers.values()) {
      oct.setTintMix(this.globalPointTintMix);
    }
  }

  updateGlobalElevationBounds(minZ: number, maxZ: number): void {
    this.globalMinZ = minZ;
    this.globalMaxZ = maxZ;
    for (const oct of this.octreeManagers.values()) {
      oct.updateElevationRange(minZ, maxZ);
    }
    for (const mgr of this.tinManagers.values()) {
      mgr.updateElevationRange(minZ, maxZ);
    }
    frontendLogger.debug(
      'VIEWPORT',
      () => `全局高程色带范围更新: [${minZ.toFixed(3)}, ${maxZ.toFixed(3)}] m`
    );
  }

  setEdlEnabled(enabled: boolean): void {
    this.edlMaterial.uniforms.uEnableEdl.value = enabled ? 1 : 0;
  }

  setAllUserTinRenderMode(mode: TinRenderMode): void {
    this.globalTinRenderMode = mode;
    for (const [slot, mgr] of this.tinManagers.entries()) {
      if (slot !== SurfaceSlot.GradingDesignOnly) {
        mgr.setRenderMode(mode);
      }
    }
  }

  setAllTinWireframe(show: boolean): void {
    this.globalShowWireframe = show;
    for (const mgr of this.tinManagers.values()) {
      mgr.setWireframeVisible(show);
    }
  }

  /**
   * 将相机平滑居中对齐到指定局部三维包围盒
   */
  focusOnLocalBounds(
    minPt: [number, number, number],
    maxPt: [number, number, number]
  ): void {
    const cx = 0.5 * (minPt[0] + maxPt[0]);
    const cy = 0.5 * (minPt[1] + maxPt[1]);
    const cz = 0.5 * (minPt[2] + maxPt[2]);
    const spanX = Math.max(10, maxPt[0] - minPt[0]);
    const spanY = Math.max(10, maxPt[1] - minPt[1]);
    const diag = Math.hypot(spanX, spanY);

    this.controls.target.set(cx, cy, cz);
    this.camera.position.set(cx, cy - diag * 0.72, cz + diag * 0.65);
    this.camera.near = Math.max(0.2, diag * 0.0005);
    this.camera.far = Math.max(20000, diag * 25.0);
    this.camera.updateProjectionMatrix();
    this.controls.update();

    frontendLogger.info(
      'VIEWPORT',
      () =>
        `相机居中对齐包围盒: center=(${cx.toFixed(2)}, ${cy.toFixed(2)}, ${cz.toFixed(2)}), diag=${diag.toFixed(2)} m`
    );
  }

  /**
   * 更新正在交互绘制的特征线/多边形三维预览线
   */
  updateDraftPolyline(points: Array<[number, number, number]>, closeLoop: boolean): void {
    if (this.draftLineObj) {
      this.vectorOverlayGroup.remove(this.draftLineObj);
      this.draftLineObj.geometry.dispose();
      this.draftLineObj = null;
    }
    if (points.length === 0) return;

    const verts: number[] = [];
    for (const p of points) {
      verts.push(p[0], p[1], p[2] + 0.15);
    }
    if (closeLoop && points.length >= 3) {
      verts.push(points[0][0], points[0][1], points[0][2] + 0.15);
    }

    const geo = new THREE.BufferGeometry();
    geo.setAttribute('position', new THREE.Float32BufferAttribute(verts, 3));
    const mat = new THREE.LineBasicMaterial({ color: 0xffeb3b, depthTest: false });
    this.draftLineObj = new THREE.Line(geo, mat);
    this.draftLineObj.renderOrder = 950;
    this.vectorOverlayGroup.add(this.draftLineObj);
  }

  /**
   * 渲染多级放坡基准设计轮廓与封闭坡脚线（Daylight Line）
   */
  updateGradingOverlayLines(
    basePoly: Array<[number, number, number]>,
    daylightPoly: Array<[number, number, number]>
  ): void {
    if (this.gradingBaseLineObj) {
      this.vectorOverlayGroup.remove(this.gradingBaseLineObj);
      this.gradingBaseLineObj.geometry.dispose();
      this.gradingBaseLineObj = null;
    }
    if (this.daylightLineObj) {
      this.vectorOverlayGroup.remove(this.daylightLineObj);
      this.daylightLineObj.geometry.dispose();
      this.daylightLineObj = null;
    }

    if (basePoly.length >= 3) {
      const verts: number[] = [];
      for (const p of basePoly) verts.push(p[0], p[1], p[2] + 0.12);
      verts.push(basePoly[0][0], basePoly[0][1], basePoly[0][2] + 0.12);
      const geo = new THREE.BufferGeometry();
      geo.setAttribute('position', new THREE.Float32BufferAttribute(verts, 3));
      const mat = new THREE.LineBasicMaterial({ color: 0x38bdf8, depthTest: false });
      this.gradingBaseLineObj = new THREE.Line(geo, mat);
      this.gradingBaseLineObj.renderOrder = 940;
      this.vectorOverlayGroup.add(this.gradingBaseLineObj);
    }

    if (daylightPoly.length >= 2) {
      const verts: number[] = [];
      for (const p of daylightPoly) verts.push(p[0], p[1], p[2] + 0.12);
      const geo = new THREE.BufferGeometry();
      geo.setAttribute('position', new THREE.Float32BufferAttribute(verts, 3));
      const mat = new THREE.LineBasicMaterial({ color: 0xff5252, depthTest: false });
      this.daylightLineObj = new THREE.Line(geo, mat);
      this.daylightLineObj.renderOrder = 945;
      this.vectorOverlayGroup.add(this.daylightLineObj);
    }
  }

  getStatsSummary(): ViewportStatsSummary {
    let totalVisPts = 0;
    let totalTiles = 0;
    for (const oct of this.octreeManagers.values()) {
      totalVisPts += oct.getVisiblePointCount();
      totalTiles += oct.getCachedTileCount();
    }

    let totalTris = 0;
    for (const mgr of this.tinManagers.values()) {
      totalTris += mgr.getTotalTriangleCount();
      totalTiles += mgr.getLoadedTileCount();
    }

    return {
      fps: this.currentFps,
      visiblePoints: totalVisPts,
      tinTriangles: totalTris,
      cachedTiles: totalTiles,
    };
  }

  // ============================================================================
  // 3. 鼠标三维射线计算与任意期数交互拾取
  // ============================================================================

  private computeScreenRay(clientX: number, clientY: number): {
    origin: [number, number, number];
    dir: [number, number, number];
  } {
    const rect = this.canvas.getBoundingClientRect();
    const ndcX = ((clientX - rect.left) / Math.max(1, rect.width)) * 2 - 1;
    const ndcY = -((clientY - rect.top) / Math.max(1, rect.height)) * 2 + 1;
    const raycaster = new THREE.Raycaster();
    raycaster.setFromCamera(new THREE.Vector2(ndcX, ndcY), this.camera);
    const o = raycaster.ray.origin;
    const d = raycaster.ray.direction;
    return {
      origin: [o.x, o.y, o.z],
      dir: [d.x, d.y, d.z],
    };
  }

  private resolvePickTargetSlot(): SlotId | null {
    if (this.activeInteractionSlot !== null) {
      const activeMgr = this.tinManagers.get(this.activeInteractionSlot);
      if (activeMgr && activeMgr.getLoadedTileCount() > 0) {
        return this.activeInteractionSlot;
      }
    }
    // 若当前指定编辑期尚未构网，自动回退至首个可见且已加载的期数三角网
    for (const [slot, mgr] of this.tinManagers.entries()) {
      if (!isSystemDerivedSlot(slot) && mgr.isLayerVisible() && mgr.getLoadedTileCount() > 0) {
        return slot;
      }
    }
    return null;
  }

  private handlePointerDown = (e: PointerEvent): void => {
    this.pointerDownPos.set(e.clientX, e.clientY);
  };

  private handlePointerMove = (e: PointerEvent): void => {
    const now = performance.now();
    if (now - this.lastRayPickTime < 35) return;
    this.lastRayPickTime = now;

    const targetSlot = this.resolvePickTargetSlot();
    if (targetSlot === null) {
      this.cursorMarkerObj.visible = false;
      this.onCursorTerrainMove?.(null, null);
      return;
    }

    const ray = this.computeScreenRay(e.clientX, e.clientY);
    void this.bridge
      .pickTinByRay(targetSlot, ray.origin, ray.dir)
      .then((hit) => {
        if (hit.hit) {
          this.cursorMarkerObj.visible = true;
          this.cursorMarkerObj.position.set(
            hit.hitLocalPos[0],
            hit.hitLocalPos[1],
            hit.hitLocalPos[2]
          );
          const dist = this.camera.position.distanceTo(this.cursorMarkerObj.position);
          const s = Math.max(0.25, dist * 0.006);
          this.cursorMarkerObj.scale.set(s, s, s);
          this.onCursorTerrainMove?.(hit.hitLocalPos, targetSlot);
        } else {
          this.cursorMarkerObj.visible = false;
          this.onCursorTerrainMove?.(null, targetSlot);
        }
      })
      .catch((err) => {
        if (frontendLogger.isEnabled() && now - this.lastLoggedMoveErrorTime > 1000) {
          this.lastLoggedMoveErrorTime = now;
          frontendLogger.warn('PICK', () => `光标实时悬停拾取异常 (槽位 ${targetSlot})`, err);
        }
      });
  };

  private handlePointerUp = (e: PointerEvent): void => {
    const moveDist = Math.hypot(
      e.clientX - this.pointerDownPos.x,
      e.clientY - this.pointerDownPos.y
    );
    if (moveDist > 4.0) return;

    const ray = this.computeScreenRay(e.clientX, e.clientY);
    const targetSlot = this.resolvePickTargetSlot();
    if (targetSlot === null) {
      this.onViewportClickPick?.({
        slot: this.activeInteractionSlot ?? SurfaceSlot.Period1Original,
        button: e.button,
        rayOrigin: ray.origin,
        rayDir: ray.dir,
        hit: {
          hit: false,
          triangleId: 0xffffffff,
          nearestVertexId: 0xffffffff,
          nearestEdgeSlot: -1,
          distance: 0,
          hitLocalPos: [0, 0, 0],
        },
      });
      return;
    }

    void this.bridge
      .pickTinByRay(targetSlot, ray.origin, ray.dir)
      .then((hit) => {
        frontendLogger.debug(
          'PICK',
          () =>
            `点击拾取(slot=${targetSlot}, button=${e.button}): hit=${hit.hit}, tri=${hit.triangleId}, vid=${hit.nearestVertexId}, edge=${hit.nearestEdgeSlot}`
        );
        this.onViewportClickPick?.({
          slot: targetSlot,
          button: e.button,
          rayOrigin: ray.origin,
          rayDir: ray.dir,
          hit,
        });
      })
      .catch((err) => {
        frontendLogger.error(
          'PICK',
          () => `点击射线拾取失败 (slot=${targetSlot}, button=${e.button})`,
          err
        );
      });
  };

  // ============================================================================
  // 4. 60FPS 双通道主渲染循环
  // ============================================================================

  private startRenderLoop(): void {
    const tick = (): void => {
      this.animFrameId = requestAnimationFrame(tick);
      try {
        this.controls.update();
        const dpr = this.renderer.getPixelRatio();
        const viewportH = Math.max(1, this.canvas.clientHeight * dpr);

        let totalVisiblePts = 0;
        for (const oct of this.octreeManagers.values()) {
          oct.update(this.camera, viewportH);
          totalVisiblePts += oct.getVisiblePointCount();
        }

        // 通道 1：渲染全部可见期数点云至离屏目标，再执行全屏 EDL 眼罩光照立体合成
        this.renderer.setRenderTarget(this.edlRenderTarget);
        this.renderer.clear(true, true, true);
        if (totalVisiblePts > 0) {
          this.renderer.render(this.pointCloudScene, this.camera);
        }

        this.renderer.setRenderTarget(null);
        this.renderer.clear(true, true, true);
        this.edlMaterial.uniforms.tColor.value = this.edlRenderTarget.texture;
        this.edlMaterial.uniforms.tDepth.value = this.edlRenderTarget.depthTexture;
        this.edlMaterial.uniforms.uCameraNear.value = this.camera.near;
        this.edlMaterial.uniforms.uCameraFar.value = this.camera.far;
        this.renderer.render(this.edlQuadScene, this.edlQuadCamera);

        // 通道 2：在主帧缓冲上叠加渲染全部可见期数三角网实体、真实拓扑格网线与交互矢量标注
        this.renderer.clearDepth();
        this.renderer.render(this.meshOverlayScene, this.camera);

        ++this.frameCountSinceLastFps;
        const now = performance.now();
        if (now - this.lastFpsTime >= 500) {
          this.currentFps = Math.round(
            (this.frameCountSinceLastFps * 1000) / (now - this.lastFpsTime)
          );
          this.frameCountSinceLastFps = 0;
          this.lastFpsTime = now;
        }
      } catch (err) {
        frontendLogger.error('RENDER', '主渲染循环单帧执行发生未预期异常', err);
      }
    };
    tick();
  }

  dispose(): void {
    frontendLogger.info('VIEWPORT', '正在销毁多期三维主视口并释放 WebGL 显存资源');
    cancelAnimationFrame(this.animFrameId);
    window.removeEventListener('resize', this.handleResize);
    this.canvas.removeEventListener('contextmenu', this.handleCanvasContextMenu);
    this.canvas.removeEventListener('webglcontextlost', this.handleContextLost);
    this.canvas.removeEventListener('webglcontextrestored', this.handleContextRestored);
    this.canvas.removeEventListener('pointerdown', this.handlePointerDown);
    this.canvas.removeEventListener('pointermove', this.handlePointerMove);
    this.canvas.removeEventListener('pointerup', this.handlePointerUp);

    for (const oct of this.octreeManagers.values()) {
      oct.dispose();
    }
    this.octreeManagers.clear();

    for (const mgr of this.tinManagers.values()) {
      mgr.dispose();
    }
    this.tinManagers.clear();

    this.edlRenderTarget.dispose();
    this.renderer.dispose();
  }
}

