/**
 * @file geo_viewport.ts
 * @brief Z-up 测绘三维交互主视口（双通道 EDL 眼罩光照 + 三角网实体渲染 + 轨道控制器 + 射线拾取）
 *
 * 渲染与交互工程规范：
 *   1. 双通道混合渲染：
 *      - 通道 1（点云可见时执行）：将八叉树点云渲染至带高精度深度纹理的离屏 `WebGLRenderTarget`，
 *        随后运行全屏 EDL 眼罩光照着色器合成立体点云至主帧缓冲。
 *      - 通道 2：在主帧缓冲叠加绘制三角网实体、等高线、多级放坡坡体与交互矢量标注。
 *   2. 交互保护：
 *      - 画布层明确拦截 `contextmenu` 阻止任何默认 HTML 右键菜单弹出，同时完整保留右键轨道平移
 *        与右键结束多段线绘制功能。
 *   3. Debug 诊断增强：
 *      - 捕获并记录 WebGL 上下文异常、拾取命令错误（不再静默吞错）及相机/视口变更详情；
 *        Release 模式零开销跳过。
 */
import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import { IGeoKernelBridge, frontendLogger } from '../bridge/kernel_bridge';
import { RayHitResult, SurfaceSlot } from '../types/geo_protocol';
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
  button: number;
  rayOrigin: [number, number, number];
  rayDir: [number, number, number];
  hit: RayHitResult;
}

export class GeoViewport3D {
  private readonly canvas: HTMLCanvasElement;
  private readonly bridge: IGeoKernelBridge;
  private readonly renderer: THREE.WebGLRenderer;
  private readonly camera: THREE.PerspectiveCamera;
  private readonly controls: OrbitControls;
  // 独立点云场景与独立三角网/矢量标注场景（支持双通道 EDL 混合深度合成）
  private readonly pointCloudScene: THREE.Scene;
  private readonly meshOverlayScene: THREE.Scene;
  // EDL 离屏渲染目标与全屏合成 Quad
  private edlRenderTarget: THREE.WebGLRenderTarget;
  private readonly edlQuadScene: THREE.Scene;
  private readonly edlQuadCamera: THREE.OrthographicCamera;
  private readonly edlMaterial: THREE.ShaderMaterial;
  // 材质与图层管理器
  private readonly pointMaterial: THREE.ShaderMaterial;
  private readonly octreeP1: OctreeLodManager;
  private readonly tinManagers = new Map<SurfaceSlot, TinTileManager>();
  // 交互矢量叠加组（特征线草图、放坡轮廓、坡脚线、高亮拾取点）
  private readonly vectorOverlayGroup: THREE.Group;
  private draftLineObj: THREE.Line | null = null;
  private daylightLineObj: THREE.Line | null = null;
  private gradingBaseLineObj: THREE.Line | null = null;
  private cursorMarkerObj: THREE.Mesh;
  // 回调函数
  public onCursorTerrainMove?: (localPos: [number, number, number] | null) => void;
  public onViewportClickPick?: (event: ViewportPickEvent) => void;

  private animFrameId = 0;
  private lastFpsTime = performance.now();
  private frameCountSinceLastFps = 0;
  private currentFps = 60;
  private pointerDownPos = new THREE.Vector2();
  private lastRayPickTime = 0;
  private lastLoggedMoveErrorTime = 0;

  constructor(canvas: HTMLCanvasElement, bridge: IGeoKernelBridge) {
    this.canvas = canvas;
    this.bridge = bridge;

    frontendLogger.info('VIEWPORT', '正在初始化 WebGL2 Z-up 三维工程主视口...');

    // 1. 初始化 WebGL2 渲染器（开启抗锯齿与高精深度缓冲）
    this.renderer = new THREE.WebGLRenderer({
      canvas: this.canvas,
      antialias: true,
      powerPreference: 'high-performance',
    });
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    this.renderer.autoClear = false;
    this.renderer.setClearColor(0x0d131a, 1.0);

    // 2. 初始化测绘工程 Z-up 透视相机与轨道控制器
    this.camera = new THREE.PerspectiveCamera(45, 16 / 9, 0.5, 50000);
    this.camera.up.set(0, 0, 1); // 测绘行业标准：Z 轴朝上（高程方向）
    this.camera.position.set(0, -350, 320);

    this.controls = new OrbitControls(this.camera, this.canvas);
    this.controls.enableDamping = true;
    this.controls.dampingFactor = 0.08;
    this.controls.screenSpacePanning = false;
    this.controls.maxPolarAngle = Math.PI * 0.495; // 防止相机钻入地下穿模
    this.controls.target.set(0, 0, 0);

    // 3. 初始化双场景（点云场景 + 三角网实体叠加场景）
    this.pointCloudScene = new THREE.Scene();
    this.meshOverlayScene = new THREE.Scene();

    // 4. 初始化 EDL 离屏深度渲染目标
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

    // 5. 初始化点云八叉树 LOD 管理器
    this.pointMaterial = createPointCloudMaterial(-20, 120, 2.2);
    this.octreeP1 = new OctreeLodManager(
      this.bridge,
      SurfaceSlot.Period1Original,
      this.pointCloudScene,
      this.pointMaterial
    );

    // 6. 初始化 4 个槽位的分块三角网管理器（各自拥有独立材质实例，支持独立着色与透明度调节）
    this.tinManagers.set(
      SurfaceSlot.Period1Original,
      new TinTileManager(
        this.bridge,
        SurfaceSlot.Period1Original,
        this.meshOverlayScene,
        createTerrainTinMaterial({
          minZ: -20,
          maxZ: 120,
          renderMode: TinRenderMode.ElevationAndContours,
          opacity: 1.0,
        })
      )
    );
    this.tinManagers.set(
      SurfaceSlot.Period2Target,
      new TinTileManager(
        this.bridge,
        SurfaceSlot.Period2Target,
        this.meshOverlayScene,
        createTerrainTinMaterial({
          minZ: -20,
          maxZ: 120,
          renderMode: TinRenderMode.SolidShaded,
          tintColor: new THREE.Color(0.25, 0.56, 0.86),
          opacity: 0.72,
        })
      )
    );
    this.tinManagers.set(
      SurfaceSlot.GradingDesignOnly,
      new TinTileManager(
        this.bridge,
        SurfaceSlot.GradingDesignOnly,
        this.meshOverlayScene,
        createTerrainTinMaterial({
          minZ: -20,
          maxZ: 120,
          renderMode: TinRenderMode.GradingRegionClass,
          isGradingSurface: true,
          opacity: 0.94,
        })
      )
    );
    const mergedMgr = new TinTileManager(
      this.bridge,
      SurfaceSlot.GradingMergedTerrain,
      this.meshOverlayScene,
      createTerrainTinMaterial({
        minZ: -20,
        maxZ: 120,
        renderMode: TinRenderMode.ElevationAndContours,
        isGradingSurface: true,
        opacity: 1.0,
      })
    );
    mergedMgr.setVisible(false);
    this.tinManagers.set(SurfaceSlot.GradingMergedTerrain, mergedMgr);

    // 7. 初始化三维交互矢量标注层与光标拾取球体
    this.vectorOverlayGroup = new THREE.Group();
    this.vectorOverlayGroup.name = 'InteractiveVectorOverlays';
    this.meshOverlayScene.add(this.vectorOverlayGroup);

    const markerGeo = new THREE.SphereGeometry(0.8, 16, 12);
    const markerMat = new THREE.MeshBasicMaterial({ color: 0xffd43b, depthTest: false });
    this.cursorMarkerObj = new THREE.Mesh(markerGeo, markerMat);
    this.cursorMarkerObj.visible = false;
    this.cursorMarkerObj.renderOrder = 999;
    this.vectorOverlayGroup.add(this.cursorMarkerObj);

    // 8. 绑定尺寸自适应、WebGL 上下文保护、画布右键菜单屏蔽与鼠标交互事件
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
      () => `WebGL2 视口初始化完成 | 初始尺寸: ${initW}x${initH}, DPR: ${this.renderer.getPixelRatio()}`
    );

    // 启动 60FPS 主渲染循环
    this.startRenderLoop();
  }

  private handleCanvasContextMenu = (e: MouseEvent): void => {
    // 严格阻止浏览器/WebView 默认右键菜单，确保右键拖拽与右键完成绘制体验纯净
    e.preventDefault();
  };

  private handleContextLost = (e: Event): void => {
    e.preventDefault();
    frontendLogger.error('VIEWPORT', '检测到 WebGL 上下文丢失 (webglcontextlost)！渲染已暂停等待恢复');
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

    this.edlRenderTarget.dispose();
    this.edlRenderTarget = this.createEdlTarget(rtW, rtH);
    this.edlMaterial.uniforms.uResolution.value.set(rtW, rtH);
    this.pointMaterial.uniforms.uViewportHeight.value = rtH;

    frontendLogger.debug('VIEWPORT', () => `视口尺寸自适应更新: CSS=${width}x${height}, RT=${rtW}x${rtH}`);
  };

  // ============================================================================
  // 公开控制与数据同步接口
  // ============================================================================
  getOctreeManager(): OctreeLodManager {
    return this.octreeP1;
  }

  getTinManager(slot: SurfaceSlot): TinTileManager {
    const mgr = this.tinManagers.get(slot);
    if (!mgr) {
      const msg = `未找到槽位 ${slot} 对应的三角网渲染管理器`;
      frontendLogger.error('VIEWPORT', msg);
      throw new Error(msg);
    }
    return mgr;
  }

  updateGlobalElevationBounds(minZ: number, maxZ: number): void {
    this.pointMaterial.uniforms.uMinZ.value = minZ;
    this.pointMaterial.uniforms.uMaxZ.value = maxZ;
    for (const mgr of this.tinManagers.values()) {
      mgr.updateElevationRange(minZ, maxZ);
    }
    frontendLogger.debug('VIEWPORT', () => `全局高程色带范围更新: [${minZ.toFixed(3)}, ${maxZ.toFixed(3)}] m`);
  }

  setEdlEnabled(enabled: boolean): void {
    this.edlMaterial.uniforms.uEnableEdl.value = enabled ? 1 : 0;
    frontendLogger.debug('VIEWPORT', () => `EDL 眼罩光照开关切换为: ${enabled}`);
  }

  setAllTinRenderMode(mode: TinRenderMode): void {
    this.tinManagers.get(SurfaceSlot.Period1Original)?.setRenderMode(mode);
    this.tinManagers.get(SurfaceSlot.GradingMergedTerrain)?.setRenderMode(mode);
  }

  setAllTinWireframe(show: boolean): void {
    for (const mgr of this.tinManagers.values()) {
      mgr.setWireframeVisible(show);
    }
  }

  /**
   * 将相机平滑居中对齐到指定局部三维包围盒
   */
  focusOnLocalBounds(minPt: [number, number, number], maxPt: [number, number, number]): void {
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
      () => `相机已居中对齐至包围盒: center=(${cx.toFixed(2)}, ${cy.toFixed(2)}, ${cz.toFixed(2)}), diag=${diag.toFixed(2)} m`
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

  getStatsSummary(): {
    fps: number;
    visiblePoints: number;
    tinTriangles: number;
    cachedTiles: number;
  } {
    let totalTris = 0;
    let totalTiles = this.octreeP1.getCachedTileCount();
    for (const mgr of this.tinManagers.values()) {
      totalTris += mgr.getTotalTriangleCount();
      totalTiles += mgr.getLoadedTileCount();
    }
    return {
      fps: this.currentFps,
      visiblePoints: this.octreeP1.getVisiblePointCount(),
      tinTriangles: totalTris,
      cachedTiles: totalTiles,
    };
  }

  // ============================================================================
  // 鼠标三维射线计算与交互拾取
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

  private handlePointerDown = (e: PointerEvent): void => {
    this.pointerDownPos.set(e.clientX, e.clientY);
  };

  private handlePointerMove = (e: PointerEvent): void => {
    const now = performance.now();
    // 节流 35ms 执行一次实时光标坐标查询，兼顾流畅度与 IPC 轻量化
    if (now - this.lastRayPickTime < 35) return;
    this.lastRayPickTime = now;

    if (this.tinManagers.get(SurfaceSlot.Period1Original)?.getLoadedTileCount() === 0) {
      return;
    }

    const ray = this.computeScreenRay(e.clientX, e.clientY);
    void this.bridge
      .pickTinByRay(SurfaceSlot.Period1Original, ray.origin, ray.dir)
      .then((hit) => {
        if (hit.hit) {
          this.cursorMarkerObj.visible = true;
          this.cursorMarkerObj.position.set(
            hit.hitLocalPos[0],
            hit.hitLocalPos[1],
            hit.hitLocalPos[2]
          );
          // 随相机距离动态缩放拾取球体大小，保持屏幕视觉尺寸恒定
          const dist = this.camera.position.distanceTo(this.cursorMarkerObj.position);
          const s = Math.max(0.25, dist * 0.006);
          this.cursorMarkerObj.scale.set(s, s, s);
          this.onCursorTerrainMove?.(hit.hitLocalPos);
        } else {
          this.cursorMarkerObj.visible = false;
          this.onCursorTerrainMove?.(null);
        }
      })
      .catch((err) => {
        // 限频记录光标悬停拾取异常（1 秒最多打印一次，防止刷屏同时保证不丢失错误线索）
        if (frontendLogger.isEnabled() && now - this.lastLoggedMoveErrorTime > 1000) {
          this.lastLoggedMoveErrorTime = now;
          frontendLogger.warn('PICK', '光标实时悬停射线拾取异常', err);
        }
      });
  };

  private handlePointerUp = (e: PointerEvent): void => {
    const moveDist = Math.hypot(
      e.clientX - this.pointerDownPos.x,
      e.clientY - this.pointerDownPos.y
    );
    // 仅当鼠标按下与抬起位移 <= 4 像素时判定为有效点击拾取（排除相机拖拽旋转）
    if (moveDist > 4.0) return;

    const ray = this.computeScreenRay(e.clientX, e.clientY);
    if (this.tinManagers.get(SurfaceSlot.Period1Original)?.getLoadedTileCount() === 0) {
      // 未导入三角网时仍支持右键结束等纯按钮事件透传
      this.onViewportClickPick?.({
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
      .pickTinByRay(SurfaceSlot.Period1Original, ray.origin, ray.dir)
      .then((hit) => {
        frontendLogger.debug(
          'PICK',
          () => `点击拾取(button=${e.button}): hit=${hit.hit}, tri=${hit.triangleId}, vid=${hit.nearestVertexId}, edge=${hit.nearestEdgeSlot}`
        );
        this.onViewportClickPick?.({
          button: e.button,
          rayOrigin: ray.origin,
          rayDir: ray.dir,
          hit,
        });
      })
      .catch((err) => {
        frontendLogger.error('PICK', () => `点击射线拾取失败 (button=${e.button})`, err);
      });
  };

  // ============================================================================
  // 60FPS 双通道主渲染循环
  // ============================================================================
  private startRenderLoop(): void {
    const tick = (): void => {
      this.animFrameId = requestAnimationFrame(tick);
      try {
        this.controls.update();

        const dpr = this.renderer.getPixelRatio();
        const viewportH = Math.max(1, this.canvas.clientHeight * dpr);
        this.octreeP1.update(this.camera, viewportH);

        // 通道 1：将点云八叉树渲染到 EDL 离屏目标并执行深度梯度阴影合成
        this.renderer.setRenderTarget(this.edlRenderTarget);
        this.renderer.clear(true, true, true);
        if (this.octreeP1.getVisiblePointCount() > 0) {
          this.renderer.render(this.pointCloudScene, this.camera);
        }

        // 将离屏点云颜色与深度缓冲区输出到屏幕主帧缓冲
        this.renderer.setRenderTarget(null);
        this.renderer.clear(true, true, true);
        this.edlMaterial.uniforms.tColor.value = this.edlRenderTarget.texture;
        this.edlMaterial.uniforms.tDepth.value = this.edlRenderTarget.depthTexture;
        this.edlMaterial.uniforms.uCameraNear.value = this.camera.near;
        this.edlMaterial.uniforms.uCameraFar.value = this.camera.far;
        this.renderer.render(this.edlQuadScene, this.edlQuadCamera);

        // 通道 2：在主帧缓冲上叠加渲染三维三角网实体、等高线、多级放坡坡体与交互矢量线
        this.renderer.clearDepth();
        this.renderer.render(this.meshOverlayScene, this.camera);

        // 统计实时 FPS
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
    frontendLogger.info('VIEWPORT', '正在销毁三维主视口并释放 WebGL 显存资源');
    cancelAnimationFrame(this.animFrameId);
    window.removeEventListener('resize', this.handleResize);
    this.canvas.removeEventListener('contextmenu', this.handleCanvasContextMenu);
    this.canvas.removeEventListener('webglcontextlost', this.handleContextLost);
    this.canvas.removeEventListener('webglcontextrestored', this.handleContextRestored);
    this.canvas.removeEventListener('pointerdown', this.handlePointerDown);
    this.canvas.removeEventListener('pointermove', this.handlePointerMove);
    this.canvas.removeEventListener('pointerup', this.handlePointerUp);
    this.octreeP1.clearAll();
    for (const mgr of this.tinManagers.values()) {
      mgr.clearAll();
    }
    this.edlRenderTarget.dispose();
    this.renderer.dispose();
  }
}
