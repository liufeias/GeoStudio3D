/**
 * @file main.ts
 * @brief GeoStudio 3D 桌面端主应用控制器
 *
 * 完整业务逻辑协调：
 *   1. 初始化解耦通信桥接层（Tauri Native / Web Remote）、调试日志系统与 Z-up 三维主视口；
 *   2. 严格屏蔽应用全局 HTML 默认右键菜单，同时完整保留三维视口右键拖拽与右键完成绘制功能；
 *   3. 捕获并记录全部全局未处理异常与 Promise 拒绝错误至 Debug 日志；
 *   4. 管理第一期原地面与第二期目标面 .dat 点云异步导入、八叉树构建及三角网增量渲染；
 *   5. 驱动六大三角网交互式拓扑编辑工具与 Ctrl+Z / Ctrl+Y 事务级撤销重做；
 *   6. 执行两期精确三棱柱叠加土方算量、快速网格估算及向内/向外多级放坡建模。
 */
import {
  createAutoKernelBridge,
  frontendLogger,
  IGeoKernelBridge,
} from './bridge/kernel_bridge';
import { GeoViewport3D, ViewportPickEvent } from './renderer/geo_viewport';
import { TinRenderMode } from './shaders/terrain_tin_shader';
import {
  DatSummary,
  EarthworkSummary,
  GradingParams,
  SlopeStageSpec,
  SurfaceSlot,
} from './types/geo_protocol';

type EditorToolMode =
  | 'orbit'
  | 'flip-edge'
  | 'edit-z'
  | 'insert-pt'
  | 'breakline'
  | 'clip-poly'
  | 'grading-poly';

class GeoStudioApplication {
  private readonly bridge: IGeoKernelBridge;
  private readonly viewport: GeoViewport3D;
  private activeTool: EditorToolMode = 'orbit';
  private draftVertices: Array<[number, number, number]> = [];
  private customGradingPolygon: Array<[number, number, number]> = [];
  private period1Summary: DatSummary | null = null;
  private globalMinZ = -20;
  private globalMaxZ = 120;

  // 默认三级放坡配置（方便用户一键体验多级放坡与马道建模）
  private gradingStages: SlopeStageSpec[] = [
    { stageHeight: 6.0, slopeRatio: 1.5, bermWidth: 2.0, bermSlopeRatio: -0.02 },
    { stageHeight: 6.0, slopeRatio: 1.75, bermWidth: 2.0, bermSlopeRatio: -0.02 },
    { stageHeight: 10.0, slopeRatio: 2.0, bermWidth: 0.0, bermSlopeRatio: 0.0 },
  ];

  constructor() {
    this.installGlobalProtectionAndErrorHooks();

    const canvas = document.getElementById('gl-canvas') as HTMLCanvasElement;
    this.bridge = createAutoKernelBridge();
    void frontendLogger.initWithBridge(this.bridge);

    this.viewport = new GeoViewport3D(canvas, this.bridge);
    this.bindViewportEvents();
    this.bindUiEvents();
    this.renderGradingStagesList();
    this.startHudMonitor();

    this.setStatus(`内核连接就绪 (${this.bridge.backendName})`);
    frontendLogger.info('APP', () => `GeoStudio 3D 主应用初始化完毕 | 后端: ${this.bridge.backendName}`);
  }

  /**
   * 安装全局防护钩子：
   *   1. 彻底禁用整个应用界面内任意 DOM 元素上的 HTML 默认右键弹出菜单；
   *   2. 在 Debug 模式下捕获全局未处理同步错误与异步 Promise 异常并写入详细日志。
   */
  private installGlobalProtectionAndErrorHooks(): void {
    const blockContextMenu = (event: MouseEvent): void => {
      event.preventDefault();
    };
    window.addEventListener('contextmenu', blockContextMenu, { capture: true });
    document.addEventListener('contextmenu', blockContextMenu, { capture: true });

    window.addEventListener('error', (ev: ErrorEvent) => {
      frontendLogger.error(
        'GLOBAL',
        () => `捕获未处理窗口异常: ${ev.message} (${ev.filename}:${ev.lineno}:${ev.colno})`,
        ev.error
      );
    });

    window.addEventListener('unhandledrejection', (ev: PromiseRejectionEvent) => {
      frontendLogger.error('GLOBAL', '捕获未处理异步 Promise 拒绝异常', ev.reason);
    });
  }

  private setStatus(msg: string): void {
    const el = document.getElementById('hud-status-msg');
    if (el) el.textContent = msg;
  }

  // ============================================================================
  // 1. 视口鼠标拾取与坐标读数绑定
  // ============================================================================
  private bindViewportEvents(): void {
    this.viewport.onCursorTerrainMove = (localPos) => {
      const coordEl = document.getElementById('hud-cursor-coord');
      if (!coordEl) return;
      if (!localPos || !this.period1Summary) {
        coordEl.textContent = '大地坐标: E = ---, N = ---, H = ---';
        return;
      }
      const rtc = this.period1Summary.rtcOrigin;
      const easting = (rtc[0] + localPos[0]).toFixed(3);
      const northing = (rtc[1] + localPos[1]).toFixed(3);
      const elev = (rtc[2] + localPos[2]).toFixed(3);
      coordEl.textContent = `大地坐标: E = ${easting} m, N = ${northing} m, H = ${elev} m`;
    };

    this.viewport.onViewportClickPick = (ev: ViewportPickEvent) => {
      // 右键点击结束多段线/多边形绘制
      if (ev.button === 2 && this.draftVertices.length > 0) {
        frontendLogger.debug('TOOL', () => `右键完成折线/多边形采集 (${this.draftVertices.length} 个顶点)`);
        void this.commitActivePolylineTool();
        return;
      }
      if (ev.button !== 0 || !ev.hit.hit) return;
      void this.handleTerrainPickAction(ev);
    };
  }

  private async handleTerrainPickAction(ev: ViewportPickEvent): Promise<void> {
    const { hit } = ev;
    try {
      switch (this.activeTool) {
        case 'orbit': {
          this.setStatus(
            `拾取三角形 #${hit.triangleId} | 最近顶点 #${hit.nearestVertexId} | 局部高程 Z = ${hit.hitLocalPos[2].toFixed(3)} m`
          );
          break;
        }
        case 'flip-edge': {
          frontendLogger.info(
            'EDIT',
            () => `请求翻转对角线边: tri=#${hit.triangleId}, edgeSlot=${hit.nearestEdgeSlot}`
          );
          await this.bridge.flipTinEdge(
            SurfaceSlot.Period1Original,
            hit.triangleId,
            hit.nearestEdgeSlot,
            false
          );
          const updated = await this.viewport
            .getTinManager(SurfaceSlot.Period1Original)
            .syncDirtyTiles();
          await this.refreshUndoRedoButtons();
          this.setStatus(`对角线翻转完成，已增量刷新 ${updated} 个脏瓦片`);
          break;
        }
        case 'edit-z': {
          const rtcZ = this.period1Summary ? this.period1Summary.rtcOrigin[2] : 0;
          const curGlobalZ = rtcZ + hit.hitLocalPos[2];
          const inputStr = window.prompt(
            `修改顶点 #${hit.nearestVertexId} 的大地高程 H (当前: ${curGlobalZ.toFixed(3)} m):`,
            curGlobalZ.toFixed(3)
          );
          if (inputStr === null) return;
          const newGlobalZ = Number.parseFloat(inputStr);
          if (!Number.isFinite(newGlobalZ)) {
            frontendLogger.warn('EDIT', () => `用户输入非法高程值: "${inputStr}"`);
            this.setStatus('输入高程数值非法，已取消修改');
            return;
          }
          frontendLogger.info(
            'EDIT',
            () => `请求修改顶点 #${hit.nearestVertexId} 大地高程: ${curGlobalZ.toFixed(3)} -> ${newGlobalZ.toFixed(3)} m`
          );
          await this.bridge.modifyVertexElevation(
            SurfaceSlot.Period1Original,
            hit.nearestVertexId,
            newGlobalZ - rtcZ
          );
          await this.viewport.getTinManager(SurfaceSlot.Period1Original).syncDirtyTiles();
          await this.refreshUndoRedoButtons();
          this.setStatus(`顶点 #${hit.nearestVertexId} 高程已更新为 ${newGlobalZ.toFixed(3)} m`);
          break;
        }
        case 'insert-pt': {
          const rtcZ = this.period1Summary ? this.period1Summary.rtcOrigin[2] : 0;
          const defaultGlobalZ = rtcZ + hit.hitLocalPos[2];
          const inputStr = window.prompt(
            `在 (${hit.hitLocalPos[0].toFixed(2)}, ${hit.hitLocalPos[1].toFixed(2)}) 插入新测点，请输入大地高程 H:`,
            defaultGlobalZ.toFixed(3)
          );
          if (inputStr === null) return;
          const newGlobalZ = Number.parseFloat(inputStr);
          if (!Number.isFinite(newGlobalZ)) {
            frontendLogger.warn('EDIT', () => `插点输入非法高程值: "${inputStr}"`);
            return;
          }
          frontendLogger.info(
            'EDIT',
            () => `请求插入新测点: local=(${hit.hitLocalPos[0].toFixed(3)}, ${hit.hitLocalPos[1].toFixed(3)}, ${(newGlobalZ - rtcZ).toFixed(3)})`
          );
          const newVid = await this.bridge.insertVertex(
            SurfaceSlot.Period1Original,
            [hit.hitLocalPos[0], hit.hitLocalPos[1], newGlobalZ - rtcZ],
            100
          );
          await this.viewport.getTinManager(SurfaceSlot.Period1Original).syncDirtyTiles();
          await this.refreshUndoRedoButtons();
          this.setStatus(`已插入新顶点 #${newVid} 并局部恢复 Delaunay 剖分`);
          break;
        }
        case 'breakline':
        case 'clip-poly':
        case 'grading-poly': {
          this.draftVertices.push([...hit.hitLocalPos]);
          const closeLoop = this.activeTool !== 'breakline';
          this.viewport.updateDraftPolyline(this.draftVertices, closeLoop);
          frontendLogger.debug(
            'TOOL',
            () => `工具[${this.activeTool}] 采集节点 #${this.draftVertices.length}: (${hit.hitLocalPos[0].toFixed(2)}, ${hit.hitLocalPos[1].toFixed(2)}, ${hit.hitLocalPos[2].toFixed(2)})`
          );
          this.setStatus(
            `已采集 ${this.draftVertices.length} 个节点（点击“完成并嵌入”或右键结束绘制）`
          );
          break;
        }
      }
    } catch (err) {
      frontendLogger.error('EDIT', () => `地形交互编辑工具 [${this.activeTool}] 执行失败`, err);
      this.setStatus(`编辑操作失败: ${err instanceof Error ? err.message : String(err)}`);
    }
  }

  private async commitActivePolylineTool(): Promise<void> {
    const pts = [...this.draftVertices];
    if (pts.length === 0) return;

    try {
      if (this.activeTool === 'breakline') {
        if (pts.length < 2) {
          this.setStatus('特征折线至少需要 2 个顶点');
          return;
        }
        frontendLogger.info('EDIT', () => `提交强制特征线嵌入 (${pts.length} 个顶点)`);
        await this.bridge.insertBreakline(SurfaceSlot.Period1Original, pts, 201, true);
        await this.viewport.getTinManager(SurfaceSlot.Period1Original).syncDirtyTiles();
        await this.refreshUndoRedoButtons();
        this.setStatus(`强制特征线 (${pts.length} 节点) 已嵌入三角网并锁定约束边`);
      } else if (this.activeTool === 'clip-poly') {
        if (pts.length < 3) {
          this.setStatus('裁剪多边形至少需要 3 个顶点');
          return;
        }
        frontendLogger.info('EDIT', () => `提交多边形外边界裁剪 (${pts.length} 个顶点)`);
        await this.bridge.clipByPolygon(SurfaceSlot.Period1Original, pts, true);
        await this.viewport.getTinManager(SurfaceSlot.Period1Original).syncDirtyTiles();
        await this.refreshUndoRedoButtons();
        this.setStatus(`已按 ${pts.length} 边形边界完成三角网平滑裁剪`);
      } else if (this.activeTool === 'grading-poly') {
        if (pts.length < 3) {
          this.setStatus('放坡基准轮廓至少需要 3 个顶点');
          return;
        }
        this.customGradingPolygon = pts;
        this.viewport.updateGradingOverlayLines(this.customGradingPolygon, []);
        frontendLogger.info('GRADING', () => `已锁定自定义放坡设计轮廓 (${pts.length} 个顶点)`);
        this.setStatus(`已锁定 ${pts.length} 边形放坡设计轮廓，请点击“生成放坡与算量”`);
      }
    } catch (err) {
      frontendLogger.error('TOOL', () => `提交多段线/多边形工具 [${this.activeTool}] 失败`, err);
      this.setStatus(`执行失败: ${err instanceof Error ? err.message : String(err)}`);
    } finally {
      this.draftVertices = [];
      this.viewport.updateDraftPolyline([], false);
      this.switchTool('orbit');
    }
  }

  // ============================================================================
  // 2. 顶部工具栏与三栏面板事件绑定
  // ============================================================================
  private bindUiEvents(): void {
    // 导入第一期原地面 .dat
    document.getElementById('btn-import-p1')?.addEventListener('click', () => {
      void this.handleImportDat(SurfaceSlot.Period1Original);
    });
    // 导入第二期目标面 .dat
    document.getElementById('btn-import-p2')?.addEventListener('click', () => {
      void this.handleImportDat(SurfaceSlot.Period2Target);
    });

    // 撤销 / 重做按钮与键盘快捷键
    document.getElementById('btn-undo')?.addEventListener('click', () => void this.handleUndo());
    document.getElementById('btn-redo')?.addEventListener('click', () => void this.handleRedo());
    window.addEventListener('keydown', (e) => {
      if ((e.ctrlKey || e.metaKey) && e.key.toLowerCase() === 'z') {
        e.preventDefault();
        if (e.shiftKey) void this.handleRedo();
        else void this.handleUndo();
      } else if ((e.ctrlKey || e.metaKey) && e.key.toLowerCase() === 'y') {
        e.preventDefault();
        void this.handleRedo();
      }
    });

    // 全图居中按钮
    document.getElementById('btn-reset-camera')?.addEventListener('click', () => {
      if (this.period1Summary) {
        this.viewport.focusOnLocalBounds(
          this.period1Summary.localMin,
          this.period1Summary.localMax
        );
      }
    });

    // 图层可见性开关
    document.getElementById('chk-layer-octree')?.addEventListener('change', (e) => {
      this.viewport.getOctreeManager().setVisible((e.target as HTMLInputElement).checked);
    });
    document.getElementById('chk-layer-tin1')?.addEventListener('change', (e) => {
      this.viewport
        .getTinManager(SurfaceSlot.Period1Original)
        .setVisible((e.target as HTMLInputElement).checked);
    });
    document.getElementById('chk-layer-tin2')?.addEventListener('change', (e) => {
      this.viewport
        .getTinManager(SurfaceSlot.Period2Target)
        .setVisible((e.target as HTMLInputElement).checked);
    });
    document.getElementById('chk-layer-grading')?.addEventListener('change', (e) => {
      this.viewport
        .getTinManager(SurfaceSlot.GradingDesignOnly)
        .setVisible((e.target as HTMLInputElement).checked);
    });
    document.getElementById('chk-layer-merged')?.addEventListener('change', (e) => {
      this.viewport
        .getTinManager(SurfaceSlot.GradingMergedTerrain)
        .setVisible((e.target as HTMLInputElement).checked);
    });

    // 着色器模式、线框、EDL、点数预算控制
    document.getElementById('sel-tin-shader')?.addEventListener('change', (e) => {
      const mode = Number.parseInt((e.target as HTMLSelectElement).value, 10) as TinRenderMode;
      this.viewport.setAllTinRenderMode(mode);
    });
    document.getElementById('chk-tin-wireframe')?.addEventListener('change', (e) => {
      this.viewport.setAllTinWireframe((e.target as HTMLInputElement).checked);
    });
    document.getElementById('chk-edl-enable')?.addEventListener('change', (e) => {
      this.viewport.setEdlEnabled((e.target as HTMLInputElement).checked);
    });
    document.getElementById('rng-point-budget')?.addEventListener('input', (e) => {
      const budget = Number.parseInt((e.target as HTMLInputElement).value, 10);
      this.viewport.getOctreeManager().setPointBudget(budget);
      const lbl = document.getElementById('lbl-point-budget');
      if (lbl) lbl.textContent = `${Math.round(budget / 10000)} 万点`;
    });

    // 交互编辑工具栏切换
    document.querySelectorAll<HTMLButtonElement>('.tool-btn').forEach((btn) => {
      btn.addEventListener('click', () => {
        const tool = (btn.dataset.tool ?? 'orbit') as EditorToolMode;
        this.switchTool(tool);
      });
    });
    document.getElementById('btn-commit-poly')?.addEventListener('click', () => {
      void this.commitActivePolylineTool();
    });
    document.getElementById('btn-cancel-poly')?.addEventListener('click', () => {
      this.draftVertices = [];
      this.viewport.updateDraftPolyline([], false);
      this.switchTool('orbit');
    });

    // 两期土方算量按钮
    document.getElementById('btn-calc-ew-exact')?.addEventListener('click', () => {
      void this.handleComputeEarthwork(true);
    });
    document.getElementById('btn-calc-ew-grid')?.addEventListener('click', () => {
      void this.handleComputeEarthwork(false);
    });

    // 多级放坡操作按钮
    document.getElementById('btn-add-stage')?.addEventListener('click', () => {
      this.gradingStages.push({
        stageHeight: 6.0,
        slopeRatio: 1.5,
        bermWidth: 1.5,
        bermSlopeRatio: 0.0,
      });
      this.renderGradingStagesList();
      frontendLogger.debug('GRADING', () => `新增放坡级数, 当前共 ${this.gradingStages.length} 级`);
    });
    document.getElementById('btn-draw-grade-poly')?.addEventListener('click', () => {
      this.switchTool('grading-poly');
    });
    document.getElementById('btn-run-grading')?.addEventListener('click', () => {
      void this.handleRunSlopeGrading();
    });
  }

  private switchTool(tool: EditorToolMode): void {
    this.activeTool = tool;
    this.draftVertices = [];
    this.viewport.updateDraftPolyline([], false);
    document.querySelectorAll<HTMLButtonElement>('.tool-btn').forEach((btn) => {
      btn.classList.toggle('active', btn.dataset.tool === tool);
    });
    const polyBar = document.getElementById('poly-edit-actions');
    const isPolyTool = tool === 'breakline' || tool === 'clip-poly' || tool === 'grading-poly';
    if (polyBar) {
      polyBar.style.display = isPolyTool ? 'flex' : 'none';
    }
    const hintMap: Record<EditorToolMode, string> = {
      orbit: '当前模式：自由旋转/平移/缩放视口，左键点击查询三角网精确坐标与高程。',
      'flip-edge': '对角线翻转：左键点击任意三角网边附近，自动检查凸性并翻转共享对角线。',
      'edit-z': '修改点高程：左键点击任意三角网顶点，输入新的大地高程 H 并刷新局部法线。',
      'insert-pt': '插入新测点：左键点击三角形内部，输入高程自动剖分并恢复 Delaunay 性质。',
      breakline: '绘制特征线：依次左键点击坎顶/坎脚/道路边线节点，点击“完成并嵌入”锁定约束边。',
      'clip-poly': '多边形裁剪：依次左键点击测区外边界顶点，完成后自动平滑裁剪多边形外部三角形。',
      'grading-poly': '框选放坡轮廓：依次左键点击场地或基坑轮廓角点，完成后点击“生成放坡与算量”。',
    };
    const hintEl = document.getElementById('lbl-tool-hint');
    if (hintEl) hintEl.textContent = hintMap[tool];
    frontendLogger.debug('TOOL', () => `切换交互编辑模式 -> ${tool}`);
  }

  // ============================================================================
  // 3. 点云导入、八叉树与三角网构建全链路
  // ============================================================================
  private async handleImportDat(slot: SurfaceSlot): Promise<void> {
    const label = slot === SurfaceSlot.Period1Original ? '第一期原地面' : '第二期目标面';
    const filePath = await this.bridge.pickDatFileDialog(`选择${label} .dat 点云文件`);
    if (!filePath) return;

    try {
      frontendLogger.info('IMPORT', () => `开始导入${label}点云: slot=${slot}, path=${filePath}`);
      this.setStatus(`正在并行解析${label}点云文件，请稍候...`);
      const summary = await this.bridge.loadDatFile(slot, filePath, 0);
      frontendLogger.info(
        'IMPORT',
        () => `${label}解析完成: valid=${summary.validPoints}, malformed=${summary.malformedLines}, outliers=${summary.filteredOutliers}, speed=${summary.throughputMptsPerSec.toFixed(2)} Mpts/s`
      );

      if (slot === SurfaceSlot.Period1Original) {
        this.period1Summary = summary;
        this.globalMinZ = summary.localMin[2];
        this.globalMaxZ = summary.localMax[2];

        const rtcEl = document.getElementById('hud-rtc-origin');
        if (rtcEl) {
          rtcEl.textContent = `E=${summary.rtcOrigin[0].toFixed(0)}, N=${summary.rtcOrigin[1].toFixed(0)}, H=${summary.rtcOrigin[2].toFixed(0)}`;
        }

        // 默认将放坡平台高程初始化为测区平均大地高程 + 8m
        const elevInput = document.getElementById('num-grade-elev') as HTMLInputElement | null;
        if (elevInput) {
          const midGlobalZ =
            summary.rtcOrigin[2] + 0.5 * (summary.localMin[2] + summary.localMax[2]) + 8.0;
          elevInput.value = midGlobalZ.toFixed(1);
        }

        // 1. 构建多级 LOD 八叉树
        this.setStatus(`点云解析完成 (${summary.validPoints.toLocaleString()} 点)，正在构建多级八叉树...`);
        const octMgr = this.viewport.getOctreeManager();
        octMgr.clearAll();
        const octSum = await this.bridge.buildOctree(slot);
        frontendLogger.info(
          'IMPORT',
          () => `八叉树构建完成: nodes=${octSum.totalNodes}, leaves=${octSum.leafNodes}, depth=${octSum.maxReachedDepth}, cost=${(octSum.elapsedSeconds * 1000).toFixed(1)} ms`
        );
        octMgr.setBuilt(true);
      } else {
        this.globalMinZ = Math.min(this.globalMinZ, summary.localMin[2]);
        this.globalMaxZ = Math.max(this.globalMaxZ, summary.localMax[2]);
      }

      this.viewport.updateGlobalElevationBounds(this.globalMinZ, this.globalMaxZ);

      // 2. 构建分块邻接 Delaunay 三角网并拉取首屏 GPU 网格瓦片
      const decimateInput = document.getElementById('num-tin-decimate') as HTMLInputElement | null;
      const decimateStep = decimateInput ? Math.max(0, Number.parseFloat(decimateInput.value)) : 0.5;
      this.setStatus(`正在构建${label}分块 Delaunay 三角网...`);
      const tinSum = await this.bridge.buildTin(slot, decimateStep, 128.0);
      frontendLogger.info(
        'IMPORT',
        () => `${label}三角网构建完成: vertices=${tinSum.tinVertices}, triangles=${tinSum.tinTriangles}, tiles=${tinSum.totalTiles}, cost=${(tinSum.elapsedSeconds * 1000).toFixed(1)} ms`
      );

      const tinMgr = this.viewport.getTinManager(slot);
      tinMgr.clearAll();
      await tinMgr.syncDirtyTiles();

      if (slot === SurfaceSlot.Period1Original) {
        this.viewport.focusOnLocalBounds(summary.localMin, summary.localMax);
      }

      await this.refreshUndoRedoButtons();
      this.setStatus(
        `${label}加载成功: ${summary.validPoints.toLocaleString()} 点 (${summary.throughputMptsPerSec.toFixed(1)} Mpts/s) -> ${tinSum.tinTriangles.toLocaleString()} 个三角形`
      );
    } catch (err) {
      frontendLogger.error('IMPORT', () => `导入${label} (.dat) 全链路失败: ${filePath}`, err);
      this.setStatus(`导入${label}失败: ${err instanceof Error ? err.message : String(err)}`);
    }
  }

  private async handleUndo(): Promise<void> {
    try {
      await this.bridge.undoTin(SurfaceSlot.Period1Original);
      await this.viewport.getTinManager(SurfaceSlot.Period1Original).syncDirtyTiles();
      await this.refreshUndoRedoButtons();
      this.setStatus('已撤销最近一次三角网编辑操作');
    } catch (err) {
      frontendLogger.error('HISTORY', '撤销三角网编辑操作失败', err);
      this.setStatus(`撤销失败: ${err instanceof Error ? err.message : String(err)}`);
    }
  }

  private async handleRedo(): Promise<void> {
    try {
      await this.bridge.redoTin(SurfaceSlot.Period1Original);
      await this.viewport.getTinManager(SurfaceSlot.Period1Original).syncDirtyTiles();
      await this.refreshUndoRedoButtons();
      this.setStatus('已重做三角网编辑操作');
    } catch (err) {
      frontendLogger.error('HISTORY', '重做三角网编辑操作失败', err);
      this.setStatus(`重做失败: ${err instanceof Error ? err.message : String(err)}`);
    }
  }

  private async refreshUndoRedoButtons(): Promise<void> {
    const state = await this.bridge.getHistoryState(SurfaceSlot.Period1Original);
    const undoBtn = document.getElementById('btn-undo') as HTMLButtonElement | null;
    const redoBtn = document.getElementById('btn-redo') as HTMLButtonElement | null;
    if (undoBtn) undoBtn.disabled = !state.canUndo;
    if (redoBtn) redoBtn.disabled = !state.canRedo;
  }

  // ============================================================================
  // 4. 两期三角网土方算量
  // ============================================================================
  private async handleComputeEarthwork(exactMode: boolean): Promise<void> {
    try {
      frontendLogger.info('EARTHWORK', () => `启动两期土方算量: exactMode=${exactMode}`);
      this.setStatus(
        exactMode
          ? '正在执行高精度三棱柱布尔叠加土方计算...'
          : '正在执行并行网格微分快速土方计算...'
      );
      const report: EarthworkSummary = exactMode
        ? await this.bridge.computeEarthworkExact(
            SurfaceSlot.Period1Original,
            SurfaceSlot.Period2Target
          )
        : await this.bridge.computeEarthworkGrid(
            SurfaceSlot.Period1Original,
            SurfaceSlot.Period2Target,
            1.0
          );
      this.updateEarthworkUi(report);
      frontendLogger.info(
        'EARTHWORK',
        () => `土方算量成功: cut=${report.cutVolumeM3.toFixed(3)} m³, fill=${report.fillVolumeM3.toFixed(3)} m³, net=${report.netVolumeM3.toFixed(3)} m³, overlap=${report.totalOverlapAreaM2.toFixed(2)} m², prisms=${report.intersectedPrisms}`
      );
      this.setStatus(
        `土方算量完成 (${(report.elapsedSeconds * 1000).toFixed(1)} ms, 积分微元: ${report.intersectedPrisms.toLocaleString()})`
      );
    } catch (err) {
      frontendLogger.error('EARTHWORK', () => `两期土方算量失败 (exactMode=${exactMode})`, err);
      this.setStatus(`土方算量失败: ${err instanceof Error ? err.message : String(err)}`);
    }
  }

  private updateEarthworkUi(rep: EarthworkSummary): void {
    const setTxt = (id: string, val: string) => {
      const el = document.getElementById(id);
      if (el) el.textContent = val;
    };
    setTxt('res-ew-cut', `${rep.cutVolumeM3.toFixed(2)} m³`);
    setTxt('res-ew-fill', `${rep.fillVolumeM3.toFixed(2)} m³`);
    setTxt('res-ew-net', `${rep.netVolumeM3.toFixed(2)} m³`);
    setTxt('res-ew-area', `${rep.totalOverlapAreaM2.toFixed(2)} m²`);
  }

  // ============================================================================
  // 5. 向内/向外多级放坡建模与原地面缝合
  // ============================================================================
  private renderGradingStagesList(): void {
    const container = document.getElementById('grading-stages-list');
    if (!container) return;
    container.innerHTML = '';
    this.gradingStages.forEach((st, idx) => {
      const row = document.createElement('div');
      row.className = 'stage-row';
      row.innerHTML = `
        <div class="stage-row-title">
          <span>第 ${idx + 1} 级边坡</span>
          ${
            this.gradingStages.length > 1
              ? `<button class="btn btn-sm" data-del-stage="${idx}">删除</button>`
              : ''
          }
        </div>
        <div class="stage-inputs">
          <div>
            <label>坡高 (m)</label>
            <input type="number" step="0.5" min="0.5" value="${st.stageHeight}" data-stage-idx="${idx}" data-field="stageHeight" />
          </div>
          <div>
            <label>坡比 1:m</label>
            <input type="number" step="0.1" min="0" value="${st.slopeRatio}" data-stage-idx="${idx}" data-field="slopeRatio" />
          </div>
          <div>
            <label>马道宽 (m)</label>
            <input type="number" step="0.5" min="0" value="${st.bermWidth}" data-stage-idx="${idx}" data-field="bermWidth" />
          </div>
        </div>
      `;
      container.appendChild(row);
    });

    container.querySelectorAll<HTMLInputElement>('input[data-stage-idx]').forEach((inp) => {
      inp.addEventListener('change', () => {
        const idx = Number.parseInt(inp.dataset.stageIdx ?? '0', 10);
        const field = inp.dataset.field as keyof SlopeStageSpec;
        const val = Number.parseFloat(inp.value);
        if (Number.isFinite(val) && this.gradingStages[idx]) {
          this.gradingStages[idx][field] = val;
        }
      });
    });

    container.querySelectorAll<HTMLButtonElement>('button[data-del-stage]').forEach((btn) => {
      btn.addEventListener('click', () => {
        const idx = Number.parseInt(btn.dataset.delStage ?? '0', 10);
        if (this.gradingStages.length > 1) {
          this.gradingStages.splice(idx, 1);
          this.renderGradingStagesList();
        }
      });
    });
  }

  private async handleRunSlopeGrading(): Promise<void> {
    if (!this.period1Summary) {
      this.setStatus('请先导入第一期原地面 .dat 点云后再执行多级放坡');
      return;
    }
    try {
      const dirVal = Number.parseInt(
        (document.getElementById('sel-grade-dir') as HTMLSelectElement).value,
        10
      ) as 0 | 1;
      const vModeVal = Number.parseInt(
        (document.getElementById('sel-grade-vmode') as HTMLSelectElement).value,
        10
      ) as -1 | 0 | 1;
      const globalDesignElev = Number.parseFloat(
        (document.getElementById('num-grade-elev') as HTMLInputElement).value
      );
      const sampleStep = Number.parseFloat(
        (document.getElementById('num-grade-step') as HTMLInputElement).value
      );
      const autoDaylight = (document.getElementById('chk-grade-daylight') as HTMLInputElement)
        .checked;
      const stitchTerrain = (document.getElementById('chk-grade-stitch') as HTMLInputElement)
        .checked;

      const localDesignZ = globalDesignElev - this.period1Summary.rtcOrigin[2];

      // 若用户未手动绘制轮廓，自动在测区中心生成一个默认 60m x 40m 基准设计轮廓
      let basePoly = this.customGradingPolygon;
      if (basePoly.length < 3) {
        const cx = 0.5 * (this.period1Summary.localMin[0] + this.period1Summary.localMax[0]);
        const cy = 0.5 * (this.period1Summary.localMin[1] + this.period1Summary.localMax[1]);
        basePoly = [
          [cx - 30, cy - 20, localDesignZ],
          [cx + 30, cy - 20, localDesignZ],
          [cx + 30, cy + 20, localDesignZ],
          [cx - 30, cy + 20, localDesignZ],
        ];
      } else {
        basePoly = basePoly.map((p) => [p[0], p[1], localDesignZ]);
      }

      const params: GradingParams = {
        direction: dirVal,
        verticalMode: vModeVal,
        useUniformBaseElevation: true,
        baseElevation: localDesignZ,
        autoDaylightToTerrain: autoDaylight,
        capInwardBottom: true,
        perimeterSampleStep: Math.max(0.25, sampleStep),
        outputTileSize: 128.0,
      };

      frontendLogger.info(
        'GRADING',
        () => `启动多级放坡建模: dir=${dirVal}, vMode=${vModeVal}, globalH=${globalDesignElev}, stages=${this.gradingStages.length}, autoDaylight=${autoDaylight}, stitch=${stitchTerrain}`
      );
      this.setStatus('正在执行多级放坡三维建模、原地面求交与土方计算...');

      const res = await this.bridge.computeSlopeGrading(
        SurfaceSlot.Period1Original,
        basePoly,
        this.gradingStages,
        params,
        stitchTerrain
      );

      // 同步拉取生成的放坡设计面 GPU 网格包
      const gradingMgr = this.viewport.getTinManager(SurfaceSlot.GradingDesignOnly);
      gradingMgr.clearAll();
      await gradingMgr.syncDirtyTiles();
      gradingMgr.setVisible(true);

      if (stitchTerrain) {
        const mergedMgr = this.viewport.getTinManager(SurfaceSlot.GradingMergedTerrain);
        mergedMgr.clearAll();
        await mergedMgr.syncDirtyTiles();
      }

      // 高亮显示基准轮廓与提取的封闭坡脚线
      this.viewport.updateGradingOverlayLines(basePoly, res.daylightPolyline);

      // 刷新右侧工程量报告面板
      const setTxt = (id: string, val: string) => {
        const el = document.getElementById(id);
        if (el) el.textContent = val;
      };
      setTxt('res-gr-cut', `${res.earthwork.cutVolumeM3.toFixed(2)} m³`);
      setTxt('res-gr-fill', `${res.earthwork.fillVolumeM3.toFixed(2)} m³`);
      setTxt('res-gr-slope-area', `${res.slopeSurfaceAreaM2.toFixed(2)} m²`);
      setTxt('res-gr-berm-area', `${res.bermSurfaceAreaM2.toFixed(2)} m²`);
      setTxt('res-gr-perim', `${res.daylightPerimeterM.toFixed(2)} m`);
      setTxt('res-gr-conv', res.daylightConverged ? '全封闭相交' : '到达最大级数');

      frontendLogger.info(
        'GRADING',
        () => `放坡建模完成: converged=${res.daylightConverged}, stages=${res.actualStagesUsed}, perimeter=${res.daylightPerimeterM.toFixed(2)} m, fill=${res.earthwork.fillVolumeM3.toFixed(2)} m³, cut=${res.earthwork.cutVolumeM3.toFixed(2)} m³`
      );
      this.setStatus(
        `多级放坡建模完成 (${(res.elapsedSeconds * 1000).toFixed(1)} ms) | 填方: ${res.earthwork.fillVolumeM3.toFixed(1)} m³, 挖方: ${res.earthwork.cutVolumeM3.toFixed(1)} m³`
      );
    } catch (err) {
      frontendLogger.error('GRADING', '多级放坡计算与三维网格生成失败', err);
      this.setStatus(`多级放坡计算失败: ${err instanceof Error ? err.message : String(err)}`);
    }
  }

  private startHudMonitor(): void {
    window.setInterval(() => {
      const s = this.viewport.getStatsSummary();
      const fpsEl = document.getElementById('hud-fps');
      const ptsEl = document.getElementById('hud-vis-pts');
      const trisEl = document.getElementById('hud-tin-tris');
      const tilesEl = document.getElementById('hud-gpu-tiles');
      if (fpsEl) fpsEl.textContent = String(s.fps);
      if (ptsEl) ptsEl.textContent = s.visiblePoints.toLocaleString();
      if (trisEl) trisEl.textContent = s.tinTriangles.toLocaleString();
      if (tilesEl) tilesEl.textContent = String(s.cachedTiles);
    }, 300);
  }
}

window.addEventListener('DOMContentLoaded', () => {
  new GeoStudioApplication();
});
