/**
 * @file main.ts
 * @brief GeoStudio 3D 任意多期测绘点云、三角网拓扑编辑与土方放坡主应用控制器
 *
 * 完整企业级业务逻辑协调：
 *   1. 任意期数土方数据批量/增量导入与管理：
 *      - 支持一次多选或分次添加任意数量的 `.dat` / `.txt` / `.csv` 测绘点云文件，动态申请内核槽位、
 *        自动归算至工程统一全局主 RTC 坐标原点，并分配高区分度主题配色；
 *      - 每期均支持独立开关点云八叉树、独立开关三角网、调节叠加透明度、自定义重命名、
 *        按新抽稀/长边裁剪参数单独重建三角网、相机局部居中以及彻底卸载释放内存与显存。
 *   2. 任意期数交互式三角网拓扑编辑与多步增量 Undo/Redo：
 *      - 支持选择任意已导入期数作为编辑目标，完整驱动 7 大拓扑编辑工具（浏览拾取、对角线翻转、
 *        修改点高程、插入新测点、删除顶点、强制特征线嵌入、多边形裁剪）与 Ctrl+Z / Ctrl+Y 撤销重做。
 *   3. 任意两期精确土方算量与任意基准期多级放坡建模：
 *      - 可从已导入的任意多期数据中自由选择基准期与对比期执行精确三棱柱叠加算量或快速网格估算；
 *      - 可基于任意选定期数作为原地面执行向内/向外多级放坡、坡脚线求交与原地面无缝缝合。
 */

import {
  createAutoKernelBridge,
  frontendLogger,
  IGeoKernelBridge,
} from './bridge/kernel_bridge';
import { GeoViewport3D, ViewportPickEvent } from './renderer/geo_viewport';
import { TinRenderMode } from './shaders/terrain_tin_shader';
import {
  EarthworkSummary,
  GradingParams,
  PERIOD_COLOR_PALETTE,
  PeriodRecord,
  SlopeStageSpec,
  SlotId,
  SurfaceSlot,
} from './types/geo_protocol';

type EditorToolMode =
  | 'orbit'
  | 'flip-edge'
  | 'edit-z'
  | 'insert-pt'
  | 'delete-pt'
  | 'breakline'
  | 'clip-poly'
  | 'grading-poly';

/**
 * 从跨平台文件路径提取短文件名（不含扩展名可选）
 */
function extractShortFileName(filePath: string): string {
  const normalized = filePath.replace(/\\/g, '/');
  const lastSlash = normalized.lastIndexOf('/');
  return lastSlash >= 0 ? normalized.slice(lastSlash + 1) : normalized;
}

function stripFileExtension(fileName: string): string {
  const dot = fileName.lastIndexOf('.');
  return dot > 0 ? fileName.slice(0, dot) : fileName;
}

class GeoStudioApplication {
  private readonly bridge: IGeoKernelBridge;
  private readonly viewport: GeoViewport3D;

  // 任意期数工程数据有序集合（按导入顺序维护）
  private readonly periods: PeriodRecord[] = [];
  private nextPeriodDisplaySeq = 1;
  private masterRtcOrigin: [number, number, number] | null = null;
  private globalMinZ = -20;
  private globalMaxZ = 120;
  private isImportBusy = false;

  // 当前交互编辑、土方对比与放坡基准选中的期数槽位
  private activeEditSlot: SlotId | null = null;
  private selectedEwBaseSlot: SlotId | null = null;
  private selectedEwTargetSlot: SlotId | null = null;
  private selectedGradingBaseSlot: SlotId | null = null;

  private activeTool: EditorToolMode = 'orbit';
  private draftVertices: Array<[number, number, number]> = [];
  private customGradingPolygon: Array<[number, number, number]> = [];

  // 默认三级放坡配置（开箱即用，可动态增删任意级数）
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
    this.renderPeriodLayersList();
    this.syncAllPeriodSelectors();
    this.renderGradingStagesList();
    this.startHudMonitor();

    this.setStatus(`内核连接就绪 (${this.bridge.backendName}) — 支持导入任意多期土方数据`);
    frontendLogger.info(
      'APP',
      () => `GeoStudio 3D 多期主应用初始化完毕 | 后端: ${this.bridge.backendName}`
    );
  }

  /**
   * 安装全局右键防护与未处理异常诊断钩子
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

  private findPeriodBySlot(slot: SlotId | null): PeriodRecord | undefined {
    if (slot === null) return undefined;
    return this.periods.find((p) => p.slot === slot);
  }

  /**
   * 汇总所有已导入期数的联合局部包围盒与高程极值
   */
  private recomputeGlobalBounds(): {
    min: [number, number, number];
    max: [number, number, number];
  } | null {
    if (this.periods.length === 0) {
      this.globalMinZ = -20;
      this.globalMaxZ = 120;
      this.viewport.updateGlobalElevationBounds(this.globalMinZ, this.globalMaxZ);
      return null;
    }

    let minX = Number.POSITIVE_INFINITY;
    let minY = Number.POSITIVE_INFINITY;
    let minZ = Number.POSITIVE_INFINITY;
    let maxX = Number.NEGATIVE_INFINITY;
    let maxY = Number.NEGATIVE_INFINITY;
    let maxZ = Number.NEGATIVE_INFINITY;

    for (const p of this.periods) {
      minX = Math.min(minX, p.datSummary.localMin[0]);
      minY = Math.min(minY, p.datSummary.localMin[1]);
      minZ = Math.min(minZ, p.datSummary.localMin[2]);
      maxX = Math.max(maxX, p.datSummary.localMax[0]);
      maxY = Math.max(maxY, p.datSummary.localMax[1]);
      maxZ = Math.max(maxZ, p.datSummary.localMax[2]);
    }

    this.globalMinZ = minZ;
    this.globalMaxZ = maxZ;
    this.viewport.updateGlobalElevationBounds(minZ, maxZ);

    return {
      min: [minX, minY, minZ],
      max: [maxX, maxY, maxZ],
    };
  }

  // ============================================================================
  // 1. 视口鼠标实时拾取与任意期数拓扑编辑响应
  // ============================================================================

  private bindViewportEvents(): void {
    this.viewport.onCursorTerrainMove = (localPos, slot) => {
      const coordEl = document.getElementById('hud-cursor-coord');
      if (!coordEl) return;
      if (!localPos || !this.masterRtcOrigin) {
        coordEl.textContent = '大地坐标: E = ---, N = ---, H = ---';
        return;
      }
      const period = this.findPeriodBySlot(slot);
      const tag = period ? `[${period.name}] ` : '';
      const easting = (this.masterRtcOrigin[0] + localPos[0]).toFixed(3);
      const northing = (this.masterRtcOrigin[1] + localPos[1]).toFixed(3);
      const elev = (this.masterRtcOrigin[2] + localPos[2]).toFixed(3);
      coordEl.textContent = `${tag}大地坐标: E = ${easting} m, N = ${northing} m, H = ${elev} m`;
    };

    this.viewport.onViewportClickPick = (ev: ViewportPickEvent) => {
      if (ev.button === 2 && this.draftVertices.length > 0) {
        frontendLogger.debug(
          'TOOL',
          () => `右键结束折线/多边形绘制 (${this.draftVertices.length} 个顶点)`
        );
        void this.commitActivePolylineTool();
        return;
      }
      if (ev.button !== 0 || !ev.hit.hit) return;
      void this.handleTerrainPickAction(ev);
    };
  }

  private async handleTerrainPickAction(ev: ViewportPickEvent): Promise<void> {
    const { hit, slot } = ev;
    const period = this.findPeriodBySlot(slot);
    const periodLabel = period ? period.name : `槽位 ${slot}`;
    const rtcZ = this.masterRtcOrigin ? this.masterRtcOrigin[2] : 0;

    try {
      switch (this.activeTool) {
        case 'orbit': {
          const globalH = (rtcZ + hit.hitLocalPos[2]).toFixed(3);
          this.setStatus(
            `[${periodLabel}] 拾取三角形 #${hit.triangleId} | 最近顶点 #${hit.nearestVertexId} | 大地高程 H = ${globalH} m`
          );
          break;
        }
        case 'flip-edge': {
          await this.bridge.flipTinEdge(slot, hit.triangleId, hit.nearestEdgeSlot, false);
          const updated = (await this.viewport.getTinManager(slot)?.syncDirtyTiles()) ?? 0;
          await this.refreshUndoRedoButtons();
          this.setStatus(`[${periodLabel}] 对角线翻转完成，已增量刷新 ${updated} 个脏瓦片`);
          break;
        }
        case 'edit-z': {
          const curGlobalZ = rtcZ + hit.hitLocalPos[2];
          const inputStr = window.prompt(
            `[${periodLabel}] 修改顶点 #${hit.nearestVertexId} 的大地高程 H (当前: ${curGlobalZ.toFixed(3)} m):`,
            curGlobalZ.toFixed(3)
          );
          if (inputStr === null) return;
          const newGlobalZ = Number.parseFloat(inputStr);
          if (!Number.isFinite(newGlobalZ)) {
            this.setStatus('输入的高程数值非法，已取消修改');
            return;
          }
          await this.bridge.modifyVertexElevation(slot, hit.nearestVertexId, newGlobalZ - rtcZ);
          await this.viewport.getTinManager(slot)?.syncDirtyTiles();
          await this.refreshUndoRedoButtons();
          this.setStatus(
            `[${periodLabel}] 顶点 #${hit.nearestVertexId} 高程已更新为 ${newGlobalZ.toFixed(3)} m`
          );
          break;
        }
        case 'insert-pt': {
          const defaultGlobalZ = rtcZ + hit.hitLocalPos[2];
          const inputStr = window.prompt(
            `[${periodLabel}] 在局部坐标 (${hit.hitLocalPos[0].toFixed(2)}, ${hit.hitLocalPos[1].toFixed(2)}) 插入新测点，请输入大地高程 H:`,
            defaultGlobalZ.toFixed(3)
          );
          if (inputStr === null) return;
          const newGlobalZ = Number.parseFloat(inputStr);
          if (!Number.isFinite(newGlobalZ)) {
            this.setStatus('输入的高程数值非法，已取消插点');
            return;
          }
          const newVid = await this.bridge.insertVertex(
            slot,
            [hit.hitLocalPos[0], hit.hitLocalPos[1], newGlobalZ - rtcZ],
            100
          );
          await this.viewport.getTinManager(slot)?.syncDirtyTiles();
          await this.refreshUndoRedoButtons();
          this.setStatus(`[${periodLabel}] 已插入新顶点 #${newVid} 并恢复局部 Delaunay 拓扑`);
          break;
        }
        case 'delete-pt': {
          await this.bridge.deleteVertex(slot, hit.nearestVertexId);
          await this.viewport.getTinManager(slot)?.syncDirtyTiles();
          await this.refreshUndoRedoButtons();
          this.setStatus(`[${periodLabel}] 已删除顶点 #${hit.nearestVertexId} 并完成空腔局部重剖分`);
          break;
        }
        case 'breakline':
        case 'clip-poly':
        case 'grading-poly': {
          this.draftVertices.push([...hit.hitLocalPos]);
          const closeLoop = this.activeTool !== 'breakline';
          this.viewport.updateDraftPolyline(this.draftVertices, closeLoop);
          this.setStatus(
            `已采集 ${this.draftVertices.length} 个节点（点击“完成并嵌入”或右键结束绘制）`
          );
          break;
        }
      }
    } catch (err) {
      frontendLogger.error('EDIT', () => `地形编辑工具 [${this.activeTool}] 执行失败`, err);
      this.setStatus(`编辑操作失败: ${err instanceof Error ? err.message : String(err)}`);
    }
  }

  private async commitActivePolylineTool(): Promise<void> {
    const pts = [...this.draftVertices];
    if (pts.length === 0) return;

    const editSlot = this.activeEditSlot;
    try {
      if (this.activeTool === 'breakline') {
        if (editSlot === null) {
          this.setStatus('请先导入并选择一个编辑目标期数');
          return;
        }
        if (pts.length < 2) {
          this.setStatus('特征折线至少需要 2 个顶点');
          return;
        }
        await this.bridge.insertBreakline(editSlot, pts, 201, true);
        await this.viewport.getTinManager(editSlot)?.syncDirtyTiles();
        await this.refreshUndoRedoButtons();
        this.setStatus(`强制特征线 (${pts.length} 节点) 已嵌入当前编辑期三角网并锁定约束边`);
      } else if (this.activeTool === 'clip-poly') {
        if (editSlot === null) {
          this.setStatus('请先导入并选择一个编辑目标期数');
          return;
        }
        if (pts.length < 3) {
          this.setStatus('裁剪多边形至少需要 3 个顶点');
          return;
        }
        await this.bridge.clipByPolygon(editSlot, pts, true);
        await this.viewport.getTinManager(editSlot)?.syncDirtyTiles();
        await this.refreshUndoRedoButtons();
        this.setStatus(`已按 ${pts.length} 边形边界完成当前编辑期三角网平滑裁剪`);
      } else if (this.activeTool === 'grading-poly') {
        if (pts.length < 3) {
          this.setStatus('放坡设计轮廓至少需要 3 个顶点');
          return;
        }
        this.customGradingPolygon = pts;
        this.viewport.updateGradingOverlayLines(this.customGradingPolygon, []);
        this.setStatus(`已锁定 ${pts.length} 边形放坡设计轮廓，请点击“生成放坡与算量”`);
      }
    } catch (err) {
      frontendLogger.error('TOOL', () => `提交绘制工具 [${this.activeTool}] 失败`, err);
      this.setStatus(`执行失败: ${err instanceof Error ? err.message : String(err)}`);
    } finally {
      this.draftVertices = [];
      this.viewport.updateDraftPolyline([], false);
      this.switchTool('orbit');
    }
  }

  // ============================================================================
  // 2. 界面事件绑定
  // ============================================================================

  private bindUiEvents(): void {
    // 批量/单次导入任意期数点云文件
    document.getElementById('btn-import-periods')?.addEventListener('click', () => {
      void this.handleImportPeriods();
    });
    document.getElementById('btn-quick-add-period')?.addEventListener('click', () => {
      void this.handleImportPeriods();
    });

    // 清空全部期数
    document.getElementById('btn-clear-periods')?.addEventListener('click', () => {
      void this.handleClearAllPeriods();
    });

    // 撤销 / 重做按钮与快捷键
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

    // 全图居中（覆盖全部已导入期数）
    document.getElementById('btn-reset-camera')?.addEventListener('click', () => {
      const bounds = this.recomputeGlobalBounds();
      if (bounds) {
        this.viewport.focusOnLocalBounds(bounds.min, bounds.max);
      }
    });

    // 系统派生放坡图层开关
    document.getElementById('chk-layer-grading')?.addEventListener('change', (e) => {
      this.viewport
        .getTinManager(SurfaceSlot.GradingDesignOnly)
        ?.setVisible((e.target as HTMLInputElement).checked);
    });
    document.getElementById('chk-layer-merged')?.addEventListener('change', (e) => {
      this.viewport
        .getTinManager(SurfaceSlot.GradingMergedTerrain)
        ?.setVisible((e.target as HTMLInputElement).checked);
    });

    // 着色模式、真实拓扑格网线、EDL、多期点云主题色与总点数预算
    document.getElementById('sel-tin-shader')?.addEventListener('change', (e) => {
      const mode = Number.parseInt((e.target as HTMLSelectElement).value, 10) as TinRenderMode;
      this.viewport.setAllUserTinRenderMode(mode);
    });
    document.getElementById('chk-tin-wireframe')?.addEventListener('change', (e) => {
      this.viewport.setAllTinWireframe((e.target as HTMLInputElement).checked);
    });
    document.getElementById('chk-edl-enable')?.addEventListener('change', (e) => {
      this.viewport.setEdlEnabled((e.target as HTMLInputElement).checked);
    });
    document.getElementById('chk-pc-period-tint')?.addEventListener('change', (e) => {
      const useTint = (e.target as HTMLInputElement).checked;
      this.viewport.setPointCloudTintMix(useTint ? 0.85 : 0.0);
    });
    document.getElementById('rng-point-budget')?.addEventListener('input', (e) => {
      const budget = Number.parseInt((e.target as HTMLInputElement).value, 10);
      this.viewport.setGlobalPointBudget(budget);
      const lbl = document.getElementById('lbl-point-budget');
      if (lbl) lbl.textContent = `${Math.round(budget / 10000)} 万点`;
    });

    // 按新抽稀步长/外围长边裁剪参数重建当前编辑期三角网
    document.getElementById('btn-rebuild-active-tin')?.addEventListener('click', () => {
      if (this.activeEditSlot !== null) {
        void this.handleRebuildPeriodTin(this.activeEditSlot);
      } else {
        this.setStatus('请先导入至少一期土方数据后再重建三角网');
      }
    });

    // 编辑目标期数选择器
    document.getElementById('sel-edit-period')?.addEventListener('change', (e) => {
      const val = Number.parseInt((e.target as HTMLSelectElement).value, 10);
      if (Number.isInteger(val)) {
        this.setActiveEditSlot(val);
      }
    });

    // 土方算量两期选择器与互换按钮
    document.getElementById('sel-ew-base-period')?.addEventListener('change', (e) => {
      const val = Number.parseInt((e.target as HTMLSelectElement).value, 10);
      this.selectedEwBaseSlot = Number.isInteger(val) ? val : null;
    });
    document.getElementById('sel-ew-target-period')?.addEventListener('change', (e) => {
      const val = Number.parseInt((e.target as HTMLSelectElement).value, 10);
      this.selectedEwTargetSlot = Number.isInteger(val) ? val : null;
    });
    document.getElementById('btn-swap-ew-periods')?.addEventListener('click', () => {
      const tmp = this.selectedEwBaseSlot;
      this.selectedEwBaseSlot = this.selectedEwTargetSlot;
      this.selectedEwTargetSlot = tmp;
      this.syncAllPeriodSelectors();
    });

    // 放坡基准原地面选择器
    document.getElementById('sel-grading-base-period')?.addEventListener('change', (e) => {
      const val = Number.parseInt((e.target as HTMLSelectElement).value, 10);
      this.selectedGradingBaseSlot = Number.isInteger(val) ? val : null;
    });

    // 交互编辑工具切换
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

    // 任意两期土方算量按钮
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
    const isPolyTool =
      tool === 'breakline' || tool === 'clip-poly' || tool === 'grading-poly';
    if (polyBar) {
      polyBar.style.display = isPolyTool ? 'flex' : 'none';
    }

    const hintMap: Record<EditorToolMode, string> = {
      orbit: '当前模式：自由旋转/平移/缩放视口，左键点击查询当前目标期三角网精确坐标与高程。',
      'flip-edge': '对角线翻转：左键点击任意三角网边附近，自动检查凸性并翻转共享对角线。',
      'edit-z': '修改点高程：左键点击任意三角网顶点，输入新的大地高程 H 并刷新局部法线。',
      'insert-pt': '插入新测点：左键点击三角形内部，输入高程自动剖分并恢复局部 Delaunay 性质。',
      'delete-pt': '删除顶点：左键点击待删顶点，自动提取星形空腔并局部重剖分三角网。',
      breakline: '绘制特征线：依次左键点击坎顶/坎脚/道路边线节点，点击“完成并嵌入”锁定约束边。',
      'clip-poly': '多边形裁剪：依次左键点击测区外边界顶点，完成后自动平滑裁剪多边形外部三角形。',
      'grading-poly': '框选放坡轮廓：依次左键点击场地或基坑轮廓角点，完成后点击“生成放坡与算量”。',
    };
    const hintEl = document.getElementById('lbl-tool-hint');
    if (hintEl) hintEl.textContent = hintMap[tool];
  }

  private setActiveEditSlot(slot: SlotId | null): void {
    this.activeEditSlot = slot;
    this.viewport.setActiveInteractionSlot(slot);
    this.renderPeriodLayersList();
    this.syncAllPeriodSelectors();
    void this.refreshUndoRedoButtons();
  }

  private readTinBuildParamsFromUi(): { decimationStep: number; maxEdgeLength: number } {
    const decInput = document.getElementById('num-tin-decimate') as HTMLInputElement | null;
    const edgeInput = document.getElementById('num-tin-max-edge') as HTMLInputElement | null;
    const decRaw = decInput ? Number.parseFloat(decInput.value) : 0.5;
    const edgeRaw = edgeInput ? Number.parseFloat(edgeInput.value) : 0.0;
    return {
      decimationStep: Number.isFinite(decRaw) && decRaw > 0 ? decRaw : 0.0,
      maxEdgeLength: Number.isFinite(edgeRaw) && edgeRaw > 0 ? edgeRaw : 0.0,
    };
  }

  // ============================================================================
  // 3. 任意期数土方数据批量/增量导入、重建与卸载管理
  // ============================================================================

  private async handleImportPeriods(): Promise<void> {
    if (this.isImportBusy) {
      this.setStatus('当前正在导入点云数据，请稍候完成...');
      return;
    }

    const filePaths = await this.bridge.pickDatFilesDialog(
      '选择一个或多个测绘点云文件（每文件自动建立独立土方期数）',
      true
    );
    if (filePaths.length === 0) return;

    this.isImportBusy = true;
    const { decimationStep, maxEdgeLength } = this.readTinBuildParamsFromUi();
    let successCount = 0;
    const hadAnyBefore = this.periods.length > 0;

    try {
      for (let i = 0; i < filePaths.length; ++i) {
        const filePath = filePaths[i];
        const fileName = extractShortFileName(filePath);
        const periodIdx = this.nextPeriodDisplaySeq++;
        const defaultName = `第${periodIdx}期 · ${stripFileExtension(fileName)}`;
        const colorHex = PERIOD_COLOR_PALETTE[(periodIdx - 1) % PERIOD_COLOR_PALETTE.length];
        const progressTag =
          filePaths.length > 1 ? `[${i + 1}/${filePaths.length}] ` : '';

        let allocatedSlot: SlotId | null = null;
        try {
          allocatedSlot = await this.bridge.createPeriodSlot();
          this.setStatus(`${progressTag}正在多线程解析 ${defaultName} (${fileName})...`);

          const datSummary = await this.bridge.loadDatFile(allocatedSlot, filePath, 0);

          // 同步全局统一主 RTC 坐标原点（由首个导入期数锁定，后续期数由内核自动精确归算对齐）
          if (!this.masterRtcOrigin) {
            this.masterRtcOrigin = [...datSummary.rtcOrigin];
            const rtcEl = document.getElementById('hud-rtc-origin');
            if (rtcEl) {
              rtcEl.textContent = `E=${datSummary.rtcOrigin[0].toFixed(0)}, N=${datSummary.rtcOrigin[1].toFixed(0)}, H=${datSummary.rtcOrigin[2].toFixed(0)}`;
            }
            const elevInput = document.getElementById('num-grade-elev') as HTMLInputElement | null;
            if (elevInput) {
              const midGlobalZ =
                datSummary.rtcOrigin[2] +
                0.5 * (datSummary.localMin[2] + datSummary.localMax[2]) +
                8.0;
              elevInput.value = midGlobalZ.toFixed(1);
            }
          }

          // 多期叠加默认不透明度与微 Z 偏置：第 1 期实心 1.0，后续新增期默认 0.82 便于透视比对
          const isFirst = this.periods.length === 0;
          const initialOpacity = isFirst ? 1.0 : 0.82;
          const zBias = Math.min(0.018, this.periods.length * 0.003);

          const { octree, tin } = this.viewport.registerPeriodLayer(
            allocatedSlot,
            colorHex,
            initialOpacity,
            zBias
          );

          // 构建多级 LOD 八叉树
          this.setStatus(
            `${progressTag}${defaultName} 解析完毕 (${datSummary.validPoints.toLocaleString()} 点)，正在构建八叉树...`
          );
          const octreeSummary = await this.bridge.buildOctree(allocatedSlot);
          octree.clearAll();
          octree.setBuilt(true);
          // 超过 1 期时默认仅显示新增期的三角网，点云可按需在左侧一键勾选，保持画面整洁与高帧率
          const initialShowPc = isFirst;
          octree.setVisible(initialShowPc);

          // 构建空间分块 Delaunay 三角网并拉取 GPU 瓦片
          this.setStatus(`${progressTag}正在构建 ${defaultName} 分块 Delaunay 三角网...`);
          const tinSummary = await this.bridge.buildTin(
            allocatedSlot,
            decimationStep,
            128.0,
            maxEdgeLength
          );
          tin.clearAll();
          await tin.syncDirtyTiles();
          tin.setVisible(true);

          const record: PeriodRecord = {
            slot: allocatedSlot,
            periodIndex: periodIdx,
            name: defaultName,
            filePath,
            fileName,
            colorHex,
            showPointCloud: initialShowPc,
            showTin: true,
            tinOpacity: initialOpacity,
            decimationStep,
            maxEdgeLength,
            datSummary,
            octreeSummary,
            tinSummary,
          };
          this.periods.push(record);
          ++successCount;

          // 智能初始化编辑目标、土方两期选择器与放坡原地面选择器
          if (this.activeEditSlot === null) {
            this.activeEditSlot = allocatedSlot;
            this.viewport.setActiveInteractionSlot(allocatedSlot);
          }
          if (this.selectedEwBaseSlot === null) {
            this.selectedEwBaseSlot = allocatedSlot;
          } else if (
            this.selectedEwTargetSlot === null ||
            this.selectedEwTargetSlot === this.selectedEwBaseSlot
          ) {
            this.selectedEwTargetSlot = allocatedSlot;
          }
          if (this.selectedGradingBaseSlot === null) {
            this.selectedGradingBaseSlot = allocatedSlot;
          }

          this.recomputeGlobalBounds();
          this.viewport.rebalancePointBudgets();
          this.renderPeriodLayersList();
          this.syncAllPeriodSelectors();

          frontendLogger.info(
            'IMPORT',
            () =>
              `期数 ${defaultName} (slot=${allocatedSlot}) 导入完成: ${datSummary.validPoints} 点, ${tinSummary.tinTriangles} 三角形`
          );
        } catch (err) {
          // 单文件导入失败时自动回滚已申请的后端槽位与显存图层，避免残留空期数
          if (allocatedSlot !== null) {
            this.viewport.removePeriodLayer(allocatedSlot);
            await this.bridge.removePeriodSlot(allocatedSlot).catch(() => undefined);
          }
          frontendLogger.error('IMPORT', () => `导入文件 ${fileName} 失败`, err);
          this.setStatus(
            `导入 ${fileName} 失败: ${err instanceof Error ? err.message : String(err)}`
          );
        }
      }

      if (successCount > 0) {
        const unionBounds = this.recomputeGlobalBounds();
        if (!hadAnyBefore && unionBounds) {
          this.viewport.focusOnLocalBounds(unionBounds.min, unionBounds.max);
        }
        await this.refreshUndoRedoButtons();
        this.setStatus(
          `成功导入 ${successCount} 期土方数据（当前工程共 ${this.periods.length} 期，已自动统一坐标系）`
        );
      }
    } finally {
      this.isImportBusy = false;
    }
  }

  private async handleRebuildPeriodTin(slot: SlotId): Promise<void> {
    const period = this.findPeriodBySlot(slot);
    if (!period) return;

    const { decimationStep, maxEdgeLength } = this.readTinBuildParamsFromUi();
    try {
      this.setStatus(
        `正在按 抽稀=${decimationStep}m / 长边裁剪=${maxEdgeLength}m 重建 [${period.name}] 三角网...`
      );
      const tinSummary = await this.bridge.buildTin(slot, decimationStep, 128.0, maxEdgeLength);
      period.decimationStep = decimationStep;
      period.maxEdgeLength = maxEdgeLength;
      period.tinSummary = tinSummary;

      const tinMgr = this.viewport.getTinManager(slot);
      if (tinMgr) {
        tinMgr.clearAll();
        await tinMgr.syncDirtyTiles();
      }

      this.renderPeriodLayersList();
      await this.refreshUndoRedoButtons();
      this.setStatus(
        `[${period.name}] 三角网重建完成: ${tinSummary.tinVertices.toLocaleString()} 顶点, ${tinSummary.tinTriangles.toLocaleString()} 三角形`
      );
    } catch (err) {
      frontendLogger.error('TIN', () => `重建期数槽位 ${slot} 三角网失败`, err);
      this.setStatus(`重建三角网失败: ${err instanceof Error ? err.message : String(err)}`);
    }
  }

  private async handleRemovePeriod(slot: SlotId): Promise<void> {
    const idx = this.periods.findIndex((p) => p.slot === slot);
    if (idx < 0) return;

    const removed = this.periods[idx];
    try {
      await this.bridge.removePeriodSlot(slot);
      this.viewport.removePeriodLayer(slot);
      this.periods.splice(idx, 1);

      if (this.periods.length === 0) {
        this.masterRtcOrigin = null;
        this.nextPeriodDisplaySeq = 1;
        this.activeEditSlot = null;
        this.selectedEwBaseSlot = null;
        this.selectedEwTargetSlot = null;
        this.selectedGradingBaseSlot = null;
        this.customGradingPolygon = [];
        this.viewport.updateGradingOverlayLines([], []);
        const rtcEl = document.getElementById('hud-rtc-origin');
        if (rtcEl) rtcEl.textContent = '未加载';
      } else {
        if (this.activeEditSlot === slot) {
          this.activeEditSlot = this.periods[0].slot;
          this.viewport.setActiveInteractionSlot(this.activeEditSlot);
        }
        if (this.selectedEwBaseSlot === slot) {
          this.selectedEwBaseSlot = this.periods[0].slot;
        }
        if (this.selectedEwTargetSlot === slot) {
          this.selectedEwTargetSlot =
            this.periods.length > 1 ? this.periods[1].slot : this.periods[0].slot;
        }
        if (this.selectedGradingBaseSlot === slot) {
          this.selectedGradingBaseSlot = this.periods[0].slot;
        }
      }

      this.recomputeGlobalBounds();
      this.viewport.rebalancePointBudgets();
      this.renderPeriodLayersList();
      this.syncAllPeriodSelectors();
      await this.refreshUndoRedoButtons();
      this.setStatus(`已卸载期数 [${removed.name}] 并释放全部内核与显存资源`);
    } catch (err) {
      frontendLogger.error('PERIOD', () => `卸载期数槽位 ${slot} 失败`, err);
      this.setStatus(`卸载期数失败: ${err instanceof Error ? err.message : String(err)}`);
    }
  }

  private async handleClearAllPeriods(): Promise<void> {
    if (this.periods.length === 0) {
      this.setStatus('当前工程中暂无可清空的土方期数');
      return;
    }
    const ok = window.confirm(
      `确定清空当前工程全部 ${this.periods.length} 期土方数据及派生放坡成果吗？`
    );
    if (!ok) return;

    try {
      await this.bridge.clearAllPeriods();
      this.viewport.clearAllUserPeriodLayers();
      this.periods.length = 0;
      this.nextPeriodDisplaySeq = 1;
      this.masterRtcOrigin = null;
      this.activeEditSlot = null;
      this.selectedEwBaseSlot = null;
      this.selectedEwTargetSlot = null;
      this.selectedGradingBaseSlot = null;
      this.customGradingPolygon = [];

      const rtcEl = document.getElementById('hud-rtc-origin');
      if (rtcEl) rtcEl.textContent = '未加载';

      this.recomputeGlobalBounds();
      this.renderPeriodLayersList();
      this.syncAllPeriodSelectors();
      await this.refreshUndoRedoButtons();
      this.setStatus('已清空全部土方期数并重置工程统一坐标原点');
    } catch (err) {
      frontendLogger.error('PERIOD', '清空全部工程期数失败', err);
      this.setStatus(`清空失败: ${err instanceof Error ? err.message : String(err)}`);
    }
  }

  // ============================================================================
  // 4. 多期图层卡片与全界面下拉选择器渲染同步
  // ============================================================================

  private renderPeriodLayersList(): void {
    const countEl = document.getElementById('lbl-period-count');
    if (countEl) countEl.textContent = String(this.periods.length);

    const container = document.getElementById('period-layers-list');
    if (!container) return;

    if (this.periods.length === 0) {
      container.innerHTML =
        '<div class="empty-period-hint">尚未导入土方数据。点击上方“导入期数数据”可一次导入单期或批量导入任意多期 .dat 点云文件。</div>';
      return;
    }

    container.innerHTML = '';

    for (const period of this.periods) {
      const isEditTarget = period.slot === this.activeEditSlot;
      const triCount = period.tinSummary?.tinTriangles ?? 0;
      const card = document.createElement('div');
      card.className = `period-item${isEditTarget ? ' active-edit' : ''}`;

      const header = document.createElement('div');
      header.className = 'period-header';

      const titleWrap = document.createElement('div');
      titleWrap.className = 'period-title-wrap';

      const swatch = document.createElement('span');
      swatch.className = 'period-swatch';
      swatch.style.backgroundColor = period.colorHex;

      const nameEl = document.createElement('span');
      nameEl.className = 'period-name';
      nameEl.title = `${period.name}\n文件: ${period.filePath}\n双击可自定义重命名`;
      nameEl.textContent = period.name;
      nameEl.addEventListener('dblclick', () => {
        const renamed = window.prompt('请输入该期土方数据的新名称:', period.name);
        if (renamed && renamed.trim().length > 0) {
          period.name = renamed.trim();
          this.renderPeriodLayersList();
          this.syncAllPeriodSelectors();
        }
      });

      titleWrap.appendChild(swatch);
      titleWrap.appendChild(nameEl);
      header.appendChild(titleWrap);

      if (isEditTarget) {
        const badge = document.createElement('span');
        badge.className = 'period-badge-edit';
        badge.textContent = '编辑中';
        header.appendChild(badge);
      }
      card.appendChild(header);

      const meta = document.createElement('div');
      meta.className = 'period-meta';
      meta.textContent = `点云 ${period.datSummary.validPoints.toLocaleString()} | 三角网 ${triCount.toLocaleString()} 面`;
      card.appendChild(meta);

      // 点云开关 + 三角网开关 + 透明度滑块
      const toggleRow = document.createElement('div');
      toggleRow.className = 'period-toggles-row';

      const checks = document.createElement('div');
      checks.className = 'period-inline-checks';

      const pcLabel = document.createElement('label');
      const pcChk = document.createElement('input');
      pcChk.type = 'checkbox';
      pcChk.checked = period.showPointCloud;
      pcChk.addEventListener('change', () => {
        period.showPointCloud = pcChk.checked;
        this.viewport.getOctreeManager(period.slot)?.setVisible(pcChk.checked);
        this.viewport.rebalancePointBudgets();
      });
      pcLabel.appendChild(pcChk);
      pcLabel.appendChild(document.createTextNode('点云'));

      const tinLabel = document.createElement('label');
      const tinChk = document.createElement('input');
      tinChk.type = 'checkbox';
      tinChk.checked = period.showTin;
      tinChk.addEventListener('change', () => {
        period.showTin = tinChk.checked;
        this.viewport.getTinManager(period.slot)?.setVisible(tinChk.checked);
      });
      tinLabel.appendChild(tinChk);
      tinLabel.appendChild(document.createTextNode('三角网'));

      checks.appendChild(pcLabel);
      checks.appendChild(tinLabel);

      const opacityWrap = document.createElement('div');
      opacityWrap.className = 'period-opacity-wrap';
      opacityWrap.title = '调节该期三角网不透明度，方便多期叠加透视对比';
      const opText = document.createElement('span');
      opText.textContent = `${Math.round(period.tinOpacity * 100)}%`;
      const opRange = document.createElement('input');
      opRange.type = 'range';
      opRange.min = '0.15';
      opRange.max = '1';
      opRange.step = '0.05';
      opRange.value = String(period.tinOpacity);
      opRange.addEventListener('input', () => {
        const val = Number.parseFloat(opRange.value);
        period.tinOpacity = val;
        opText.textContent = `${Math.round(val * 100)}%`;
        this.viewport.getTinManager(period.slot)?.setOpacity(val);
      });
      opacityWrap.appendChild(opText);
      opacityWrap.appendChild(opRange);

      toggleRow.appendChild(checks);
      toggleRow.appendChild(opacityWrap);
      card.appendChild(toggleRow);

      // 快捷操作行：设为编辑 / 居中定位 / 重建网 / 删除
      const actionsRow = document.createElement('div');
      actionsRow.className = 'period-actions-row';

      const btnEdit = document.createElement('button');
      btnEdit.className = 'btn btn-xs';
      btnEdit.textContent = isEditTarget ? '当前编辑' : '设为编辑';
      btnEdit.disabled = isEditTarget;
      btnEdit.addEventListener('click', () => this.setActiveEditSlot(period.slot));

      const btnFocus = document.createElement('button');
      btnFocus.className = 'btn btn-xs';
      btnFocus.textContent = '居中';
      btnFocus.title = '将三维相机居中定位至本期测区范围';
      btnFocus.addEventListener('click', () => {
        this.viewport.focusOnLocalBounds(
          period.datSummary.localMin,
          period.datSummary.localMax
        );
      });

      const btnRebuild = document.createElement('button');
      btnRebuild.className = 'btn btn-xs';
      btnRebuild.textContent = '重构网';
      btnRebuild.title = '使用当前抽稀步长与外围长边裁剪阈值重新构建本期三角网';
      btnRebuild.addEventListener('click', () => {
        void this.handleRebuildPeriodTin(period.slot);
      });

      const btnDel = document.createElement('button');
      btnDel.className = 'btn btn-xs btn-danger-ghost';
      btnDel.textContent = '删除';
      btnDel.addEventListener('click', () => {
        void this.handleRemovePeriod(period.slot);
      });

      actionsRow.appendChild(btnEdit);
      actionsRow.appendChild(btnFocus);
      actionsRow.appendChild(btnRebuild);
      actionsRow.appendChild(btnDel);
      card.appendChild(actionsRow);

      container.appendChild(card);
    }
  }

  private syncAllPeriodSelectors(): void {
    const populateSelect = (
      id: string,
      selectedSlot: SlotId | null,
      emptyText: string
    ): void => {
      const sel = document.getElementById(id) as HTMLSelectElement | null;
      if (!sel) return;
      sel.innerHTML = '';

      if (this.periods.length === 0) {
        const opt = document.createElement('option');
        opt.value = '';
        opt.textContent = emptyText;
        sel.appendChild(opt);
        sel.disabled = true;
        return;
      }

      sel.disabled = false;
      for (const p of this.periods) {
        const opt = document.createElement('option');
        opt.value = String(p.slot);
        opt.textContent = p.name;
        if (selectedSlot !== null && p.slot === selectedSlot) {
          opt.selected = true;
        }
        sel.appendChild(opt);
      }
    };

    populateSelect('sel-edit-period', this.activeEditSlot, '请先导入期数数据');
    populateSelect('sel-ew-base-period', this.selectedEwBaseSlot, '请先导入至少两期数据');
    populateSelect('sel-ew-target-period', this.selectedEwTargetSlot, '请先导入至少两期数据');
    populateSelect('sel-grading-base-period', this.selectedGradingBaseSlot, '请先导入原地面数据');
  }

  // ============================================================================
  // 5. 撤销 / 重做（自动作用于当前选中的编辑目标期数）
  // ============================================================================

  private async handleUndo(): Promise<void> {
    if (this.activeEditSlot === null) return;
    try {
      await this.bridge.undoTin(this.activeEditSlot);
      await this.viewport.getTinManager(this.activeEditSlot)?.syncDirtyTiles();
      await this.refreshUndoRedoButtons();
      this.setStatus('已撤销当前编辑期最近一次三角网拓扑修改');
    } catch (err) {
      frontendLogger.error('HISTORY', '撤销三角网编辑失败', err);
      this.setStatus(`撤销失败: ${err instanceof Error ? err.message : String(err)}`);
    }
  }

  private async handleRedo(): Promise<void> {
    if (this.activeEditSlot === null) return;
    try {
      await this.bridge.redoTin(this.activeEditSlot);
      await this.viewport.getTinManager(this.activeEditSlot)?.syncDirtyTiles();
      await this.refreshUndoRedoButtons();
      this.setStatus('已重做当前编辑期三角网拓扑修改');
    } catch (err) {
      frontendLogger.error('HISTORY', '重做三角网编辑失败', err);
      this.setStatus(`重做失败: ${err instanceof Error ? err.message : String(err)}`);
    }
  }

  private async refreshUndoRedoButtons(): Promise<void> {
    const undoBtn = document.getElementById('btn-undo') as HTMLButtonElement | null;
    const redoBtn = document.getElementById('btn-redo') as HTMLButtonElement | null;
    if (this.activeEditSlot === null) {
      if (undoBtn) undoBtn.disabled = true;
      if (redoBtn) redoBtn.disabled = true;
      return;
    }
    try {
      const state = await this.bridge.getHistoryState(this.activeEditSlot);
      if (undoBtn) undoBtn.disabled = !state.canUndo;
      if (redoBtn) redoBtn.disabled = !state.canRedo;
    } catch {
      if (undoBtn) undoBtn.disabled = true;
      if (redoBtn) redoBtn.disabled = true;
    }
  }

  // ============================================================================
  // 6. 任意两期三角网高精度土方算量
  // ============================================================================

  private async handleComputeEarthwork(exactMode: boolean): Promise<void> {
    if (this.periods.length < 2) {
      this.setStatus('请至少导入 2 期土方点云数据后再执行两期土方算量');
      return;
    }
    if (this.selectedEwBaseSlot === null || this.selectedEwTargetSlot === null) {
      this.setStatus('请在右侧面板选择基准期与对比期');
      return;
    }
    if (this.selectedEwBaseSlot === this.selectedEwTargetSlot) {
      this.setStatus('基准期与对比期不能相同，请选择两个不同期数进行土方计算');
      return;
    }

    const basePeriod = this.findPeriodBySlot(this.selectedEwBaseSlot);
    const targetPeriod = this.findPeriodBySlot(this.selectedEwTargetSlot);
    const pairLabel = `${basePeriod?.name ?? this.selectedEwBaseSlot} → ${targetPeriod?.name ?? this.selectedEwTargetSlot}`;

    try {
      frontendLogger.info(
        'EARTHWORK',
        () =>
          `启动任意两期土方算量: base=${this.selectedEwBaseSlot}, target=${this.selectedEwTargetSlot}, exact=${exactMode}`
      );
      this.setStatus(
        exactMode
          ? `正在执行 [${pairLabel}] 高精度三棱柱布尔叠加土方计算...`
          : `正在执行 [${pairLabel}] 并行网格微分快速土方估算...`
      );

      const report: EarthworkSummary = exactMode
        ? await this.bridge.computeEarthworkExact(
            this.selectedEwBaseSlot,
            this.selectedEwTargetSlot
          )
        : await this.bridge.computeEarthworkGrid(
            this.selectedEwBaseSlot,
            this.selectedEwTargetSlot,
            1.0
          );

      this.updateEarthworkUi(report);
      this.setStatus(
        `[${pairLabel}] 算量完成 (${(report.elapsedSeconds * 1000).toFixed(1)} ms, 积分微元: ${report.intersectedPrisms.toLocaleString()})`
      );
    } catch (err) {
      frontendLogger.error('EARTHWORK', '任意两期土方算量失败', err);
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
    setTxt('res-ew-maxcut', `${rep.maxCutDepthM.toFixed(2)} m`);
    setTxt('res-ew-maxfill', `${rep.maxFillHeightM.toFixed(2)} m`);
  }

  // ============================================================================
  // 7. 基于任意基准期原地面的向内/向外多级放坡建模
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
    const basePeriod = this.findPeriodBySlot(this.selectedGradingBaseSlot);
    if (!basePeriod || !this.masterRtcOrigin) {
      this.setStatus('请先导入至少一期原地面点云并选择放坡基准期后再执行多级放坡');
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
      const autoDaylight = (
        document.getElementById('chk-grade-daylight') as HTMLInputElement
      ).checked;
      const stitchTerrain = (
        document.getElementById('chk-grade-stitch') as HTMLInputElement
      ).checked;

      const localDesignZ = globalDesignElev - this.masterRtcOrigin[2];

      let basePoly = this.customGradingPolygon;
      if (basePoly.length < 3) {
        const cx =
          0.5 * (basePeriod.datSummary.localMin[0] + basePeriod.datSummary.localMax[0]);
        const cy =
          0.5 * (basePeriod.datSummary.localMin[1] + basePeriod.datSummary.localMax[1]);
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

      this.setStatus(
        `正在基于 [${basePeriod.name}] 执行多级放坡建模、原地面求交与土方计算...`
      );
      const res = await this.bridge.computeSlopeGrading(
        basePeriod.slot,
        basePoly,
        this.gradingStages,
        params,
        stitchTerrain
      );

      const gradingMgr = this.viewport.getTinManager(SurfaceSlot.GradingDesignOnly);
      if (gradingMgr) {
        gradingMgr.clearAll();
        await gradingMgr.syncDirtyTiles();
        const chkGrad = document.getElementById('chk-layer-grading') as HTMLInputElement | null;
        if (chkGrad) chkGrad.checked = true;
        gradingMgr.setVisible(true);
      }

      if (stitchTerrain) {
        const mergedMgr = this.viewport.getTinManager(SurfaceSlot.GradingMergedTerrain);
        if (mergedMgr) {
          mergedMgr.clearAll();
          await mergedMgr.syncDirtyTiles();
        }
      }

      this.viewport.updateGradingOverlayLines(basePoly, res.daylightPolyline);

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

      this.setStatus(
        `多级放坡建模完成 (${(res.elapsedSeconds * 1000).toFixed(1)} ms) | 填方: ${res.earthwork.fillVolumeM3.toFixed(1)} m³, 挖方: ${res.earthwork.cutVolumeM3.toFixed(1)} m³`
      );
    } catch (err) {
      frontendLogger.error('GRADING', '多级放坡建模与算量失败', err);
      this.setStatus(`多级放坡计算失败: ${err instanceof Error ? err.message : String(err)}`);
    }
  }

  private startHudMonitor(): void {
    window.setInterval(() => {
      const s = this.viewport.getStatsSummary();
      const fpsEl = document.getElementById('hud-fps');
      const perEl = document.getElementById('hud-period-total');
      const ptsEl = document.getElementById('hud-vis-pts');
      const trisEl = document.getElementById('hud-tin-tris');
      const tilesEl = document.getElementById('hud-gpu-tiles');
      if (fpsEl) fpsEl.textContent = String(s.fps);
      if (perEl) perEl.textContent = String(this.periods.length);
      if (ptsEl) ptsEl.textContent = s.visiblePoints.toLocaleString();
      if (trisEl) trisEl.textContent = s.tinTriangles.toLocaleString();
      if (tilesEl) tilesEl.textContent = String(s.cachedTiles);
    }, 300);
  }
}

window.addEventListener('DOMContentLoaded', () => {
  new GeoStudioApplication();
});

