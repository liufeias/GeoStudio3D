/**
 * @file terrain_tin_shader.ts
 * @brief 三角网 (TIN) 与多级放坡设计面多模式实时 WebGL2 着色器（GLSL ES 3.00）
 *
 * 核心工程修复与视觉增强：
 *   1. 严格使用 `glslVersion: THREE.GLSL3` 标准语法，彻底解决 WebGL2 下 `uint`/`flat`/`fwidth`
 *      在不同显卡与 ANGLE 后端行为不一致的问题。
 *   2. 开启硬件多边形深度偏移 (`polygonOffset: true, polygonOffsetFactor: 1, polygonOffsetUnits: 1`)，
 *      确保叠加渲染的三角网拓扑格网线在平地、缓坡及近乎垂直的基坑/边坡陡坎上均清晰连续，
 *      彻底杜绝格网线闪烁断续或悬空错位。
 *   3. 双法线鲁棒光照（顶点平滑法线 + 屏幕导数面片几何法线兜底与适度混合）：
 *      当某些退化顶点法线接近零向量时自动回退至精确片元几何法线，同时保留 22% 几何面片清晰度，
 *      使三角网的坡折线、山脊线与马道阶梯轮廓一目了然。
 *   4. 四种实时工程着色模式零显存重传瞬间切换：
 *      - Mode 0：高程分层设色 + 屏幕空间导数抗锯齿动态等高线（1m 计曲线 + 5m 首曲线）
 *      - Mode 1：地形坡度热力分析（0° 平地绿 -> 15° 缓坡黄 -> 30° 陡坡橙 -> >=50° 危岩深红）
 *      - Mode 2：放坡工程分区设色（自动区分基准平台/马道、填挖边坡斜面与原地面地形起伏）
 *      - Mode 3：期数主题单色光照模式（多期三角网同屏叠加对比时按各自主题色清晰区分）
 */

import * as THREE from 'three';

export enum TinRenderMode {
  ElevationAndContours = 0,
  SlopeGradient = 1,
  GradingRegionClass = 2,
  SolidShaded = 3,
}

export interface TinShaderOptions {
  minZ: number;
  maxZ: number;
  renderMode?: TinRenderMode;
  tintColor?: THREE.Color;
  opacity?: number;
  isGradingSurface?: boolean;
}

export function createTerrainTinMaterial(opts: TinShaderOptions): THREE.ShaderMaterial {
  const opacity = opts.opacity ?? 1.0;
  const isTranslucent = opacity < 0.999;

  return new THREE.ShaderMaterial({
    glslVersion: THREE.GLSL3,
    uniforms: {
      uMinZ: { value: opts.minZ },
      uMaxZ: { value: opts.maxZ },
      uRenderMode: { value: opts.renderMode ?? TinRenderMode.ElevationAndContours },
      uTintColor: { value: (opts.tintColor ?? new THREE.Color(0.22, 0.72, 0.52)).clone() },
      uOpacity: { value: opacity },
      uContourInterval: { value: 1.0 },
      uMajorContourEvery: { value: 5.0 },
      uIsGradingSurface: { value: opts.isGradingSurface ? 1 : 0 },
      uSunDirection: { value: new THREE.Vector3(0.45, -0.55, 0.70).normalize() },
      uFillLightDirection: { value: new THREE.Vector3(-0.60, 0.45, 0.55).normalize() },
    },
    vertexShader: /* glsl */ `
      // 硬件 UNSIGNED_SHORT 零拷贝解包：x = code_id (0..65535), y = flags
      in vec2 aAttrPair;

      out vec3 vWorldLocalPos;
      out vec3 vSmoothNormal;
      flat out vec2 vAttrPair;

      void main() {
        vWorldLocalPos = position;
        vSmoothNormal = normal;
        vAttrPair = aAttrPair;
        gl_Position = projectionMatrix * modelViewMatrix * vec4(position, 1.0);
      }
    `,
    fragmentShader: /* glsl */ `
      uniform float uMinZ;
      uniform float uMaxZ;
      uniform int uRenderMode;
      uniform vec3 uTintColor;
      uniform float uOpacity;
      uniform float uContourInterval;
      uniform float uMajorContourEvery;
      uniform int uIsGradingSurface;
      uniform vec3 uSunDirection;
      uniform vec3 uFillLightDirection;

      in vec3 vWorldLocalPos;
      in vec3 vSmoothNormal;
      flat in vec2 vAttrPair;

      vec3 elevationPalette(float t) {
        t = clamp(t, 0.0, 1.0);
        vec3 c0 = vec3(0.14, 0.42, 0.66);
        vec3 c1 = vec3(0.24, 0.68, 0.48);
        vec3 c2 = vec3(0.74, 0.84, 0.40);
        vec3 c3 = vec3(0.84, 0.54, 0.26);
        vec3 c4 = vec3(0.92, 0.88, 0.84);
        if (t < 0.25) return mix(c0, c1, t * 4.0);
        if (t < 0.50) return mix(c1, c2, (t - 0.25) * 4.0);
        if (t < 0.75) return mix(c2, c3, (t - 0.50) * 4.0);
        return mix(c3, c4, (t - 0.75) * 4.0);
      }

      vec3 slopePalette(float slopeDeg) {
        float t = clamp(slopeDeg / 50.0, 0.0, 1.0);
        vec3 s0 = vec3(0.16, 0.68, 0.38);
        vec3 s1 = vec3(0.92, 0.82, 0.22);
        vec3 s2 = vec3(0.92, 0.44, 0.14);
        vec3 s3 = vec3(0.75, 0.10, 0.14);
        if (t < 0.33) return mix(s0, s1, t * 3.0);
        if (t < 0.66) return mix(s1, s2, (t - 0.33) * 3.0);
        return mix(s2, s3, (t - 0.66) * 3.0);
      }

      void main() {
        // 1. 计算片元精确几何面法线，作为平滑法线的清晰度增强与退化法线兜底
        vec3 dpdx = dFdx(vWorldLocalPos);
        vec3 dpdy = dFdy(vWorldLocalPos);
        vec3 rawFaceCross = cross(dpdx, dpdy);
        float faceLen = length(rawFaceCross);
        vec3 faceNormal = (faceLen > 1e-8) ? (rawFaceCross / faceLen) : vec3(0.0, 0.0, 1.0);
        if (faceNormal.z < 0.0) {
          faceNormal = -faceNormal;
        }

        float smoothLen = length(vSmoothNormal);
        vec3 smoothNormal = (smoothLen > 1e-5) ? (vSmoothNormal / smoothLen) : faceNormal;
        if (smoothNormal.z < 0.0) {
          smoothNormal = -smoothNormal;
        }

        // 融合 78% 顶点平滑法线 + 22% 几何面法线，兼顾曲面顺滑与坡折线清晰度
        vec3 n = normalize(mix(smoothNormal, faceNormal, 0.22));
        if (!gl_FrontFacing) {
          n = -n;
        }

        // 双光源工程地形光照（主太阳光 + 柔和背阴补光，立体感强且背坡不死黑）
        float keyNdotL = clamp(dot(n, normalize(uSunDirection)), 0.0, 1.0);
        float fillNdotL = clamp(dot(n, normalize(uFillLightDirection)), 0.0, 1.0);
        float lighting = 0.36 + 0.56 * keyNdotL + 0.18 * fillNdotL;

        int codeId = int(vAttrPair.x + 0.5);
        float slopeDeg = degrees(acos(clamp(abs(faceNormal.z), 0.0, 1.0)));

        vec3 albedo = uTintColor;

        if (uRenderMode == 0) {
          // 模式 0：高程分层设色 + 屏幕空间抗锯齿等高线
          float span = max(uMaxZ - uMinZ, 0.1);
          float normZ = (vWorldLocalPos.z - uMinZ) / span;
          albedo = elevationPalette(normZ);

          float z = vWorldLocalPos.z;
          float dz = max(fwidth(z), 1e-5);
          float minorCoord = abs(fract(z / uContourInterval + 0.5) - 0.5) * uContourInterval;
          float minorLine = 1.0 - smoothstep(0.0, dz * 1.15, minorCoord);

          float majorStep = uContourInterval * uMajorContourEvery;
          float majorCoord = abs(fract(z / majorStep + 0.5) - 0.5) * majorStep;
          float majorLine = 1.0 - smoothstep(0.0, dz * 1.9, majorCoord);

          albedo = mix(albedo, vec3(0.10, 0.13, 0.16), minorLine * 0.36);
          albedo = mix(albedo, vec3(0.04, 0.05, 0.07), majorLine * 0.78);
        } else if (uRenderMode == 1) {
          // 模式 1：基于精确面片几何法线的坡度热力分析
          albedo = slopePalette(slopeDeg);
        } else if (uRenderMode == 2) {
          // 模式 2：放坡工程分区设色（平台 / 边坡 / 马道 / 原地面）
          if (uIsGradingSurface == 1 || codeId >= 100) {
            if (slopeDeg < 2.5) {
              // 设计基准平台或马道平台（清澈工程天蓝）
              albedo = vec3(0.24, 0.62, 0.90);
            } else {
              // 边坡斜面（醒目暖橙，便于检查坡比与阶梯层次）
              albedo = vec3(0.92, 0.49, 0.20);
            }
          } else {
            // 原地面区域保留柔和绿褐高程明暗过渡，不死板成纯单色
            float span = max(uMaxZ - uMinZ, 0.1);
            float normZ = clamp((vWorldLocalPos.z - uMinZ) / span, 0.0, 1.0);
            albedo = mix(vec3(0.24, 0.44, 0.36), vec3(0.42, 0.58, 0.44), normZ);
          }
        } else {
          // 模式 3：期数主题单色实体光照（叠加微小高程亮度渐变便于辨识起伏）
          float span = max(uMaxZ - uMinZ, 0.1);
          float normZ = clamp((vWorldLocalPos.z - uMinZ) / span, 0.0, 1.0);
          albedo = uTintColor * mix(0.84, 1.12, normZ);
        }

        vec3 finalRgb = albedo * lighting;
        gl_FragColor = vec4(finalRgb, uOpacity);
      }
    `,
    transparent: isTranslucent,
    side: THREE.DoubleSide,
    depthTest: true,
    depthWrite: !isTranslucent,
    // 硬件深度偏移：使实心三角面略微退后，保证叠加的三角网真实边线无 Z-Fighting 闪烁
    polygonOffset: true,
    polygonOffsetFactor: 1.0,
    polygonOffsetUnits: 1.0,
  });
}

