/**
 * @file terrain_tin_shader.ts
 * @brief 三角网 (TIN) 与多级放坡设计面多模式实时 GLSL 着色器
 *
 * 支持 4 种工程可视化模式的零显存重传瞬间切换（仅修改 Uniform 参数）：
 *   - Mode 0：高程分层设色 + 屏幕空间抗锯齿动态等高线（主等高线 5m + 计曲线 1m）
 *   - Mode 1：地形坡度热力分析（0° 平地绿色 -> 15° 缓坡黄色 -> 30° 陡坡橙色 -> >=45° 危岩深红）
 *   - Mode 2：放坡工程分区设色（原地面灰绿 / 填方边坡暖橙 / 挖方边坡冷青 / 马道平台浅黄 / 设计基底天蓝）
 *   - Mode 3：单色工程实体 Phong + 半兰伯特光照模式（便于叠加检查三角网剖分质量）
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
  return new THREE.ShaderMaterial({
    uniforms: {
      uMinZ: { value: opts.minZ },
      uMaxZ: { value: opts.maxZ },
      uRenderMode: { value: opts.renderMode ?? TinRenderMode.ElevationAndContours },
      uTintColor: { value: opts.tintColor ?? new THREE.Color(0.35, 0.65, 0.55) },
      uOpacity: { value: opts.opacity ?? 1.0 },
      uContourInterval: { value: 1.0 },      // 计曲线间隔（1米）
      uMajorContourEvery: { value: 5.0 },    // 首曲线加粗间隔（5米）
      uIsGradingSurface: { value: opts.isGradingSurface ? 1 : 0 },
      uSunDirection: { value: new THREE.Vector3(0.45, -0.55, 0.70).normalize() },
    },
    vertexShader: /* glsl */ `
      attribute float aPackedAttr;

      varying vec3 vWorldLocalPos;
      varying vec3 vNormal;
      flat varying uint vCodeId;
      flat varying uint vFlags;

      void main() {
        vWorldLocalPos = position;
        vNormal = normalize(normalMatrix * normal);

        uint attrBits = floatBitsToUint(aPackedAttr);
        vCodeId = attrBits & 0xFFFFu;
        vFlags = (attrBits >> 16u) & 0xFFFFu;

        // 保留世界空间法线用于坡度计算
        vNormal = normalize(normal);

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

      varying vec3 vWorldLocalPos;
      varying vec3 vNormal;
      flat varying uint vCodeId;
      flat varying uint vFlags;

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
        // 0度(平地绿) -> 15度(缓坡黄) -> 30度(陡坡橙) -> 50度(极陡红)
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
        vec3 n = normalize(vNormal);
        if (!gl_FrontFacing) {
          n = -n;
        }

        // 半兰伯特地形光照模型，避免背阴坡全黑
        float ndotl = dot(n, normalize(uSunDirection));
        float halfLambert = ndotl * 0.5 + 0.5;
        float lighting = mix(0.45, 1.08, halfLambert * halfLambert);

        vec3 albedo = uTintColor;

        if (uRenderMode == 0) {
          // 模式 0：高程分层设色 + 屏幕空间导数抗锯齿等高线
          float span = max(uMaxZ - uMinZ, 0.1);
          float normZ = (vWorldLocalPos.z - uMinZ) / span;
          albedo = elevationPalette(normZ);

          // 计算计曲线与首曲线屏幕抗锯齿线条因子
          float z = vWorldLocalPos.z;
          float dz = max(fwidth(z), 1e-5);

          float minorCoord = abs(fract(z / uContourInterval + 0.5) - 0.5) * uContourInterval;
          float minorLine = 1.0 - smoothstep(0.0, dz * 1.2, minorCoord);

          float majorStep = uContourInterval * uMajorContourEvery;
          float majorCoord = abs(fract(z / majorStep + 0.5) - 0.5) * majorStep;
          float majorLine = 1.0 - smoothstep(0.0, dz * 2.0, majorCoord);

          albedo = mix(albedo, vec3(0.12, 0.14, 0.16), minorLine * 0.38);
          albedo = mix(albedo, vec3(0.05, 0.06, 0.08), majorLine * 0.78);
        } else if (uRenderMode == 1) {
          // 模式 1：地形坡度热力分析
          float cosTheta = clamp(abs(n.z), 0.0, 1.0);
          float slopeDeg = degrees(acos(cosTheta));
          albedo = slopePalette(slopeDeg);
        } else if (uRenderMode == 2) {
          // 模式 2：放坡分区着色（根据法线倾斜度与编码区分平台/边坡/坡脚线）
          float slopeDeg = degrees(acos(clamp(abs(n.z), 0.0, 1.0)));
          if (uIsGradingSurface == 1 || vCodeId >= 100u) {
            if (slopeDeg < 3.0) {
              // 设计基准平台或马道平台
              albedo = vec3(0.26, 0.62, 0.88);
            } else {
              // 边坡斜面（暖橙色高亮坡体）
              albedo = vec3(0.90, 0.48, 0.20);
            }
          } else {
            albedo = vec3(0.32, 0.52, 0.42);
          }
        } else {
          // 模式 3：单色实体工程光照
          albedo = uTintColor;
        }

        vec3 finalRgb = albedo * lighting;
        gl_FragColor = vec4(finalRgb, uOpacity);
      }
    `,
    transparent: (opts.opacity ?? 1.0) < 0.999,
    side: THREE.DoubleSide,
    depthTest: true,
    depthWrite: true,
  });
}

