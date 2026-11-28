/**
 * @file point_cloud_edl.ts
 * @brief 亿级多期测绘点云高程/期数混合设色着色器与 WebGL2 眼罩光照 (EDL) 后处理管线
 *
 * 关键工程修复与升级：
 *   1. 严格声明 `glslVersion: THREE.GLSL3`，完全消除 WebGL2 上下文中混用 GLSL ES 1.00 与 3.00 语法
 *      导致的着色器编译警告或部分显卡驱动渲染异常。
 *   2. 顶点属性 `aAttrPair`（由零拷贝 `Uint16Array` 硬件直接解包为 `vec2(code_id, flags)`）替代原
 *      `floatBitsToUint(aPackedAttr)`，彻底杜绝 Windows Direct3D11/ANGLE 驱动对次正规浮点数清零
 *      (FTZ) 引起的特征点高亮失效问题。
 *   3. 支持多期点云同屏叠加时的独立主题色混合（`uTintColor` + `uTintMix`）：
 *      单期浏览时展示高程分层色带，多期叠加对比时可按期数主题色清晰辨识各期点云空间关系。
 */

import * as THREE from 'three';

export interface PointCloudMaterialOptions {
  minZ: number;
  maxZ: number;
  pointSize?: number;
  tintColor?: THREE.Color;
  /** 0.0 = 纯高程色带，1.0 = 纯期数主题色，默认 0.0 */
  tintMix?: number;
}

/**
 * 第一通道：多期点云实体渲染着色器（WebGL2 GLSL ES 3.00）
 */
export function createPointCloudMaterial(
  opts: PointCloudMaterialOptions
): THREE.ShaderMaterial {
  return new THREE.ShaderMaterial({
    glslVersion: THREE.GLSL3,
    uniforms: {
      uMinZ: { value: opts.minZ },
      uMaxZ: { value: opts.maxZ },
      uPointSize: { value: opts.pointSize ?? 2.2 },
      uViewportHeight: { value: 1080.0 },
      uTintColor: { value: (opts.tintColor ?? new THREE.Color('#22c55e')).clone() },
      uTintMix: { value: opts.tintMix ?? 0.0 },
    },
    vertexShader: /* glsl */ `
      // 硬件 UNSIGNED_SHORT 零拷贝解包：x = code_id (0..65535), y = flags (bit1 = kFlagFeature)
      in vec2 aAttrPair;

      uniform float uMinZ;
      uniform float uMaxZ;
      uniform float uPointSize;
      uniform float uViewportHeight;
      uniform vec3 uTintColor;
      uniform float uTintMix;

      out vec3 vColor;

      // 五段式工程地形高程色带（深蓝洼地 -> 青绿平地 -> 浅黄缓坡 -> 橙棕山脊 -> 灰白峰顶）
      vec3 hypsometricRamp(float t) {
        t = clamp(t, 0.0, 1.0);
        vec3 c0 = vec3(0.11, 0.36, 0.68);
        vec3 c1 = vec3(0.18, 0.67, 0.55);
        vec3 c2 = vec3(0.66, 0.82, 0.36);
        vec3 c3 = vec3(0.86, 0.55, 0.22);
        vec3 c4 = vec3(0.95, 0.93, 0.90);
        if (t < 0.25) return mix(c0, c1, t * 4.0);
        if (t < 0.50) return mix(c1, c2, (t - 0.25) * 4.0);
        if (t < 0.75) return mix(c2, c3, (t - 0.50) * 4.0);
        return mix(c3, c4, (t - 0.75) * 4.0);
      }

      void main() {
        vec4 mvPosition = modelViewMatrix * vec4(position, 1.0);
        gl_Position = projectionMatrix * mvPosition;

        // 结合视口高度与观察距离进行透视自适应点径缩放，避免高分屏点云过细
        float dist = max(-mvPosition.z, 0.5);
        float heightScale = clamp(uViewportHeight / 900.0, 0.75, 2.0);
        float adaptiveSize = uPointSize * heightScale * (320.0 / dist);
        gl_PointSize = clamp(adaptiveSize, 1.6, 6.5);

        float span = max(uMaxZ - uMinZ, 0.1);
        float normZ = (position.z - uMinZ) / span;
        vec3 elevCol = hypsometricRamp(normZ);
        vec3 baseCol = mix(elevCol, uTintColor, clamp(uTintMix, 0.0, 1.0));

        // 精确还原特征点位标志 (bit1 = 0x0002)
        int flagsInt = int(aAttrPair.y + 0.5);
        bool isFeature = (flagsInt & 2) != 0;
        if (isFeature) {
          vColor = mix(baseCol, vec3(1.0, 0.85, 0.18), 0.70);
          gl_PointSize = clamp(gl_PointSize * 1.32, 2.2, 7.5);
        } else {
          vColor = baseCol;
        }
      }
    `,
    fragmentShader: /* glsl */ `
      in vec3 vColor;

      void main() {
        // 圆形点精灵裁剪，消除方形像素锯齿
        vec2 coord = gl_PointCoord - vec2(0.5);
        if (dot(coord, coord) > 0.25) {
          discard;
        }
        gl_FragColor = vec4(vColor, 1.0);
      }
    `,
    depthTest: true,
    depthWrite: true,
  });
}

/**
 * 第二通道：全屏眼罩光照 (Eye-Dome Lighting, EDL) 深度梯度阴影合成着色器（WebGL2 GLSL ES 3.00）
 */
export function createEdlPostProcessMaterial(): THREE.ShaderMaterial {
  return new THREE.ShaderMaterial({
    glslVersion: THREE.GLSL3,
    uniforms: {
      tColor: { value: null },
      tDepth: { value: null },
      uResolution: { value: new THREE.Vector2(1920, 1080) },
      uCameraNear: { value: 0.5 },
      uCameraFar: { value: 20000.0 },
      uEdlStrength: { value: 1.35 },
      uEdlRadius: { value: 1.5 },
      uEnableEdl: { value: 1 },
    },
    vertexShader: /* glsl */ `
      out vec2 vUv;
      void main() {
        vUv = uv;
        gl_Position = vec4(position.xy, 0.0, 1.0);
      }
    `,
    fragmentShader: /* glsl */ `
      uniform sampler2D tColor;
      uniform sampler2D tDepth;
      uniform vec2 uResolution;
      uniform float uCameraNear;
      uniform float uCameraFar;
      uniform float uEdlStrength;
      uniform float uEdlRadius;
      uniform int uEnableEdl;

      in vec2 vUv;

      // 将透视深度缓冲还原为线性观察深度并取对数，适配数十米至数十公里测区跨度
      float getLogLinearDepth(vec2 uv) {
        float rawDepth = texture(tDepth, uv).r;
        if (rawDepth >= 0.99999) {
          return -1.0;
        }
        float ndc = rawDepth * 2.0 - 1.0;
        float denom = max(uCameraFar + uCameraNear - ndc * (uCameraFar - uCameraNear), 1e-5);
        float linearZ = (2.0 * uCameraNear * uCameraFar) / denom;
        return log2(max(linearZ, 0.001));
      }

      void main() {
        vec4 baseColor = texture(tColor, vUv);
        if (uEnableEdl == 0) {
          gl_FragColor = baseColor;
          return;
        }

        float centerLogDepth = getLogLinearDepth(vUv);
        if (centerLogDepth < 0.0) {
          gl_FragColor = baseColor;
          return;
        }

        vec2 safeRes = max(uResolution, vec2(1.0));
        vec2 texel = uEdlRadius / safeRes;

        vec2 offsets[8] = vec2[8](
          vec2(-1.0,  0.0),
          vec2( 1.0,  0.0),
          vec2( 0.0, -1.0),
          vec2( 0.0,  1.0),
          vec2(-0.7071, -0.7071),
          vec2( 0.7071, -0.7071),
          vec2(-0.7071,  0.7071),
          vec2( 0.7071,  0.7071)
        );

        float response = 0.0;
        for (int i = 0; i < 8; ++i) {
          float nbLogDepth = getLogLinearDepth(vUv + offsets[i] * texel);
          if (nbLogDepth > 0.0) {
            response += max(0.0, centerLogDepth - nbLogDepth);
          }
        }

        float shade = exp(-response * 210.0 * uEdlStrength);
        gl_FragColor = vec4(baseColor.rgb * shade, baseColor.a);
      }
    `,
    depthTest: false,
    depthWrite: false,
  });
}

