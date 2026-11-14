/**
 * @file point_cloud_edl.ts
 * @brief 亿级测绘点云高程设色着色器与屏幕空间眼罩光照 (Eye-Dome Lighting, EDL) 管线
 *
 * 技术原理与工程价值：
 *   1. 原始测绘 .dat 点云没有法向量（每点仅 16 字节：float32 xyz + uint32 attr），
 *      直接渲染会糊成一片无法分辨山脊、坎线与边坡立体层次。
 *   2. EDL 眼罩光照在离屏 WebGLRenderTarget 中同时输出颜色缓冲与高精线性深度缓冲，
 *      随后在全屏后处理片元着色器中采样邻域像素深度梯度，实时合成模拟天光遮蔽与轮廓阴影，
 *      无需存储法向量即可呈现媲美激光雷达三维实景的立体视觉效果。
 *   3. 特征点（如坎顶 KD、坎脚 KJ、控制点）自动通过 packed_attr 标志位高亮为醒目金色标记。
 */

import * as THREE from 'three';

/**
 * 第一通道：点云实体高程设色 + 线性对数深度输出着色器
 */
export function createPointCloudMaterial(
  minZ: number,
  maxZ: number,
  pointSize = 2.2
): THREE.ShaderMaterial {
  return new THREE.ShaderMaterial({
    uniforms: {
      uMinZ: { value: minZ },
      uMaxZ: { value: maxZ },
      uPointSize: { value: pointSize },
      uViewportHeight: { value: 1080.0 },
    },
    vertexShader: /* glsl */ `
      attribute float aPackedAttr;

      uniform float uMinZ;
      uniform float uMaxZ;
      uniform float uPointSize;
      uniform float uViewportHeight;

      varying vec3 vColor;

      // 五段式测绘地形高程色带（深蓝低洼 -> 青绿平地 -> 浅黄缓坡 -> 橙棕山脊 -> 峰顶白雪）
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

        // 透视自适应点大小（近大远小，并限制最小/最大像素尺寸）
        float dist = max(-mvPosition.z, 0.5);
        float adaptiveSize = uPointSize * (320.0 / dist);
        gl_PointSize = clamp(adaptiveSize, 1.5, 6.0);

        float span = max(uMaxZ - uMinZ, 0.1);
        float normZ = (position.z - uMinZ) / span;
        vec3 baseCol = hypsometricRamp(normZ);

        // 解包高 16 位属性标志（bit1 = kFlagFeature 特征点）
        uint attrBits = floatBitsToUint(aPackedAttr);
        uint flags = (attrBits >> 16u) & 0xFFFFu;
        bool isFeature = (flags & 2u) != 0u;

        if (isFeature) {
          vColor = mix(baseCol, vec3(1.0, 0.84, 0.15), 0.72);
          gl_PointSize = clamp(gl_PointSize * 1.35, 2.0, 7.5);
        } else {
          vColor = baseCol;
        }
      }
    `,
    fragmentShader: /* glsl */ `
      varying vec3 vColor;

      void main() {
        // 圆形软边缘裁剪，消除方块像素感
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
 * 第二通道：全屏眼罩光照 (Eye-Dome Lighting, EDL) 深度梯度合成着色器
 */
export function createEdlPostProcessMaterial(): THREE.ShaderMaterial {
  return new THREE.ShaderMaterial({
    uniforms: {
      tColor: { value: null },
      tDepth: { value: null },
      uResolution: { value: new THREE.Vector2(1920, 1080) },
      uCameraNear: { value: 0.5 },
      uCameraFar: { value: 20000.0 },
      uEdlStrength: { value: 1.4 },
      uEdlRadius: { value: 1.5 },
      uEnableEdl: { value: 1 },
    },
    vertexShader: /* glsl */ `
      varying vec2 vUv;
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

      varying vec2 vUv;

      // 将硬件透视深度缓冲还原为线性观察空间深度，并取对数压缩大跨度地形尺度
      float getLogLinearDepth(vec2 uv) {
        float rawDepth = texture2D(tDepth, uv).r;
        if (rawDepth >= 0.99999) {
          return -1.0; // 背景天空盒像素
        }
        float ndc = rawDepth * 2.0 - 1.0;
        float linearZ = (2.0 * uCameraNear * uCameraFar) /
                        (uCameraFar + uCameraNear - ndc * (uCameraFar - uCameraNear));
        return log2(max(linearZ, 0.001));
      }

      void main() {
        vec4 baseColor = texture2D(tColor, vUv);
        if (uEnableEdl == 0) {
          gl_FragColor = baseColor;
          return;
        }

        float centerLogDepth = getLogLinearDepth(vUv);
        if (centerLogDepth < 0.0) {
          gl_FragColor = baseColor;
          return;
        }

        // 8 邻域方向采样（十字 + 对角线）计算局部遮蔽响应
        vec2 texel = uEdlRadius / uResolution;
        vec2 offsets[8];
        offsets[0] = vec2(-1.0,  0.0);
        offsets[1] = vec2( 1.0,  0.0);
        offsets[2] = vec2( 0.0, -1.0);
        offsets[3] = vec2( 0.0,  1.0);
        offsets[4] = vec2(-0.7071, -0.7071);
        offsets[5] = vec2( 0.7071, -0.7071);
        offsets[6] = vec2(-0.7071,  0.7071);
        offsets[7] = vec2( 0.7071,  0.7071);

        float response = 0.0;
        for (int i = 0; i < 8; ++i) {
          float nbLogDepth = getLogLinearDepth(vUv + offsets[i] * texel);
          if (nbLogDepth > 0.0) {
            // 仅累加邻域比当前像素更靠近相机的遮挡贡献
            response += max(0.0, centerLogDepth - nbLogDepth);
          }
        }

        float shade = exp(-response * 220.0 * uEdlStrength);
        gl_FragColor = vec4(baseColor.rgb * shade, baseColor.a);
      }
    `,
    depthTest: false,
    depthWrite: false,
  });
}

