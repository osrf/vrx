// Copyright (c) 2016 The UUV Simulator Authors.
// Copyright (C) 2026 Honu Robotics
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Baseline water surface shader. Intentionally simple:
//   - One bump-map octave perturbs the surface normal.
//   - Reflection direction samples a cubemap once (no LOD bias).
//   - "Cheap HDR" boosts bright reflection pixels by their own
//     luminance × hdrMultiplier.
//   - Fresnel approximated as pow(1 - cos θ, fresnelPower).
//   - Water body colour blends between shallow and deep by the same
//     angular factor (more deep at grazing angles — artistic choice).
//
// Visual extras (foam mask, Schlick Fresnel, reflection roughness,
// crest colour modulation, multi-octave normals) live in follow-up
// commits so each can be evaluated independently against this
// baseline.

#version 330

uniform sampler2D bumpMap;
uniform samplerCube cubeMap;
// Wave heightmap grid (η, Dx, Dy, foam). Always bound: the vertex shader
// samples it for the surface displacement. Here in the fragment shader it is
// read only for the foam alpha when foamStrength > 0 (the gerstner path leaves
// foamStrength at 0, so this read is skipped, but the texture stays bound).
uniform sampler2D heightMap;

uniform vec4 deepColor;
uniform vec4 shallowColor;
uniform float fresnelPower;
uniform float hdrMultiplier;
// Reflection roughness in [0, 1]. Adds a LOD bias to the cubemap
// sample so the reflected sky reads as a soft diffuse colour rather
// than a sharp mirror — closer to how real water reflects light.
uniform float roughness;

// FFT tile extent in metres — needed to convert finite-difference
// texel deltas back to world-space derivatives.
uniform float tileSize;
// Foam controls. foamStrength=0 disables the foam path entirely and
// the heightmap sampler is never touched.
uniform float foamStrength;
uniform float foamThreshold;
// When non-zero, the heightmap's alpha channel already holds an analytic
// folding metric (the displacement Jacobian's minimum eigenvalue, 1 = flat,
// < 1 → folding) computed on the CPU. The foam mask then reads it with a
// single sample instead of finite-differencing the displacement.
uniform int useFoamMap;

in block
{
  mat3 rotMatrix;
  vec3 eyeVec;
  vec2 bumpCoord;
  vec2 baseXY;
} inPs;

out vec4 fragColor;

// Tessendorf foam mask. The 2D Jacobian of the chop transform
//   (x, y) → (x + Dx, y + Dy)
// (Dx/Dy already carry the engine-applied choppiness) drops below 1
// in compression zones (where chop is bunching the surface).
// Sustained low J is where foam forms in real ocean —
// we ramp foam in below `foamThreshold` and saturate at full white
// once the Jacobian crosses into negative (folding).
float ComputeFoamMask()
{
  if (foamStrength <= 0.0)
    return 0.0;
  vec2 heightUV = fract(inPs.baseXY / tileSize);

  // Precomputed analytic folding metric in alpha (Encino path): one sample,
  // no finite differencing, resolution-independent. 1 = flat, < 1 → folding.
  if (useFoamMap != 0)
  {
    float J = texture(heightMap, heightUV).a;
    return 1.0 - smoothstep(-foamThreshold, foamThreshold, J);
  }

  ivec2 texSize = textureSize(heightMap, 0);

  // Sample the chop derivatives over a FIXED PHYSICAL stencil
  // (~1 m), not one texel. A per-texel stencil makes the foam mask
  // resolution-dependent: at grid_size=1024 / tile_size=256 the
  // step would be 0.25 m, capturing aggressive high-frequency
  // gradients that drive the Jacobian negative across the entire
  // surface → 100% white foam. A fixed metric step keeps the
  // visual identical across grid resolutions.
  float dStep = max(1.0, tileSize / float(texSize.x));
  vec2 stencil = vec2(dStep / tileSize);  // back into UV space

  vec4 px = texture(heightMap, heightUV + vec2( stencil.x, 0.0));
  vec4 nx = texture(heightMap, heightUV + vec2(-stencil.x, 0.0));
  vec4 py = texture(heightMap, heightUV + vec2(0.0,  stencil.y));
  vec4 ny = texture(heightMap, heightUV + vec2(0.0, -stencil.y));

  float dDxdx = (px.g - nx.g) * 0.5 / dStep;
  float dDydy = (py.b - ny.b) * 0.5 / dStep;
  float dDxdy = (py.g - ny.g) * 0.5 / dStep;
  float dDydx = (px.b - nx.b) * 0.5 / dStep;

  float J = (1.0 + dDxdx) * (1.0 + dDydy) - dDxdy * dDydx;

  // Symmetric smoothstep window centred around J = 0 widens the
  // transition so foam edges don't cut hard.
  return 1.0 - smoothstep(-foamThreshold, foamThreshold, J);
}

void main()
{
  vec4 bump = texture(bumpMap, inPs.bumpCoord) * 2.0 - 1.0;
  vec3 N = normalize(inPs.rotMatrix * bump.xyz);

  vec3 E = normalize(inPs.eyeVec);
  vec3 R = reflect(E, N);
  R = vec3(R.x, R.y, -R.z);  // gz-rendering cubemap Z flip

  // Soft reflection: the LOD bias picks a coarser mip of the
  // cubemap, blurring the sky into a diffuse tint instead of a
  // sharp mirror of clouds.
  float lodBias = roughness * 8.0;
  vec4 envColor = texture(cubeMap, R, lodBias);
  envColor.rgb *= (envColor.r + envColor.g + envColor.b) * hdrMultiplier;

  float facing = 1.0 - dot(-E, N);
  float waterEnvRatio = clamp(pow(facing, fresnelPower), 0.0, 1.0);

  vec4 waterColor = mix(shallowColor, deepColor, facing);
  vec4 color = mix(waterColor, envColor, waterEnvRatio);

  // Foam overlay: blend toward white where the Tessendorf Jacobian
  // identifies a compression zone.
  float foam = ComputeFoamMask() * foamStrength;
  color.rgb = mix(color.rgb, vec3(1.0), foam);

  // Opaque surface: alpha 1.0 keeps the water out of the transparent
  // render path (no blend state is configured), avoiding depth-sort artifacts.
  fragColor = vec4(color.xyz, 1.0);
}
