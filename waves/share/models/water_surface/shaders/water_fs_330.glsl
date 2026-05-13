// Copyright (c) 2016 The UUV Simulator Authors.
// Copyright (C) 2026 Open Source Robotics Foundation
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
// FFT heightmap. Only sampled when foamStrength > 0 (the gerstner
// path leaves that uniform at zero so this sampler can be unbound).
uniform sampler2D heightMap;

uniform vec4 deepColor;
uniform vec4 shallowColor;
uniform float fresnelPower;
uniform float hdrMultiplier;

// Tessendorf chop factor (negative = bunch particles toward crests).
// Used by the foam mask's Jacobian computation.
uniform float chopFactor;
// FFT tile extent in metres — needed to convert finite-difference
// texel deltas back to world-space derivatives.
uniform float tileSize;
// Foam controls. foamStrength=0 disables the foam path entirely and
// the heightmap sampler is never touched.
uniform float foamStrength;
uniform float foamThreshold;

in block
{
  mat3 rotMatrix;
  vec3 eyeVec;
  vec2 bumpCoord;
  vec2 baseXY;
} inPs;

out vec4 fragColor;

// Tessendorf foam mask. The 2D Jacobian of the chop transform
//   (x, y) → (x + c·Dx, y + c·Dy)
// drops below 1 in compression zones (where chop is bunching the
// surface). Sustained low J is where foam forms in real ocean —
// we ramp foam in below `foamThreshold` and saturate at full white
// once the Jacobian crosses into negative (folding).
float ComputeFoamMask()
{
  if (foamStrength <= 0.0)
    return 0.0;
  vec2 heightUV = fract(inPs.baseXY / tileSize);
  ivec2 texSize = textureSize(heightMap, 0);
  vec2 texel = 1.0 / vec2(texSize);
  float dStep = tileSize / float(texSize.x);

  vec4 px = texture(heightMap, heightUV + vec2( texel.x, 0.0));
  vec4 nx = texture(heightMap, heightUV + vec2(-texel.x, 0.0));
  vec4 py = texture(heightMap, heightUV + vec2(0.0,  texel.y));
  vec4 ny = texture(heightMap, heightUV + vec2(0.0, -texel.y));

  float dDxdx = (px.g - nx.g) * 0.5 / dStep;
  float dDydy = (py.b - ny.b) * 0.5 / dStep;
  float dDxdy = (py.g - ny.g) * 0.5 / dStep;
  float dDydx = (px.b - nx.b) * 0.5 / dStep;

  float J = (1.0 + chopFactor * dDxdx) * (1.0 + chopFactor * dDydy)
          - (chopFactor * chopFactor) * dDxdy * dDydx;

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

  vec4 envColor = texture(cubeMap, R, 0.0);
  envColor.rgb *= (envColor.r + envColor.g + envColor.b) * hdrMultiplier;

  float facing = 1.0 - dot(-E, N);
  float waterEnvRatio = clamp(pow(facing, fresnelPower), 0.0, 1.0);

  vec4 waterColor = mix(shallowColor, deepColor, facing);
  vec4 color = mix(waterColor, envColor, waterEnvRatio);

  // Foam overlay: blend toward white where the Tessendorf Jacobian
  // identifies a compression zone.
  float foam = ComputeFoamMask() * foamStrength;
  color.rgb = mix(color.rgb, vec3(1.0), foam);

  fragColor = vec4(color.xyz, 0.9);
}
