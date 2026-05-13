// Copyright (c) 2016 The UUV Simulator Authors.
// All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#version 330

////////// Input parameters //////////
// Textures
uniform sampler2D bumpMap;
uniform samplerCube cubeMap;
// FFT heightmap (only sampled when foamStrength > 0). Layout:
//   .r = η      .g = Dx     .b = Dy     .a = (reserved)
uniform sampler2D heightMap;

// Colors
uniform vec4 deepColor;
uniform vec4 shallowColor;
// `fresnelPower` is kept for backward compatibility but is unused
// when fresnelF0 > 0 — Schlick replaces the older pow(facing, k)
// approximation. Set fresnelF0 = 0 to fall back to the old code path.
uniform float fresnelPower;
uniform float hdrMultiplier;
// Schlick Fresnel reflectance at normal incidence. Water is ~0.02.
// Higher values bias the surface toward more reflection everywhere.
uniform float fresnelF0;
// Reflection roughness in [0, 1]. Adds a LOD bias to the cubemap
// sample so high roughness produces softer, blurrier reflections
// instead of a sharp mirror of the skybox.
uniform float roughness;

// Tessendorf chop displacement scale (negative bunches particles
// toward crests). Used to compute the Jacobian of the chop transform,
// which drives the foam mask.
uniform float chopFactor;
// Physical extent of one heightmap tile in metres. Needed to convert
// finite-difference texel deltas back to world-space derivatives.
uniform float tileSize;
// Foam controls. foamStrength = 0 disables foam entirely (and the
// heightMap sampling that goes with it), so the gerstner path —
// which doesn't bind a heightmap — leaves this at 0.
uniform float foamStrength;
uniform float foamThreshold;

// Crest colour modulation. Where η > 0 we lerp the water colour
// toward `crestColor` to give wave tops visible relief and break the
// uniform deep-blue. Strength scales the maximum blend; reference η
// at which crestStrength is fully applied is `crestRefHeight` (m).
uniform vec3  crestColor;
uniform float crestStrength;
uniform float crestRefHeight;

////////// Input computed in vertex shader //////////
in block
{
  mat3 rotMatrix;
  vec3 eyeVec;
  vec2 bumpCoord;
  vec2 baseXY;       // undisplaced world XY at this fragment
} inPs;

out vec4 fragColor;

// Tessendorf foam mask. Computes the 2D Jacobian of the chop
// transform (x, y) → (x + c·Dx, y + c·Dy) at the surface point. Where
// J drops below 1 the surface is compressing — that's where foam
// forms in practice. Below `foamThreshold` we ramp foam in, smoothly
// saturating to 1 just past J = 0.
float ComputeFoamMask()
{
  if (foamStrength <= 0.0)
    return 0.0;
  vec2 heightUV = fract(inPs.baseXY / tileSize);
  ivec2 texSize = textureSize(heightMap, 0);
  vec2 texel = 1.0 / vec2(texSize);
  // World-space step across one texel.
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

  // Foam ramps from 0 at J = foamThreshold to 1 at J = 0 (full
  // compression), with smoothstep to soften the transition.
  return 1.0 - smoothstep(0.0, foamThreshold, J);
}

void main()
{
  // Apply bump mapping to normal vector to make waves look more detailed:
  vec4 bump = texture(bumpMap, inPs.bumpCoord)*2.0 - 1.0;
  vec3 N = normalize(inPs.rotMatrix * bump.xyz);

  // Reflected ray:
  vec3 E = normalize(inPs.eyeVec);
  vec3 R = reflect(E, N);

  // negate z for use with the skybox texture that comes with gz-rendering
  R = vec3(R.x, R.y, -R.z);

  // Get environment color of reflected ray. The LOD bias mimics
  // pre-filtered roughness — high roughness reads from a higher
  // mip level so the reflected sky is softer/blurrier instead of a
  // sharp mirror. The factor `8.0` is roughly log2 of a typical 256
  // cube face; tighter or coarser values trade detail for softness.
  float lodBias = roughness * 8.0;
  vec4 envColor = texture(cubeMap, R, lodBias);

  // Cheap hdr effect:
  envColor.rgb *= (envColor.r+envColor.g+envColor.b)*hdrMultiplier;

  // Schlick's Fresnel approximation:
  //   F = F0 + (1 - F0) · (1 - cos θ)^5
  // For water (F0 ≈ 0.02) this gives a small reflection near
  // straight-down view and ramps to ~1 at grazing angles, instead of
  // the older `pow(1 - cos θ, fresnelPower)` heuristic that
  // ballooned reflections uniformly.
  float facing = 1.0 - dot(-E, N);
  float schlick = fresnelF0 + (1.0 - fresnelF0) * pow(facing, 5.0);
  // Fall back to the legacy formula if fresnelF0 is zero (e.g. for
  // existing scenes that haven't migrated to Schlick parameters yet).
  float legacy = clamp(pow(facing, fresnelPower), 0.05, 1.0);
  float waterEnvRatio = (fresnelF0 > 0.0) ? schlick : legacy;

  // Refracted ray only considers deep and shallow water colors:
  vec4 waterColor = mix(shallowColor, deepColor, facing);

  // Height-based color: read η at the heightmap-aligned UV (only
  // meaningful on the FFT path, where foamStrength > 0 also guards
  // an otherwise-unbound heightMap sampler) and lerp toward
  // crestColor on the positive-η side. Negative η leaves the colour
  // untouched, so troughs stay the existing deep colour.
  if (foamStrength > 0.0)
  {
    vec2 heightUV = fract(inPs.baseXY / tileSize);
    float eta = texture(heightMap, heightUV).r;
    float crestMix = clamp(eta / max(crestRefHeight, 0.01), 0.0, 1.0);
    crestMix *= crestStrength;
    waterColor.rgb = mix(waterColor.rgb, crestColor, crestMix);
  }

  // Perform linear interpolation between reflection and refraction.
  vec4 color = mix(waterColor, envColor, waterEnvRatio);

  // Foam overlay: blends a flat white toward crests/compression
  // regions identified by the Tessendorf Jacobian.
  float foam = ComputeFoamMask() * foamStrength;
  color.rgb = mix(color.rgb, vec3(1.0), foam);

  fragColor = vec4(color.xyz, 0.9);
}
