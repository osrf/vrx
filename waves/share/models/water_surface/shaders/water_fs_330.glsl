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
uniform float fresnelPower;
uniform float hdrMultiplier;

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

////////// Input computed in vertex shader //////////
in block
{
  mat3 rotMatrix;
  vec3 eyeVec;
  vec2 bumpCoord;
  vec2 heightUV;
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
  ivec2 texSize = textureSize(heightMap, 0);
  vec2 texel = 1.0 / vec2(texSize);
  // World-space step across one texel.
  float dStep = tileSize / float(texSize.x);

  vec4 px = texture(heightMap, inPs.heightUV + vec2( texel.x, 0.0));
  vec4 nx = texture(heightMap, inPs.heightUV + vec2(-texel.x, 0.0));
  vec4 py = texture(heightMap, inPs.heightUV + vec2(0.0,  texel.y));
  vec4 ny = texture(heightMap, inPs.heightUV + vec2(0.0, -texel.y));

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

  // Get environment color of reflected ray:
  vec4 envColor = texture(cubeMap, R, 0.0);

  // Cheap hdr effect:
  envColor.rgb *= (envColor.r+envColor.g+envColor.b)*hdrMultiplier;

  // Compute refraction ratio (Fresnel):
  float facing = 1.0 - dot(-E, N);
  float waterEnvRatio = clamp(pow(facing, fresnelPower), 0.05, 1.0);

  // Refracted ray only considers deep and shallow water colors:
  vec4 waterColor = mix(shallowColor, deepColor, facing);

  // Perform linear interpolation between reflection and refraction.
  vec4 color = mix(waterColor, envColor, waterEnvRatio);

  // Foam overlay: blends a flat white toward crests/compression
  // regions identified by the Tessendorf Jacobian.
  float foam = ComputeFoamMask() * foamStrength;
  color.rgb = mix(color.rgb, vec3(1.0), foam);

  fragColor = vec4(color.xyz, 0.9);
}
