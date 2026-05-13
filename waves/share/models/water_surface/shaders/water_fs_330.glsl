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

uniform vec4 deepColor;
uniform vec4 shallowColor;
uniform float fresnelPower;
uniform float hdrMultiplier;

in block
{
  mat3 rotMatrix;
  vec3 eyeVec;
  vec2 bumpCoord;
} inPs;

out vec4 fragColor;

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

  fragColor = vec4(color.xyz, 0.9);
}
