// Copyright (C) 2026 Open Source Robotics Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
//
// Vertex displacement from a precomputed FFT heightmap. The heightmap is a
// 2D scalar texture of surface elevation η(x, y, t) generated each tick on
// the CPU (via FFTWaveSimulation) and uploaded to this shader's sampler.
//
// World→texture mapping: the FFT tile is periodic with extent `tileSize`
// metres along both axes; UV = fract(world.xy / tileSize). Surface normal
// is reconstructed by sampling the height at four neighbour texels.
//
// The vertex output block matches the existing water_fs_330.glsl so the
// fragment shader doesn't need a variant — both the Gerstner and FFT
// vertex shaders produce (rotMatrix, eyeVec, bumpCoord).

#version 330

in vec4 vertex;
in vec4 uv0;

uniform mat4 worldviewproj_matrix;
uniform vec3 camera_position_object_space;
uniform float t;
uniform float tau;
uniform float rescale;
uniform vec2 bumpScale;
uniform vec2 bumpSpeed;

uniform float tileSize;          // physical extent of the heightmap tile [m]
uniform int   gridSize;          // heightmap resolution per axis
uniform sampler2D heightMap;     // R32F texture; .r is η(x, y, t)

out block
{
  mat3 rotMatrix;
  vec3 eyeVec;
  vec2 bumpCoord;
} outVs;

out gl_PerVertex
{
  vec4 gl_Position;
};

void main()
{
  vec4 P = vertex;

  // World position → tile-local UV. fract() handles wrap.
  vec2 uv = fract(P.xy / tileSize);

  // Sample η at the vertex.
  float h = texture(heightMap, uv).r;

  // Startup ramp factor matches the analytic Gerstner one.
  float ramp = 1.0 - exp(-1.0 * t / tau);
  P.z += h * ramp;

  // Finite-difference surface normal.
  // texel side in tile space = 1 / gridSize.
  float du = 1.0 / float(gridSize);
  float hL = texture(heightMap, fract(uv + vec2(-du, 0.0))).r;
  float hR = texture(heightMap, fract(uv + vec2( du, 0.0))).r;
  float hD = texture(heightMap, fract(uv + vec2(0.0, -du))).r;
  float hU = texture(heightMap, fract(uv + vec2(0.0,  du))).r;
  // Each texel corresponds to (tileSize / gridSize) metres of world span.
  float dx = (hR - hL) * 0.5 * float(gridSize) / tileSize;
  float dy = (hU - hD) * 0.5 * float(gridSize) / tileSize;
  vec3 N = normalize(vec3(-dx * ramp, -dy * ramp, 1.0));

  // Build a tangent basis (T, B) compatible with the fragment shader,
  // which expects rotMatrix to transform tangent-space bump-map normals
  // back into world space. T is chosen as the orthogonalised world-x axis
  // projected onto the surface plane.
  vec3 T = normalize(vec3(1.0, 0.0, dx * ramp));
  vec3 B = cross(N, T);
  T = cross(B, N);            // re-orthogonalise

  outVs.rotMatrix = mat3(B * rescale, T * rescale, N);

  gl_Position = worldviewproj_matrix * P;

  outVs.bumpCoord = uv0.xy * bumpScale + t * bumpSpeed;
  outVs.eyeVec = P.xyz - camera_position_object_space;
}
