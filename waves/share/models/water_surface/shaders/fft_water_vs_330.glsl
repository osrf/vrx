// Copyright (C) 2026 Open Source Robotics Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
//
// Tessendorf ocean vertex shader. Reads a single RGBA32F heightmap
// where each texel packs (η, Dx, Dy, _) and uses it to displace the
// vertex both vertically (η) and laterally (Dx, Dy scaled by
// chopFactor).
//
// Per-vertex surface normal is computed via central differences on
// the same heightmap — sample four world-space-neighbour cells, take
// the cross product of the two finite-difference vectors, normalise.
// The tangent basis (T, B, N) is forwarded to the fragment shader as
// a rotation matrix.
//
// Bumpmap UV uses a hardcoded `bumpResolution` factor (matches the
// asv_wave_sim convention of a dense per-uv-unit tiling): the
// effective bumpScale is `bumpScale × bumpResolution`. With the
// default bumpScale of (64, 64), that's 1024 bumpmap repeats per uv
// unit — plenty of micro-texture detail.

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

uniform float tileSize;
uniform int   gridSize;
uniform float chopFactor;
uniform sampler2D heightMap;

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

vec3 SampleDisplaced(vec2 xy)
{
  vec2 uv = fract(xy / tileSize);
  vec4 hd = texture(heightMap, uv);
  vec2 dxy = chopFactor * hd.gb;
  return vec3(xy + dxy, hd.r);
}

void main()
{
  vec4 P = vertex;

  vec3 disp = SampleDisplaced(P.xy);
  P.xy = disp.xy;
  P.z += disp.z;

  // Finite-difference normal across one texel of the heightmap.
  float texel = tileSize / float(gridSize);
  vec3 px = SampleDisplaced(vertex.xy + vec2( texel, 0.0));
  vec3 nx = SampleDisplaced(vertex.xy + vec2(-texel, 0.0));
  vec3 py = SampleDisplaced(vertex.xy + vec2(0.0,  texel));
  vec3 ny = SampleDisplaced(vertex.xy + vec2(0.0, -texel));
  vec3 dxv = (px - nx) * 0.5;
  vec3 dyv = (py - ny) * 0.5;
  vec3 N = normalize(cross(dxv, dyv));
  if (N.z < 0.0) N = -N;

  vec3 T = normalize(dxv - dot(dxv, N) * N);
  if (length(T) < 1e-4) T = vec3(1.0, 0.0, 0.0);
  vec3 B = cross(N, T);
  outVs.rotMatrix = mat3(B * rescale, T * rescale, N);

  gl_Position = worldviewproj_matrix * P;

  // Bumpmap tiling. ×16 matches asv_wave_sim's reference scaling.
  // WaterVisual patches the bumpMap sampler with anisotropic
  // trilinear filtering at material-binding time, so the dense
  // tiling reads correctly at distance via mipmap LOD.
  const float bumpResolution = 16.0;
  outVs.bumpCoord = uv0.xy * bumpScale * bumpResolution + t * bumpSpeed;

  outVs.eyeVec = P.xyz - camera_position_object_space;
}
