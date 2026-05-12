// Copyright (C) 2026 Open Source Robotics Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
//
// Tessendorf ocean vertex shader. The heightmap texture is RGBA32F:
//   .r = η(x, y, t)           surface elevation
//   .g = Dx(x, y, t)          horizontal x-displacement
//   .b = Dy(x, y, t)          horizontal y-displacement
//   .a = (reserved)
// All four channels come from CPU-side IFFTs (see FFTWaveSimulation) and
// are uploaded once per simulation tick.
//
// Displacement model (Tessendorf 2001, eq. 29):
//   x' = x + chopFactor · Dx
//   y' = y + chopFactor · Dy
//   z' = z + η
// `chopFactor` is typically in [-2, -1]: negative values bunch particles
// toward wave crests, producing the sharp, asymmetric crests that
// distinguish a Tessendorf ocean from a pure height-field. The default of
// 0 falls back to a non-choppy surface, identical to the old r32f path.
//
// Surface normals are reconstructed with central differences against the
// displaced field, so the same shader handles chop=0 and chop≠0 without
// branching.

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
uniform float chopFactor;        // Tessendorf choppiness multiplier
uniform sampler2D heightMap;     // RGBA32F: (η, Dx, Dy, _)

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

// Apply the displacement field at world position `xy` and return the
// displaced 3D position. The startup ramp is already baked into the
// uploaded grids on the CPU side, so we don't double-multiply by ramp here.
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

  // Displaced position at the vertex.
  vec3 disp = SampleDisplaced(P.xy);
  P.xy = disp.xy;
  P.z += disp.z;

  // Finite-difference normal from neighbouring displaced positions. One
  // texel side in world space is (tileSize / gridSize) metres.
  float texel = tileSize / float(gridSize);
  vec3 px = SampleDisplaced(vertex.xy + vec2( texel, 0.0));
  vec3 nx = SampleDisplaced(vertex.xy + vec2(-texel, 0.0));
  vec3 py = SampleDisplaced(vertex.xy + vec2(0.0,  texel));
  vec3 ny = SampleDisplaced(vertex.xy + vec2(0.0, -texel));
  vec3 dxv = (px - nx) * 0.5;
  vec3 dyv = (py - ny) * 0.5;
  // dxv now holds (Δx', Δy', Δz') along the +x sampling axis (and similarly
  // for dyv). Cross product gives the surface normal.
  vec3 N = normalize(cross(dxv, dyv));
  // Match the existing fragment shader's expectation that +Z is "up";
  // flip if the cross product produced a downward-facing normal.
  if (N.z < 0.0) N = -N;

  // Tangent basis: T along the +x finite-difference direction (projected
  // onto the tangent plane), B = N × T.
  vec3 T = normalize(dxv - dot(dxv, N) * N);
  if (length(T) < 1e-4) T = vec3(1.0, 0.0, 0.0);
  vec3 B = cross(N, T);
  outVs.rotMatrix = mat3(B * rescale, T * rescale, N);

  gl_Position = worldviewproj_matrix * P;

  outVs.bumpCoord = uv0.xy * bumpScale + t * bumpSpeed;
  outVs.eyeVec = P.xyz - camera_position_object_space;
}
