// Copyright (C) 2026 Open Source Robotics Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
//
// Hello-world compute shader for the GPU-FFT pipeline. Writes a
// deterministic procedural pattern to an RGBA32F texture that the FFT
// vertex shader binds as `heightMap`. The pattern moves with simulation
// time so we can confirm: (a) the compute dispatch runs every frame,
// (b) the output texture is correctly bound to the visual material,
// (c) the visual receives time-varying displacement.
//
// Once Stage 1 is validated we replace this with the real Phillips +
// IFFT pipeline (Stages 2-3).
//
// Layout: 16×16 workgroup (256 threads), one thread per texel.

#version 430

// `image2D` (vs `sampler2D`) lets us WRITE to the texture. Format must
// match the TextureGpu's pixel format (PFG_RGBA32_FLOAT).
layout(rgba32f, binding = 0) uniform image2D heightMapOut;

uniform float t;          // simulation time [s]
uniform int   gridSize;   // texture resolution per axis (e.g. 128)
uniform float tileSize;   // physical tile size [m]

layout(local_size_x = 16, local_size_y = 16, local_size_z = 1) in;

void main()
{
  ivec2 texel = ivec2(gl_GlobalInvocationID.xy);
  if (texel.x >= gridSize || texel.y >= gridSize)
    return;

  // Normalised position in [0, 1) across the tile.
  vec2 uv = (vec2(texel) + 0.5) / float(gridSize);

  // World-space position in metres for the centre of this texel.
  vec2 pos = uv * tileSize;

  // Two-component sine pattern — a slow "wave" along x, a faster one
  // along y. ~1 m amplitude so it's clearly visible.
  float kx = 0.05;  // [rad/m]
  float ky = 0.10;
  float omega_x = 0.5;  // [rad/s]
  float omega_y = 0.7;

  float eta = 0.5 * sin(kx * pos.x - omega_x * t)
            + 0.5 * sin(ky * pos.y - omega_y * t);

  // Choppy displacement: small horizontal motion in phase with η.
  float Dx = 0.2 * cos(kx * pos.x - omega_x * t);
  float Dy = 0.2 * cos(ky * pos.y - omega_y * t);

  imageStore(heightMapOut, texel, vec4(eta, Dx, Dy, 0.0));
}
