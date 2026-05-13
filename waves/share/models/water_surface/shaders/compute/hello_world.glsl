// Copyright (C) 2026 Open Source Robotics Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
//
// Hello-world compute shader for the GPU-FFT pipeline. Writes a
// time-varying procedural pattern to an RGBA32F texture that the FFT
// vertex shader binds as `heightMap`. Used to validate Stage 1 of
// `docs/waves_gpu_fft_plan.md`: the compute dispatch runs every frame,
// the output texture is bound to the visual material, the visual
// receives time-varying displacement.
//
// Once Stage 1 is validated we replace this with the real
// Phillips + IFFT pipeline (Stages 2-3).

#version 430

// UAV slot 0 — matches the bridge's `_setUavTexture(0u, ...)` call.
layout(rgba32f, binding = 0) uniform writeonly image2D heightMapOut;

// std140-packed const buffer slot 0 — matches the bridge's
// `setConstBuffer(0u, paramsBuffer)` upload.
layout(std140, binding = 0) uniform Params
{
  float t;          // simulation time [s]
  float tileSize;   // physical tile size [m]
  int   gridSize;   // texture resolution per axis
  float _pad;
};

// 16×16 workgroup, one thread per texel of the heightmap.
layout(local_size_x = 16, local_size_y = 16, local_size_z = 1) in;

void main()
{
  ivec2 texel = ivec2(gl_GlobalInvocationID.xy);
  if (texel.x >= gridSize || texel.y >= gridSize)
    return;

  // Normalised position in [0, 1) across the tile, then in metres.
  vec2 uv  = (vec2(texel) + 0.5) / float(gridSize);
  vec2 pos = uv * tileSize;

  // Two-component sine pattern — a slow wave along x, a faster one
  // along y. Amplitude exaggerated (~3 m peaks) so it's unambiguously
  // visible from the default camera (15 m back, 2 m up), making it
  // easy to confirm the GPU compute path is producing data and the
  // visual is sampling it.
  const float kx = 0.05;       // [rad/m] → 125 m wavelength along x
  const float ky = 0.10;       // [rad/m] → 63 m wavelength along y
  const float wx = 0.5;        // [rad/s]
  const float wy = 0.7;        // [rad/s]

  float eta = 1.5 * sin(kx * pos.x - wx * t)
            + 1.5 * sin(ky * pos.y - wy * t);
  // Choppy displacement in phase with η so wave crests visibly sharpen.
  float Dx  = 0.6 * cos(kx * pos.x - wx * t);
  float Dy  = 0.6 * cos(ky * pos.y - wy * t);

  imageStore(heightMapOut, texel, vec4(eta, Dx, Dy, 0.0));
}
