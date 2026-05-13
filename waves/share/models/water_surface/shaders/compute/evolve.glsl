// Copyright (C) 2026 Open Source Robotics Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
//
// Stage 2 of the GPU-FFT plan: time-evolve the Phillips spectrum on the
// GPU. Computes h(k, t) = h0(k)·exp(i·ω·t) + conj(h0(-k))·exp(-i·ω·t)
// for every cell of the spectrum grid. The result is the time-domain
// frequency-space amplitude that Stage 3's IFFT will transform into the
// spatial heightmap.
//
// Resource bindings:
//   UAV  slot 0 (image2D)    hktTex   RGBA32F  output
//   Tex  slot 0 (sampler2D)  h0Tex    RGBA32F  read-only spectrum
//   Tex  slot 1 (sampler2D)  omegaTex R32F     read-only frequencies
//
// We deliberately put inputs on TEXTURE slots, not UAV slots — in
// OgreNext's OpenGL compute path, UAVs and textures share slots, and
// reading from `image2D` at UAV slot 1+ in a multi-UAV job silently
// returns zero. Texture-sampler binding works correctly. We use
// `texelFetch` (no filtering, no LOD) so the read is equivalent to
// `imageLoad`.
//
// Workgroup is 16×16 → one thread per spectrum cell.

#version 430

layout(rgba32f, binding = 0) uniform writeonly image2D hktTex;
layout(binding = 0) uniform sampler2D h0Tex;
layout(binding = 1) uniform sampler2D omegaTex;

layout(std140, binding = 0) uniform Params
{
  float t;          // simulation time [s]
  float _pad0;
  int   gridSize;   // texture resolution per axis
  int   _pad1;
};

layout(local_size_x = 16, local_size_y = 16, local_size_z = 1) in;

void main()
{
  ivec2 texel = ivec2(gl_GlobalInvocationID.xy);
  if (texel.x >= gridSize || texel.y >= gridSize)
    return;

  vec4 h0Pack = texelFetch(h0Tex, texel, 0);   // (re_h0, im_h0, re_conj, im_conj)
  float omega = texelFetch(omegaTex, texel, 0).r;

  float c = cos(omega * t);
  float s = sin(omega * t);

  // Let h0      = (h0.re, h0.im)
  //     h0conj  = (conj.re, conj.im)
  //     e_plus  = (c,  s)   = exp(+i·ω·t)
  //     e_minus = (c, -s)   = exp(-i·ω·t)
  //
  // h(k,t) = h0·e_plus + h0conj·e_minus
  //        = (h0.re*c - h0.im*s + conj.re*c + conj.im*s,
  //           h0.re*s + h0.im*c - conj.re*s + conj.im*c)
  float h_re = h0Pack.x * c - h0Pack.y * s
             + h0Pack.z * c + h0Pack.w * s;
  float h_im = h0Pack.x * s + h0Pack.y * c
             - h0Pack.z * s + h0Pack.w * c;

  imageStore(hktTex, texel, vec4(h_re, h_im, 0.0, 0.0));
}
