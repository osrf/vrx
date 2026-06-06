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
// Output packs two complex signals per RGBA32F texel:
//   .rg = h(k, t)            for the height field η
//   .ba = Dx(k, t)           for the Tessendorf x-chop displacement
// Dx(k, t) = -i · (kx / |k|) · h(k, t). The y-chop Dy is computed by a
// separate evolve_dy dispatch into its own hktTex so the radix-2
// butterfly (which carries 2 complex per texel) only has to be invoked
// twice per frame: once for the packed (η, Dx) and once for Dy.
//
// Resource bindings:
//   UAV  slot 0 (image2D)    hktTex   RGBA32F  output
//   Tex  slot 0 (sampler2D)  h0Tex    RGBA32F  read-only spectrum
//
// We deliberately put inputs on TEXTURE slots, not UAV slots — in
// OgreNext's OpenGL compute path, UAVs and textures share slots, and
// reading from `image2D` at UAV slot 1+ in a multi-UAV job silently
// returns zero.
//
// IMPORTANT: TEXTURE samplers ALSO don't bind reliably at slot 1+ in
// OgreNext's compute path — a sampler declared at `binding = 1`
// silently returns slot-0's texture's data. So we only bind h0Tex at
// slot 0 and compute ω from the cell index on the fly instead of
// reading it from an omegaTex texture. (The omegaGrid upload still
// happens — and the bridge's omega readback is bit-exact CPU↔GPU —
// but evolve doesn't actually sample it.)
//
// Workgroup is 16×16 → one thread per spectrum cell.

#version 430

layout(rgba32f, binding = 0) uniform writeonly image2D hktTex;
layout(binding = 0) uniform sampler2D h0Tex;

layout(std140, binding = 0) uniform Params
{
  float t;          // simulation time [s]
  float tau;        // ramp time constant (matches CPU FFTWaveSimulation)
  int   gridSize;   // N (texture resolution per axis)
  float tileSize;   // L (physical tile extent in meters)
};

layout(local_size_x = 16, local_size_y = 16, local_size_z = 1) in;

const float TWO_PI = 6.28318530717958647692;
const float G      = 9.81;

void main()
{
  ivec2 texel = ivec2(gl_GlobalInvocationID.xy);
  if (texel.x >= gridSize || texel.y >= gridSize)
    return;

  // h0Tex is uploaded so pixel (col=j, row=i) = h0(i, j). To make
  // thread (x, y) process the spectrum at index (x, y) — frequency
  // (kx[x], ky[y]) — we read pixel (col=y, row=x) via texel.yx.
  ivec2 specT = texel.yx;
  vec4 h0Pack = texelFetch(h0Tex, specT, 0);   // (re_h0, im_h0, re_conj, im_conj)

  // Compute ω(k) = sqrt(g · |k|) from the cell's FFT-layout index.
  // kx[i] = (2π/L) · (i if i<N/2 else i-N); same for ky[j].
  int ix = (texel.x < gridSize / 2) ? texel.x : texel.x - gridSize;
  int iy = (texel.y < gridSize / 2) ? texel.y : texel.y - gridSize;
  float kx = TWO_PI * float(ix) / tileSize;
  float ky = TWO_PI * float(iy) / tileSize;
  float kmag = sqrt(kx * kx + ky * ky);
  float omega = sqrt(G * kmag);

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

  // Startup ramp, matches CPU FFTWaveSimulation. Multiplied into the
  // spectrum here (linear → equivalent to multiplying η by ramp on
  // the spatial side, like CPU's Ifft2DReal(hkt, ramp)).
  float ramp = (tau > 0.0) ? (1.0 - exp(-t / tau)) : 1.0;
  h_re *= ramp;
  h_im *= ramp;

  // Dx(k, t) = -i · (kx / |k|) · h(k, t).
  // -i · (a + i·b) = b - i·a, so Dx = (kxn·h_im, -kxn·h_re).
  // At kmag = 0 the displacement vanishes (DC mode).
  float kxn = (kmag > 0.0) ? (kx / kmag) : 0.0;
  float dx_re =  kxn * h_im;
  float dx_im = -kxn * h_re;

  imageStore(hktTex, texel, vec4(h_re, h_im, dx_re, dx_im));
}
