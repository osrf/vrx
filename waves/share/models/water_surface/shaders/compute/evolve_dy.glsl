// Copyright (C) 2026 Open Source Robotics Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
//
// Companion of evolve.glsl. Writes the Tessendorf y-chop displacement
// spectrum Dy(k, t) = -i · (ky / |k|) · h(k, t) into hktTexDy. We
// can't pack a third complex signal into a single RGBA32F texel
// alongside (η, Dx), so Dy lives in its own texture and runs through
// its own IFFT pipeline. The butterfly itself is unchanged — it just
// happens to do 2 useful complex butterflies on (η, Dx) and 1 useful
// + 1 zero-padded on Dy.
//
// Resource bindings mirror evolve.glsl:
//   UAV  slot 0 (image2D)    hktTexDy  RGBA32F  output
//   Tex  slot 0 (sampler2D)  h0Tex     RGBA32F  read-only spectrum

#version 430

layout(rgba32f, binding = 0) uniform writeonly image2D hktTexDy;
layout(binding = 0) uniform sampler2D h0Tex;

layout(std140, binding = 0) uniform Params
{
  float t;
  float tau;
  int   gridSize;
  float tileSize;
};

layout(local_size_x = 16, local_size_y = 16, local_size_z = 1) in;

const float TWO_PI = 6.28318530717958647692;
const float G      = 9.81;

void main()
{
  ivec2 texel = ivec2(gl_GlobalInvocationID.xy);
  if (texel.x >= gridSize || texel.y >= gridSize)
    return;

  ivec2 specT = texel.yx;
  vec4 h0Pack = texelFetch(h0Tex, specT, 0);

  int ix = (texel.x < gridSize / 2) ? texel.x : texel.x - gridSize;
  int iy = (texel.y < gridSize / 2) ? texel.y : texel.y - gridSize;
  float kx = TWO_PI * float(ix) / tileSize;
  float ky = TWO_PI * float(iy) / tileSize;
  float kmag = sqrt(kx * kx + ky * ky);
  float omega = sqrt(G * kmag);

  float c = cos(omega * t);
  float s = sin(omega * t);

  float h_re = h0Pack.x * c - h0Pack.y * s
             + h0Pack.z * c + h0Pack.w * s;
  float h_im = h0Pack.x * s + h0Pack.y * c
             - h0Pack.z * s + h0Pack.w * c;

  float ramp = (tau > 0.0) ? (1.0 - exp(-t / tau)) : 1.0;
  h_re *= ramp;
  h_im *= ramp;

  // Dy(k, t) = -i · (ky / |k|) · h(k, t).
  float kyn = (kmag > 0.0) ? (ky / kmag) : 0.0;
  float dy_re =  kyn * h_im;
  float dy_im = -kyn * h_re;

  imageStore(hktTexDy, texel, vec4(dy_re, dy_im, 0.0, 0.0));
}
