// Copyright (C) 2026 Open Source Robotics Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
//
// Stage 4 of the GPU-FFT plan, first half: copy η and Dx out of the
// packed (η, η_im, Dx, Dx_im) IFFT output texture into the layout the
// vertex shader expects, (η, Dx, Dy, _). The companion shader
// combine_displacement_dy.glsl fills in .b with Dy in a second pass.
//
// We need two passes because OgreNext's OpenGL compute path can only
// bind one texture sampler reliably per dispatch (slot 1+ samplers
// silently return slot-0's data).
//
// Resource bindings:
//   UAV  slot 0 (image2D)    combinedTex   RGBA32F  output
//   Tex  slot 0 (sampler2D)  packedTex     RGBA32F  IFFT output
//                                                   (η in .r, Dx in .b)

#version 430

layout(rgba32f, binding = 0) uniform writeonly image2D combinedTex;
layout(binding = 0) uniform sampler2D packedTex;

layout(local_size_x = 16, local_size_y = 16, local_size_z = 1) in;

void main()
{
  ivec2 texel = ivec2(gl_GlobalInvocationID.xy);
  ivec2 size  = imageSize(combinedTex);
  if (texel.x >= size.x || texel.y >= size.y)
    return;
  vec4 src = texelFetch(packedTex, texel, 0);
  // src = (η_real, η_imag ≈ 0, Dx_real, Dx_imag ≈ 0).
  // Output layout matches the CPU upload path's (η, Dx, Dy, _).
  // Dy gets written by the second combine pass (RMW).
  imageStore(combinedTex, texel, vec4(src.r, src.b, 0.0, 0.0));
}
