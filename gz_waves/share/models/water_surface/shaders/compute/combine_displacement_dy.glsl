// Copyright (C) 2026 Open Source Robotics Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
//
// Second half of the displacement combine. Reads Dy from the separate
// IFFT output and writes it into the .b channel of `combinedTex` while
// preserving .r (η) and .g (Dx) that the first pass already wrote.
//
// Resource bindings:
//   UAV  slot 0 (image2D)    combinedTex      RGBA32F  output (RMW)
//   Tex  slot 0 (sampler2D)  dyPackedTex      RGBA32F  Dy IFFT output
//                                                      (Dy in .r)

#version 430

layout(rgba32f, binding = 0) uniform image2D combinedTex;
layout(binding = 0) uniform sampler2D dyPackedTex;

layout(local_size_x = 16, local_size_y = 16, local_size_z = 1) in;

void main()
{
  ivec2 texel = ivec2(gl_GlobalInvocationID.xy);
  ivec2 size  = imageSize(combinedTex);
  if (texel.x >= size.x || texel.y >= size.y)
    return;
  vec4 existing = imageLoad(combinedTex, texel);
  float dy = texelFetch(dyPackedTex, texel, 0).r;
  imageStore(combinedTex, texel, vec4(existing.r, existing.g, dy, 0.0));
}
