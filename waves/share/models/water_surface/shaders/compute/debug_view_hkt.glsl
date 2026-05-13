#version 430

// Diagnostic: reads hktTex (Stage 2 evolve's output, h(k, t) in the
// frequency domain) and writes a scaled version of its real channel
// directly to ifftFinalTex. Skips the IFFT entirely.
//
// If the visual shows non-zero patterns with this shader bound, the
// evolve stage IS producing data and the IFFT is the broken link.
// If still flat, the evolve stage (or h0 upload) is the broken link.
//
// Note: hktTex contains frequency-domain data, NOT spatial waves —
// the displayed pattern will look like discrete bright spots / noise,
// not smooth ocean waves. We're just looking for "any non-zero
// activity" as a smoke test.

layout(rgba32f, binding = 0) uniform writeonly image2D dst;
layout(binding = 0) uniform sampler2D src;

layout(std140, binding = 0) uniform Params
{
  float scale;       // multiplier applied to src.x (e.g. 0.01)
  float _pad0;
  int   gridSize;
  int   _pad1;
};

layout(local_size_x = 16, local_size_y = 16, local_size_z = 1) in;

void main()
{
  ivec2 texel = ivec2(gl_GlobalInvocationID.xy);
  if (texel.x >= gridSize || texel.y >= gridSize)
    return;

  vec4 s = texelFetch(src, texel, 0);
  // |h(k,t)| (magnitude) is the most useful single-channel signal —
  // a non-zero spectrum should show a recognisable, bright pattern.
  float mag = length(s.xy);
  imageStore(dst, texel, vec4(scale * mag, 0.0, 0.0, 0.0));
}
