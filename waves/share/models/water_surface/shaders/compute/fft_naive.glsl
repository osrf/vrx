#version 430

// Naive O(N²) 2D IFFT in a single dispatch. Used as a reference
// implementation to verify the radix-2 Cooley-Tukey pipeline.
//
// For each output spatial position (x, y):
//   out[x, y] = (1/N²) Σ_kx Σ_ky in[kx, ky] · exp(+i · 2π · (kx·x + ky·y) / N)
//
// At N=128 that's 16384 inner-loop iterations per output texel,
// 16384 texels, so ~270 M ops/frame. Slow but trivial for a modern
// GPU. We don't care about performance here — only correctness.

layout(rgba32f, binding = 0) uniform writeonly image2D dst;
layout(binding = 0) uniform sampler2D src;

layout(std140, binding = 0) uniform Params
{
  int gridSize;
  int _pad0;
  int _pad1;
  int _pad2;
};

layout(local_size_x = 16, local_size_y = 16, local_size_z = 1) in;

const float TWO_PI = 6.28318530717958647692;

void main()
{
  ivec2 t = ivec2(gl_GlobalInvocationID.xy);
  if (t.x >= gridSize || t.y >= gridSize)
    return;

  vec2 sum = vec2(0.0);
  // Sum over all spectrum cells (kx, ky).
  for (int ky = 0; ky < gridSize; ++ky)
  {
    for (int kx = 0; kx < gridSize; ++kx)
    {
      vec2 inK = texelFetch(src, ivec2(kx, ky), 0).xy;
      float angle = TWO_PI * float(kx * t.x + ky * t.y) /
                    float(gridSize);
      float c = cos(angle);
      float s = sin(angle);
      // inK * exp(+i·angle):
      sum.x += inK.x * c - inK.y * s;
      sum.y += inK.x * s + inK.y * c;
    }
  }

  // 1/N² normalisation, matching the CPU's Eigen::FFT::inv() applied
  // row-then-column (each divides by N).
  sum /= float(gridSize * gridSize);

  imageStore(dst, t, vec4(sum, 0.0, 0.0));
}
