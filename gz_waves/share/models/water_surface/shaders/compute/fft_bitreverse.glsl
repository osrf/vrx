#version 430

// Bit-reverse permutation along one axis. For each output texel,
// reads from the source texel whose coordinate along the active axis
// has its low log2N bits reversed. This is the canonical pre-pass for
// a decimation-in-time Cooley-Tukey FFT/IFFT: after bit-reversal the
// log2N butterfly stages of fft_butterfly.glsl execute in-order with
// adjacent-pair access.
//
// Run once per axis: axis=0 reverses x for the row pass, axis=1
// reverses y for the column pass.

layout(rgba32f, binding = 0) uniform writeonly image2D dst;
layout(binding = 0) uniform sampler2D src;

layout(std140, binding = 0) uniform Params
{
  int gridSize;   // N (power of two)
  int axis;       // 0 = row (reverse x), 1 = column (reverse y)
  int log2N;      // log2(gridSize)
  int _pad;
};

layout(local_size_x = 16, local_size_y = 16, local_size_z = 1) in;

void main()
{
  ivec2 t = ivec2(gl_GlobalInvocationID.xy);
  if (t.x >= gridSize || t.y >= gridSize)
    return;

  uint k  = uint((axis == 0) ? t.x : t.y);
  uint kr = bitfieldReverse(k) >> uint(32 - log2N);

  ivec2 srcT = (axis == 0)
      ? ivec2(int(kr), t.y)
      : ivec2(t.x,     int(kr));

  imageStore(dst, t, texelFetch(src, srcT, 0));
}
