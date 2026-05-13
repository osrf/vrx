#version 430

// One Cooley-Tukey radix-2 butterfly stage along one axis. After
// log2(N) butterfly stages — preceded by a bit-reverse pass — the
// 1D FFT/IFFT of that axis is complete. Doing it once along each
// axis yields the 2D FFT/IFFT.
//
// The data layout is (real, imag, _, _) per RGBA32F texel: the FFT is
// run on the complex values in xy; zw are unused (reserved for a future
// 2-spectrum-per-texel packing that would let us IFFT η and Dx (or Dy)
// together for the same dispatch cost).
//
// invertSign selects the direction: +1.0 for IFFT (used here, sign in
// the wave equation is exp(+iωt)), -1.0 for forward FFT. We do NOT
// apply the 1/N normalisation — the CPU FFTWaveSimulation bakes the
// scale into h0 (matching Eigen/KissFFT's unnormalised convention), so
// the GPU output reproduces the same physical amplitude.

layout(rgba32f, binding = 0) uniform readonly  image2D src;
layout(rgba32f, binding = 1) uniform writeonly image2D dst;

layout(std140, binding = 0) uniform Params
{
  int   gridSize;     // N
  int   stage;        // 0..log2(N)-1
  int   axis;         // 0 = row, 1 = column
  float invertSign;   // +1 for IFFT, -1 for forward FFT
};

layout(local_size_x = 16, local_size_y = 16, local_size_z = 1) in;

const float TWO_PI = 6.28318530717958647692;

void main()
{
  ivec2 t = ivec2(gl_GlobalInvocationID.xy);
  if (t.x >= gridSize || t.y >= gridSize)
    return;

  // Coordinates along the active axis and the orthogonal axis.
  int k    = (axis == 0) ? t.x : t.y;
  int orth = (axis == 0) ? t.y : t.x;

  int blockSize = 1 << (stage + 1);  // 2^(s+1) — size of one butterfly block
  int halfBlock = 1 << stage;        // 2^s     — size of each half

  int kInBlock   = k & (blockSize - 1);
  int blockStart = k - kInBlock;
  int pairIdx    = kInBlock & (halfBlock - 1);
  bool isLower   = (kInBlock < halfBlock);

  // Twiddle factor: exp(invertSign · i · 2π · pairIdx / blockSize).
  float angle = invertSign * TWO_PI * float(pairIdx) / float(blockSize);
  vec2 tw     = vec2(cos(angle), sin(angle));

  // The pair this output texel combines is (aPos, bPos) along the axis,
  // both at the same `orth` coordinate.
  int aPos = blockStart + pairIdx;
  int bPos = aPos + halfBlock;
  ivec2 aT = (axis == 0) ? ivec2(aPos, orth) : ivec2(orth, aPos);
  ivec2 bT = (axis == 0) ? ivec2(bPos, orth) : ivec2(orth, bPos);

  vec4 aVal = imageLoad(src, aT);
  vec4 bVal = imageLoad(src, bT);

  // Complex multiply: tw_b = tw * b (b in bVal.xy).
  vec2 tw_b = vec2(bVal.x * tw.x - bVal.y * tw.y,
                   bVal.x * tw.y + bVal.y * tw.x);

  vec2 result = isLower ? (aVal.xy + tw_b)
                        : (aVal.xy - tw_b);

  imageStore(dst, t, vec4(result, 0.0, 0.0));
}
