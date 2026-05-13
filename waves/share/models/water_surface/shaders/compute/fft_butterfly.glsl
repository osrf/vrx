#version 430

// One Cooley-Tukey radix-2 butterfly stage along one axis. After
// log2(N) butterfly stages — preceded by a bit-reverse pass — the
// 1D FFT/IFFT of that axis is complete. Doing it once along each
// axis yields the 2D FFT/IFFT.
//
// The data layout is two complex values per RGBA32F texel: xy carries
// the first signal (η for the packed pipeline, Dy for the Dy
// pipeline), zw carries the second signal (Dx for the packed pipeline,
// or zero padding for Dy). Both pairs run through identical butterfly
// arithmetic so a single dispatch produces both 2D IFFTs.
//
// invertSign selects the direction: +1.0 for IFFT (used here, sign in
// the wave equation is exp(+iωt)), -1.0 for forward FFT. We do NOT
// apply the 1/N normalisation — the CPU FFTWaveSimulation bakes the
// scale into h0 (matching Eigen/KissFFT's unnormalised convention), so
// the GPU output reproduces the same physical amplitude.

layout(rgba32f, binding = 0) uniform writeonly image2D dst;
layout(binding = 0) uniform sampler2D src;

layout(std140, binding = 0) uniform Params
{
  int   gridSize;     // N
  int   stage;        // 0..log2(N)-1
  int   axis;         // 0 = row, 1 = column
  float invertSign;   // +1 for IFFT, -1 for forward FFT
  float extraScale;   // multiplier applied to the output (1/N on the
                      // last stage of each axis → 1/N² total for the
                      // 2D IFFT, matching Eigen::FFT::inv()'s default
                      // normalisation). 1.0 on all other stages.
  float _pad0;
  float _pad1;
  float _pad2;
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

  vec4 aVal = texelFetch(src, aT, 0);
  vec4 bVal = texelFetch(src, bT, 0);

  // Two independent complex butterflies per texel: signal 1 in xy,
  // signal 2 in zw. tw_b1 = tw * b1, tw_b2 = tw * b2.
  vec2 tw_b1 = vec2(bVal.x * tw.x - bVal.y * tw.y,
                    bVal.x * tw.y + bVal.y * tw.x);
  vec2 tw_b2 = vec2(bVal.z * tw.x - bVal.w * tw.y,
                    bVal.z * tw.y + bVal.w * tw.x);

  vec2 r1 = isLower ? (aVal.xy + tw_b1) : (aVal.xy - tw_b1);
  vec2 r2 = isLower ? (aVal.zw + tw_b2) : (aVal.zw - tw_b2);

  imageStore(dst, t, vec4(r1, r2) * extraScale);
}
