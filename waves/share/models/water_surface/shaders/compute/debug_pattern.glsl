#version 430

// Debug pattern: writes a moving sine wave directly to a UAV texture,
// bypassing the IFFT pipeline. Used to isolate whether the
// binding+sampling path works end-to-end. If the visual shows visible
// waves with this shader bound, the bug is upstream (evolve/IFFT
// compute). If it's still flat, the bug is downstream (material
// binding, texture creation, sampling).

layout(rgba32f, binding = 0) uniform writeonly image2D dst;

layout(std140, binding = 0) uniform Params
{
  float t;
  float amplitude;
  int   gridSize;
  int   _pad;
};

layout(local_size_x = 16, local_size_y = 16, local_size_z = 1) in;

void main()
{
  ivec2 texel = ivec2(gl_GlobalInvocationID.xy);
  if (texel.x >= gridSize || texel.y >= gridSize)
    return;

  // A coherent 2D sine in spatial coordinates that drifts with t.
  // 4 wavelengths across the tile so peaks are clearly visible.
  float fx = float(texel.x) / float(gridSize);
  float fy = float(texel.y) / float(gridSize);
  float phase = 8.0 * 3.14159265 * (fx + 0.5 * fy) + 0.5 * t;
  float eta = amplitude * sin(phase);

  // Match the IFFT output's layout: .x = η. The visual's FFT shader
  // reads .r as the height. .yzw stay zero so chop is also zero.
  imageStore(dst, texel, vec4(eta, 0.0, 0.0, 0.0));
}
