# waves compute shaders

GLSL compute shaders for the GPU-FFT pipeline (see
`docs/waves_gpu_fft_plan.md`).

## Status

**Stage 1 — Ogre Next compute-shader infrastructure: IN PROGRESS.**

The `hello_world.glsl` compute shader is in place. The bridge entry
point that creates an `Ogre::HlmsComputeJob`, binds the heightmap
texture as UAV slot 0, allocates a `ConstBufferPacked` for the runtime
uniforms (`t`, `tileSize`, `gridSize`), and dispatches the job each
frame is **not yet wired**. That's the bulk of Stage 1 and needs a
focused implementation session.

When it's wired:

1. The bridge adds a `waves_ogre2_heightmap_compute_dispatch` C-ABI
   entry that takes a shader filename + sim time and dispatches the
   compute job.
2. `WaterVisual::OnSceneUpdate` calls the dispatch instead of the
   CPU `Upload` when `GZ_WAVES_GPU_FFT=1` is set.
3. The procedural displacement pattern in `hello_world.glsl` appears
   on the water surface and moves with time.

Validation criteria for declaring Stage 1 done:

- Pattern visibly applied to the water surface, time-varying.
- Pattern is regenerated each frame via compute (no CPU upload).
- Smoke-test the FFT load time on Jetty + Blackwell. If load is fast
  (~5 s), the compute path already bypasses the `HlmsLowLevel` slow
  init and Stage 6 (HlmsPbs migration) may be unnecessary.

## Notes for the implementer

Useful reference: `gz_rendering_vendor`'s `ClearUav` compute shader at
`/opt/ros/rolling/opt/gz_rendering_vendor/share/gz/gz-rendering/ogre2/media/Compute/Tools/`
shows the standard OgreNext compute-shader template format with HLMS
piece substitution. We don't need full HLMS templating for our case —
plain GLSL 430 with explicit `layout(rgba32f, binding=0) uniform
image2D` is sufficient.

For runtime uniforms (t, tileSize, gridSize), allocate one
`ConstBufferPacked` via `vaoManager->createConstBuffer(16, BT_DYNAMIC_PERSISTENT_COHERENT, nullptr, false)`,
update its contents each frame, and bind via
`job->setConstBuffer(0, buf)`. The shader declares it as a
`layout(std140, binding = 0) uniform Params { ... }`.
