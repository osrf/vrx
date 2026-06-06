# waves compute shaders

GLSL compute shaders for the GPU-FFT pipeline (see
`docs/waves_gpu_fft_plan.md`).

## Status

**Stage 1 — Ogre Next compute-shader infrastructure: WIRED, needs smoke test.**

What's now in place:

- `hello_world.glsl`: a GLSL 430 compute shader that writes a
  time-varying procedural displacement pattern to the heightmap
  texture.
- `Ogre2HeightMapBridge.cc`'s `waves_ogre2_heightmap_compute_dispatch`
  C-ABI entry: creates an `Ogre::HlmsComputeJob` on the first call,
  allocates a 16-byte `ConstBufferPacked` for runtime uniforms,
  registers the shader directory as a resource location, binds the
  heightmap as UAV slot 0, and dispatches the job each frame.
- `HeightMapTexture::Dispatch(...)`: thin C++ wrapper for the bridge
  C-ABI.
- `WaterVisual::OnSceneUpdate`: when `GZ_WAVES_GPU_FFT=1` is set in the
  environment AND the model SDF supplies `<gpu_compute>`, calls
  `Dispatch` instead of `Upload`. CPU IFFT is skipped on that path.
- `model.sdf`: a new `<gpu_compute>` element under `<shader>` lets the
  user pick the compute-shader source.

## How to test

```bash
# Run as usual but with GPU-FFT enabled:
GZ_WAVES_GPU_FFT=1 ros2 launch vrx_bringup simulation.launch.xml
```

Look for the log line:

```
[WaterVisual] GPU-FFT dispatch online — compute shader …/hello_world.glsl
```

Validation criteria:

1. A time-varying sinusoidal displacement pattern visibly applied to
   the water surface.
2. No `[WaterVisual] first FFT upload: …` line (CPU IFFT skipped).
3. Load time on the FFT path: if it's now ~5 s instead of ~2 min, the
   compute pipeline already bypasses the `HlmsLowLevel` slow init and
   Stage 6 (HlmsPbs migration) might be unnecessary.

## Next stages

- Stage 2: replace the procedural pattern with `phillips_init.glsl`
  (one-shot, builds `h0(k)`) plus `evolve.glsl` (per-frame, evolves to
  `h(k, t)`).
- Stage 3: `fft_butterfly.glsl`, a 2D inverse FFT in `log₂N` passes per
  axis.
- Stage 4: bridge's `_upload` becomes a no-op when GPU path is active.

## Notes for the implementer

The OgreNext compute API surface we're using:

- `Ogre::HlmsCompute::createComputeJob(jobName, refName, sourceFilename, includedPieces)`
- `Ogre::HlmsComputeJob::setNumThreadGroups(x, y, z)`
- `Ogre::HlmsComputeJob::setNumUavUnits(n)`
- `Ogre::HlmsComputeJob::_setUavTexture(slot, DescriptorSetUav::TextureSlot)`
- `Ogre::HlmsComputeJob::setConstBuffer(slot, ConstBufferPacked *)`
- `Ogre::VaoManager::createConstBuffer(size, bufferType, data, keepAsShadow)`
- `Ogre::HlmsCompute::dispatch(job, sceneManager, camera = nullptr)`

The GLSL uses standard OpenGL 4.30 syntax with explicit `layout(binding
= N)` qualifiers — we don't use HLMS template `@insertpiece` directives.
Reference: `gz_rendering_vendor`'s `ClearUav` compute shader at
`/opt/ros/rolling/opt/gz_rendering_vendor/share/gz/gz-rendering/ogre2/media/Compute/Tools/`
shows the HLMS-template form, but plain GLSL is sufficient for our case.
