# GPU-FFT plan for VRX waves

## Context

This document is the follow-up plan to `waves_integration_plan.md`. The
CPU-FFT implementation on `vrx4-waves-fft` works correctly but has a
~2-minute first-frame stall on Ubuntu 24.04 + NVIDIA Blackwell + locally
built Gazebo Jetty + system OgreNext 2.3.1, traced (after extensive
investigation, see `waves/README.md`) to the first-render-of-an-
`HlmsLowLevel`-material-with-custom-GLSL code path inside OgreNext.

asv_wave_sim against Gazebo Harmonic on the same hardware renders waves
in seconds. Two architectural fixes are available:

1. **asv_wave_sim port** — clean-room reimplementation of their custom
   `RenderEngineExtension` + `Ogre2OceanVisual` + `Ogre2DynamicMesh`
   pattern (~4500 LOC, 5-10 working days). Verified to work on this
   hardware. Bypasses `HlmsLowLevel` for the visual by attaching the
   material to a custom Ogre Renderable via `SubItem::setDatablock`.

2. **GPU FFT** (this plan) — move the FFT itself to a compute shader on
   the GPU, eliminating both the per-frame CPU→GPU upload and (via a
   migration to `HlmsPbs` with custom piece hooks) the slow
   `HlmsLowLevel` first-render path. ~7-14 working days. Higher upside
   long-term (no CPU FFT cost, scales to 256²/512² grids cheaply,
   architecturally cleaner for upstream) but more unknowns.

## Architecture overview

```
                      ┌─────────────────────────────────────────────────┐
                      │                    GPU                          │
                      │                                                 │
                      │   ┌────────────┐    ┌────────────────────────┐  │
init once ────────────┼──▶│   h0(k)    │    │      η(x,y,t),         │  │
(Phillips, Gaussian)  │   │  RGBA32F   │    │      Dx(x,y,t),        │  │
                      │   │  (static)  │    │      Dy(x,y,t)         │  │
                      │   └─────┬──────┘    │   in RGBA32F texture   │  │
                      │         │           │  (sampled by visual)   │  │
                      │  ┌──────▼──────┐    └─────────────▲──────────┘  │
                      │  │  evolve.cs  │                  │             │
each frame ───────────┼─▶│  h(k, t) =  ├──┐               │             │
(t uniform)           │  │  h0·e^iωt + │  │               │             │
                      │  │  conj·e^-iωt│  │               │             │
                      │  └─────────────┘  │               │             │
                      │                   │               │             │
                      │                   ▼               │             │
                      │            ┌─────────────┐        │             │
                      │            │ fft_butter- │        │             │
                      │            │ fly.cs      │────────┘             │
                      │            │ (2·log₂N    │                      │
                      │            │  dispatches)│                      │
                      │            └─────────────┘                      │
                      │                                                 │
                      └─────────────────────────────────────────────────┘
                                            ▲
                                            │
                                            │ Bound as a regular texture
                                            │ on an HlmsPbs material's
                                            │ vertex displacement hook
                                            ▼
                                  Standard Ogre HlmsPbs pipeline
                                  (no HlmsLowLevel = no slow path)
```

CPU side keeps a parallel `FFTWaveSimulation` used only by
`WaveBuoyancy` — physics sampling at 30 Hz doesn't justify GPU readback
overhead, and the two simulations stay in sync via the shared
`<seed>` in SDF.

## Stages

Each stage produces a working, commit-able milestone. Same staging
discipline as the original CPU-FFT plan.

### Stage 1 — Ogre Next compute-shader infrastructure

**Deliverable.** A "hello world" compute shader writes a simple
deterministic pattern (e.g. `(sin(2π·x/W), cos(2π·y/H), 0, 0)`) into an
`Ogre::TextureGpu` of `PFG_RGBA32_FLOAT`, dispatched once per frame from
the render thread. The existing FFT vertex shader samples it as
`heightMap`. The standard textured-plane visual moves with this pattern
visibly applied as a displacement.

**Files.**
- `waves/src/systems/Ogre2HeightMapCompute.{hh,cc}` (new, in the bridge
  library — handles `Ogre::HlmsCompute` + `Ogre::HlmsComputeJob` setup).
- `waves/share/models/water_surface/shaders/compute/hello_world.glsl`
  (new GLSL compute shader).
- `waves/src/systems/WaterVisual.cc` — replace the CPU `Upload` call
  with a `Dispatch` call into the bridge.
- Bridge C-ABI gets a new entry: `waves_ogre2_compute_dispatch(handle)`.

**Validation.**
- Procedurally generated displacement visibly applied.
- Smoke test: launch time on FFT path. Two outcomes:
  - **Fast (~5 s):** the compute-shader → texture → HlmsLowLevel-vertex-shader path
    avoids the stall. Stages 2-3 just need to replace the pattern with real wave physics
    and we are done modulo Stage 5 physics readback.
  - **Still ~2 min:** the stall is in `HlmsLowLevel` regardless of how the texture is
    sourced. Stage 6 (HlmsPbs migration) is mandatory.

**Risk.** Compute-shader API examples in the gz-rendering / OgreNext
2.3.x ecosystem are sparse. Most Ogre Next compute-shader docs target
Vulkan; we're on GL. A few days of API ramp-up is expected.

### Stage 2 — Phillips spectrum on GPU

**Deliverable.** `h₀(k)` is generated by a compute shader at startup
into a persistent RGBA32F texture (real/imag in two channels). A second
compute shader runs each frame to produce `h(k, t)` in a temporary
RGBA32F texture. The spatial visual is still driven by Stage 1's hello-
world pattern; what we're validating here is the spectral data.

**Files.**
- `waves/share/models/water_surface/shaders/compute/phillips_init.glsl`
  (Phillips spectrum + Gaussian RNG, dispatched once).
- `waves/share/models/water_surface/shaders/compute/evolve.glsl`
  (`h(k,t) = h₀·exp(iω·t) + conj(h₀(-k))·exp(-iω·t)`, dispatched per
  frame).
- Persistent GPU resources owned by the bridge: `h0Tex`, `omegaTex`,
  `hktTex`.

**Validation.** Unit test compares GPU `h(k, t)` against the existing
CPU `FFTWaveSimulation`'s `hkt` matrix for fixed seed + time. Should
agree bit-for-bit up to float rounding. Add to existing `fft_test`.

### Stage 3 — IFFT on GPU

**Deliverable.** A 2D inverse FFT compute pipeline running on the GPU.
Output is real-valued η(x, y, t) in an RGBA32F texture, with optional
extra channels for Dx, Dy (computed by multiplying the spectrum by
`-i·k/|k|` before the IFFT — same trick as the CPU path).

**Approach.** Cooley-Tukey radix-2, `log₂N` passes per axis, `2·log₂N`
total compute dispatches per frame. For N=128 this is 14 small
dispatches — well within budget. Compute-side input/output ping-pong
between two textures.

**Files.**
- `waves/share/models/water_surface/shaders/compute/fft_butterfly.glsl`
  (parameterised by stage index + axis + direction).
- A bit-reversal lookup texture for the initial reorder.
- Optional: `glsl-fft`-style helper headers if we find an Apache-2 or
  BSD reference implementation worth adapting clean-room.

**Validation.**
- Unit test compares GPU η, Dx, Dy against Eigen IFFT output for the
  same spectrum + time. Acceptance threshold: 1e-5 RMSE.
- σ_η of the GPU output should match the analytic Phillips integral for
  given wind speed (sanity check on amplitude).

### Stage 4 — Drop the CPU→GPU upload

**Deliverable.** `WaterVisual::OnSceneUpdate` no longer calls
`fftSim->Update + heightMap->Upload`. The visual binds the GPU-FFT
output texture directly via the existing material → texture-unit path
in the bridge. Per-frame CPU FFT cost on the GUI side is zero.

**Files.**
- `waves/src/systems/WaterVisual.cc`: gate the FFT branch on whether
  GPU compute is available; in that branch, dispatch the compute job
  and skip the upload.
- Bridge: stop allocating the staging texture. The pass binds the
  compute-output texture (already on GPU) directly to the material's
  `heightMap` texture unit.

**Validation.** Same visual quality as Stage 5 of the CPU-FFT plan.

### Stage 5 — Physics readback / CPU FFT for buoyancy

**Deliverable.** `WaveBuoyancy` continues to work. Three options
discussed:

- **A.** Keep `FFTWaveSimulation` on the CPU server-side just for
  buoyancy sampling. Same seed → bit-for-bit identical wave field as
  GPU. Effort: zero (status quo). Cost: still 1 CPU FFT/frame at 30 Hz.
- **B.** Read the GPU heightmap back to a CPU buffer per physics tick
  (30 Hz × 256 KB = 7.7 MB/s, ~1 ms readback stall per tick).
- **C.** Full GPU physics path. Out of scope.

**Recommendation: A** for the first ship. CPU FFT at 128² is ~1 ms;
fine for 30 Hz physics. Revisit if grid resolution scales to ≥512².

### Stage 6 — HlmsPbs migration for the visual

**Deliverable.** The water material is no longer a custom-GLSL
`HlmsLowLevel` material; it's an `HlmsPbsDatablock` with custom *piece*
hooks that inline vertex-displacement code into the standard PBS vertex
shader. This is what's expected to eliminate the slow first-render
stall on Jetty + Blackwell.

If Stage 1 already loaded fast, this stage may be unnecessary and can
be skipped. If Stage 1 still showed the ~2 min stall, this is the
load-time fix.

**Files.**
- `waves/share/models/water_surface/hlms/WavesPbsPiece_vs.any` — Ogre
  Next *piece* file inlined into the standard PBS vertex shader at the
  `custom_vs_posExecution` hook.
- `waves/src/systems/Ogre2HeightMapBridge.cc` — switch material setup
  from `Ogre::v1::MaterialPtr` configured via `gz::rendering::Material`
  to `Ogre::HlmsPbsDatablock` configured via `HlmsPbs` directly.
- `waves/src/systems/WaterVisual.cc` — stop calling `SetVertexShader` /
  `SetFragmentShader`; configure datablock directly through the bridge.

**Validation.** Measure GUI load time. Target: ≤5 s, comparable to
Gerstner.

### Stage 7 — Optional polish

Things we can do once the core works:

- Bump grid resolution to 256² or 512² (cheap on GPU, expensive on CPU).
- Add CPU-readable foam mask from the Jacobian determinant of the
  displacement field, packed into the alpha channel.
- Multi-tile rendering with different wave parameters per tile, for
  large competition courses (asv_wave_sim does this).
- Vulkan backend (Ogre Next 2.3 supports both GL and Vulkan; we'd
  benefit on the Vulkan path).

## Effort summary

| Stage | Optimistic | Realistic | Worst case |
|---|---|---|---|
| 1. Compute infra + hello world | 1 d | 2 d | 3 d |
| 2. Phillips on GPU | 1 d | 1 d | 2 d |
| 3. IFFT on GPU | 3 d | 4 d | 7 d |
| 4. Drop CPU upload | 0.5 d | 1 d | 2 d |
| 5. Physics readback (option A: zero work) | 0 d | 0.5 d | 1 d |
| 6. HlmsPbs migration | 1 d | 3 d | 5 d |
| **Total** | **6.5 d** | **11.5 d** | **20 d** |

Comparable to the asv_wave_sim port (~5-10 working days) with higher
upside (no CPU FFT cost, scales to higher resolution, cleaner for
upstream) and higher risk (Ogre Next compute-shader docs are sparse,
HlmsPbs piece hooks less well-trodden).

## Branch strategy

- **`vrx4-waves-fft`** — current CPU-FFT implementation. Keep as the
  baseline. If GPU-FFT exploration stalls or runs out of time, this is
  the fallback shippable.
- **`vrx-waves-gpu-fft`** (this plan) — new branch off
  `vrx4-waves-fft`. Lands stages incrementally as commits.
- **`vrx-waves-asv-port`** (not yet created) — would land the
  asv_wave_sim architectural port if we decide to pursue that path
  instead.
