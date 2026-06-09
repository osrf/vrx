# VRX Wave Design Reference

A single place to understand how VRX simulates ocean waves: the architecture, how
to choose a backend, every configuration knob, and how the in-progress **Encino**
spectrum engine differs from the default path.

This is a *reference*. For the design rationale and upstreaming roadmap see
[`waves_integration_plan.md`](waves_integration_plan.md); for the known FFT
first-frame load issue see [`../waves/README.md`](../waves/README.md).

---

## 1. Architecture

The wave system is three decoupled layers that communicate through a single
`Wavefield` ECM component on the world entity:

| Layer | Plugin (`filename` / class) | Responsibility |
|---|---|---|
| **Source** | `gz-sim-waves-system` / `Waves` | Builds the wave field from SDF, writes the `Wavefield` component, advances it (throttled) |
| **Vessel coupling** | `gz-sim-wave-buoyancy-system` / `WaveBuoyancy` | Samples surface elevation under a hull to apply buoyancy forces |
| **Visual** | `gz-sim-water-visual-system` / `WaterVisual` | Renders the surface in custom GLSL, kept in sync via the same parameters |

Key design points:

- The wave backend is hidden behind the abstract `IWaveSimulation` interface
  (`waves/include/gz/sim/waves/WaveSimulation.hh`). Consumers never see the
  backend; they query through the free functions in `Eval.hh`
  (`SurfaceElevation`, `Normal`, `ParticleVelocity`, `Jacobian`, `FoamMask`).
- The backend is selected by a factory, `CreateWaveSimulation()`
  (`waves/src/WaveSimulation.cc:22`).
- The `Wavefield` component carries a `generation` counter so the GUI/visual
  process can cheaply detect parameter changes, and an `updateRate` so the visual
  throttles in step with the server.
- All Ogre Next access lives behind a `dlopen`'d C-ABI bridge
  (`libwaves-ogre2-bridge.so`), so the system plugins carry no link dependency on
  the renderer (which would break gz-rendering's engine-plugin loader).

---

## 2. Choosing a backend — `<algorithm>`

The factory recognizes exactly **two** user-selectable backends:

| `<algorithm>` | Type | Trade-off |
|---|---|---|
| `gerstner` *(default)* | Analytic Gerstner / trochoidal sum-of-sines (Tessendorf 2001). Stateless, unbounded in space, vertex-shader displacement. | Loads in ~5 s, lower visual fidelity. |
| `fft` | Stochastic spectral FFT. Phillips spectrum + Tessendorf choppy displacement, CPU IFFT each tick uploaded to a GPU heightmap. Periodic tile (queries wrap). | Higher fidelity, but a **~2 min first-frame stall** on the FFT visual — see [`../waves/README.md`](../waves/README.md). |

An unknown value logs an error and aborts (`WaveSimulation.cc:35`).

---

## 3. Spectrum model — `<model>`

Both backends take a spectrum/sampling model:

- **`PMS`** *(default)* — Pierson-Moskowitz. Wind-derived: `<period>` encodes the
  wind speed via the deep-water relation `V ≈ 0.879·g/ω_peak`.
- **`CWR`** — Constant Wave Representation. You set `<amplitude>` directly.

---

## 4. Configuration surface

### 4.1 Wave source — system level

Parsed in `waves/src/systems/Waves.cc` (`ParseSdf`, line 54).

| Tag | Type | Default | Meaning |
|---|---|---|---|
| `<algorithm>` | string | `gerstner` | Backend: `gerstner` or `fft` |
| `<update_rate>` | double [Hz] | `30.0` | Throttles backend `Update()` (esp. the FFT IFFT); mirrored to the visual |

### 4.2 Wave source — `<wave>` parameters

Defaults live in `WaveParameters` (`waves/include/gz/sim/waves/Wavefield.hh:29`).

**Shared (both backends):**

| Tag | Type | Default | Meaning |
|---|---|---|---|
| `<model>` | string | `PMS` | Spectrum model: `PMS` or `CWR` |
| `<number>` | uint | `3` | Number of component waves (Gerstner) |
| `<period>` | double [s] | `5.0` | Mean/peak wave period; encodes wind speed under PMS |
| `<amplitude>` | double [m] | `0.0` | Mean amplitude (CWR only) |
| `<direction>` | double [rad] | `0.0` | Mean propagation direction from +X |
| `<angle>` | double [rad] | `0.4` | Angular spread between components |
| `<scale>` | double | `1.1` | Frequency scale between components |
| `<steepness>` | double [0,1] | `0.0` | Gerstner crest sharpness (0 = sine) |
| `<phase>` | double [rad] | `0.0` | Common phase offset |
| `<tau>` | double [s] | `2.0` | Startup ramp constant `(1 − exp(−t/τ))` |
| `<gain>` | double | `1.0` | PMS amplitude multiplier |

**FFT-only (ignored by Gerstner):**

| Tag | Type | Default | Meaning |
|---|---|---|---|
| `<tile_size>` | double [m] | `200.0` | Physical extent of the periodic tile per axis |
| `<grid_size>` | uint (pow-2) | `128` | Grid samples per axis (64/128/256); must be a power of two |
| `<seed>` | uint | `0` | RNG seed for the Phillips amplitudes; same seed → identical field |
| `<choppiness>` | double | `-1.0` | Tessendorf horizontal-displacement multiplier (range ~[−2, 0]; 0 disables) |

### 4.3 Vessel buoyancy

Parsed in `waves/src/systems/WaveBuoyancy.cc` (`Configure`, line 92). Uses a
cylindrical circular-segment immersion model, sampling elevation at each
`<point>`.

| Tag | Type | Default | Meaning |
|---|---|---|---|
| `<link_name>` | string | — *(required)* | Link to apply buoyancy forces to |
| `<hull_length>` | double [m] | `4.9` | Cylinder length |
| `<hull_radius>` | double [m] | `0.213` | Cylinder radius |
| `<fluid_level>` | double [m] | `0.0` | Still-water reference height |
| `<fluid_density>` | double [kg/m³] | `1000.0` | Water density (≈1025 for seawater) |
| `<points>`/`<point>` | list of vec3 [m] | — *(required)* | Hull sample points in link frame |

> **Gotcha.** `WaveBuoyancy` provides a model's *complete* wave-coupled
> buoyancy. Do **not** also let gz-sim's standard `gz-sim-buoyancy-system`
> (`<graded_buoyancy>`) act on the same link — it models a *flat* static water
> plane, and its (larger, full-volume) force pins the model to the still
> surface, masking the wave excitation so the body never bobs. Disable it, or
> scope it away from wave-buoyancy models with `<enable>…</enable>`. (This is
> why a buoy can sit dead-flat even with waves running.)

### 4.4 Visual / shader

The shipped `<parameters>` in `waves/share/models/water_surface/model.sdf` (line
54). The `<shader>` block also wires the GLSL files and the GPU-compute shaders.

| Tag | Type | Default | Meaning |
|---|---|---|---|
| `<rescale>` | float | `0.5` | Vertex displacement / tangent magnitude scale |
| `<bumpScale>` | vec2 | `64 64` | Bumpmap tiling (×16 in the VS) |
| `<bumpSpeed>` | vec2 | `0.01 0.01` | Bumpmap scroll velocity (uv/s) |
| `<hdrMultiplier>` | float | `0.4` | Reflected-sky brightness boost |
| `<fresnelPower>` | float | `5.0` | Fresnel exponent for reflection blend |
| `<shallowColor>` | rgba | `0 0.1 0.3 1.0` | Shallow-water tint |
| `<deepColor>` | rgba | `0 0.05 0.2 1.0` | Deep-water tint |
| `<bumpMap>` | path | `textures/wave_normals.dds` | Normal-perturbation texture |
| `<cubeMap>` | path | `textures/skybox_lowres.dds` | Reflection cubemap |

> Additional visual knobs (`roughness`, `foamStrength`, `foamThreshold`) exist as
> defaults inside `WaterVisual` but are not present in the shipped `model.sdf`.

---

## 5. Experimental / developer environment variables

These are **not** part of the stable SDF surface — they gate work-in-progress
paths and are read from the process environment:

| Env var | Effect |
|---|---|
| `GZ_WAVES_USE_ENCINO=1` | Swap the in-tree Phillips spectrum for the **EncinoWaves** library inside the `fft` backend (read once at field build, `FFTWaveSimulation.cc:46`). Tunable via the `GZ_WAVES_ENCINO_*` knobs in §6.3. See §6. |
| `GZ_WAVES_GPU_FFT=1` (+ `_STAGE2`, `_STAGE3`, `_NAIVE`, `_CPU_FEED`, `_DEBUG`, `_VIEW_HKT`, `_TEST_PATTERN`) | Staged GPU compute-shader IFFT path in `WaterVisual`, intended to eventually replace the CPU IFFT and fix the slow-load issue. |
| `GZ_WAVES_HLMS_PBS=1` | Experimental HlmsPbs visual path. |

---

## 6. Encino vs. the default Phillips path

### 6.1 What EncinoWaves is

`encino_waves/` is a vendored, Apache-2.0 C++ library implementing Christopher
Horvath's 2015 paper *"Empirical Directional Wave Spectra for Computer
Graphics."* It is a Tessendorf-style spectral FFT ocean, but built around
oceanographically-validated spectra and directional-spreading models rather than
the single hand-tuned spectrum of classic graphics ocean code. It is header-only
templates (instantiated for `float`/`double`) plus three small `.cpp` files.

It is **not** a third `<algorithm>` — it is an optional spectrum engine swapped
*inside* the existing `fft` backend, gated by `GZ_WAVES_USE_ENCINO=1`.

### 6.2 Components (the pipeline)

Each stage is pluggable via `Parameters` enums
(`encino_waves/include/EncinoWaves/`):

| Component | Header | Provides |
|---|---|---|
| **Parameters** | `Parameters.h` | resolution, domain, gravity, wind, depth, fetch, pinch, gain, trough damping + the model selectors below |
| **Spectra** | `Spectra.h` | `PiersonMoskowitz`, `JONSWAP`, **`TMA`** (default; JONSWAP × Kitaigorodskii depth term) |
| **DirectionalSpreading** | `DirectionalSpreading.h` | **`Hasselmann`** (default), `Mitsuyasu`, `Donelan-Banner`, `Pos-Cos²`; plus a `swell` knob |
| **Dispersion** | `Dispersion.h` | `Deep`, `FiniteDepth`, **`Capillary`** (default; adds surface tension + finite depth) |
| **Filter** | `Filter.h` | `Null` (default) or smooth invertible band-pass on wavelength |
| **Random** | `Random.h` | `Normal` / `LogNormal` amplitude draws, per-wavenumber seeded |
| **InitialState** | `InitialState.h` | Runs the cascade once → `h₀(k)`, `conj(h₀(−k))`, `ω(k)` |
| **Propagation** | `Propagation.h` | Per frame: `h(k,t)=h₀e^{iωt}+h₀*e^{−iωt}` → IFFT → `Height`, `Dx`, `Dy`, plus `MinE` (Jacobian foam) and trough damping |
| **FftwWrapper** | `FftwWrapper.h` | GPL-free FFTW replacement: 2D c2r IFFT on **Eigen::FFT**, **TBB-parallelized** (`grainSize=8`) |
| Supporting | `SpectralSpatialField.h`, `Normals.h`, `MipMap.h`, `Stats.h`, `Basics.h` | Field containers, pinched normals, LOD pyramid, grid stats, math helpers |

### 6.3 How VRX wires it in

The integration (`FFTWaveSimulation.cc:172`) is deliberately partial. It builds an
`EncinoState` and maps **only five** VRX parameters into Encino, accepting
Horvath's "good ocean" defaults (TMA + Hasselmann + Capillary + Normal) for
everything else:

| VRX param | → Encino param |
|---|---|
| `grid_size` | `resolutionPowerOfTwo` (= log2) |
| `tile_size` | `domain` |
| `period` | `windSpeed` (via `0.879·g/ω_peak`) |
| `gain` | `amplitudeGain` |
| `seed` | `random.seed` |

Each tick it calls `propagation->propagate(...)`, applies the `<tau>` startup
ramp, a physics-based amplitude calibration, and the `<gain>` multiplier, then
copies `Height`, `Dx`, `Dy` into VRX's grids (`FFTWaveSimulation.cc:283`).

> **Amplitude calibration.** EncinoWaves' `amplitudeGain` only feeds its own
> (unused) normal computation — it does **not** scale the height field — and
> Encino's intrinsic variance is ~10× a physical sea state at low wind speeds.
> The integration measures Encino's intrinsic RMS once at construction and
> rescales the output so the significant wave height follows the standard
> fully-developed Pierson-Moskowitz wind-sea law `Hs = 0.21·V19.5²/g`; the
> selected spectrum still sets the spectral *shape*. `<gain>` is then applied
> as a user multiplier on top (it would otherwise be inert, since it maps to
> `amplitudeGain`). The factor and target Hs are echoed in the startup log
> (`ampCalib=… targetHs=…`).

**Tuning knobs (env vars).** Encino's distinctive controls are overridable at
launch without touching the SDF schema (consistent with the `GZ_WAVES_USE_ENCINO`
toggle itself). They are read once at field build, validated (unknown values are
ignored with a warning), and echoed in the `EncinoWaves spectrum library active
(...)` log line:

| Env var | Values / units | Default |
|---|---|---|
| `GZ_WAVES_ENCINO_SPECTRUM` | `pms` \| `jonswap` \| `tma` | `tma` |
| `GZ_WAVES_ENCINO_DISPERSION` | `deep` \| `finite` \| `capillary` | `capillary` |
| `GZ_WAVES_ENCINO_SPREADING` | `poscos2` \| `mitsuyasu` \| `hasselmann` \| `donelanbanner` | `hasselmann` |
| `GZ_WAVES_ENCINO_DEPTH` | metres | `100` |
| `GZ_WAVES_ENCINO_FETCH` | kilometres | `300` |
| `GZ_WAVES_ENCINO_SWELL` | swell elongation | `0` |
| `GZ_WAVES_ENCINO_TROUGH_DAMPING` | `[0,1]` breaking-wave damping | `0` |

### 6.4 Comparison

The "non-Encino" path is the in-tree Phillips spectrum (Tessendorf 2001)
implemented directly in `FFTWaveSimulation.cc` (`Phillips()` at line 214,
evolution at 307–391).

| Aspect | Default (Phillips, in-tree) | Encino (`GZ_WAVES_USE_ENCINO=1`) |
|---|---|---|
| Spectrum | Single Phillips form, deep-water only | TMA (default), or JONSWAP / Pierson-Moskowitz |
| Directional spreading | Baked-in `\|k̂·ŵ\|²` cosine weight | 4 empirical models (Hasselmann default) + swell |
| Dispersion | `ω=√(g·k)` deep-water only | Capillary (default), finite-depth, or deep |
| Choppiness | Tessendorf `Dx,Dy=−i·k̂·h`; `<choppiness>` applied in shader | Same — `<choppiness>` applied in shader to Encino's `Dx`/`Dy`; Encino's internal `pinch` only feeds its own (unused) normals |
| Normals | Analytic slope grids via extra IFFTs | **Not plumbed** → finite-difference fallback |
| Foam | Tessendorf Jacobian from chop-derivative grids | Library computes `MinE`, but **not plumbed** |
| FFT threading | Single-threaded `Ifft2DReal` (Eigen) | TBB-parallel c2r (`grainSize=8`) + parallel propagation |
| Startup ramp | `(1−exp(−t/τ))` applied | `(1−exp(−t/τ))` applied (matches Phillips) |
| Amplitude | Phillips spectrum w/ calibrated `specScale` | Output calibrated to PM `Hs=0.21·V²/g`; `<gain>` multiplier on top (§6.3) |
| Parameter surface | Full SDF set | 5 SDF knobs + `GZ_WAVES_ENCINO_*` env overrides (§6.3) |
| Maturity | Stable default | Experimental, env-gated, integration in progress |
| License | Apache-2.0 | Apache-2.0 (FftwWrapper avoids GPL FFTW) |

### 6.5 Current integration status

The Encino path delivers height + horizontal displacement, with the startup
ramp, `<choppiness>`, a physics-based amplitude calibration (§6.3), and a
working `<gain>` multiplier, plus the distinctive spectral models tunable via
`GZ_WAVES_ENCINO_*` (§6.3). Remaining limitations (verified against
`FFTWaveSimulation.cc:172–330` and `WaterVisual.cc:460`):

- Wave **`direction`** is **not** applied to Encino. Encino assumes wind along
  +X and expects the *whole field* to be externally transformed
  (`Parameters.h:89`); rotating a single periodic FFT tile by an arbitrary angle
  breaks its seamless tiling, so honouring `<direction>` requires the heading
  baked into Encino's spectrum generation (not yet exposed). Until then Encino
  waves travel along the tile's +X axis regardless of `<direction>`.
- **Normals and foam** use the finite-difference fallback: the Encino branch
  leaves the slope and chop-derivative grids zeroed and the visual runs with
  `useSlopeMap=0` (`WaterVisual.cc:460`). Encino's own normals / `MinE` foam are
  computed by the library but not yet plumbed through the heightmap bridge.

---

## 7. Example configuration

From `vrx_gazebo/worlds/open_water.sdf` (line 94) — FFT, moderate seas, tuned to
mirror asv_wave_sim's reference scene:

```xml
<plugin filename="gz-sim-waves-system" name="gz::sim::systems::Waves">
  <algorithm>fft</algorithm>
  <update_rate>30</update_rate>
  <wave>
    <model>PMS</model>
    <number>3</number>
    <period>3.2</period>        <!-- ≈ 5 m/s wind equivalent -->
    <gain>1.0</gain>
    <direction>2.356</direction> <!-- 135° -->
    <angle>0.4</angle>
    <scale>1.1</scale>
    <steepness>0.0</steepness>
    <tau>2.0</tau>
    <tile_size>256</tile_size>
    <grid_size>128</grid_size>
    <seed>42</seed>
    <choppiness>-2.0</choppiness>
  </wave>
</plugin>
```

To try the Encino spectrum with this same world:

```bash
GZ_WAVES_USE_ENCINO=1 gz sim open_water.sdf
# Look for: [FFTWaveSimulation] EncinoWaves spectrum library active (res=128 ...)
```

---

## 8. See also

- [`../waves/README.md`](../waves/README.md) — backend summary + the FFT
  first-frame load limitation
- [`waves_integration_plan.md`](waves_integration_plan.md) — design rationale &
  upstreaming plan
