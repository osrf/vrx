# VRX Wave Design Reference

A single place to understand how VRX simulates ocean waves: the architecture, how
to choose a backend, every configuration knob, and how the **Encino** spectrum
(the default FFT spectrum) differs from the in-tree Phillips fallback.

This is a *reference* for the **current implementation**. For the conceptual
"wave socket" design and the upstreaming roadmap see
[`wave_provider_architecture.md`](wave_provider_architecture.md) and
[`waves_integration_plan.md`](waves_integration_plan.md); for the known FFT
first-frame load issue see [`../gz_waves/README.md`](../gz_waves/README.md).

---

## 1. Architecture

The wave system is three decoupled layers that communicate through a single
`Wavefield` ECM component on the world entity:

| Layer | Plugin (`filename` / class) | Responsibility |
|---|---|---|
| **Source** | `gz-sim-waves-fft-system` / `FftWaves`  ·  `gz-sim-waves-gerstner-system` / `GerstnerWaves` | Builds the wave field from SDF, writes the `Wavefield` component, advances it (throttled) |
| **Vessel coupling** | `gz-sim-wave-buoyancy-system` / `WaveBuoyancy` | Samples surface elevation under a hull to apply buoyancy forces |
| **Visual** | `gz-sim-water-visual-system` / `WaterVisual` | Renders the surface in custom GLSL, kept in sync via the same component |

Key design points:

- The wave backend is hidden behind the abstract `IWaveField` interface
  (`gz_waves/include/gz/sim/waves/WaveSimulation.hh`): `Elevation`,
  `ParticleVelocity`, `Normal`, `Jacobian`, `SetParameters`, `Kind` (pure), plus
  `Update`, `Bounds`, `Field` (defaulted). Consumers never see the concrete
  engine; they query through the free functions in `Eval.hh` (`SurfaceElevation`,
  `ParticleVelocity`, `Normal`, `Jacobian`, `FoamMask`).
- **There is no `<algorithm>` SDF tag.** The engine *is* the system plugin you
  load by filename. Each engine has its own thin `WavesSystemBase` subclass that
  overrides `EngineToken()` (`"fft"` / `"gerstner"`) and `MakeEngine()`.
- Engines are plain libraries (`gz-waves-provider-fft`,
  `gz-waves-provider-gerstner`) linked directly by the system plugins — **no
  dlopen engine loader**. They register a token → factory mapping in an
  in-process registry (`RegisterWaveEngineFactory` / `CreateWaveSimulation` in
  `gz_waves/src/WaveSimulation.cc`); `CreateWaveSimulation(token, params)` returns
  a configured `IWaveField`, or `nullptr` for an unknown token.
- The `Wavefield` component carries an `algorithm` token + a `generation` counter
  (so the visual cheaply detects parameter changes) and an `updateRate` (so the
  visual throttles in step with the server). It is replicated to the GUI by
  **component serialization** (`operator<<` / `operator>>` in `Wavefield.hh`);
  `operator>>` rebuilds the engine GUI-side via `CreateWaveSimulation` (the GUI
  factories are registered in `gz_waves_rendering/src/RegisterEngines.cc`).
- All Ogre Next access lives behind a `dlopen`'d C-ABI bridge
  (`libwaves-ogre2-bridge.so`), so the visual plugin carries no link dependency on
  the renderer (which would interfere with gz-rendering's own engine-plugin
  loader).

---

## 2. Choosing a backend — by plugin filename

There is no algorithm tag; you select the backend by **which source plugin you
load**:

| Plugin `filename` (class) | Engine | Trade-off |
|---|---|---|
| `gz-sim-waves-gerstner-system` (`GerstnerWaves`) | Analytic Gerstner / trochoidal sum-of-sines (Tessendorf 2001). Stateless, unbounded in space, vertex-shader displacement. | Loads in ~5 s, lower visual fidelity. |
| `gz-sim-waves-fft-system` (`FftWaves`) | Stochastic spectral FFT. EncinoWaves spectra by default (Phillips fallback), CPU IFFT each tick uploaded to a GPU heightmap. Periodic tile (queries wrap). | Higher fidelity, but a **~2 min first-frame stall** on the FFT visual — see [`../gz_waves/README.md`](../gz_waves/README.md). |

Load exactly one source plugin. The `name=` attribute must match the class
(`gz::sim::systems::FftWaves` or `gz::sim::systems::GerstnerWaves`).

---

## 3. Spectrum model — `<model>`

Both backends take a sampling model:

- **`PMS`** *(default)* — Pierson-Moskowitz. Wind-derived: `<period>` encodes the
  wind speed via the deep-water relation `V ≈ 0.879·g/ω_peak`.
- **`CWR`** — Constant Wave Representation. You set `<amplitude>` directly.

---

## 4. Sea state — `<sea_state>`

`<sea_state>` (WMO sea-state code 0–9) is a convenience that overrides `<period>`
and `<gain>` with values approximating a fully-developed sea of that roughness.
Source: WMO code table 3700 / <https://en.wikipedia.org/wiki/Sea_state>.

| Code | Description | Significant height Hs [m] |
|---|---|---|
| 0 | Calm (glassy) | 0.0 |
| 1 | Calm (rippled) | 0.05 |
| 2 | Smooth | 0.3 |
| 3 | Slight | 0.875 |
| 4 | Moderate | 1.875 |
| 5 | Rough | 3.25 |
| 6 | Very rough | 5.0 |
| 7 | High | 7.5 |
| 8 | Very high | 11.5 |
| 9 | Phenomenal | 16.0 |

From Hs the code derives the 19.5 m wind speed `V = √(Hs·g / 0.21)` and the peak
period `Tp = 2π·V / (0.879·g)`, then sets `period = Tp` and `gain = 1.0` (physical
amplitude, no artistic boost; for `CWR`, `amplitude = Hs/2`). `<sea_state>` unset
(`-1`, the default) leaves `<period>`/`<gain>` as given.

---

## 5. Configuration surface

### 5.1 Wave source — system level

Parsed in `gz_waves/src/systems/WavesSystemBase.cc` (`ParseSdf`).

| Tag | Type | Default | Meaning |
|---|---|---|---|
| `<update_rate>` | double [Hz] | `30.0` | Throttles backend `Update()` (esp. the FFT IFFT); mirrored to the visual |

### 5.2 Wave source — `<wave>` parameters

Defaults live in `WaveParameters` (`gz_waves/include/gz/sim/waves/Wavefield.hh`).

**Shared (both backends):**

| Tag | Type | Default | Meaning |
|---|---|---|---|
| `<model>` | string | `PMS` | Spectrum model: `PMS` or `CWR` |
| `<number>` | uint | `3` | Number of component waves (Gerstner) |
| `<period>` | double [s] | `5.0` | Mean/peak wave period; encodes wind speed under PMS (overridden by `<sea_state>`) |
| `<amplitude>` | double [m] | `0.0` | Mean amplitude (CWR only) |
| `<direction>` | double [rad] | `0.0` | Mean propagation direction from +X |
| `<angle>` | double [rad] | `0.4` | Angular spread between components |
| `<scale>` | double | `1.1` | Amplitude/length ratio between mean and extreme waves |
| `<steepness>` | double [0,1] | `0.0` | Gerstner crest sharpness (0 = round) |
| `<phase>` | double [rad] | `0.0` | Common phase offset |
| `<tau>` | double [s] | `2.0` | Startup ramp constant `(1 − exp(−t/τ))` |
| `<gain>` | double | `1.0` | PMS amplitude multiplier (overridden by `<sea_state>`) |
| `<sea_state>` | int [0–9] | `-1` | WMO sea-state code; when ≥ 0, overrides `<period>` and `<gain>` (§4) |

**FFT-only (ignored by Gerstner):**

| Tag | Type | Default | Meaning |
|---|---|---|---|
| `<tile_size>` | double [m] | `200.0` | Physical extent of the periodic tile per axis |
| `<grid_size>` | uint (pow-2) | `128` | Grid samples per axis (64/128/256); must be a power of two |
| `<seed>` | uint | `0` | RNG seed for the Phillips amplitudes; same seed → identical field |
| `<choppiness>` | double | `-1.0` | Tessendorf horizontal-displacement multiplier (range ~[−2, 0]; 0 disables) |

### 5.3 Vessel buoyancy

Parsed in `gz_waves/src/systems/WaveBuoyancy.cc`. A cylindrical circular-segment
immersion model sampling elevation at each `<point>`.

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
> scope it away from wave-buoyancy models with `<enable>…</enable>`.

### 5.4 Visual / shader

The shipped `<parameters>` live in
`gz_waves_rendering/share/models/water_surface/model.sdf`. The `<shader>` block
wires the GLSL files; `<textures>` wires the bump/cube maps. Shipped values:

| Tag | Type | Shipped | Meaning |
|---|---|---|---|
| `<fft_vertex>` | path | `shaders/fft_water_vs_330.glsl` | Vertex shader (heightmap sampling + displacement) |
| `<fragment>` | path | `shaders/water_fs_330.glsl` | Fragment shader (colour, normals, Fresnel, foam) |
| `<rescale>` | float | `0.5` | Vertex displacement / tangent magnitude scale |
| `<bumpScale>` | vec2 | `64 64` | Bumpmap tiling (×16 in the VS → 1024 repeats/uv) |
| `<bumpSpeed>` | vec2 | `0.01 0.01` | Bumpmap scroll velocity (uv/s) |
| `<hdrMultiplier>` | float | `0.4` | Reflected-sky brightness boost |
| `<fresnelPower>` | float | `5.0` | Fresnel exponent for reflection blend |
| `<shallowColor>` | rgba | `0 0.1 0.3 1.0` | Shallow-water tint |
| `<deepColor>` | rgba | `0 0.05 0.2 1.0` | Deep-water tint |
| `<bumpMap>` | path | `textures/wave_normals.dds` | Normal-perturbation texture |
| `<cubeMap>` | path | `textures/skybox_lowres.dds` | Reflection cubemap |

> Additional visual knobs (`roughness`, `foamStrength`, `foamThreshold`) and the
> tile-instancing tags (`<tiles_radius>`, `<tile_mesh_size>`) have defaults inside
> `WaterVisual` but are not present in the shipped `model.sdf`.

---

## 6. The FFT engine: Encino (default) vs. Phillips (fallback)

The `fft` engine (`gz_waves_provider_fft`) runs one of two spectra, chosen **at
build time**:

- **EncinoWaves** — used when the package is built with the optional
  `encinowaves_vendor` (compile flag `GZ_WAVES_WITH_ENCINO`). **This is the
  default.**
- **In-tree Phillips** (Tessendorf 2001) — the fallback when built without encino.

There is **no `GZ_WAVES_USE_ENCINO` environment variable** (removed); the spectrum
is decided purely by whether encino is present at build time. The build prints
`gz_waves_provider_fft: EncinoWaves spectrum ENABLED` or `… → Phillips-only`.

### 6.1 What EncinoWaves is

`encino_waves/` is a vendored, Apache-2.0 C++ library implementing Christopher
Horvath's 2015 paper *"Empirical Directional Wave Spectra for Computer
Graphics."* It is a Tessendorf-style spectral FFT ocean built around
oceanographically-validated spectra and directional-spreading models rather than
a single hand-tuned spectrum. Header-only templates (`float`/`double`) plus a few
`.cpp` files; its `FftwWrapper` runs the 2D c2r IFFT on **Eigen::FFT** (no GPL
FFTW), **TBB-parallelized**.

### 6.2 Components (the pipeline)

Each stage is pluggable via `Parameters` enums
(`encino_waves/include/EncinoWaves/`):

| Component | Provides |
|---|---|
| **Spectra** | `PiersonMoskowitz`, `JONSWAP`, **`TMA`** (default) |
| **DirectionalSpreading** | **`Hasselmann`** (default), `Mitsuyasu`, `Donelan-Banner`, `Pos-Cos²` + swell |
| **Dispersion** | `Deep`, `FiniteDepth`, **`Capillary`** (default) |
| **Filter** | `Null` (default) or smooth invertible band-pass on wavelength |
| **Random** | `Normal` / `LogNormal` amplitude draws, per-wavenumber seeded |
| **InitialState** | Runs the cascade once → `h₀(k)`, `conj(h₀(−k))`, `ω(k)` |
| **Propagation** | Per frame: `h(k,t)=h₀e^{iωt}+h₀*e^{−iωt}` → IFFT → `Height`, `Dx`, `Dy`, plus `MinE` (Jacobian foam) |

### 6.3 How VRX wires it in

The integration (`FFTWaveSimulation.cc`) maps **five** VRX SDF parameters into
Encino and accepts Horvath's defaults (TMA + Hasselmann + Capillary + Normal) for
the rest:

| VRX param | → Encino param |
|---|---|
| `grid_size` | `resolutionPowerOfTwo` (= log2) |
| `tile_size` | `domain` |
| `period` | `windSpeed` (via `0.879·g/ω_peak`) |
| `gain` | `amplitudeGain` |
| `seed` | `random.seed` |

Each tick it propagates the spectrum, applies the `<tau>` startup ramp, a
physics-based amplitude calibration, and the `<gain>` multiplier, then copies
`Height`, `Dx`, `Dy` (and `MinE` → foam) into VRX's grids.

> **Amplitude calibration.** EncinoWaves' `amplitudeGain` does not scale its
> height field, and its intrinsic variance is ~10× a physical sea state at low
> wind speeds. The integration measures Encino's intrinsic RMS once at
> construction and rescales the output so the significant wave height follows the
> fully-developed Pierson-Moskowitz wind-sea law `Hs = 0.21·V19.5²/g`; the
> selected spectrum still sets the spectral *shape*. `<gain>` is applied as a user
> multiplier on top. The factor and target Hs are echoed in the startup log.

**Tuning knobs (env vars).** Encino's distinctive controls are overridable at
launch without touching the SDF schema. They are read once at field build,
validated (unknown values warn and are ignored), and echoed in the `EncinoWaves
spectrum library active (...)` log line:

| Env var | Values | Default |
|---|---|---|
| `GZ_WAVES_ENCINO_SPECTRUM` | `pms` \| `jonswap` \| `tma` | `tma` |
| `GZ_WAVES_ENCINO_DISPERSION` | `deep` \| `finite` \| `capillary` | `capillary` |
| `GZ_WAVES_ENCINO_SPREADING` | `poscos2` \| `mitsuyasu` \| `hasselmann` \| `donelanbanner` | `hasselmann` |
| `GZ_WAVES_ENCINO_DEPTH` | metres | `100` |
| `GZ_WAVES_ENCINO_FETCH` | kilometres | `300` |
| `GZ_WAVES_ENCINO_SWELL` | swell elongation | `0` |
| `GZ_WAVES_ENCINO_TROUGH_DAMPING` | breaking-wave damping `[0,1]` | `0` |
| `GZ_WAVES_ENCINO_FILTER_*` | `MIN`, `MIN_WL`, `MAX_WL`, `SOFT`, `INVERT` — band-pass on wavelength | off |

### 6.4 Comparison

The non-Encino path is the in-tree Phillips spectrum (Tessendorf 2001) in
`FFTWaveSimulation.cc`.

| Aspect | Phillips (in-tree fallback) | Encino (default) |
|---|---|---|
| Spectrum | Single Phillips form, deep-water only | TMA (default), or JONSWAP / Pierson-Moskowitz |
| Directional spreading | Baked-in `\|k̂·ŵ\|²` cosine weight | 4 empirical models (Hasselmann default) + swell |
| Dispersion | `ω=√(g·k)` deep-water only | Capillary (default), finite-depth, or deep |
| Choppiness | Tessendorf `Dx,Dy=−i·k̂·h`; `<choppiness>` applied in shader | Same — `<choppiness>` in shader on Encino's `Dx`/`Dy` |
| Output grids | `Height`, `Dx`, `Dy` (3 IFFTs) | `Height`, `Dx`, `Dy`, `MinE` |
| Normals | Finite differences of the heightmap in the shader | Same (shader finite differences) |
| Foam | None — leaves the foam channel flat | From Encino's `MinE` (Jacobian → foam channel) |
| FFT threading | Single-threaded `Ifft2DReal` (Eigen) | TBB-parallel c2r + parallel propagation |
| Startup ramp | `(1−exp(−t/τ))` | `(1−exp(−t/τ))` (matches Phillips) |
| Amplitude | Phillips spectrum w/ calibrated `specScale` | Calibrated to PM `Hs=0.21·V²/g`; `<gain>` on top (§6.3) |
| Maturity | Fallback only | Default |
| License | Apache-2.0 | Apache-2.0 (Eigen::FFT, no GPL FFTW) |

> **Note.** Earlier revisions uploaded per-vertex slope and chop-derivative grids
> (5 extra IFFTs) to drive spectrum-accurate normals via a GPU slope-map. That
> path — together with the experimental GPU-compute IFFT — was removed; the FFT
> `Update` now runs 3 IFFTs (height + Dx + Dy) and normals are finite-differenced
> in the fragment shader for both spectra.

### 6.5 Current integration limitations

- Wave **`direction`** is **not** applied to Encino. Encino assumes wind along +X
  and expects the whole field to be externally transformed; rotating a single
  periodic FFT tile by an arbitrary angle breaks its seamless tiling, so honouring
  `<direction>` requires the heading baked into Encino's spectrum generation (not
  yet exposed). Encino waves travel along the tile's +X axis regardless of
  `<direction>`.

---

## 7. Example configuration

FFT source for an open-water scene (`vrx_gazebo/worlds/open_water.sdf`):

```xml
<plugin filename="gz-sim-waves-fft-system" name="gz::sim::systems::FftWaves">
  <update_rate>30</update_rate>
  <wave>
    <!-- <sea_state>5</sea_state>  optional: WMO 0-9, overrides period+gain -->
    <model>PMS</model>
    <period>3.2</period>          <!-- ≈ 5 m/s wind equivalent -->
    <gain>1.0</gain>
    <direction>2.356</direction>  <!-- 135° -->
    <tau>2.0</tau>
    <tile_size>256</tile_size>
    <grid_size>128</grid_size>
    <seed>42</seed>
    <choppiness>-2.0</choppiness>
  </wave>
</plugin>
```

To switch to the analytic Gerstner backend, load the other source plugin instead
(it ignores the FFT-only tags `tile_size`, `seed`, `choppiness`):

```xml
<plugin filename="gz-sim-waves-gerstner-system" name="gz::sim::systems::GerstnerWaves">
  <update_rate>30</update_rate>
  <wave>
    <model>PMS</model>
    <number>3</number>
    <period>3.2</period>
    <direction>2.356</direction>
    <angle>0.4</angle>
    <scale>1.1</scale>
    <steepness>0.5</steepness>
    <gain>1.0</gain>
  </wave>
</plugin>
```

The EncinoWaves spectrum is the default for the FFT engine when built with
`encinowaves_vendor` — no env var or SDF flag is needed. Confirm at launch with:

```
[FFTWaveSimulation] EncinoWaves spectrum library active (res=128 ...)
```

---

## 8. See also

- [`../gz_waves/README.md`](../gz_waves/README.md) — backend summary + the FFT
  first-frame load limitation
- [`wave_provider_architecture.md`](wave_provider_architecture.md) — the "wave
  socket" design and abstraction boundaries
- [`waves_integration_plan.md`](waves_integration_plan.md) — design rationale &
  upstreaming plan
