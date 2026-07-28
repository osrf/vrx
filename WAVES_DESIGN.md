# VRX Wave Simulation — Design & Contributor Reference

This document is the single reference for the VRX wave simulation packages
(`gz_waves*`): the layer that connects a pluggable wave field engine to Gazebo
physics and rendering. It captures the **requirements**, the **architecture and
design decisions**, the **per-package implementation details**, and a
step-by-step **guide to contributing a new wave field engine**.

## Terminology

The wave literature, and our own earlier notes, use several near-synonyms
loosely ("backend", "wave model", "wave rendering model", "generator"). This
document fixes the following vocabulary and uses it consistently:

| Term | Meaning |
|------|---------|
| **wave field** | The *output*: the water-surface state (elevation η, particle velocity, surface normal, and the folding/Jacobian metric) over space and time. It is what consumers read; it is **not** the thing that produces it. |
| **wave field engine (WFE)** | A concrete `IWaveField` implementation that *produces* a wave field (e.g. the analytic Gerstner engine, the spectral FFT engine). Supersedes the looser "backend" and "wave (rendering) model"; "rendering model" especially misleads, since an engine drives physics too, not just the visual. |
| **provider** | The package that ships a WFE plus its server/GUI plumbing (`gz_waves_provider_*`). |
| **consumer** | Anything that *reads* the wave field through the core API (vessel buoyancy/hydrodynamics, the renderer, future sensors). `WaveBuoyancy` is the illustrative consumer used throughout. |

**What defines a WFE.** A wave field engine synthesizes the surface as a sum of
components, η(x, t) = Σ aₙ cos(kₙ·x − ωₙ t + φₙ). Per the VRX waves roadmap
(`vrx4_waves_roadmap.md`), the method is defined by **five independent choices**,
three of them statistical:

| Choice | Determines | Options |
|--------|------------|---------|
| **Inverse transform** | how the spectrum becomes the field | direct summation (sum of cosines) vs. FFT / IFFT |
| **Wave kinematics** | the shape of each component | linear / Airy (vertical only) vs. Gerstner / trochoidal (adds horizontal chop) |
| **Amplitude sampling** *(statistical)* | the amplitudes aₙ | deterministic vs. random (Rayleigh / Gaussian) |
| **Frequency sampling** *(statistical)* | the frequencies ωₙ | deterministic (evenly spaced) vs. random (drawn within bands) |
| **Phase sampling** *(statistical)* | the phases φₙ | deterministic vs. random (uniform) |

The choices are independent, though the roadmap notes only a handful of
combinations are physically sensible (e.g. the fully deterministic legacy sea, or
the random amplitude Gaussian sea that EncinoWaves draws). The engines shipped
today are two points in this space, and their package names are a **pragmatic
shorthand for each engine's dominant distinguishing choice**, not a claim that the
choices are coupled:

- `gz_waves_provider_gerstner`: direct summation, Gerstner kinematics, fully
  deterministic amplitude, frequency, and phase. Implemented in this repository.
- `gz_waves_provider_fft`: IFFT, Gerstner chop, random amplitude, evenly spaced
  frequency, random phase (the Gaussian sea). The transform, spectra, sampling,
  and kinematics all live in the **external EncinoWaves library**; the VRX package
  is a thin wrapper (§6).

The `IWaveField` contract itself is **agnostic** to all five choices: a new engine
(in this repo or wrapping an external library) may implement any sensible
combination. (Whether the package *names* should move from this shorthand toward
the roadmap's design-choice vocabulary is an open design question.)

## Packages

All `gz_waves*` packages live in **this repository** (vrx). **EncinoWaves is the
one external dependency**: its own repository, installed separately, **not**
vendored.

| Package | Role |
|---------|------|
| `gz_waves` | Engine-agnostic **core**: the `IWaveField` contract, the engine registry, the `Wavefield` ECM component, the `Eval` query facade, and `WavesSystemBase` (the source-plugin base). |
| `gz_waves_provider_gerstner` | Analytic **Gerstner** WFE + source system + GUI registrar. |
| `gz_waves_provider_fft` | Spectral **FFT** WFE (wraps EncinoWaves) + source system + GUI registrar. |
| `gz_waves_rendering` | Engine-agnostic **WaterVisual** renderer + the Ogre2 C-ABI bridge + the `water_surface` model; wires a runnable demo world. |

All new source files carry the **Honu Robotics** Apache-2.0 copyright header.

---

## 1. Requirements

### Functional

- **R1 — Multiple selectable wave field engines.** Support more than one WFE
  (analytic Gerstner, spectral FFT) chosen per world, with room to add
  more without touching existing packages.
- **R2 — One physical wave field, many consumers.** A single authoritative wave
  field drives every consumer — vessel buoyancy/hydrodynamics, the rendered
  surface, future sensors — so they stay mutually consistent.
- **R3 — Visualization.** The wave surface is rendered (displaced + shaded mesh)
  in the Gazebo GUI, matching the simulated field.
- **R4 — One-knob sea control.** A single `<sea_state>` (WMO code 0–9) sets a
  realistic sea, with an explicit manual mode (`<period>` + `<gain>`) for
  fine control.
- **R5 — Runtime reconfiguration.** Wave parameters are changeable at runtime via
  a transport service, without restarting the simulation.
- **R6 — Determinism.** Given the same parameters and seed, the FFT field is
  reproducible run-to-run (and across the server/GUI split).
- **R7 — Runnable out of the box.** `ros2 launch vrx_bringup simulation.launch.xml`
  brings up a moving ocean with no extra arguments.

### Non-functional

- **N1 — Engine-agnostic core and renderer.** Neither the core nor the renderer
  may build-depend on any concrete engine; adding an engine must not modify them.
- **N2 — Thread safety in the GUI.** The renderer runs across the ECM thread and
  the render thread (and gz-sim's GuiRunner double-loads GUI systems); the wave
  engine it drives must never be raced.
- **N3 — Engine encapsulation.** Consumers query the field through a stable,
  null-safe API and never include a concrete engine's headers.
- **N4 — Render-engine isolation.** Ogre Next symbols must not leak into the
  WaterVisual plugin's link map (they break gz-rendering's own engine loader).
- **N5 — Seamless tiling.** The rendered surface tiles across its periodic domain
  with no visible seams.
- **N6 — House style + tests.** Gazebo code conventions throughout, with unit
  tests covering each package.

---

## 2. Architecture overview

Three decoupled layers communicate **only** through one ECM component,
`components::Wavefield`, attached to the world entity. The component carries the
**recipe** (algorithm token + parameters + a generation counter) and, on the
server, a live `shared_ptr<IWaveField>`. Only the recipe is replicated to the
GUI; each process/consumer rebuilds its own engine from it.

```
            ┌──────────────────────── server process ─────────────────────────┐
   SDF ───> gz-sim-waves-<engine>-system        (a WavesSystemBase subclass)    │
            │   • parses <wave>, reads world <gravity>                          │
            │   • builds engine via MakeEngine()  ─────────┐                    │
            │   • advertises .../wave/set_parameters        v                   │
            │                                  components::Wavefield            │
            │                                  { algorithm, params,             │
            │                                    generation, updateRate,        │
            │                                    shared_ptr<IWaveField> }        │
            │            ┌──────────────────────────┴───────────────┐          │
            │     WaveBuoyancy (consumer)                  state replication     │
            │     via Eval free functions                          │           │
            └───────────────────────────────────────────────────────┼──────────┘
                                                                     v  (recipe only;
            ┌──────────────────────── GUI process ───────────────────┴──────────┐
            │  gz-sim-waves-<engine>-gui  → RegisterWaveEngineFactory(token,fn)   │
            │  WaterVisual                                                        │
            │    └─ CreateWaveSimulation(algorithm, params) → private IWaveField  │
            │         └─ Field() → HeightMapTexture                               │
            │              └─ C-ABI → libwaves-ogre2-bridge.so → Ogre Next GPU    │
            └─────────────────────────────────────────────────────────────────────┘
```

**Data flow.** A source system *produces* the component; consumers (buoyancy,
the visual) *read* it. The engine pointer is process-local: it is **never
serialized** — replication carries the recipe, and each consumer reconstructs
its engine through the registry. This is what keeps the core and the renderer
engine-agnostic (N1, N3).

---

## 3. The core contract (`gz_waves`)

### 3.1 `IWaveField` — the wave field engine interface

`include/gz/sim/waves/WaveSimulation.hh`. A concrete engine implements:

| Method | Purpose |
|--------|---------|
| `double Elevation(double _x, double _y, double _t) const` | Surface elevation η [m] above still water. |
| `gz::math::Vector3d ParticleVelocity(double _x, double _y, double _t) const` | Water-particle velocity [m/s] (drag hydrodynamics). |
| `gz::math::Vector3d Normal(double _x, double _y, double _t) const` | Outward unit surface normal. |
| `double Jacobian(double _x, double _y, double _t) const` | Horizontal-displacement Jacobian; low values ⇒ folding/whitecaps. |
| `void SetParameters(const WaveParameters &_params)` | (Re)build all internal state from the recipe. |
| `void Update(double _simTime)` | Advance time-dependent state (no-op default for stateless engines). |
| `std::string_view Kind() const` | Engine token, e.g. `"gerstner"`, `"fft"`. |
| `std::optional<TileSize> Bounds() const` | Periodic extent (grid engines); `nullopt` for unbounded analytic ones. |
| `const WaveField2D *Field() const` | The renderable grid (see below); `nullptr` if the engine has none. |

**`WaveField2D` — the rendering contract.** A Plain Old Data (POD) view the
renderer consumes without knowing the engine:

```cpp
struct WaveField2D {
  std::size_t   n{0};        // grid resolution N (N×N)
  double        tile{0.0};   // tile extent [m]; cell spacing = tile / N
  const double *dz{nullptr}; // vertical displacement η [m]  (required)
  const double *dx{nullptr}; // FINAL horizontal displacement x [m] (null ⇒ 0)
  const double *dy{nullptr}; // FINAL horizontal displacement y [m] (null ⇒ 0)
  const double *foam{nullptr}; // folding metric: 1 flat, <1 folding (null ⇒ none)
};
```

Arrays are **column-major N×N**, periodic over `tile`, owned by the engine, and
valid until the next `SetParameters`; `Update` refreshes their contents in place.
`TileSize` is `{double x, double y}` (a centred, periodic tile).

### 3.2 Engine registry — token → factory

```cpp
using WaveEngineFactory = std::function<std::shared_ptr<IWaveField>()>;
void RegisterWaveEngineFactory(const std::string &_token, WaveEngineFactory _factory);
std::shared_ptr<IWaveField> CreateWaveSimulation(
    const std::string &_algorithm, const WaveParameters &_params);
```

A process-wide, mutex-guarded map. `CreateWaveSimulation` looks up `_algorithm`,
builds a fresh engine, calls `SetParameters(_params)`, and returns it — or
returns `nullptr` (with a logged error) for an unknown token or a null-returning
factory. This indirection is what lets a consumer build an engine it doesn't
link against (the renderer's whole premise).

### 3.3 `WaveParameters` — the recipe

`include/gz/sim/waves/Wavefield.hh`. One struct shared by all engines; each
engine reads the subset it understands.

*Shared:* `model{"PMS"}`, `number{3}`, `period{5.0}`, `direction{0.0}`,
`angle{0.4}`, `scale{1.1}`, `steepness{0.0}`, `phase{0.0}`, `tau{2.0}`,
`gain{1.0}`, `gravity{9.8}` (populated from world `<gravity>`), `seaState{-1}`.
*Gerstner-only:* `amplitude{0.0}` (CWR model).
*FFT-only:* `tileSize{200.0}`, `gridSize{128}`, `seed{0}`, `choppiness{-1.0}`,
`spectrum{"tma"}`, `spreading{"hasselmann"}`, `dispersion{"capillary"}`,
`depth{100.0}`, `fetch{300.0}`, `swell{0.0}`, `troughDamping{0.0}`, and the
band-pass filter set (`filterMinWavelength`, `filterMaxWavelength`,
`filterSoftWidth`, `filterMin{0.0}`, `filterInvert{false}`).

### 3.4 `Wavefield` ECM component

`components::Wavefield` wraps `waves::WavefieldData`:

```cpp
struct WavefieldData {
  std::string                 algorithm{"gerstner"}; // engine token
  WaveParameters              params;                // the recipe
  std::shared_ptr<IWaveField> simulation;            // the live engine (process-local)
  std::uint64_t               generation{0};         // bumped on every reconfigure
  double                      updateRate{30.0};      // server Update() throttle [Hz]
};
```

Serialization writes only `algorithm`, `generation`, every `params` field, and
`updateRate`; `operator>>` leaves `simulation == nullptr`. **Consequence:** a
replicated copy is recipe-only — consumers rebuild the engine via
`CreateWaveSimulation`, and compare `generation` to cheaply detect changes.

### 3.5 `Eval` facade — null-safe queries

`include/gz/sim/waves/Eval.hh`. Free functions consumers use **instead of**
touching `IWaveField`, so the engine type stays opaque (N3):

- `Advance(WavefieldData&, double _t)` — drives `Update` if an engine is present.
- `SurfaceElevation`, `ParticleVelocity`, `Normal`, `Jacobian`,
  `FoamMask(..., _threshold = 0.6)` — each takes `(const WavefieldData&, _x, _y, _t)`.

When `simulation` is null they return **still-water** defaults (elevation 0,
velocity 0, normal +Z, Jacobian 1, foam 0), so a consumer that runs before the
field exists degrades gracefully.

### 3.6 Sea-state helpers

`SeaStateSpec { significantWaveHeight, peakPeriod, windSpeed }`,
`SeaStateFromCode(int _code, SeaStateSpec &_out, double _g)`,
`WaveParameters WithSeaState(const WaveParameters &_p)`, and
`StartupRamp(double _t, double _tau)` ( `1 - exp(-t/τ)`, or 1 when `τ ≤ 0`).

**`<sea_state>` precedence (R4):** when `seaState ≥ 0`, `WithSeaState` overrides
`period` (← peak period) and `gain` (← 1.0, physical Hs); `0` is flat water;
`-1` (default/omitted) means manual via `<period>` + `<gain>`. Every other knob
is independent and applies in both modes.

### 3.7 `WavesSystemBase` — the source-plugin base

`include/gz/sim/systems/WavesSystemBase.hh`. A `System` +
`ISystemConfigure`/`ISystemPreUpdate`/`ISystemReset` that does all the
producer plumbing so each engine's plugin is tiny. Subclasses override two
protected virtuals:

```cpp
protected: virtual std::string EngineToken() const = 0;                 // "fft", "gerstner", …
protected: virtual std::shared_ptr<waves::IWaveField>
             MakeEngine(const waves::WaveParameters &_params) const = 0; // build + SetParameters
```

The base: parses `<update_rate>` + the `<wave>` block, reads world `<gravity>`,
builds the engine, writes the `Wavefield` component (with `generation = 1`), and
advertises **`/world/<name>/wave/set_parameters`** (a `gz.msgs.Param` →
`gz.msgs.Boolean` service). `PreUpdate` drains queued parameter updates
(bumping `generation`), re-points the engine if an ECM deserialize cleared it,
and throttles `Update()` to `updateRate`. The service applies on the ECM thread
(updates are queued from the transport thread), so runtime reconfiguration (R5)
is race-free.

**One parameter table, four uses.** A single `GZ_WAVES_PARAM_TABLE` (in
`Wavefield.hh`) lists every `WaveParameters` field once and drives all four
sites that must agree: the struct defaults, the serialization order, the SDF
`<wave>` parsing, and the `set_parameters` service keys. Adding a parameter is a
one-line table edit — no risk of the four falling out of sync. (`gravity` is the
deliberate exception: it is sourced from the world's `<gravity>`, not SDF or the
service, and streamed explicitly so it still round-trips.)

*Tests:* `wave_core_test.cc` covers Eval, the registry, `Wavefield`
serialization, the sea-state helpers, and the full Configure/PreUpdate/Reset +
`set_parameters` path through a real `EntityComponentManager`, using a stub
engine (no engine dependency, no cycle).

---

## 4. Gerstner engine (`gz_waves_provider_gerstner`)

- **Engine** `GerstnerWaveSimulation` (`MakeGerstnerWaveField()` factory): a
  closed-form sum of up to N Gerstner components; deterministic; spatially
  unbounded (`Bounds()` ⇒ `nullopt`). It also samples itself onto an N×N tile
  each `Update` and exposes it via `Field()` so it renders through the same path
  as the FFT engine.
- **Seamless tiling (N5).** The render tile is sized to the longest component
  wavelength, then **every component's wave vector is quantized to the tile
  lattice** (`k` snapped to integer multiples of `k0 = 2π/tile`), so each
  component completes a whole number of cycles over the tile and the surface
  tiles with no edge seams. `ω` and the steepness clamp are recomputed from the
  snapped wavenumber.
- **Source system** `gz-sim-waves-gerstner-system` (`GerstnerWaves : WavesSystemBase`):
  `EngineToken()` ⇒ `"gerstner"`, `MakeEngine()` ⇒ builds + configures the engine.
  Its doc block documents the Gerstner SDF surface: `<model>` (PMS/CWR),
  `<number>`, `<period>`, `<amplitude>` (CWR), `<direction>`, `<angle>`,
  `<scale>`, `<steepness>`, `<phase>`, `<tau>`, `<gain>`, `<sea_state>`.
- **GUI registrar** `gz-sim-waves-gerstner-gui` (see §6.2).
- **Robustness.** A non-positive `<period>` is rejected (logs via `gzerr` and
  leaves the field flat) rather than dividing by zero into `NaN`/`Inf` through
  `2π/period` and the `1/ω⁵` spectrum tail.
- *Tests:* `gerstner_test.cc` covers elevation, normals, particle velocity,
  dispersion, sea-state scaling, serialization round-trip, the ramp, and the
  period guard.

---

## 5. Rendering (`gz_waves_rendering`)

### 5.1 `WaterVisual` — the renderer

A `System` + `ISystemConfigure`/`ISystemPreUpdate`, loaded as
`gz-sim-water-visual-system` (class `gz::sim::systems::WaterVisual`). It reads
the `Wavefield` component from the world entity; if the component hasn't
replicated yet it does a one-shot **pull-on-ready** request to
`/world/<name>/state` and seeds the recipe from that snapshot, so a GUI that
joins late still gets the field.

**Thread-safety design (N2) — the central decision.** `WaterVisual` builds and
**owns a private** engine via `CreateWaveSimulation(data->algorithm, data->params)`;
it deliberately does **not** alias the component's `simulation`. From the code:

> *"We deliberately do NOT alias `data.simulation`: that is a process-global
> engine shared by every consumer (gz-sim's GuiRunner loads this system twice,
> and the deserialize path drives the same instance on another thread). The
> per-instance `mutex_` below cannot serialise a shared instance, so sharing it
> races `OnSceneUpdate`'s `Update()`/`Field()` on the render thread against this
> thread — the cause of the intermittent `PreUpdate` segfault. A private
> instance is touched only by `PreUpdate` + `OnSceneUpdate`, both under
> `mutex_`, so it is fully serialised."*

It rebuilds the private engine when `data.generation` changes, and each render
frame pulls the grid via `Field()` and uploads it. The column-major
`WaveField2D` grids flow to the Ogre bridge unchanged; the bridge's texel pack
maps them so texture (u, v) = grid (x_u, y_v), preserving the physics
orientation end to end.

### 5.2 Engine-agnostic, via per-engine GUI registrars

`gz_waves_rendering` depends on **`gz_waves` only** — no engine `find_package`,
no engine link, no engine `<depend>` (N1). The factory registration that lets
`WaterVisual` rebuild an engine in the GUI lives in **per-engine GUI plugins**
shipped by the provider packages. The `water_surface` model loads them in the
**same `<visual>`** as `WaterVisual`:

```xml
<plugin filename="gz-sim-water-visual-system" name="gz::sim::systems::WaterVisual"> … </plugin>
<plugin filename="gz-sim-waves-gerstner-gui"  name="gz::sim::systems::GerstnerWavesGui"/>
<plugin filename="gz-sim-waves-fft-gui"       name="gz::sim::systems::FftWavesGui"/>
```

They **must** sit at the `<visual>` level: gz-sim's GuiRunner loads SDF system
plugins only at the visual level — model/link-level plugins never reach the GUI
process (loading them there yields a flat/white ocean because the factory is
never registered). One registrar per engine the world might select; adding an
engine never edits the rendering package.

### 5.3 Ogre2 C-ABI bridge (N4)

All Ogre Next access is isolated in a **separate** `libwaves-ogre2-bridge.so`,
loaded on demand via `dlopen`. Linking it directly would put
`libgz-rendering-ogre2.so` in WaterVisual's `DT_NEEDED`, which collides with
gz-rendering's own engine-plugin loader (same SONAME already in process ⇒ the
engine never initialises and the GUI shows a default-material plane). The seam
is a small C-ABI: `waves_ogre2_heightmap_create` / `_upload` / `_ready` /
`_set_tex_filtering` / `_destroy`. `HeightMapTexture` (a pimpl) calls it; the
bridge packs (η, Dx, Dy, foam) into an `RGBA32F` GPU texture bound to the
material's `heightMap` sampler.

### 5.4 Targets, model, world

CMake builds `waves-ogre2-bridge` and `gz-sim-water-visual-system` (the latter
links `${CMAKE_DL_LIBS}`, not the bridge), and probes the Ogre Next header path
to match the resolved `gz-rendering::ogre2`. The `water_surface` model ships the
mesh (`water.dae`), the shaders (`fft_water_vs_330.glsl`, `water_fs_330.glsl`),
and the normal/skybox textures. `vrx_gazebo/worlds/open_water.sdf` is wired as a
self-contained ocean so `ros2 launch vrx_bringup simulation.launch.xml` renders
moving waves with no extra setup (R3, R7).

---

## 6. FFT engine (`gz_waves_provider_fft`)

### 6.1 Engine + system

**VRX vs. EncinoWaves.** For the FFT engine the external **EncinoWaves** library
owns the wave-field *generation* across all five WFE choices: the inverse
transform (IFFT), the spectra/spreading/dispersion models, the random amplitude
and uniform phase sampling (with an evenly spaced frequency grid), and the
horizontal-displacement (Gerstner "chop") kinematics. VRX's `gz_waves_provider_fft` is a **thin wrapper**: it configures
EncinoWaves from the `<wave>` recipe, samples the grid, computes particle
velocity by finite difference (below), calibrates RMS to the target `Hs`, and
exposes the result through `Field()`/`Elevation()`. The contrast with the
in-repo Gerstner engine, whose kinematics live in VRX, is deliberate, and
illustrates that the `IWaveField` boundary lets an engine externalize as much or
as little of the generation as it likes.

- **Engine** `FFTWaveSimulation` (`MakeFFTWaveField()`): an inverse-FFT of an
  empirically-modelled directional spectrum from the external **EncinoWaves**
  library (Horvath 2015, Apache-2.0). `Update` propagates the spectrum and runs
  the IFFT once per tick (cached by time, so multiple consumers share one FFT);
  output is calibrated so the RMS matches the Pierson–Moskowitz `Hs`. Periodic
  over `tileSize`; `Elevation` bilinearly samples the grid; `Field()` exposes
  height + x/y displacement + a folding (foam) metric.
- **Particle velocity** is computed by **time finite-difference**: EncinoWaves
  exposes no analytic velocity field, so a scratch state is propagated a small
  `dt` ahead and the displacement grids differenced (∂Dx/∂t, ∂Dy/∂t, ∂η/∂t).
  This happens **lazily**, on the first `ParticleVelocity` call after an
  `Update` — the extra propagation roughly doubles the per-tick cost, so
  consumers that never query velocity (the renderer reads `Field()` only)
  never pay it.
- **Selectable spectral models** (SDF string → Encino enum): spectrum
  `pms`/`pm`, `jonswap`, `tma`; spreading `poscos2`/`poscossqr`, `mitsuyasu`,
  `hasselmann`, `donelanbanner`/`donelan`; dispersion `deep`,
  `finite`/`finite_depth`, `capillary`. A smooth band-pass (or notch) filter is
  enabled by the `<filter_*>` wavelength parameters (see §8).
- **Source system** `gz-sim-waves-fft-system` (`FftWaves : WavesSystemBase`):
  `EngineToken()` ⇒ `"fft"`. Its doc block carries the full FFT SDF table and the
  `<sea_state>` precedence section (see §3.6 / §8). Note `<direction>` is parsed
  but not yet applied (EncinoWaves assumes wind along +X).
- *Tests:* `fft_test.cc` covers finiteness/bounds, ramp-up, foam from the
  Jacobian, band-pass reshaping, spectrum selection, time evolution, `Field()`
  views, determinism vs. seed, periodicity, unit normals, particle velocity, and
  sea-state Hs.

### 6.2 GUI registrar pattern (per engine)

A GUI registrar (`gz-sim-waves-<engine>-gui`) is a **bare `System` with no
`ISystem` interface and no SDF**. It registers its factory from a **file-scope
static initializer** that runs the moment the GUI `dlopen`s the library:

```cpp
namespace {
const bool kFftGuiRegistered = []{
  gz::sim::waves::RegisterWaveEngineFactory("fft", &gz::sim::waves::MakeFFTWaveField);
  return true;
}();
}  // namespace
GZ_ADD_PLUGIN(gz::sim::systems::FftWavesGui, gz::sim::System)
```

The empty `System` body exists only so gz-sim has a plugin to load (which
triggers the `dlopen`). Carrying no `Configure`/SDF deliberately avoids gz-sim's
empty-plugin SDF re-parse warning.

### 6.3 EncinoWaves dependency

`gz_waves_provider_fft` does `find_package(EncinoWaves REQUIRED)` and links
`EncinoWaves::EncinoWaves` (pulling in Eigen3 / TBB / Imath). EncinoWaves is an
external **system** package (installed from `HonuRobotics/encinowaves`, not
vendored); it has no rosdep key, so `package.xml` lists its transitive system
deps (`libtbb-dev`, `libimath-dev`) rather than EncinoWaves itself.

---

## 7. Cross-cutting design decisions (rationale)

| Decision | Why |
|----------|-----|
| **One ECM component as the only coupling.** | Producers and consumers never link each other; the field is shared and consistent (R2, N1). |
| **Recipe-only serialization; engine never replicated.** | The engine is process-local state; replicating a recipe + rebuilding via the registry keeps consumers engine-agnostic and decouples server/GUI (N1, N3, R6). |
| **Token → factory registry.** | Lets the renderer build an engine it does not link; new engines self-register (N1). |
| **Per-engine GUI registrar at the `<visual>` level.** | The renderer stays core-only; the engine dependency lives in the registrar. Visual-level is the only level the GuiRunner loads systems at. |
| **WaterVisual owns a private engine.** | The component's engine is shared/multi-threaded in the GUI; a private, mutex-guarded instance removes the render-vs-ECM race (N2). |
| **Ogre behind a dlopen'd C-ABI.** | Keeps `libgz-rendering-ogre2` out of WaterVisual's `DT_NEEDED` so gz-rendering's own engine loader still works (N4). |
| **`Eval` free functions, null-safe.** | Consumers never touch `IWaveField`; pre-field ticks degrade to still water (N3). |
| **Gerstner wave-vector quantization.** | Makes every component periodic over the tile ⇒ seamless tiling (N5). |
| **`generation` counter.** | Cheap change detection so consumers rebuild only on actual reconfiguration. |
| **`WavesSystemBase` holds the plumbing.** | Each engine's source plugin is two small overrides; the SDF parse + service + throttle live once. |

---

## 8. Configuration reference

### Selecting the engine

In the world, load **one** source system and the matching GUI registrar:

```xml
<!-- server: pick the engine -->
<plugin filename="gz-sim-waves-fft-system" name="gz::sim::systems::FftWaves">
  <update_rate>30</update_rate>
  <wave>
    <sea_state>5</sea_state>   <!-- one-knob; or comment it for manual mode -->
  </wave>
</plugin>
```

The `water_surface` model already loads a GUI registrar for each engine, so
whichever engine the world selects renders. `open_water.sdf` ships the
alternative engines as commented `<plugin>` blocks — switch by commenting the
active block and uncommenting the one you want.

### Shared `<wave>` parameters (all engines)

`<sea_state>` (int 0–9, default −1/off — overrides `<period>`+`<gain>`),
`<period>` [s], `<gain>`, `<direction>` [rad], `<tau>` [s], plus `<update_rate>`
[Hz] at plugin level. (Gerstner additionally: `<model>`, `<number>`,
`<amplitude>`, `<angle>`, `<scale>`, `<steepness>`, `<phase>`.)

### FFT-specific `<wave>` parameters

| Tag | Type | Default | Meaning |
|-----|------|---------|---------|
| `<tile_size>` | double [m] | 200.0 | Periodic tile extent per axis. |
| `<grid_size>` | uint | 128 | Grid samples/axis (power of two). |
| `<seed>` | uint | 0 | Spectrum RNG seed. |
| `<choppiness>` | double | −1.0 | Tessendorf horizontal-displacement multiplier (~[−2, 0]). |
| `<spectrum>` | string | `tma` | `pms`, `jonswap`, `tma`. |
| `<spreading>` | string | `hasselmann` | `poscos2`, `mitsuyasu`, `hasselmann`, `donelanbanner`. |
| `<dispersion>` | string | `capillary` | `deep`, `finite`, `capillary`. |
| `<depth>` | double [m] | 100 | Dispersion input. |
| `<fetch>` | double [km] | 300 | Spectrum input. |
| `<swell>` | double | 0 | Swell elongation. |
| `<trough_damping>` | double [0,1] | 0 | Breaking-wave trough damping. |
| `<filter_min_wl>` / `<filter_max_wl>` / `<filter_soft>` / `<filter_min>` / `<filter_invert>` | — | 0 / 0 / 0 / 0 / false | Band-pass (or notch) on wavelength. |

### Runtime reconfiguration

Call `/world/<name>/wave/set_parameters` (`gz.msgs.Param` → `gz.msgs.Boolean`)
with any of the parameter keys (snake_case for multi-word). The change bumps
`generation`; consumers (incl. WaterVisual) rebuild automatically.

---

## 9. How to contribute a new wave field engine

A new wave field engine is a fresh `IWaveField` implementation. The renderer,
the buoyancy consumer, and the core need **zero
changes** — you add one self-contained provider package, mirroring
`gz_waves_provider_gerstner` (the deliberately-minimal template). The same
package contributes its server source plugin, its GUI registrar, and its world
wiring.

### Step 1 — Create the package

```
gz_waves_provider_<name>/
  CMakeLists.txt
  package.xml
  include/gz/sim/waves/<Name>WaveSimulation.hh
  src/<Name>WaveSimulation.cc     # the IWaveField engine + factory
  src/<Name>WavesSystem.cc        # gz-sim-waves-<name>-system  (server source)
  src/<Name>WavesGui.cc           # gz-sim-waves-<name>-gui      (GUI registrar)
  test/<name>_test.cc
```

Use the Honu Robotics Apache-2.0 header on every file.

### Step 2 — Implement `IWaveField` + a factory

Subclass `IWaveField` (`#include "gz/sim/waves/WaveSimulation.hh"`) and implement
`Elevation`, `ParticleVelocity`, `Normal`, `Jacobian`, `SetParameters`, `Kind()`
(return your token), and — to be renderable — `Update` + `Field()`. To render,
`SetParameters` should allocate column-major N×N buffers and bind them into a
`WaveField2D field_`; `Update` fills them for the given time; `Field()` returns
`&field_`. Read whatever subset of `WaveParameters` you need (apply
`WithSeaState`/`StartupRamp` for consistent sea-state and ramp behavior). Export
the factory:

```cpp
std::shared_ptr<IWaveField> Make<Name>WaveField();   // default-constructed engine
```

### Step 3 — Source system (server)

```cpp
#include "gz/sim/systems/WavesSystemBase.hh"
#include "gz/sim/waves/<Name>WaveSimulation.hh"
#include <gz/plugin/Register.hh>

namespace gz::sim::systems {
class <Name>Waves : public WavesSystemBase {
  // Documentation inherited
  protected: std::string EngineToken() const override { return "<name>"; }
  // Documentation inherited
  protected: std::shared_ptr<waves::IWaveField>
    MakeEngine(const waves::WaveParameters &_params) const override {
      auto e = waves::Make<Name>WaveField();
      e->SetParameters(_params);
      return e;
    }
};
}  // namespace gz::sim::systems
GZ_ADD_PLUGIN(gz::sim::systems::<Name>Waves, gz::sim::System,
              gz::sim::systems::<Name>Waves::ISystemConfigure,
              gz::sim::systems::<Name>Waves::ISystemPreUpdate,
              gz::sim::systems::<Name>Waves::ISystemReset)
GZ_ADD_PLUGIN_ALIAS(gz::sim::systems::<Name>Waves, "gz::sim::systems::<Name>Waves")
```

Document your SDF parameter surface in the class doc comment (and the
`<sea_state>` precedence note), the way the FFT/Gerstner systems do.

### Step 4 — GUI registrar

A bare `System`, **no** `ISystem`, **no** SDF; register the factory from a
file-scope static initializer (so it runs at `dlopen`):

```cpp
#include "gz/sim/System.hh"
#include "gz/sim/waves/<Name>WaveSimulation.hh"
#include "gz/sim/waves/WaveSimulation.hh"
#include <gz/plugin/Register.hh>

namespace gz::sim::systems { class <Name>WavesGui : public System {}; }
namespace {
const bool k<Name>GuiRegistered = []{
  gz::sim::waves::RegisterWaveEngineFactory(
      "<name>", &gz::sim::waves::Make<Name>WaveField);
  return true;
}();
}  // namespace
GZ_ADD_PLUGIN(gz::sim::systems::<Name>WavesGui, gz::sim::System)
GZ_ADD_PLUGIN_ALIAS(gz::sim::systems::<Name>WavesGui, "gz::sim::systems::<Name>WavesGui")
```

### Step 5 — CMake + package.xml (mirror Gerstner)

Build **three** libraries — the engine (exported, no `GZ_ADD_PLUGIN`), the
`-system` plugin, and the `-gui` plugin — each linking
`gz_waves::gz_waves gz-waves-provider-<name> gz-sim::core gz-plugin::register`
(the engine links its own math deps PUBLIC). `package.xml` depends on
`gz_waves`, `gz_sim_vendor`, `gz_plugin_vendor` (+ any math/3rd-party deps your
engine needs). See `gz_waves_provider_gerstner/CMakeLists.txt` verbatim as the
template — copy it and rename.

### Step 6 — Tests

Add `<name>_test.cc`: link `gz_waves::gz_waves` + your engine library, and check
the registry route (`CreateWaveSimulation("<name>", …)` → `Kind() == "<name>"`),
`SetParameters`/sampling, serialization round-trip, and any engine-specific math.

### Step 7 — Wire it into a world

Add the GUI registrar next to `WaterVisual` in the `water_surface` model's
`<visual>` (so the renderer can rebuild it):

```xml
<plugin filename="gz-sim-waves-<name>-gui" name="gz::sim::systems::<Name>WavesGui"/>
```

Select it in the world by loading `gz-sim-waves-<name>-system` as the wave
source. Done — `WaterVisual`, buoyancy, and the core are untouched.

### Checklist

- [ ] `IWaveField` fully implemented; `Field()` returns a valid column-major,
      periodic grid (if renderable).
- [ ] `Make<Name>WaveField()` exported; `Kind()`/`EngineToken()` agree on the token.
- [ ] Three libraries build; engine carries **no** `GZ_ADD_PLUGIN`.
- [ ] GUI registrar is interface-less + SDF-less (no re-parse warning).
- [ ] GUI registrar added to the `water_surface` `<visual>`.
- [ ] Tests pass; the engine renders in `gz sim -r` (waves move, seamless tiling).
- [ ] Gazebo house style: `_`-prefixed parameters, `//////` separators before
      each out-of-line definition/test, per-member access specifiers, Doxygen
      `\brief`/`\param`/`\return`, Honu Robotics copyright header.

---

## 10. Build, run, test

### Prerequisites

- ROS Lyrical with Gazebo Jetty: the target platform, and currently the most
  modern stable ROS and Gazebo combination. If your system defaults to a
  different Gazebo version, install Gazebo Jetty before building.
- EncinoWaves installed and on CMAKE_PREFIX_PATH (from HonuRobotics/encinowaves),
  required by the FFT package.
- Remaining Gazebo dependencies via rosdep install --from-paths src --ignore-src -y.
- A real GPU for the GUI: the Ogre2 render path does not initialise under software GL.
- Container setup: TBD.

```bash
# Build (EncinoWaves must be installed and on CMAKE_PREFIX_PATH for the FFT package)
cd ~/vrx_ws
colcon build --merge-install

# Run (FFT or Gerstner per open_water.sdf)
source install/setup.bash
ros2 launch vrx_bringup simulation.launch.xml
#   or directly:  gz sim -r src/vrx/vrx_gazebo/worlds/open_water.sdf

# Tests
./build/gz_waves/wave_core_test
./build/gz_waves_provider_gerstner/gerstner_test
./build/gz_waves_provider_fft/fft_test
```

---

## 11. Status notes

- `<direction>` is parsed by the FFT system but not yet applied (EncinoWaves
  assumes wind along +X).
- The FFT engine hands sim time to EncinoWaves in single precision (float
  API), so on multi-hour runs the float grid coarsens and gradually degrades
  the wave animation and the particle-velocity finite difference.
- Foam/whitecaps are effectively **FFT-only**. The renderer derives foam from
  the engine's folding (Jacobian) metric; the analytic Gerstner engine's
  Jacobian barely leaves 1.0, so it carries no usable folding signal and the
  shader's foam pass is skipped for it.
- The wave-coupled `test_buoy` demo vessel and the `WaveBuoyancy` consumer ship
  with `gz_waves_buoyancy`, which also adds the buoy to `open_water.sdf`.
