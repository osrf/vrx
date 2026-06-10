# Plan: Integrating Waves, Wind, and Currents into Gazebo

This document describes the plan for building a generic ocean simulation layer
(wave field, wind field, current field, wave-aware buoyancy and hydrodynamics,
water rendering) and contributing it to upstream Gazebo. VRX is the
first consumer but the work is not VRX-specific.

The plan is the synthesis of a design discussion. Decisions marked **open**
have an explicit trade-off that needs a call before code lands.

> **Status update.** The **wave-field** layer described here is now implemented in
> VRX — see [`wave_design_reference.md`](wave_design_reference.md) (the current
> system) and [`wave_provider_architecture.md`](wave_provider_architecture.md)
> (the architecture, as built). Several "open" decisions below are
> resolved: the wave abstraction is the virtual `IWaveField` interface (virtual
> dispatch, **not** a concrete-struct representation — see §4); the visual vertex shader
> shipped as `fft_water_vs_330.glsl`; and the code is split into `gz_waves` (core)
> + `gz_waves_provider_{fft,gerstner}` (engines) + `gz_waves_rendering` (visual).
> The **wind** and **current** field layers remain future work, so this plan is
> kept as the forward-looking roadmap for those.

---

## 1. Goals

1. Move the wave/wind/current/water-rendering layer **upstream into `gz-sim`**
   (and possibly `gz-rendering` for shader assets). VRX keeps only the
   things that are genuinely VRX-specific: the WAM-V model, competition tasks,
   scenario worlds, and bringup.
2. Develop the code in VRX first, in upstream-shaped form, so the migration
   into `gz-sim` is mechanical (`git mv` for new files, a small reviewable
   diff for modifications to existing files).
3. Use `asv_wave_sim` as the reference for the wave-math substrate and the
   ECM-component pattern. Improve where the existing design has known issues.

## 2. Guiding principles

1. **One writer, many readers.** All shared environmental state (wave field,
   wind, currents) lives as a Gazebo ECM component on the world entity. One
   system owns the writes; everyone else queries via the ECM. No transport
   topics for shared state. No per-plugin parameter replicas.
2. **Same math everywhere.** The wave evaluation function used by the visual
   shader and the buoyancy plugin is the *same* function. No "physics uses
   Airy / visual uses Gerstner" divergence.
3. **Visual fidelity ≠ physics fidelity.** Physics samples the canonical wave
   field at body points. The visual layer adds richness (foam, refraction,
   lighting) that does not feed back to physics.
4. **Minimum boilerplate.** Each stage produces something runnable. Don't
   build the cathedral before the chapel. YAGNI by default.
5. **Upstream-shaped from day one.** VRX code uses upstream namespaces,
   header paths, and registered IDs. Migration is file movement, not
   refactoring.

## 3. Target architecture (in `gz-sim`)

```
gz-sim/
├── include/gz/sim/
│   ├── components/
│   │   ├── Wavefield.hh        (new)
│   │   ├── Current.hh          (new)
│   │   └── Wind.hh             (revised — already exists, may grow)
│   └── waves/
│       ├── Eval.hh             (new — free functions: Elevation, ParticleVelocity,
│       │                              Normal, Jacobian)
│       └── WaveSpectrum.hh     (new — PMS, JONSWAP, ECKV; pure math)
│
└── src/systems/
    ├── waves/                  (new — wave field manager system)
    │   ├── Waves.{cc,hh}
    │   ├── GerstnerWaveSimulation.{cc,hh}
    │   └── CMakeLists.txt
    ├── currents/               (new — current field manager)
    ├── wind/                   (rework of `wind_effects`)
    │   ├── Wind.{cc,hh}                Writes WindComponent
    │   └── WindEffects.{cc,hh}         Per-vessel drag, reads WindComponent
    ├── water_visual/           (new — rendering system)
    │   ├── WaterVisual.{cc,hh}
    │   ├── shaders/
    │   │   ├── fft_water_vs_330.glsl
    │   │   └── water_fs_330.glsl
    │   └── CMakeLists.txt
    ├── buoyancy/               (existing — extended to read Wavefield)
    └── hydrodynamics/          (existing — extended to read Wavefield + Current)
```

`water_visual` lives under `src/systems/` because "system" in `gz-sim` is a
lifecycle category (anything implementing `ISystemConfigure`/`PreUpdate`),
not a domain category. Precedents: `sensors/`, `scene_broadcaster/`,
`particle_emitter/` all live there.

## 4. Wave model abstraction (resolved)

The wave interface supports multiple backends (Gerstner analytic, FFT spectral).
It shipped as a **virtual `IWaveField`** — concrete backends implement it and the
`Eval::*` free functions are one-line delegators. This was chosen over a concrete
`WavefieldData` struct (array-of-components, header-inlined `Eval`) because a
virtual interface is **ABI-safe**: a new backend is a new class, with no struct
layout change and no wait for a major-version bump. The ~5% per-call vtable cost
is negligible (sub-microsecond per sample), and the `Eval::*` signatures are the
same either way, so consumers are unaffected by the representation. (A
`std::variant` of backends, or a pimpl with public accessors, were rejected —
both ABI-break when a backend is added and preclude separate-library backends.)

See [`wave_provider_architecture.md`](wave_provider_architecture.md) for the
shipped interface.

## 5. Components

### `Wavefield`

```cpp
namespace gz::sim::components {
  using Wavefield = Component<WavefieldData, class WavefieldTag>;
  GZ_SIM_REGISTER_COMPONENT("gz.sim.components.Wavefield", Wavefield)
}
```

Attached to the world entity. Written by the `Waves` system. Read by
`Buoyancy`, `Hydrodynamics`, `WaterVisual`, and any other wave-aware code.

### `Current`

Parallel to `Wavefield`. Represents ocean currents as a `CurrentData` struct
(constant, depth-varying, or time-varying — designed to grow). Buoyancy and
Hydrodynamics optionally read it for additional water velocity.

### `Wind`

Replaces the loose wind parameters scattered across `wind_effects`. Component
holds mean velocity, optional turbulence intensity, optional gust model.
Read by `WindEffects` (per-vessel air drag), and optionally by `Waves`
(when `<couple_to_wind>` is enabled, the wave spectrum amplitude and
direction follow wind).

## 6. Eval free functions

```cpp
// gz-sim/include/gz/sim/waves/Eval.hh
namespace gz::sim::waves {

double          SurfaceElevation(const WavefieldData &wf, double x, double y, double t);
Eigen::Vector3d ParticleVelocity(const WavefieldData &wf, double x, double y, double t);
Eigen::Vector3d Normal(const WavefieldData &wf, double x, double y, double t);
double          Jacobian(const WavefieldData &wf, double x, double y, double t);

// Foam intensity derived from Jacobian; useful for visual and (rarely) physics.
double          FoamMask(const WavefieldData &wf, double x, double y, double t);

}  // namespace gz::sim::waves
```

These are null-safe one-line delegators to the backing `IWaveField`
(`wf.simulation`); `WaveBuoyancy` consumes `SurfaceElevation`.

## 7. Buoyancy and Hydrodynamics upgrades

Additive changes to existing `gz-sim` systems. Behavior with no `Wavefield`
component present is exactly today's behavior — no regression risk.

### `Buoyancy`

```cpp
// PreUpdate, after computing existing graded-buoyancy contribution:
auto *wfComp = ecm.Component<components::Wavefield>(worldEnt);
if (wfComp) {
  // For each sample point in the link's collision geometry:
  //   eta = waves::SurfaceElevation(wfComp->Data(), x, y, t);
  //   waterLevelAtSample = fluid_level + eta;
}
```

Sample points are derived from the link's collision geometry (cylinder, box,
sphere have closed-form `SubmergedVolume(depth)`; mesh collisions use a
slower per-triangle path optionally).

This unifies the two competing buoyancy patterns from old VRX (`Surface`
with hardcoded cylinder hulls + `PolyhedraBuoyancyDrag` with single-plane
clipping) into one geometry-driven plugin. Per-sample wave evaluation
captures wave slope across the body, which neither old plugin did.

### `Hydrodynamics`

```cpp
Eigen::Vector3d vWater;  // default zero — still water
auto *wfComp = ecm.Component<components::Wavefield>(worldEnt);
auto *curComp = ecm.Component<components::Current>(worldEnt);
if (wfComp)  vWater += waves::ParticleVelocity(wfComp->Data(), x, y, t);
if (curComp) vWater += currents::Velocity(curComp->Data(), x, y, z, t);

auto vRel = vWater - linkVel;   // drag against relative velocity
```

Currently `Hydrodynamics` is wave-blind; drag uses absolute body velocity.
With this change, drag includes orbital wave currents and explicit ocean
currents — closing the wave-drift loop that the existing plugin can't model.

## 8. Wind coupling

The wind-wave coupling is **opt-in** via SDF flag on the `Waves` system:

```xml
<plugin filename="gz-sim-waves-system" name="gz::sim::systems::Waves">
  <couple_to_wind>true</couple_to_wind>
  <wave>
    <model>PMS</model>
    <!-- period, direction ignored when couple_to_wind is true -->
  </wave>
</plugin>
```

When enabled, the `Waves` system reads the `Wind` component and derives
spectrum amplitude (`~U²` for PM) and direction from wind state.

When disabled, the SDF wave parameters apply unchanged (matches today's
VRX semantics where scenarios specify exact sea state).

Stokes drift (~3% of wind speed) is included in `ParticleVelocity` when
wind coupling is active. This couples wind drag and water-current drag
correctly — a vessel at rest in 10 kt of wind feels both air drag *and*
wind-driven surface current.

## 9. Visual fidelity layer

All CPU-side. None feed back to physics. Stages, in order of impact:

1. **Whitecap foam from Jacobian.** Single biggest visual win. CPU computes
   the Jacobian (already in `WavefieldData` as derivative arrays); shader
   mixes white where `J < threshold`. ~100 LOC.
2. **GGX sun spec lobe.** Add sun-direction uniform; sharp specular peak in
   fragment shader. Water sparkles. ~50 LOC (mostly shader).
3. **Wake foam from vessel motion.** Per-vessel Gaussian stamps into a
   world-space foam texture; fragment shader samples and combines with
   whitecap foam. ~200 LOC.
4. **Depth-aware water color (Beer-Lambert).** Sample seabed depth texture;
   absorb red first, green second, blue last. Shows the seabed in shallow
   water. ~80 LOC.
5. **Screen-space refraction.** Render below-water scene; sample with
   normal-driven offset in water fragment shader. ~200 LOC + pipeline work.

Stop at the level needed for the target scenario.

Optional later: multi-scale wave cascades (multiple `WavefieldComponent`s at
different tile sizes), particle splash at hull contact, GPU FFT backend.

## 10. Naming conventions

| Package / dir | Why this name |
|---|---|
| `waves/` | Generic, upstream-shaped. No `vrx_` prefix because it's not VRX-specific. |
| `currents/` | Parallel. |
| `wind/` | Parallel. |
| `dynamics/` | Vendored copies of upstream Buoyancy/Hydrodynamics with wave additions. |
| `water_visual/` (under `src/systems/`) | Named after the *artifact* rendered (water surface), not the data source (waves). Generalizes if foam, depth, refraction grow into it. |
| `vrx_wamv/` | VRX-specific — WAM-V URDF + meshes. |
| `vrx_tasks/` | VRX-specific — competition scoring. |
| `vrx_gazebo/` | VRX-specific — scenario worlds. |
| `vrx_bringup/` | VRX-specific — launch + bridge config. |

Rule: prefix with `vrx_` if and only if the package is VRX-specific.

If the generic packages are ever published to ROS Index, rename to
`gz_waves`, `gz_currents`, etc., matching Gazebo's library conventions.
The C++ namespaces are already `gz::sim::*` so the rename is cosmetic.

## 11. Migration patterns

### Pattern A — New components and systems

For *new* code (Wavefield, Waves system, Eval, Currents, Wind component,
water_visual): **write in VRX, exactly upstream-shaped**.

- Headers in `vrx_pkg/include/gz/sim/...` — matches upstream include path.
- Namespaces are `gz::sim::components::*`, `gz::sim::systems::*`,
  `gz::sim::waves::*`.
- Component IDs registered as `"gz.sim.components.Wavefield"` (upstream-final).
- Plugin filenames and class names match what will land upstream.

**Migration:** when the upstream PR merges, delete the file from VRX,
bump the `gz-sim` version dependency. Consumer SDFs and `#include`s
unchanged.

### Pattern B — Modifications to existing systems

For *modifications* (Buoyancy, Hydrodynamics, WindEffects): **vendor a copy
of the upstream file with the wave additions**, build as a separate library
to avoid plugin-name collision.

```
dynamics/src/systems/buoyancy/
  Buoyancy.cc        ← upstream's Buoyancy.cc + wave additions
  Buoyancy.hh
  CMakeLists.txt     ← builds libvrx-buoyancy-system.so
```

Class name stays `gz::sim::systems::Buoyancy`; library file gets a different
name to avoid collision with the stock library. SDF references the VRX
library by filename during the bridging period.

**Migration:** when the upstream PR merges, delete the VRX file, change
SDF `filename="vrx-buoyancy-system"` to `filename="gz-sim-buoyancy-system"`.

**Discipline:** keep the diff against upstream minimal. No reformatting, no
incidental refactoring. Track upstream changes by rebasing periodically.
Open the upstream PR early so the diff is reviewable while VRX uses it.

## 12. Staging plan — upstream PRs

| PR | What lands | Pattern | Risk |
|---|---|---|---|
| 1 | `components/Wavefield.hh` + `waves/Eval.hh` + `WaveSpectrum.hh` + tests | A | Low |
| 2 | `systems/waves/` with Gerstner backend | A | Low |
| 3 | `components/Current.hh` + `systems/currents/` + math | A | Low |
| 4 | Buoyancy upgrade — optional Wavefield + Current consumption | B | Medium |
| 5 | Hydrodynamics upgrade — relative-velocity drag | B | Medium |
| 6 | `systems/water_visual/` with Gerstner shader | A | Medium (Ogre quirks) |
| 7 | Wind rework — `components/Wind.hh` + system split; optional wave coupling | A + B | Medium |
| 8 | Foam from Jacobian + foam-mask field in shader | A | Low |

Each PR ships: tests, an example world, header docs, migration notes if
any API changes.

PR 1 should be opened **early** — before much code accumulates in VRX —
so the public API (component shape, free-function signatures, SDF schema)
gets reviewed while the design is still cheap to change.

## 13. Improvements over `asv_wave_sim`

| Aspect | `asv_wave_sim` today | Proposed upstream |
|---|---|---|
| Component discovery | `EntityByComponents(Name("wavefield"))` — hardcoded entity name | `EntityByComponents<Wavefield>()` — find by type |
| Component placement | Child entity of a "waves" model plugin | Directly on the world entity (matches `Gravity`, `MagneticField`) |
| Concurrency | `std::recursive_mutex` in `WavefieldPrivate` | Plain mutex or none (ECM phase serialization) |
| Point queries on FFT | `IWaveField::Elevation` `#if 0`-ed out for FFT | Two-tier interface — analytic-able backends expose point queries; grid-only backends require sampling explicitly |
| Tile periodicity | Inherited by Gerstner backend unnecessarily | Periodicity is a backend property: `Bounds()` returns `nullopt` for Gerstner, tile size for FFT |
| Wind coupling | Wind speed in `WaveParameters` but no wind system | First-class `Wind` component; `Waves` reads it optionally |
| Currents | Not modeled | First-class `Current` component, parallel pattern |
| Foam | Implicit in displacement derivatives | `Eval::FoamMask` is a first-class output of the wave model |
| Hydrodynamics concerns | Physics solver + marker visualization in one struct | Split: physics in `Hydrodynamics`, markers in optional `HydrodynamicsMarkers` |

## 14. What stays in VRX

```
vrx/
├── vrx_gazebo/         Worlds (open water, sydney_regatta, etc.)
├── vrx_bringup/        Launch + ros_gz_bridge config
├── vrx_wamv/           WAM-V URDF, meshes, xacros
└── vrx_tasks/          Competition scoring plugins
```

During the bridging period, VRX also temporarily holds:

```
waves/        currents/        wind/        dynamics/
```

These shrink to nothing as upstream PRs land.

## 15. Bugs fixed during implementation

Issues flagged in the initial analysis of the old VRX wave code, all addressed
in the current system: half-Archimedean buoyancy saturation (the submerged-depth
clamp now spans the full diameter, `[0, 2·radius]`); the render-teardown
re-upload gap (the visual keys uploads off a generation counter and resets on
`OnRenderTeardown`); and hardcoded texture paths (now `<textures>` SDF tags). The
legacy Gerstner vertex-shader y-displacement bug is moot — that shader is gone;
the visual renders every backend from the unified `Field()` height grid with
finite-difference normals.

## 16. Coordination

1. **Rhys Mainwaring** (`asv_wave_sim` author). Long-standing investment in
   this domain. Worth a conversation about either:
   - Co-authoring the upstream proposal (he brings wave-math expertise; we
     bring `gz-sim` integration), or
   - `asv_wave_sim` continuing to host the heavy backends (FFT, Kerner-style
     per-triangle hydrodynamics) while upstream takes the core abstractions.

2. **Gazebo maintainers**. Open a Gazebo Enhancement Proposal (GEP) or
   design discussion in `gz-sim` issues covering the component additions,
   the wave-aware Buoyancy/Hydrodynamics changes, and the dependency
   philosophy (keep `gz-sim` core dep-light; plug heavier backends in
   separate packages).

3. **Downstream users besides VRX**: MBARI's `lrauv-application`, RobotX
   teams, AUV simulation users. The proposed APIs should be useful to
   them; soliciting feedback before locking in interfaces avoids breaking
   changes later.

## 17. Open decisions

Resolved during the wave-field implementation: the backend abstraction is the
virtual `IWaveField` (§4); the component lives directly on the **world entity**;
and spatial extent is a backend property via `Bounds()` (`nullopt` for unbounded
Gerstner, a tile size for FFT — consumers wrap queries outside the tile). Still
open for upstreaming:

1. **`asv_wave_sim` relationship** — depend on it during bridging, or duplicate
   the parts we need? (See §16.)

## 18. Sequencing recap

| Stage | What ships | Where |
|---|---|---|
| 0 | Minimal scaffold (open water world + launch) | VRX (done) |
| 1 | Wavefield component + Eval header | VRX `waves/` → upstream PR 1 |
| 2 | Waves system with Gerstner backend | VRX `waves/` → upstream PR 2 |
| 3 | water_visual rendering plugin | VRX `waves/` → upstream PR 6 |
| 4 | Buoyancy reading Wavefield + primitive vessel | VRX `dynamics/` → upstream PR 4 |
| 5 | Hydrodynamics reading ParticleVelocity + WAM-V | VRX `dynamics/`, `vrx_wamv/` → upstream PR 5 |
| 6 | WindField component + WindDrag + optional coupling | VRX `wind/` → upstream PR 7 |
| 7 | Foam from Jacobian + visual fidelity passes | VRX `waves/` → upstream PR 8 |
| 8 | Currents component + system + Buoyancy/Hydrodynamics integration | VRX `currents/` → upstream PR 3 |

At each stage, what's running in the simulator is functionally complete.
The upstream PRs land asynchronously; VRX-side files are deleted as each
PR ships.
