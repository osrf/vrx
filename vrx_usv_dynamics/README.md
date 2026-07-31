# vrx_usv_dynamics

Surface-vessel dynamics for the BlueBoat sandbox. Two gz-sim systems, ported
from VRX 3's `vrx_gz` package:

| Plugin `filename=` | `name=` | What it does |
|---|---|---|
| `vrx-surface-system` | `vrx::Surface` | Per-point circle-segment buoyancy, sampled against the wave field. |
| `vrx-simple-hydrodynamics-system` | `vrx::SimpleHydrodynamics` | Fossen added-mass, Coriolis and linear/quadratic drag. |

## Retire path for `Surface` — read this before extending it

`Surface` is **interim**. `WAVES_DESIGN.md` §11 and `AGENTS.md` both describe a
`gz_waves_buoyancy` package providing a `WaveBuoyancy` consumer, which is the
intended long-term home for buoyancy on this branch. It had not landed when the
BlueBoat sandbox was ported, and "float and drive" is blocked without *some*
buoyancy, so this port fills the gap.

It is deliberately structured so that retiring it is a deletion rather than a
refactor:

- The package is **not** named `gz_waves_buoyancy`, so there is no collision
  when the real one lands.
- `Surface` is its own `add_library` target in its own source file, sharing no
  code with `SimpleHydrodynamics`.
- It is the **only** thing here that links `gz_waves`.
- Per-model configuration is one self-contained `<plugin>` block, so switching
  a model over is a localized SDF edit.

To retire: delete `src/Surface.{cc,hh}`, the `vrx-surface-system` target and its
`install()` entry, and the `find_package(gz_waves)` line in `CMakeLists.txt`.
Then swap the `<plugin filename="vrx-surface-system">` block in each model for
whatever `gz_waves_buoyancy` provides.

One caveat: the two buoyancy models are unlikely to be parameter-compatible.
`Surface` uses discrete points with a circle-segment hull approximation
(`<hull_length>`, `<hull_radius>`, `<points>`), and the BlueBoat's values were
tuned for it. Budget a re-tune, not a drop-in swap.

## How `Surface` couples to the wave field

It follows the consumer contract in `WAVES_DESIGN.md` §3.5:

- Reads `components::Wavefield` from the **world** entity each tick.
- Queries elevation through `gz::sim::waves::SurfaceElevation`, one of the
  null-safe `Eval` free functions — never an engine header.
- Does **not** call `waves::Advance`; the world's wave source system owns
  advancing the field, throttled to its `<update_rate>`.
- Does **not** watch the `generation` counter; it caches nothing derived from
  the recipe, so `set_parameters` takes effect on the next tick for free.

Note that `SurfaceElevation` returns elevation above still water, positive up.
The VRX 3 function this replaced was named `ComputeDepthSimply` but returned the
same quantity with the same sign, so the buoyancy arithmetic is unchanged.

## Wave engine choice matters for buoyancy

Prefer the **Gerstner** engine for anything that floats.
`FFTWaveSimulation::Elevation` ignores its time argument and samples the grid
last built by the throttled (30 Hz) `Advance`, while physics runs at 250 Hz — so
buoyancy would sample a stale field. Gerstner's `Elevation` is analytic in `t`.
