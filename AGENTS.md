# AGENTS.md

Operational guide for AI agents working in this repository. Read this first, then
the deeper design docs it points to. This file applies to the whole repo; if a
subdirectory adds its own `AGENTS.md`, the closest one to the file you are
editing wins.

VRX (Virtual RobotX) is a Gazebo + ROS 2 maritime simulation. The substantive
subsystem today is the **wave simulation packages** (`gz_waves*`); more
subsystems will be added over time.

## Project layout

| Path | What |
|------|------|
| `gz_waves/` | Engine-agnostic wave **core** (the `IWaveField` contract, engine registry, `Wavefield` ECM component, `Eval` facade, `WavesSystemBase`). |
| `gz_waves_provider_gerstner/`, `gz_waves_provider_fft/` | Wave field **engines** (an `IWaveField` implementation + its server source plugin + a GUI registrar). |
| `gz_waves_rendering/` | Engine-agnostic **renderer** (`WaterVisual`) + the Ogre2 C-ABI bridge + the `water_surface` model. |
| `gz_waves_buoyancy/` | A wave-field **consumer** (`WaveBuoyancy`). |
| `vrx_gazebo/` | Worlds (`open_water.sdf`) + resource-path hooks. |
| `vrx_bringup/` | ROS 2 launch + `ros_gz_bridge` config. |
| `WAVES_DESIGN.md` | Full design + contributor reference for the wave packages. |

## Setup & build

ROS 2 Rolling + Gazebo Jetty (+ matching `ros_gz`). The workspace is `~/vrx_ws`,
this repo is `~/vrx_ws/src/vrx`.

```bash
cd ~/vrx_ws
colcon build --symlink-install
source install/setup.bash
```

The FFT wave engine depends on **EncinoWaves**, an external system library
(installed separately, not vendored). It must be discoverable via
`CMAKE_PREFIX_PATH` (e.g. installed under `~/.local`) or
`gz_waves_provider_fft` will fail to configure.

## Run

```bash
ros2 launch vrx_bringup simulation.launch.xml          # boots open_water.sdf with a moving ocean
gz sim -r src/vrx/vrx_gazebo/worlds/open_water.sdf      # world directly
```

## Test — run before declaring work done

```bash
colcon test --packages-select <pkg> && colcon test-result --verbose
# or run a gtest binary directly:
./build/gz_waves/wave_core_test
./build/gz_waves_provider_gerstner/gerstner_test
./build/gz_waves_provider_fft/fft_test
```

Render-path changes cannot be verified headless — they need a real `gz sim -r`
on a GPU. State plainly when a change is build-only and GUI-unverified.

## Code style (Gazebo house style)

Match the surrounding code. For C++:

- **`_`-prefix every function/method parameter** (`void Foo(int _x)`); loop and
  local variables are exempt.
- A **50-slash separator** `//////////////////////////////////////////////////`
  before each out-of-line function/method definition **and** before each
  `TEST`/`TEST_F`.
- **Per-member access specifiers** — prefix every member/method with
  `public:` / `private:` / `protected:` (Gazebo style, not one block per group).
- **Doxygen** on declarations: `\brief` above each class/struct member (not a
  trailing `///<`), `\param`/`\return` on functions, `// Documentation inherited`
  on overrides.
- **Copyright header**: new files use the **Honu Robotics** Apache-2.0 header;
  when editing an existing file, keep its existing header.

## Commit & PR conventions

- Branch off the default branch; do not commit directly to it.
- Recommended one-line, imperative commit subject (`package: do the thing`).
- Sign off (`git commit --signoff`) and end the message with the trailer:
  `Co-Authored-By:`

## Critical invariants (do not regress these)

These are non-obvious and have each caused real bugs. See `WAVES_DESIGN.md` for
the full rationale (section refs below).

- **Core and renderer stay engine-agnostic.** `gz_waves` and
  `gz_waves_rendering` must not build-depend on, link, or include any concrete
  engine. Adding an engine must not modify them. (§2, §7)
- **The engine is never serialized.** Replication carries the *recipe* only;
  each consumer rebuilds its engine via `CreateWaveSimulation`. (§3.4)
- **`WaterVisual` owns a private engine.** Never alias the component's
  `simulation` into the renderer — the render thread and ECM thread race it and
  it segfaults. Build a private, mutex-guarded instance. (§5.1, N2)
- **GUI registrars load at the `<visual>` level.** gz-sim's GuiRunner loads
  system plugins only at `<visual>`; model/link/world-level never reach the GUI
  process → flat/white ocean. Put `gz-sim-waves-<engine>-gui` next to
  `WaterVisual` in the `water_surface` model's `<visual>`. (§5.2)
- **Ogre stays behind the dlopen'd C-ABI bridge.** Never link
  `gz-rendering-ogre2` into `WaterVisual` — the resulting `DT_NEEDED` breaks
  gz-rendering's own engine loader (default-material plane / RTF collapse). (§5.3, N4)
- **Consumers query via `Eval` free functions**, never via `IWaveField`
  directly; the engine type stays opaque. (§3.5, N3)

## Adding a new wave engine

A new engine is one self-contained `gz_waves_provider_<name>` package (engine +
source system + GUI registrar); the core, renderer, and consumers need **zero
changes**. Follow the step-by-step recipe in **`WAVES_DESIGN.md` §9**, using
`gz_waves_provider_gerstner/` as the template.

## Deeper docs

- `WAVES_DESIGN.md` — wave-package requirements, architecture, per-package detail,
  and the "add a new engine" guide.
- `README.md` — user-facing build/run summary.
