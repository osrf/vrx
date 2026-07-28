# BlueBoat demo — handoff notes

Working notes for the `bsb/blueboat_demo` branch so a new machine (or a fresh
Claude Code session) can pick up instantly. This is a throwaway testing branch.

## What this branch is

Adding a **Blue Robotics BlueBoat** USV model to VRX and using it to test how the
simulation behaves with different visual/collision meshes (geometry-complexity
sensitivity). The model lives in the `vrx_gz` package and is spawned in the
`nbpark` world.

## Build & run

```bash
rocker --pull --devices /dev/input/js0 --x11 --nvidia --user --home ghcr.io/osrf/vrx-devel:latest /bin/bash
colcon build --merge-install
. install/setup.bash
ros2 launch vrx_gz competition.launch.py world:=nbpark
```

The `blueboat` model is included directly in the world
(`vrx_gz/worlds/nbpark.sdf`, near the `roboboat01` include), so launching
`nbpark` spawns it automatically.

## Key files

| Path | Purpose |
|------|---------|
| `vrx_gz/models/blueboat/model.sdf` | The model. **Currently in a TEMPORARY hull-only config** — propellers/thrusters removed for mesh testing; buoyancy (`Surface` x2) + `SimpleHydrodynamics` kept so it still floats. |
| `vrx_gz/models/blueboat/model.config` | Model metadata. |
| `vrx_gz/models/blueboat/meshes/blueboat.glb` | Current visual hull (Y-up glTF; ~10.7k tris). |
| `vrx_gz/models/blueboat/meshes/blueboat_collision.glb` | Low-poly collision hull (~64 tris — keep it cheap). |
| `vrx_gz/models/blueboat/meshes/blueboat_7july2026.glb` | Prior visual hull, kept for low/high geometry comparison. |
| `vrx_gz/models/blueboat/meshes/blueboat_prop*.glb` | Propeller visual/collision meshes (currently unused while props are stripped). |
| `vrx_gz/scripts/glb_stats.py` | Reports vertex/triangle counts, textures, extents of `.glb` files. `python3 vrx_gz/scripts/glb_stats.py <files|dir|glob> [--csv]` |
| `vrx_gz/worlds/nbpark.sdf` | World; contains the `blueboat` `<include>`. |
| `vrx_gz/worlds/ocean.sdf` | WIP world (untracked experiment, not referenced by launch). |

## Important technical notes

- **Meshes are Y-up (glTF), Gazebo is Z-up.** Every mesh in `model.sdf` carries a
  corrective `<pose>0 0 0 1.5708 0 0</pose>` (roll +90°). After correction the
  body frame is +X forward, +Y port, +Z up. If a new mesh loads on its side,
  revisit this roll; if bow points aft, add yaw π on the world `<include>`.
- **Collision meshes must stay low-poly.** Triangle count on the *collision* mesh
  is paid every physics step in DART; the *visual* triangle count only costs when
  the GUI or a sensor renders it. Verified low so far (64 / 20 tris) — keep it.
- **Restore propulsion**: the powered version (2 prop links + 2 revolute joints +
  2 `gz-sim-thruster-system` plugins) is in git history at commit `2e446ac9`.
  `git show 2e446ac9:vrx_gz/models/blueboat/model.sdf` to retrieve it, or
  `git checkout 2e446ac9 -- vrx_gz/models/blueboat/model.sdf`.

## Open next steps

1. **Verify hull-only mesh** in sim: orientation upright, sane waterline, no mesh
   load errors. Tune the mesh `<pose>` and `Surface` points/mass if needed.
2. **Restore propellers** once hull is validated (see above), then tune thruster
   placement and `SimpleHydrodynamics` drag.
3. **Complexity-sensitivity study** (the point of this branch): sweep low- vs
   high-poly meshes and measure load. Isolate subsystems — physics (collision
   tris + link/joint count) vs rendering (visual tris, only under GUI/sensors).
   Metrics: uncapped RTF (`<real_time_update_rate>0</real_time_update_rate>`,
   headless, fixed `--iterations`), the gz-sim profiler (Physics/Sensors/Render
   split), CPU/GPU. `glb_stats.py --csv` gives the independent variable.
   *Not yet built:* a headless benchmark harness (launch nbpark N iters, log
   mean/σ RTF + profiler split to CSV, parametrized by mesh variant).
4. **Accessory-assembly system** (discussed, not started): parts library under
   `vrx_gz/models/blueboat_parts/` with `*.visual.glb`/`*.collision.glb` pairs +
   `part.yaml` metadata, and a generator (analogous to WAM-V's
   `vrx_urdf/vrx_gazebo/scripts/generate_wamv.py` + `configure_wamv`) that
   assembles chosen accessories into one model. One runtime model; structural
   mounts merge into `base_link`, sensors get their own link+joint.

## Aside (unrelated pre-existing issue)

The `DetachableJoint.cc: Child Link dummy_upper could not be found` warning when
launching `nbpark` is a **spawn-timing** issue (the WAM-V tries to attach to the
`platform` model's `dummy_upper` link before `platform` finishes loading). It is
pre-existing and unrelated to the BlueBoat work; `nbpark.sdf` *does* include the
`platform` model. Not yet investigated.
