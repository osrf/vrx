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

All four blueboat models are included directly in `vrx_gz/worlds/nbpark.sdf`
(near the `roboboat01` include), spawned 2 m apart along Y so they can be
compared side by side:

| World instance | Pose | What it is |
|---|---|---|
| `blueboat_7July2026` | `-185 1090 1` | Older hull (~13.5k tris), powered. |
| `blueboat_bare_29July2026` | `-185 1092 1` | Current hull, hull-only (1 link, no props). |
| `blueboat_29July2026` | `-185 1094 1` | Current hull (~10.7k tris), powered. |
| `blueboat_assembly` | `-185 1096 1` | Current hull + every accessory mounted. |

## Key files

| Path | Purpose |
|------|---------|
| `vrx_gz/models/blueboat_29July2026/` | Current powered model: `base_link` + 2 prop links/revolute joints, buoyancy (`Surface` x2), `SimpleHydrodynamics`, 2 `gz-sim-thruster-system` plugins. Owns the current hull meshes. |
| `vrx_gz/models/blueboat_7July2026/` | Same structure, older/heavier hull mesh. Kept for the geometry-complexity comparison. |
| `vrx_gz/models/blueboat_bare_29July2026/` | Hull-only variant of the current model (no props/thrusters). |
| `vrx_gz/models/blueboat_assembly/` | Current hull + all 10 accessories. Meshless on disk — references hull meshes from `blueboat_29July2026/` and parts from `blueboat_parts/`. |
| `vrx_gz/models/blueboat_parts/meshes/` | Accessory mesh library (10 parts), plus the propeller meshes the powered models reference. |
| `vrx_gz/scripts/glb_stats.py` | Reports vertex/triangle counts, textures, extents of `.glb` files. `python3 vrx_gz/scripts/glb_stats.py <files|dir|glob> [--csv]` |
| `vrx_gz/worlds/nbpark.sdf` | World; contains the four blueboat `<include>`s. |
| `vrx_gz/worlds/ocean.sdf` | WIP world (untracked experiment, not referenced by launch). |

Meshes are cross-referenced between models by `file://<model_dir>/meshes/...`,
which works because all models sit on the same resource path. Fixing a source
mesh therefore fixes every model using it — but renaming a model directory
breaks every other model that points into it.

## Important technical notes

- **Meshes are Y-up (glTF), Gazebo is Z-up.** Every mesh carries a corrective
  `<pose>0 0 0 1.5708 0 0</pose>` (roll +90°). After correction +Z is up and
  +Y is port. If a new mesh loads on its side, revisit this roll.
- **Bow is at −X on this hull**, stern at +X — the opposite of the usual
  "+X forward" convention. Confirmed visually (props sit at +0.4832). Since the
  thruster joints use axis `1 0 0`, positive thrust likely drives the boat
  *astern*; fix with axis `-1 0 0` or yaw π on the world `<include>` once
  confirmed on the water.
- **Propeller pose** (current model): links at `x=0.4832, y=±0.30, z=-0.1308`,
  mesh pose `0 0 0 0 3.1416 0`. The 180° pitch puts the disc in the vertical
  plane with its spin axis along X; 90° left it lying flat.
- **Naming convention:** meshes are `<part>.visual.glb` / `<part>.collision.glb`,
  lower case with underscores. Model directories are date-stamped
  (`blueboat_<D><Month><YYYY>`), and each model's `<model name>` / `model.config`
  `<name>` must match its directory — duplicates are legal (the world
  `<include><name>` wins) but make the GUI entity tree ambiguous.
- **Gazebo does not import GLB-embedded materials reliably — declare PBR in SDF.**
  `blueboat_29July2026`'s hull rendered with bright mis-lit facets under Ogre2
  while looking correct in gltf-viewer. Proven cause: the material embedded in
  the GLB. The identical mesh renders correctly when the maps are extracted to
  PNGs and declared in an SDF `<material><pbr><metal>` block — normal map
  included, and confirmed visually to be contributing. `blueboat_trial_29July2026`
  is that proof; `roboboat02/model.sdf` is the pre-existing example of the same
  pattern. Ruled out: missing `TANGENT` vertex attribute (neither hull has one,
  and the SDF path works regardless).
  Practical rule: any mesh with more than a base-colour texture needs its
  material declared in SDF. Base-colour-only meshes (the 7 July hull, `flag`)
  and material-free ones import fine. Most of `blueboat_parts` is factor-only,
  so unaffected — but new textured parts will hit this.
- **Direction: collision geometry moves to SDF primitives, not meshes.** For most
  accessories (and probably the hull), use `<box>`/`<cylinder>`/`<sphere>` in the
  visual's sibling `<collision>` rather than a `*.collision.glb`. Cheaper in DART
  than any mesh, no dependency on the modeller getting a proxy right, and it
  sidesteps the mesh-scale defects already found (`ping_mount.collision` is ~15x
  its own visual). Consequence: the parts library does not need a collision GLB
  per part, and `MODELER_NOTES.md` no longer asks for them. The existing
  `*.collision.glb` files are kept for now but are on the way out.
- **When a collision mesh *is* used, keep it low-poly.** Triangle count on the
  *collision* mesh is paid every physics step in DART; the *visual* triangle
  count only costs when the GUI or a sensor renders it. Current ones are cheap
  (12-124 tris) — keep it that way.
- **Three part meshes have bad scale** (found via `glb_stats.py`, not yet fixed at
  source): `ping_mount.collision` is 3.68 × 1.66 × 1.35 m against a 0.24 m visual
  (~15× oversized — left out of `blueboat_assembly` because it would wrap a
  collision volume larger than the boat around the hull); `side_scan_sonar.visual`
  is 6.12 m and `surveyor.visual` is 3.42 m on a 1.2 m boat, while both of their
  collisions are ~0.17 m. `blueboat_assembly` mounts the two oversized visuals at
  native scale with commented-out `<scale>` lines beside them.
- **Thrusters are not ROS-bridged.** The `Thruster` plugins carry no
  `<namespace>`/`<topic>`, so they listen on the gz default —
  `/model/<world instance name>/joint/<joint_name>/cmd_thrust`, i.e. per-instance
  (`/model/blueboat_29July2026/...`). Confirm with `gz topic -l`. Driving from ROS
  needs explicit `<namespace>`/`<topic>` plus a bridge entry like
  `vrx_gz/src/vrx_gz/payload_bridges.py:152` (`thrusters/<side>/thrust`).
- **`propeller_diameter` is 0.1 but the prop mesh measures 0.1115 m.** Thrust
  scales with diameter⁴, so reconcile when tuning propulsion.

## Open next steps

1. ~~**Verify hull-only mesh**~~ — done, hull looks good in sim.
2. ~~**Restore and place propellers**~~ — done; position and orientation verified
   visually. Still to do: confirm thrust *direction* on the water (see the bow-at-−X
   note above), then tune `thrust_coefficient` / `propeller_diameter` and
   `SimpleHydrodynamics` drag.
2b. **Verify `blueboat_assembly` in sim** — every part visible, upright, plausibly
   scaled. Placements are first-cut guesses picked for visibility, especially deck
   height (z ≈ 0.15). Fix the three bad-scale meshes at source, then re-place.
3. **Complexity-sensitivity study** (the point of this branch): sweep low- vs
   high-poly meshes and measure load. Isolate subsystems — physics (collision
   tris + link/joint count) vs rendering (visual tris, only under GUI/sensors).
   Metrics: uncapped RTF (`<real_time_update_rate>0</real_time_update_rate>`,
   headless, fixed `--iterations`), the gz-sim profiler (Physics/Sensors/Render
   split), CPU/GPU. `glb_stats.py --csv` gives the independent variable.
   *Not yet built:* a headless benchmark harness (launch nbpark N iters, log
   mean/σ RTF + profiler split to CSV, parametrized by mesh variant).
4. **Accessory-assembly system** — parts library now exists at
   `vrx_gz/models/blueboat_parts/meshes/`, and `blueboat_assembly` hand-mounts
   all 10 parts. Still to do: `part.yaml` metadata per part and a generator
   (analogous to WAM-V's `vrx_urdf/vrx_gazebo/scripts/generate_wamv.py` +
   `configure_wamv`) that assembles chosen accessories into one model. One
   runtime model; structural mounts merge into `base_link`, sensors get their
   own link+joint.
   Revised by the primitive-collision decision above: a part is a *visual* mesh
   plus a primitive collision described in metadata (shape + dimensions +
   offset), not a visual/collision GLB pair. `part.yaml` is the natural place to
   carry that shape, and `glb_stats.py` extents give a first-cut box for free.

## Aside (unrelated pre-existing issue)

The `DetachableJoint.cc: Child Link dummy_upper could not be found` warning when
launching `nbpark` is a **spawn-timing** issue (the WAM-V tries to attach to the
`platform` model's `dummy_upper` link before `platform` finishes loading). It is
pre-existing and unrelated to the BlueBoat work; `nbpark.sdf` *does* include the
`platform` model. Not yet investigated.
