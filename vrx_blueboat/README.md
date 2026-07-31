# vrx_blueboat

Blue Robotics BlueBoat USV models and an accessory mesh library. Assets only —
no compiled code. Spawned by `vrx_gazebo/worlds/blueboat_sandbox.sdf`.

This is a sandbox for testing how meshes, PBR materials and part assemblies
behave in Gazebo, not a competition vehicle.

## Models

| Model | What it is |
|---|---|
| `blueboat_29July2026` | Current hull (~10.7k tris), powered. PBR material embedded in the GLB. Owns the current hull meshes. |
| `blueboat_trial_29July2026` | **Temporary.** Same meshes as above, but the PBR maps are extracted to PNGs and declared in SDF. The A/B control for a Gazebo material-import defect — see the comment at the top of its `model.sdf`. |
| `blueboat_bare_29July2026` | Hull-only (one link, no props). Cheapest float test and the baseline for the geometry-complexity study. |
| `blueboat_7July2026` | Older hull (~13.5k tris), base-colour texture only. The other data point for the complexity study. |
| `blueboat_assembly` | Current hull with all 10 accessories mounted. Meshless — references the other two directories. |
| `blueboat_parts` | Accessory mesh library (10 parts) plus the propeller meshes. Not a model; no `model.config`. |

## Conventions

**Mesh naming** is `<part>.visual.glb` / `<part>.collision.glb`, lower case with
underscores.

**Meshes are Y-up (glTF), Gazebo is Z-up**, so every mesh carries a corrective
`<pose>0 0 0 1.5708 0 0</pose>` roll. After that, +Z is up and +Y is port.

**Bow is at −X on this hull**, stern at +X — the reverse of the usual
"+X forward" convention. Confirmed visually: the propellers sit at +0.4832.
Since the thruster joints use axis `1 0 0`, positive thrust may drive the boat
astern; verify on the water before trusting a sign.

**Models cross-reference each other's meshes** with
`file://<model_dir>/meshes/...`. That resolves only because all of these
directories install under one `share/models` on `GZ_SIM_RESOURCE_PATH` — see
the comment in `CMakeLists.txt` before moving anything.

## Known asset defects

Found with `tools/glb_stats.py`, not yet fixed at source (reported to the
modeller in `../MODELER_NOTES.md`):

- `ping_mount.collision.glb` — 3.68 m against a 0.24 m visual, ~15× oversized.
  Deliberately left out of `blueboat_assembly`; it would wrap a collision volume
  larger than the boat around the hull.
- `side_scan_sonar.visual.glb` — 6.12 m, and `surveyor.visual.glb` — 3.42 m,
  both on a 1.19 m boat. Mounted at native scale in `blueboat_assembly` with
  commented-out `<scale>` lines beside them, so the problem stays visible.
- `blueboat_prop.*` and `thruster_propeller.visual.glb` carry no material, so
  they render default grey.

## Tools

```bash
python3 tools/glb_stats.py share/models --csv
```

Reports vertex/triangle counts, texture inventory and bounding-box extents.
Note it reports raw accessor extents and does **not** apply glTF node
transforms, so a mesh with a baked node scale will read at its authored size.
