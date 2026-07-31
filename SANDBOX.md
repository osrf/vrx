# BlueBoat sandbox branch (`bsb/blueboat-vrx4`)

A testing sandbox for Blue Robotics BlueBoat assets on the VRX 4 base. Used for
checking how meshes, PBR materials and part assemblies behave in Gazebo, and for
a geometry-complexity study.

**This branch is not intended to be merged into `vrx4` or the default branch.**
It is a working sandbox, shared so others can run it, not a feature branch.

## What it adds

Everything here is **additive** — no file that exists on `vrx4` is modified.
Verify with `git diff --name-status origin/vrx4 HEAD`; every entry should be `A`.

| Path | What |
|---|---|
| `vrx_blueboat/` | BlueBoat models + accessory mesh library (assets only). |
| `vrx_usv_dynamics/` | `Surface` buoyancy + `SimpleHydrodynamics`, ported from VRX 3. |
| `vrx_gazebo/worlds/blueboat_sandbox.sdf` | The sandbox world. New file in an existing directory. |
| `MODELER_NOTES.md` | Asset feedback for the 3D modeller. |

Deleting the sandbox is `rm -rf vrx_blueboat vrx_usv_dynamics
vrx_gazebo/worlds/blueboat_sandbox.sdf MODELER_NOTES.md`, or just deleting the
branch.

## Build and run

```bash
./docker/run_compose.bash                    # ROS 2 Lyrical + Gazebo Jetty
# inside the container:
cd ~/vrx_ws
colcon build --merge-install --packages-up-to \
  vrx_bringup vrx_gazebo vrx_blueboat vrx_usv_dynamics \
  gz_waves_provider_gerstner gz_waves_rendering
. install/setup.bash
ros2 launch vrx_bringup simulation.launch.xml world:=blueboat_sandbox.sdf
```

`--packages-up-to` deliberately routes around `gz_waves_provider_fft`, which
fails to *configure* (not merely skip) when EncinoWaves is absent, as it is in
the dev image. The sandbox uses the Gerstner engine anyway — see the wave source
comment in `blueboat_sandbox.sdf` for why that matters when things float.

Expect one benign error on startup: `Failed to load system plugin
[gz-sim-waves-fft-gui]`. Upstream `water_surface/model.sdf` requests both
engines' GUI registrars and we do not build the FFT one.

## Keeping current with `vrx4`

**Merge, don't rebase.** This branch is shared, so rebasing would rewrite
history under anyone who has it checked out.

```bash
git fetch origin
git merge origin/vrx4
```

Because the sandbox touches no file that `vrx4` owns, this should stay
conflict-free. Keep it that way — if something on `vrx4` needs changing, prefer
raising it there over editing it here.

## Notes for anyone picking this up

- The hull mesh has **bow at −X**, the reverse of the usual convention. The
  propeller joints therefore use axis `-1 0 0` so positive `cmd_thrust` drives
  ahead. Measured, not assumed — see `vrx_blueboat/README.md`.
- `vrx_usv_dynamics/Surface` is **interim**, filling in until
  `gz_waves_buoyancy`'s `WaveBuoyancy` lands. Its README has the retire path.
- Three accessory meshes have bad scale at source; see
  `vrx_blueboat/README.md` and `MODELER_NOTES.md`.
