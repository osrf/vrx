### Optional follow-up: lift the generic smoke test into a reusable action

Not blocking this PR — flagging it for later. The smoke-test trio (`check_render.py`,
`observation_camera.sdf`, and most of `smoke_test.sh`) is project-neutral; only the
launch invocation and the `ci.yml` package names are VRX-specific. If a second
Gazebo + `ros_gz` repo ever needs the same "does it come up and render headless?"
gate, here's how I'd factor it out. Rule of three applies — no need to do this until
that second consumer exists; merging in-repo as-is is the right call now.

#### What moves vs. what stays

| Stays in VRX (`ci.yml`) | Moves to a shared repo |
|---|---|
| `container: ros:<distro>-ros-base` | GL/Mesa + `ros-gz` install |
| `actions/checkout` into `src/vrx` | `rosdep install` + `colcon build` + install assert |
| Fuel cache (path is env-specific) | `check_render.py`, `observation_camera.sdf`, `smoke_test.sh` |

#### Prerequisite (cheap, can land in this PR)

`smoke_test.sh` already parameterizes the world/topic via env vars; the only
hardcoded VRX bit is the launch line. Make it injectable too:

```bash
LAUNCH=(ros2 launch "${VRX_SMOKE_LAUNCH_PKG:-vrx_bringup}" \
        "${VRX_SMOKE_LAUNCH_FILE:-simulation.launch.xml}" \
        gazebo_gui:=false world:="${WORLD}")
```

After that, the three files are a self-contained, neutral unit you can `git mv`
verbatim. (On the move, drop the `VRX_` env prefix — `SMOKE_*` — since it's no
longer VRX-specific.)

#### Recommended form: a **composite action** (not a ROS package)

A ROS package would need `package.xml` + install rules + a rosdep/rosdistro entry
to be installable — heavy machinery for a ~290-line CI helper. A composite action
ships the scripts naturally (via `$GITHUB_ACTION_PATH`) and hides the gnarly
install/build steps. Proposed repo `‹owner›/ros-gz-render-smoke`:

```
ros-gz-render-smoke/
├── action.yml
├── smoke_test.sh
├── check_render.py
├── observation_camera.sdf
└── README.md
```

`action.yml`:

```yaml
name: ROS + Gazebo headless render smoke test
description: >
  Build a ROS 2 / Gazebo workspace (source under src/) and verify the sim comes
  up and renders headlessly under Xvfb + Mesa llvmpipe: /clock advances and a
  spawned camera produces frames.

inputs:
  ros-distro:     { description: "ROS 2 distro (selects apt prefix + setup path)", default: lyrical }
  launch-package: { description: "Package with the production launch file",        required: true }
  launch-file:    { description: "Launch file to run headless",                    default: simulation.launch.xml }
  world:          { description: "World file (resolved via GZ_SIM_RESOURCE_PATH)", default: open_water.sdf }
  world-name:     { description: "<world name> attr, for /world/<name>/create",    default: default }
  image-topic:    { description: "ROS image topic the camera is bridged to",       default: /observation_camera/image }
  min-frames:     { description: "Min camera frames over the measurement window",  default: "5" }

runs:
  using: composite
  steps:
    - name: Headless GL + ros2-testing repo
      shell: bash
      run: |
        apt-get update
        apt-get install -y --no-install-recommends \
          ca-certificates git xvfb libgl1-mesa-dri libglx-mesa0 mesa-utils \
          ros2-testing-apt-source

    - name: Install ros-gz (${{ inputs.ros-distro }}) + workspace deps
      shell: bash
      run: |
        apt-get update
        apt-get install -y --no-install-recommends \
          ros-${{ inputs.ros-distro }}-ros-gz-sim ros-${{ inputs.ros-distro }}-ros-gz-bridge
        rosdep update
        rosdep install --from-paths src --ignore-src -y \
          --rosdistro ${{ inputs.ros-distro }} \
          --skip-keys "ros_gz_sim ros_gz_bridge"

    - name: Build
      shell: bash
      run: |
        . /opt/ros/${{ inputs.ros-distro }}/setup.sh
        colcon build --merge-install
        test -d "install/share/${{ inputs.launch-package }}" \
          || { echo "colcon did not build ${{ inputs.launch-package }}" >&2; exit 1; }

    - name: Headless smoke test + camera FPS (xvfb)
      shell: bash
      env:
        SMOKE_LAUNCH_PKG:  ${{ inputs.launch-package }}
        SMOKE_LAUNCH_FILE: ${{ inputs.launch-file }}
        SMOKE_WORLD:       ${{ inputs.world }}
        SMOKE_WORLD_NAME:  ${{ inputs.world-name }}
        SMOKE_IMAGE_TOPIC: ${{ inputs.image-topic }}
        SMOKE_MIN_FRAMES:  ${{ inputs.min-frames }}
      run: |
        . /opt/ros/${{ inputs.ros-distro }}/setup.sh
        . install/setup.sh
        bash "${{ github.action_path }}/smoke_test.sh"
```

VRX's `ci.yml` then collapses to env shell + checkout + cache + one `uses:`:

```yaml
name: vrx4 CI
on:
  push:        { branches: [vrx4] }
  pull_request:{ branches: [vrx4] }
  workflow_dispatch:

jobs:
  smoke:
    name: Smoke xvfb (Lyrical + Jetty)
    runs-on: ubuntu-latest
    timeout-minutes: 45
    container: { image: ros:lyrical-ros-base }
    env: { DEBIAN_FRONTEND: noninteractive }
    steps:
      - run: apt-get update && apt-get install -y --no-install-recommends ca-certificates git
      - uses: actions/checkout@v4
        with: { path: src/vrx }
      - name: Cache Gazebo Fuel assets
        uses: actions/cache@v4
        with:
          path: /github/home/.gz/fuel
          key: gz-fuel-${{ hashFiles('src/vrx/vrx_gazebo/worlds/*.sdf') }}
          restore-keys: gz-fuel-
      - uses: ‹owner›/ros-gz-render-smoke@v1
        with:
          ros-distro: lyrical
          launch-package: vrx_bringup
          world: open_water.sdf
          world-name: default
```

The gnarly Mesa / `ros-gz` / `rosdep` / build / smoke logic lives in one versioned
place; each consumer keeps only what's genuinely its own (the container image and
the Fuel cache path).

#### Alternative: a reusable workflow (`on: workflow_call`)

If you'd rather the caller write *only* `jobs.smoke.uses: ‹owner›/repo/.github/workflows/smoke.yml@v1`
+ `with:`, a reusable workflow can own the `container:` too. Two caveats: (1) it must
`actions/checkout` both the caller's repo (the `github` context already points at the
caller) **and** its own repo to get the bundled scripts (pinned `ref`); (2) a job that
`uses:` a reusable workflow can't also run steps, so the Fuel cache must move *inside*
the workflow as inputs. Thinnest caller, slightly less per-repo control. I'd start with
the composite action and only promote to a reusable workflow if the container/runner
setup also turns out to be identical across consumers.
