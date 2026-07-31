# VRX 4 dev container (ROS 2 Lyrical + Gazebo Jetty)

Quick start for building and running VRX 4 in the Docker dev container.
Replace `PATH/vrx_ws` with the path to your own workspace.


## 1. Build the Lyrical/Jetty dev image — ON THE HOST

```bash
cd PATH/vrx_ws/src/vrx
./docker/build.bash docker lyrical            # -> vrx_dev:lyrical
```

## 2. Start an interactive dev container — ON THE HOST

```bash
./docker/run_compose.bash                     # drops into bash in the container
```

GPU, X11, home mount and joystick are all wired up by `compose.yaml`.

`./docker/run_compose.bash <cmd...>` runs a single command instead of dropping
into a shell — handy for scripted builds and headless test runs. Each
invocation is a fresh `--rm` container, so write anything you want to keep
under the mounted home rather than `/tmp`.

## 3. Build — INSIDE THE CONTAINER

```bash
source /opt/ros/lyrical/setup.bash
cd PATH/vrx_ws
colcon build --merge-install
```

The FFT wave provider needs **EncinoWaves**, which is not in the image. Without
it `gz_waves_provider_fft` fails to *configure*, which aborts the whole build —
it is not skipped, so the error is not obviously about one optional package.

### Add EncinoWaves to the workspace (recommended)

Clone it into `src/` next to this repo, so the workspace is self-contained and
does not depend on anything installed system-wide or in another workspace:

```bash
cd PATH/vrx_ws
vcs import src < src/vrx/vrx.repos       # or: git clone <url> src/encinowaves
colcon build --merge-install
```

colcon builds it as a plain CMake package and everything downstream finds it —
no `CMAKE_PREFIX_PATH` to set, and `rm -rf install` cannot leave a stale copy
behind somewhere else.

**Build-order caveat.** EncinoWaves has no `package.xml`, so colcon cannot know
that `gz_waves_provider_fft` depends on it and may schedule them concurrently.
In practice it works, because `gz_waves_provider_fft` waits on `gz_waves`, which
takes longer to build than EncinoWaves — but that is timing, not a guarantee. If
a clean build ever fails saying EncinoWaves was not found, build it first:

```bash
colcon build --merge-install --packages-select EncinoWaves
colcon build --merge-install
```

The durable fix is a `package.xml` in the EncinoWaves repo declaring it an
ament/cmake package, plus a `<depend>` on it from `gz_waves_provider_fft`.

### Or skip FFT entirely

Nothing needs it unless you want the FFT wave engine — the BlueBoat sandbox uses
Gerstner. Select around it:

```bash
colcon build --merge-install --packages-up-to \
  vrx_bringup vrx_gazebo gz_waves_provider_gerstner gz_waves_rendering
```

### Stale CMake cache

Add `rm -rf build install log` first if you have switched branches — a stale
CMake cache pointing at a package path that no longer exists fails the build
with a confusing "source directory does not exist".

## 4. Run the open-water demo — INSIDE THE CONTAINER

```bash
source install/setup.bash
ros2 launch vrx_bringup simulation.launch.xml
```

## 5. Run the BlueBoat sandbox — INSIDE THE CONTAINER

A row of BlueBoat variants floating on open water, for testing meshes, PBR
materials and part assembly. See `../SANDBOX.md`.

```bash
colcon build --merge-install --packages-up-to \
  vrx_bringup vrx_gazebo vrx_blueboat vrx_usv_dynamics \
  gz_waves_provider_gerstner gz_waves_rendering
source install/setup.bash
ros2 launch vrx_bringup simulation.launch.xml world:=blueboat_sandbox.sdf
```

The sandbox uses the Gerstner wave engine, so it does not need EncinoWaves. One
error on startup is expected and harmless — `Failed to load system plugin
[gz-sim-waves-fft-gui]` — because the water surface model requests both
engines' GUI registrars and the FFT one is not built.

### Headless

`simulation.launch.xml` has no `paused` argument. To start paused, or to run
without a GUI, drive the server directly — omitting `-r` starts it paused:

```bash
gz sim -s -r install/share/vrx_gazebo/worlds/blueboat_sandbox.sdf   # headless, running
gz sim    install/share/vrx_gazebo/worlds/blueboat_sandbox.sdf      # paused
```

Note that a paused sim shows models at their **spawn** pose, not floating —
the buoyancy and hydrodynamics systems both skip a paused step.

### Driving a boat

Thruster topics are per model instance:

```bash
gz topic -t /model/bb_glb_pbr/joint/left_engine_propeller_joint/cmd_thrust \
         -m gz.msgs.Double -p 'data: 20.0'
```

Positive thrust drives ahead. Wave conditions can be changed live (0 = flat,
2 = the sandbox default, 5 = rough):

```bash
gz service -s /world/blueboat_sandbox/wave/set_parameters \
  --reqtype gz.msgs.Param --reptype gz.msgs.Boolean --timeout 3000 \
  --req 'params { key: "sea_state" value { type: INT32 int_value: 0 } }'
```
