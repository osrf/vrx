# VRX 4 dev container (ROS 2 Lyrical + Gazebo Jetty)

Quick start for building and running VRX 4 in the Docker dev container.
Replace `PATH/vrx_ws` with the path to your own workspace.

## 1. Build the Lyrical/Jetty dev image

```bash
cd PATH/vrx_ws/src/vrx
./docker/build.bash docker lyrical            # -> vrx_dev:lyrical
```

## 2. Start an interactive dev container (GPU + X11 + home mount)

```bash
./docker/run_compose.bash                     # drops into bash in the container
```

`./docker/run_compose.bash <cmd...>` runs a single command instead of dropping
into a shell — handy for scripted builds and headless test runs. Each
invocation is a fresh `--rm` container, so write anything you want to keep
under the mounted home rather than `/tmp`.

## 3. Inside the container: build

```bash
source /opt/ros/lyrical/setup.bash
cd PATH/vrx_ws
colcon build --merge-install
```

The FFT wave provider needs **EncinoWaves**, which is not in the image. Without
it `gz_waves_provider_fft` fails to *configure*, which aborts the whole build —
it is not skipped. Either point CMake at your EncinoWaves build:

```bash
export CMAKE_PREFIX_PATH=/path/to/encinowaves/build:$CMAKE_PREFIX_PATH
```

or build around it, selecting only what you need:

```bash
colcon build --merge-install --packages-up-to \
  vrx_bringup vrx_gazebo gz_waves_provider_gerstner gz_waves_rendering
```

Add `rm -rf build install log` first if you have switched branches — a stale
CMake cache pointing at a package path that no longer exists will fail the
build with a confusing "source directory does not exist".

## 4. Run the open-water demo

```bash
source install/setup.bash
ros2 launch vrx_bringup simulation.launch.xml
```

## 5. Run the BlueBoat sandbox

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
