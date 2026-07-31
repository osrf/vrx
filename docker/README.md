# VRX 4 dev container (ROS 2 Lyrical + Gazebo Jetty)

Quick start for building and running VRX 4 in the Docker dev container. Replace `PATH/vrx_ws` with the path to your own workspace.


## 1. Build the Lyrical/Jetty dev image — ON THE HOST

```bash
cd PATH/vrx_ws/src/vrx
./docker/build.bash docker lyrical            # -> vrx_dev:lyrical
```

## 2. Start an interactive dev container — ON THE HOST

```bash
./docker/run_compose.bash                     # drops into bash in the container
```

## 3. Build — INSIDE THE CONTAINER

```bash
source /opt/ros/lyrical/setup.bash
cd PATH/vrx_ws
rm -rf build install log
colcon build --merge-install
```

**Build-order caveat.** EncinoWaves has no `package.xml`, so colcon cannot know that `gz_waves_provider_fft` depends on it and may schedule them concurrently. In practice it works, because `gz_waves_provider_fft` waits on `gz_waves`, which takes longer to build than EncinoWaves — but that is timing, not a guarantee. If a clean build ever fails saying EncinoWaves was not found, build it first:

```bash
colcon build --merge-install --packages-select EncinoWaves
colcon build --merge-install
```

The durable fix is a `package.xml` in the EncinoWaves repo declaring it an ament/cmake package, plus a `<depend>` on it from `gz_waves_provider_fft`.

### Or skip FFT entirely

Nothing needs it unless you want the FFT wave engine. Select around it:

```bash
colcon build --merge-install --packages-up-to \
  vrx_bringup vrx_gazebo gz_waves_provider_gerstner gz_waves_rendering
```


## 4. Run the open-water demo — INSIDE THE CONTAINER

```bash
source install/setup.bash
ros2 launch vrx_bringup simulation.launch.xml
```

## 5. Run the BlueBoat sandbox — INSIDE THE CONTAINER

A row of BlueBoat variants floating on open water, for testing meshes, PBR materials and part assembly. See `../SANDBOX.md`.

```bash
colcon build --merge-install
source install/setup.bash
ros2 launch vrx_bringup simulation.launch.xml world:=blueboat_sandbox.sdf
```

The sandbox world uses the FFT wave engine, so it needs EncinoWaves — see `../vrx.repos`. To run without it, switch the world to Gerstner by commenting the FFT `<plugin>` block and uncommenting the Gerstner one, then build around FFT as in section 3.

### Paused and headless

`simulation.launch.xml` takes `paused` and `gazebo_gui`:

```bash
ros2 launch vrx_bringup simulation.launch.xml world:=blueboat_sandbox.sdf paused:=true
ros2 launch vrx_bringup simulation.launch.xml world:=blueboat_sandbox.sdf gazebo_gui:=false
```

Or drive the server directly, which is simpler for scripted runs — omitting `-r` starts it paused:

```bash
gz sim -s -r install/share/vrx_gazebo/worlds/blueboat_sandbox.sdf   # headless, running
gz sim    install/share/vrx_gazebo/worlds/blueboat_sandbox.sdf      # paused
```

Note that a paused sim shows models at their **spawn** pose, not floating — the buoyancy and hydrodynamics systems both skip a paused step.

Implementation note: `paused` does not reach `gz_server`, which has no such parameter — its component declares only `world_sdf_file`, `world_sdf_string` and `initial_sim_time`. The launch file instead bypasses `gz_server` on the paused path and runs `gz sim -s` directly, and the bridge creates its own composition container in that case since nothing else does.

### Driving a boat

Thruster topics are per model instance, named after the `<name>` in the world's `<include>`:

```bash
gz topic -t /model/blueboat_29July2026/joint/left_engine_propeller_joint/cmd_thrust \
         -m gz.msgs.Double -p 'data: 20.0'
```

Positive thrust drives ahead. Wave conditions can be changed live (0 = flat, 2 = the sandbox default, 5 = rough):

```bash
gz service -s /world/blueboat_sandbox/wave/set_parameters \
  --reqtype gz.msgs.Param --reptype gz.msgs.Boolean --timeout 3000 \
  --req 'params { key: "sea_state" value { type: INT32 int_value: 0 } }'
```

### Running faster than real time

The `<physics>` block in the world file caps speed. `<real_time_factor>` is the target ratio of sim time to real time, default `1.0`; set it to `0` for no throttle:

```xml
<physics name="default_physics" type="dart">
  <max_step_size>0.004</max_step_size>
  <real_time_factor>0</real_time_factor>
</physics>
```

Unthrottling physics alone is not enough for a fast run: with the GUI up, rendering paces the loop, and `gz-sim-sensors-system` renders too. Use `gz sim -s -r` for genuine maximum speed.
