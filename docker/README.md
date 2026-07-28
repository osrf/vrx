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

## 3. Inside the container: build the wave stack

```bash
source /opt/ros/lyrical/setup.bash
cd PATH/vrx_ws
rm -rf build install log
colcon build --merge-install
```

## 4. Run the sim (EncinoWaves ocean, GPU-rendered)

```bash
source install/setup.bash
ros2 launch vrx_bringup simulation.launch.xml
```
