# VRX

Minimal boilerplate for the Virtual RobotX (VRX) simulation: a Gazebo world
with open water and the ROS 2 launch glue to bring it up. No vehicles are
included yet — this is the starting point for a fresh rebuild.

## Requirements

- ROS 2 Rolling
- Gazebo Jetty
- `ros_gz` (matching the Rolling / Jetty pairing)

## Packages

- `vrx_gazebo` — Gazebo worlds. Installs `open_water.sdf` and exports
  `GZ_SIM_RESOURCE_PATH` via ament hooks so the world can be referenced by
  filename.
- `vrx_bringup` — ROS 2 launch file and `ros_gz_bridge` configuration.

## Build

```bash
cd ~/vrx_ws
colcon build --symlink-install
source install/setup.bash
```

## Run

```bash
ros2 launch vrx_bringup simulation.launch.xml
```

Launch arguments:

- `world` (default: `open_water.sdf`) — world file name, resolved through
  `GZ_SIM_RESOURCE_PATH`.
- `gazebo_gui` (default: `true`) — start the Gazebo GUI.
- `use_composition` (default: `true`) — run the Gazebo server and bridge as
  composable nodes in a shared container.

The open water world pulls the Portuguese Ledge bathymetry tile from
[Gazebo Fuel](https://app.gazebosim.org/) on first run.

## License

Apache-2.0 (see `LICENSE`).
