# gazebo_rangebearing_plugin

This Gazebo plugin implements a simple model of a sensor that provides range, bearing and elevation from a robot sensor to a fixed location.  The plugin was developed to simulate an acoustic ranging system to support the [VMRC](https://bitbucket.org/osrf/vmrc) project.

## Theory of Operation

The model plugin is associated with a specific link in the robot URDF/Xacro definition file.  The fixed location of the range/bearing target is defined in the plugin SDF using the `beaconPoint` tag.  The geometry of the scenario is used to determine the range [m], bearing [rad] and elevation [rad] from the robot (the link specified by the `bodyName` tag) to te `beaconPoint`.

The range, bearing and elevtion is reported as a three element [std_msgs::Float32MultiArray](std_msgs::Float32MultiArray) message, where the range = element 0, bearing = element 1, elevation = element 2.

Independent Gaussian noise is added to each measurment.  The noise is generated using Gazebo's [sensor noise](http://gazebosim.org/tutorials?tut=sensor_noise) functionality.

## Working Example

The repository contains a simple, standalone working example...

```
roslaunch rangebearing_gazebo_plugin box.launch
```

which demonstrates the configuration of the plugin.  The [boxbot.urdf](https://bitbucket.org/osrf/vmrc/src/acoustic-pinger/rangebearing_gazebo_plugin/urdf/boxbot.urdf) defines an example robot (just a small box) and configures the plugin to be associated with the robot model.

In a separate terminal run

```
rostopic echo /range_bearing 
```

to view the published range, bearing, elevation simulated measurements.
