# Virtual Maritime RobotX Challenge (VMRC)

This is the entry point for teams participating in the current [RobotX Maritime Challenge](http://robotx.org/) or the upcoming VMRC in 2019.

Currently this project includes a basic set of Gazebo simulation elements and examples to support development of RobotX systems.  
This is an active development project so we are adding and improving things all the time.  The project contains a simulation foundation, including an environment similar 
to the RobotX venue and description of the WAM-V platform.  It is intended as a first step for teams that would then extend the tools for their specific development needs.

![VMRC](images/vmrc.jpg)

## Getting Started

The [VMRC Wiki](https://bitbucket.org/osrf/vmrc/wiki) provides documentation on how to setup the simulation.  
The instructions assume a basic familiarity with the ROS environment and Gazebo.  If these tools are new to you, we recommend starting with the excellent [ROS Tutorials](http://wiki.ros.org/ROS/Tutorials)


## Resources

1. [VMRC Wiki](https://bitbucket.org/osrf/vmrc/wiki): Documentation, tutorials, etc.

## Contributing

The simulation tools are being actively developed and extended to support the RobotX teams.  We are starting simple with the important fundamental aspects of the robot and environment, 
but will rely on the community to develop additional functionality around their particular use cases.  Below are some thoughts for ways that teams can contribute, but if you
are working on anything that others may find useful, please share!

 * Acoustics: Including some simple models of the pinger-based tasks will require a rudimentary acoustics model.  There is some evidence of past acoustics sources/sinks in Gazebo with the [OpenAL API](http://osrf-distributions.s3.amazonaws.com/gazebo/api/7.1.0/classgazebo_1_1util_1_1OpenAL.html), but does not appear to be frequently used.
 * Course Models:  Currently we have supplied models of the WAM-V platform and some of the simple buoys.  Developing representations of the other course components, such as the dock, totems, etc. would expand number of tasks amenable to simulation. 
 * Sensor Examples: Use of simple sensors such as GPS, IMU, camera, etc. are illustrated tutorial section of the wiki.  There are obviously many more sensors being used by the teasm (Velodyne lidar).  Providing examples or recipes for using these sensors in simulation would be a nice contribution.
 * Wave Field: The current wave field is very simple, based on three element Gerstner waves.  Furthermore, the visual representation of the wave field is independent of the displacement seen by the USV.  Adding the ability to simulate higher fidelity wave spectra would be an improvement.
 * Rviz: We have not tested these tools working with Rviz, which often exposes problems with the tf tree.  
 
If you have any questions about these topics, or would like to work on other aspects, please contribute.  You can contact us directly (see below), submit an issue or, even better yet, submit a pull request.

## Contacts

 * Carlos Aguero <caguero@osrfoundation.org>
 * Brian Bingham <bbingham@nps.edu>
