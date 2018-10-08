^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package usv_gazebo_plugins
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.3.2 (2018-10-08)
------------------
* Include jrivero as maintainer of the ROS packages
* Include headers in the installation of usv_gazebo_plugins
* Contributors: Jose Luis Rivero <jrivero@osrfoundation.org>

0.3.1 (2018-10-05)
------------------
* Decleare eigen as dependency for usv_gazebo_plugins
* modifying grid spacing
* Contributors: Brian Bingham <briansbingham@gmail.com>, Jose Luis Rivero <jrivero@osrfoundation.org>

0.3.0 (2018-09-28)
------------------
* vmrc metapackage and spring cleaning.
* adding publication of forces/moments
* trying to get wamv to be static using a fixed joint
* Adding publication from dynamics plugin for wave height at USV CG for Josh's thesis work
* Tweak
* Changelog and minor tweaks.
* Remove extra dependency.
* Merged in generalize-thruster-desc (pull request #34)
  Generalize thruster desc
  Approved-by: Brian Bingham <briansbingham@gmail.com>
  Approved-by: Carlos Agüero <cen.aguero@gmail.com>
* merging changes from PR branch into development branch
* resolving merge conflict
* Adding bits to repond to PR comments
* adding examples for T and X thruster configurations - accessible as args to sandisland.launch. Prototype - too much redundancy in the various urdf.xacro file hierarchy, but functional.
* Tweaks.
* Tabs -> spaces
* Initial style pass
* props now spinning, removed old method of thrust implementation, removed custome UsvDrive message
* working prototype - next remove old method
* prior to splitting thruster into its own header
* increment - builds, but need to go home
* catching up with default
* increment, pushing to work from home
* first steps towards new structure
* Drop log level to DEBUG for imformation unimportant to user
* Minor style changes in the gazebo_ros_color plugin.
* Tweak
* Move log message to DEBUG.
* adding a bit more doxygen, including link to Theory of Operation document
* Tweaks.
* adding doxygen comments
* Doxygen and cleaning up
* Rename buoyLinks to buoyancyLinks and remove debug output.
* More style.
* More tweaks.
* Initial style changes.
* Merge from default.
* Apply Gazebo style.
* Move some ROS_INFO messages to ROS_DEBUG and remove ros::init().
* More tweaks.
* Tweaks
* Tweaks
* Initial work
* Publish joint_states from thrust plugin
* Tweak
* Refactor wind plugin.
* Split the wamv xacro file.
* Generate messages before building the Thrust plugin.
* More modular model with spinning propellers.
* Merge from default
* Add message_generation.
* Backed out changeset 8023d94fc0e1
* Add light buoy challenge
* Remove unsused buoyancy plugin (already in gazebo)
* Boostrap usv_gazebo_plugins
* Move gazebo plugins to usv_gazebo_plugins
* Contributors: Brian Bingham <briansbingham@gmail.com>, Carlos Aguero, Carlos Agüero <caguero@osrfoundation.org>, Kevin Allen <kallen@osrfoundation.org>
