^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package vrx_gazebo
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Merge from default.
* tweak the example
* addressing missing documentation and simplifying by removing start_index parameter
* Removing leftovers
* Tweaks
* Style changes.
* Merge from default.
* Merged in symbols_dock_part3 (pull request #66)
  Scan and dock scoring plugin - Part3
  Approved-by: Brian Bingham <briansbingham@gmail.com>
* syncing with default
* Change to use real-time pose for error calculation
* Simplifying by removing some of the timing bits that appear to be specific to the ARIAC Population plugin
* Renaming internal
* Rename part 2 of 2
* Renaming part 1
* Adding scoring and running a quick test
* Functional plugin prototype
* Merge from default.
* Two variants of the scan and dock.
* Remove unused code.
* updating topic names so they match tasks
* tweak
* now publishing waypoints as a latched GeoPath message
* fix function name PublishWaypoints
* only start scoring when in running state
* fixing task names
* Re-basing poplulation plugin to scoring_plugin and adding ROS functionality.  Incomplete, but going home to work from there
* tweak a comment
* tweak
* Granting extra points for docking.
* Tweaks
* PR feedback
* Wrong merges.
* Merge from default.
* Merged in wayfinding-task (pull request #69)
  Wayfinding task
  Approved-by: Brian Bingham <briansbingham@gmail.com>
* remove pointless latch in waypoints topic
* fix timer
* publish at 1 Hz, latch waypoints topic, tweaks
* Merge from default.
* Tweaks.
* Merge from default.
* Prototype of population plugin - only for a single object at a time.  Moves it back to original position when done
* Updates to PopulationPlugin
* Prototype - using PopulationPlugin straight from ARIAC source
* Remove warnings.
* More vrx updates.
* Merge from symbols_dock_part2
* More vrx tweaks.
* Merge from default.
* More updates.
* Porting to Gazebo 9
* Custom tweaks
* Updating the station keeping task.
* More leftovers.
* Rename vmrc to vrx.
* Contributors: Brian Bingham <briansbingham@gmail.com>, Carlos Aguero, Carlos Aguero <caguero@osrfoundation.org>, Carlos Agüero <cen.aguero@gmail.com>, Michael McCarrin <mrmccarr@nps.edu>, m1chaelm

0.3.2 (2018-10-08)
------------------
* Include jrivero as maintainer of the ROS packages
* Contributors: Jose Luis Rivero <jrivero@osrfoundation.org>

0.3.1 (2018-10-05)
------------------

0.3.0 (2018-09-28)
------------------
* Tweak
* Rename robotx_gazebo to vrx_gazebo and remove metapackage.
* Contributors: Carlos Agüero <caguero@osrfoundation.org>
