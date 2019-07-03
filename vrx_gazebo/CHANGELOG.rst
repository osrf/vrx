^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package vrx_gazebo
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.1 (2019-07-03)
------------------
* Reinterpret the wind 'gain' parameter.  Set defaults to zero
* Add replaces cluase to vrx_gazebo
* Contributors: Brian Bingham <briansbingham@gmail.com>, Jose Luis Rivero <jrivero@osrfoundation.org>

1.1.0 (2019-07-01)
------------------
* Merged in issue#94-buoyancy (pull request #122)
  Issue#94 buoyancy
  Approved-by: Carlos Agüero <cen.aguero@gmail.com>
* changing buoy buoyancy to sphere, adding feature to generator
* Merge from default.
* Merged in rename_scan_dock (pull request #133)
  renaming "dock" and "scan and dock" files to match new task names
  Approved-by: Brian Bingham <briansbingham@gmail.com>
* renaming files to match new task names
* updating default values in example
* now interpreting time of spawned objects relative to start of running state
* removed overidden OnFinished method
* clean up: removed overriden methods that made no alterations from parent class
* added a gzmsg where missing to overriden methods
* added a gzmsg to default implementations of OnReady, OnRunning, OnFinished
* New score policy.
* Update scores.
* Light buoy with 2 seconds off.
* Now impliments Enviornment variable instad of debug sdf parameters
* Incremental
* ready for detailed lidar spec input
* updated markers + polyform models for wave buoyancy
* functional. no recording
* added wind to navigation task
* Add extra_gazebo_args to all launch files and remove recording arg
* Light buoy should now be synced with scoring and visual plugin through the definition in scan_and_dock_b.launch
* updated vrx model buoyancy plugin
* Add playback.launch to play back recorded log files
* Add recording functionality to sandisland, and add extra_gazebo_args to optionally choose record path
* Incremental
* no longer supported for gz7 or older
* clunky version - but visuals and placards stay with dock for 2018
* working version with dock buoyancy, but need to attach placards
* first cut - dock elements work, but to build a full dock need to add joints between elements
* changing perception transition
* attempt build gz <=7 issue
* attempt fix build issue
* incremental
* Added allowences for post_Y and moved wamv_imu, wamv_gps default locations to be within compliance
* attempt fix gz 7 compatability issue
* functionsal. needs cleaning
* initializing sampleCount to 0 and change to int
* added wind capabilities
* Merged in add-wind-support-for-yaml-world-gen (pull request #115)
  added support for wind in yaml world gen and updates wiki
  Approved-by: Carlos Agüero <cen.aguero@gmail.com>
* Style
* testing side by side scaling
* latest case
* case 2
* case 0
* Testing scalability of new implementation - updated hgignore vmrc->vrx
* code styling
* styling
* styling
* incremental
* build
* merge
* removed unused header
* incremental
* finished rename
* added support for wind in yaml world gen and updates wiki
* added support for default wamv effects on vrx.launch and sandisland.launch
* incremental
* Reshow instructions after some speed change updates (match with twist_teleop_keyboard)
* Remove extra diffdrive yaml file
* Implement new getch function to fix output issues
* Remove set_thrust_angle parameter
* Reverse angles when teleoperation.
* merge
* incremental
* incremental
* incremental
* styling fixes
* made more user friendly
* Now builds. Currently, the MOC in CMake requires the header and source file to be in the same directory.
* fixed ros issues
* merging default
* Add new .yaml file for joy teleop to publish thrust angles
* Add settable max_angle parameter upon usv_keydrive launch startup
* Add ability to change thrust angle speed
* Add key2thrust_angle.py node that allows for h and ; to control thruster angle
* Merge from default.
* Merge from default, conflicts and style.
* Merge default
* fixed builf issues
* Merged in remove-README (pull request #111)
  removed README.txt from yaml_world_genreeration and created wiki page instead
  Approved-by: Carlos Agüero <cen.aguero@gmail.com>
* Add style checker.
* removed README.txt from yaml_world_genreeration and created wiki page instead
* finish rename
* fix build issue
* incremental
* fix build issue
* renamed xacro
* updates xacro
* fix build issues
* incremental
* Merged in Issue#90_YAML_world_genreation (pull request #102)
  Issue#90 YAML world generation
  Approved-by: Carlos Agüero <cen.aguero@gmail.com>
* No markdown
* Use markdown
* Fix typos.
* Added thruster compliance
* added more flexibility to permitted parameters
* fixed math error
* Partially fix compile issues in code with Task msg, still issue with FormatTime and duration
* Attempt to fix build issue by adding Qt5IncludeDirs and spreading out find_packages
* added roslaunch params
* styling
* filled out the SensorCompliance. It is formatted by the sensors_compliance files
* Attempt to implement basic GUI overlay to show VRX Task Info. Stuck on build issues with Qt5
* added white spaceing to make more readable
* edited launch file
* incremental
* merge for api update
* merged with Issue#97-yaml-thruster-configuration for api update
* fixed launch file
* changed sdf sytax for passing models to be effected by wind and addressed styling
* Changing name of ocean model in sandisland test
* adding pdf for pr
* Remove unneeded images and add documentation
* adding to docs and allowing for both PMS and CWR wavefield models
* merged. expanded xacro capabilities
* Add back unused functions in utils.py for future compliance tests
* Move gazebo thruster config tags to new function
* Update python files using flake8, all files pass
* Update Changelog and remove available_sensors param
* Remove unnecessary files
* Add generate_wamv launch and bin files
* Clean create_xacro_file() function and add comments
* Remove old sensor and thruster config files
* Remove unused utils.py functions
* Remove unneeded files and improve clarity with documentation
* Added support for any parameter to be evaluated as lambda vs string. updated README.
* fixed functional evaluation bug
* testing wave fields
* Added support for ** xacro inserts. used as normal parameters, but prfaced with /**. (this is to help with the wind and ave plugins in the future.
* Added wind xacro (utilizes xacro inserts). NOTE: wind plugin only applies force to one link per model
* Working implementation of generate_wamv, which takes both sensor and yaml files
* adding exponential increase in wave field and LaTeX doc^C
* CMakeLists improvement and spacing
* changelog update
* added more to README.txt, added scene_macro and sandisland2 to give more confiuration flexibilty to the worlds. NOTE: time SDF is being written into the world file correctly(I think), but gazebo appears to not change anything under the scene tab in the gui.
* Make thruster config with yaml work without affecting use of sensor yaml config, still need to clean up
* Move engine.xacro to thrusters directory to allow for different types of thrusters
* more README stuff
* Merge
* increment
* Copy similar sensor yaml files for thrusters, needs to be adjusted, particularly utils.py
* increment
* merging default into branch
* README incremental
* added more comments
* Added Quick Start Instructions
* added README for filling out the YAML file
* fixed for real this time
* fixed build problem
* Merged in yaml_sensor_configuration (pull request #99)
  Yaml sensor configuration
  Approved-by: Carlos Agüero <cen.aguero@gmail.com>
* added README
* updated README
* Merged in ykhky/vrx/Issue#49-collision-detection (pull request #94)
  Issue#49 collision detection
  Approved-by: Carlos Agüero <cen.aguero@gmail.com>
* calling on collision
* spelling correcting
* move variables to correct section in header file
* OnCollision virtual + documenting stuff+ renaming variables
* remove extra bracket
* logging collisions and timestamps
* spacing
* removing world name hard code
* remove cout + adding buffer to nav task
* formatting + exposing collision buffer
* Doc format
* counter + cleanup
* frequency of collision reporting reduced to 1/3 Hz
* added collision detection node
* restored sensors params to sandisland.launch
* Added wavegauge plugin to visualize physical wave height.  Setup example with buoy world.  Implemented simplified wave height calculation in WavefieldSampler for regularly spaced grid (steepness=1=0).
* removed directory xacro checking and variance features
* incremental
* incremental
* Added sequence override option in YAML
* verifying with examples
* increased flexibility of compliance.py
* fixed xacro parsing bug
* Added support for sequence breakout specified in yaml file
* Added xacros for feild elements
* toward buoy examples
* Added launch file
* Script will now be installed, added coordinate generation
* merge
* incremental
* merge
* now auto-generates the world.xacro(may need to be changed to devel) file in src
* incremental
* merge, added launch file
* incremental
* merge
* made branch
* fixed build issue for real this time
* fixed build issue
* commited setup.py, removed unrelated files from vrx_gazebo_python
* scripts will now be installed
* updated readme, changed operation procedure, still not installed
* fixed styling problems with flake8, updated readme
* incremental
* Add mono_camera mesh and .sdf .config files with correct collision and inertia
* changed directory, added launch file support
* incremental, now supports macros with no parameters
* made boiler plate usage more flexible
  H: Enter commit message.  Lines beginning with 'HG:' are removed.
* incremental
* Add sensor_post_arm.dae
* Break sensor_post.dae into two files, then fix model
* Add fixed joint and position arm relative to post
* Add sensor post mesh with correct collision and inertia
* merging default into named branch
* incremental
* Added readme
* moved script. Improved File System
* Flip the ground stations and spread the posts.
* added chairs
* Adding chairs.
* Change cpu case collision box from 1 box to 2 boxes
* incremental
* Add CPU cases only in VRX configuration + remove redundant pose info
* removed pose 0 tags from models
* Tweak indentation.
* documentation, incremental
* incremental
* incremental
* fixed battery/model.sdf
* Add 3D Lidar mesh and put it on WAM-V
* Fix formatting (tab->spaces, etc.)
* Fix .sdf file
* Add CPU case model to WAM-V
* review commented implemented
* finished ground station without chairs
* added table
* added tent and antenna model
* incremental
* incremental
* Added Batteries to vrx_gazebo/models(sdf format) and macro(urdf format) to place on wamv
* Updated texture with a flat area in the beach to place the tents in the future.
* Tweaks.
* Using WAM-V yaw in setting where objects are moved during perception task
* Minor tweak.
* moving station keeping goal closer to wam-v spawn point
* turning wind off to better test - tweaking waypoints in wayfinding task example
* Tweaking positions and adding post and navigation course.
* Restoring cameras and laser visuals and creating demo.launch
* Sandisland texture, sensor meshes and extra objects.
* Restore generate_xxx
* Tweak CMakeLists.txt
* Run the plugin at 1Hz sim time.
* Use sim time to update the light buoy plugin.
* Fix placard symbols.
* Deterministic sequence in light buoy plugin
* Use a ROS subscription for changing the color sequence.
* Modify velodyne configuration to set intensity filtering
  Alter ocean laser retro to be filtered by the lidar sensor
* Remove more trailing whitespace
  Redundant codepath in usv_gazwebo_dynamics_plugin removed.  Euler values now derived identically between gazebo 7 and 9.
* Fix trailing whitespace
* Use auto keyword
* Fix ign method for staionkeeping_scoing_plugin
* Alter patch to use .Ign method to convert between gazebo::math and Ignition::math types
* Fix indention
* Add support for Kinetic/Gazebo-7
  The ignition types are mostly kept, with code transforming from the methods deprecated in gazebo-8
* adding a rqt config file for a perspective task tutorial
* Issue #23: Coordinate the physics and visualization of the wave field
  1. Use the asv_wave_sim_gazebo_plugins package for wave field visualisation and depth calculation.
  2. Update the buoyancy and dynamics plugins for buoyancy calculations.
  3. Update sdf and xacro for models that require buoyancy.
  4. Replace the ocean model with ocean_waves in the sandisland world.
* Red placards and rearrange a bit the sensors.
* Port to VRX code using Gazebo9.
* Contributors: Brian Bingham <briansbingham@gmail.com>, Carlos Aguero, Carlos Aguero <caguero@osrfoundation.org>, Carlos Agüero <caguero@osrfoundation.org>, Carlos Agüero <cen.aguero@gmail.com>, Jonathan Wheare <jonathan.wheare@flinders.edu.au>, MarshallRawson, Rhys Mainwaring <rhys.mainwaring@me.com>, Rumman Waqar <rumman.waqar05@gmail.com>, Tyler Lum <tylergwlum@gmail.com>, Youssef Khaky <youssefkhaky@hotmail.com>, m1chaelm

1.0.1 (2019-03-01)
------------------

1.0.0 (2019-02-28)
------------------
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
