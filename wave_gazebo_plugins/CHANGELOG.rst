^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package wave_gazebo_plugins
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.2.0 (2019-08-19)
------------------
* Go back to custom material, note if you change mytexture2 -> mytexture, it breaks it from resource group can't find error
* Try to change plane material to use existing reflection material and only edit the texture, but does not work
* Fix code quality to pass pipeline
* Try to change material script to match the newly created texture, did not work
* Add jpg texture mix with ocean, worked decently
* Try to add miniscreen to see the material/texture, but not working for some reason
* Disable clip plane each post render, working very well
* Turn on and off reflection and clip plane in pre/post render
* Rewrite code to match with rendertotexture tutorial
* Add reflection to plane
* Add comments and documentation and removed unneeded parts
* Hide minimap, water constant texture, try get plane to be reflection, shows reflection but wrong geometry
* Add texture material to water
* Go back to orig user camera
* Unsuccessful attempt to switch cameras
* Show difference between Ogre::Cam and gz:rend:Cam position
* Add code from book to use new camera, needs update
* Remove enable/disable refl to fix render issue
* Hide plane from texture
* Change to ogre user camera pos and orient, try but fail shaders
* BIG CLEANUP, removed old unused lines of code
* Add enableRefl and disableRelf
* Scale plane and mesh to show it
* Flip plane to be flat, need to next hide the original water
* Create new texture unit
* Change texture name, miniscreen and plane work but not water
* Try to change ocean to show the texture, did not work yet
* Put texture onto plane
* Make only one visualplugin to remove extra miniscreen
* Add rendertargetlistener to not show miniscreen (still shows because there are two)
* Try to implement it, did not work
* Add WavefieldRenderTargetListener, completely untested
* Update miniscreen continuously
* SUCCESSFULLY show small version in mini screen
* Add view to miniscreen, ugly
* Add miniscreen
* Change position and angle of camera
* Change angle to view something
* Save to image file, it is blank
* Add render texture
* Add texture
* Change to valid image
* Add plane image, looks weird
* Move user camera
* Added a light
* Add render updates
* Add RTShaderSystem
* Add static function variable to differentiate between Ogre names
* Fix scene, still not working
* Not working setup, likely need to use visualptr to get scene
* Add scene ptr
* Add viewport setup
* Add scene nodes and camera setup
* Add root, scenemgr
* Add unworking Ogre texture creation
* Work off ocean model, clean out visual plugin and use new simple material scripts
* Modiying world definitions in wave_gazebo package to use xacro
* Contributors: Brian Bingham <briansbingham@gmail.com>, Tyler Lum <tylergwlum@gmail.com>

1.1.2 (2019-07-10)
------------------
* Workaround to fix compile errors on Kinetic
  The version of ign-math2 present in Ubuntu Xenial (2.2.3) lacks
  of some features (Zero or Length) implemented starting on 2.3.x.
  This change add some preprocessors defines to workaround the
  problem. A more elegant solution would be ideal.
* Contributors: Jose Luis Rivero <jrivero@osrfoundation.org>

1.1.1 (2019-07-03)
------------------

1.1.0 (2019-07-01)
------------------
* Generate changelog for new packages
* Merged in wave_visualization_refactor (pull request #114)
  Wave visual / physics refactor
  Approved-by: Carlos Agüero <cen.aguero@gmail.com>
* Restoring waves parameters.
* Smooth water.
* Style
* Removing gazebo::msg::Param references and cleaning up for gazebo version < 8 compatibility.
* Removed gazebo messaging, introduces redundancy in model.sdf for ocean. USV and buoyancy plugins only get wave parameters once instead of every update.
* style
* adding to docs and allowing for both PMS and CWR wavefield models
* addin PM spectrum
* adding exponential increase in wave field and LaTeX doc^C
* increment
* increment
* Clean up some of the diagnostic messages
* Added wavegauge plugin to visualize physical wave height.  Setup example with buoy world.  Implemented simplified wave height calculation in WavefieldSampler for regularly spaced grid (steepness=1=0).
* verifying with examples
* changing wind to waves
* Added an example to illustrate using request/response to transport the wave_params and fixed a couple tiny typos
* Overtly requiring C++14 for the wave_gazebo_plugins package - required for use of autos in lambda functions.  Only necessary for supporting Kinetic build.
* Setting wave parameters by hand in source for testing
* Removing superfluous models and empty tests
* Changing license text
* Modifications from original source for integration in VRX
* Adding two packages from asv_wave_sim as a part of VRC
* Contributors: Brian Bingham <briansbingham@gmail.com>, Carlos Aguero, Carlos Agüero <cen.aguero@gmail.com>, Jose Luis Rivero <jrivero@osrfoundation.org>

* Merged in wave_visualization_refactor (pull request #114)
  Wave visual / physics refactor
  Approved-by: Carlos Agüero <cen.aguero@gmail.com>
* Removing gazebo::msg::Param references and cleaning up for gazebo version < 8 compatibility.
* Removed gazebo messaging, introduces redundancy in model.sdf for ocean. USV and buoyancy plugins only get wave parameters once instead of every update.
* Added wavegauge plugin to visualize physical wave height.  Setup example with buoy world.  Implemented simplified wave height calculation in WavefieldSampler for regularly spaced grid (steepness=1=0).
* Added an example to illustrate using request/response to transport the wave_params and fixed a couple tiny typos
* Overtly requiring C++14 for the wave_gazebo_plugins package - required for use of autos in lambda functions.  Only necessary for supporting Kinetic build.
* Setting wave parameters by hand in source for testing
* Modifications from original source for integration in VRX
* Adding two packages from asv_wave_sim as a part of VRC
* Contributors: Brian Bingham <briansbingham@gmail.com>, Carlos Aguero, Carlos Agüero <cen.aguero@gmail.com>

1.0.1 (2019-03-01)
------------------

1.0.0 (2019-02-28)
------------------

0.3.3 (2018-10-19)
------------------

0.3.2 (2018-10-08)
------------------

0.3.1 (2018-10-05)
------------------

0.3.0 (2018-09-28)
------------------
