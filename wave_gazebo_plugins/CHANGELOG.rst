^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package wave_gazebo_plugins
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
