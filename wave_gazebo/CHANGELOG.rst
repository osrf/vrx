^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package wave_gazebo
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Merged in wave_visualization_refactor (pull request #114)
  Wave visual / physics refactor
  Approved-by: Carlos Agüero <cen.aguero@gmail.com>
* Removed gazebo messaging, introduces redundancy in model.sdf for ocean. USV and buoyancy plugins only get wave parameters once instead of every update.
* Added wavegauge plugin to visualize physical wave height.  Setup example with buoy world.  Implemented simplified wave height calculation in WavefieldSampler for regularly spaced grid (steepness=1=0).
* Modifications from original source for integration in VRX
* Adding two packages from asv_wave_sim as a part of VRC
* Contributors: Brian Bingham <briansbingham@gmail.com>, Carlos Aguero, Carlos Aguero <caguero@osrfoundation.org>, Carlos Agüero <cen.aguero@gmail.com>, MarshallRawson

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
