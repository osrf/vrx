YAML filling instructions:
AXIS:
each initial feild of the yaml is considered its own axis(the name of the axies do not matter, so long they are unique)

STEPS:
each axis MUST have a steps field of at least 1.
the generated world xacros and wordls will reflect the linear combinations of all axis
 ie: 3 axies with 3 steps each will generate 27 world files

MACROS:
each axis MUST have a macros feild at the same level of steps.
this is a list of xacro macros which will be contributed to the world files by this axis

MACROS PARAMS:
each macro must have at least one instance (-)
under each instance of a macro, the parameters are to be filled.
multiple instances of a macro may be included by adding more (-)
(most macros have a name feild which must be filled(sandisland is an exception))
params will be functionally determined by the step of its axis(n) as evaluated as a lambda in pyhton.
NOTE: desired macros must be included by vrx_gazebo/worlds/xacros/include_all_macros.xacro. Otherwise, XML erros will be produced


SEQUENCE:
each axis MUST have a sequence tag at the same level as steps and macros.
the sequence feature provides the ability to breakout of the programmed sequence.
the numbers after sequence tag refer to the step which is being overridden.
the over ride must be a number and nothing else.
any macro excluded in a step's override will not be included in the the worlds created with this step of this axis.
the whole set of macros (for that axis) must be specified as numbers.
The params of the macros in a sequence override will NOT be evaluated as a Lambda, they must be the literal values.


Quick Start Instructions:
1.
create a directory some where to hold some things ie:
mkdir ~/generated_worlds
cd generated_worlds/

2.
create a yaml according to the yaml filling instructions (also see example) ie:
gedit worlds.yaml

3.
make a new directory to hold the world xacros for convienience ie:
mkdir world_xacros/

4.
same for worlds
mkdir worlds/

4.
run the script:
roslaunch vrx_gazebo generate_worlds.launch requested:=/home/<username>/generated_worlds/worlds.yaml world_xacro_target:=/home/<username>/generated_worlds/world_xacros/

5.
examine the generated xacros under world_xacros/ and make sure they are what you want. (these are meant to be more human readable than the .worlds files generated in your devel space) ie:
gedit world_xacros/world0.world.xacro

6.
run one of your new worlds:
roslaunch vrx_gazebo sandisland.launch world:=/home/<username>/generated_worlds/worlds/world0.world

