TUTORIAL
Lets start with a though experiment.

Lets say we have code for a robot written and are able to simulate this robot in Gazebo. Lets also say we need to test this code on doing 2 tasks in bright and dark ligthing conditions.

We could write all 4 world files COMPLETELY by hand, but that would be tedious, so lets write them with xacro macros (see xacro tutorial if this is new to you).

But what if in the future it turns out that your robot needs to work in 3 diffrent weather conditions, 3 NOT 2 lighting conditions, and do a whole OTHER task/feild setup. Now, to test all these factors independently from one another we need to manually write the xacros for 27 (3*3*3) worlds!!!

That is far too tedious, if only we had a program to generate all 27 of those world files!

Good News, we do!

1.
create a directory some where to hold some things ie:
mkdir ~/generated_worlds
cd generated_worlds/

2.
create a yaml according to the yaml filling instructions (also see example) ie:
gedit worlds.yaml

(example as applied to Robot X)

########worlds.yaml###########
#we need to decide what is constant between all of our simulations. We will call the 'axis' constant
constant:
    #the constant axis will have only one variation on itself, so 1 step.
    steps: 1
    #this is a list of the macros that this axis will contribute to a given world
    macros:
        #for RobotX, the sandilsand xacro macro is constant
        #NOTE: to refence a macro here, it's file MUST be included by vrx_gazebo/worlds/xacors/include_all_xacros.xacro
        sandisland:
        #sandisland has no parameters, but all macros MUST have a (-) indented on the line below to show that there is at least ONE instance of them(even if it is empty, more on this later)
            -
   #every axis MUST have a sequence feild (even if it is empty)(more on what sequence does later)
   sequence:
#now lets get those tasks in here
tasks:
    macros:
        









YAML filling RULES:
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
roslaunch vrx_gazebo generate_worlds.launch requested:=/home/<username>/generated_worlds/worlds.yaml world_xacro_target:=/home/<username>/generated_worlds/world_xacros/ world_target:=/home/<username>/generated_worlds/worlds/ --screen

5.
see the success message: All  <n>  worlds generated

6.
examine the generated xacros under world_xacros/ and make sure they are what you want. (these are meant to be more human readable than the .worlds files) ie:
gedit world_xacros/world0.world.xacro

7.
run one of your new worlds:
roslaunch vrx_gazebo sandisland.launch world:=/home/<username>/generated_worlds/worlds/world0.world

