yaml filling instructions:
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

SEQUENCE:
each axis MUST have a sequence tag at the same level as steps and macros.
the sequence feature provides the ability to breakout of the programmed sequence.
the numbers after sequence tag refer to the step which is being overridden.
the over ride must be a number and nothing else.
any macro excluded in a step's override will not be included in the the worlds created with this step of this axis.
the whole set of macros (for that axis) must be specified as numbers.
The params of the macros in a sequence override will NOT be evaluated as a Lambda, they must be the literal values.


