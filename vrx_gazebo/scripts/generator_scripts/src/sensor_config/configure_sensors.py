#!/usr/bin/env python
import rospy


from compliance import param_compliance, sensor_number_compliance
from .. utils import macro_block_gen
# generate '../wamv_description/urdf/sensors.xacro'
macro_block_gen(available=rospy.get_param('available'),
                # directory containing the .xacro files allowed
                # NOTE:(ONE macro per file)
                requested=rospy.get_param('requested'),
                # yaml requesting which sensors with what parameters go where
                target=rospy.get_param('target'),
                # where the macro block is going
                boiler_plate_top='<?xml version="1.0"?>\n' +
                '<robot xmlns:xacro="http://ros.org/wiki/xacro" ' +
                'name="wam-v-sensors">\n' +
                '  <xacro:macro name="sensors">\n',
                # things to start/open the macro
                boiler_plate_bot='  </xacro:macro>\n</robot>',
                # things to close the macro
                num_test=sensor_number_compliance,
                # function to test the number of a type of sensor is allowed
                param_test=param_compliance
                # function to test if parameters of a sensor are in compliance
                )
print 'sensors sucessfully genreated'
