#!/usr/bin/env python
import os
import rospy


from compliance import param_compliance, sensor_number_compliance
from .. utils import macro_block_gen
#generate '../wamv_description/urdf/sensors.xacro'
macro_block_gen(availible=rospy.get_param('availible'),#directory containing the .xacro files allowed NOTE:(ONE macro per file)
                requested=rospy.get_param('requested'),#yaml requesting which sensors with what parameters go where 
                target=rospy.get_param('target'),#where the macro block is going
                boiler_plate_top='<?xml version="1.0"?>\n<robot xmlns:xacro="http://ros.org/wiki/xacro" name="wam-v-sensors">\n<xacro:macro name="sensors">\n',#things to start/open the macro
                boiler_plate_bot = '  </xacro:macro>\n</robot>',#things to close the macro
                num_test = sensor_number_compliance,#function to test the number of a certain type of sensor is in compliance
                param_test = param_compliance)#function to test if a given instance of a sensor is in compliance
