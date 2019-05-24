#!/usr/bin/env python
import os
from compliance import param_compliance, sensor_number_compliance
import utils
#genrate '../wamv_description/urdf/sensors.xacro'
macro_block_gen(availible='../../wamv_gazebo/urdf/sensors',#directory containing the .xacro files allowed NOTE:(ONE macro per file)
                requested='sensor_config.yaml',#yaml requesting which sensors with what parameters go where 
                target='../../wamv_description/urdf/sensors.xacro',#where the macro block is going
                boiler_plate='<?xml version="1.0"?>\n<robot xmlns:xacro="http://ros.org/wiki/xacro" name="wam-v-sensors">\n<xacro:macro name="sensors">\n',#things to start/open the macro NOTE:the closing </xacro:macro> and</robot> are alread included
                num_test = sensor_number_compliance,#function to test the number of a certain type of sensor is in compliance
                param_test = param_compliance)#function to test if a given instance of a sensor is in compliance
