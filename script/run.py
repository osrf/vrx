#!/usr/bin/env python
import os
from compliance import param_compliance, sensor_number_compliance
from utils import macro_block_gen
macro_block_gen(avalible='../wamv_gazebo/urdf/sensors/',
                requested='sensor_config.yaml',
                target='../wamv_description/urdf/sensors.xacro',
                boiler_plate='<?xml version="1.0"?>\n<robot xmlns:xacro="http://ros.org/wiki/xacro" name="wam-v-sensors">\n<xacro:macro name="sensors">\n',
                num_test = sensor_number_compliance,
                param_test = param_compliance)
cmd = "roslaunch vrx_gazebo sandisland.launch"
os.system(cmd)
