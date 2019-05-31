#!/usr/bin/env python
import rospy
import os

from compliance import param_compliance, sensor_number_compliance
from .. utils import macro_block_gen
# generate '../wamv_description/urdf/sensors.xacro'


def main():
  macro_block_gen(available=rospy.get_param('available'),
                  # directory containing the .xacro files allowed
                  # NOTE:(ONE macro per file)
                  requested=rospy.get_param('requested'),
                  # yaml requesting which sensors with what parameters go where
                  target=rospy.get_param('xacro_target'),
                  # where the macro block is going
                  boiler_plate_top='<?xml version="1.0"?>\n' +
                  '<robot xmlns:xacro="http://ros.org/wiki/xacro" ' +
                  'name="wam-v-sensors">\n' +
                  '  <xacro:macro name="yaml_sensors">\n',
                  # things to start/open the macro
                  boiler_plate_bot='  </xacro:macro>\n</robot>'#,
                  # things to close the macro
                  #num_test=sensor_number_compliance,
                  # function to test the number of a type of sensor is allowed
                  #param_test=param_compliance
                  # function to test if parameters of a sensor are in compliance
                   )
 
  os.system('rosrun xacro xacro --inorder -o ' +
            rospy.get_param('wamv_target') +
            " '" + rospy.get_param('wamv_gazebo') + "' " +
            'yaml_sensor_generation:=true ' +
           'sensor_xacro_file:=' +
            rospy.get_param('xacro_target'))
 
  print 'wamv sucessfully generated'
 
 
 
 
 
 
 
 
