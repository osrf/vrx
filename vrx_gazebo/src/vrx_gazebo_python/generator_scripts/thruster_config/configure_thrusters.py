#!/usr/bin/env python
import rospy
import os

from compliance import Compliance
from .. utils import macro_block_gen
# generate '../wamv_description/urdf/thrusters.xacro'


def main():
    comp = Compliance() 
    # Setup parameter variables

    # yaml requesting which thrusters with what parameters go where
    requested = rospy.get_param('requested')

    # where the macro block is going
    target=rospy.get_param('xacro_target')

    # things to start/open the macro
    boiler_plate_top=('<?xml version="1.0"?>\n' + 
            '<robot xmlns:xacro="http://ros.org/wiki/xacro" ' +
            'name="wam-v-thrusters">\n' +
            '  <xacro:include filename="$(find wamv_description)/urdf/thrusters/engine.xacro" />\n')

    # things to close the macro
    boiler_plate_bot = '</robot>'

    # function to test the number of a type of thruster is allowed
    num_test = comp.thruster_number_compliance

    # function to test if parameters of a thruster are in compliance
    param_test = comp.param_compliance

    macro_block_gen(requested=requested,
                    target=target,
                    boiler_plate_top=boiler_plate_top,
                    boiler_plate_bot=boiler_plate_bot,
                    num_test=num_test,
                    param_test=param_test,
                    macro_type="thruster"
                    )
    
    os.system('rosrun xacro xacro --inorder -o ' +
              rospy.get_param('wamv_target') +
              " '" + rospy.get_param('wamv_gazebo') + "' " +
              'yaml_thruster_generation:=true ' +
             'thruster_xacro_file:=' +
              rospy.get_param('xacro_target'))
    
    print 'wamv sucessfully generated'
 
 
 
 
 
 
 
 
