#!/usr/bin/env python
import rospy
import os

from compliance import Sensor_Compliance
from compliance import Thruster_Compliance

from .. utils import macro_block_gen


def main():

    received_thruster_yaml = len(rospy.get_param('thruster_yaml')) > 0
    received_sensor_yaml = len(rospy.get_param('sensor_yaml')) > 0
    
    ## Setup thruster xacro
    if received_thruster_yaml:
        print("Received thruster yaml")
        create_thruster_xacro()

    ## Setup sensor xacro
    if received_sensor_yaml:
        print("Received sensor yaml")
        create_sensor_xacro()

    ## Setup command to generate WAM-V urdf file
    wamv_target = rospy.get_param('wamv_target')
    wamv_gazebo = rospy.get_param('wamv_gazebo')

    create_urdf_command = "rosrun xacro xacro --inorder -o " + wamv_target + " '" + wamv_gazebo + "'"

    ## Add xacro files if created
    if received_thruster_yaml:
        thruster_xacro_target = change_to_xacro_extension(rospy.get_param('thruster_yaml'))
        create_urdf_command += " yaml_thruster_generation:=true thruster_xacro_file:=" + thruster_xacro_target
    if received_sensor_yaml:
        sensor_xacro_target = change_to_xacro_extension(rospy.get_param('sensor_yaml'))
        create_urdf_command += " yaml_sensor_generation:=true sensor_xacro_file:=" + sensor_xacro_target
    print("Create urdf command: " + create_urdf_command)

    os.system(create_urdf_command)
    print('WAM-V urdf file sucessfully generated. File location: ' + wamv_target)
    
def create_thruster_xacro():
    # Get yaml files for thruster number and pose
    thruster_yaml = rospy.get_param('thruster_yaml')

    # Set thruster xacro target
    thruster_xacro_target = change_to_xacro_extension(thruster_yaml)

    # Things to start/open the macro
    thruster_boiler_plate_top=('<?xml version="1.0"?>\n' + 
            '<robot xmlns:xacro="http://ros.org/wiki/xacro" name="wam-v-thrusters">\n' +
            '  <xacro:include filename="$(find wamv_description)/urdf/thrusters/engine.xacro" />\n')

    # things to close the macro
    thruster_boiler_plate_bot = '</robot>'
    print("thruster_yaml: " + thruster_yaml)
    print("thruster_xacro_target: " + thruster_xacro_target)

    # Check if valid number of thrusters and valid thruster parameters
    comp = Thruster_Compliance() 
    thruster_num_test = comp.number_compliance
    thruster_param_test = comp.param_compliance

    macro_block_gen(yaml_file=thruster_yaml,
                    xacro_target=thruster_xacro_target,
                    boiler_plate_top=thruster_boiler_plate_top,
                    boiler_plate_bot=thruster_boiler_plate_bot,
                    num_test=thruster_num_test,
                    param_test=thruster_param_test,
                    macro_type="thruster"
                    )

def create_sensor_xacro():
    # Get yaml files for sensor number and pose
    sensor_yaml = rospy.get_param('sensor_yaml')

    # Set sensor xacro target
    sensor_xacro_target = change_to_xacro_extension(sensor_yaml)

    # Things to start/open the macro
    sensor_boiler_plate_top=('<?xml version="1.0"?>\n' + 
            '<robot xmlns:xacro="http://ros.org/wiki/xacro" name="wam-v-sensors">\n' +
            '  <xacro:macro name="yaml_sensors">\n')

    # things to close the macro
    sensor_boiler_plate_bot = '  </xacro:macro>\n</robot>'
    print("sensor_yaml: " + sensor_yaml)
    print("sensor_xacro_target: " + sensor_xacro_target)

    # Check if valid number of sensors and valid sensor parameters
    comp = Sensor_Compliance() 
    sensor_num_test = comp.number_compliance
    sensor_param_test = comp.param_compliance

    macro_block_gen(yaml_file=sensor_yaml,
                    xacro_target=sensor_xacro_target,
                    boiler_plate_top=sensor_boiler_plate_top,
                    boiler_plate_bot=sensor_boiler_plate_bot,
                    num_test=sensor_num_test,
                    param_test=sensor_param_test,
                    macro_type="sensor"
                    )

def change_to_xacro_extension(string):
    return string[0:string.index('.')] + '.xacro'
