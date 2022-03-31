#!/usr/bin/env python3
import rospy
import os

from vrx_gazebo.compliance import ComponentCompliance
from vrx_gazebo.compliance import ThrusterCompliance

from vrx_gazebo.utils import create_xacro_file
from vrx_gazebo.utils import add_gazebo_thruster_config


def main():

    rospy.init_node("wamv_generator", anonymous=True)
    # Check if yaml files were given
    received_thruster_yaml = len(rospy.get_param('thruster_yaml')) > 0
    received_component_yaml = len(rospy.get_param('component_yaml')) > 0

    # Setup thruster xacro
    if received_thruster_yaml:
        thruster_compliant = create_thruster_xacro()

    # Setup component xacro
    if received_component_yaml:
        component_compliant = create_component_xacro()

    # Setup command to generate WAM-V urdf file
    wamv_target = rospy.get_param('wamv_target')
    wamv_gazebo = rospy.get_param('wamv_gazebo')
    wamv_locked = rospy.get_param('wamv_locked')

    create_urdf_command = ("rosrun xacro xacro -o " + wamv_target +
                           " '" + wamv_gazebo + "'")

    if wamv_locked:
        create_urdf_command += (" locked:=" +
                                str(wamv_locked))
    
    # Add xacro files if created
    if received_thruster_yaml:
        thruster_yaml = rospy.get_param('thruster_yaml')
        thruster_xacro_target = os.path.splitext(thruster_yaml)[0] + '.xacro'
        create_urdf_command += (" yaml_thruster_generation:=true "
                                "thruster_xacro_file:=" +
                                thruster_xacro_target)
    if received_component_yaml:
        component_yaml = rospy.get_param('component_yaml')
        component_xacro_target = os.path.splitext(component_yaml)[0] + '.xacro'
        create_urdf_command += (" yaml_component_generation:=true "
                                "component_xacro_file:=" + component_xacro_target)

    # Create urdf and print to console
    os.system(create_urdf_command)
    if not (thruster_compliant and component_compliant):
        rospy.logerr('\nThis component/thruster configuration is NOT compliant ' +
                     'with the (current) VRX constraints. A urdf file will ' +
                     'be created, but please note that the above errors ' +
                     'must be fixed for this to be a valid configuration ' +
                     'for the VRX competition.\n')

    print('WAM-V urdf file sucessfully generated. File location: ' +
          wamv_target)

def create_thruster_xacro():
    """
    Purpose: Create a thruster xacro file using the given
             rosparameters
    """
    # Get yaml files for thruster number and pose
    thruster_yaml = rospy.get_param('thruster_yaml')
    rospy.loginfo('\nUsing %s as the thruster configuration yaml file\n' %
                  thruster_yaml)

    # Set thruster xacro target
    thruster_xacro_target = os.path.splitext(thruster_yaml)[0] + '.xacro'
    rospy.loginfo('\nTrying to open %s \n' % thruster_xacro_target)

    # Things to start/open the macro
    thruster_boiler_plate_top = ('<?xml version="1.0"?>\n'
                                 '<robot '
                                 'xmlns:xacro="http://ros.org/wiki/xacro" '
                                 'name="wam-v-thrusters">\n'
                                 '  <xacro:include filename='
                                 '"$(find wamv_description)/urdf/thrusters/'
                                 'engine.xacro" />\n')

    # Things to close the macro
    thruster_boiler_plate_bot = ''

    # Check if valid number of thrusters and valid thruster parameters
    comp = ThrusterCompliance()
    thruster_num_test = comp.number_compliance
    thruster_param_test = comp.param_compliance

    # Create thruster xacro with thruster macros
    compliant = create_xacro_file(yaml_file=thruster_yaml,
                                  xacro_target=thruster_xacro_target,
                                  boiler_plate_top=thruster_boiler_plate_top,
                                  boiler_plate_bot=thruster_boiler_plate_bot,
                                  num_test=thruster_num_test,
                                  param_test=thruster_param_test)

    gz_boiler_plate_top = ('  <gazebo>\n'
                           '    <plugin name="wamv_gazebo_thrust" '
                           'filename="libusv_gazebo_thrust_plugin.so">\n'
                           '      <cmdTimeout>1.0</cmdTimeout>\n'
                           '      <robotNamespace>${namespace}</robotNamespace>\n'
                           '      <xacro:include filename="$(find wamv_gazebo)'
                           '/urdf/thruster_layouts/'
                           'wamv_gazebo_thruster_config.xacro" />\n')
    gz_boiler_plate_bot = ('    </plugin>\n'
                           '  </gazebo>\n'
                           '</robot>')

    # Append gazebo thruster config to thruster xacro
    add_gazebo_thruster_config(yaml_file=thruster_yaml,
                               xacro_target=thruster_xacro_target,
                               boiler_plate_top=gz_boiler_plate_top,
                               boiler_plate_bot=gz_boiler_plate_bot,
                               )
    return compliant


def create_component_xacro():
    """
    Purpose: Create a component xacro file using the given
             rosparameters
    """
    # Get yaml files for component number and pose
    component_yaml = rospy.get_param('component_yaml')
    rospy.loginfo('\nUsing %s as the component configuration yaml file\n' %
                  component_yaml)

    # Set component xacro target
    component_xacro_target = os.path.splitext(component_yaml)[0] + '.xacro'

    # Things to start/open the macro
    component_boiler_plate_top = ('<?xml version="1.0"?>\n'
                               '<robot '
                               'xmlns:xacro="http://ros.org/wiki/xacro" '
                               'name="wam-v-components">\n' +
                               '  <xacro:macro name="yaml_components">\n')

    # Things to close the macro
    component_boiler_plate_bot = '  </xacro:macro>\n</robot>'

    # Check if valid number of components and valid component parameters
    comp = ComponentCompliance()
    component_num_test = comp.number_compliance
    component_param_test = comp.param_compliance

    # Create component xacro with component macros
    return create_xacro_file(yaml_file=component_yaml,
                             xacro_target=component_xacro_target,
                             boiler_plate_top=component_boiler_plate_top,
                             boiler_plate_bot=component_boiler_plate_bot,
                             num_test=component_num_test,
                             param_test=component_param_test)

