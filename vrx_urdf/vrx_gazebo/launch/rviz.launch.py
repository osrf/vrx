from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os.path



def generate_launch_description():

    urdf_file = os.path.join(get_package_share_directory('vrx_gazebo'), 'models', 'wamv', 'tmp', 'model.urdf')
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()
    params = {'robot_description': robot_desc}

    rsp = Node(package='robot_state_publisher',
                                  executable='robot_state_publisher',
                                  output='both',
                                  parameters=[params])
    jsp = Node(package='joint_state_publisher',
                                  executable='joint_state_publisher',
                                  output='both',
                                  parameters=[])
    tfp = Node(package='tf2_ros',
                                  executable='static_transform_publisher',
                                  arguments=['0', '0', '0', '0', '0', '0', 'world', 'wamv/base_link']) 
    rvz = Node(
            package='rviz2',
            namespace='',
            executable='rviz2',
            name='rviz2',
            arguments=['-d' + os.path.join(get_package_share_directory('vrx_gazebo'), 'config', 'rviz_vrx.rviz')]
        )
    return LaunchDescription([rsp,jsp,tfp,rvz])