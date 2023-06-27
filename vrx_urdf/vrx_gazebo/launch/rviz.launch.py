from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os.path



def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    model_dir = os.path.join(get_package_share_directory('vrx_gazebo'), 'models/wamv/tmp')
    urdf_file = os.path.join(model_dir, 'model.urdf')
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()
    DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'),
    params = {'use_sim_time': use_sim_time, 'frame_prefix': 'wamv/', 'robot_description': robot_desc, 'publish_frequency': 0.001}

    rsp = Node(package='robot_state_publisher',
                                  executable='robot_state_publisher',
                                  output='both',
                                  parameters=[params])
    rvz = Node(
            package='rviz2',
            namespace='',
            executable='rviz2',
            name='rviz2',
            arguments=['-d' + os.path.join(get_package_share_directory('vrx_gazebo'), 'config', 'rviz_vrx_final.rviz')]
        )
    return LaunchDescription([rsp,rvz])