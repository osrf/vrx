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
    params = {'robot_description': robot_desc}

    rdp = Node(package='vrx_gazebo',
                                  executable='robot_desc_publisher.py',
                                  output='both',
                                  parameters=[params])
    rvz = Node(
            package='rviz2',
            namespace='',
            executable='rviz2',
            name='rviz2',
            arguments=['-d' + os.path.join(get_package_share_directory('vrx_gazebo'), 'config', 'rviz_vrx_final.rviz')]
        )
    return LaunchDescription([rdp,rvz])