# Ros imports
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace
from launch.substitutions import LaunchConfiguration

# Local imports
import os.path


# --------------------------------------------------------------------------------
def generate_launch_description():
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')
    
    model_dir = os.path.join(get_package_share_directory('vrx_gazebo'), 'models', 'wamv', 'tmp')
    urdf_file = os.path.join(model_dir, 'model.urdf')
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    # Robot state publisher for publishing static joints tf and urdf
    rsp_params = {'robot_description': robot_desc, 'use_sim_time': use_sim_time}
    rsp = Node(package='robot_state_publisher',
                executable='robot_state_publisher',
                output='screen',
                parameters=[rsp_params])
    
    # Joint state publisher for publishing joint tf and urdf
    jsp_params = {'tf_prefix': 'wamv', 'use_sim_time': use_sim_time}
    jsp = Node(package='joint_state_publisher',
                executable='joint_state_publisher',
                output='screen',
                parameters=[jsp_params])
    
    # rviz
    rviz = Node(
            package='rviz2',
            namespace='',
            executable='rviz2',
            name='rviz2',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['-d' + os.path.join(get_package_share_directory('vrx_gazebo'), 'config', 'rviz.config.rviz')]
        )

    group_action = GroupAction([
                PushRosNamespace("wamv"),
                rsp, 
                jsp,
                rviz
            ])

    return LaunchDescription([group_action])
