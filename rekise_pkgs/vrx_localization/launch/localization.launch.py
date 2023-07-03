# Copyright 2018 Open Source Robotics Foundation, Inc.
# Copyright 2019 Samsung Research America
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
import os
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace
from launch.actions import GroupAction
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # navsat node
    navsat_params_file = os.path.join(get_package_share_directory("vrx_localization"), 'params', 'navsat.yaml')
    navsat_transform_node = Node(
       package='robot_localization',
       executable='navsat_transform_node',
       name='navsat_transform_node',
       output='screen',
       parameters=[
           navsat_params_file,
           {'use_sim_time': use_sim_time}
        ],
       remappings=[
            ('/gps/fix', '/wamv/sensors/gps/gps/fix'),
            ('odometry/gps', '/wamv/sensors/gps/gps/odometry'),
            ('imu', '/wamv/sensors/imu/imu/data'),
            #('imu/data', '/wamv/sensors/imu/imu/data'),
            # ('odometry/filtered', '/robot_localization/odometry/filtered'),
            # ('gps/filtered', '/wamv/robot_localization/gps/gps/filtered'),
        ],
    )

    # ekf node
    ekf_params_file = os.path.join(get_package_share_directory("vrx_localization"), 'params', 'ekf.yaml')
    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            ekf_params_file, 
            {'use_sim_time': use_sim_time},
            {'imu0' : '/wamv/sensors/imu/imu/data'},
            {'odom0': '/wamv/sensors/gps/gps/odometry'},
        ])
        
    # package_prefix = get_package_share_directory('vrx_gazebo')
    # rviz_r = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([package_prefix, '/launch/rviz.launch.py']))

    tf_map_to_odom            = Node(package='tf2_ros', 
                                     executable='static_transform_publisher',
                                     arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']) 
    
    tf_imu_link_to_imu_sensor = Node(package='tf2_ros',
                                     executable='static_transform_publisher',
                                     arguments=['0', '0', '0', '0', '0', '0', 'wamv/imu_wamv_link', 'wamv/wamv/imu_wamv_link/imu_wamv_sensor']) 

    tf_gps_link_to_gps_sensor = Node(package='tf2_ros',
                                     executable='static_transform_publisher',
                                     arguments=['0', '0', '0', '0', '0', '0', 'wamv/gps_wamv_link', 'wamv/wamv/gps_wamv_link/navsat']) 
    

    return LaunchDescription([
        robot_localization_node, 
        navsat_transform_node, 
        tf_map_to_odom, 
        tf_imu_link_to_imu_sensor, 
        tf_gps_link_to_gps_sensor,
        # rviz_r
    ])
