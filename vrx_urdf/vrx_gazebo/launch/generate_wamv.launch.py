# Copyright 2021 Open Source Robotics Foundation, Inc.
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

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def launch(context, *args, **kwargs):
    wamv_locked = LaunchConfiguration('wamv_locked').perform(context)
    component_yaml = LaunchConfiguration('component_yaml').perform(context)
    thruster_yaml = LaunchConfiguration('thruster_yaml').perform(context)
    wamv_target = LaunchConfiguration('wamv_target').perform(context)

    if not component_yaml:
        component_yaml = os.path.join(get_package_share_directory('vrx_gazebo'),
                                      'config', 'wamv_config', 'example_component_config.yaml')
    if not thruster_yaml:
        thruster_yaml = os.path.join(get_package_share_directory('vrx_gazebo'),
                                     'config', 'wamv_config', 'example_thruster_config.yaml')

    components_dir = os.path.join(get_package_share_directory('wamv_gazebo'),
                                  'urdf', 'components')
    thrusters_dir = os.path.join(get_package_share_directory('wamv_description'),
                                 'urdf', 'thrusters')
    wamv_gazebo = os.path.join(get_package_share_directory('wamv_gazebo'),
                                 'urdf', 'wamv_gazebo.urdf.xacro')

    node = Node(package='vrx_gazebo',
                executable='generate_wamv.py',
                output='screen',
                parameters=[{'wamv_locked': wamv_locked},
                            {'component_yaml': component_yaml},
                            {'thruster_yaml': thruster_yaml},
                            {'wamv_target': wamv_target},
                            {'components_dir': components_dir},
                            {'thrusters_dir': thrusters_dir},
                            {'wamv_gazebo': wamv_gazebo}])

    return [node]

def generate_launch_description():
    return LaunchDescription([
        # Launch Arguments
        DeclareLaunchArgument(
            'wamv_locked',
            default_value='False',
            description='WAM-V locked'),
        DeclareLaunchArgument(
            'component_yaml',
            default_value='',
            description='Path to component yaml file.'),
        DeclareLaunchArgument(
            'thruster_yaml',
            default_value='',
            description='Path to thruster yaml file.'),
        DeclareLaunchArgument(
            'wamv_target',
            default_value='',
            description='WAM-V target output URDF file'),
        OpaqueFunction(function=launch),
    ])
