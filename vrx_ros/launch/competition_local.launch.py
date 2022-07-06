# Copyright 2022 Open Source Robotics Foundation, Inc.
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

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.actions import ExecuteProcess
from launch.actions import OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

import vrx_ign.bridges

import os

def launch(context, *args, **kwargs):

    ign_args = LaunchConfiguration('ign_args').perform(context)

    ign_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
        get_package_share_directory('ros_ign_gazebo'), 'launch'),
        '/ign_gazebo.launch.py']),
        launch_arguments = {'ign_args': ign_args}.items())


    bridges = [
      vrx_ign.bridges.score(),
      vrx_ign.bridges.clock(),
      vrx_ign.bridges.run_clock(),
      vrx_ign.bridges.phase(),
      vrx_ign.bridges.stream_status(),
    ]
    nodes = []
    nodes.append(Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        output='screen',
        arguments=[bridge.argument() for bridge in bridges],
        remappings=[bridge.remapping() for bridge in bridges],
    ))


    return [ign_gazebo,
            *nodes]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('ign_args', default_value='',
            description='Arguments to be passed to Ignition Gazebo'),
        OpaqueFunction(function = launch)
        ])

