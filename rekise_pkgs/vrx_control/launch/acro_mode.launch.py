# Copyright 2022 Open Source Robotics Foundation.
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
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
import launch_ros.actions
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    # joy stick launch
    package_prefix = get_package_share_directory('vrx_control')
    joy_controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([package_prefix, '/launch/joy_acro_mode.py']))
    
    # joystick command consolidator
    joy_cmd_consolidator = Node(package='vrx_control', executable='joy_command_consolidator.py')
    
    # controllers
    yaw_rate_controller  = Node(package='vrx_control', executable='yaw_rate_controller.py')
    fwd_vel_controller   = Node(package='vrx_control', executable='forward_velocity_controller.py')

    return LaunchDescription([
        joy_controller,
        joy_cmd_consolidator,
        yaw_rate_controller,
        fwd_vel_controller
    ])

