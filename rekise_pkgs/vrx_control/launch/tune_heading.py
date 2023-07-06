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
import launch.actions
import launch_ros.actions
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    # Launch joystick mapping for heading control 
    package_prefix = get_package_share_directory('vrx_control')
    joy_map_heading_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([package_prefix, '/launch/joy_abs_heading.py']))
    
    # heading controller
    heading_controller = Node(package='vrx_control', executable='absolute_heading_controller.py')
    
    # heading publisher in rpy from quaternion
    qt2rpy_heading = Node(package='vrx_control', executable='angles_in_rpy.py')

    return LaunchDescription([
        joy_map_heading_control,
        heading_controller,
        qt2rpy_heading
    ])
