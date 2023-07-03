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

    # use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Launch rviz
    package_prefix = get_package_share_directory('vrx_gazebo')
    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([package_prefix, '/launch/rviz.launch.py']))
    
    # Launch localization 
    package_prefix = get_package_share_directory('vrx_localization')
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([package_prefix, '/launch/localization.launch.py']))

    return LaunchDescription([
        rviz,
        localization
    ])
