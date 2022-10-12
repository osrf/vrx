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

def generate_launch_description():

    parameters_file = os.path.join(
        get_package_share_directory('vrx_gz'),
        'config', 'wamv.yaml'
    )

    ld = LaunchDescription([
        launch.actions.DeclareLaunchArgument('cmd_vel', default_value='cmd_vel'),
        launch.actions.DeclareLaunchArgument('teleop_config', default_value=parameters_file),
    ])

    ld.add_action(launch_ros.actions.Node(package='joy', executable='joy_node'))

    ld.add_action(launch_ros.actions.Node(
        package='joy_teleop', executable='joy_teleop',
        parameters=[launch.substitutions.LaunchConfiguration('teleop_config')]))

    return ld
