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
from launch.actions import EmitEvent
from launch.actions import ExecuteProcess
from launch.actions import RegisterEventHandler
from launch.events import Shutdown
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

import vrx_gz.bridges

import os

def generate_launch_description():
    gz_args = LaunchConfiguration('gz_args')
    gz_args_launch = DeclareLaunchArgument(
        'gz_args', 
        default_value='',
        description='Arguments to be passed to Gazebo'
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
        get_package_share_directory('ros_gz_sim'), 'launch'),
        '/gz_sim.launch.py']),
        launch_arguments = {'gz_args': gz_args}.items())

    # Register handler for shutting down ros launch when gazebo process exits
    # monitor_sim.py will run until it can not find the gazebo process.
    # Once monitor_sim.py exits, a process exit event is triggered which causes
    # the handler to emit a Shutdown event
    p = os.path.join(get_package_share_directory('vrx_ros'), 'launch',
                     'monitor_sim.py')
    monitor_sim_proc = ExecuteProcess(
        cmd=['python3', p],
        name='monitor_sim',
        output='screen',
    )
    sim_exit_event_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=monitor_sim_proc,
            on_exit=[
                EmitEvent(event=Shutdown(reason='Simulation ended'))
            ]
        )
    )

    bridges = [
      vrx_gz.bridges.score(),
      vrx_gz.bridges.clock(),
      vrx_gz.bridges.run_clock(),
      vrx_gz.bridges.phase(),
    ]

    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        arguments=[bridge.argument() for bridge in bridges],
        remappings=[bridge.remapping() for bridge in bridges],
    )

    wamv_args = {'name': 'wamv',
                 'world': 'sydney_regatta',
                 'model': 'wam-v',
                 'x': '-532',
                 'y': '162',
                 'z': '0',
                 'R': '0',
                 'P': '0',
                 'Y': '1'
                }

    spawn_wamv = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
        get_package_share_directory('vrx_gz'), 'launch'),
        '/spawn.launch.py']),
        launch_arguments = wamv_args.items())

    return LaunchDescription([
        gz_args_launch,
        gz_sim,
        bridge_node,
        spawn_wamv,
        monitor_sim_proc,
        sim_exit_event_handler
        ])
