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

from launch.actions import GroupAction
from launch.actions import IncludeLaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import ExecuteProcess, EmitEvent
from launch.events import Shutdown

from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace

import vrx_gz.bridges

import os

GYMKHANA_WORLDS = [
  'gymkhana_task'
]

PERCEPTION_WORLDS = [
  'perception_task',
  'practice_2023_perception0_task',
  'practice_2023_perception1_task',
  'practice_2023_perception2_task',
  'perception0',
  'perception1',
  'perception2',
  'perception3',
  'perception4',
  'perception5'
]

STATIONKEEPING_WORLDS = [
  'stationkeeping_task',
  'practice_2023_stationkeeping0_task',
  'practice_2023_stationkeeping1_task',
  'practice_2023_stationkeeping2_task',
  'stationkeeping0',
  'stationkeeping1',
  'stationkeeping2',
  'stationkeeping3',
  'stationkeeping4',
  'stationkeeping5'
]

WAYFINDING_WORLDS = [
  'wayfinding_task',
  'practice_2023_wayfinding0_task',
  'practice_2023_wayfinding1_task',
  'practice_2023_wayfinding2_task',
  'wayfinding0',
  'wayfinding1',
  'wayfinding2',
  'wayfinding3',
  'wayfinding4',
  'wayfinding5'
]

WILDLIFE_WORLDS = [
  'wildlife_task',
  'practice_2023_wildlife0_task',
  'practice_2023_wildlife1_task',
  'practice_2023_wildlife2_task',
  'wildlife0',
  'wildlife1',
  'wildlife2',
  'wildlife3',
  'wildlife4',
  'wildlife5'
]

SCAN_DOCK_DELIVER_WORLDS = [
  'scan_dock_deliver_task',
  'practice_2023_scan_dock_deliver0_task',
  'practice_2023_scan_dock_deliver1_task',
  'practice_2023_scan_dock_deliver2_task',
  'scan_dock_deliver0',
  'scan_dock_deliver1',
  'scan_dock_deliver2',
  'scan_dock_deliver3',
  'scan_dock_deliver4',
  'scan_dock_deliver5'
]

ACOUSTIC_TRACKING_WORLDS = [
  'acoustic_tracking_task',
  'practice_2023_acoustic_tracking0_task',
  'practice_2023_acoustic_tracking1_task',
  'practice_2023_acoustic_tracking2_task',
  'acoustic_tracking0',
  'acoustic_tracking1',
  'acoustic_tracking2',
  'acoustic_tracking3',
  'acoustic_tracking4',
  'acoustic_tracking5'
]

FOLLOWPATH_WORLDS = [
  'follow_path_task',
  'practice_2023_follow_path0_task',
  'practice_2023_follow_path1_task',
  'practice_2023_follow_path2_task',
  'follow_path0',
  'follow_path1',
  'follow_path2',
  'follow_path3',
  'follow_path4',
  'follow_path5'
]

def simulation(world_name, headless=False, paused=False, extra_gz_args=''):
    gz_args = ['-v 4']
    if not paused:
        gz_args.append('-r')

    if headless:
        gz_args.append('-s')

    gz_args.append(extra_gz_args)

    gz_args.append(f'{world_name}.sdf')

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch'),
            '/gz_sim.launch.py']),
        launch_arguments={'gz_args': ' '.join(gz_args)}.items())

    # Register handler for shutting down ros launch when ign gazebo process exits
    # monitor_sim.py will run until it can not find the ign gazebo process.
    # Once monitor_sim.py exits, a process exit event is triggered which causes the
    # handler to emit a Shutdown event
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

    return [gz_sim, monitor_sim_proc, sim_exit_event_handler]


def competition_bridges(world_name, competition_mode=False):
    bridges = [
        vrx_gz.bridges.clock(),
        vrx_gz.bridges.task_info(),
    ]

    if not competition_mode:
        bridges.extend([
            vrx_gz.bridges.usv_wind_speed(),
            vrx_gz.bridges.usv_wind_direction()
        ])

    task_bridges = []
    if world_name in PERCEPTION_WORLDS:
        task_bridges = [
            vrx_gz.bridges.perception_reports(),
        ]
    elif world_name in STATIONKEEPING_WORLDS:
        task_bridges = [
            vrx_gz.bridges.stationkeeping_goal(),
        ]
        if not competition_mode:
            task_bridges.extend([
                vrx_gz.bridges.stationkeeping_mean_pose_error(),
                vrx_gz.bridges.stationkeeping_pose_error(),
            ])
    elif world_name in WAYFINDING_WORLDS:
        task_bridges = [
            vrx_gz.bridges.wayfinding_waypoints(),
        ]
        if not competition_mode:
            task_bridges.extend([
                vrx_gz.bridges.wayfinding_mean_error(),
                vrx_gz.bridges.wayfinding_min_errors(),
            ])
    elif world_name in GYMKHANA_WORLDS:
        task_bridges = [
            vrx_gz.bridges.gymkhana_blackbox_goal(),
        ]
        if not competition_mode:
            task_bridges.extend([
                vrx_gz.bridges.gymkhana_blackbox_mean_pose_error(),
                vrx_gz.bridges.gymkhana_blackbox_pose_error(),
            ])
    elif world_name in WILDLIFE_WORLDS:
        task_bridges = [
            vrx_gz.bridges.animal_pose('/vrx/wildlife/animal0/pose'),
            vrx_gz.bridges.animal_pose('/vrx/wildlife/animal1/pose'),
            vrx_gz.bridges.animal_pose('/vrx/wildlife/animal2/pose'),
        ]
    elif world_name in SCAN_DOCK_DELIVER_WORLDS:
        task_bridges = [
            vrx_gz.bridges.color_sequence_reports(),
        ]
    bridges.extend(task_bridges)

    nodes = []
    nodes.append(Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        arguments=[bridge.argument() for bridge in bridges],
        remappings=[bridge.remapping() for bridge in bridges],
    ))
    return nodes


def spawn(sim_mode, world_name, models, robot=None):
    if type(models) != list:
        models = [models]
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'),

    launch_processes = []
    for model in models:
        if robot and model.model_name != robot:
            continue

        # Script to insert model in running simulation
        if sim_mode == 'full' or sim_mode == 'sim':
            gz_spawn_entity = Node(
                package='ros_gz_sim',
                executable='create',
                output='screen',
                arguments=model.spawn_args()
            )
            launch_processes.append(gz_spawn_entity)

        if sim_mode == 'full' or sim_mode == 'bridge':
            bridges, nodes, custom_launches = model.bridges(world_name)

            payload = model.payload_bridges(world_name)
            payload_bridges = payload[0]
            payload_nodes = payload[1]
            payload_launches = payload[2]

            bridges.extend(payload_bridges)
            nodes.extend(payload_nodes)

            nodes.append(Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                output='screen',
                arguments=[bridge.argument() for bridge in bridges],
                remappings=[bridge.remapping() for bridge in bridges],
            ))

            # tf broadcaster (sensors)
            nodes.append(Node(
                package='vrx_ros',
                executable='pose_tf_broadcaster',
                output='screen',
            ))

            # robot_state_publisher (tf for wamv)
            model_dir = os.path.join(get_package_share_directory('vrx_gazebo'), 'models/wamv/tmp')
            urdf_file = os.path.join(model_dir, 'model.urdf')
            with open(urdf_file, 'r') as infp:
                robot_desc = infp.read()
            params = {'use_sim_time': use_sim_time, 'frame_prefix': 'wamv/', 'robot_description': robot_desc}
            nodes.append(Node(package='robot_state_publisher',
                                  executable='robot_state_publisher',
                                  output='both',
                                  parameters=[params],
                                  remappings=[('/joint_states', '/wamv/joint_states')]))

            group_action = GroupAction([
                PushRosNamespace(model.model_name),
                *nodes
            ])

            if sim_mode == 'full':
                handler = RegisterEventHandler(
                    event_handler=OnProcessExit(
                        target_action=gz_spawn_entity,
                        on_exit=[group_action],
                    )
                )
                launch_processes.append(handler)
            elif sim_mode == 'bridge':
                launch_processes.append(group_action)

            launch_processes.extend(payload_launches)
            launch_processes.extend(custom_launches)

    return launch_processes
