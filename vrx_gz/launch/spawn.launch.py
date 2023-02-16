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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration

import vrx_gz.bridges
import vrx_gz.launch

from vrx_gz.model import Model


def parse_from_cli(context, world_name):
    robot_name = LaunchConfiguration('name').perform(context)
    model_type = LaunchConfiguration('model').perform(context)

    x_pos = LaunchConfiguration('x').perform(context)
    y_pos = LaunchConfiguration('y').perform(context)
    z_pos = LaunchConfiguration('z').perform(context)
    r_rot = LaunchConfiguration('R').perform(context)
    p_rot = LaunchConfiguration('P').perform(context)
    y_rot = LaunchConfiguration('Y').perform(context)
    position = [x_pos, y_pos, z_pos, r_rot, p_rot, y_rot]

    robot_urdf = LaunchConfiguration('urdf').perform(context)

    model = Model(robot_name, model_type, position)

    if robot_urdf and robot_urdf != '':
        model.set_urdf(robot_urdf)

    slot0_payload = LaunchConfiguration('slot0').perform(context)
    slot1_payload = LaunchConfiguration('slot1').perform(context)
    slot2_payload = LaunchConfiguration('slot2').perform(context)
    slot3_payload = LaunchConfiguration('slot3').perform(context)
    slot4_payload = LaunchConfiguration('slot4').perform(context)
    slot5_payload = LaunchConfiguration('slot5').perform(context)
    slot6_payload = LaunchConfiguration('slot6').perform(context)
    slot7_payload = LaunchConfiguration('slot7').perform(context)

    slot0_rpy = LaunchConfiguration('slot0_rpy').perform(context)
    slot1_rpy = LaunchConfiguration('slot1_rpy').perform(context)
    slot2_rpy = LaunchConfiguration('slot2_rpy').perform(context)
    slot3_rpy = LaunchConfiguration('slot3_rpy').perform(context)
    slot4_rpy = LaunchConfiguration('slot4_rpy').perform(context)
    slot5_rpy = LaunchConfiguration('slot5_rpy').perform(context)
    slot6_rpy = LaunchConfiguration('slot6_rpy').perform(context)
    slot7_rpy = LaunchConfiguration('slot7_rpy').perform(context)

    payloads = {
        'slot0': {'sensor': slot0_payload, 'rpy': slot0_rpy},
        'slot1': {'sensor': slot1_payload, 'rpy': slot1_rpy},
        'slot2': {'sensor': slot2_payload, 'rpy': slot2_rpy},
        'slot3': {'sensor': slot3_payload, 'rpy': slot3_rpy},
        'slot4': {'sensor': slot4_payload, 'rpy': slot4_rpy},
        'slot5': {'sensor': slot5_payload, 'rpy': slot5_rpy},
        'slot6': {'sensor': slot6_payload, 'rpy': slot6_rpy},
        'slot7': {'sensor': slot7_payload, 'rpy': slot7_rpy},
    }

    if model.is_UAV():
        # take flight time in minutes
        flight_time = LaunchConfiguration('flightTime').perform(context)

        model.set_flight_time(flight_time)
    elif model.is_USV():

        model.set_wavefield(world_name)

    model.set_payload(payloads)
    return model


def launch(context, *args, **kwargs):
    world_name = LaunchConfiguration('world').perform(context)
    sim_mode = LaunchConfiguration('sim_mode').perform(context)
    bridge_competition_topics = LaunchConfiguration(
        'bridge_competition_topics').perform(context).lower() == 'true'

    model = parse_from_cli(context, world_name)

    launch_processes = []

    launch_processes.extend(vrx_gz.launch.spawn(sim_mode, world_name, model))
    if (sim_mode == 'bridge' or sim_mode == 'full') and bridge_competition_topics:
        launch_processes.extend(vrx_gz.launch.competition_bridges(world_name))

    return launch_processes


def generate_launch_description():
    return LaunchDescription([
        # Launch Arguments
        DeclareLaunchArgument(
            'world',
            default_value='simple_demo',
            description='Name of world'),
        DeclareLaunchArgument(
            'sim_mode',
            default_value='full',
            description='Simulation mode: "full", "sim", "bridge".'
                        'full: spawns robot and launch ros_gz bridges, '
                        'sim: spawns robot only, '
                        'bridge: launch ros_gz bridges only.'),
        DeclareLaunchArgument(
            'bridge_competition_topics',
            default_value='True',
            description='True to bridge competition topics, False to disable bridge.'),

        DeclareLaunchArgument(
            'name',
            default_value='',
            description='Name of robot to spawn'),
        DeclareLaunchArgument(
            'model',
            default_value='',
            description='SDF model to spawn'),
        DeclareLaunchArgument(
            'x',
            default_value='0',
            description='X position to spawn'),
        DeclareLaunchArgument(
            'y',
            default_value='0',
            description='y position to spawn'),
        DeclareLaunchArgument(
            'z',
            default_value='0',
            description='z position to spawn'),
        DeclareLaunchArgument(
            'R',
            default_value='0',
            description='R rotation to spawn'),
        DeclareLaunchArgument(
            'P',
            default_value='0',
            description='P rotation to spawn'),
        DeclareLaunchArgument(
            'Y',
            default_value='0',
            description='Y rotation to spawn'),
        DeclareLaunchArgument(
            'flightTime',
            default_value='10',
            description='Battery flight time in minutes (only for UAVs)'),
        DeclareLaunchArgument(
            'slot0',
            default_value='',
            description='Payload mounted to slot 0'),
        DeclareLaunchArgument(
            'slot0_rpy',
            default_value='0 0 0',
            description='Roll, Pitch, Yaw in degrees of payload mount'),
        DeclareLaunchArgument(
            'slot1',
            default_value='',
            description='Payload mounted to slot 1'),
        DeclareLaunchArgument(
            'slot1_rpy',
            default_value='0 0 0',
            description='Roll, Pitch, Yaw in degrees of payload mount'),
        DeclareLaunchArgument(
            'slot2',
            default_value='',
            description='Payload mounted to slot 2'),
        DeclareLaunchArgument(
            'slot2_rpy',
            default_value='0 0 0',
            description='Roll, Pitch, Yaw in degrees of payload mount'),
        DeclareLaunchArgument(
            'slot3',
            default_value='',
            description='Payload mounted to slot 3'),
        DeclareLaunchArgument(
            'slot3_rpy',
            default_value='0 0 0',
            description='Roll, Pitch, Yaw in degrees of payload mount'),
        DeclareLaunchArgument(
            'slot4',
            default_value='',
            description='Payload mounted to slot 4'),
        DeclareLaunchArgument(
            'slot4_rpy',
            default_value='0 0 0',
            description='Roll, Pitch, Yaw in degrees of payload mount'),
        DeclareLaunchArgument(
            'slot5',
            default_value='',
            description='Payload mounted to slot 5'),
        DeclareLaunchArgument(
            'slot5_rpy',
            default_value='0 0 0',
            description='Roll, Pitch, Yaw in degrees of payload mount'),
        DeclareLaunchArgument(
            'slot6',
            default_value='',
            description='Payload mounted to slot 6'),
        DeclareLaunchArgument(
            'slot6_rpy',
            default_value='0 0 0',
            description='Roll, Pitch, Yaw in degrees of payload mount'),
        DeclareLaunchArgument(
            'slot7',
            default_value='',
            description='Payload mounted to slot 7'),
        DeclareLaunchArgument(
            'slot7_rpy',
            default_value='0 0 0',
            description='Roll, Pitch, Yaw in degrees of payload mount'),
        DeclareLaunchArgument(
            'urdf',
            default_value='',
            description='URDF file of the wam-v model. '),
        # launch setup
        OpaqueFunction(function=launch)
    ])
