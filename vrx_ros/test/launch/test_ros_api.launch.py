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

import unittest

import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription

from launch.actions import IncludeLaunchDescription
from launch.actions import ExecuteProcess
from launch.actions import RegisterEventHandler
from launch.actions import TimerAction
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

import launch_testing

# launch empty_platform and spawn quadrotor and hexrotor UAVs
def generate_test_description():

    process_under_test = Node(
        package='vrx_ros',
        executable='test_ros_api',
        output='screen'
    )

    # launch empty_platform world
    ign_args = '--headless-rendering -v 4 -s -r empty_platform.sdf'
    arguments={'ign_args'  : ign_args}
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('vrx_ros'),
                'launch/competition_local.launch.py')
        ),
        launch_arguments=arguments.items())

    # ros launch does not bring down the ign gazebo process so manually kill it
    # \todo(anyone) figure out a proper way to terminate the ign gazebo process
    pid = 'ps aux | grep -v grep | grep \'ign gazebo ' + ign_args + '\' | awk \'{print $2}\''
    kill_gazebo = ExecuteProcess(
        cmd=['kill `' + pid +'`'],
        output='screen',
        shell=True
    )

    kill_proc_handler = RegisterEventHandler(
        OnShutdown(
            on_shutdown=[
               kill_gazebo
            ]
        )
    )

    return LaunchDescription([
        gazebo,
        process_under_test,
        kill_proc_handler,
        launch_testing.util.KeepAliveProc(),
        launch_testing.actions.ReadyToTest(),
    ]), locals()


class RosApiTest(unittest.TestCase):

    def test_termination(self, process_under_test, proc_info):
        proc_info.assertWaitForShutdown(process=process_under_test, timeout=300)

@launch_testing.post_shutdown_test()
class RosApiTestAfterShutdown(unittest.TestCase):

    def test_exit_code(self, process_under_test, proc_info):
        launch_testing.asserts.assertExitCodes(
            proc_info,
            [launch_testing.asserts.EXIT_OK],
            process_under_test
        )
