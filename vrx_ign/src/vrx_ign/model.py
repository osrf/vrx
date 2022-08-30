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

import codecs
import os
import subprocess

from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import PackageNotFoundError

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

import vrx_ign.bridges
import vrx_ign.payload_bridges
import pathlib
import shutil

import yaml

FIXED_WING_UAVS = [
    'vrx_fixed_wing',
]

UAVS = [
    'vrx_fixed_wing',
    'vrx_hexrotor',
    'vrx_quadrotor'
]

USVS = [
    'usv',
    'wam-v'
]

ARMS = [
    'vrx_oberon7_arm',
]

GRIPPERS = [
    'vrx_oberon7_gripper',
    'vrx_suction_gripper'
]

WAVEFIELD_SIZE = {'simple_demo': 1000, 'coast': 6000}


class Model:

    def __init__(self, model_name, model_type, position):
        self.model_name = model_name
        self.model_type = model_type
        self.position = position
        self.battery_capacity = 0
        self.wavefield_size = 0
        self.payload = {}
        self.arm_payload = {}
        self.arm = ''
        self.gripper = ''
        self.arm_slot = '0'

    def is_UAV(self):
        return self.model_type in UAVS

    def is_fixed_wing_UAV(self):
        return self.model_type in FIXED_WING_UAVS

    def is_USV(self):
        return self.model_type in USVS

    def has_valid_arm(self):
        return self.arm in ARMS

    def has_valid_gripper(self):
        return self.gripper in GRIPPERS

    def bridges(self, world_name):
        custom_launches = []
        nodes = []
        bridges = [
            # IMU
            vrx_ign.bridges.imu(world_name, self.model_name),
            # pose
            vrx_ign.bridges.pose(self.model_name),
            # pose static
            vrx_ign.bridges.pose_static(self.model_name),
            # comms tx
            vrx_ign.bridges.comms_tx(self.model_name),
            # comms rx
            vrx_ign.bridges.comms_rx(self.model_name),
        ]
        if self.is_UAV():
            bridges.extend([
                # Magnetometer
                vrx_ign.bridges.magnetometer(world_name, self.model_name),
                # Air Pressure
                vrx_ign.bridges.air_pressure(world_name, self.model_name),
            ])
            if not self.is_fixed_wing_UAV():
                bridges.extend([
                    # twist
                    vrx_ign.bridges.cmd_vel(self.model_name)
                ])
        elif self.is_USV():
            bridges.extend([
                # thrust cmd
                vrx_ign.bridges.thrust(self.model_name, 'left'),
                vrx_ign.bridges.thrust(self.model_name, 'right'),
                # thrust joint pos cmd
                vrx_ign.bridges.thrust_joint_pos(self.model_name, 'left'),
                vrx_ign.bridges.thrust_joint_pos(self.model_name, 'right'),
            ])
            nodes.append(Node(
                package='vrx_ros',
                executable='usv_bridge',
            ))

        if self.has_valid_arm():
            # arm joint states
            bridges.append(
                vrx_ign.bridges.arm_joint_states(world_name, self.model_name)
            )

            if self.arm == 'vrx_oberon7_arm':
                # arm joint pos cmd
                arm_joints = ['azimuth', 'shoulder', 'elbow', 'roll', 'pitch', 'wrist']
                for joint in arm_joints:
                    bridges.append(
                        vrx_ign.bridges.arm_joint_pos(self.model_name, joint)
                    )

                bridges.append(
                    vrx_ign.bridges.wrist_joint_force_torque(self.model_name),
                )
                # default to oberon7 gripper if not specified.
                if not self.gripper:
                    self.gripper = 'vrx_oberon7_gripper'
        elif self.is_custom_model(self.arm):
            custom_launch = self.custom_model_launch(world_name, self.model_name,
                                                     self.arm)
            if custom_launch is not None:
                custom_launches.append(custom_launch)

            # default to oberon7 gripper if not specified.
            if not self.gripper:
                self.gripper = 'vrx_oberon7_gripper'

        if self.has_valid_gripper():
            isAttachedToArm = self.is_USV()
            # gripper joint pos cmd
            if self.gripper == 'vrx_oberon7_gripper':
                # gripper_joint states
                bridges.append(
                    vrx_ign.bridges.gripper_joint_states(world_name, self.model_name,
                                                            isAttachedToArm)
                )
                gripper_joints = ['finger_left', 'finger_right']
                for joint in gripper_joints:
                    bridges.append(
                        vrx_ign.bridges.gripper_joint_pos(self.model_name, joint,
                                                             isAttachedToArm)
                    )
                    bridges.append(
                        vrx_ign.bridges.gripper_joint_force_torque(
                            self.model_name, joint, isAttachedToArm)
                    )
            elif self.gripper == 'vrx_suction_gripper':
                bridges.append(
                    vrx_ign.bridges.gripper_suction_control(self.model_name, isAttachedToArm)
                )
                bridges.extend([
                    vrx_ign.bridges.gripper_contact(self.model_name, isAttachedToArm, 'center'),
                    vrx_ign.bridges.gripper_contact(self.model_name, isAttachedToArm, 'left'),
                    vrx_ign.bridges.gripper_contact(self.model_name, isAttachedToArm, 'right'),
                    vrx_ign.bridges.gripper_contact(self.model_name, isAttachedToArm, 'top'),
                    vrx_ign.bridges.gripper_contact(self.model_name, isAttachedToArm, 'bottom')
                ])

        return [bridges, nodes, custom_launches]

    def payload_bridges(self, world_name, payloads=None):
        # payloads on usv and uav
        if not payloads:
            payloads = self.payload
        bridges, nodes, payload_launches = self.payload_bridges_impl(world_name, payloads)

        # payloads on arm
        b_out, n_out, l_out = self.payload_bridges_impl(world_name, self.arm_payload, True)
        bridges.extend(b_out)
        nodes.extend(n_out)
        payload_launches.extend(l_out)

        return [bridges, nodes, payload_launches]

    def payload_bridges_impl(self, world_name, payloads, is_arm=False):
        bridges = []
        nodes = []
        payload_launches = []
        for (idx, k) in enumerate(sorted(payloads.keys())):
            p = payloads[k]
            if not p['sensor'] or p['sensor'] == 'None' or p['sensor'] == '':
                continue

            # check if it is a custom payload
            if self.is_custom_model(p['sensor']):
                payload_launch = self.custom_payload_launch(world_name, self.model_name,
                                                            p['sensor'], idx)
                if payload_launch is not None:
                    payload_launches.append(payload_launch)

            # if not custom payload, add our own bridges and nodes
            else:
                model_prefix = ''
                ros_slot_prefix = f'slot{idx}'
                if is_arm:
                    model_prefix = 'arm'
                    ros_slot_prefix = 'arm/' + ros_slot_prefix
                bridges.extend(
                    vrx_ign.payload_bridges.payload_bridges(
                        world_name, self.model_name, p['sensor'], idx, model_prefix))

                if p['sensor'] in vrx_ign.payload_bridges.camera_models():
                    nodes.append(Node(
                        package='vrx_ros',
                        executable='optical_frame_publisher',
                        arguments=['1'],
                        remappings=[('input/image', f'{ros_slot_prefix}/image_raw'),
                                    ('output/image', f'{ros_slot_prefix}/optical/image_raw'),
                                    ('input/camera_info', f'{ros_slot_prefix}/camera_info'),
                                    ('output/camera_info',
                                     f'{ros_slot_prefix}/optical/camera_info')]))
                elif p['sensor'] in vrx_ign.payload_bridges.rgbd_models():
                    nodes.append(Node(
                        package='vrx_ros',
                        executable='optical_frame_publisher',
                        arguments=['1'],
                        remappings=[('input/image', f'{ros_slot_prefix}/image_raw'),
                                    ('output/image', f'{ros_slot_prefix}/optical/image_raw'),
                                    ('input/camera_info', f'{ros_slot_prefix}/camera_info'),
                                    ('output/camera_info',
                                     f'{ros_slot_prefix}/optical/camera_info')]))
                    nodes.append(Node(
                        package='vrx_ros',
                        executable='optical_frame_publisher',
                        arguments=['1'],
                        remappings=[('input/image', f'{ros_slot_prefix}/depth'),
                                    ('output/image', f'{ros_slot_prefix}/optical/depth')]))
        return [bridges, nodes, payload_launches]

    def is_custom_model(self, model):
        if not model:
            return False
        try:
            get_package_share_directory(model)
        except PackageNotFoundError:
            return False
        return True

    def custom_model_launch(self, world_name, model_name, model):
        custom_launch = None
        path = os.path.join(
            get_package_share_directory(model), 'launch')
        if os.path.exists(path):
            custom_launch = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([path, '/bridge.launch.py']),
                launch_arguments={'world_name': world_name,
                                  'model_name': model_name}.items())
        return custom_launch

    def custom_payload_launch(self, world_name, model_name, payload, idx):
        payload_launch = None
        path = os.path.join(
            get_package_share_directory(payload), 'launch')
        if os.path.exists(path):
            payload_launch = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([path, '/bridge.launch.py']),
                launch_arguments={'world_name': world_name,
                                  'model_name': model_name,
                                  'slot_idx': str(idx)}.items())
        return payload_launch

    def set_flight_time(self, flight_time):
        # UAV specific, sets flight time

        # calculate battery capacity from time
        # capacity (Ah) = flight time (in hours) * load (watts) / voltage
        # assume constant voltage for battery to keep things simple for now.
        self.battery_capacity = (float(flight_time) / 60) * 6.6 / 12.694

    def set_payload(self, payload):
        # UAV specific
        self.payload = payload

    def set_arm_payload(self, arm_payload):
        self.arm_payload = arm_payload

    def set_wavefield(self, world_name):
        if world_name not in WAVEFIELD_SIZE:
            print(f'Wavefield size not found for {world_name}')
        else:
            self.wavefield_size = WAVEFIELD_SIZE[world_name]

    def set_arm(self, arm):
        self.arm = arm

    def set_arm_slot(self, arm_slot):
        self.arm_slot = arm_slot

    def set_gripper(self, gripper):
        self.gripper = gripper

    def generate(self):
        # Generate SDF by executing ERB and populating templates
        template_file = os.path.join(
            get_package_share_directory('vrx_ign'),
            'models', self.model_type, 'model.sdf.erb')

        model_dir = os.path.join(get_package_share_directory('vrx_ign'), 'models')
        model_tmp_dir = os.path.join(model_dir, 'tmp')

        command = ['erb']
        command.append(f'name={self.model_name}')

        for (slot, payload) in self.payload.items():
            if payload['sensor'] and payload['sensor'] != 'None':
                command.append(f"{slot}={payload['sensor']}")
            if 'rpy' in payload:
                if type(payload['rpy']) is str:
                    r, p, y = payload['rpy'].split(' ')
                else:
                    r, p, y = payload['rpy']
                command.append(f'{slot}_pos={r} {p} {y}')

        if self.model_type in UAVS:
            if self.battery_capacity == 0:
                raise RuntimeError('Battery Capacity is zero, was flight_time set?')
            command.append(f'capacity={self.battery_capacity}')
            if self.has_valid_gripper():
                command.append(f'gripper={self.gripper}_{self.model_name}')

        if self.model_type in USVS or self.model_type == 'static_arm':
            command.append(f'wavefieldSize={self.wavefield_size}')

            # run erb for arm to attach the user specified gripper
            # and also for arm and gripper to generate unique topic names
            if self.has_valid_arm() or self.is_custom_model(self.arm):
                command.append(f'arm={self.arm}')
                command.append(f'arm_slot={self.arm_slot}')
                arm_package = 'vrx_ign'
                if self.is_custom_model(self.arm):
                    arm_package = self.arm
                arm_model_file = os.path.join(
                    get_package_share_directory(arm_package), 'models',
                    self.arm, 'model.sdf.erb')
                arm_model_output_file = os.path.join(
                    get_package_share_directory(arm_package), 'models',
                    self.arm, 'model.sdf')
                arm_command = ['erb']

                if self.gripper:
                    arm_command.append(f'gripper={self.gripper}_{self.model_name}')

                # arm payloads
                for (slot, payload) in self.arm_payload.items():
                    if payload['sensor'] and payload['sensor'] != 'None':
                        arm_command.append(f"arm_{slot}={payload['sensor']}")
                    if 'rpy' in payload:
                        if type(payload['rpy']) is str:
                            r, p, y = payload['rpy'].split(' ')
                        else:
                            r, p, y = payload['rpy']
                        arm_command.append(f'arm_{slot}_pos={r} {p} {y}')

                arm_command.append(f'topic_prefix={self.model_name}')
                arm_command.append(arm_model_file)
                process = subprocess.Popen(arm_command, stdout=subprocess.PIPE)
                stdout = process.communicate()[0]
                str_output = codecs.getdecoder('unicode_escape')(stdout)[0]
                with open(arm_model_output_file, 'w') as f:
                    f.write(str_output)
                # print(arm_command, str_output)

        if self.has_valid_gripper():
            gripper_model_file = os.path.join(model_dir, self.gripper, 'model.sdf.erb')
            gripper_model_output_file = os.path.join(model_tmp_dir,
                                                     self.gripper + "_" + self.model_name,
                                                     'model.sdf')
            gripper_command = ['erb']
            topic_prefix = f'{self.model_name}'
            if (self.is_USV()):
                topic_prefix += '/arm'
            gripper_command.append(f'topic_prefix={topic_prefix}')
            gripper_command.append(gripper_model_file)

            # create unique gripper model in mbzic_ign/models/tmp
            # and symlink original model contents to new dir
            output_dir = os.path.dirname(gripper_model_output_file)
            if not os.path.exists(model_tmp_dir):
                pathlib.Path(model_tmp_dir).mkdir(parents=True, exist_ok=True)
            if os.path.exists(output_dir):
                shutil.rmtree(output_dir)
            pathlib.Path(output_dir).mkdir(parents=True, exist_ok=True)
            meshes_dir = os.path.join(model_dir, self.gripper, 'meshes')
            if os.path.exists(meshes_dir):
                os.symlink(meshes_dir, os.path.join(output_dir, 'meshes'))
            model_config = os.path.join(model_dir, self.gripper, 'model.config')
            os.symlink(model_config, os.path.join(output_dir, 'model.config'))

            # Ru erb to generate new model.sdf file
            process = subprocess.Popen(gripper_command, stdout=subprocess.PIPE)
            stdout = process.communicate()[0]
            str_output = codecs.getdecoder('unicode_escape')(stdout)[0]
            with open(gripper_model_output_file, 'w') as f:
                f.write(str_output)
            # print(gripper_command, str_output)

        command.append(template_file)
        process = subprocess.Popen(command,
                                   stdout=subprocess.PIPE,
                                   stderr=subprocess.PIPE)

        # evaluate error output to see if there were undefined variables
        # for the ERB process
        stderr = process.communicate()[1]
        err_output = codecs.getdecoder('unicode_escape')(stderr)[0]
        for line in err_output.splitlines():
            if line.find('undefined local') > 0:
                raise RuntimeError(line)

        stdout = process.communicate()[0]
        model_sdf = codecs.getdecoder('unicode_escape')(stdout)[0]
        print(command)

        return command, model_sdf

    def spawn_args(self, model_sdf=None):
        if not model_sdf:
            [command, model_sdf] = self.generate()

        return ['-string', model_sdf,
                '-name', self.model_name,
                '-allow_renaming', 'false',
                '-x', str(self.position[0]),
                '-y', str(self.position[1]),
                '-z', str(self.position[2]),
                '-R', str(self.position[3]),
                '-P', str(self.position[4]),
                '-Y', str(self.position[5])]

    @classmethod
    def FromConfig(cls, stream):
        # Generate a Model instance (or multiple instances) from a stream
        # Stream can be either a file input or string
        config = yaml.safe_load(stream)

        if type(config) == list:
            return cls._FromConfigList(config)
        elif type(config) == dict:
            return cls._FromConfigDict(config)

    @classmethod
    def _FromConfigList(cls, entries):
        # Parse an array of configurations
        ret = []
        for entry in entries:
            ret.append(cls._FromConfigDict(entry))
        return ret

    @classmethod
    def _FromConfigDict(cls, config):
        # Parse a single configuration
        if 'model_name' not in config:
            raise RuntimeError('Cannot construct model without model_name in config')
        if 'model_type' not in config:
            raise RuntimeError('Cannot construct model without model_type in config')

        xyz = [0, 0, 0]
        rpy = [0, 0, 0]
        if 'position' not in config:
            print('Position not found in config, defaulting to (0, 0, 0), (0, 0, 0)')
        else:
            if 'xyz' in config['position']:
                xyz = config['position']['xyz']
            if 'rpy' in config['position']:
                rpy = config['position']['rpy']
        model = cls(config['model_name'], config['model_type'], [*xyz, *rpy])

        if 'flight_time' in config:
            model.set_flight_time(config['flight_time'])

        if 'payload' in config:
            model.set_payload(config['payload'])

        if 'arm_payload' in config:
            model.set_arm_payload(config['arm_payload'])

        if 'arm' in config:
            model.set_arm(config['arm'])

        if 'arm_slot' in config:
            model.set_arm_slot(config['arm_slot'])

        if 'gripper' in config:
            model.set_gripper(config['gripper'])

        return model
