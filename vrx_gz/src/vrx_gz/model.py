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

import sdformat13 as sdf

from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import PackageNotFoundError

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

import vrx_gz.bridges
import vrx_gz.payload_bridges
import pathlib
import re
import shutil
import yaml

UAVS = [
    'vrx_hexrotor',
    'vrx_quadrotor'
]

USVS = [
    'usv',
    'wam-v',
]

WAVEFIELD_SIZE = {'sydney_regatta': 1000,}


class Model:

    def __init__(self, model_name, model_type, position):
        self.model_name = model_name
        self.model_type = model_type
        self.position = position
        self.battery_capacity = 0
        self.wavefield_size = 0
        self.payload = {}
        self.urdf = ''

    def is_UAV(self):
        return self.model_type in UAVS

    def is_USV(self):
        return self.model_type in USVS

    def bridges(self, world_name):
        custom_launches = []
        nodes = []
        bridges = [
            # pose
            vrx_gz.bridges.pose(self.model_name),
            # pose static
            vrx_gz.bridges.pose_static(self.model_name),
            # joint states
            vrx_gz.bridges.joint_states(world_name, self.model_name),
            # comms tx
            # vrx_gz.bridges.comms_tx(self.model_name),
            # comms rx
            # vrx_gz.bridges.comms_rx(self.model_name),
        ]
        if self.is_UAV():
            bridges.extend([
                # Magnetometer
                vrx_gz.bridges.magnetometer(world_name, self.model_name),
                # Air Pressure
                vrx_gz.bridges.air_pressure(world_name, self.model_name),
            ])
            bridges.extend([
                # twist
                vrx_gz.bridges.cmd_vel(self.model_name)
            ])
        elif self.is_USV():
            pass

        return [bridges, nodes, custom_launches]

    def payload_bridges(self, world_name, payloads=None):
        # payloads on usv and uav
        if not payloads:
            payloads = self.payload
        bridges, nodes, payload_launches = self.payload_bridges_impl(world_name, payloads)

        return [bridges, nodes, payload_launches]

    def payload_bridges_impl(self, world_name, payloads):
        bridges = []
        nodes = []
        payload_launches = []
        for sensor_name, value in payloads.items():
            link_name = value[0]
            sensor_type = value[1]

            bridges.extend(
                vrx_gz.payload_bridges.payload_bridges(
                    world_name, self.model_name, link_name, sensor_name, sensor_type))

            if sensor_type == sdf.Sensortype.CAMERA:
                ros_sensor_prefix = f'sensors/cameras/{sensor_name}'
                nodes.append(Node(
                    package='vrx_ros',
                    executable='optical_frame_publisher',
                    arguments=['1'],
                    remappings=[('input/image', f'{ros_sensor_prefix}/image_raw'),
                                ('output/image', f'{ros_sensor_prefix}/optical/image_raw'),
                                ('input/camera_info', f'{ros_sensor_prefix}/camera_info'),
                                ('output/camera_info',
                                 f'{ros_sensor_prefix}/optical/camera_info')]))
            elif sensor_type == sdf.Sensortype.RGBD_CAMERA:
                ros_sensor_prefix = f'sensors/cameras/{sensor_name}'
                nodes.append(Node(
                    package='vrx_ros',
                    executable='optical_frame_publisher',
                    arguments=['1'],
                    remappings=[('input/image', f'{ros_sensor_prefix}/image_raw'),
                                ('output/image', f'{ros_sensor_prefix}/optical/image_raw'),
                                ('input/camera_info', f'{ros_sensor_prefix}/camera_info'),
                                ('output/camera_info',
                                 f'{ros_sensor_prefix}/optical/camera_info')]))
                nodes.append(Node(
                    package='vrx_ros',
                    executable='optical_frame_publisher',
                    arguments=['1'],
                    remappings=[('input/image', f'{ros_sensor_prefix}/depth'),
                                ('output/image', f'{ros_sensor_prefix}/optical/depth')]))

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

    def set_wavefield(self, world_name):
        if world_name not in WAVEFIELD_SIZE:
            print(f'Wavefield size not found for {world_name}')
        else:
            self.wavefield_size = WAVEFIELD_SIZE[world_name]


    def erb_cmd(self):
        # this function is currently not called.
        # xacro_cmd is used instead

        # Generate SDF by executing ERB and populating templates
        template_file = os.path.join(
            get_package_share_directory('vrx_gz'),
            'models', self.model_type, 'model.sdf.erb')

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

        if self.model_type in USVS:
            command.append(f'wavefieldSize={self.wavefield_size}')

        command.append(template_file)
        return command


    def xacro_cmd(self):
        # run xacro to generate urdf file
        xacro_command = ['xacro']
        xacro_command.append(self.urdf)
        xacro_command.append(f'namespace:={self.model_name}')
        xacro_command.append(f'locked:=true')
        xacro_command.append(f'vrx_sensors_enabled:=true')
        xacro_command.append(f'thruster_config:=H')
        xacro_process = subprocess.Popen(xacro_command,
                                         stdout=subprocess.PIPE,
                                         stderr=subprocess.PIPE)
        stdout = xacro_process.communicate()[0]
        urdf_str = codecs.getdecoder('unicode_escape')(stdout)[0]
        print(xacro_command)

        # run gz sdf print to generate sdf file
        model_dir = os.path.join(get_package_share_directory('vrx_gazebo'), 'models', self.model_name)
        model_tmp_dir = os.path.join(model_dir, 'tmp')
        model_output_file = os.path.join(model_tmp_dir, 'model.urdf')
        if not os.path.exists(model_tmp_dir):
            pathlib.Path(model_tmp_dir).mkdir(parents=True, exist_ok=True)
        with open(model_output_file, 'w') as f:
            f.write(urdf_str)
        command = ['gz', 'sdf', '-p']
        command.append(model_output_file)
        return command

    def generate(self):
        command = None
        if not self.urdf:
            self.urdf = os.path.join(get_package_share_directory('wamv_gazebo'),
                                     'urdf', 'wamv_gazebo.urdf.xacro')
        command = self.xacro_cmd()
        process = subprocess.Popen(command,
                                   stdout=subprocess.PIPE,
                                   stderr=subprocess.PIPE)

        # evaluate error output for the xacro process
        stderr = process.communicate()[1]
        err_output = codecs.getdecoder('unicode_escape')(stderr)[0]
        for line in err_output.splitlines():
            if line.find('undefined local') > 0:
                raise RuntimeError(line)

        stdout = process.communicate()[0]
        model_sdf = codecs.getdecoder('unicode_escape')(stdout)[0]

        # parse sdf for payloads if model is urdf
        if self.urdf != '':
            self.payload = self.payload_from_sdf(model_sdf)

        # for debugging generated sdf file
        # with open('/tmp/wamv.sdf', 'w') as f:
        #     f.write(model_sdf)
        # print(command)

        return command, model_sdf

    def name_from_plugin(self, plugin_sdf):
        result = re.search(r"/*<name>(.*)<\/name>", plugin_sdf)
        if (result):
            return result.group(1)

    def payload_from_sdf(self, model_sdf):
        payload = {}
        root = sdf.Root()
        root.load_sdf_string(model_sdf)
        model = root.model()
        for link_index in range(model.link_count()):
            link = model.link_by_index(link_index)
            for sensor_index in range(link.sensor_count()):
                sensor = link.sensor_by_index(sensor_index)
                payload[sensor.name()] = [link.name(), sensor.type()]
        plugins = model.plugins()
        for plugin in plugins:
            if plugin.name() == 'gz::sim::systems::Thruster':
                name = self.name_from_plugin(plugin.__str__())
                payload['thruster_thrust_' + name] = [link.name(), name]
            elif plugin.name() == 'gz::sim::systems::JointPositionController':
                name = self.name_from_plugin(plugin.__str__())
                payload['thruster_rotate_' + name] = [link.name(), name]
            else:
                payload[plugin.name()] = ['', plugin.filename()]
        return payload


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

    def set_urdf(self, urdf):
        self.urdf = urdf

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

        return model
