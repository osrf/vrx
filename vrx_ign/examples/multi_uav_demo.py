#!/usr/bin/env python3

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

import rclpy
import argparse
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist, Vector3


def parse_args():
    parser = argparse.ArgumentParser('Set UAV command velocity.')
    parser.add_argument('-x', dest='x', type=float, default=0.0,
                        help='Set the x velocity of the UAVs')
    parser.add_argument('-y', dest='y', type=float, default=0.0,
                        help='Set the y velocity of the UAVs')
    parser.add_argument('-z', dest='z', type=float, default=0.0,
                        help='Set the z velocity of the UAVs')
    parser.add_argument('-d', '--detach', dest='detach', action='store_true',
                        help='Turns off suction gripper')
    parser.set_defaults(detach=False)
    args = parser.parse_args()
    return args


def main():
    args = parse_args()

    rclpy.init()
    node = rclpy.create_node('multiuav_demo')

    # Create a publisher for the two suction grippers
    quadrotor_gripper1 = \
        node.create_publisher(Bool, '/quadrotor_1/gripper/suction_on', 10)
    quadrotor_gripper2 = \
        node.create_publisher(Bool, '/quadrotor_2/gripper/suction_on', 10)

    # Create a publisher for the two quadrotor's thrust
    quadrotor_cmd1 = node.create_publisher(Twist, '/quadrotor_1/cmd_vel', 10)
    quadrotor_cmd2 = node.create_publisher(Twist, '/quadrotor_2/cmd_vel', 10)

    # Enable the grippers
    quadrotor_gripper1.publish(Bool(data=not args.detach))
    quadrotor_gripper2.publish(Bool(data=not args.detach))

    # liftoff
    quadrotor_cmd1.publish(Twist(linear=Vector3(x=args.x, y=args.y, z=args.z)))
    quadrotor_cmd2.publish(Twist(linear=Vector3(x=args.x, y=args.y, z=args.z)))

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
