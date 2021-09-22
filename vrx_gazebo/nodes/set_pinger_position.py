#!/usr/bin/env python3
# Copyright 2018 Joanthan Wheare
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

import random
import rclpy

from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import ReliabilityPolicy
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Vector3
from usv_msgs.msg import RangeBearing
from visualization_msgs.msg import Marker

"""@package set_pinger_position.py
Automatically generate a position for a simulated USV pinger.  This
position is published both as a Vector3 to the plugin, and as a Marker that
can be visualised.  This visualisation can be used as a ground truth to
compare with the estimated pinger position.
Possible positions are stored as ROS parameters.  The node assumes that
positions will be three element vectors each stored in the nodes parameter
space in the format position_n starting with position_1.  If no positions
are available, the node will default to the origin.
"""


class PingerPosition(Node):
    """Class used to store the parameters and variables for the script.
    """
    def __init__(self):
        """Initialise and run the class."""
        super().__init__("set_pinger_position", allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)

        # Load the positions of the pingers.
        self.pingerPositions = list()
        i = 1

        while self.has_parameter('position_' + str(i)):
            pos = self.get_parameter('position_' + str(i)).get_parameter_value().double_array_value
            self.pingerPositions.append(pos)
            i = i + 1

        # Define a qos with latching properties
        qos = QoSProfile(
              history=HistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
              depth=1,
              durability=DurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
              reliability=ReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE
              )

        # If there are no matching positions, initialise to the origin.
        if i == 1:
            self.pingerPositions.append([0, 0, 0])
        self.pingerPub = self.create_publisher(Vector3, 
            "/wamv/sensors/pingers/pinger/set_pinger_position", qos_profile=qos)

        self.markerPub = self.create_publisher(Marker, 
            "/wamv/sensors/pingers/pinger/marker/ground_truth", qos_profile=qos)

        # Change position every 10 seconds.
        self.update_timer = self.create_timer(10, self.update)

    def update(self):
        # Choose a position for the pinger.
        position = random.choice(self.pingerPositions)

        # Create a vector message.
        msg = Vector3()
        msg.x = float (position[0])
        msg.y = float (position[1])
        msg.z = float (position[2])
        self.pingerPub.publish(msg)

        msg = Marker()
        msg.header.frame_id = "/map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.ns = ""
        msg.id = 0
        msg.type = Marker.SPHERE
        msg.action = Marker.ADD

        msg.pose.position.x = float(position[0])
        msg.pose.position.y = float(position[1])
        msg.pose.position.z = float(position[2])
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0
        msg.pose.orientation.w = 1.0

        msg.scale.x = 1.0
        msg.scale.y = 1.0
        msg.scale.z = 1.0

        msg.color.r = 1.0
        msg.color.g = 0.0
        msg.color.b = 0.0
        msg.color.a = 1.0

        self.markerPub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    pinger = PingerPosition()
    rclpy.spin(pinger)
    pinger.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
