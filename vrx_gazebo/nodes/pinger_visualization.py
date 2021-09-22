#!/usr/bin/env python3

# Copyright 2018 Jonathan Wheare (jonathan.wheare@flinders.edu.au)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
# this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its contributors
# may be used to endorse or promote products derived from this software without
# specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# BSD 3-term license.  Source: https://opensource.org/licenses/BSD-3-Clause
# Used under creative-commons attribution license.
#

import math
import rclpy

from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import ReliabilityPolicy
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Point
from usv_msgs.msg import RangeBearing
from visualization_msgs.msg import Marker

"""@package pinger_visualisation.py
This node takes a RangeBearing message and produces a corresponding Marker
message suitable for use with rviz.
"""


class PingerVisualisation(Node):
    """Class used to store the parameters and variables for the script.
    """
    def __init__(self):
        """Initialise and run the class."""
        super().__init__("pinger_visualisation")

        # Define a qos with latching properties
        qos = QoSProfile(
              history=HistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
              depth=1,
              durability=DurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
              reliability=ReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE
              )

        # Startup a publisher of the marker messages
        self.markerPub = self.create_publisher(Marker, 
            "/wamv/sensors/pingers/pinger/marker/signal", qos_profile=qos)

        # Start subscriber
        self.pingerSub = self.create_subscription(RangeBearing, 
            "/wamv/sensors/pingers/pinger/range_bearing", self.pingerCallback, qos_profile=10)


    # Callback to handle an incoming range bearing message
    def pingerCallback(self, msg):
        """Callback to handle receipt of a RangeBearing message."""

        # Create a Marker message
        visMsg = Marker()

        # The frame ID and timestamp should be identical to the received pinger
        # message
        visMsg.header.frame_id = msg.header.frame_id
        visMsg.header.stamp = msg.header.stamp
        visMsg.ns = ""
        visMsg.id = 0

        # Visualisation will be an arrow
        visMsg.type = Marker.ARROW
        visMsg.action = Marker.ADD

        # Position will be specified by start and end points.
        visMsg.pose.position.x = 0.0
        visMsg.pose.position.y = 0.0
        visMsg.pose.position.z = 0.0
        visMsg.pose.orientation.x = 0.0
        visMsg.pose.orientation.y = 0.0
        visMsg.pose.orientation.z = 0.0
        visMsg.pose.orientation.w = 1.0

        visMsg.scale.x = 0.1
        visMsg.scale.y = 0.5
        visMsg.scale.z = 0.5

        # Make it blue
        visMsg.color.r = 0.0
        visMsg.color.g = 0.0
        visMsg.color.b = 1.0
        visMsg.color.a = 1.0

        # When using start and end points, we need to include two more messages
        # of type geometry_msgs/Point. These are appended to the list of
        # points. Origin is a 0,0. Since the visualisation is in the sensor
        # frame, arrow should start at the sensor.
        startPoint = Point()
        startPoint.x = 0.0
        startPoint.y = 0.0
        startPoint.z = 0.0
        visMsg.points.append(startPoint)

        # Finish at the estimated pinger position.
        endPoint = Point()
        endPoint.x = msg.range*math.cos(msg.elevation)*math.cos(msg.bearing)
        endPoint.y = msg.range*math.cos(msg.elevation)*math.sin(msg.bearing)
        endPoint.z = msg.range*math.sin(msg.elevation)
        visMsg.points.append(endPoint)

        # Publish the message.
        self.markerPub.publish(visMsg)


def main(args=None):
    rclpy.init(args=args)
    pinger = PingerVisualisation()
    rclpy.spin(pinger)
    pinger.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
