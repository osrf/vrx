#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from rcl_interfaces.srv import SetParameters
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
import math
from utils import *

class AnglesInRPY(Node):

    def __init__(self):
        super().__init__('angles_in_rpy_node')
        
        self.state_topic   = '/odometry/filtered'
        self.heading_topic = '/heading_rpy'

        self.state_subscription = self.create_subscription(Odometry, self.state_topic, self.state_callback, 10)
        self.heading_publisher = self.create_publisher(Float64, self.heading_topic, 10)


    def state_callback(self, msg : Odometry):
        q = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w)
        
        roll, pitch, yaw = euler_from_quaternion(q)

        headingmsg      = Float64()
        headingmsg.data = float(yaw)

        self.heading_publisher.publish(headingmsg)

        # print('\t', roll* 180.0/math.pi, ' ', pitch*180.0/math.pi, ' ', yaw*180.0/math.pi)
        print('\t', roll, ' ', pitch, ' ', yaw)

        # self.get_logger().info('Current yaw : "%s"' % self.current_yaw)


def main(args=None):
    rclpy.init(args=args)
    angles_in_rpy_node = AnglesInRPY()
    rclpy.spin(angles_in_rpy_node)
    angles_in_rpy_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
