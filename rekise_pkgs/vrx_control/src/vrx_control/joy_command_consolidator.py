#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import tf2_ros
from tf2_msgs.msg import TFMessage
import math
from std_msgs.msg import Float64

# from vrx_ros.msg import PidDiagnose
# from vrx_ros.cfg import TwistDynamicConfig
# from dynamic_reconfigure.server import Server

#
import pypid

class JoyCommandConsolidator(Node):
    def __init__(self):
        super().__init__('joy_command_consolidator')

        # Declare topics 
        self.fwd_vel_cmd_topic  = '/cmd_fwd_velocity'
        self.yaw_rate_cmd_topic = '/cmd_yaw_rate'
        self.cmd_vel_topic      = '/cmd_vel'
        
        # Subscribers
        self.fwd_vel_cmd_subsc   = self.create_subscription(Float64, self.fwd_vel_cmd_topic, self.cmd_fwd_vel_callback, 10)
        self.yaw_rate_cmd_subsc  = self.create_subscription(Float64, self.yaw_rate_cmd_topic, self.cmd_yaw_rate_callback, 10)

        # Publishers
        self.timer = self.create_timer(0.1, self.publish_twist)
        self.cmd_twist_publisher = self.create_publisher(Twist, self.cmd_vel_topic, 10)

        # Other data variables
        self.twist_msg = Twist()    
        
        self.get_logger().info('joy_command_consolidator initialized')


    def cmd_fwd_vel_callback(self, msg : Float64) : 
        self.twist_msg.linear.x = msg.data
    
    def cmd_yaw_rate_callback(self, msg : Float64) : 
        self.twist_msg.angular.z = msg.data


    def publish_twist(self) : 
        # self.get_logger().info('Sending command velocity : "%s"' % msg)
        self.cmd_twist_publisher.publish(self.twist_msg)


def main(args=None):
    rclpy.init(args=args)
    joy_command_consolidator = JoyCommandConsolidator()
    rclpy.spin(joy_command_consolidator)
    joy_command_consolidator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()