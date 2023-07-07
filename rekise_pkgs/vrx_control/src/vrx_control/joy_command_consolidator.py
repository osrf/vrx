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

        # Declare parameters 
        self.fwd_vel_cmd_topic = '/cmd_fwd_velocity'
        self.cmd_vel_topic     = '/cmd_vel'
        
        self.cmd_subscription   = self.create_subscription(Float, self.fwd_vel_cmd_topic, self.cmd_fwd_vel_callback, 10)

        self.cmd_vel_publisher  = self.create_publisher(Twist, self.cmd_vel_topic, 10)        
        # self.debug_publisher      = self.create_publisher(PidDiagnose, self.pid_diagnose_topic, 10)

        self.get_logger().info('joy_command_consolidator initialized')


    def cmd_fwd_vel_callback(self, msg : Float) : 
        self.fwd_vel_cmd = msg.data

    def cmd_vel_publisher(mst : Twist) : 
        self.cmd_vel_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    joy_command_consolidator = JoyCommandConsolidator()
    rclpy.spin(joy_command_consolidator)
    joy_command_consolidator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()