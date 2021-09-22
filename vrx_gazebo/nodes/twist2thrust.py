#!/usr/bin/env python3
# license removed for brevity

import sys
import os
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

class Twist2DriveNode(Node):

    def __init__(self):
        super().__init__("twist2drive" + "_" + str(os.getpid()))

        # ROS Parameters
        # Scaling from Twist.linear.x to (left+right)
        self.linear_scaling = self.declare_parameter("linear_scaling", 0.2).get_parameter_value().double_value

        # Scaling from Twist.angular.z to (right-left)
        self.angular_scaling = self.declare_parameter("angular_scaling", 0.05).get_parameter_value().double_value

        # Keyboard
        self.keyboard = self.declare_parameter("keyboard", False).get_parameter_value().bool_value

        self.get_logger().debug("Linear scaling=%f, Angular scaling=%f"%(self.linear_scaling, self.angular_scaling))

        # key = '--keyboard' in sys.argv
        # node=Node(linear_scaling,angular_scaling,keyboard=key)

        # Publisher
        self.left_pub = self.create_publisher(Float32, "left_cmd", qos_profile=10)
        self.right_pub = self.create_publisher(Float32, "right_cmd", qos_profile=10)
        self.left_msg = Float32()
        self.right_msg = Float32()

        # Subscriber
        self.vel_sub = self.create_subscription(Twist, "cmd_vel", self.callback, qos_profile=10)


    def callback(self,data):
        self.get_logger().debug("RX: Twist")
        self.get_logger().debug("\tlinear:")

        self.get_logger().debug("\t\tx:%f,y:%f,z:%f"%(data.linear.x,
                                            data.linear.y,
                                            data.linear.z))
        self.get_logger().debug("\tangular:")
        self.get_logger().debug("\t\tx:%f,y:%f,z:%f"%(data.angular.x,
                                            data.angular.y,
                                            data.angular.z))
        # scaling factors
        linfac = self.linear_scaling
        angfac = self.angular_scaling

        if self.keyboard:
            self.left_msg.data = data.linear.x
            self.right_msg.data = data.linear.x
            self.left_msg.data += -1*data.angular.z
            self.right_msg.data += data.angular.z
        else:
            self.left_msg.data = data.linear.x
            self.right_msg.data = data.angular.z

        self.get_logger().debug("TX ")
        self.get_logger().debug("\tleft:%f, right:%f"%(self.left_msg.data,
                                              self.right_msg.data))
        self.left_pub.publish(self.left_msg)
        self.right_pub.publish(self.right_msg)


def main(args=None):
    rclpy.init(args=args)
    node = Twist2DriveNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
