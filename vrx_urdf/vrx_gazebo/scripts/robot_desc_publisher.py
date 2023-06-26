#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from std_msgs.msg import String


class DescriptionPublisher(Node):

    def __init__(self):
        super().__init__('description_publisher')
        latching_qos = QoSProfile(depth=1,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL)
        self.publisher_ = self.create_publisher(String, '/robot_description', latching_qos)
        self.declare_parameter('robot_description', '')
        robot_desc = self.get_parameter('robot_description').get_parameter_value().string_value
        msg = String()
        msg.data = robot_desc
        self.publisher_.publish(msg)



def main(args=None):
    rclpy.init(args=args)

    description_publisher = DescriptionPublisher()

    try:
        rclpy.spin(description_publisher)
    except KeyboardInterrupt:
        description_publisher.get_logger().info("Ctrl-C detected, shutting down")
    finally:
        description_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()