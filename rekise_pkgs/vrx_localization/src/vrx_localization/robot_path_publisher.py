#!/usr/bin/env python3
# license removed for brevity

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path

class RobotPathPublisher(Node):
    path = Path()

    def __init__(self):
        super().__init__('robot_path_publisher')
        self.republisher  = self.create_publisher(Path, '/path', 10)
        self.subscription = self.create_subscription(
            Odometry,
            '/odometry/filtered',  # Replace with the name of the pose topic
            self.odom_callback,
            10
        )

        self.subscription  # Prevent unused variable warning
        self.get_logger().info('robot_path_publisher initialized')


    def odom_callback(self, msg : Odometry) : 
        
        self.path.header = msg.header
        
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose

        self.path.poses.append(pose)
        self.republisher.publish(self.path)


def main(args=None):
    rclpy.init(args=args)
    robot_path_publisher = RobotPathPublisher()
    rclpy.spin(robot_path_publisher)
    robot_path_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()