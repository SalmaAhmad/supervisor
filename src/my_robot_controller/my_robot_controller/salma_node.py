#!/usr/bin/env python3
import rclpy
from rclpy.node import Node


class TalkerNode(Node):

    def __init__(self):
        super().__init__("salma_node") 
        self.create_timer(1.0, self.timer_callback)  # Timer triggers every 1 second

    def timer_callback(self):
        self.get_logger().info("hello salma")  # Message to log


def main(args=None):
    rclpy.init(args=args)  # Initialize ROS 2
    node = TalkerNode()  # Create an instance of the TalkerNode
    rclpy.spin(node)  # Keep the node running
    rclpy.shutdown()  # Shut down ROS 2


if __name__ == '__main__':
    main()
