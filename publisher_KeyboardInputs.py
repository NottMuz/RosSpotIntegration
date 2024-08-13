#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class KeyPublisher(Node):
    def __init__(self):
        super().__init__('key_publisher')
        self.publisher_ = self.create_publisher(String, 'spot_keypress', 10)
        self.get_logger().info('Key Publisher Node has been started.')

        # Simulating keypresses
        self.timer = self.create_timer(1.0, self.publish_key)

    def publish_key(self):
        msg = String()
        msg.data = 'a'  # Simulating 'a' key press for stop command
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = KeyPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
