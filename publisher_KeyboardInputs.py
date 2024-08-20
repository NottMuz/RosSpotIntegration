#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_default
from std_msgs.msg import String
# from geometry_msgs.msg import Twist
import sys, select, termios, tty


settings = termios.tcgetattr(sys.stdin)

msg = """

Commands: 
[TAB]: Shut down and quit
[SPACE]: Estop, [P]: Power
[ESC]: Stop, [s] Start Connection With Spot

[f]: Stand, [r]: Self-right
[v]: Sit


CTRL-C to quit
"""

moveBindings = {
		' ': 1, # toggle estop
		'\t': 2, # quit program
		'p': 3, # toggle power
		'l': 4, # toggle lease
		'r': 5, # self right
		'v': 6, # sit
		'f': 7, # stand
		's': 8, # start
		'O': 9,
		'I': 10,
	       }

class KeyPublisher(Node):
    def __init__(self):
        super().__init__('key_publisher')
        self.publisher_ = self.create_publisher(String, 'spot_keypress', 10)
        self.get_logger().info('Key Publisher Node has been started.')

        
        self.timer = self.create_timer(1.0, self.publish_key)

    def publish_key(self):
        msg = String()


        # CONNECT THIS TO getKey() 
        # msg.data = 'a'  # Simulating 'a' key press for stop command
                             
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published: {msg.data}')

    def getKey():
        #enter is not required, takes the raw inputs from the keyboards
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

def main(args=None):
    rclpy.init(args=args)
    node = KeyPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()
