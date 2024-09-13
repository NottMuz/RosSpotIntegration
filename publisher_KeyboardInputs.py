#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String
import sys, select, termios, tty


settings = termios.tcgetattr(sys.stdin)

msg = """
Commands: 
   [b]  : Begin controlling Spot (auto. turn on power, 
   toggle Estop, and take lease)
   
   [s]  : Take Lease Set Estop Endpoint
 [SPACE]: Toggle Estop
   [p]  : Toggle Power
  [TAB] : Shut Down & Quit

[f]: Stand
[r]: Self-right
[v]: Sit
[w]: Stop Moving

CTRL-C to quit
"""

def getKey():
    #enter is not required, takes the raw inputs from the keyboards
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def main(args=None):
		
	if args is None:
		args = sys.argv

	rclpy.init()

	node = rclpy.create_node('KeyboardInputsForSpotControl')

	pub = node.create_publisher(String, 'spot_keypress', qos_profile = QoSProfile(depth=10))
	
	try:
		print(msg)
		while(1):
			input_command = String()
			key = getKey()

			if (key == '\x03'):
					break
			input_command.data = key
			pub.publish(input_command)

	except:
		print('Keys were not able to be read')
		
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

 
if __name__ == '__main__':
    main()
