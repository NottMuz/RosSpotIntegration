#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String
import sys, select, termios, tty


settings = termios.tcgetattr(sys.stdin)

msg = """

Commands: 
[TAB]: Shut down and quit
[SPACE]: Estop, [P]: Power
[ESC]: Stop, [s] Start Connection With Spot


[f]: Stand, [r]: Self-right
[v]: Sit , [w]: Stop


CTRL-C to quit
"""

moveBindings = {
		' ': 1, # toggle estop
		'\t': 2, # shut down and quit
		'p': 3, # toggle power
		'l': 4, # toggle lease
		'r': 5, # self right
		'v': 6, # sit
		'f': 7, # stand
		's': 8, # start
		'w': 9, # Stop moving
	       }

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
