#!/usr/bin/env python

import rclpy
from std_msgs.msg import String
import sys
import tty
import termios

def get_keypress():
    """Capture a single keypress from the terminal."""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

def keypress_publisher():
    rclpy.init_node('keypress_publisher')
    pub = rclpy.Publisher('keypress_topic', String, queue_size=10)
    rate = rclpy.Rate(10)  # 10 Hz

    rclpy.loginfo("Ready to capture keypresses. Press 'q' to quit.")

    while not rclpy.is_shutdown():
        key = get_keypress()
        rclpy.loginfo(f"Publishing keypress: {key}")
        pub.publish(key)
        
        if key == 'q':  # Quit on 'q' keypress
            rclpy.loginfo("Exiting keypress publisher.")
            break
        
        rate.sleep()

if __name__ == '__main__':
    try:
        keypress_publisher()
    except rclpy.ROSInterruptException:
        pass
