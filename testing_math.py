
import curses
import io
import logging
import math
import os
import signal
import sys
import threading
import time
from collections import OrderedDict

from PIL import Image, ImageEnhance

import bosdyn.api.basic_command_pb2 as basic_command_pb2
import bosdyn.api.power_pb2 as PowerServiceProto
import bosdyn.api.robot_command_pb2 as robot_command_pb2
import bosdyn.api.robot_state_pb2 as robot_state_proto
import bosdyn.api.spot.robot_command_pb2 as spot_command_pb2
import bosdyn.client.util
from bosdyn.api import geometry_pb2
from bosdyn.client import ResponseError, RpcError, create_standard_sdk
from bosdyn.client.async_tasks import AsyncGRPCTask, AsyncPeriodicQuery, AsyncTasks
from bosdyn.client.estop import EstopClient, EstopEndpoint, EstopKeepAlive
from bosdyn.client.frame_helpers import ODOM_FRAME_NAME
from bosdyn.client.image import ImageClient
from bosdyn.client.lease import Error as LeaseBaseError
from bosdyn.client.lease import LeaseClient, LeaseKeepAlive
from bosdyn.client.power import PowerClient
from bosdyn.client.robot_state import RobotStateClient
from bosdyn.client.time_sync import TimeSyncError
from bosdyn.util import duration_str, format_metric, secs_to_hms
from bosdyn.client.util import authenticate

#for trajectory planning
from bosdyn.client import math_helpers
from bosdyn.api.basic_command_pb2 import RobotCommandFeedbackStatus
from bosdyn.client.frame_helpers import (BODY_FRAME_NAME, ODOM_FRAME_NAME, VISION_FRAME_NAME,
                                         get_se2_a_tform_b)
from bosdyn.client.robot_command import (RobotCommandBuilder, RobotCommandClient,
                                         block_for_trajectory_cmd, blocking_stand)

# Define the circle parameters
radius = 0.5# [meters]
total_time = 600 # [seconds] Total time to complete the circle
steps = 100# Number of steps for the full circle

# Calculate the angular increment per step
angle_increment = 2 * math.pi / steps

# Calculate time per step
time_per_step = total_time / steps

for i in range(steps):
    # Calculate the incremental dx, dy, dyaw for each step
    total_angle = i * angle_increment
    dx = radius * math.cos(total_angle)
    dy = radius * math.sin(total_angle)
    print('dx, dy:', dx, dy)
    if i < steps - 1:
        next_angle = (i + 1) * angle_increment
        next_dx = radius * math.cos(next_angle)
        next_dy = radius * math.sin(next_angle)
        print('next dx, next dy:', next_dx, next_dy)
        print('Vab =' ,next_dx - dx, next_dy - dy )
        dyaw = math.atan2(next_dx - dx, next_dy - dy)
        print('dyaw :', dyaw)
        print('\n')
    else:
        dyaw = 0
    # Plan the small segment of the trajectory