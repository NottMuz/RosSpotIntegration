
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
total_time = 400 # [seconds] Total time to complete the circle
steps = 16# Number of steps for the full circle

# Calculate the angular increment per step
angle_increment = 2 * math.pi / steps

# Calculate time per step
time_per_step = total_time / steps

counter = 0

for i in range(steps):


    #first setpoint
    if  i == 0 :

        print(i)
        total_angle = 0
        body_frame_dx = 0
        body_frame_dy = 0
        dyaw = 0

    #remaining setpoints
    elif i > 0: 

        ''' -------------------------------------------------------   QUADRANT A   ----------------------------------------------------------------'''
        if i <= steps/4 and i < steps - 1 :

            # second setpoint
            if i == 1: 

                total_angle = i * angle_increment
                wanted_x = radius * math.cos(total_angle)
                wanted_y = radius * math.sin(total_angle)
                print('Wanted X & Y Pos:', wanted_x, wanted_y )
                

                #Current Position
                prev_angle = 0
                current_x = radius
                current_y = 0

                body_frame_dx = wanted_y
                body_frame_dy = ((-1) *(wanted_x - current_x))
                
                #opp/adj
                dyaw = math.atan2(body_frame_dy, body_frame_dy)


            #remaining setpoints
            else:
                total_angle = i * angle_increment
                wanted_x = radius * math.cos(total_angle)
                wanted_y = radius * math.sin(total_angle)
                print('Wanted X & Y Pos:', wanted_x, wanted_y )
               

                #Current Position
                current_angle = (i - 1) * angle_increment
                current_x = radius * math.cos(current_angle)
                current_y = radius * math.sin(current_angle)
                print('Current X, Y:       ',current_x, current_y)

                #Previous Position
                prev_angle = (i - 2) * angle_increment
                prev_x = radius * math.cos(prev_angle)
                prev_y = radius * math.sin(prev_angle)
                print('Prev dX, dY:        ',prev_x, prev_y)


                #vector/hypotnuese from current point --> desired point
                c = math.sqrt( pow( ((-1) *(wanted_x - current_x)),2) + pow((wanted_y - current_y),2))
                
                #dyaw = theta(next_total) - theta(old_increment)
                dyaw =  math.atan2( ((-1) *(wanted_x - current_x)), wanted_y - current_y) - math.atan2((-1*(current_x - prev_x)), current_y - prev_y)

                # Plan the small segment of the trajectory
                body_frame_dx = c * math.cos(dyaw)
                body_frame_dy = c * math.sin(dyaw)

        ''' -------------------------------------------------------   QUADRANT S  ----------------------------------------------------------------'''
        if i > round(steps/4) and i <= round(steps/2) :


            # second setpoint
            if i == round(steps/4) + 1: 

                j = 1
                total_angle = j * angle_increment
                wanted_x = radius * math.cos(total_angle)
                wanted_y = radius * math.sin(total_angle)
                print('Wanted X & Y Pos:', wanted_x, wanted_y )

                #Current Position
                prev_angle = 0
                current_x = radius
                current_y = 0

                body_frame_dx = wanted_y
                body_frame_dy = (-1) *(wanted_x - current_x)
                
                #opp/adj
                dyaw = math.atan2(body_frame_dy, body_frame_dy)

                j += 1


            #remaining setpoints
            else:

                total_angle = j * angle_increment
                wanted_x = radius * math.cos(total_angle)
                wanted_y = radius * math.sin(total_angle)
                print('Wanted X & Y Pos:', wanted_x, wanted_y )

                #Current Position   
                current_angle = (j - 1) * angle_increment
                current_x = radius * math.cos(current_angle)
                current_y = radius * math.sin(current_angle)
                print('Current X, Y:       ',current_x, current_y)

                #Previous Position
                prev_angle = (j - 2) * angle_increment
                prev_x = radius * math.cos(prev_angle)
                prev_y = radius * math.sin(prev_angle)
                print('Prev dX, dY:        ',prev_x, prev_y)


                #vector/hypotnuese from current point --> desired point
                c = math.sqrt( pow( ((-1) *(wanted_x - current_x)),2) + pow((wanted_y - current_y),2))
                
                #dyaw = theta(next_total) - theta(old_increment)
                dyaw =  math.atan2(((-1) *(wanted_x - current_x)), wanted_y - current_y) - math.atan2((-1*(current_x - prev_x)), current_y - prev_y)

                # Plan the small segment of the trajectory
                body_frame_dx = c * math.cos(dyaw)
                body_frame_dy = c * math.sin(dyaw)

                j += 1


        ''' -------------------------------------------------------   QUADRANT T    ----------------------------------------------------------------'''
        if i > round(steps/2) and i <= round(steps * 0.75) :


            # second setpoint
            if i == round(steps/2) + 1: 

                j = 1
                total_angle = j * angle_increment
                wanted_x = radius * math.cos(total_angle)
                wanted_y = radius * math.sin(total_angle)
                print('Wanted X & Y Pos:', wanted_x, wanted_y )

                #Current Position
                prev_angle = 0
                current_x = radius
                current_y = 0

                body_frame_dx = wanted_y
                body_frame_dy = (-1) *(wanted_x - current_x)
                
                #opp/adj
                dyaw = math.atan2(body_frame_dy, body_frame_dy)

                j += 1


            #remaining setpoints
            else:

                total_angle = j * angle_increment
                wanted_x = radius * math.cos(total_angle)
                wanted_y = radius * math.sin(total_angle)
                print('Wanted X & Y Pos:', wanted_x, wanted_y )

                #Current Position   
                current_angle = (j - 1) * angle_increment
                current_x = radius * math.cos(current_angle)
                current_y = radius * math.sin(current_angle)
                print('Current X, Y:       ',current_x, current_y)

                #Previous Position
                prev_angle = (j - 2) * angle_increment
                prev_x = radius * math.cos(prev_angle)
                prev_y = radius * math.sin(prev_angle)
                print('Prev dX, dY:        ',prev_x, prev_y)


                #vector/hypotnuese from current point --> desired point
                c = math.sqrt( pow( ((-1) *(wanted_x - current_x)),2) + pow((wanted_y - current_y),2))
                
                #dyaw = theta(next_total) - theta(old_increment)
                dyaw =  math.atan2(((-1) *(wanted_x - current_x)), wanted_y - current_y) - math.atan2((-1*(current_x - prev_x)), current_y - prev_y)

                # Plan the small segment of the trajectory
                body_frame_dx = c * math.cos(dyaw)
                body_frame_dy = c * math.sin(dyaw)

                j += 1
            
        ''' -------------------------------------------------------   QUADRANT C     ----------------------------------------------------------------'''
        if i >= round(steps * 0.75) and i < steps - 1:

            # second setpoint
            if i == round(steps*0.75) + 1: 

                j = 1
                total_angle = j * angle_increment
                wanted_x = radius * math.cos(total_angle)
                wanted_y = radius * math.sin(total_angle)
                print('Wanted X & Y Pos:', wanted_x, wanted_y )

                #Current Position
                prev_angle = 0
                current_x = radius
                current_y = 0

                body_frame_dx = wanted_y
                body_frame_dy = (-1) *(wanted_x - current_x)
                
                #opp/adj
                dyaw = math.atan2(body_frame_dy, body_frame_dy)

                j += 1


            #remaining setpoints
            else:

                total_angle = j * angle_increment
                wanted_x = radius * math.cos(total_angle)
                wanted_y = radius * math.sin(total_angle)
                print('Wanted X & Y Pos:', wanted_x, wanted_y )

                #Current Position   
                current_angle = (j - 1) * angle_increment
                current_x = radius * math.cos(current_angle)
                current_y = radius * math.sin(current_angle)
                print('Current X, Y:       ',current_x, current_y)

                #Previous Position
                prev_angle = (j - 2) * angle_increment
                prev_x = radius * math.cos(prev_angle)
                prev_y = radius * math.sin(prev_angle)
                print('Prev dX, dY:        ',prev_x, prev_y)


                #vector/hypotnuese from current point --> desired point
                c = math.sqrt( pow((wanted_x - current_x),2) + pow((wanted_y - current_y),2))
                
                #dyaw = theta(next_total) - theta(old_increment)
                dyaw =  math.atan2(((-1) *(wanted_x - current_x)), wanted_y - current_y) - math.atan2((-1*(current_x - prev_x)), current_y - prev_y)

                # Plan the small segment of the trajectory
                body_frame_dx = c * math.cos(dyaw)
                body_frame_dy = c * math.sin(dyaw)

                j += 1
        



        print('Body Frame dX & dY: ', body_frame_dx, body_frame_dy)

        print(f"Counter value : {i}")
        counter+=1

        print('Passed Angle [rad]:',dyaw , '\n')