
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


LOGGER = logging.getLogger()

#loading robot identification from environment
from dotenv import load_dotenv
load_dotenv()
ROBOT_IP = os.getenv('ROBOT_IP')
print(ROBOT_IP)
USERNAME = os.getenv('SPOT_ROBOT_USERNAME')
print(USERNAME)
PASSWORD = os.getenv('PASSWORD')
print(PASSWORD)

def _grpc_or_log(desc, thunk):
    try:
        return thunk()
    except (ResponseError, RpcError) as err:
        LOGGER.error('Failed %s: %s', desc, err)

class AsyncRobotState(AsyncPeriodicQuery):
  
    def __init__(self, robot_state_client):
        super(AsyncRobotState, self).__init__('robot_state', robot_state_client, LOGGER,
                                              period_sec=0.2)

    def _start_query(self):
        print("Starting robot state query.")
        try:
            future = self._client.get_robot_state_async()
            if future:
                result = future.result()
                if result:
                    print("Robot state query successful.")
                else:
                    print("Robot state query returned no result.")
            else:
                print("Failed to initiate async query.")
            return future
        except RpcError as e:
            print(f"RPC Error during async query: {e}")
            return None


class SpotControlInterface():

    def __init__(self):

        sdk = bosdyn.client.create_standard_sdk('CircularPathingClient')
        robot = sdk.create_robot(ROBOT_IP)
        self._robot = robot
    
        try:
            robot.authenticate( USERNAME, PASSWORD)
            robot.start_time_sync()
        except RpcError as err:
            LOGGER.error('Failed to communicate with robot: %s', err)

        self._power_client = robot.ensure_client(PowerClient.default_service_name)
        self._lease_client = robot.ensure_client(bosdyn.client.lease.LeaseClient.default_service_name)
        self._robot_command_client = robot.ensure_client(RobotCommandClient.default_service_name)

        try:
            self._estop_client = self._robot.ensure_client(EstopClient.default_service_name)
            self._estop_endpoint = EstopEndpoint(self._estop_client, 'GNClient', 9.0)
            #self._estop_keepalive = EstopKeepAlive(self._estop_endpoint)
        except:
            self._estop_client = None
            self._estop_endpoint = None
        self.robot_state_client = robot.ensure_client(RobotStateClient.default_service_name)
    
        self._lock = threading.Lock()

        self._locked_messages = ['', '', '']  # string: displayed message for user
        self._estop_keepalive = None
        self._exit_check = None
        self._robot_id = None
        self._lease_keepalive = None

        self.start()
        self._toggle_estop()
        self._toggle_power()


        

    ################################### Initializing/Shutting Down Methods #############################################
    
    
    def start(self):
        """Begin communication with the robot."""

        self._lease = self._lease_client.take()
        print("Lease forcefully taken.")
        
        # Construct our lease keep-alive object, which begins RetainLease calls in a thread.
        self._lease_keepalive = LeaseKeepAlive(self._lease_client, must_acquire=True,
                                               return_at_exit=True)
        
        self._robot_id = self._robot.get_id()
        
        
        if self._estop_endpoint is not None:
            self._estop_endpoint.force_simple_setup(
            )  # Set this endpoint as the robot's sole estop.

        print('Spot Connection Command Completed')
        
    def _toggle_estop(self):
        """toggle estop on/off. Initial state is ON"""
        if self._estop_client is not None and self._estop_endpoint is not None:
            if not self._estop_keepalive:
                self._estop_keepalive = EstopKeepAlive(self._estop_endpoint)
                print('estop is active')
            else:
                self._try_grpc('stopping estop', self._estop_keepalive.stop)
                self._estop_keepalive.shutdown()
                self._estop_keepalive = None
                print('estop is inactive')
    
    @property
    def robot_state(self):
        """Get latest robot state proto."""
        state = self.robot_state_client.get_robot_state()
        if state is None:
            print("Robot state task proto is not available")
        return state
    
    def _request_power_on(self):
        request = PowerServiceProto.PowerCommandRequest.REQUEST_ON
        return self._power_client.power_command_async(request)
    
    
    def _toggle_power(self):
        '''  Checks for power state, if not available nothing occures, however if on or off the opposite action will occure (power on/off) '''
        power_state = self._power_state()
        if power_state is None:
            print('Could not toggle power because power state is unknown')
            return

        if power_state == robot_state_proto.PowerState.STATE_OFF:
            self._try_grpc_async('powering-on', self._request_power_on)

        else:
            self._try_grpc('powering-off', self._robot._safe_power_off)
        print('Power Command Completed')
    
    def _power_state(self):
        state = self.robot_state
        if not state:
            print("Robot state is None")
            return None
        print("Robot state:", state)
        return state.power_state.motor_power_state

    def _toggle_lease(self):
        """toggle lease acquisition. Initial state is acquired"""
        if self._lease_client is not None:
            if self._lease_keepalive is None:
                self._lease_keepalive = LeaseKeepAlive(self._lease_client, must_acquire=True,
                                                       return_at_exit=True)
            else:
                self._lease_keepalive.shutdown()
                self._lease_keepalive = None

    def shutdown(self):
        """Release control of robot as gracefully as possible."""
        self._sit()
        if self._estop_keepalive:
            # This stops the check-in thread but does not stop the robot.
            self._estop_keepalive.shutdown()
        if self._lease_keepalive:
            self._lease_keepalive.shutdown()

        print('Spot Shutdown Completed')


    def _try_grpc(self, desc, thunk):

        try:
            return thunk()
        except (ResponseError, RpcError, LeaseBaseError) as err:
            print(f'Failed {desc}: {err}')
            return None

    
    def _try_grpc_async(self, desc, thunk):

        def on_future_done(fut):
            try:
                fut.result()
            except (ResponseError, RpcError, LeaseBaseError) as err:
                print(f'Failed {desc}: {err}')
                return None

        future = thunk()
        future.add_done_callback(on_future_done)
        
################################# Other Methods (Movement Related Etc.) #############################################
    

    def _start_robot_command(self, desc, command_proto, end_time_secs=None):

        def _start_command():
            self._robot_command_client.robot_command(command=command_proto,
                                                        end_time_secs=end_time_secs)

        self._try_grpc(desc, _start_command) 

    def _self_right(self):
        self._start_robot_command('self_right', RobotCommandBuilder.selfright_command())

    def _sit(self):
        self._start_robot_command('sit', RobotCommandBuilder.synchro_sit_command())

    def _stand(self):
        self._start_robot_command('stand', RobotCommandBuilder.synchro_stand_command())
   
    def _stop(self):
        self._start_robot_command('stop', RobotCommandBuilder.stop_command())



################################################## Trajectory Planning #######################################################
    def _forward_line_trajectory(self):
        print("Completed forward trajectory.")
        frame_name = ODOM_FRAME_NAME

        traj_dx = 1 # [meters]

        traj_dy = 0  # [meters]

        traj_dyaw = math.pi # [degrees] ccw

        ending_time = 20 # [seconds]

        try:
            return self.trajectory_planner(traj_dx, traj_dy, traj_dyaw, frame_name, ending_time)
        finally:
            # Send a Stop at the end, regardless of what happened.
            self._robot_command_client.robot_command(RobotCommandBuilder.stop_command())
    


    def _circle_trajectory(self):
        frame_name = ODOM_FRAME_NAME

        # Define the circle parameters
        radius = 0.5# [meters]
        total_time = 200 # [seconds] Total time to complete the circle
        steps = 30# Number of steps for the full circle

        # Calculate the angular increment per step
        angle_increment = 2 * math.pi / steps

        # Calculate time per step
        time_per_step = total_time / steps

        for i in range(steps):
            # Calculate the incremental dx, dy, dyaw for each step
            total_angle = i * angle_increment
            dx = radius * math.cos(total_angle)
            dy = radius * math.sin(total_angle)
            if i < steps - 1:
                next_angle = (i + 1) * angle_increment
                next_dx = radius * math.cos(next_angle)
                next_dy = radius * math.sin(next_angle)
                dyaw = math.atan2(next_dy - dy, next_dx - dx)
            else:
                dyaw = 0
            # Plan the small segment of the trajectory
            self.trajectory_planner(dx, dy, dyaw, frame_name, time_per_step)

        print("Completed circular trajectory")
    

    def trajectory_planner(self, dx, dy, dyaw, frame_name, ending_time):

        
        #get the current transformation snapshot (provides us with Spot's pose relative to the different frames)
        transforms = self.robot_state_client.get_robot_state().kinematic_state.transforms_snapshot

        print(dyawS)
        #sets the end goal of where we want Spot to be, as well as its orientation
        body_tform_goal = math_helpers.SE2Pose(x=dx, y=dy, angle=dyaw)
        #print("body transform goal:", body_tform_goal)
        
        #converts the pose from the bodyframe to the designated 'global' frame (ODOM or VISION)
        out_tform_body = get_se2_a_tform_b(transforms, frame_name, BODY_FRAME_NAME)

        # Check if the transform was found
        if out_tform_body is None:
            print(f"Error: No transform found between {frame_name} and {BODY_FRAME_NAME}.")
            return False
        else:
            print("Output transform body:", out_tform_body)

        #convert the goal pose from the BODY to the desired 'global' frame (ODOM or VISION);'
        out_tform_goal = out_tform_body * body_tform_goal
        #print('output transform goal:', out_tform_goal)

        #builds the trajectory command based on the above transforms
        robot_cmd = RobotCommandBuilder.synchro_se2_trajectory_point_command(
            goal_x=out_tform_goal.x, goal_y=out_tform_goal.y, goal_heading=out_tform_goal.angle,
            frame_name=frame_name)

        #send the command
        end_time = ending_time
        cmd_id = self._robot_command_client.robot_command(command=robot_cmd,
                                                           end_time_secs=time.time() + end_time)


        #Wait until the robot has reached the goal.
        while True:
            feedback = self._robot_command_client.robot_command_feedback(cmd_id)
            mobility_feedback = feedback.feedback.synchronized_feedback.mobility_command_feedback
            if mobility_feedback.status != RobotCommandFeedbackStatus.STATUS_PROCESSING:
                print('Failed to reach the goal')
                return False
            traj_feedback = mobility_feedback.se2_trajectory_feedback
            if (traj_feedback.status == traj_feedback.STATUS_AT_GOAL and
                    traj_feedback.body_movement_status == traj_feedback.BODY_STATUS_SETTLED):
                print('Arrived at the goal.')

                transforms = self.robot_state_client.get_robot_state().kinematic_state.transforms_snapshot
                out_tform_body = get_se2_a_tform_b(transforms, frame_name, BODY_FRAME_NAME)
                #print("Output transform body after completion:", out_tform_body)
                #print('output transform goal after completion:', out_tform_goal)

                return True
            time.sleep(1)
            
        return True







################################################# ROS SUBSCRIPTION NODE ("Front-End") ######################################

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String
import sys, select, termios, tty

class KeySubscriber(Node):

    def __init__(self):
        
        super().__init__('key_subscriber')
     
        self.subscription = self.create_subscription(String,'spot_keypress',self.listener_callback,10)
        self.get_logger().info('Keypress Subscriber Node has been started.')
        self.spot_interface = SpotControlInterface()

        self._command_dictionary = {
            'w': self.spot_interface._stop,              # Stop moving
            ' ': self.spot_interface._toggle_estop,      # Toggle estop
            '\t': self.spot_interface.shutdown,          # Shut down and quit
            'p': self.spot_interface._toggle_power,      # Toggle power
            'l': self.spot_interface._toggle_lease,      # Toggle lease
            's': self.spot_interface.start,              # Start
            'r': self.spot_interface._self_right,        # Self right
            'v': self.spot_interface._sit,               # Sit
            'f': self.spot_interface._stand,             # Stand
            'c': self.spot_interface._circle_trajectory, # Circle trajectory
            'y': self.spot_interface._forward_line_trajectory    # Forward. trajectory
        }

    def listener_callback(self, msg):
        """Run user commands at each update."""
        try:
            key = msg.data 
            cmd_function = self._command_dictionary[key]
            cmd_function()

        except:
            if key not in self._command_dictionary:
                self.get_logger().info(f"Unrecognized keyboard command: '{key}'")

def main(args=None):
    
    rclpy.init(args=args)
    
    key_subscriber = KeySubscriber()

    rclpy.spin(key_subscriber)

    key_subscriber.destroy_node()
    rclpy.shutdown()









    

#############################################################  CODE FOR TESTING ROS CONNECTION ###############################################
# class testing_ROS_Interface():

#     def __init__(self):
#         print('interface initialized')

#     def _self_right(self):
#         print('Testing_ROS_Interface self_right method called')

#     def _sit(self):
#         print('Testing_ROS_Interface sit method called')

#     def stand(self):
#         print('Testing_ROS_Interface stand method called')

#     def _stop(self):
#         print('Testing_ROS_Interface stop method called')

# class KeySubscriber(Node):
    
#     def __init__(self):
#         super().__init__('key_subscriber')
#         self.subscription = self.create_subscription(String,'spot_keypress',self.listener_callback,10)
#         self.get_logger().info('Keypress Subscriber Node has been started.')

#         self.testing_R0S_Interface = testing_ROS_Interface()

#         self.command_dictionary = {
#             'w': self.testing_R0S_Interface._self_right,          # Stop moving
#             ' ': self.testing_R0S_Interface._sit,  # Toggle estop
#             '\t': self.testing_R0S_Interface.stand,      # Shut down and quit
#         }

#     def listener_callback(self, msg):
#         """Run user commands at each update."""
#         try:
#             key = msg.data 
#             cmd_function = self.command_dictionary[key]
#             cmd_function()

#         except:
#             if key not in self.command_dictionary:
#                 self.get_logger().info(f"Unrecognized keyboard command: '{key}'")


# def main(args=None):

#     rclpy.init(args=args)

#     key_subscriber = KeySubscriber()

#     rclpy.spin(key_subscriber)

#     key_subscriber.destroy_node()
#     rclpy.shutdown()




if __name__ == '__main__':
     main()
