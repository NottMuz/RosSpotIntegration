''' SPOT API ("Back-End) '''

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
from bosdyn.client.robot_command import RobotCommandBuilder, RobotCommandClient
from bosdyn.client.robot_state import RobotStateClient
from bosdyn.client.time_sync import TimeSyncError
from bosdyn.util import duration_str, format_metric, secs_to_hms

LOGGER = logging.getLogger()

VELOCITY_BASE_SPEED = 0.5  # m/s
VELOCITY_BASE_ANGULAR = 0.8  # rad/sec
VELOCITY_CMD_DURATION = 0.6  # seconds
COMMAND_INPUT_RATE = 0.1


from dotenv import load_dotenv
# Load the .env file
load_dotenv()
# Retrieve the values from the environment variables
ROBOT_IP = os.getenv('ROBOT_IP')
USERNAME = os.getenv('USERNAME')
PASSWORD = os.getenv('PASSWORD')



def _grpc_or_log(desc, thunk):
    try:
        return thunk()
    except (ResponseError, RpcError) as err:
        LOGGER.error('Failed %s: %s', desc, err)
AsyncRobotStateAsyncPeriodicQuery

# for exiting loops ('while' loops, etc)
# class ExitCheck(object):
    # """A class to help exiting a loop, also capturing SIGTERM to exit the loop."""

    # def __init__(self):
    #     self._kill_now = False
    #     signal.signal(signal.SIGTERM, self._sigterm_handler)
    #     signal.signal(signal.SIGINT, self._sigterm_handler)

    # def __enter__(self):
    #     return self

    # def __exit__(self, _type, _value, _traceback):
    #     return False

    # def _sigterm_handler(self, _signum, _frame):
    #     self._kill_now = True

    # def request_exit(self):
    #     """Manually trigger an exit (rather than sigterm/sigint)."""
    #     self._kill_now = True

    # @property
    # def kill_now(self):
    #     """Return the status of the exit checker indicating if it should exit."""
    #     return self._kill_now


class CursesHandler(logging.Handler):
    """logging handler which puts messages into the curses interface"""

    def __init__(self, wasd_interface):
        super(CursesHandler, self).__init__()
        self._wasd_interface = wasd_interface

    def emit(self, record):
        msg = record.getMessage()
        msg = msg.replace('\n', ' ').replace('\r', '')
        self._wasd_interface.add_message(f'{record.levelname:s} {msg:s}')


class AsyncRobotState(AsyncPeriodicQuery):

    #  What is the passed parameter it is handling? :
        """ query name, SDK client, timing of queries, handled by the state client passed to it """
    
    """Grab robot state."""

        """Includes a few key states:
        1. Battery State: Charge level, status, health
        2. Motor State: Temperatures, power consumption, errors/warnings
        3. Joint State: Positions, Velocitoes, efforts of the joints
        4. Kinematic State: Robot's current pose and position in the world 
        5. Sensor Data: Data from the robot's various sensors, such as cameras, LIDAR, and force sensors
        6. E-Stop State
        7. Power State: active power faults or warnings"""

    def __init__(self, robot_state_client):
        super(AsyncRobotState, self).__init__('robot_state', robot_state_client, LOGGER,
                                              period_sec=0.2)

    def _start_query(self):
        return self._client.get_robot_state_async()


class SpotControlInterface(object):

    def __init__(self, robot):

        sdk = bosdyn.client.create_standard_sdk('CircularPathingClient')

        #gets the IP address from the robot from the 
        robot = sdk.create_robot(ROBOT_IP)
        bosdyn.client.util.authenticate(robot, USERNAME, PASSWORD)

        # Time-sync
        time_sync_client = robot.ensure_client(TimeSyncClient.default_service_name)
        bosdyn.client.util.sync_robot_time(robot)

        assert not robot.is_estopped(),'Robot is estopped. Please use an external E-Stop client to configure E-Stop.'


        # Create the lease client and grab the lease
        lease_client = robot.ensure_client(bosdyn.client.lease.LeaseClient.default_service_name)
        # Command client
        command_client = robot.ensure_client(RobotCommandClient.default_service_name)
        # E-Stop client
        try:
            self._estop_client = self._robot.ensure_client(EstopClient.default_service_name)
            '''Need to know whats happening below, come back '''
            self._estop_endpoint = EstopEndpoint(self._estop_client, 'GNClient', 9.0)
        except:
            # Not the estop.
            self._estop_client = None
            self._estop_endpoint = None
        # Robot state client
        robot_state_client = robot.ensure_client(RobotStateClient.default_service_name)
        # Power client
        power_client = robot.ensure_client(PowerClient.default_service_name)
        # Data acquisition client
        data_acquisition_client = robot.ensure_client(DataAcquisitionClient.default_service_name)

        #initiate power on
        self._power_motors()
        self._robot_state_task = AsyncRobotState(self._robot_state_client)
        self._lock = threading.Lock()
        
        self._async_tasks = AsyncTasks([self._robot_state_task])
        self._lock = threading.Lock()

        self._locked_messages = ['', '', '']  # string: displayed message for user
       
        self._estop_keepalive = None
        self._exit_check = None

        # Stuff that is set in start()
        self._robot_id = None
        self._lease_keepalive = None

######################################## Initializing/Shutting Down Methods #############################################
    
    
    def start(self):
        """Begin communication with the robot."""
        # Construct our lease keep-alive object, which begins RetainLease calls in a thread.
        self._lease_keepalive = LeaseKeepAlive(self._lease_client, must_acquire=True,
                                               return_at_exit=True)

        self._robot_id = self._robot.get_id()
        if self._estop_endpoint is not None:
            self._estop_endpoint.force_simple_setup(
            )  # Set this endpoint as the robot's sole estop.

    ''' will need this after the program is requested to be exited '''
    def shutdown(self):
        """Release control of robot as gracefully as possible."""
        self._sit()
        if self._estop_keepalive:
            # This stops the check-in thread but does not stop the robot.
            self._estop_keepalive.shutdown()
        if self._lease_keepalive:
            self._lease_keepalive.shutdown()
    
        #DIRECTLY CALLED (FOR EXITING A WHILE LOOP)
    # def _quit_program(self):
        #     ''' Quits Program Via Exit Check Method '''
        #     self._sit()
        #     if self._exit_check is not None:
        #         self._exit_check.request_exit()

    def _request_power_on(self):
        '''  PLACE HOLDER FOR EXPLANATION '''
        request = PowerServiceProto.PowerCommandRequest.REQUEST_ON
        return self._power_client.power_command_async(request)
        
    def _power_motors(self):
        """Powers the motors on in the robot. """

        if self.motors_powered or not self.has_robot_control or not self.estop_keepalive or self.robot.is_powered_on(
        ):
            return

        self.robot.power_on(timeout_sec=20)
        assert robot.is_powered_on(), 'Robot power on failed.'
        self.motors_powered = True

    #DIRECTLY CALLED
    def _toggle_estop(self):
        """toggle estop on/off. Initial state is ON"""
        if self._estop_client is not None and self._estop_endpoint is not None:
            if not self._estop_keepalive:
                self._estop_keepalive = EstopKeepAlive(self._estop_endpoint)
            else:
                self._try_grpc('stopping estop', self._estop_keepalive.stop)
                self._estop_keepalive.shutdown()
                self._estop_keepalive = None

    #DIRECTLY CALLED
    def _toggle_power(self):
        '''  PLACE HOLDER FOR EXPLANATION '''
        power_state = self._power_state()
        if power_state is None:
            self.add_message('Could not toggle power because power state is unknown')
            return

        if power_state == robot_state_proto.PowerState.STATE_OFF:
            self._try_grpc_async('powering-on', self._request_power_on)

        else:
            self._try_grpc('powering-off', self._safe_power_off)

    #DIRECTLY CALLED
    def _toggle_lease(self):
        """toggle lease acquisition. Initial state is acquired"""
        if self._lease_client is not None:
            if self._lease_keepalive is None:
                self._lease_keepalive = LeaseKeepAlive(self._lease_client, must_acquire=True,
                                                       return_at_exit=True)
            else:
                self._lease_keepalive.shutdown()
                self._lease_keepalive = None

    @property
    def robot_state(self):
        """Get latest robot state proto."""
        return self._robot_state_task.proto

    def _try_grpc(self, desc, thunk):
        '''  PLACE HOLDER FOR EXPLANATION '''
        try:
            return thunk()
        except (ResponseError, RpcError, LeaseBaseError) as err:
            # self.add_message(f'Failed {desc}: {err}')
            return None

    
    def _try_grpc_async(self, desc, thunk):
        '''  PLACE HOLDER FOR EXPLANATION '''
        def on_future_done(fut):
            try:
                fut.result()
            except (ResponseError, RpcError, LeaseBaseError) as err:
                # self.add_message(f'Failed {desc}: {err}')
                return None

        future = thunk()
        future.add_done_callback(on_future_done)

    # start allowing robot commands to happen
    def _start_robot_command(self, desc, command_proto, end_time_secs=None):

        def _start_command():
            self._robot_command_client.robot_command(command=command_proto,
                                                        end_time_secs=end_time_secs)

        self._try_grpc(desc, _start_command) 
    

######################################## Other Methods (Movement Related Etc.) #############################################









#########################################################################################################################################################################################################
'''ROS SUBSCRIPTION NODE ("Front-End")'''

class KeySubscriber(Node):
    def __init__(self):
        super().__init__('key_subscriber')
        self.subscription = self.create_subscription(String,'spot_keypress',self.listener_callback,10)
        self.get_logger().info('Key Subscriber Node has been started.')

    def listener_callback(self, msg):
        self.get_logger().info(f'Received: {msg.data}')
        # Here you would add the logic to interact with the Spot API based on the key press
        if msg.data == 'a':
            self.stop_spot()

    def stop_spot(self):
        # Placeholder function to stop the robot
        self.get_logger().info('Spot Stop Command Triggered')

def main(args=None):
    rclpy.init(args=args)
    node = KeySubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
