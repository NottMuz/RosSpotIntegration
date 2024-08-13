import rclpy
from std_msgs.msg import String
from bosdyn.client import create_standard_sdk
from bosdyn.client.robot_command import RobotCommandBuilder, RobotCommandClient
from bosdyn.client.lease import LeaseClient
from bosdyn.client.estop import EstopClient

def handle_keypress(data):
    key = data.data
    rclpy.loginfo(f"Received keypress: {key}")

    if key == 'a':
        rclpy.loginfo("Stopping Spot.")
        # Send stop command to Spot here
        stop_command = RobotCommandBuilder.synchro_stop_command()
        command_client.robot_command(command=stop_command)
    elif key == 'f':
        rclpy.loginfo("Spot: Stand up.")
        # Command Spot to stand
        stand_command = RobotCommandBuilder.synchro_stand_command()
        command_client.robot_command(command=stand_command)
    # Add more key-to-command mappings here

def spot_subscriber():
    rclpy.init_node('spot_subscriber')

    # Setup Spot SDK and create clients
    sdk = create_standard_sdk('SpotSubscriber')
    robot = sdk.create_robot('ROBOT_IP')
    robot.authenticate('USERNAME', 'PASSWORD')
    robot.time_sync.wait_for_sync()

    global command_client
    command_client = robot.ensure_client(RobotCommandClient.default_service_name)

    rclpy.Subscriber('keypress_topic', String, handle_keypress)
    rclpy.loginfo("Spot subscriber ready and listening to keypresses.")

    rclpy.spin()

if __name__ == '__main__':
    try:
        spot_subscriber()
    except rclpy.ROSInterruptException:
        pass