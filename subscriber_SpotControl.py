import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class KeySubscriber(Node):
    def __init__(self):
        super().__init__('key_subscriber')
        self.subscription = self.create_subscription(
            String,
            'spot_keypress',
            self.listener_callback,
            10)
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
