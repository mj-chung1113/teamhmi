import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import time

class ButtonStatePublisher(Node):
    def __init__(self):
        super().__init__('button_state_publisher')
        self.publisher_ = self.create_publisher(Bool, '/button_state', 10)
        self.timer_period = 10.0  # 10 seconds
        self.button_state = False
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.get_logger().info('ButtonStatePublisher node started.')

    def timer_callback(self):
        # Toggle the button state
        self.button_state = not self.button_state
        msg = Bool()
        msg.data = self.button_state

        # Publish the button state
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published: {msg.data}')

def main(args=None):
    rclpy.init(args=args)

    button_state_publisher = ButtonStatePublisher()

    try:
        rclpy.spin(button_state_publisher)
    except KeyboardInterrupt:
        button_state_publisher.get_logger().info('Node stopped by user.')
    finally:
        button_state_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()