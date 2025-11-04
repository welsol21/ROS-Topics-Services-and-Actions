import rclpy
from rclpy.node import Node
from custom_interfaces.msg import Time

class TimeListener(Node):
    def __init__(self):
        super().__init__('time_listener')
        self.subscription = self.create_subscription(Time, 'time_topic', self.callback, 10)

    def callback(self, msg):
        self.get_logger().info(f'Received time: {msg.hours:02d}:{msg.minutes:02d}:{msg.seconds:02d}')

def main():
    rclpy.init()
    node = TimeListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
