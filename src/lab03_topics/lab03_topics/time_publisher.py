import rclpy
from rclpy.node import Node
from custom_interfaces.msg import Time

class TimePublisher(Node):
    def __init__(self):
        super().__init__('time_publisher')
        self.publisher = self.create_publisher(Time, 'time_topic', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = Time()
        msg.seconds = self.i % 60
        msg.minutes = (self.i // 60) % 60
        msg.hours = (self.i // 3600) % 24
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: {msg.hours}:{msg.minutes}:{msg.seconds}')
        self.i += 1

def main():
    rclpy.init()
    node = TimePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
