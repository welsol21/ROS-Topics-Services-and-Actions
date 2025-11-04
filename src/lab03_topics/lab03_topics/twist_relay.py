import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TwistRelay(Node):
    def __init__(self):
        super().__init__('twist_relay')
        self.subscriber = self.create_subscription(
            Twist,
            '/control_cmd',
            self.cmd_callback,
            10
        )
        self.publisher = self.create_publisher(
            Twist,
            '/turtle1/cmd_vel',
            10
        )

    def cmd_callback(self, msg):
        self.publisher.publish(msg)
        self.get_logger().info(f'Relayed: linear.x={msg.linear.x}, angular.z={msg.angular.z}')

def main():
    rclpy.init()
    node = TwistRelay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
