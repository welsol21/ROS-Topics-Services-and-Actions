import rclpy
from rclpy.node import Node
from custom_interfaces.msg import Sphere

class SphereListener(Node):
    def __init__(self):
        super().__init__('sphere_listener')
        self.subscription = self.create_subscription(Sphere, 'sphere_topic', self.callback, 10)

    def callback(self, msg):
        x = msg.center.x
        y = msg.center.y
        r = msg.radius
        self.get_logger().info(f'Received sphere: center=({x:.2f}, {y:.2f}), radius={r:.2f}')

def main():
    rclpy.init()
    node = SphereListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
