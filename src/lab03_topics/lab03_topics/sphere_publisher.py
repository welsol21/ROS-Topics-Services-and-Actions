import rclpy
from rclpy.node import Node
from custom_interfaces.msg import Sphere
from geometry_msgs.msg import Point

class SpherePublisher(Node):
    def __init__(self):
        super().__init__('sphere_publisher')
        self.publisher = self.create_publisher(Sphere, 'sphere_topic', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.r = 1.0

    def timer_callback(self):
        msg = Sphere()
        msg.center = Point(x=5.5, y=5.5, z=0.0)
        msg.radius = self.r
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing sphere: center=({msg.center.x}, {msg.center.y}), radius={msg.radius:.2f}')
        self.r += 0.1

def main():
    rclpy.init()
    node = SpherePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
