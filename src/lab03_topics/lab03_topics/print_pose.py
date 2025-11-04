import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose

class PrintPose(Node):
    def __init__(self):
        super().__init__('pose_logger')
        self.subscription = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.pose_callback,
            10
        )

    def pose_callback(self, msg):
        self.get_logger().info(
            f'Turtle position: x={msg.x:.2f}, y={msg.y:.2f}, theta={msg.theta:.2f}'
        )

def main():
    rclpy.init()
    node = PrintPose()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
