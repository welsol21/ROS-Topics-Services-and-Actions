import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
import math

class MoveDistance(Node):
    def __init__(self):
        super().__init__('move_distance')
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.subscription = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)

        self.prev_pose = None
        self.total_distance = 0.0
        self.target_distance = 5.0  # meters
        self.moving = True

    def pose_callback(self, msg):
        if not self.moving:
            return


        if msg.x < 0.5 or msg.x > 10.5 or msg.y < 0.5 or msg.y > 10.5:
            self.get_logger().info('Boundary reached — stopping')
            self.publisher.publish(Twist())
            self.moving = False
            return

        if self.prev_pose is None:
            self.prev_pose = msg
            self.get_logger().info('Starting point recorded')
            return

        dx = msg.x - self.prev_pose.x
        dy = msg.y - self.prev_pose.y
        delta = math.sqrt(dx**2 + dy**2)
        self.total_distance += delta
        self.prev_pose = msg

        self.get_logger().info(f'Distance traveled: {self.total_distance:.2f} m')

        if self.total_distance >= self.target_distance:
            self.get_logger().info('Target reached — stopping')
            self.publisher.publish(Twist())
            self.moving = False
        else:
            move = Twist()
            move.linear.x = 1.0
            self.publisher.publish(move)


def main():
    rclpy.init()
    node = MoveDistance()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
