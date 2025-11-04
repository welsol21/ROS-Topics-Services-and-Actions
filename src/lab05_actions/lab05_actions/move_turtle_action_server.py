import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from custom_interfaces.action import MoveTurtle

class MoveTurtleActionServer(Node):
    def __init__(self):
        super().__init__('move_turtle_action_server')

        self.action_server = ActionServer(
            self,
            MoveTurtle,
            'move_turtle',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=ReentrantCallbackGroup()
        )

        self.pose = [0.0, 0.0, 0.0]
        self.linear_gain = 2.0
        self.angular_gain = 4.0

        self.pose_sub = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

    def pose_callback(self, msg):
        self.pose = [msg.x, msg.y, msg.theta]

    def goal_callback(self, goal_request):
        self.get_logger().info('Received goal request')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    def get_distance(self, x, y):
        return np.sqrt((x - self.pose[0])**2 + (y - self.pose[1])**2)

    def get_angle(self, x, y):
        angle = np.arctan2(y - self.pose[1], x - self.pose[0]) - self.pose[2]
        if angle <= -np.pi:
            angle += 2 * np.pi
        elif angle > np.pi:
            angle -= 2 * np.pi
        return angle

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        x = goal_handle.request.x
        y = goal_handle.request.y

        feedback_msg = MoveTurtle.Feedback()
        move_msg = Twist()

        start_time = time.time()

        while True:
            distance = self.get_distance(x, y)
            angle = self.get_angle(x, y)

            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return MoveTurtle.Result(success=False)

            if time.time() - start_time > 10.0:
                goal_handle.canceled()
                self.get_logger().info('Goal timed out')
                return MoveTurtle.Result(success=False)

            if distance < 0.5:
                break

            feedback_msg.distance = distance
            goal_handle.publish_feedback(feedback_msg)

            move_msg.linear.x = self.linear_gain * distance
            move_msg.angular.z = self.angular_gain * angle
            self.cmd_vel_pub.publish(move_msg)

            time.sleep(0.1)

        self.cmd_vel_pub.publish(Twist())  # Stop turtle
        goal_handle.succeed()
        self.get_logger().info('Goal succeeded')
        return MoveTurtle.Result(success=True)

def main(args=None):
    rclpy.init(args=args)
    server = MoveTurtleActionServer()
    executor = MultiThreadedExecutor()
    rclpy.spin(server, executor=executor)
    server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
