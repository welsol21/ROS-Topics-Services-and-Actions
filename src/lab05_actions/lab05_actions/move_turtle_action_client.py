import sys
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from custom_interfaces.action import MoveTurtle
from action_msgs.msg import GoalStatus

class MoveTurtleActionClient(Node):
    def __init__(self):
        super().__init__('move_turtle_action_client')
        self.action_client = ActionClient(self, MoveTurtle, 'move_turtle')

    def send_goal(self, x, y):
        self.action_client.wait_for_server()

        goal_msg = MoveTurtle.Goal()
        goal_msg.x = x
        goal_msg.y = y

        self.send_goal_future = self.action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Remaining distance: {feedback.distance:.2f}')

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')
        self.goal_handle = goal_handle
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)

        # Таймер отмены через 10 секунд
        self.timer = self.create_timer(10.0, self.timer_callback)

    def timer_callback(self):
        if not self.get_result_future.done():
            self.cancel_future = self.goal_handle.cancel_goal_async()
            self.cancel_future.add_done_callback(self.cancel_callback)
            self.timer.cancel()

    def cancel_callback(self, future):
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info('Goal successfully canceled')
        else:
            self.get_logger().info('Goal failed to cancel')
        rclpy.shutdown()

    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Goal succeeded!')
        else:
            self.get_logger().info(f'Goal failed with status: {status}')
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    client = MoveTurtleActionClient()
    if len(sys.argv) < 3:
        print("Usage: ros2 run lab05_actions move_turtle_client <x> <y>")
        return
    x = float(sys.argv[1])
    y = float(sys.argv[2])
    client.send_goal(x, y)
    rclpy.spin(client)

if __name__ == '__main__':
    main()
