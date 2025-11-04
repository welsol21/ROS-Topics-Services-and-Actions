import time
import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty
from geometry_msgs.msg import Twist
from custom_interfaces.srv import CircleDuration

class MoveTurtleArcService(Node):
    def __init__(self):
        super().__init__('move_turtle_arc_server')
        # self.srv = self.create_service(Empty, 'move_turtle_arc_service', self.move_turtle_callback)
        self.srv = self.create_service(CircleDuration, 'move_turtle_arc_service', self.move_turtle_callback)
        self.cmd_vel_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.get_logger().info('Service move_turtle_circle is ready.')

    def move_turtle_callback(self, request, response):
        self.get_logger().info(f'Received request: duration = {request.duration} seconds')

        move_msg = Twist()
        move_msg.linear.x = 0.5
        move_msg.angular.z = 0.5

        for i in range(request.duration):
            self.cmd_vel_pub.publish(move_msg)
            time.sleep(1)
        
        stop_msg = Twist()
        self.cmd_vel_pub.publish(stop_msg)

        response.success = True
        self.get_logger().info('Movement complete.')

        # self.cmd_vel_pub.publish(move_msg)
        return response

def main():
    rclpy.init()
    node = MoveTurtleArcService()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
