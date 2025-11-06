#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from custom_interfaces.srv import FindClosestTurtle


class ClosestTurtleClient(Node):
    def __init__(self):
        super().__init__('closest_turtle_client')
        self.client = self.create_client(FindClosestTurtle, '/find_closest_turtle')
        
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /find_closest_turtle service...')
        
        self.get_logger().info('Service available, calling every 3 seconds...')
        self.create_timer(3.0, self.call_service)
    
    def call_service(self):
        request = FindClosestTurtle.Request()
        future = self.client.call_async(request)
        future.add_done_callback(self.handle_response)
    
    def handle_response(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(
                    f'✅ {response.message}'
                )
            else:
                self.get_logger().warning(f'⚠️  {response.message}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')


def main():
    rclpy.init()
    node = ClosestTurtleClient()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
