import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty
from custom_interfaces.srv import CircleDuration

class MoveTurtleArcClient(Node):
    def __init__(self):
        super().__init__('move_turtle_arc_client')
        # self.cli = self.create_client(Empty, 'move_turtle_arc_service')
        self.cli = self.create_client(CircleDuration, 'move_turtle_arc_service')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service...')
        # self.req = Empty.Request()
        self.req = CircleDuration.Request()

    def send_request(self, duration):
        self.req.duration = duration
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main():
    rclpy.init()
    client = MoveTurtleArcClient()
    response = client.send_request(5)
    client.get_logger().info(f'Service called successfully: {response.success}')
    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
