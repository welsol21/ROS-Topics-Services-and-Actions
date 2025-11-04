import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class ServiceClient(Node):
    def __init__(self):
        super().__init__('service_client')
        self.client = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service...')
        self.request = AddTwoInts.Request()
        self.request.a = 7
        self.request.b = 5

    def send_request(self):
        self.future = self.client.call_async(self.request)

def main(args=None):
    rclpy.init(args=args)
    client = ServiceClient()
    client.send_request()
    rclpy.spin_until_future_complete(client, client.future)
    response = client.future.result()
    client.get_logger().info(f'Result: {response.sum}')
    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
