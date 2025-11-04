import rclpy
from rclpy.node import Node
from std_msgs.msg import String
package_name = 'lab03_topics'

class Listener(Node):
    def __init__(self):
        super().__init__('listener')
        self.sub = self.create_subscription(String, 'chatter', self.cb, 10)

    def cb(self, msg: String):
        self.get_logger().info(f'Received: {msg.data}')

def main():
    rclpy.init()
    node = Listener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
