#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from turtlesim.msg import Pose

CHECK_TIMEOUT = 2.5  # Pose timeout threshold (seconds)
HARD_REMOVE  = 5.0  # Hard remove threshold (seconds)

class TurtleMonitorService(Node):
    # Subscribe to turtle1 and create timers
    def __init__(self):
        super().__init__('turtle_monitor_service')


        self.turtles = {}
        self.create_service(Trigger, '/monitor_turtles', self.on_monitor)
        self.create_timer(1.0, self._health_scan)

        self.create_timer(2.0, self._scan_topics)


        self._ensure_subscription('turtle1')

        self.get_logger().info('🐢 Turtle Monitor Service started')

    # Ensure subscription to a pose topic for the given turtle
    def _ensure_subscription(self, name: str):
        if name in self.turtles:
            return
        self.turtles[name] = {'last_pose': time.time(), 'alive': False}
        self.create_subscription(Pose, f'/{name}/pose',
                                 lambda msg, n=name: self._on_pose(n),
                                 10)
        self.get_logger().info(f'📡 Now monitoring pose of: {name}')

    def _on_pose(self, name: str):
        now = time.time()
        info = self.turtles.get(name)
        if not info:

            self._ensure_subscription(name)
            info = self.turtles[name]
        info['last_pose'] = now
        info['alive'] = True

    # Scan topics and subscribe to new turtles
    def _scan_topics(self):

        topic_list = self.get_topic_names_and_types()
        

        active_topics = set()
        for topic_name, _ in topic_list:

            if topic_name.endswith('/pose'):

                turtle_name = topic_name.split('/')[1]
                if turtle_name:
                    active_topics.add(turtle_name)

                    if turtle_name not in self.turtles:
                        self._ensure_subscription(turtle_name)
        

        for turtle_name in list(self.turtles.keys()):
            if turtle_name not in active_topics:
                if self.turtles[turtle_name]['alive']:
                    self.get_logger().warning(f'🗑️  Topic disappeared, removing turtle from tracking: {turtle_name}')

                del self.turtles[turtle_name]

    # Periodic health scan: mark alive/removed based on pose timestamps
    def _health_scan(self):
        now = time.time()
        for name, info in list(self.turtles.items()):
            dt = now - info['last_pose']
            if dt > HARD_REMOVE:

                if info['alive']:
                    self.get_logger().warning(f'⚠️  Turtle appears removed: {name}')
                info['alive'] = False
            elif dt > CHECK_TIMEOUT:

                info['alive'] = False

    # Service callback: report ACTIVE and REMOVED turtles
    def on_monitor(self, request, response):

        alive = [n for n, d in self.turtles.items() if d['alive']]
        removed = [n for n, d in self.turtles.items() if not d['alive']]


        response.success = True
        response.message = f"ACTIVE:{','.join(alive)};REMOVED:{','.join(removed)}"
        return response

def main():
    rclpy.init()
    node = TurtleMonitorService()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
