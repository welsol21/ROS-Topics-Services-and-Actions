#!/usr/bin/env python3
"""
Service: monitors turtles and reports ACTIVE/REMOVED
"""

import time
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from turtlesim.msg import Pose

CHECK_TIMEOUT = 2.5   # pose timeout threshold (seconds)
HARD_REMOVE  = 5.0    # hard remove threshold (seconds)

class TurtleMonitorService(Node):
    """Service: monitors turtle topics and status"""
    def __init__(self):
        super().__init__('turtle_monitor_service')

        # name -> {'last_pose': timestamp, 'alive': bool}
        self.turtles = {}
        self.create_service(Trigger, '/monitor_turtles', self.on_monitor)
        self.create_timer(1.0, self._health_scan)
        # Timer to scan topics and detect new turtles
        self.create_timer(2.0, self._scan_topics)

        # Subscribe to turtle1 by default
        self._ensure_subscription('turtle1')

        self.get_logger().info('🐢 Turtle Monitor Service started')

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
            # new name discovered; ensure subscription
            self._ensure_subscription(name)
            info = self.turtles[name]
        info['last_pose'] = now
        info['alive'] = True

    def _scan_topics(self):
        """Scan topics and add subscriptions for new turtles"""
        topic_list = self.get_topic_names_and_types()
        
        # Build list of active turtles from topics
        active_topics = set()
        for topic_name, _ in topic_list:
            # Look for topics of form /<name>/pose
            if topic_name.endswith('/pose'):
                # Extract turtle name
                turtle_name = topic_name.split('/')[1]
                if turtle_name:
                    active_topics.add(turtle_name)
                    # Ensure it's not already tracked
                    if turtle_name not in self.turtles:
                        self._ensure_subscription(turtle_name)
        
        # Remove turtles whose topics disappeared
        for turtle_name in list(self.turtles.keys()):
            if turtle_name not in active_topics:
                if self.turtles[turtle_name]['alive']:
                    self.get_logger().warning(f'🗑️  Topic disappeared, removing turtle from tracking: {turtle_name}')
                # Remove from tracking
                del self.turtles[turtle_name]

    def _health_scan(self):
        now = time.time()
        for name, info in list(self.turtles.items()):
            dt = now - info['last_pose']
            if dt > HARD_REMOVE:
                # mark as removed (pose missing too long)
                if info['alive']:
                    self.get_logger().warning(f'⚠️  Turtle appears removed: {name}')
                info['alive'] = False
            elif dt > CHECK_TIMEOUT:
                # temporarily no signal; not permanently removed
                info['alive'] = False

    def on_monitor(self, request, response):
        # Do not scan topics here — monitor only reports current status
        alive = [n for n, d in self.turtles.items() if d['alive']]
        removed = [n for n, d in self.turtles.items() if not d['alive']]

        # String format: "ACTIVE:a,b;REMOVED:x,y"
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
