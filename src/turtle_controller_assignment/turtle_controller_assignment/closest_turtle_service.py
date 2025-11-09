#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from custom_interfaces.srv import FindClosestTurtle
from turtlesim.msg import Pose


# Service: returns closest turtle to turtle1
class ClosestTurtleService(Node):
    def __init__(self):
        super().__init__('closest_turtle_service')
        
        # Store turtle poses: name -> Pose
        self.turtle_poses = {}
        
        # Create service
        self.create_service(
            FindClosestTurtle,
            '/find_closest_turtle',
            self.on_find_closest
        )
        
        # Timer to scan for new turtles
        self.create_timer(2.0, self._scan_topics)
        
        # Subscribe to turtle1 by default
        self._ensure_subscription('turtle1')
        
        self.get_logger().info('🎯 Closest Turtle Service started')
    
    # Ensure subscription to a turtle's pose if not already subscribed
    def _ensure_subscription(self, name: str):
        """Subscribe to a turtle's pose topic if not already subscribed"""
        if name in self.turtle_poses:
            return
        
        self.turtle_poses[name] = None
        self.create_subscription(
            Pose,
            f'/{name}/pose',
            lambda msg, n=name: self._on_pose(n, msg),
            10
        )
        self.get_logger().info(f'📡 Now tracking pose of: {name}')
    
    def _on_pose(self, name: str, msg: Pose):
        """Callback for pose updates"""
        self.turtle_poses[name] = msg
    
    # Scan topics and manage subscriptions to active turtles
    def _scan_topics(self):
        """Scan for new turtle pose topics and subscribe to them"""
        topic_list = self.get_topic_names_and_types()
        
        # Find all turtle pose topics with active publishers
        active_topics = set()
        for topic_name, topic_types in topic_list:
            if topic_name.endswith('/pose') and 'turtlesim/msg/Pose' in topic_types:
                # Check if this topic has any publishers
                publisher_count = self.count_publishers(topic_name)
                
                if publisher_count > 0:
                    # Extract turtle name from topic like /turtle1/pose
                    parts = topic_name.split('/')
                    if len(parts) >= 2:
                        turtle_name = parts[1]
                        if turtle_name:
                            active_topics.add(turtle_name)
                            if turtle_name not in self.turtle_poses:
                                self._ensure_subscription(turtle_name)
        
        # Remove turtles that no longer exist or have no publishers
        for turtle_name in list(self.turtle_poses.keys()):
            if turtle_name not in active_topics:
                self.get_logger().info(f'🗑️  Removing disappeared turtle: {turtle_name}')
                del self.turtle_poses[turtle_name]
    
    # Service callback: compute closest turtle to turtle1
    def on_find_closest(self, request, response):
        """Service callback to find the closest turtle to turtle1"""
        
        # Scan topics on-demand to ensure fresh state
        self._scan_topics()
        
        # Check if turtle1 exists and has pose data
        if 'turtle1' not in self.turtle_poses or self.turtle_poses['turtle1'] is None:
            response.success = False
            response.message = 'turtle1 not found or no pose data'
            response.closest_turtle_name = ''
            response.distance = 0.0
            return response
        
        turtle1_pose = self.turtle_poses['turtle1']
        
        # Find the closest turtle (excluding turtle1 itself)
        min_distance = float('inf')
        closest_name = ''
        
        for name, pose in self.turtle_poses.items():
            if name == 'turtle1' or pose is None:
                continue
            
            # Calculate Euclidean distance
            dx = pose.x - turtle1_pose.x
            dy = pose.y - turtle1_pose.y
            distance = math.sqrt(dx * dx + dy * dy)
            
            if distance < min_distance:
                min_distance = distance
                closest_name = name
        
        # Prepare response
        if closest_name:
            response.success = True
            response.closest_turtle_name = closest_name
            response.distance = min_distance
            response.message = f'Closest turtle is {closest_name} at distance {min_distance:.2f}'
            self.get_logger().info(f'✅ {response.message}')
        else:
            response.success = False
            response.closest_turtle_name = ''
            response.distance = 0.0
            response.message = 'No other turtles found'
            self.get_logger().warning('⚠️  No other turtles to compare')
        
        return response


def main():
    rclpy.init()
    node = ClosestTurtleService()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
