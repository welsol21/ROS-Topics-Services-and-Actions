#!/usr/bin/env python3
"""
Task 5: Turtle Collection Action Client

Sends a goal to the turtle collection action server to collect all turtles.
Waits for the action server and required number of turtles before sending the goal.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from custom_interfaces.action import MoveTurtle


class TurtleCollectionClient(Node):
    def __init__(self):
        super().__init__('turtle_collection_client')
        
        # Create action client
        self.action_client = ActionClient(
            self,
            MoveTurtle,
            'collect_turtles'
        )
        
        self.get_logger().info('🎯 Turtle Collection Action Client started')
        self.get_logger().info('   Waiting for action server...')
        
        # Wait for action server
        if not self.action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('❌ Action server not available!')
            return
        
        self.get_logger().info('   ✅ Action server available')
        self.get_logger().info('   Waiting for turtles to spawn...')
        
        # Wait for at least 10 turtles (1 turtle1 + 10 bots)
        self.wait_for_turtles(min_turtles=10)
        
        # Send goal
        self.send_goal()
    
    def wait_for_turtles(self, min_turtles=10, timeout_sec=120.0):
        """Wait until at least min_turtles are present in the simulation"""
        import time
        start_time = time.time()
        
        while True:
            # Count pose topics (excluding turtle1)
            topic_list = self.get_topic_names_and_types()
            bot_count = 0
            
            for topic_name, topic_types in topic_list:
                if topic_name.endswith('/pose') and 'turtlesim/msg/Pose' in topic_types:
                    parts = topic_name.split('/')
                    if len(parts) >= 2:
                        turtle_name = parts[1]
                        if turtle_name and turtle_name != 'turtle1':
                            bot_count += 1
            
            if bot_count >= min_turtles:
                self.get_logger().info(f'   ✅ Found {bot_count} bots, ready to start collection')
                return
            
            if time.time() - start_time > timeout_sec:
                self.get_logger().warning(f'⚠️  Timeout waiting for turtles (found {bot_count})')
                return
            
            time.sleep(1.0)
    
    def send_goal(self):
        """Send goal to collect all turtles"""
        goal_msg = MoveTurtle.Goal()
        # MoveTurtle.Goal has x and y fields, but we don't use them for collection
        goal_msg.x = 0.0
        goal_msg.y = 0.0
        
        self.get_logger().info('📤 Sending goal to collect all turtles...')
        
        send_goal_future = self.action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        """Handle goal acceptance/rejection"""
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('❌ Goal rejected by server')
            return
        
        self.get_logger().info('✅ Goal accepted by server')
        
        # Wait for result
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)
    
    def feedback_callback(self, feedback_msg):
        """Handle feedback from action server"""
        feedback = feedback_msg.feedback
        self.get_logger().info(f'📊 Distance to target: {feedback.distance:.2f}')
    
    def result_callback(self, future):
        """Handle final result"""
        result = future.result().result
        
        if result.success:
            self.get_logger().info('🎉 Collection completed successfully!')
        else:
            self.get_logger().error('❌ Collection failed')
        
        # Shutdown after completion
        rclpy.shutdown()


def main():
    rclpy.init()
    client = TurtleCollectionClient()
    
    try:
        rclpy.spin(client)
    except KeyboardInterrupt:
        pass
    finally:
        client.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
