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
from std_srvs.srv import Trigger


# Action client: starts collection when bots ≥ threshold
class TurtleCollectionClient(Node):
    def __init__(self):
        super().__init__('turtle_collection_client')
        
        # Create action client
        self.action_client = ActionClient(
            self,
            MoveTurtle,
            'collect_turtles'
        )
        
        self.collection_in_progress = False
        
        # Create client for monitor service to get accurate turtle count
        self.monitor_client = self.create_client(Trigger, '/monitor_turtles')
        
        self.get_logger().info('🎯 Turtle Collection Action Client started')
        self.get_logger().info('   Waiting for action server...')
        
        # Wait for action server
        if not self.action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('❌ Action server not available!')
            return
        
        self.get_logger().info('   ✅ Action server available')
        
        # Create timer to monitor turtle count every second
        self.create_timer(1.0, self.check_turtle_count)
        # Timer to check cancellation condition during an active goal
        self.create_timer(1.0, self.check_cancel_condition)
        # Store goal handle for cancellation
        self.goal_handle = None
    
    # Periodically start collection when enough bots are active
    def check_turtle_count(self):
        """Periodically check turtle count and start collection when 10 bots present"""
        # Skip if collection already in progress
        if self.collection_in_progress:
            return
        
        # Use monitor service to get accurate count of active turtles
        if not self.monitor_client.service_is_ready():
            return
        
        request = Trigger.Request()
        future = self.monitor_client.call_async(request)
        future.add_done_callback(self._handle_monitor_response)
    
    def _handle_monitor_response(self, future):
        """Handle response from monitor service"""
        try:
            response = future.result()
            if response.success:
                # Parse message: "ACTIVE:turtle1,bot_1,bot_2;REMOVED:"
                parts = response.message.split(';')
                active_part = parts[0].replace('ACTIVE:', '')
                active_turtles = [t.strip() for t in active_part.split(',') if t.strip()]
                
                # Count bots (excluding turtle1)
                bot_count = sum(1 for t in active_turtles if t != 'turtle1')
                
                # Start collection when 3 or more bots present
                if bot_count >= 3:
                    self.get_logger().info(f'Starting collection with {bot_count} active bots (>=3)...')
                    self.send_goal()
        except Exception as e:
            self.get_logger().error(f'Error checking turtle count: {e}')
    
    def send_goal(self):
        """Send goal to collect all turtles"""
        self.collection_in_progress = True
        
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
    
    # Cancel goal if environment grows beyond threshold during execution
    def check_cancel_condition(self):
        """Cancel goal if bots >= 10 during collection"""
        if not self.collection_in_progress or not self.monitor_client.service_is_ready():
            return
        request = Trigger.Request()
        future = self.monitor_client.call_async(request)
        def _cancel_if_needed(fut):
            try:
                response = fut.result()
                if response and response.success:
                    parts = response.message.split(';')
                    active_part = parts[0].replace('ACTIVE:', '')
                    active_turtles = [t.strip() for t in active_part.split(',') if t.strip()]
                    bot_count = sum(1 for t in active_turtles if t != 'turtle1')
                    # Log current bots left to collect (for clarity, e.g., 9)
                    self.get_logger().info(f'Bots left to collect: {bot_count}')
                    if bot_count >= 10 and self.goal_handle:
                        self.get_logger().info(f'Canceling goal: active bots={bot_count} (>=10)')
                        self.goal_handle.cancel_goal_async()
            except Exception as e:
                self.get_logger().error(f'Error checking cancel condition: {e}')
        future.add_done_callback(_cancel_if_needed)
    
    def goal_response_callback(self, future):
        """Handle goal acceptance/rejection"""
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('❌ Goal rejected by server')
            return
        
        self.get_logger().info('✅ Goal accepted by server')
        # Store handle for cancellation (required to meet cancel rule)
        self.goal_handle = goal_handle
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
        
        # Mark collection as complete, ready for next cycle
        self.collection_in_progress = False
        self.goal_handle = None
        self.get_logger().info('⏳ Ready for next collection')


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
