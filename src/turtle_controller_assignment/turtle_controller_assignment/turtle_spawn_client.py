#!/usr/bin/env python3
"""
Task 1: Turtle Spawn Client with real Turtle Manager integration
"""

import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn
from std_srvs.srv import Trigger

class TurtleSpawnClient(Node):
    """
    Client node that requests unique names from Turtle Manager
    """
    
    def __init__(self):
        super().__init__('turtle_spawn_client')
        
        # Client for spawn service
        self.spawn_client = self.create_client(Spawn, 'spawn')
        
        # Client for name manager service
        self.name_client = self.create_client(Trigger, 'generate_unique_name')
        
        # Wait for services to be available
        self.wait_for_services()
        
        self.get_logger().info('Turtle spawn client ready!')
    
    def wait_for_services(self):
        """Wait for both spawn and name manager services"""
        self.get_logger().info('Waiting for spawn service...')
        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Spawn service not available, waiting...')
        
        self.get_logger().info('Waiting for name manager service...')
        while not self.name_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Name manager service not available, waiting...')
    
    def get_unique_name(self):
        """Request a unique name from Turtle Manager"""
        request = Trigger.Request()
        future = self.name_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            response = future.result()
            if response.success:
                return response.message
            else:
                self.get_logger().error('Failed to get unique name from manager')
        else:
            self.get_logger().error('Service call failed to name manager')
        
        return None
    
    def spawn_turtle(self, x, y, theta=0.0, name=None):
        """
        Spawn a turtle with unique name from manager
        """
        # Get unique name from manager
        if name is None:
            name = self.get_unique_name()
            if name is None:
                self.get_logger().error('Could not get unique name, aborting spawn')
                return None
        
        # Prepare spawn request
        request = Spawn.Request()
        request.x = x
        request.y = y
        request.theta = theta
        request.name = name
        
        self.get_logger().info(f'Spawning turtle "{name}" at ({x}, {y})')
        future = self.spawn_client.call_async(request)
        future.add_done_callback(lambda f: self.spawn_callback(f, name))
        return future
    
    def spawn_callback(self, future, turtle_name):
        """Callback when spawn service responds"""
        try:
            response = future.result()
            self.get_logger().info(f'✅ Turtle spawned successfully: {response.name}')
        except Exception as e:
            self.get_logger().error(f'❌ Failed to spawn turtle "{turtle_name}": {e}')

def main(args=None):
    rclpy.init(args=args)
    
    # Create the spawn client
    spawn_client = TurtleSpawnClient()
    
    # Spawn multiple turtles to test unique names
    for i in range(3):
        future = spawn_client.spawn_turtle(
            x=2.0 + i, 
            y=3.0 + i, 
            name=None  # Let manager generate name
        )
        if future:
            rclpy.spin_until_future_complete(spawn_client, future)
    
    # Cleanup
    spawn_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()