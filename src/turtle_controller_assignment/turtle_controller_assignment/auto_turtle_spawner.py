#!/usr/bin/env python3
"""
Task 3: Auto Turtle Spawner - Fixed Spawn Limiting
"""

import rclpy
from rclpy.node import Node
import math
import random
import time

from turtlesim.srv import Spawn, SetPen
from std_srvs.srv import Trigger
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class AutoTurtleSpawner(Node):
    
    def __init__(self):
        super().__init__('auto_turtle_spawner')
        
        self.spawn_client = self.create_client(Spawn, 'spawn')
        self.monitor_client = self.create_client(Trigger, '/monitor_turtles')
        
        self.active_turtles_pub = self.create_publisher(String, 'active_turtles', 10)
        
        # Track OUR turtles
        self.our_turtles = set()  # Turtles we successfully spawned
        self.pending_spawns = set()  # Turtles that are currently being spawned
        self.turtle_publishers = {}
        
        self.turtle_counter = 1
        self.max_additional_turtles = 10
        
        self.spawn_client.wait_for_service()
        
        # Timers
        self.spawn_timer = self.create_timer(3.0, self.manage_turtles)  # Check more frequently
        self.monitor_timer = self.create_timer(4.0, self.check_turtle_health)
        self.info_timer = self.create_timer(2.0, self.publish_active_turtles)
        
        self.get_logger().info('🔄 Auto Turtle Spawner - Fixed spawn limiting')
    
    def get_current_additional_count(self):
        """Get current count of OUR additional turtles (excluding turtle1)"""
        return len([name for name in self.our_turtles if name != 'turtle1'])
    
    def get_pending_count(self):
        """Get count of pending spawns"""
        return len(self.pending_spawns)
    
    def get_total_managed_count(self):
        """Get total managed turtles (spawned + pending)"""
        return self.get_current_additional_count() + self.get_pending_count()
    
    def manage_turtles(self):
        """Manage turtle count - spawn new ones if needed"""
        current_count = self.get_current_additional_count()
        pending_count = self.get_pending_count()
        total_managed = self.get_total_managed_count()
        
        self.get_logger().info(f'📊 Current: {current_count}, Pending: {pending_count}, Total: {total_managed}/{self.max_additional_turtles}')
        
        if total_managed < self.max_additional_turtles:
            needed = self.max_additional_turtles - total_managed
            self.get_logger().info(f'🎯 Spawning {needed} turtles')
            
            # Spawn only as many as needed, one at a time
            for i in range(min(needed, 2)):  # Limit to 2 at a time to avoid overload
                self.spawn_single_turtle()
        else:
            self.get_logger().info(f'✅ Target reached: {current_count}/{self.max_additional_turtles} turtles')
    
    def spawn_single_turtle(self):
        """Spawn a single turtle and track it as pending"""
        turtle_name = f"auto_turtle_{self.turtle_counter}"
        self.turtle_counter += 1
        
        # Add to pending spawns
        self.pending_spawns.add(turtle_name)
        
        x, y, theta, radius, direction = self.generate_spawn_params()
        
        self.get_logger().info(f'🐢 Spawning: {turtle_name} at ({x:.1f}, {y:.1f})')
        
        request = Spawn.Request()
        request.x = x
        request.y = y
        request.theta = theta
        request.name = turtle_name
        
        future = self.spawn_client.call_async(request)
        future.add_done_callback(
            lambda f, name=turtle_name, r=radius, d=direction: 
            self.on_spawn_complete(f, name, r, d)
        )
    
    def on_spawn_complete(self, future, turtle_name, radius, direction):
        """Handle spawn completion"""
        try:
            response = future.result()
            spawned_name = response.name
            
            # Remove from pending and add to our turtles
            if turtle_name in self.pending_spawns:
                self.pending_spawns.remove(turtle_name)
            
            self.our_turtles.add(spawned_name)
            
            # Setup turtle behavior
            self.setup_turtle_behavior(spawned_name, radius, direction)
            
            current_count = self.get_current_additional_count()
            self.get_logger().info(f'✅ {spawned_name} ready! Total: {current_count}/{self.max_additional_turtles}')
            
        except Exception as e:
            self.get_logger().error(f'❌ Failed to spawn {turtle_name}: {e}')
            # Remove from pending if spawn failed
            if turtle_name in self.pending_spawns:
                self.pending_spawns.remove(turtle_name)
    
    def check_turtle_health(self):
        """Check for removed turtles"""
        if not self.monitor_client.wait_for_service(timeout_sec=1.0):
            return
        
        request = Trigger.Request()
        future = self.monitor_client.call_async(request)
        future.add_done_callback(self.on_monitor_response)
    
    def on_monitor_response(self, future):
        """Handle monitor service response"""
        try:
            response = future.result()
            
            if response.success and "MANAGED_REMOVED:" in response.message:
                removed_section = response.message.split("MANAGED_REMOVED:")[1]
                removed_turtles = [name.strip() for name in removed_section.split(",")]
                
                for turtle_name in removed_turtles:
                    if turtle_name in self.our_turtles and turtle_name != 'turtle1':
                        self.get_logger().warning(f'🔄 Turtle {turtle_name} removed')
                        self.cleanup_turtle(turtle_name)
                        
        except Exception as e:
            self.get_logger().error(f'Monitor error: {e}')
    
    def cleanup_turtle(self, turtle_name):
        """Clean up a removed turtle"""
        if turtle_name in self.our_turtles:
            self.our_turtles.remove(turtle_name)
        
        if turtle_name in self.turtle_publishers:
            del self.turtle_publishers[turtle_name]
    
    def setup_turtle_behavior(self, turtle_name, radius, direction):
        """Setup pen and movement for a turtle"""
        self.set_pen_off(turtle_name)
        self.start_circular_movement(turtle_name, radius, direction)
    
    def set_pen_off(self, turtle_name):
        """Turn off pen"""
        try:
            pen_client = self.create_client(SetPen, f'/{turtle_name}/set_pen')
            if pen_client.wait_for_service(timeout_sec=1.0):
                request = SetPen.Request()
                request.off = True
                pen_client.call_async(request)
        except:
            pass
    
    def start_circular_movement(self, turtle_name, radius, direction):
        """Start circular movement"""
        vel_publisher = self.create_publisher(Twist, f'/{turtle_name}/cmd_vel', 10)
        self.turtle_publishers[turtle_name] = vel_publisher
        
        twist = Twist()
        twist.linear.x = 1.0
        twist.angular.z = direction * (1.0 / radius)
        
        def move_callback():
            if turtle_name in self.turtle_publishers:
                self.turtle_publishers[turtle_name].publish(twist)
        
        self.create_timer(0.1, move_callback)
    
    def generate_spawn_params(self):
        """Generate spawn parameters"""
        r = random.uniform(1.0, 5.0)
        phi = random.uniform(0.0, 2 * math.pi)
        
        x_prime = random.uniform(-5.5 + r, 5.5 - r)
        y_prime = random.uniform(-5.5 + r, 5.5 - r)
        
        x = 5.5 + x_prime + r * math.cos(phi)
        y = 5.5 + y_prime + r * math.sin(phi)
        
        x = max(0.5, min(10.5, x))
        y = max(0.5, min(10.5, y))
        
        direction = random.choice([-1, 1])
        theta = (phi + direction * math.pi / 2) % (2 * math.pi)
        
        return x, y, theta, r, direction
    
    def publish_active_turtles(self):
        """Publish active turtles list"""
        msg = String()
        msg.data = ','.join(sorted(self.our_turtles))
        self.active_turtles_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    spawner = AutoTurtleSpawner()
    rclpy.spin(spawner)
    spawner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()