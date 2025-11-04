#!/usr/bin/env python3
"""
Task 3: Auto Turtle Spawner Node
Spawns turtles at random coordinates every 5 seconds with circular movement
"""

import rclpy
from rclpy.node import Node
import math
import random
import time
from threading import Lock

from turtlesim.srv import Spawn, Kill, SetPen
from std_srvs.srv import Trigger
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from std_msgs.msg import String

class AutoTurtleSpawner(Node):
    """
    Node that automatically spawns turtles with circular movement
    """
    
    def __init__(self):
        super().__init__('auto_turtle_spawner')
        
        # Service clients
        self.spawn_client = self.create_client(Spawn, 'spawn')
        self.kill_client = self.create_client(Kill, 'kill')
        self.name_client = self.create_client(Trigger, 'generate_unique_name')
        
        # Publisher for active turtles information
        self.active_turtles_pub = self.create_publisher(String, 'active_turtles', 10)
        
        # Dictionary to track spawned turtles and their publishers
        self.turtles = {}  # {turtle_name: {'publisher': pub, 'radius': r, 'direction': sign, 'pen_client': client}}
        self.turtle_poses = {}  # {turtle_name: pose}
        self.lock = Lock()
        
        self.services_ready = False
        
        # Timer for checking services and spawning
        self.main_timer = self.create_timer(1.0, self.main_callback)
        
        self.get_logger().info('Auto Turtle Spawner starting...')
    
    def check_services_ready(self):
        """Check if all required services are ready"""
        spawn_ready = self.spawn_client.wait_for_service(timeout_sec=0.1)
        kill_ready = self.kill_client.wait_for_service(timeout_sec=0.1)
        name_ready = self.name_client.wait_for_service(timeout_sec=0.1)
        
        return spawn_ready and kill_ready and name_ready
    
    def main_callback(self):
        """Main callback that handles both service checking and spawning"""
        # Check if services are ready
        if not self.services_ready:
            if self.check_services_ready():
                self.services_ready = True
                self.get_logger().info('✅ All services ready! Starting auto-spawner...')
            else:
                self.get_logger().info('⏳ Waiting for services... (run turtle_name_manager in another terminal)')
                return
        
        # If services are ready, proceed with spawning logic
        self.try_spawn_turtle()
        self.publish_active_turtles()
    
    def try_spawn_turtle(self):
        """Attempt to spawn a turtle if conditions are met"""
        # Spawn every 5 seconds
        current_time = time.time()
        if hasattr(self, 'last_spawn_time'):
            if current_time - self.last_spawn_time < 5.0:
                return
        else:
            self.last_spawn_time = current_time
            return
        
        with self.lock:
            # Check if we have less than 10 additional turtles (excluding turtle1)
            additional_turtles_count = len([name for name in self.turtles.keys() if name != 'turtle1'])
            
            if additional_turtles_count >= 10:
                self.get_logger().info(f'Maximum additional turtles ({additional_turtles_count}/10) reached')
                self.last_spawn_time = current_time
                return
            
            # Get unique name
            turtle_name = self.get_unique_name()
            if not turtle_name:
                self.get_logger().error('Could not get unique name, will retry...')
                self.last_spawn_time = current_time
                return
            
            # Generate random spawn parameters
            x, y, theta, radius, direction = self.generate_random_spawn_params()
            
            self.get_logger().info(f'🔄 Attempting to spawn {turtle_name} at ({x:.2f}, {y:.2f})')
            
            # Spawn the turtle
            self.spawn_turtle(x, y, theta, turtle_name, radius, direction)
            self.last_spawn_time = current_time
    
    def get_unique_name(self):
        """Request unique name from Turtle Name Manager"""
        request = Trigger.Request()
        future = self.name_client.call_async(request)
        
        # Use non-blocking wait with timeout
        start_time = time.time()
        while time.time() - start_time < 2.0:
            rclpy.spin_once(self, timeout_sec=0.1)
            if future.done():
                break
        
        if future.done() and future.result() is not None:
            response = future.result()
            if response.success:
                return response.message
            else:
                self.get_logger().error('Name manager returned failure')
        else:
            self.get_logger().warn('Name manager service call timeout')
        
        return None
    
    def set_pen_off(self, turtle_name):
        """Turn off the pen for a turtle"""
        try:
            pen_client = self.create_client(SetPen, f'/{turtle_name}/set_pen')
            
            if pen_client.wait_for_service(timeout_sec=1.0):
                request = SetPen.Request()
                request.r = 0
                request.g = 0  
                request.b = 0
                request.width = 0
                request.off = True  # Turn pen OFF
                future = pen_client.call_async(request)
                
                # Wait briefly for response
                start_time = time.time()
                while time.time() - start_time < 1.0:
                    rclpy.spin_once(self, timeout_sec=0.1)
                    if future.done():
                        break
                
                self.get_logger().info(f'✏️ Pen turned OFF for {turtle_name}')
            else:
                self.get_logger().warn(f'Could not set pen for {turtle_name}')
        except Exception as e:
            self.get_logger().warn(f'Error setting pen for {turtle_name}: {e}')
    
    def generate_random_spawn_params(self):
        """
        Generate random spawn parameters according to assignment formula
        Returns: (x, y, theta, radius, direction)
        """
        # Random radius between 1 and 5
        r = random.uniform(1.0, 5.0)
        
        # Random angle for circular path orientation
        phi = random.uniform(0.0, 2 * math.pi)
        
        # Random offsets ensuring turtle stays within bounds
        x_prime = random.uniform(-5.5 + r, 5.5 - r)
        y_prime = random.uniform(-5.5 + r, 5.5 - r)
        
        # Calculate final position
        x = 5.5 + x_prime + r * math.cos(phi)
        y = 5.5 + y_prime + r * math.sin(phi)
        
        # Orientation perpendicular to radius
        direction = random.choice([-1, 1])  # Clockwise or counter-clockwise
        theta = phi + direction * math.pi / 2
        
        # Normalize theta
        theta = theta % (2 * math.pi)
        
        return x, y, theta, r, direction
    
    def spawn_turtle(self, x, y, theta, name, radius, direction):
        """Spawn a turtle and set it moving in a circle"""
        # Create spawn request
        spawn_request = Spawn.Request()
        spawn_request.x = x
        spawn_request.y = y
        spawn_request.theta = theta
        spawn_request.name = name
        
        # Call spawn service
        spawn_future = self.spawn_client.call_async(spawn_request)
        spawn_future.add_done_callback(
            lambda future, tname=name, r=radius, d=direction: 
            self.spawn_complete_callback(future, tname, r, d)
        )
    
    def spawn_complete_callback(self, future, turtle_name, radius, direction):
        """Callback when spawn service completes"""
        try:
            response = future.result()
            actual_name = response.name
            
            self.get_logger().info(f'✅ Successfully spawned turtle: {actual_name}')
            
            # Small delay to ensure turtle is fully initialized
            time.sleep(0.5)
            
            # Turn off the pen for this turtle
            self.set_pen_off(actual_name)
            
            # Create velocity publisher for this turtle
            cmd_vel_topic = f'/{actual_name}/cmd_vel'
            vel_publisher = self.create_publisher(Twist, cmd_vel_topic, 10)
            
            # Store turtle information
            with self.lock:
                self.turtles[actual_name] = {
                    'publisher': vel_publisher,
                    'radius': radius,
                    'direction': direction,
                    'move_timer': None
                }
            
            # Start circular movement
            self.start_circular_movement(actual_name, radius, direction)
            
            # Create pose subscriber for this turtle
            pose_topic = f'/{actual_name}/pose'
            self.create_subscription(
                Pose, 
                pose_topic, 
                lambda msg, name=actual_name: self.pose_callback(msg, name),
                10
            )
            
        except Exception as e:
            self.get_logger().error(f'❌ Failed to spawn turtle {turtle_name}: {e}')
    
    def start_circular_movement(self, turtle_name, radius, direction):
        """Start the turtle moving in a circular path"""
        if turtle_name not in self.turtles:
            return
        
        # Create twist message for circular motion
        twist = Twist()
        twist.linear.x = 1.0  # Constant linear velocity
        twist.angular.z = direction * (1.0 / radius)  # Angular velocity for circular path
        
        # Create timer for continuous movement
        def move_callback():
            if turtle_name in self.turtles:
                self.turtles[turtle_name]['publisher'].publish(twist)
        
        move_timer = self.create_timer(0.1, move_callback)  # 10Hz
        
        # Store the timer
        with self.lock:
            self.turtles[turtle_name]['move_timer'] = move_timer
        
        self.get_logger().info(f'🎯 Started circular movement for {turtle_name}: v={twist.linear.x}, ω={twist.angular.z:.2f}')
    
    def pose_callback(self, msg, turtle_name):
        """Callback for receiving turtle pose updates"""
        with self.lock:
            self.turtle_poses[turtle_name] = msg
    
    def publish_active_turtles(self):
        """Publish information about currently active turtles"""
        if not self.services_ready:
            return
            
        with self.lock:
            active_turtles_list = list(self.turtles.keys())
            additional_count = len([name for name in active_turtles_list if name != 'turtle1'])
            
            active_turtles_str = ','.join(active_turtles_list)
            msg = String()
            msg.data = f"active_turtles:{active_turtles_str}"
            
            self.active_turtles_pub.publish(msg)
            
            # Log periodically (every 10 seconds or when count changes)
            current_time = time.time()
            if not hasattr(self, 'last_log_time') or current_time - self.last_log_time > 10.0:
                self.get_logger().info(f'📊 Active turtles: {additional_count}/10 additional turtles')
                self.last_log_time = current_time

def main(args=None):
    rclpy.init(args=args)
    
    spawner = AutoTurtleSpawner()
    
    try:
        rclpy.spin(spawner)
    except KeyboardInterrupt:
        spawner.get_logger().info('Auto spawner shutting down...')
    finally:
        spawner.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()