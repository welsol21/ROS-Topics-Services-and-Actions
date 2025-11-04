#!/usr/bin/env python3
"""
Task 2: Pen Control Client with turtle movement demonstration
"""

import rclpy
from rclpy.node import Node
from turtlesim.srv import SetPen
from geometry_msgs.msg import Twist
import time

class PenControlClient(Node):
    """
    Client node for demonstrating pen control with turtle movement
    """
    
    def __init__(self):
        super().__init__('pen_control_client')
        
        # Client for set_pen service
        self.pen_client = self.create_client(SetPen, 'turtle1/set_pen')
        
        # Publisher for turtle movement
        self.cmd_vel_pub = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
        
        # Wait for the service to be available
        while not self.pen_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('SetPen service not available, waiting...')
        
        self.get_logger().info('Pen control client ready!')
    
    def set_pen(self, r=255, g=255, b=255, width=4, off=False):
        """Set pen properties"""
        request = SetPen.Request()
        request.r = r
        request.g = g
        request.b = b
        request.width = width
        request.off = off
        
        future = self.pen_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result() is not None
    
    def move_turtle(self, linear_x=1.0, angular_z=0.0, duration=2.0):
        """Move turtle for demonstration"""
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        
        start_time = time.time()
        while time.time() - start_time < duration:
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.1)
        
        # Stop the turtle
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    
    demo = PenControlClient()
    
    print("\nPen Control Demonstration")
    print("=" * 30)
    
    # Demo 1: Default pen (white)
    print("\n1. Default white pen")
    demo.set_pen(r=255, g=255, b=255, width=4, off=False)
    demo.move_turtle(linear_x=1.0, angular_z=1.0, duration=3.0)
    time.sleep(1)
    
    # Demo 2: Red pen
    print("2. Red pen")
    demo.set_pen(r=255, g=0, b=0, width=6, off=False)
    demo.move_turtle(linear_x=1.0, angular_z=-1.0, duration=3.0)
    time.sleep(1)
    
    # Demo 3: Green pen
    print("3. Green pen")
    demo.set_pen(r=0, g=255, b=0, width=3, off=False)
    demo.move_turtle(linear_x=0.5, angular_z=2.0, duration=3.0)
    time.sleep(1)
    
    # Demo 4: Pen off
    print("4. Pen OFF - no drawing")
    demo.set_pen(off=True)
    demo.move_turtle(linear_x=1.0, angular_z=0.0, duration=2.0)
    time.sleep(1)
    
    # Demo 5: Blue pen
    print("5. Blue pen - drawing resumed")
    demo.set_pen(r=0, g=0, b=255, width=8, off=False)
    demo.move_turtle(linear_x=1.0, angular_z=1.5, duration=3.0)
    
    print("\nPen control demonstration completed!")
    
    demo.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()