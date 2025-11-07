#!/usr/bin/env python3
"""
Test movement control logic for turtle1 pursuing a moving target
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import time


class MovementTester(Node):
    def __init__(self):
        super().__init__('movement_tester')
        
        self.turtle1_pose = None
        self.target_pose = None
        
        # Subscribe to poses
        self.create_subscription(Pose, '/turtle1/pose', self.turtle1_callback, 10)
        
        # Publisher for turtle1
        self.cmd_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        
        # Control parameters - TEST DIFFERENT VALUES
        self.linear_gain = 2.0
        self.angular_gain = 4.0
        
        self.get_logger().info('Movement Tester Started')
        self.get_logger().info(f'Linear gain: {self.linear_gain}, Angular gain: {self.angular_gain}')
    
    def turtle1_callback(self, msg):
        self.turtle1_pose = msg
    
    def set_target(self, x, y):
        """Set a static target for testing"""
        self.target_pose = Pose()
        self.target_pose.x = x
        self.target_pose.y = y
        self.get_logger().info(f'Target set to ({x}, {y})')
    
    def get_distance(self):
        if not self.turtle1_pose or not self.target_pose:
            return float('inf')
        dx = self.target_pose.x - self.turtle1_pose.x
        dy = self.target_pose.y - self.turtle1_pose.y
        return math.sqrt(dx * dx + dy * dy)
    
    def get_angle(self):
        if not self.turtle1_pose or not self.target_pose:
            return 0.0
        dx = self.target_pose.x - self.turtle1_pose.x
        dy = self.target_pose.y - self.turtle1_pose.y
        desired_angle = math.atan2(dy, dx)
        
        angle_diff = desired_angle - self.turtle1_pose.theta
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi
        
        return angle_diff
    
    def move_to_target(self, max_iterations=100):
        """Move turtle1 to target and log behavior"""
        if not self.target_pose:
            self.get_logger().error('No target set!')
            return
        
        iteration = 0
        while iteration < max_iterations:
            distance = self.get_distance()
            angle = self.get_angle()
            
            if iteration % 10 == 0:  # Log every 10 iterations
                self.get_logger().info(
                    f'Iter {iteration}: distance={distance:.3f}, angle={angle:.3f} rad ({math.degrees(angle):.1f}°)'
                )
            
            if distance < 0.5:
                self.get_logger().info('✅ Target reached!')
                # Stop
                msg = Twist()
                self.cmd_pub.publish(msg)
                return True
            
            # Apply proportional control
            msg = Twist()
            linear_vel = self.linear_gain * distance
            angular_vel = self.angular_gain * angle
            
            # Log velocities
            if iteration % 10 == 0:
                self.get_logger().info(
                    f'  Commanded: linear.x={linear_vel:.3f}, angular.z={angular_vel:.3f}'
                )
            
            msg.linear.x = linear_vel
            msg.angular.z = angular_vel
            self.cmd_pub.publish(msg)
            
            time.sleep(0.1)
            iteration += 1
        
        self.get_logger().warning('❌ Max iterations reached without reaching target')
        msg = Twist()
        self.cmd_pub.publish(msg)
        return False


def main():
    rclpy.init()
    tester = Node('test_node')
    
    # Wait for user to start turtlesim
    print("\n=== Movement Control Test ===")
    print("1. Start turtlesim: ros2 run turtlesim turtlesim_node")
    print("2. This test will move turtle1 to position (8, 8)")
    print("3. Watch the behavior and check logs")
    input("\nPress Enter when turtlesim is ready...")
    
    tester.destroy_node()
    
    # Create movement tester
    tester = MovementTester()
    
    # Wait for pose
    print("Waiting for turtle1 pose...")
    while tester.turtle1_pose is None:
        rclpy.spin_once(tester, timeout_sec=0.1)
    
    print(f"Current position: ({tester.turtle1_pose.x:.2f}, {tester.turtle1_pose.y:.2f})")
    print(f"Current heading: {math.degrees(tester.turtle1_pose.theta):.1f}°")
    
    # Set target
    tester.set_target(8.0, 8.0)
    
    # Move to target
    print("\nStarting movement...")
    success = tester.move_to_target(max_iterations=200)
    
    if success:
        print("\n✅ SUCCESS: Turtle reached target")
    else:
        print("\n❌ FAILED: Turtle did not reach target")
    
    tester.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
