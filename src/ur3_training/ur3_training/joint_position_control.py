#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import sys
import select
import tty
import termios
import math
from tf2_ros import TransformListener, Buffer
from geometry_msgs.msg import TransformStamped

class UR3JointPositionControl(Node):
    def __init__(self):
        super().__init__('ur3_joint_position_control')
        
        # UR3 joint names in order
        self.joint_names = [
            'shoulder_pan_joint',
            'shoulder_lift_joint', 
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint'
        ]
        
        # Initialize joint positions array
        self.current_joint_positions = [0.0] * 6
        
        # Create publisher for joint commands
        self.joint_command_pub = self.create_publisher(
            Float64MultiArray, 
            '/forward_position_controller/commands', 
            10
        )
        
        # Create subscriber for joint states
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        # TF buffer and listener for end-effector pose
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.get_logger().info('UR3 Joint Position Control Node initialized')
        self.get_logger().info('Make sure the UR3 simulation is running with forward_position_controller activated')

    def joint_state_callback(self, msg):
        """Callback for joint state updates"""
        # Update current joint positions
        for i, name in enumerate(msg.name):
            if name in self.joint_names:
                idx = self.joint_names.index(name)
                self.current_joint_positions[idx] = msg.position[i]

    def get_user_input(self):
        """Get joint selection and movement amount from user"""
        # Display current joint positions
        print("\n" + "="*50)
        print("UR3 Joint Position Control")
        print("="*50)
        print("Current joint positions:")
        for i, (name, pos) in enumerate(zip(self.joint_names, self.current_joint_positions)):
            print(f"  {i+1}. {name}: {pos:.3f} rad")
        
        print("\nSelect joint to move:")
        for i, name in enumerate(self.joint_names):
            print(f"  {i+1}. {name}")
        print("  0. Exit")
        
        try:
            joint_selection = int(input("\nEnter joint number (0-6): "))
            if joint_selection == 0:
                return None, None
            if joint_selection < 1 or joint_selection > 6:
                print("Invalid joint selection. Please enter a number between 1-6.")
                return self.get_user_input()
                
            joint_idx = joint_selection - 1
            movement = float(input(f"Enter movement amount for {self.joint_names[joint_idx]} (radians): "))
            
            return joint_idx, movement
        except ValueError:
            print("Invalid input. Please enter numeric values.")
            return self.get_user_input()

    def move_joint(self, joint_idx, movement):
        """Move specified joint by given amount"""
        # Calculate target position (current + movement)
        target_position = self.current_joint_positions[joint_idx] + movement
        
        # Create command message
        cmd_msg = Float64MultiArray()
        cmd_msg.data = self.current_joint_positions.copy()
        cmd_msg.data[joint_idx] = target_position
        
        # Send command
        self.joint_command_pub.publish(cmd_msg)
        self.get_logger().info(f'Moving {self.joint_names[joint_idx]} by {movement:.3f} rad to {target_position:.3f} rad')
        
        # Wait for movement to complete (simple approach)
        self.wait_for_movement_completion(joint_idx, target_position)
        
        # Print results
        self.print_results(joint_idx, target_position)

    def wait_for_movement_completion(self, joint_idx, target_position, timeout=10.0):
        """Wait for joint to reach target position"""
        start_time = self.get_clock().now()
        tolerance = 0.01  # 10 milliradians
        
        while (self.get_clock().now() - start_time).nanoseconds / 1e9 < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            error = abs(self.current_joint_positions[joint_idx] - target_position)
            if error < tolerance:
                self.get_logger().info('Movement completed')
                return
                
        self.get_logger().warn('Movement timeout reached')

    def get_end_effector_pose(self):
        """Get end-effector pose relative to base"""
        try:
            trans = self.tf_buffer.lookup_transform('base_link', 'tool0', rclpy.time.Time())
            return trans
        except Exception as e:
            self.get_logger().warn(f'Could not get end-effector pose: {e}')
            return None

    def print_results(self, joint_idx, target_position):
        """Print final position, error, and end-effector pose"""
        final_position = self.current_joint_positions[joint_idx]
        error = abs(final_position - target_position)
        
        print(f"\n--- Movement Results ---")
        print(f"Joint: {self.joint_names[joint_idx]}")
        print(f"Target position: {target_position:.4f} rad")
        print(f"Final position: {final_position:.4f} rad")
        print(f"Position error: {error:.4f} rad")
        
        # Get end-effector pose
        ee_pose = self.get_end_effector_pose()
        if ee_pose:
            print(f"End-effector position: x={ee_pose.transform.translation.x:.4f}, "
                  f"y={ee_pose.transform.translation.y:.4f}, "
                  f"z={ee_pose.transform.translation.z:.4f}")
            print(f"End-effector orientation: x={ee_pose.transform.rotation.x:.4f}, "
                  f"y={ee_pose.transform.rotation.y:.4f}, "
                  f"z={ee_pose.transform.rotation.z:.4f}, "
                  f"w={ee_pose.transform.rotation.w:.4f}")
        else:
            print("End-effector pose: Not available")

    def run_control_loop(self):
        """Main control loop"""
        try:
            while rclpy.ok():
                joint_idx, movement = self.get_user_input()
                
                if joint_idx is None:  # User wants to exit
                    print("Exiting...")
                    break
                
                # Move joint
                self.move_joint(joint_idx, movement)
                
        except KeyboardInterrupt:
            pass

def main(args=None):
    rclpy.init(args=args)
    
    node = UR3JointPositionControl()
    
    # Use a separate thread for user input to avoid blocking
    import threading
    control_thread = threading.Thread(target=node.run_control_loop)
    control_thread.daemon = True
    control_thread.start()
    
    # Spin the node in the main thread
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()