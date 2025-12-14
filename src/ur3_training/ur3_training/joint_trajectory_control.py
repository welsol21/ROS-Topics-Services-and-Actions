#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
import math
from tf2_ros import TransformListener, Buffer
from geometry_msgs.msg import TransformStamped

class UR3JointTrajectoryControl(Node):
    def __init__(self):
        super().__init__('ur3_joint_trajectory_control')
        
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
        
        # Create action client for joint trajectory controller
        self._action_client = ActionClient(self, FollowJointTrajectory, '/joint_trajectory_controller/follow_joint_trajectory')
        
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
        
        self.get_logger().info('UR3 Joint Trajectory Control Node initialized')
        self.get_logger().info('Make sure the UR3 simulation is running with joint_trajectory_controller activated')

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
        print("UR3 Joint Trajectory Control")
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

    def send_trajectory_goal(self, joint_idx, movement):
        """Send trajectory goal to move specified joint by given amount"""
        # Calculate target position (current + movement)
        target_positions = self.current_joint_positions.copy()
        target_positions[joint_idx] = target_positions[joint_idx] + movement
        
        # Create goal
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.joint_names
        
        # Create trajectory point
        point = JointTrajectoryPoint()
        point.positions = target_positions
        point.time_from_start.sec = 2  # Movement duration
        
        goal_msg.trajectory.points = [point]
        
        # Wait for action server
        self._action_client.wait_for_server()
        
        # Send goal
        self.get_logger().info(f'Moving {self.joint_names[joint_idx]} by {movement:.3f} rad to {target_positions[joint_idx]:.3f} rad')
        future = self._action_client.send_goal_async(goal_msg)
        
        return future, target_positions[joint_idx]

    def wait_for_trajectory_completion(self, goal_future):
        """Wait for trajectory execution to complete"""
        # Wait for goal acceptance
        rclpy.spin_until_future_complete(self, goal_future, timeout_sec=5.0)
        
        if not goal_future.result():
            self.get_logger().error('Failed to send goal')
            return False
            
        goal_handle = goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected')
            return False
            
        self.get_logger().info('Goal accepted')
        
        # Request result
        result_future = self._action_client._get_result(goal_handle)
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=10.0)
        
        if not result_future.result():
            self.get_logger().error('Failed to get result')
            return False
            
        result = result_future.result()
        if result.code == FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().info('Trajectory execution completed successfully')
            return True
        else:
            self.get_logger().warn(f'Trajectory execution failed with error code: {result.code}')
            return False

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
                
                # Send trajectory goal
                goal_future, target_position = self.send_trajectory_goal(joint_idx, movement)
                
                # Wait for completion
                success = self.wait_for_trajectory_completion(goal_future)
                
                if success:
                    # Print results
                    self.print_results(joint_idx, target_position)
                
        except KeyboardInterrupt:
            pass

def main(args=None):
    rclpy.init(args=args)
    
    node = UR3JointTrajectoryControl()
    
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