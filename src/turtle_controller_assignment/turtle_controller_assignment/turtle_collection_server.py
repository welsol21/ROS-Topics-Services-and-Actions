#!/usr/bin/env python3
"""
Task 5: Turtle Collection Action Server

Action server that moves turtle1 to collect all other turtles by:
1. Finding the closest turtle using /find_closest_turtle service
2. Moving turtle1 towards it using proportional control (linear gain=2, angular gain=4)
3. Removing the turtle when reached (distance < 0.5)
4. Repeating until no turtles remain
5. Publishing feedback with current target and distance
6. Supporting goal cancellation with simulation reset
"""

import math
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import Kill, TeleportAbsolute
from std_srvs.srv import Trigger, Empty
from custom_interfaces.action import MoveTurtle
from custom_interfaces.srv import FindClosestTurtle


# Action server: collects turtles using proportional control
class TurtleCollectionServer(Node):
    def __init__(self):
        super().__init__('turtle_collection_server')
        
        # Store poses of all turtles (up to 11: turtle1 + 10 bots)
        self.turtle_poses = {}
        self.max_turtles = 11
        
        # Track if a goal is currently executing to prevent parallel goals
        self.goal_executing = False
        # Current locked target name; only updated when collected or lost
        self.current_target_name = None
        
        # Control gains as specified in assignment
        self.linear_gain = 2.0
        self.angular_gain = 4.0
        self.collection_distance = 0.5
        # Lost target re-checks (to handle turtles going off-screen briefly)
        self.lost_target_retries = 4
        self.lost_target_check_delay = 0.25
        
        # Publishers and subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        
        # Subscribe to turtle1 pose
        self.create_subscription(Pose, '/turtle1/pose', self._turtle1_pose_callback, 10)
        
        # Timer to discover and subscribe to all turtle poses
        self.create_timer(2.0, self._scan_and_subscribe_turtles)
        
        # Service clients
        self.closest_turtle_client = self.create_client(
            FindClosestTurtle,
            '/find_closest_turtle'
        )
        self.kill_client = self.create_client(Kill, '/kill')
        self.reset_client = self.create_client(Empty, '/reset')
        self.teleport_client = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute')
        
        # Spawner control clients
        self.spawner_enable_client = self.create_client(Trigger, '/spawner/enable')
        self.spawner_disable_client = self.create_client(Trigger, '/spawner/disable')
        
        # Action server
        self.action_server = ActionServer(
            self,
            MoveTurtle,
            'collect_turtles',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=ReentrantCallbackGroup()
        )
        
        self.get_logger().info('🎯 Turtle Collection Action Server started')
        self.get_logger().info('   Waiting for services...')
        
        # Wait for required services
        self.closest_turtle_client.wait_for_service(timeout_sec=5.0)
        self.get_logger().info('   ✅ Connected to closest_turtle service')
    
    def _turtle1_pose_callback(self, msg: Pose):
        """Store turtle1's pose"""
        self.turtle_poses['turtle1'] = msg
        # Debug: log pose updates
        if not hasattr(self, '_pose_count'):
            self._pose_count = 0
        self._pose_count += 1
        if self._pose_count % 50 == 0:  # Log every 50 updates
            self.get_logger().info(f'DEBUG: turtle1 pose updated: x={msg.x:.2f}, y={msg.y:.2f}')
    
    def _wait_for_future(self, future, timeout_sec=2.0):
        """Wait for a future without blocking the executor"""
        start_time = time.time()
        while not future.done():
            if time.time() - start_time > timeout_sec:
                return False
            time.sleep(0.01)  # Small sleep to avoid busy waiting
        return True
    
    def _scan_and_subscribe_turtles(self):
        """Scan for turtle pose topics and subscribe to them"""
        topic_list = self.get_topic_names_and_types()
        
        for topic_name, topic_types in topic_list:
            if topic_name.endswith('/pose') and 'turtlesim/msg/Pose' in topic_types:
                # Extract turtle name from topic like /turtle1/pose
                parts = topic_name.split('/')
                if len(parts) >= 2:
                    turtle_name = parts[1]
                    # Skip turtle1 (already subscribed in __init__)
                    if turtle_name and turtle_name != 'turtle1' and turtle_name not in self.turtle_poses:
                        # Subscribe to this turtle's pose
                        self.turtle_poses[turtle_name] = None
                        self.create_subscription(
                            Pose,
                            topic_name,
                            lambda msg, name=turtle_name: self._pose_callback(name, msg),
                            10
                        )
                        self.get_logger().info(f'📡 Subscribed to {topic_name}')
    
    def _pose_callback(self, turtle_name: str, msg: Pose):
        """Store pose for a specific turtle"""
        self.turtle_poses[turtle_name] = msg
    
    # Accept or reject goal requests based on current execution and available targets
    def goal_callback(self, goal_request):
        """Accept or reject goal based on system state"""
        # Reject if another goal is already executing
        if self.goal_executing:
            self.get_logger().warning('⚠️  Goal rejected: Another goal is already executing')
            return GoalResponse.REJECT
        
        # Count available turtles (excluding turtle1)
        available_turtles = sum(1 for name in self.turtle_poses.keys() 
                               if name != 'turtle1' and self.turtle_poses[name] is not None)
        
        if available_turtles == 0:
            self.get_logger().warning('⚠️  Goal rejected: No turtles to collect')
            return GoalResponse.REJECT
        
        self.get_logger().info(f'✅ Goal accepted: {available_turtles} turtles to collect')
        return GoalResponse.ACCEPT
    
    def cancel_callback(self, goal_handle):
        """Accept cancellation requests"""
        self.get_logger().info('🛑 Cancellation requested')
        return CancelResponse.ACCEPT
    
    def _disable_spawner(self):
        """Disable auto spawner during collection"""
        if self.spawner_disable_client.service_is_ready():
            request = Trigger.Request()
            future = self.spawner_disable_client.call_async(request)
            self._wait_for_future(future, timeout_sec=2.0)
            if future.result():
                self.get_logger().info('⏸️  Auto spawner disabled')
        else:
            self.get_logger().warning('⚠️  Spawner disable service not available')
    
    def _enable_spawner(self):
        """Re-enable auto spawner after collection"""
        if self.spawner_enable_client.service_is_ready():
            request = Trigger.Request()
            future = self.spawner_enable_client.call_async(request)
            self._wait_for_future(future, timeout_sec=2.0)
            if future.result():
                self.get_logger().info('▶️  Auto spawner re-enabled')
        else:
            self.get_logger().warning('⚠️  Spawner enable service not available')
    
    def _stop_turtle1(self):
        """Stop turtle1 movement"""
        stop_msg = Twist()
        self.cmd_vel_pub.publish(stop_msg)
    
    def _reset_simulation(self):
        """Reset the simulation"""
        self.get_logger().info('🔄 Resetting simulation...')
        if self.reset_client.service_is_ready():
            request = Empty.Request()
            future = self.reset_client.call_async(request)
            self._wait_for_future(future, timeout_sec=2.0)
            self.get_logger().info('✅ Simulation reset complete')
    
    def _find_closest_turtle(self):
        """Call the closest turtle service to find the nearest turtle"""
        if not self.closest_turtle_client.service_is_ready():
            self.get_logger().error('Closest turtle service not ready')
            return None, 0.0
        
        request = FindClosestTurtle.Request()
        future = self.closest_turtle_client.call_async(request)
        self._wait_for_future(future, timeout_sec=2.0)
        
        if future.result():
            response = future.result()
            if response.success:
                return response.closest_turtle_name, response.distance
        
        return None, 0.0
    
    def _get_distance(self, target_x: float, target_y: float) -> float:
        """Calculate linear distance to target"""
        turtle1_pose = self.turtle_poses.get('turtle1')
        if not turtle1_pose:
            return float('inf')
        
        dx = target_x - turtle1_pose.x
        dy = target_y - turtle1_pose.y
        return math.sqrt(dx * dx + dy * dy)
    
    def _get_angle(self, target_x: float, target_y: float) -> float:
        """Calculate angular displacement to target"""
        turtle1_pose = self.turtle_poses.get('turtle1')
        if not turtle1_pose:
            return 0.0
        
        dx = target_x - turtle1_pose.x
        dy = target_y - turtle1_pose.y
        desired_angle = math.atan2(dy, dx)
        
        # Normalize angle difference to [-pi, pi]
        angle_diff = desired_angle - turtle1_pose.theta
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi
        
        return angle_diff
    
    # Remove a turtle via /kill and clean local tracking
    def _kill_turtle(self, turtle_name: str):
        """Remove a turtle from the simulation"""
        if not self.kill_client.service_is_ready():
            self.get_logger().error('Kill service not ready')
            return False
        
        request = Kill.Request()
        request.name = turtle_name
        future = self.kill_client.call_async(request)
        self._wait_for_future(future, timeout_sec=2.0)
        
        # Remove from local tracking
        if turtle_name in self.turtle_poses:
            del self.turtle_poses[turtle_name]
        
        self.get_logger().info(f'💀 Collected turtle: {turtle_name}')
        return True
    
    # Main execution loop: lock target, move towards it, collect or handle loss
    def execute_callback(self, goal_handle):
        """Execute the turtle collection action"""
        self.goal_executing = True
        self.get_logger().info('🎬 Starting turtle collection...')
        
        feedback_msg = MoveTurtle.Feedback()
        collected_count = 0
        
        try:
            # Main collection loop - only find new target when needed
            while True:
                # Check for cancellation
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    self._stop_turtle1()
                    self._reset_simulation()
                    self.get_logger().info('❌ Goal canceled, simulation reset')
                    return MoveTurtle.Result(success=False)
                
                # Use locked target if already set; otherwise find closest once with retry
                target_name = self.current_target_name
                if not target_name:
                    for attempt in range(3):
                        target_name, _ = self._find_closest_turtle()
                        if target_name:
                            break
                        if attempt < 2:
                            self.get_logger().debug(f'No target found, retrying... (attempt {attempt+1}/3)')
                            time.sleep(0.5)
                    if target_name:
                        self.current_target_name = target_name
                
                if not target_name:
                    # No more turtles to collect after retries
                    self._stop_turtle1()
                    self.get_logger().info(f'🎉 Collection complete! Collected {collected_count} turtles')
                    goal_handle.succeed()
                    return MoveTurtle.Result(success=True)
                
                self.get_logger().info(f'🎯 Targeting: {target_name} (locked until collected/lost)')
                
                # Movement loop - stick to THIS target until reached or lost
                while True:
                    # Check cancellation
                    if goal_handle.is_cancel_requested:
                        goal_handle.canceled()
                        self._stop_turtle1()
                        self._reset_simulation()
                        self.get_logger().info('❌ Goal canceled, simulation reset')
                        return MoveTurtle.Result(success=False)
                    
                    # Get target pose
                    target_pose = self.turtle_poses.get(target_name)
                    if not target_pose:
                        # Target temporarily missing; retry to reacquire a few times
                        for attempt in range(self.lost_target_retries):
                            time.sleep(self.lost_target_check_delay)
                            target_pose = self.turtle_poses.get(target_name)
                            if target_pose:
                                break
                        if not target_pose:
                            # Target truly lost after retries; unlock and find next
                            self.get_logger().warning(f'⚠️  Lost target {target_name} after {self.lost_target_retries} retries, unlocking and finding next...')
                            self.current_target_name = None
                            break
                        # Pose reacquired; continue movement without changing target
                    
                    # Calculate distance and angle
                    distance = self._get_distance(target_pose.x, target_pose.y)
                    angle = self._get_angle(target_pose.x, target_pose.y)
                    
                    # Publish feedback
                    feedback_msg.distance = distance
                    goal_handle.publish_feedback(feedback_msg)
                    
                    # Check if reached
                    if distance < self.collection_distance:
                        if self._kill_turtle(target_name):
                            collected_count += 1
                            # Clear current target lock after collection
                            self.current_target_name = None
                            break
                        else:
                            # Kill failed, target might already be gone
                            self.current_target_name = None
                            break
                    
                    # Move towards target with proportional control
                    move_msg = Twist()
                    move_msg.linear.x = self.linear_gain * distance
                    move_msg.angular.z = self.angular_gain * angle
                    
                    # Log commands every 20 iterations for debugging
                    if collected_count == 0:  # Only for first turtle
                        self.get_logger().info(
                            f'CMD: dist={distance:.2f}, angle={angle:.2f}, '
                            f'vx={move_msg.linear.x:.2f}, wz={move_msg.angular.z:.2f}'
                        )
                    
                    self.cmd_vel_pub.publish(move_msg)
                    
                    # Short sleep - callbacks are processed by MultiThreadedExecutor
                    time.sleep(0.1)
        
        except Exception as e:
            self.get_logger().error(f'❌ Error during collection: {e}')
            self._stop_turtle1()
            goal_handle.abort()
            return MoveTurtle.Result(success=False)
        finally:
            # Always clear the flag when execution ends
            self.goal_executing = False


def main(args=None):
    rclpy.init(args=args)
    server = TurtleCollectionServer()
    executor = MultiThreadedExecutor()
    rclpy.spin(server, executor=executor)
    server.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
