#!/usr/bin/env python3
"""
Unit tests for the ClosestTurtleService node.

Tests verify:
1. Service initialization and availability
2. Correct distance calculation between turtles
3. Handling of edge cases (no turtles, only turtle1, etc.)
4. Response format and validity
"""

import unittest
import math
import rclpy
from rclpy.node import Node
from custom_interfaces.srv import FindClosestTurtle
from turtlesim.msg import Pose


class TestClosestTurtleService(unittest.TestCase):
    """Test suite for ClosestTurtleService"""

    @classmethod
    def setUpClass(cls):
        """Initialize ROS2 once for all tests"""
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        """Shutdown ROS2 after all tests"""
        rclpy.shutdown()

    def setUp(self):
        """Set up test fixtures before each test"""
        # Import here to ensure ROS is initialized
        from turtle_controller_assignment.closest_turtle_service import ClosestTurtleService
        
        self.service_node = ClosestTurtleService()
        self.test_node = Node('test_closest_turtle_client')
        
        # Create a client to test the service
        self.client = self.test_node.create_client(
            FindClosestTurtle,
            '/find_closest_turtle'
        )
        
        # Wait for service to be available
        timeout = 5.0
        if not self.client.wait_for_service(timeout_sec=timeout):
            self.fail(f'Service not available after waiting {timeout} seconds')

    def tearDown(self):
        """Clean up after each test"""
        self.service_node.destroy_node()
        self.test_node.destroy_node()

    def _inject_turtle_pose(self, name: str, x: float, y: float, theta: float = 0.0):
        """Helper method to inject a turtle pose into the service node"""
        pose = Pose()
        pose.x = x
        pose.y = y
        pose.theta = theta
        pose.linear_velocity = 0.0
        pose.angular_velocity = 0.0
        self.service_node.turtle_poses[name] = pose

    def _call_service(self):
        """Helper method to call the service and wait for response"""
        request = FindClosestTurtle.Request()
        future = self.client.call_async(request)
        
        # Spin until future completes
        while rclpy.ok() and not future.done():
            rclpy.spin_once(self.service_node, timeout_sec=0.1)
            rclpy.spin_once(self.test_node, timeout_sec=0.1)
        
        if future.done():
            return future.result()
        else:
            self.fail('Service call timed out')

    def test_service_available(self):
        """Test 1: Verify service is available"""
        self.assertTrue(
            self.client.service_is_ready(),
            'Service should be available'
        )

    def test_no_turtle1_returns_failure(self):
        """Test 2: Service should fail if turtle1 doesn't exist"""
        # Clear all turtle poses
        self.service_node.turtle_poses.clear()
        
        response = self._call_service()
        
        self.assertFalse(response.success, 'Should fail when turtle1 is missing')
        self.assertEqual(response.closest_turtle_name, '', 'Name should be empty')
        self.assertEqual(response.distance, 0.0, 'Distance should be 0.0')
        self.assertIn('turtle1 not found', response.message.lower())

    def test_only_turtle1_returns_failure(self):
        """Test 3: Service should fail if only turtle1 exists (no other turtles)"""
        # Clear all poses and add only turtle1
        self.service_node.turtle_poses.clear()
        self._inject_turtle_pose('turtle1', 5.5, 5.5)
        
        response = self._call_service()
        
        self.assertFalse(response.success, 'Should fail when no other turtles exist')
        self.assertEqual(response.closest_turtle_name, '', 'Name should be empty')
        self.assertEqual(response.distance, 0.0, 'Distance should be 0.0')
        self.assertIn('no other turtles', response.message.lower())

    def test_finds_single_closest_turtle(self):
        """Test 4: Service should find the only other turtle"""
        self.service_node.turtle_poses.clear()
        self._inject_turtle_pose('turtle1', 5.0, 5.0)
        self._inject_turtle_pose('turtle2', 8.0, 5.0)
        
        response = self._call_service()
        
        self.assertTrue(response.success, 'Should succeed with two turtles')
        self.assertEqual(response.closest_turtle_name, 'turtle2')
        self.assertAlmostEqual(response.distance, 3.0, places=2)

    def test_finds_closest_among_multiple_turtles(self):
        """Test 5: Service should identify the closest turtle among several"""
        self.service_node.turtle_poses.clear()
        self._inject_turtle_pose('turtle1', 5.0, 5.0)
        self._inject_turtle_pose('turtle2', 10.0, 10.0)  # Distance: ~7.07
        self._inject_turtle_pose('turtle3', 7.0, 5.0)   # Distance: 2.0 (closest)
        self._inject_turtle_pose('turtle4', 5.0, 10.0)  # Distance: 5.0
        
        response = self._call_service()
        
        self.assertTrue(response.success)
        self.assertEqual(response.closest_turtle_name, 'turtle3')
        self.assertAlmostEqual(response.distance, 2.0, places=2)

    def test_distance_calculation_accuracy(self):
        """Test 6: Verify distance calculation is accurate (Euclidean distance)"""
        self.service_node.turtle_poses.clear()
        self._inject_turtle_pose('turtle1', 0.0, 0.0)
        self._inject_turtle_pose('turtle2', 3.0, 4.0)  # Distance should be 5.0
        
        response = self._call_service()
        
        expected_distance = math.sqrt(3.0**2 + 4.0**2)  # = 5.0
        self.assertTrue(response.success)
        self.assertEqual(response.closest_turtle_name, 'turtle2')
        self.assertAlmostEqual(response.distance, expected_distance, places=2)

    def test_ignores_turtles_with_none_pose(self):
        """Test 7: Service should ignore turtles that don't have pose data yet"""
        self.service_node.turtle_poses.clear()
        self._inject_turtle_pose('turtle1', 5.0, 5.0)
        self._inject_turtle_pose('turtle2', 7.0, 5.0)
        
        # Add a turtle with None pose (not yet received data)
        self.service_node.turtle_poses['turtle3'] = None
        
        response = self._call_service()
        
        self.assertTrue(response.success)
        self.assertEqual(response.closest_turtle_name, 'turtle2', 
                        'Should ignore turtle3 with None pose')
        self.assertAlmostEqual(response.distance, 2.0, places=2)

    def test_response_message_format(self):
        """Test 8: Verify response message contains expected information"""
        self.service_node.turtle_poses.clear()
        self._inject_turtle_pose('turtle1', 5.0, 5.0)
        self._inject_turtle_pose('turtle_alpha', 6.0, 5.0)
        
        response = self._call_service()
        
        self.assertTrue(response.success)
        self.assertIn('turtle_alpha', response.message)
        self.assertIn('1.00', response.message)  # Distance formatted to 2 decimals

    def test_diagonal_distance(self):
        """Test 9: Test distance calculation for diagonal movement"""
        self.service_node.turtle_poses.clear()
        self._inject_turtle_pose('turtle1', 1.0, 1.0)
        self._inject_turtle_pose('turtle2', 4.0, 5.0)
        
        response = self._call_service()
        
        expected_distance = math.sqrt((4.0 - 1.0)**2 + (5.0 - 1.0)**2)  # = 5.0
        self.assertTrue(response.success)
        self.assertAlmostEqual(response.distance, expected_distance, places=2)

    def test_same_position_returns_zero_distance(self):
        """Test 10: Turtles at same position should have distance 0.0"""
        self.service_node.turtle_poses.clear()
        self._inject_turtle_pose('turtle1', 5.0, 5.0)
        self._inject_turtle_pose('turtle2', 5.0, 5.0)
        
        response = self._call_service()
        
        self.assertTrue(response.success)
        self.assertEqual(response.closest_turtle_name, 'turtle2')
        self.assertAlmostEqual(response.distance, 0.0, places=2)


if __name__ == '__main__':
    unittest.main()
