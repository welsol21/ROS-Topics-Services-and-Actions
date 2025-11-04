#!/usr/bin/env python3
"""
Turtle Monitor Service - With Managed/External Turtle Distinction
"""

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from turtlesim.msg import Pose
import time
from threading import Lock

class TurtleMonitorService(Node):
    
    def __init__(self):
        super().__init__('turtle_monitor_service')
        
        self.monitor_service = self.create_service(
            Trigger, 
            '/monitor_turtles', 
            self.monitor_callback
        )
        
        # Separate tracking for managed vs external turtles
        self.managed_turtles = {}    # Turtles registered by spawner
        self.external_turtles = {}   # Turtles detected automatically
        self.lock = Lock()
        
        self.get_logger().info('🐢 Turtle Monitor Service started (managed/external distinction)')
    
    def register_managed_turtle(self, turtle_name):
        """Register a turtle as managed by spawner"""
        with self.lock:
            if turtle_name not in self.managed_turtles:
                self.managed_turtles[turtle_name] = self.create_turtle_data()
                self.create_pose_subscription(turtle_name)
                self.get_logger().info(f'📡 Monitoring MANAGED turtle: {turtle_name}')
                return True
            return False
    
    def create_turtle_data(self):
        """Create tracking data structure for a turtle"""
        return {
            'current_pose': False,
            'previous_pose': False, 
            'last_update': time.time(),
            'removed': False
        }
    
    def create_pose_subscription(self, turtle_name):
        """Create pose subscription for a turtle"""
        def pose_callback(msg):
            self.update_turtle_pose(turtle_name)
        
        self.create_subscription(
            Pose,
            f'/{turtle_name}/pose',
            pose_callback,
            10
        )
    
    def update_turtle_pose(self, turtle_name):
        """Update turtle pose status"""
        with self.lock:
            # Check both managed and external
            if turtle_name in self.managed_turtles:
                data = self.managed_turtles[turtle_name]
                data['previous_pose'] = data['current_pose']
                data['current_pose'] = True
                data['last_update'] = time.time()
                data['removed'] = False
            
            if turtle_name in self.external_turtles:
                data = self.external_turtles[turtle_name]
                data['previous_pose'] = data['current_pose']
                data['current_pose'] = True
                data['last_update'] = time.time()
                data['removed'] = False
    
    def auto_detect_external_turtles(self):
        """Automatically detect external turtles by checking pose topics"""
        # This is simplified - in real implementation you'd check available topics
        # For now, we rely on pose callbacks to auto-detect
        pass
    
    def check_turtle_health(self):
        """Check health of all turtles with double verification"""
        with self.lock:
            current_time = time.time()
            removed_managed = []
            removed_external = []
            
            # Check managed turtles
            for turtle_name, data in self.managed_turtles.items():
                if current_time - data['last_update'] > 3.0:
                    data['previous_pose'] = data['current_pose']
                    data['current_pose'] = False
                
                # Double-check removal
                if not data['current_pose'] and not data['previous_pose'] and not data['removed']:
                    data['removed'] = True
                    removed_managed.append(turtle_name)
            
            # Check external turtles
            for turtle_name, data in self.external_turtles.items():
                if current_time - data['last_update'] > 3.0:
                    data['previous_pose'] = data['current_pose']
                    data['current_pose'] = False
                
                if not data['current_pose'] and not data['previous_pose'] and not data['removed']:
                    data['removed'] = True
                    removed_external.append(turtle_name)
            
            return removed_managed, removed_external
    
    def get_monitoring_status(self):
        """Get current monitoring status"""
        with self.lock:
            status = {
                'managed_count': len(self.managed_turtles),
                'external_count': len(self.external_turtles),
                'managed_active': len([t for t, d in self.managed_turtles.items() if d['current_pose']]),
                'external_active': len([t for t, d in self.external_turtles.items() if d['current_pose']]),
                'managed_removed': len([t for t, d in self.managed_turtles.items() if d['removed']]),
                'external_removed': len([t for t, d in self.external_turtles.items() if d['removed']])
            }
            return status
    
    def monitor_callback(self, request, response):
        """Service callback for monitoring"""
        try:
            # Check health
            removed_managed, removed_external = self.check_turtle_health()
            
            # Get status
            status = self.get_monitoring_status()
            
            # Prepare response
            if removed_managed:
                response.success = True
                response.message = f"MANAGED_REMOVED:{','.join(removed_managed)}"
                for turtle_name in removed_managed:
                    self.get_logger().warning(f'🚨 MANAGED turtle removed: {turtle_name}')
                
            elif removed_external:
                response.success = True
                response.message = f"EXTERNAL_REMOVED:{','.join(removed_external)}"
                for turtle_name in removed_external:
                    self.get_logger().info(f'ℹ️  EXTERNAL turtle removed: {turtle_name}')
                    
            else:
                response.success = True
                response.message = f"OK: {status['managed_active']}/{status['managed_count']} managed active, {status['external_active']} external"
            
        except Exception as e:
            response.success = False
            response.message = f"ERROR:{str(e)}"
        
        return response

def main(args=None):
    rclpy.init(args=args)
    monitor = TurtleMonitorService()
    rclpy.spin(monitor)
    monitor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()