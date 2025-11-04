#!/usr/bin/env python3
"""
Turtle Name Manager - Service for generating unique turtle names
"""

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
import threading

class TurtleNameManager(Node):
    """
    Service server for generating unique turtle names
    """
    
    def __init__(self):
        super().__init__('turtle_name_manager')
        
        # Service for getting unique names
        self.name_service = self.create_service(
            Trigger, 
            'generate_unique_name', 
            self.generate_name_callback
        )
        
        # Counter for generating unique names
        self.name_counter = 1
        self.lock = threading.Lock()
        
        self.get_logger().info('Turtle Name Manager started - Ready to generate unique names!')
    
    def generate_name_callback(self, request, response):
        """
        Callback for generating unique turtle names
        """
        with self.lock:
            unique_name = f"turtle_{self.name_counter}"
            self.name_counter += 1
            
            response.success = True
            response.message = unique_name
            
            self.get_logger().info(f'Generated unique name: {unique_name}')
        
        return response

def main(args=None):
    rclpy.init(args=args)
    name_manager = TurtleNameManager()
    
    try:
        rclpy.spin(name_manager)
    except KeyboardInterrupt:
        pass
    finally:
        name_manager.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()