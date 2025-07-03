#!/usr/bin/env python3
"""
Simple test script to publish log messages for testing the ROS Synapse log viewer.
"""
import rclpy
from rclpy.node import Node
import time

class TestLogPublisher(Node):
    def __init__(self):
        super().__init__('test_log_publisher')
        self.timer = self.create_timer(2.0, self.publish_logs)
        self.counter = 0
        
    def publish_logs(self):
        self.counter += 1
        
        # Publish different log levels
        self.get_logger().info(f'This is an INFO message #{self.counter}')
        
        if self.counter % 3 == 0:
            self.get_logger().warn(f'This is a WARNING message #{self.counter}')
            
        if self.counter % 5 == 0:
            self.get_logger().error(f'This is an ERROR message #{self.counter}')
            
        if self.counter % 10 == 0:
            self.get_logger().debug(f'This is a DEBUG message #{self.counter}')

def main():
    rclpy.init()
    node = TestLogPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()