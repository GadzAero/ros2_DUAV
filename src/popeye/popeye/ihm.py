#!/usr/bin/env python3

from interfaces.srv import TriggerArm

import rclpy
from rclpy.node import Node

class IHM(Node):
    def __init__(self):
        super().__init__('ihm', namespace='POPEYE')
        
        # CLIENT
        self.arm_client = self.create_client(TriggerArm, 'trigger_arm')
        while not self.arm_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
            
        self.req = TriggerArm.Request()
            
    def trigger_arm(self, a):
        self.req.a = a
        return self.arm_client.call_async(self.req)
    
def main():
    rclpy.init()
    
    ihm = IHM()
    future = ihm.trigger_arm(int(10))
    rclpy.spin_until_future_complete(ihm, future)
    response = future.result()
    ihm.get_logger().info(f'Arm response: {response.success}')
    
    ihm.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
    
    