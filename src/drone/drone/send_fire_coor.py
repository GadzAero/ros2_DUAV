#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from pymavlink import mavutil
from geographic_msgs.msg import GeoPoint

class SendFireCoor(Node):
    def __init__(self):
        super().__init__('send_fire_coor')

        # self.subscription_GPS_coor_fire(GeoPoint, 'IN/CAM/GPS_coor_fire', self.lc_GPS_fire_coor, 10)

        self.get_logger().info('Node SendFireCoor initisalized.')

    def lc_GPS_fire_coor(self, GeoPoint):
        pass

def main(args=None):
    rclpy.init(args=args)
    node = SendFireCoor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
