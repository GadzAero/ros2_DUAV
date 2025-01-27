#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from custom_msgs.msg import GPSData

class AnalyseCAM(Node):
    def __init__(self):
        super().__init__('analyse_CAM')
        
        self.publisher_GPS_coor_fire = self.create_publisher(GPSData, 'IN/CAM/GPS_coor_fire', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)  # 1 second interval
        self.get_logger().info("Publisher Node Started")
        
    def tc_GPS_coor_fire(self):
        # Création du message GPSData
        gps_message = GPSData()
        gps_message.latitude = self.latitude
        gps_message.longitude = self.longitude
        gps_message.altitude = self.altitude

        # Publication
        self.publisher_GPS_coor_fire.publish(gps_message)
        self.get_logger().info(f"Published: Latitude={gps_message.latitude}, Longitude={gps_message.longitude}, Altitude={gps_message.altitude}")

def main(args=None):
    rclpy.init(args=args)
    node = AnalyseCAM()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
