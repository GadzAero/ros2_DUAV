#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from pymavlink import mavutil
from custom_msgs.msg import GPSData

class SendFireCoor(Node):
    def __init__(self):
        super().__init__('send_fire_coor')

        self.subscription_GPS_coor_fire(GPSData, 'IN/CAM/GPS_coor_fire', self.lc_GPS_fire_coor, 10)

        self.get_logger().info('Node SendFireCoor initisalized.')

    def lc_GPS_fire_coor(self, gps_data: GPSData):
       # Prepare the GPS coordinates message
        latitude = gps_data.latitude
        longitude = gps_data.longitude
        altitude = gps_data.altitude

        # Create a MAVLink STATUSTEXT message
        statustext_message = MAVLink_statustext_message(
            severity=6,  # Info-level severity
            text=f"FIRE_DETECTED_GPS_COOR: LAT={latitude}, LON={longitude}, ALT={altitude}".ljust(50)
        )

        # Convert the MAVLink message to ROS2-compatible message
        mavlink_ros_msg = Mavlink()
        mavlink_ros_msg.header.stamp = self.get_clock().now().to_msg()
        mavlink_ros_msg.framing_status = 1  # Framing OK
        mavlink_ros_msg.magic = mavutil.mavlink.MAVLINK_STX
        mavlink_ros_msg.len = statustext_message.get_msgbuf().size
        mavlink_ros_msg.incompat_flags = 0
        mavlink_ros_msg.compat_flags = 0
        mavlink_ros_msg.seq = 0  # Sequence number
        mavlink_ros_msg.sysid = 1  # System ID
        mavlink_ros_msg.compid = 1  # Component ID
        mavlink_ros_msg.msgid = statustext_message.get_msgid()
        mavlink_ros_msg.payload64 = statustext_message.pack(mavutil.mavlink.MAVLink('', 1, 1))

        # Publish the MAVLink message
        self.mavlink_publisher.publish(mavlink_ros_msg)
        self.get_logger().info(
            f"Sent MAVLink STATUSTEXT: FIRE_DETECTED_GPS_COOR: LAT={latitude}, LON={longitude}, ALT={altitude}"
        )

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
