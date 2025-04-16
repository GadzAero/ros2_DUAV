#!/usr/bin/env python3

# Import standard utils
from time import sleep
# Import ROS2 utils
import rclpy
from rclpy.node import Node
# Import Intefaces
from interfaces.srv import SetMode, Arm, Takeoff, Reposition, Land, Rtl, Disarm
# Import FSM utils
import popeye.utils_FSM as fsm
from popeye.utils_MAV import DEFAULT_LAT, DEFAULT_LON, DEFAULT_ALT

#####################################################################################################################################################################
##### Node MAVLink Manager #####################################################################################################################################################################
class FSMInterface(Node):
    def __init__(self):
        super().__init__('FSM_interface', namespace='POPEYE')
        
        ### Services
        # set_mode
        self.cli__set_mode = self.create_client(SetMode, 'set_mode')
        while not self.cli__set_mode.wait_for_service(timeout_sec=1.0):
            self.get_logger().warning('service not available, waiting again...')
        self.req__set_mode = SetMode.Request()
        # arm
        self.cli__arm = self.create_client(Arm, 'arm')
        while not self.cli__arm.wait_for_service(timeout_sec=1.0):
            self.get_logger().warning('service not available, waiting again...')
        self.req__arm = Arm.Request()
        # takeoff
        self.cli__takeoff = self.create_client(Takeoff, 'takeoff')
        while not self.cli__takeoff.wait_for_service(timeout_sec=1.0):
            self.get_logger().warning('service not available, waiting again...')
        self.req__takeoff = Takeoff.Request()
        # reposition
        self.cli__reposition = self.create_client(Reposition, 'reposition')
        while not self.cli__reposition.wait_for_service(timeout_sec=1.0):
            self.get_logger().warning('service not available, waiting again...')
        self.req__reposition = Reposition.Request()
        # land
        self.cli__land = self.create_client(Land, 'land')
        while not self.cli__land.wait_for_service(timeout_sec=1.0):
            self.get_logger().warning('service not available, waiting again...')
        self.req__land = Land.Request()
        # rtl
        self.cli__rtl = self.create_client(Rtl, 'rtl')
        while not self.cli__rtl.wait_for_service(timeout_sec=1.0):
            self.get_logger().warning('service not available, waiting again...')
        self.req__rtl = Rtl.Request()
        # disarm
        self.cli__disarm = self.create_client(Disarm, 'disarm')
        while not self.cli__disarm.wait_for_service(timeout_sec=1.0):
            self.get_logger().warning('service not available, waiting again...')
        self.req__disarm = Disarm.Request()
        
        ### Start the FSM
        sm = fsm.PopeyeFSM(self)
        
        self.get_logger().info("NODE FSM_interface STARTED.")   
    
    #---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    #----- Function to call the SET_MODE service  ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    def call__set_mode(self, mode='RTL'):
        self.req__set_mode.mode_name = mode
        self.get_logger().info(f"> Calling SET_MODE (Force:{self.req__set_mode.mode_name})")
        future = self.cli__set_mode.call_async(self.req__set_mode)
        rclpy.spin_until_future_complete(self, future)
        if future.result().success:
            self.get_logger().info(f"     -> Successful")
        else:
            self.get_logger().warning(f"     -> Failed")
            
    #---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    #----- Function to call the ARM service  ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    def call__arm(self, force=False):
        self.req__arm.force = force
        self.get_logger().info(f"> Calling ARM (Force:{self.req__arm.force})")
        future = self.cli__arm.call_async(self.req__arm)
        rclpy.spin_until_future_complete(self, future)
        if future.result().success:
            self.get_logger().info(f"     -> Successful")
        else:
            self.get_logger().warning(f"     -> Failed")
            
    #---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    #----- Function to call the TAKEOFF service  ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    def call__takeoff(self, alt=6):
        self.req__takeoff.alt = alt*1.
        self.get_logger().info(f"> Calling TAKEOFF (Alt:{self.req__takeoff.alt})")
        future = self.cli__takeoff.call_async(self.req__takeoff)
        rclpy.spin_until_future_complete(self, future)
        sleep(15)
        if future.result().success:
            self.get_logger().info(f"     -> Successful")
        else:
            self.get_logger().warning(f"     -> Failed")
            
    #---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    #----- Function to call the REPOSITION service  ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    def call__reposition(self, lat=DEFAULT_LAT, lon=DEFAULT_LON, alt=DEFAULT_ALT):
        self.req__reposition.lat = lat*1.
        self.req__reposition.lon = lon*1.
        self.req__reposition.alt = alt*1.
        self.get_logger().info(f"> Calling REPOSITION (Lat:{self.req__reposition.lat}, Lon:{self.req__reposition.lon}, Alt:{self.req__reposition.alt})")
        future = self.cli__reposition.call_async(self.req__reposition)
        rclpy.spin_until_future_complete(self, future)
        sleep(15)
        if future.result().success:
            self.get_logger().info(f"     -> Successful")
        else:
            self.get_logger().warning(f"     -> Failed")
            
    #---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    #----- Function to call the LAND service  ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    def call__land(self):
        self.get_logger().info("> Calling LAND.")
        future = self.cli__land.call_async(self.req__land)
        rclpy.spin_until_future_complete(self, future)
        sleep(15)
        if future.result().success:
            self.get_logger().info(f"     -> Successful")
        else:
            self.get_logger().warning(f"     -> Failed")
            
    #---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    #----- Function to call the RTL service  ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    def call__rtl(self):
        self.get_logger().info("> Calling RTL.")
        future = self.cli__rtl.call_async(self.req__rtl)
        rclpy.spin_until_future_complete(self, future)
        sleep(30)
        if future.result().success:
            self.get_logger().info(f"     -> Successful")
        else:
            self.get_logger().warning(f"     -> Failed")
            
    #---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    #----- Function to call the DISARM service  ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    def call__disarm(self, force=False):
        self.req__disarm.force = force
        self.get_logger().info(f"> Calling DISARM (Force:{self.req__disarm.force})")
        future = self.cli__disarm.call_async(self.req__disarm)
        rclpy.spin_until_future_complete(self, future)
        if future.result().success:
            self.get_logger().info(f"     -> Successful")
        else:
            self.get_logger().warning(f"     -> Failed")
        
#####################################################################################################################################################################
##### Node entry point #####################################################################################################################################################################
def main(args=None):
    rclpy.init(args=args)
    node = FSMInterface()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()