#!/usr/bin/env python3

# Import standard utils
from time import sleep
# Import ROS2 utils
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
# Import Intefaces
from interfaces.srv import SetMode, Arm, Reposition, Rtl, Disarm
from interfaces.action import TakeoffAct, Land
# Import FSM utils
import popeye.utils_FSM as fsm
from popeye.utils_MAV import DEFAULT_LAT, DEFAULT_LON, DEFAULT_ALT

#####################################################################################################################################################################
##### Node MAVLink Manager #####################################################################################################################################################################
class FSMInterface(Node):
    def __init__(self):
        super().__init__('FSM_interface', namespace='POPEYE')
        
        ### Actions clients
        self.cli_act__takeoff = ActionClient(self, TakeoffAct, 'takeoff', callback_group=MutuallyExclusiveCallbackGroup())
        self.cli_act__land    = ActionClient(self, Land,       'land',    callback_group=MutuallyExclusiveCallbackGroup())
        
        ### Services clients
        self.cli_srv__set_mode   = self.create_client(SetMode,    'set_mode',   callback_group=MutuallyExclusiveCallbackGroup())
        self.cli_srv__arm        = self.create_client(Arm,        'arm',        callback_group=MutuallyExclusiveCallbackGroup())
        self.cli_srv__reposition = self.create_client(Reposition, 'reposition', callback_group=MutuallyExclusiveCallbackGroup())
        self.cli_srv__rtl        = self.create_client(Rtl,        'rtl',        callback_group=MutuallyExclusiveCallbackGroup())
        self.cli_srv__disarm     = self.create_client(Disarm,     'disarm',     callback_group=MutuallyExclusiveCallbackGroup())
        while (not self.cli_srv__set_mode.      wait_for_service(timeout_sec=1.0)
                or not self.cli_srv__arm.       wait_for_service(timeout_sec=1.0)
                or not self.cli_srv__reposition.wait_for_service(timeout_sec=1.0)
                or not self.cli_srv__rtl.       wait_for_service(timeout_sec=1.0)
                or not self.cli_srv__disarm.    wait_for_service(timeout_sec=1.0)):
            self.get_logger().warning('service(s) not available, waiting again...')
        self.req__set_mode   = SetMode.Request()
        self.req__arm        = Arm.Request()
        self.req__reposition = Reposition.Request()
        self.req__rtl        = Rtl.Request()
        self.req__disarm     = Disarm.Request() 
        
        ### Start the FSM
        sm = fsm.PopeyeFSM(self)
        
        self.get_logger().info("NODE FSM_interface STARTED.")   
    
    #---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    #----- Function to call the SET_MODE service  ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    def call__set_mode(self, mode='RTL'):
        self.req__set_mode.mode_name = mode
        self.get_logger().info(f"> Calling SET_MODE (Force:{self.req__set_mode.mode_name})")
        future = self.cli_srv__set_mode.call_async(self.req__set_mode)
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
        future = self.cli_srv__arm.call_async(self.req__arm)
        rclpy.spin_until_future_complete(self, future)
        if future.result().success:
            self.get_logger().info(f"     -> Successful")
        else:
            self.get_logger().warning(f"     -> Failed")
            
    #---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    #----- Function to call the TAKEOFF action  ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    def call__takeoff(self, alt=6):
        goal_msg = TakeoffAct.Goal()
        goal_msg.alt = alt*1.
        self.get_logger().info(f"> Calling TAKEOFF action (Alt:{goal_msg.alt})")
        self.cli_act__takeoff.wait_for_server()
        self.takeoff__send_goal_future = self.cli_act__takeoff.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self.takeoff__send_goal_future.add_done_callback(self.goal_response_callback)
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')

        self.takeoff__get_result_future = goal_handle.get_result_async()
        self.takeoff__get_result_future.add_done_callback(self.get_result_callback)
    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result.success}')
        
    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: {0}'.format(feedback.current_alt))
            
    #---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    #----- Function to call the REPOSITION service  ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    def call__reposition(self, lat=DEFAULT_LAT, lon=DEFAULT_LON, alt=DEFAULT_ALT):
        sleep(20)   ########################################
        self.req__reposition.lat = lat*1.
        self.req__reposition.lon = lon*1.
        self.req__reposition.alt = alt*1.
        self.get_logger().info(f"> Calling REPOSITION (Lat:{self.req__reposition.lat}, Lon:{self.req__reposition.lon}, Alt:{self.req__reposition.alt})")
        future = self.cli_srv__reposition.call_async(self.req__reposition)
        rclpy.spin_until_future_complete(self, future)
        sleep(15)
        if future.result().success:
            self.get_logger().info(f"     -> Successful")
        else:
            self.get_logger().warning(f"     -> Failed")
            
    #---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    #----- Function to call the LAND action  ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    def call__land(self):
        self.get_logger().info("> Calling LAND action.")
        self.cli_act__land.wait_for_server()
        self.cli_act__land_future = self.cli_act__land.send_goal_async(Land.Goal())
        sleep(20)
    #     self.cli_act__land_future.add_done_callback(self.goal_response_callback)
    #     # future = self.cli_srv__land.call_async(self.req__land)
    #     # rclpy.spin_until_future_complete(self, future)
    #     # sleep(15)
    #     # if future.result().success:
    #     #     self.get_logger().info(f"     -> Successful")
    #     # else:
    #     #     self.get_logger().warning(f"     -> Failed")
    # def goal_response_callback(self, future):
    #     goal_handle = future.result()
    #     if not goal_handle.accepted:
    #         self.get_logger().info('Goal rejected')
    #         return

    #     self.get_logger().info('Goal accepted')

    #     self.cli_act__land_future = goal_handle.get_result_async()
    #     self.cli_act__land_future.add_done_callback(self.get_result_callback)
    # def get_result_callback(self, future):
    #     result = future.result().result
    #     self.get_logger().info('Result: {0}'.format(result.success))
    #     rclpy.shutdown()
            
    #---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    #----- Function to call the RTL service  ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    def call__rtl(self):
        self.get_logger().info("> Calling RTL.")
        future = self.cli_srv__rtl.call_async(self.req__rtl)
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
        future = self.cli_srv__disarm.call_async(self.req__disarm)
        rclpy.spin_until_future_complete(self, future)
        if future.result().success:
            self.get_logger().info(f"     -> Successful")
        else:
            self.get_logger().warning(f"     -> Failed")
        
#####################################################################################################################################################################
##### Node entry point #####################################################################################################################################################################
def main(args=None):
    rclpy.init(args=args)
    
    ### Creating the mutlithread executor
    node = FSMInterface()
    # executor = rclpy.executors.MultiThreadedExecutor(num_threads=3)
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()