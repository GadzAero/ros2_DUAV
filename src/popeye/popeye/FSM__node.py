#!/usr/bin/env python3

# Import standard utils
from time import sleep
from popeye.PARAMS_utils import *
# Import ROS2 utils
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
# Import Intefaces
from interfaces.srv import SetMode, Arm, Rtl, Disarm, Drop, TakePhoto, TakeVideo
from interfaces.action import Takeoff, Land, Reposition
from interfaces.msg import Fire, GpsPosition, Task, TaskParams, State, FeedbackParams
# Import FSM utils
import popeye.FSM__utils as fsm

#####################################################################################################################################################################
##### Node MAVLink Manager #####################################################################################################################################################################
class FSMNode(Node):
    def __init__(self):
        super().__init__('FSM__node', namespace='POPEYE')
        
        ### ROS2 Callbacks
        ## Callback groups
        cb_1 = ReentrantCallbackGroup()
        ## Timers
        # self.create_timer(0.1, self.timer_cb__test,       callback_group=MutuallyExclusiveCallbackGroup())
        self.create_timer(1.0,   self.timer_cb__init_fsm,   callback_group=MutuallyExclusiveCallbackGroup())
        self.create_timer(0.5, self.timer_cb__send_state, callback_group=MutuallyExclusiveCallbackGroup())
        ## Publishers
        self.pub__state = self.create_publisher(State, 'state', 10, callback_group=MutuallyExclusiveCallbackGroup())
        ## Subscribers
        self.create_subscription(Task,        'task',         self.sub_cb__task,         10, callback_group=MutuallyExclusiveCallbackGroup())
        self.create_subscription(Fire,        'fire',         self.sub_cb__fire,         10, callback_group=MutuallyExclusiveCallbackGroup())
        self.create_subscription(GpsPosition, 'uav_position', self.sub_cb__uav_position, 10, callback_group=MutuallyExclusiveCallbackGroup())
        self.create_subscription(GpsPosition, 'CAM/fire_pos', self.sub_cb__cam_fire_pos, 10, callback_group=MutuallyExclusiveCallbackGroup())
        self.create_subscription(GpsPosition, 'CAM/park_pos', self.sub_cb__cam_park_pos, 10, callback_group=MutuallyExclusiveCallbackGroup())
        ## Services clients
        self.cli_srv__set_mode       = self.create_client(SetMode,   'set_mode',       callback_group=MutuallyExclusiveCallbackGroup())
        self.cli_srv__arm            = self.create_client(Arm,       'arm',            callback_group=MutuallyExclusiveCallbackGroup())
        self.cli_srv__payload_drop   = self.create_client(Drop,      'payload_drop',   callback_group=MutuallyExclusiveCallbackGroup())
        self.cli_srv__payload_reload = self.create_client(Drop,      'payload_reload', callback_group=MutuallyExclusiveCallbackGroup())
        self.cli_srv__rtl            = self.create_client(Rtl,       'rtl',            callback_group=MutuallyExclusiveCallbackGroup())
        self.cli_srv__disarm         = self.create_client(Disarm,    'disarm',         callback_group=MutuallyExclusiveCallbackGroup())
        self.cli_srv__take_photo     = self.create_client(TakePhoto, 'take_photo',     callback_group=MutuallyExclusiveCallbackGroup())
        self.cli_srv__take_video     = self.create_client(TakeVideo, 'take_video',     callback_group=MutuallyExclusiveCallbackGroup())
        ## Actions clients
        self.cli_act__takeoff    = ActionClient(self, Takeoff,    'takeoff',    callback_group=MutuallyExclusiveCallbackGroup())
        self.cli_act__land       = ActionClient(self, Land,       'land',       callback_group=MutuallyExclusiveCallbackGroup())
        self.cli_act__reposition = ActionClient(self, Reposition, 'reposition', callback_group=MutuallyExclusiveCallbackGroup())

        ### GLOBAL PARAMS
        self.cancel_action = False
        self.is_fire = False
        self.cam_park_pos = None
        self.task_name = ""
        self.task_params = None
        self.sm = None
        self.state = None
        self.task_name = "idle"
        self.skill_name =""
        self.uav_pos = None
        self.uav_alt = None
        
        self.get_logger().info(" > NODE FSM__node STARTED.")
    
    ############################################################################################################################################################################################################################
    ##### TIMERS CALLBACKS ############################################################################################################################################################################################################################
    #---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    #----- Timer to send STATE  ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    def timer_cb__send_state(self):
        self.get_logger().info(f"SEND > State -> task_name:{self.task_name} skill_name:{self.skill_name}")
        msg = State()
        msg.task_name  = str(self.task_name)
        msg.skill_name = str(self.skill_name)
        self.pub__state.publish(msg)
    #---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    #----- Timer for FSM  ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    def timer_cb__init_fsm(self):
        self.get_logger().info(" > FSM started.")
        if False:
            self.call__disarm(force=True)
            self.get_logger().warn("!!!!!!!!! IT WILL CRASH !!!!!!!!!")
            self.get_logger().warn("BE CAREFUL : YOU HAVE TO PASS IN 'on_raspi' mode (PARAMS_utils.py)")
            self.get_logger().warn("!!!!!!!!! IT WILL CRASH !!!!!!!!!")
        ### Starting the FSM
        self.sm = fsm.PopeyeFSM(self)
        ### Save the FSM graph and destroy the timer
        self.sm._graph().write_png(path_DUAV+"/src/popeye/popeye//POPEYE_FSM.png")
        self.get_logger().warn(" > FSM ended.")
    def timer_cb__test(self):
        # self.get_logger().error(" OKKKK")
        pass
    #---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    #----- Timer for LAUNCH TASk  ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    # def timer_cb__launch_task(self):
    #     if self.task_name is None:
    #         self.get_logger().warn(f" >>> Task name is not published yet")
    #         return
    #     if self.sm is None:
    #         self.get_logger().warn(f" >>> FSM not started yet")
    #         return
    #     prev_task_name = self.task_name
    #     while True:
    #         if self.task_name != prev_task_name:
    #             self.get_logger().info(f" >>> Task received : {self.task_name}")
    #             prev_task_name = self.task_name

    #         if self.task_name == "reposition":
    #             sm.send("event_terminate")
    #         sleep(1)

    ############################################################################################################################################################################################################################
    ##### SUBSCRIBERS CALLBACKS ############################################################################################################################################################################################################################
    #---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    #----- Subscriber for TASK  ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    def sub_cb__task(self, msg):
        self.task_name   = msg.task_name # Type: string
        self.task_params = msg.arguments # Type: TaskParams(.msg)
        print(self.task_name)
        print("self.task_name")
    #---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    #----- Subscriber for FIRE  ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    def sub_cb__fire(self, msg):
        if msg.is_fire and not self.is_fire:
            self.is_fire  = True
            self.lat_fire = msg.lat_fire
            self.lon_fire = msg.lon_fire
            print()
            self.get_logger().warn(" >>> FIRE HAS BEEN SPOTTED <<<")
            self.get_logger().warn(f" >>> FIRE_LAT:{self.lat_fire} FIRE_LON:{self.lon_fire}<<<")
            print()
    #---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    #----- Subscriber for UAV_POSITION  ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    def sub_cb__uav_position(self, msg):
        self.uav_pos = (msg.lat, msg.lon)
        self.uav_alt = msg.alt
    #---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    #----- Subscriber for CAM FIRE POS  ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    def sub_cb__cam_fire_pos(self, msg):
        print("cam_pos")
        self.pos_cam_fire = (msg.lat, msg.lon)
        self.alt_cam_fire = msg.alt
        print()
        # self.get_logger().info(f" >>> CAM_FIRE_POS:{self.pos_cam_fire} <<<")
        # print()
    #---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    #----- Subscriber for CAM PARK POS  ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    def sub_cb__cam_park_pos(self, msg):
        print("park_pos")
        self.pos_cam_park = (msg.lat, msg.lon)
        self.alt_cam_park = msg.alt
        print()
        # self.get_logger().info(f" >>> CAM_PARK_POS:{self.pos_cam_park} <<<")
        # print()
    
    ############################################################################################################################################################################################################################
    ##### ACTIONS CLIENTS ############################################################################################################################################################################################################################
    #---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    #----- Function to call the TAKEOFF action  ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    def call__takeoff(self, alt=DEFAULT_ALT):
        self.get_logger().info(f"> Calling TAKEOFF action (alt:{alt})")
        if not self.cli_act__takeoff.wait_for_server(timeout_sec=3.0):
            self.get_logger().warn("       ... TAKEOFF action server not available")
            return False
        self.get_logger().info("       ... TAKEOFF action server available")
            
        goal__takeoff = Takeoff.Goal()
        goal__takeoff.alt = float(alt)
        goal_handle_future = self.cli_act__takeoff.send_goal_async(goal__takeoff, feedback_callback=lambda feedback_msg: 
                                                                   self.get_logger().info(f"       ... Feedback (current_alt:{feedback_msg.feedback.current_alt:.1f}, state:{feedback_msg.feedback.state})"))
        rclpy.spin_until_future_complete(self, goal_handle_future)
        if not goal_handle_future.result().accepted:
            self.get_logger().warn("       ... TAKEOFF goal rejected")
            return False
        self.get_logger().info("       ... TAKEOFF goal accepted")
        
        action_future = goal_handle_future.result().get_result_async()
        self.get_logger().info("       ... wainting for action end")
        while not action_future.done():
            rclpy.spin_once(self, timeout_sec=0.5)
            
        if not action_future.result().result.success:
            self.get_logger().warning("       ==> TAKEOFF failed")
            return False
        self.get_logger().info("       ==> TAKEOFF successful")
        self.takeoff_results = True
        return True
    #---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    #----- Function to call the REPOSITION action  ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    def call__reposition(self, lat=DEFAULT_LAT, lon=DEFAULT_LON, alt=DEFAULT_ALT):
        self.get_logger().info(f"> Calling REPOSITION action (lat:{lat}, lon:{lon}, alt:{alt})")
        if not self.cli_act__reposition.wait_for_server(timeout_sec=3.0):
            self.get_logger().warn("       ... REPOSITION action server not available")
            return False
        self.get_logger().info("       ... REPOSITION action server available")
            
        goal__reposition = Reposition.Goal()
        goal__reposition.lat = float(lat)
        goal__reposition.lon = float(lon)
        goal__reposition.alt = float(alt)
        goal_handle_future = self.cli_act__reposition.send_goal_async(goal__reposition, feedback_callback=lambda feedback_msg: 
                                                                      self.get_logger().info(f"       ... Feedback (dist_to_goal:{feedback_msg.feedback.dist_to_goal}, delta_alt:{feedback_msg.feedback.delta_alt:.1f}, time_at_position:{feedback_msg.feedback.time_at_position:.1f})"))
        rclpy.spin_until_future_complete(self, goal_handle_future)
        if not goal_handle_future.result().accepted:
            self.get_logger().warn("       ... REPOSITION goal rejected")
            return False
        self.get_logger().info("       ... REPOSITION goal accepted")
        
        action_future = goal_handle_future.result().get_result_async()
        self.get_logger().info("       ... async goal hangle sended")
        while not action_future.done():
            rclpy.spin_once(self, timeout_sec=0.5)
            
        rclpy.spin_until_future_complete(self, action_future)
        self.get_logger().info("       ... async run futur received")
        if not action_future.result().result.success:
            self.get_logger().warning("       ==> REPOSITION failed")
            return False
        self.get_logger().info("       ==> REPOSITION successful")
        return True
    #---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    #----- Function to call the LAND action  ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    def call__land(self):
        self.get_logger().info(f"> Calling LAND action")
        if not self.cli_act__land.wait_for_server(timeout_sec=3.0):
            self.get_logger().warn("       ... LAND action server not available")
            return False
        self.get_logger().info("       ... LAND action server available")
            
        goal__land = Land.Goal()
        goal_handle_future = self.cli_act__land.send_goal_async(goal__land, feedback_callback=lambda feedback_msg: 
                                                                   self.get_logger().info(f"       ... Feedback (Current_alt:{feedback_msg.feedback.current_alt:.1f}, State:{feedback_msg.feedback.state})"))
        rclpy.spin_until_future_complete(self, goal_handle_future)
        self.get_logger().info("       ... async goal handle sended")
        if not goal_handle_future.result().accepted:
            self.get_logger().warn("       ... LAND goal rejected")
            return False
        self.get_logger().info("       ... LAND goal accepted")
        
        result = goal_handle_future.result()
        if result is None:
            self.get_logger().error("       ... Goal handle future result is None!")
            return False
        action_future = goal_handle_future.result().get_result_async()
        self.get_logger().info("       ... waiting for land to finish")
        while not action_future.done():
            rclpy.spin_once(self, timeout_sec=0.5)

        self.get_logger().info("       ... async run sended")
        if not action_future.result().result.success:
            self.get_logger().warning("       ==> LAND failed")
            return False
        self.get_logger().info("       ==> LAND successful")
        return True
    
    ############################################################################################################################################################################################################################
    ##### SERVICES CLIENTS ############################################################################################################################################################################################################################
    #---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    #----- Function to call the SET_MODE service  ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    def call__set_mode(self, mode='RTL'):
        self.get_logger().info(f"> Calling SET_MODE (mode:{mode})")
        if not self.cli_srv__set_mode.wait_for_service(timeout_sec=3.0):
            self.get_logger().warning('       ... Service not available')
            return False
        self.get_logger().info("       ... SET_MODE service available")
            
        request           = SetMode.Request()
        request.mode_name = mode
        future            = self.cli_srv__set_mode.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result().success:
            self.get_logger().info(f"     -> Successful")
        else:
            self.get_logger().warning(f"     -> Failed")
    #---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    #----- Function to call the ARM service  ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    def call__arm(self, force=False):
        self.get_logger().info(f"> Calling ARM (force:{force})")
        if not self.cli_srv__arm.wait_for_service(timeout_sec=3.0):
            self.get_logger().warning('       ... Service not available')
            return False
        self.get_logger().info("       ... ARM service available")
        
        request       = Arm.Request()
        request.force = force
        future        = self.cli_srv__arm.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result().success:
            self.get_logger().info(f"     -> Successful")
        else:
            self.get_logger().warning(f"     -> Failed")
    #---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    #----- Function to call the PAYLOAD DROP service  ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    def call__payload_drop(self):
        self.get_logger().info(f"> Calling PAYLOAD DROP ")
        if not self.cli_srv__payload_drop.wait_for_service(timeout_sec=3.0):
            self.get_logger().warning('       ... Service not available')
            return False
        self.get_logger().info("       ... PAYLOAD DROP service available")
        
        request = Drop.Request()
        future  = self.cli_srv__payload_drop.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result().success:
            self.get_logger().info(f"     -> Successful")
        else:
            self.get_logger().warning(f"     -> Failed")
    #---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    #----- Function to call the PAYLOAD RELOAD service  ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    def call__payload_reload(self):
        self.get_logger().info(f"> Calling PAYLOAD RELOAD")
        if not self.cli_srv__payload_reload.wait_for_service(timeout_sec=3.0):
            self.get_logger().warning('       ... Service not available')
            return False
        self.get_logger().info("       ... PAYLOAD RELOAD service available")
        
        request = Drop.Request()
        future  = self.cli_srv__payload_reload.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result().success:
            self.get_logger().info(f"     -> Successful")
        else:
            self.get_logger().warning(f"     -> Failed")
    #---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    #----- Function to call the RTL service  ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    def call__rtl(self):
        self.get_logger().info("> Calling RTL.")
        if not self.cli_srv__rtl.wait_for_service(timeout_sec=3.0):
            self.get_logger().warning('       ... Service not available')
            return False
        self.get_logger().info("       ... RTL service available")
        
        request = Rtl.Request()
        future  = self.cli_srv__rtl.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result().success:
            self.get_logger().info(f"     -> Successful")
        else:
            self.get_logger().warning(f"     -> Failed")
    #---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    #----- Function to call the DISARM service  ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    def call__disarm(self, force=False):
        self.get_logger().info(f"> Calling DISARM (force:{force})")
        if not self.cli_srv__disarm.wait_for_service(timeout_sec=3.0):
            self.get_logger().warning('       ... Service not available')
            return False
        self.get_logger().info("       ... DISARM service available")
        
        request = Disarm.Request() 
        request.force = force
        future = self.cli_srv__disarm.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result().success:
            self.get_logger().info(f"     -> Successful")
        else:
            self.get_logger().warning(f"     -> Failed")
    #---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    #----- Function to call the TAKE PHOTO service  ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    def call__take_photo(self):
        self.get_logger().info(f"> Calling TAKE_PHOTO")
        if not self.cli_srv__take_photo.wait_for_service(timeout_sec=3.0):
            self.get_logger().warning('       ... Service not available')
            return False
        self.get_logger().info("       ... TAKE_PHOTO service available")
            
        request = TakePhoto.Request()
        future  = self.cli_srv__take_photo.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result().success:
            self.get_logger().info(f"     -> Successful")
        else:
            self.get_logger().warning(f"     -> Failed")
    #---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    #----- Function to call the TAKE VIDEO service  ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    def call__take_video(self, seconds):
        self.get_logger().info(f"> Calling TAKE_VIDEO")
        if not self.cli_srv__take_video.wait_for_service(timeout_sec=3.0):
            self.get_logger().warning('       ... Service not available')
            return False
        self.get_logger().info("       ... TAKE_VIDEO service available")
            
        request = TakeVideo.Request()
        request.seconds = float(seconds)
        future  = self.cli_srv__take_video.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result().success:
            self.get_logger().info(f"     -> Successful")
        else:
            self.get_logger().warning(f"     -> Failed")
        
#########################################################################################################################################################################################################
##### Node entry point #####################################################################################################################################################################
def main(args=None):
    rclpy.init(args=args)
    
    ### Creating the mutlithread executor
    node = FSMNode()
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=100)
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