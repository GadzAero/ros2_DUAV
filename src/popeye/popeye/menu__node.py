#!/usr/bin/env python3

# General importation
from time import sleep
# Import ROS2 utils
import rclpy
import rclpy.executors
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
# Import menu utils
import popeye.menu__utils as menu_utils
# Import Intefaces
from interfaces.msg import Task, State, FeedbackParams

############################################################################################################################################################################################################################
##### Class defining MenuNode ############################################################################################################################################################################################################################
class MenuNode(Node):
    def __init__(self):
        super().__init__('MenuNode', namespace='POPEYE')
        
        ### ROS2 Callbacks
        ## Timer callbacks
        self.create_timer(0.05, self.timer_cb__menu, callback_group=MutuallyExclusiveCallbackGroup())
        self.create_timer(0.5, self.timer_cb__pub_task, callback_group=MutuallyExclusiveCallbackGroup())
        ## Publishers
        self.pub__task = self.create_publisher(Task, 'task', 10, callback_group=MutuallyExclusiveCallbackGroup())
        ## Subscribers
        self.create_subscription(State, 'state', self.sub_cb__state, 10, callback_group=MutuallyExclusiveCallbackGroup())

        ### General parameter
        self.task_msg = Task(task_name="idle")
        self.task_name = ""
        self.skill_name = ""
        
        self.get_logger().info("NODE MenuNode STARTED.")

    #---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    #----- Timer to print the menu  ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    def timer_cb__menu(self):
        while True:
            sleep(3)
            self.get_logger().info(f" > Received: State -> task_name:{self.task_name}, skill_name:{self.skill_name}")
            if self.task_name == "idle" or self.task_name == "":
                choice = menu_utils.task_menu()
                next_task_msg = menu_utils.create_task_msg(choice)
                if not next_task_msg is None:
                    self.task_msg = next_task_msg
            elif self.task_name == "pause":
                choice = menu_utils.pause_menu()
                next_task_msg = menu_utils.create_pause_msg(choice)
                if not next_task_msg is None:
                    self.task_msg = next_task_msg
            else:
                choice = menu_utils.cancel_menu()
                next_task_msg = menu_utils.create_state_msg(choice)
                if not next_task_msg is None:
                    self.task_msg = next_task_msg
                
    #---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    #----- Timer to publish the TASK msg periodically  ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    def timer_cb__pub_task(self):
        self.pub__task.publish(self.task_msg)
    #---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    #----- Subsriber to get the state  ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    def sub_cb__state(self, msg):
        self.task_name = msg.task_name
        self.skill_name = msg.skill_name
        # self.get_logger().info(f" > Received: State -> task_name:{self.task_name}, skill_name:{self.skill_name}")

############################################################################################################################################################################################################################
##### Node entry point ############################################################################################################################################################################################################################
def main(args=None):
    rclpy.init(args=args)
    
    ### Creating the mutlithread executor
    node = MenuNode()
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
