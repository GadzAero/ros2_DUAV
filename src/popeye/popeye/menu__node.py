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
from interfaces.msg import Task

############################################################################################################################################################################################################################
##### Class defining MenuNode ############################################################################################################################################################################################################################
class MenuNode(Node):
    def __init__(self):
        super().__init__('MenuNode', namespace='POPEYE')
        
        ### ROS2 Callbacks
        ## Publishers
        self.pub__task = self.create_publisher(Task, 'task', 10, callback_group=MutuallyExclusiveCallbackGroup())
        ## Timer callbacks
        self.create_timer(0.05, self.timer_cb__menu, callback_group=MutuallyExclusiveCallbackGroup())
        self.create_timer(0.5, self.timer_cb__pub_task, callback_group=MutuallyExclusiveCallbackGroup())

        ### General parameter
        self.task_msg = Task(task_name="idle")
        
        self.get_logger().info("NODE MenuNode STARTED.")

    ############################################################################################################################################################################################################################
    ##### TIMER CALLBACKS ############################################################################################################################################################################################################################
    #---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    #----- Function to print the menu  ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    def timer_cb__menu(self):
        while True:
            choice = menu_utils.task_menu()
            next_task_msg = menu_utils.create_task_msg(choice)
            if not next_task_msg is None:
                self.task_msg = next_task_msg
    #---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    #----- Function to publish the TASK msg periodically  ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    def timer_cb__pub_task(self):
        self.pub__task.publish(self.task_msg)
    

############################################################################################################################################################################################################################
##### Node entry point ############################################################################################################################################################################################################################
def main(args=None):
    rclpy.init(args=args)
    
    ### Creating the mutlithread executor
    node = MenuNode()
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=3)
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
