from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    node_analyse_CAM = Node(
        package='olive',
        executable='analyse_CAM',
    )

    node_MAV_manager = Node(
        package='olive',
        executable='MAV_manager',
    )

    ld.add_action(node_analyse_CAM)
    ld.add_action(node_MAV_manager)

    return ld