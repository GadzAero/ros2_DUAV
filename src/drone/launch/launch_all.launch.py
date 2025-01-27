from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    node_analyse_CAM = Node(
        package='drone',
        executable='analyse_CAM',
    )

    node_write_MAV = Node(
        package='drone',
        executable='write_MAV',
    )

    ld.add_action(node_analyse_CAM)
    ld.add_action(node_write_MAV)

    return ld