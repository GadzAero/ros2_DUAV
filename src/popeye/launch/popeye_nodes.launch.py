from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    node_MAV_manager = Node(
        package='popeye',
        executable='MAV_manager',
    )

    ld.add_action(node_MAV_manager)

    return ld