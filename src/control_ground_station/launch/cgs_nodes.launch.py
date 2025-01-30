from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    node_MAV_manager_popeye = Node(
        package='control_ground_station',
        executable='MAV_manager_popeye',
    )

    node_MAV_manager_olive = Node(
        package='control_ground_station',
        executable='MAV_manager_olive',
    )

    ld.add_action(node_MAV_manager_popeye)
    ld.add_action(node_MAV_manager_olive)

    return ld