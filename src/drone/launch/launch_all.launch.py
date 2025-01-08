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
    node_pub_MAV = Node(
        package='drone',
        executable='pub_MAV',
    )

    node_select_mission_type = Node(
        package='drone',
        executable='select_mission_type',
    )
    node_determine_plane_state = Node(
        package='drone',
        executable='determine_plane_state',
    )
    node_create_path = Node(
        package='drone',
        executable='create_path',
    )
    node_create_costmap = Node(
        package='drone',
        executable='create_costmap',
    )

    ld.add_action(node_analyse_CAM)
    ld.add_action(node_write_MAV)
    ld.add_action(node_pub_MAV)
    ld.add_action(node_select_mission_type)
    ld.add_action(node_determine_plane_state)
    ld.add_action(node_create_path)
    ld.add_action(node_create_costmap)

    return ld