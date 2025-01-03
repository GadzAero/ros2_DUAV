from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    controller_1 = Node(
        package='joy',
        executable='joy_node',
        name='joy1_node',
        namespace='controller_1',
        parameters=[{'device_id': 0}],
        remappings=[('/joy', '/controller_1/joy')],
    )

    controller_2 = Node(
        package='joy',
        executable='joy_node',
        name='joy2_node',
        namespace='controller_2',
        parameters=[{'device_id': 1}],
        remappings=[('/joy', '/controller_2/joy')],
    )

    ld.add_action(controller_1)
    ld.add_action(controller_2)

    return ld