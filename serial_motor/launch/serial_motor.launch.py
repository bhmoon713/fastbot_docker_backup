from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="serial_motor",
                executable="motor_driver",
                name="serial_motor_driver",
                # namespace='your_namespace',
                # parameters=[{'your_parameter_key': 'your_parameter_value'}],
                # remappings=[('/input/topic', '/output/topic')],
                output="screen",
            ),
            # Add more nodes as needed
        ]
    )
