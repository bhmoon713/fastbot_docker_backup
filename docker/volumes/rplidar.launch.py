from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    robot_name_arg = DeclareLaunchArgument("robot_name", default_value="fastbot_1")

    return LaunchDescription(
        [
            robot_name_arg,  # Add the launch argument declaration to the LaunchDescription
            Node(
                name="rplidar_composition",
                package="rplidar_ros",
                executable="rplidar_composition",
                output="screen",
                parameters=[
                    {
                        "serial_port": "/dev/ttyUSB0",
                        "serial_baudrate": 115200,  # A1 / A2
                        "frame_id": LaunchConfiguration("robot_name")
                        + "_lidar",  # Retrieve the value correctly
                        "inverted": False,
                        "angle_compensate": True,
                    }
                ],
            ),
        ]
    )
