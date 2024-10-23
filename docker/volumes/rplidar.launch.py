from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    robot_name_arg = DeclareLaunchArgument("robot_name", default_value="fastbot_1")

    return LaunchDescription(
        [
            robot_name_arg,
            Node(
                name="rplidar_composition",
                package="rplidar_ros",
                executable="rplidar_composition",
                output="screen",
                parameters=[
                    {
                        "serial_port": "/dev/ttyUSB0",
                        "serial_baudrate": 115200,  # A1 / A2
                        "frame_id": LaunchConfiguration(
                            "robot_name"
                        ),  # Just use the LaunchConfiguration here
                        "inverted": False,
                        "angle_compensate": True,
                    }
                ],
                remappings=[
                    (
                        "{}_lidar".format(LaunchConfiguration("robot_name")),
                        "frame_id",
                    )  # Remapping frame_id correctly
                ],
            ),
        ]
    )
