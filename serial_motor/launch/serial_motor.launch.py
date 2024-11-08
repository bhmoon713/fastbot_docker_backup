from launch_ros.actions import Node
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, OpaqueFunction


def launch_setup(context, *args, **kwargs):
    """
    Sets up and returns the configuration for a ROS 2 node based on launch arguments.

    Args:
        context: Launch context that provides access to the launch arguments.
        *args: Additional arguments (unused).
        **kwargs: Additional keyword arguments (unused).

    Returns:
        List of Node actions to be launched.
    """

    # Access top-level launch argument 'robot_name' defined in generate_launch_description.
    robot_name = LaunchConfiguration("robot_name").perform(context)

    # Define a ROS 2 Node for the motor driver with parameters such as the serial port and baud rate.
    driver_node = Node(
        package="serial_motor",
        executable="motor_driver",
        name="serial_motor_driver",
        namespace=robot_name,
        parameters=[
            {
                "serial_port": "/dev/ttyACM0",  # Port where the device is connected.
                "baud_rate": "57600",  # Communication speed for the serial connection.
                "loop_rate": "30",  # Frequency (Hz) at which PID loop spins.
                "encoder_cpr": "2500",  # Encoder counts per revolution.
            }
        ],
        # remappings=[('/input/topic', '/output/topic')],
        output="screen",  # Output node logs to the screen.
    )

    # Return the driver node as a list for integration with launch description.
    return [driver_node]


def generate_launch_description():
    """
    Generates the launch description and declares required launch arguments.

    Returns:
        LaunchDescription: The launch configuration with arguments and nodes.
    """

    # Declare a launch argument for the robot's name with a default value.
    robot_name_arg = DeclareLaunchArgument("robot_name", default_value="robot_X")

    # Create a LaunchDescription with the argument and setup function.
    return LaunchDescription([robot_name_arg, OpaqueFunction(function=launch_setup)])
