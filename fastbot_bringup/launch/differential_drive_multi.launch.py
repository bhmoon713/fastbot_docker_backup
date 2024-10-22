from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import OpaqueFunction


def launch_setup(context, *args, **kwargs):
    # Access the robot_name_value using LaunchConfiguration
    robot_name = LaunchConfiguration("robot_name_value")

    # Create the Node for the differential drive with parameters
    differential_drive_node = Node(
        package="fastbot_firmware",
        executable="differential_multi.py",
        name="differential_drive_publisher",
        parameters=[{"robot_name_value": robot_name}],
    )

    return [differential_drive_node]  # Return nodes directly


def generate_launch_description():
    # Declare the robot_name_value launch argument
    robot_name_arg = DeclareLaunchArgument(
        "robot_name_value", default_value="fastbot_X"
    )

    # Construct the LaunchDescription with the necessary elements
    return LaunchDescription([robot_name_arg, OpaqueFunction(function=launch_setup)])


import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():

    robot_name_value_arg = DeclareLaunchArgument(
        "robot_name_value", default_value="fastbot_1"
    )

    robot_name_value_f = LaunchConfiguration("robot_name_value")

    differential_drive_node = Node(
        package="fastbot_firmware",
        executable="differential_multi.py",
        name="differential_drive_publisher",
        output="screen",
        emulate_tty=True,
        arguments=["-robot_name_value", robot_name_value_f],
    )

    # create and return launch description object
    return LaunchDescription([robot_name_value_arg, differential_drive_node])
