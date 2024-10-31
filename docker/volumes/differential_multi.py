#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32, Bool
import RPi.GPIO as GPIO
import time
import math
import argparse
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from math import pi

# Motor GPIO Pin Definitions
leftEn = 13  # PWM pin for left motor
rightEn = 12  # PWM pin for right motor
leftBackward = 5  # Direction control for left motor backward
leftForward = 6  # Direction control for left motor forward
rightForward = 16  # Direction control for right motor forward
rightBackward = 20  # Direction control for right motor backward

# Motor Specifications
motor_rpm = 205  # Max RPM of motor at full voltage
wheel_diameter = 0.065  # Wheel diameter in meters
wheel_separation = 0.17  # Distance between wheels in meters
max_pwm_val = 100  # Maximum PWM value (for Raspberry Pi)
min_pwm_val = 35  # Minimum PWM value required to move

# Global variables
previous_lPWM = min_pwm_val
previous_rPWM = min_pwm_val
wheel_radius = wheel_diameter / 2
circumference_of_wheel = 2 * pi * wheel_radius
max_speed = (circumference_of_wheel * motor_rpm) / 60  # m/sec
max_ang = (max_speed * 2) / wheel_separation  # rad/sec

lPWM = Int32()
rPWM = Int32()
lDIR = Bool()
rDIR = Bool()

# GPIO setup
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

GPIO.setup(leftEn, GPIO.OUT)
GPIO.setup(rightEn, GPIO.OUT)
GPIO.setup(leftForward, GPIO.OUT)
GPIO.setup(leftBackward, GPIO.OUT)
GPIO.setup(rightForward, GPIO.OUT)
GPIO.setup(rightBackward, GPIO.OUT)

pwmL = GPIO.PWM(leftEn, 100)
pwmL.start(0)
pwmR = GPIO.PWM(rightEn, 100)
pwmR.start(0)


class Differential(Node):
    def __init__(self, args):
        super().__init__("differential")
        self.argument_parsing(args)

        # Subscription to velocity commands
        self.vel_subscription = self.create_subscription(
            Twist, self.args.robot_name_value + "/cmd_vel", self.callback, 10
        )

        # Publisher for PWM and direction
        self.lpwm_pub = self.create_publisher(Int32, "lpwm", 10)
        self.rpwm_pub = self.create_publisher(Int32, "rpwm", 10)
        self.ldir_pub = self.create_publisher(Bool, "ldir", 10)
        self.rdir_pub = self.create_publisher(Bool, "rdir", 10)

        # Joint states publication
        self.joint_state_pub = self.create_publisher(
            JointState, self.args.robot_name_value + "/joint_states", 10
        )
        self.init_wheel_joints_state()

        # Timer for updating wheel positions
        self.timer = self.create_timer(0.5, self.timer_callback)

    def argument_parsing(self, args):
        parser = argparse.ArgumentParser(description="Arguments for Differential Drive")
        parser.add_argument(
            "-robot_name_value",
            type=str,
            metavar="box_bot_default",
            default="fastbot_1",
            help="Name of the robot",
        )
        self.args = parser.parse_args(args[1:])

    def init_wheel_joints_state(self):
        self.left_vel = 0.0
        self.right_vel = 0.0
        self.left_wheel_pos = 0.0
        self.right_wheel_pos = 0.0
        self.last_time = self.get_clock().now()

        now = self.get_clock().now()
        self.joint_state_msg = JointState()
        frame_id_name = self.args.robot_name_value + "base_link"
        self.joint_state_msg.header = Header(stamp=now.to_msg(), frame_id=frame_id_name)

        ml_name = self.args.robot_name_value + "_motor_left"
        mr_name = self.args.robot_name_value + "_motor_right"
        self.joint_state_msg.name = [ml_name, mr_name]
        self.joint_state_msg.position = [self.left_wheel_pos, self.right_wheel_pos]
        self.joint_state_msg.velocity = [self.left_vel, self.right_vel]
        self.joint_state_pub.publish(self.joint_state_msg)

    def callback(self, data):
        now = self.get_clock().now()
        elapsed_time = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        linear_vel = data.linear.x  # Linear velocity of the robot
        angular_vel = data.angular.z  # Angular velocity of the robot

        self.get_logger().info(
            f"Received linear_vel: {linear_vel}, angular_vel: {angular_vel}"
        )

        # Calculate wheel velocities
        VrplusVl = 2 * linear_vel
        VrminusVl = angular_vel * wheel_separation
        self.right_vel = (VrplusVl + VrminusVl) / 2  # Right wheel velocity
        self.left_vel = VrplusVl - self.right_vel  # Left wheel velocity

        self.get_logger().info(
            f"Calculated left_vel: {self.left_vel}, right_vel: {self.right_vel}"
        )

        if self.left_vel == 0.0 and self.right_vel == 0.0:
            self.stop()
        else:
            self.wheel_vel_executer(self.left_vel, self.right_vel)

        # Update joint states
        self.right_wheel_pos += self.right_vel * elapsed_time
        self.left_wheel_pos += self.left_vel * elapsed_time
        self.joint_state_msg.header = Header(stamp=now.to_msg(), frame_id="base_link")
        self.joint_state_msg.position = [self.left_wheel_pos, self.right_wheel_pos]
        self.joint_state_msg.velocity = [self.left_vel, self.right_vel]
        self.joint_state_pub.publish(self.joint_state_msg)

    def wheel_vel_executer(self, left_speed, right_speed):
        global lPWM, rPWM, lDIR, rDIR

        # Calculate PWM values
        lspeedPWM = max(
            min((abs(left_speed) / max_speed) * max_pwm_val, max_pwm_val), min_pwm_val
        )
        rspeedPWM = max(
            min((abs(right_speed) / max_speed) * max_pwm_val, max_pwm_val), min_pwm_val
        )

        lPWM.data = int(lspeedPWM)
        rPWM.data = int(rspeedPWM)

        # Set directions and publish
        if left_speed >= 0:
            GPIO.output(leftForward, GPIO.HIGH)
            GPIO.output(leftBackward, GPIO.LOW)
            lDIR.data = True
        else:
            GPIO.output(leftForward, GPIO.LOW)
            GPIO.output(leftBackward, GPIO.HIGH)
            lDIR.data = False

        if right_speed >= 0:
            GPIO.output(rightForward, GPIO.HIGH)
            GPIO.output(rightBackward, GPIO.LOW)
            rDIR.data = True
        else:
            GPIO.output(rightForward, GPIO.LOW)
            GPIO.output(rightBackward, GPIO.HIGH)
            rDIR.data = False

        # Change PWM duty cycles
        pwmL.ChangeDutyCycle(lPWM.data)
        pwmR.ChangeDutyCycle(rPWM.data)

        # Publish PWM and direction messages
        self.lpwm_pub.publish(lPWM)
        self.rpwm_pub.publish(rPWM)
        self.ldir_pub.publish(lDIR)
        self.rdir_pub.publish(rDIR)

    def stop(self):
        # Stop motors by setting PWM to 0
        pwmL.ChangeDutyCycle(0)
        pwmR.ChangeDutyCycle(0)
        GPIO.output(leftForward, GPIO.LOW)
        GPIO.output(leftBackward, GPIO.LOW)
        GPIO.output(rightForward, GPIO.LOW)
        GPIO.output(rightBackward, GPIO.LOW)

        lPWM.data = 0
        rPWM.data = 0
        self.lpwm_pub.publish(lPWM)
        self.rpwm_pub.publish(rPWM)

    def timer_callback(self):
        now = self.get_clock().now()
        elapsed_time = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        # Update wheel positions based on previously calculated velocities
        self.right_wheel_pos += self.right_vel * elapsed_time
        self.left_wheel_pos += self.left_vel * elapsed_time

        # Update and publish the joint states
        self.joint_state_msg.header.stamp = now.to_msg()
        self.joint_state_msg.position = [self.left_wheel_pos, self.right_wheel_pos]
        self.joint_state_msg.velocity = [self.left_vel, self.right_vel]
        self.joint_state_pub.publish(self.joint_state_msg)


def main(args=None):
    rclpy.init(args=args)

    args_without_ros = rclpy.utilities.remove_ros_args(args)

    differential_drive = Differential(args_without_ros)
    rclpy.spin(differential_drive)
    differential_drive.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    print("FastBot Differential Drive Initialized with following Params-")
    print("Motor Max RPM: ", motor_rpm)
    print("Wheel Diameter: ", wheel_diameter)
    print("Max Speed: ", max_speed)
    main()
