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
min_pwm_val = 20  # Minimum PWM value required to move

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


def same_sign(a, b):
    return a * b > 0


def wheel_vel_executer(self, left_speed, right_speed, turn_balance=2.0):
    global max_pwm_val
    global min_pwm_val
    global lPWM, rPWM, lDIR, rDIR
    global previous_lPWM, previous_rPWM

    # Convert linear speeds to PWM values
    lspeedPWM = max(
        min((abs(left_speed) / max_speed) * max_pwm_val, max_pwm_val), min_pwm_val
    )
    rspeedPWM = max(
        min((abs(right_speed) / max_speed) * max_pwm_val, max_pwm_val), min_pwm_val
    )

    # Apply turn balancing if both wheels move in the same direction
    if same_sign(left_speed, right_speed):
        if abs(right_speed) > abs(left_speed):
            factor = left_speed / right_speed
            lspeedPWM *= factor / turn_balance
            self.get_logger().info(f">>>>> Adjusted PWM LEFT: {lspeedPWM}")
        elif abs(left_speed) > abs(right_speed):
            factor = right_speed / left_speed
            rspeedPWM *= factor / turn_balance
            self.get_logger().info(f">>>>> Adjusted PWM RIGHT: {rspeedPWM}")

    # Log the calculated PWM values
    self.get_logger().info(
        f"Calculated PWM values - Left: {lspeedPWM}, Right: {rspeedPWM}"
    )

    # Handle potential NaN values
    if math.isnan(lspeedPWM) or math.isnan(rspeedPWM):
        self.get_logger().error(
            f"NaN value detected in PWM calculation. Using previous values: "
            f"previous_lPWM: {previous_lPWM}, previous_rPWM: {previous_rPWM}"
        )
        lPWM.data = previous_lPWM
        rPWM.data = previous_rPWM
    else:
        lPWM.data = int(lspeedPWM)
        rPWM.data = int(rspeedPWM)
        previous_lPWM = lPWM.data
        previous_rPWM = rPWM.data

    # Apply PWM values to the motors and log
    pwmL.ChangeDutyCycle(lPWM.data)
    pwmR.ChangeDutyCycle(rPWM.data)
    self.get_logger().info(
        f"Applied PWM values - Left PWM: {lPWM.data}, Right PWM: {rPWM.data}"
    )

    # Publish PWM values
    self.lpwm_pub.publish(lPWM)
    self.rpwm_pub.publish(rPWM)

    # Set motor directions and publish
    lDIR.data = left_speed >= 0
    rDIR.data = right_speed >= 0
    GPIO.output(leftForward, GPIO.HIGH if lDIR.data else GPIO.LOW)
    GPIO.output(leftBackward, GPIO.LOW if lDIR.data else GPIO.HIGH)
    GPIO.output(rightForward, GPIO.HIGH if rDIR.data else GPIO.LOW)
    GPIO.output(rightBackward, GPIO.LOW if rDIR.data else GPIO.HIGH)

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
