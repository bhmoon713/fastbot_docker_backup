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

leftEn = 13  #   Blue
rightEn = 12  #   Brown

leftBackward = 5  #   Green
leftForward = 6  #   Yellow
rightForward = 16  #   Orange
rightBackward = 20  #   Red

motor_rpm = 205  #   max rpm of motor on full voltage
wheel_diameter = 0.065  #   in meters
wheel_separation = 0.17  #   in meters
max_pwm_val = 100  #   100 for Raspberry Pi , 255 for Arduino
min_pwm_val = 35  #   Minimum PWM value that is needed for the robot to move

previous_lPWM = min_pwm_val
previous_rPWM = min_pwm_val

wheel_radius = wheel_diameter / 2
circumference_of_wheel = 2 * pi * wheel_radius
max_speed = (circumference_of_wheel * motor_rpm) / 60  #   m/sec
max_ang = (max_speed * 2) / wheel_separation  #   rad/sec

lPWM = Int32()
rPWM = Int32()
lDIR = Bool()
rDIR = Bool()

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


def stop(self):
    global lPWM, rPWM, lDIR, rDIR

    # print('stopping')
    pwmL.ChangeDutyCycle(0)
    GPIO.output(leftForward, GPIO.HIGH)
    GPIO.output(leftBackward, GPIO.HIGH)
    pwmR.ChangeDutyCycle(0)
    GPIO.output(rightForward, GPIO.HIGH)
    GPIO.output(rightBackward, GPIO.HIGH)
    lPWM.data = 0
    rPWM.data = 0
    lDIR.data = True
    rDIR.data = True
    self.lpwm_pub.publish(lPWM)
    self.rpwm_pub.publish(rPWM)
    self.ldir_pub.publish(lDIR)
    self.rdir_pub.publish(rDIR)


def same_sign(a, b):
    return a * b > 0


def wheel_vel_executer(self, left_speed, right_speed, turn_balance=2.0):
    global max_pwm_val
    global min_pwm_val
    global lPWM, rPWM, lDIR, rDIR
    global previous_lPWM, previous_rPWM

    # Steps for lspeedPWD selection
    aux1_l = abs(left_speed) / max_speed
    aux2_l = (aux1_l) * max_pwm_val
    aux3_l = min(aux2_l, max_pwm_val)
    lspeedPWM = max(aux3_l, min_pwm_val)

    rspeedPWM = max(
        min(((abs(right_speed) / max_speed) * max_pwm_val), max_pwm_val), min_pwm_val
    )

    # Upgrade to make it
    # We only apply this when the two wheels are going in the same dirrection
    if same_sign(left_speed, right_speed):
        # We then have to see which one is bgger to consider it as refference
        # R > L
        if abs(right_speed) > abs(left_speed):
            factor = left_speed / right_speed
            # We adjust the L PWD to go slower
            # We apply also a balance factor becuase some wheels dont behave equaly, so we try to accentuate more.
            lspeedPWM *= factor / turn_balance
            self.get_logger().info(">>>>> ADJUSTED PWD LEFT=" + str(lspeedPWM))
        # L > R
        elif abs(right_speed) < abs(left_speed):
            factor = right_speed / left_speed
            # We adjust the R PWD to go slower
            # We apply also a balance factor becuase some wheels dont behave equaly, so we try to accentuate more.
            rspeedPWM *= factor / turn_balance
            self.get_logger().info(">>>>> ADJUSTED PWD RIGHT=" + str(rspeedPWM))
        # R == L
        else:
            pass
    self.get_logger().info(f"Pre-check: lspeedPWM: {lspeedPWM}, rspeedPWM: {rspeedPWM}")

    # Check for NaN and handle appropriately
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
        # Update the previous values with the current valid ones
        previous_lPWM = lPWM.data
        previous_rPWM = rPWM.data

    # pwmL.ChangeDutyCycle(lspeedPWM)
    pwmL.ChangeDutyCycle(lPWM.data)
    # pwmR.ChangeDutyCycle(rspeedPWM)
    pwmR.ChangeDutyCycle(rPWM.data)

    self.get_logger().info("####################")
    self.get_logger().info(
        "LW SPEED=" + str(left_speed) + ", LEFT PWM=" + str(lspeedPWM)
    )
    self.get_logger().info(
        "RW SPEED=" + str(right_speed) + ", RIGHT PWM=" + str(rspeedPWM)
    )
    self.get_logger().info("####################")

    self.lpwm_pub.publish(lPWM)
    self.rpwm_pub.publish(rPWM)

    if left_speed >= 0:
        GPIO.output(leftForward, GPIO.HIGH)
        GPIO.output(leftBackward, GPIO.LOW)
        lDIR.data = True
        self.ldir_pub.publish(lDIR)
    else:
        GPIO.output(leftForward, GPIO.LOW)
        GPIO.output(leftBackward, GPIO.HIGH)
        lDIR.data = False
        self.ldir_pub.publish(lDIR)

    if right_speed >= 0:
        GPIO.output(rightForward, GPIO.HIGH)
        GPIO.output(rightBackward, GPIO.LOW)
        rDIR.data = True
        self.rdir_pub.publish(rDIR)
    else:
        GPIO.output(rightForward, GPIO.LOW)
        GPIO.output(rightBackward, GPIO.HIGH)
        rDIR.data = False
        self.rdir_pub.publish(rDIR)


class Differential(Node):
    def __init__(self, args):
        super().__init__("differential")

        self.argument_parsing(args)

        self.vel_subscription = self.create_subscription(
            Twist, self.args.robot_name_value + "/cmd_vel", self.callback, 10
        )
        self.vel_subscription
        self.lpwm_pub = self.create_publisher(Int32, "lpwm", 10)
        self.rpwm_pub = self.create_publisher(Int32, "rpwm", 10)
        self.ldir_pub = self.create_publisher(Bool, "ldir", 10)
        self.rdir_pub = self.create_publisher(Bool, "rdir", 10)

        self.joint_state_pub = self.create_publisher(
            JointState, self.args.robot_name_value + "/joint_states", 10
        )
        self.init_wheel_joints_state()

        # Timer to update and publish wheel position
        self.timer = self.create_timer(0.5, self.timer_callback)

    def argument_parsing(self, args):
        parser = argparse.ArgumentParser(description="Dummy Example for Arguments use")

        parser.add_argument(
            "-robot_name_value",
            type=str,
            metavar="box_bot_default",
            default="boxbot_1",
            help="Name of robot",
        )

        self.args = parser.parse_args(args[1:])

    def init_wheel_joints_state(self, init_value=0.0):
        self.left_vel = 0.0
        self.right_vel = 0.0
        self.left_wheel_pos = 0.0
        self.right_wheel_pos = 0.0
        self.last_time = self.get_clock().now()
        # Publish the joint states
        now = self.get_clock().now()
        self.joint_state_msg = JointState()

        frame_id_name = self.args.robot_name_value + "base_link"
        self.joint_state_msg.header = Header(stamp=now.to_msg(), frame_id=frame_id_name)

        ml_name = self.args.robot_name_value + "_motor_left"
        mr_name = self.args.robot_name_value + "_motor_right"
        self.joint_state_msg.name = [ml_name, mr_name]

        self.joint_state_msg.position = [self.left_wheel_pos, self.right_wheel_pos]
        self.joint_state_msg.velocity = [self.left_vel, self.right_vel]
        self.joint_state_msg.effort = []
        self.joint_state_pub.publish(self.joint_state_msg)

    def callback(self, data):

        now = self.get_clock().now()
        elapsed_time = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        global wheel_radius
        global wheel_separation

        linear_vel = data.linear.x  # Linear Velocity of Robot
        angular_vel = data.angular.z  # Angular Velocity of Robot

        self.get_logger().info(
            f"Received linear_vel: {linear_vel}, angular_vel: {angular_vel}"
        )

        VrplusVl = 2 * linear_vel
        VrminusVl = angular_vel * wheel_separation

        self.right_vel = (
            VrplusVl + VrminusVl
        ) / 2  # right wheel velocity along the ground
        self.left_vel = (
            VrplusVl - self.right_vel
        )  # left wheel velocity along the ground

        self.get_logger().info(
            f"Calculated left_vel: {self.left_vel}, right_vel: {self.right_vel}"
        )
        # print (str(left_vel)+"\t"+str(right_vel))

        if self.left_vel == 0.0 and self.right_vel == 0.0:
            stop(self)
        else:
            wheel_vel_executer(self, self.left_vel, self.right_vel)

        # Publish the joint states
        # Update wheel positions based on velocity and time elapsed
        self.right_wheel_pos += self.right_vel * elapsed_time
        self.left_wheel_pos += self.left_vel * elapsed_time
        self.joint_state_msg.header = Header(stamp=now.to_msg(), frame_id="base_link")
        self.joint_state_msg.position = [self.left_wheel_pos, self.right_wheel_pos]
        self.joint_state_msg.velocity = [self.left_vel, self.right_vel]
        self.joint_state_pub.publish(self.joint_state_msg)

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
    print("args===" + str(args))

    args_without_ros = rclpy.utilities.remove_ros_args(args)
    print("clean ROS args===" + str(args_without_ros))

    differential_drive = Differential(args_without_ros)
    rclpy.spin(differential_drive)
    differential_drive.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    print("Boxbot Differential Drive Initialized with following Params-")
    print("Motor Max RPM:\t" + str(motor_rpm) + " RPM")
    print("Wheel Diameter:\t" + str(wheel_diameter) + " m")
    print("Wheel Separation:\t" + str(wheel_separation) + " m")
    print("Robot Max Speed:\t" + str(max_speed) + " m/sec")
    print("Max Angular Speed:\t" + str(max_ang) + " rad/sec")
    main()
