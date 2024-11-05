import rclpy
from rclpy.node import Node
from serial_motor_demo_msgs.msg import MotorCommand, MotorVels, EncoderVals
import time
import math
import serial
from threading import Lock
from typing import List, Optional


class MotorDriver(Node):

    def __init__(self) -> None:
        # Initialize the node with name 'motor_driver'
        super().__init__("motor_driver")

        # Setup parameters for encoder counts per revolution (CPR) and loop rate
        self.declare_parameter("encoder_cpr", value=0)
        if self.get_parameter("encoder_cpr").value == 0:
            print("WARNING! ENCODER CPR SET TO 0!!")

        self.declare_parameter("loop_rate", value=0)
        if self.get_parameter("loop_rate").value == 0:
            print("WARNING! LOOP RATE SET TO 0!!")

        # Initialize parameters for serial communication settings
        self.declare_parameter("serial_port", value="/dev/ttyACM0")
        self.serial_port: str = self.get_parameter("serial_port").value

        self.declare_parameter("baud_rate", value=57600)
        self.baud_rate: int = self.get_parameter("baud_rate").value

        # Enable debugging for serial commands if set in parameters
        self.declare_parameter("serial_debug", value=False)
        self.debug_serial_cmds: bool = self.get_parameter("serial_debug").value
        if self.debug_serial_cmds:
            print("Serial debug enabled")

        # Setup ROS topics and publishers/subscribers
        self.subscription = self.create_subscription(
            MotorCommand, "motor_command", self.motor_command_callback, 10
        )

        self.speed_pub = self.create_publisher(MotorVels, "motor_vels", 10)
        self.encoder_pub = self.create_publisher(EncoderVals, "encoder_vals", 10)

        # Initialize member variables for encoder readings and motor speeds
        self.last_enc_read_time: float = time.time()
        self.last_m1_enc: int = 0
        self.last_m2_enc: int = 0
        self.m1_spd: float = 0.0
        self.m2_spd: float = 0.0

        # Mutex for thread-safe serial communication
        self.mutex = Lock()

        # Open serial communication with the motor controller
        print(f"Connecting to port {self.serial_port} at {self.baud_rate}.")
        self.conn: serial.Serial = serial.Serial(
            self.serial_port, self.baud_rate, timeout=1.0
        )
        print(f"Connected to {self.conn}")

    def send_pwm_motor_command(self, mot_1_pwm: float, mot_2_pwm: float) -> None:
        # Send PWM command for motor speeds
        self.send_command(f"o {int(mot_1_pwm)} {int(mot_2_pwm)}")

    def send_feedback_motor_command(
        self, mot_1_ct_per_loop: float, mot_2_ct_per_loop: float
    ) -> None:
        # Send feedback command with counts per loop
        self.send_command(f"m {int(mot_1_ct_per_loop)} {int(mot_2_ct_per_loop)}")

    def send_encoder_read_command(self) -> List[int]:
        # Send command to read encoder values
        resp = self.send_command("e")
        if resp:
            # Parse and return encoder values as integers
            return [int(raw_enc) for raw_enc in resp.split()]
        return []

    def motor_command_callback(self, motor_command: MotorCommand) -> None:
        if motor_command.is_pwm:
            # If command is PWM, send PWM values directly
            self.send_pwm_motor_command(
                motor_command.mot_1_req_rad_sec, motor_command.mot_2_req_rad_sec
            )
        else:
            # If command is not PWM, calculate counts per loop
            scaler: float = (
                (1 / (2 * math.pi))
                * self.get_parameter("encoder_cpr").value
                * (1 / self.get_parameter("loop_rate").value)
            )
            mot1_ct_per_loop: float = motor_command.mot_1_req_rad_sec * scaler
            mot2_ct_per_loop: float = motor_command.mot_2_req_rad_sec * scaler
            # Send feedback motor command with computed counts per loop
            self.send_feedback_motor_command(mot1_ct_per_loop, mot2_ct_per_loop)

    def check_encoders(self) -> None:
        # Read encoder values and publish updated speeds
        resp = self.send_encoder_read_command()
        if resp:
            new_time: float = time.time()
            time_diff: float = new_time - self.last_enc_read_time
            self.last_enc_read_time = new_time

            # Calculate encoder differences
            m1_diff: int = resp[0] - self.last_m1_enc
            self.last_m1_enc = resp[0]
            m2_diff: int = resp[1] - self.last_m2_enc
            self.last_m2_enc = resp[1]

            # Calculate radian speed per count
            rads_per_ct: float = 2 * math.pi / self.get_parameter("encoder_cpr").value
            self.m1_spd = m1_diff * rads_per_ct / time_diff
            self.m2_spd = m2_diff * rads_per_ct / time_diff

            # Publish speeds
            spd_msg = MotorVels()
            spd_msg.mot_1_rad_sec = self.m1_spd
            spd_msg.mot_2_rad_sec = self.m2_spd
            self.speed_pub.publish(spd_msg)

            # Publish encoder values
            enc_msg = EncoderVals()
            enc_msg.mot_1_enc_val = self.last_m1_enc
            enc_msg.mot_2_enc_val = self.last_m2_enc
            self.encoder_pub.publish(enc_msg)

    def send_command(self, cmd_string: str) -> Optional[str]:
        # Lock the mutex to ensure thread safety
        self.mutex.acquire()
        try:
            cmd_string += "\r"
            self.conn.write(cmd_string.encode("utf-8"))  # Send command
            if self.debug_serial_cmds:
                print("Sent: " + cmd_string)

            # Read response until carriage return is received
            c: str = ""
            value: str = ""
            while c != "\r":
                c = self.conn.read(1).decode("utf-8")
                if c == "":  # Handle serial timeout
                    print("Error: Serial timeout on command: " + cmd_string)
                    return None
                value += c

            value = value.strip("\r")  # Strip carriage return

            if self.debug_serial_cmds:
                print("Received: " + value)
            return value
        finally:
            # Release the mutex after command execution
            self.mutex.release()

    def close_conn(self) -> None:
        # Close the serial connection
        self.conn.close()


def main(args: Optional[List[str]] = None) -> None:
    # Initialize ROS 2 Python client library
    rclpy.init(args=args)

    # Create an instance of MotorDriver node
    motor_driver = MotorDriver()

    # Run the node loop at a fixed rate
    rate = motor_driver.create_rate(2)
    while rclpy.ok():
        # Process callbacks and check encoders
        rclpy.spin_once(motor_driver)
        motor_driver.check_encoders()

    # Cleanup and shutdown
    motor_driver.close_conn()
    motor_driver.destroy_node()
    rclpy.shutdown()
