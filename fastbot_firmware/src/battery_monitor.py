#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from adafruit_ina219 import INA219
import board
import busio


class BatteryMonitorNode(Node):
    def __init__(self):
        super().__init__("battery_monitor")
        # Publisher for battery state
        self.battery_pub = self.create_publisher(BatteryState, "battery_state", 10)

        # Timer to publish data periodically
        self.timer = self.create_timer(1.0, self.publish_battery_state)  # 1 Hz

        # Setup INA219
        i2c_bus = busio.I2C(board.SCL, board.SDA)
        self.ina219 = INA219(i2c_bus)
        self.ina219.set_calibration_32V_2A()  # Default calibration

        # Battery parameters
        self.voltage_max = 12.6  # Fully charged
        self.voltage_min = 9.6  # Fully discharged

        self.get_logger().info("Battery monitor node started.")

    def publish_battery_state(self):
        try:
            # Read voltage
            voltage = self.ina219.bus_voltage + self.ina219.shunt_voltage / 1000

            # Calculate percentage
            percentage = self.calculate_battery_percentage(voltage)

            # Create BatteryState message
            msg = BatteryState()
            msg.voltage = voltage
            msg.current = float("nan")  # Current is not being measured
            msg.percentage = percentage / 100.0  # Convert to fraction
            msg.present = True
            msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
            msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_GOOD
            msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LIPO

            # Publish the message
            self.battery_pub.publish(msg)
            self.get_logger().info(
                f"Voltage: {voltage:.2f} V, Percentage: {percentage:.2f} %"
            )

        except Exception as e:
            self.get_logger().error(f"Error reading battery data: {e}")

    def calculate_battery_percentage(self, voltage):
        # Clamp voltage between min and max
        voltage = max(min(voltage, self.voltage_max), self.voltage_min)
        # Linear interpolation between voltage_min and voltage_max
        return (
            (voltage - self.voltage_min) / (self.voltage_max - self.voltage_min) * 100.0
        )


def main(args=None):
    rclpy.init(args=args)
    node = BatteryMonitorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
