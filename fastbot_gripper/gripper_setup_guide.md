# Robot Gripper Setup and Calibration Guide

## Table of Contents
- [Introduction](#introduction)
- [Hardware Setup](#hardware-setup)
- [Power Supply Considerations](#power-supply-considerations)
- [Calibration Procedures](#calibration-procedures)
- [Software Configuration](#software-configuration)
- [Safety Measures](#safety-measures)
- [Troubleshooting](#troubleshooting)

## Introduction

This guide addresses the problems of inconsistent gripper positions across multiple robots and the potential hazard of overheating power cables. Proper setup and calibration of servo-based grippers are essential for consistent operation and to prevent hardware damage.

## Hardware Setup

### Servo Motor Selection

- **Use identical servo models** across all robots to ensure consistent performance
- **Check servo specifications** for:
  - Operating voltage (typically 4.8-6V)
  - Stall current (the maximum current draw when the servo is blocked)

### Wiring Guidelines

- **Use appropriate gauge wire** for the current requirements:
  - For servos drawing up to 1A: 22 AWG minimum
  - For servos drawing 1-2A: 20 AWG minimum
  - For servos drawing over 2A: 18 AWG or heavier
- **Minimize wire length** to reduce resistance and voltage drop
- **Use proper connectors**:
  - Ensure Dupont connectors are crimped properly and make solid contact
  - Consider using JST connectors rated for higher current instead of Dupont for power connections
- **Separate signal and power wiring** when possible

## Power Supply Considerations

### Selecting the Right Power Supply

- **Capacity**: Calculate the total current needed:
  - Each servo may draw 0.5-2A at peak load (check specifications)
  - For 3 servos, use a power supply rated for at least 3-6A
- **Voltage Regulation**: Use a regulated 5V power supply
- **Connection**: Add bulk capacitors (100-1000μF) near the power distribution point to handle current spikes

### Avoiding Voltage Drops

- Melting cables are often a sign of excessive current draw or high resistance
- **Add decoupling capacitors** (100-220μF) near each servo
- **Use a separate power supply** for servos, don't power them from the Raspberry Pi's 5V line

### Current Protection

- **Add a fuse or polyfuse** (3-5A) to the main power line
- Consider using a **current-limiting power supply** or add current-limiting circuitry

## Calibration Procedures

### Initial Setup

1. Before connecting to the Raspberry Pi, use a servo tester to identify:
   - Minimum duty cycle without stalling
   - Maximum duty cycle that doesn't strain the servo
   - Middle position duty cycle

2. Record these values for each servo to identify inconsistencies

### Finding Safe PWM Values

1. **Create a calibration script**:

```python
#!/usr/bin/env python3
import RPi.GPIO as GPIO
import time

# Setup
servo_pin = 17
GPIO.setmode(GPIO.BCM)
GPIO.setup(servo_pin, GPIO.OUT)
pwm = GPIO.PWM(servo_pin, 100)  # 100 Hz frequency

# Start with 0 duty cycle
pwm.start(0)

try:
    while True:
        # Get user input for duty cycle
        duty = float(input("Enter duty cycle (6-12, 0 to exit): "))
        if duty == 0:
            break
            
        # Apply duty cycle
        pwm.ChangeDutyCycle(duty)
        time.sleep(1)
        # Set to 0 to prevent holding current
        pwm.ChangeDutyCycle(0)
        
except KeyboardInterrupt:
    pass
finally:
    pwm.stop()
    GPIO.cleanup()
```

2. **Calibration Procedure**:
   - Start with a low duty cycle (6) and gradually increase
   - Note the exact position for fully open and fully closed
   - Ensure there's no straining or stalling at the limits

### Standardizing Across Robots

1. Create a calibration table for each robot:

| Robot ID | Fully Open Duty | Fully Closed Duty | Rest Duty |
|----------|----------------|-------------------|-----------|
| Robot 1  | 10.5           | 6.2               | 0         |
| Robot 2  | 11.0           | 6.5               | 0         |
| Robot 3  | 10.8           | 6.3               | 0         |

2. Update each robot's parameters to match its specific calibration needs

## Software Configuration

### Updating the ROS Parameters

Modify your launch file or parameter configuration to set the correct values for each robot:

```python
# Example launch file snippet
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='fastbot_gripper',
            executable='gripper_as.py',
            name='gripper_action_server',
            parameters=[{
                'close_duty': 6.3,  # Calibrated value for this specific robot
                'open_duty': 10.8,  # Calibrated value for this specific robot
            }],
            output='screen',
        )
    ])
```

### Adding Current Monitoring (Optional)

Consider modifying your code to monitor current draw:

```python
# Add this import
import board
import adafruit_ina219

# In your __init__ method
self.current_sensor = adafruit_ina219.INA219(board.I2C())
self.max_current = 1.0  # Amperes, adjust based on your servo

# In your execute_callback before changing PWM
current = self.current_sensor.current
if current > self.max_current:
    self.get_logger().warning(f"Current too high: {current}A")
    # Take protective action
```

## Safety Measures

### Preventing Overheating

1. **Add rest periods** between gripper operations
2. **Set PWM to 0** when not actively moving to reduce holding current
3. **Monitor servo temperature** if possible and shut down if too hot

### Circuit Protection

1. **Add current limiting** - a series resistor (1-2Ω) can help limit peak current
2. **Use polyfuses** that automatically reset after cooling down
3. **Add reverse polarity protection** - a diode in series with power

### Operational Guidelines

1. **Initial testing** - operate at reduced duty cycles until confident in settings
2. **Periodic inspection** - check wiring and connectors for signs of heating
3. **Limit continuous operation** - implement cooldown periods in software

## Troubleshooting

### Common Issues and Solutions

| Problem | Possible Causes | Solutions |
|---------|----------------|-----------|
| Melting cables | Excessive current, poor connections | Check for shorts, use thicker wires, ensure solid connections |
| Inconsistent positions | Different calibration, mechanical differences | Individual calibration, physical adjustments |
| Servo stalling | Duty cycle out of range, mechanical obstruction | Check PWM limits, inspect mechanism |
| Servo buzzing | PWM noise, inadequate power | Filter power supply, ensure adequate current capacity |

### When to Replace Components

- **Cables**: If insulation shows any signs of melting or discoloration
- **Servo motors**: If they make unusual noises, get unusually hot, or lose position accuracy
- **Power supply**: If it can't maintain stable voltage under load

---

By following this guide, you should be able to safely operate your grippers with consistent performance across multiple robots while avoiding the risk of overheating cables or damaging components.

