# Servo Motor Calibration Guide

## Requirements
Before starting, ensure you have the necessary packages installed:

```bash
sudo apt install python3-pip
pip install RPi.GPIO
```

## Safe Operating Ranges
For 6V hobby servos, these are the recommended duty cycle ranges:

### Standard Values
- **Middle Position (90°)**: 7.5%
- **Minimum (towards 0°)**: 5.0%
- **Maximum (towards 180°)**: 10.0%

## Testing Procedure

### Initial Code Values
```python
# Start with these conservative values
self.close_duty = 7.5  # middle position
self.open_duty = 10.0  # one extreme
```

### Calibration Steps
1. **Start at middle position**
   - Begin with 7.5% duty cycle
   - This is the safest starting point

2. **Test minimum position**
   - Gradually decrease from 7.5% towards 5.0%
   - Move in 0.5% increments
   - Stop if servo strains

3. **Test maximum position**
   - Gradually increase from 7.5% towards 10.0%
   - Move in 0.5% increments
   - Stop if servo strains

### Safety Guidelines
- Always start from middle position (7.5%)
- Make small adjustments (0.5% increments)
- Listen for straining/grinding sounds
- Stop immediately if:
    - Servo makes unusual noises
    - Movement seems forced
    - Servo gets hot
- Verify power supply is adequate
- Check wiring before assuming range issues

## Code Example
```python
# Safe testing values
middle = 7.5   # 90 degrees
min_duty = 5.0 # towards 0 degrees
max_duty = 10.0 # towards 180 degrees
```

## Current Implementation
The default values in the code (6% to 12%) are within reasonable range but start with the more conservative 5-10% range for initial testing.

### Parameter Adjustment
To modify the servo range during runtime:
```bash
ros2 run your_package gripper_action_server --ros-args -p close_duty:=5.5 -p open_duty:=11.5
```
