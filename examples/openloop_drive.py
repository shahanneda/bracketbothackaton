# Adds the lib directory to the Python path
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from lib.odrive_uart import ODriveUART
import json
import time
import math
from lib.imu import FilteredMPU6050

# Initialize IMU
imu = FilteredMPU6050()
imu.calibrate()

# Load motor directions from config
try:
    with open(os.path.expanduser('~/quickstart/lib/motor_dir.json'), 'r') as f:
        motor_dirs = json.load(f)
        left_dir = motor_dirs['left']
        right_dir = motor_dirs['right']
except Exception as e:
    raise Exception("Error reading motor_dir.json")

# Initialize ODrive
motor_controller = ODriveUART(port='/dev/ttyAMA1', left_axis=0, right_axis=1, 
                             dir_left=left_dir, dir_right=right_dir)

# Start motors and set to velocity mode
motor_controller.start_left()
motor_controller.start_right()
motor_controller.enable_velocity_mode_left()
motor_controller.enable_velocity_mode_right()

try:
    # Drive forward 1 meter
    WHEEL_DIAMETER = 0.165  # meters
    WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * math.pi
    METERS_TARGET = 1  # Target distance in meters
    TARGET_ROTATIONS = METERS_TARGET / WHEEL_CIRCUMFERENCE
    
    # Get initial position
    initial_pos = motor_controller.get_position_turns_left()
    target_pos = initial_pos + TARGET_ROTATIONS
    
    # Drive forward at 0.2 m/s
    motor_controller.set_speed_mps_left(0.2)
    motor_controller.set_speed_mps_right(0.2)
    
    # Monitor position until we've gone 1 meter
    while True:
        current_pos = motor_controller.get_position_turns_left()
        print(f"Current position: {current_pos}, Target position: {target_pos}")
        if current_pos >= target_pos:
            break
        time.sleep(0.01)
    
    # Stop
    motor_controller.stop_left()
    motor_controller.stop_right()
    time.sleep(0.5)
    
    # Get initial yaw for 90-degree turn
    _, _, initial_yaw = imu.get_orientation()
    # Normalize initial yaw to [0, 360)
    initial_yaw = initial_yaw % 360
    if initial_yaw < 0:
        initial_yaw += 360

    # Initialize unwrapped yaw
    previous_yaw = initial_yaw
    unwrapped_yaw = 0.0  # Start unwrapped yaw at zero
    target_unwrapped_yaw = 90.0  # We want to turn 90 degrees

    # Turn in place by driving wheels in opposite directions
    TURN_SPEED = 0.2
    YAW_THRESHOLD = 2.0  # Degrees of tolerance for stopping
    # Monitor yaw until we've turned 90 degrees
    stable_start_time = None
    while True:
        _, _, current_yaw = imu.get_orientation()
        # Normalize current yaw to [0, 360)
        current_yaw = current_yaw % 360
        if current_yaw < 0:
            current_yaw += 360

        # Calculate delta_yaw
        delta_yaw = current_yaw - previous_yaw
        if delta_yaw > 180:
            delta_yaw -= 360
        elif delta_yaw < -180:
            delta_yaw += 360

        # Update unwrapped yaw
        unwrapped_yaw += delta_yaw
        previous_yaw = current_yaw

        # Calculate yaw difference to target
        yaw_diff = target_unwrapped_yaw - unwrapped_yaw

        print(f"Current unwrapped yaw: {unwrapped_yaw}, Target unwrapped yaw: {target_unwrapped_yaw}")
        print(f"Yaw diff: {yaw_diff}")

        # **Adjusted Motor Commands**
        # Reverse the motor speed signs
        if yaw_diff > 0:  # Need to turn clockwise
            motor_controller.set_speed_mps_left(-TURN_SPEED)
            motor_controller.set_speed_mps_right(TURN_SPEED)
        else:  # Need to turn counter-clockwise
            motor_controller.set_speed_mps_left(TURN_SPEED)
            motor_controller.set_speed_mps_right(-TURN_SPEED)

        # Check if we're within threshold
        if abs(yaw_diff) < YAW_THRESHOLD:
            if stable_start_time is None:
                stable_start_time = time.time()
            elif time.time() - stable_start_time >= 1.0:
                break
        else:
            stable_start_time = None
            
        time.sleep(0.01)

    # Stop the motors after the turn
    motor_controller.stop_left()
    motor_controller.stop_right()

except Exception as e:
    print(f"Error: {e}")
finally:
    # Ensure the motors are stopped on exit
    motor_controller.stop_left()
    motor_controller.stop_right()
