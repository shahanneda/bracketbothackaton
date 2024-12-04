# Adds the lib directory to the Python path
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))


import json
import time
import numpy as np


from lib.odrive_uart import ODriveUART, reset_odrive
from lib.imu import FilteredMPU6050

def test_motor_direction():
    # Initialize IMU and motors
    imu = FilteredMPU6050()
    imu.calibrate()
    
    # Reset ODrive before initializing motors
    reset_odrive()
    time.sleep(3)  # Wait for ODrive to reset
    
    motor_controller = ODriveUART(port='/dev/ttyAMA1', left_axis=0, right_axis=1, dir_left=1, dir_right=1)
    
    directions = {'left': 1, 'right': 1}
    
    # Test each motor
    for name in ['left', 'right']:
        time.sleep(1)
        print(f"\nTesting {name} motor...")
        
        # Start motor in closed loop control
        if name == 'left':
            motor_controller.start_left()
            motor_controller.enable_velocity_mode_left()
        else:
            motor_controller.start_right()
            motor_controller.enable_velocity_mode_right()
            
        # Clear any errors
        if name == 'left':
            if motor_controller.check_errors_left():
                print("Clearing left motor errors...")
                motor_controller.clear_errors_left()
        else:
            if motor_controller.check_errors_right():
                print("Clearing right motor errors...")
                motor_controller.clear_errors_right()
        
        # Get baseline gyro reading
        imu.update()
        baseline_gyro = imu.gyro_RAW[2]  # Z-axis rotation
        print(f"Baseline gyro: {baseline_gyro}")
        
        # Spin motor
        if name == 'left':
            motor_controller.set_speed_rpm_left(30)
        else:
            motor_controller.set_speed_rpm_right(30)

        curr_time = time.time()
        while time.time() - curr_time < 0.5:
            time.sleep(0.01)
            imu.update()

        # Get gyro reading during spin
        spin_gyro = imu.gyro_RAW[2]
        print(f"Spin gyro: {spin_gyro}")
        
        # Stop motor
        if name == 'left':
            motor_controller.stop_left()
        else:
            motor_controller.stop_right()
        
        # Determine direction based on gyro reading
        # Positive gyro means counterclockwise rotation when viewed from above
        gyro_diff = spin_gyro - baseline_gyro
        print(f"Gyro difference: {gyro_diff}")
        
        # Set direction based on gyro reading
        # We want positive direction to be forward motion
        if name == 'left':
            directions['left'] = -1 if gyro_diff > 0 else 1
        else:
            directions['right'] = 1 if gyro_diff > 0 else -1
        
        time.sleep(0.5)  # Wait between tests
    
    # Save results to file
    with open(os.path.expanduser('~/quickstart/lib/motor_dir.json'), 'w') as f:
        json.dump(directions, f)
    
    print("\nDirection test complete!")
    print(f"Left direction: {directions['left']}, Right direction: {directions['right']}")
    print(f"Results saved to ~/quickstart/lib/motor_dir.json: {directions}")

if __name__ == '__main__':
    try:
        test_motor_direction()
    except Exception as e:
        print(f"Error occurred: {e}")
