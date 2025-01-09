# Adds the lib directory to the Python path
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

import json
import time
import threading
import cv2
# import pyrealsense2 as rs
from sshkeyboard import listen_keyboard, stop_listening
from lib.odrive_uart import ODriveUART
import numpy as np

# Load motor directions
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

motor_controller.disable_watchdog_left()
motor_controller.disable_watchdog_right()

motor_controller.clear_errors_left()
motor_controller.clear_errors_right()

# Configure depth and color streams
# pipeline = rs.pipeline()
# config = rs.config()
# config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
# pipeline.start(config)

# Constants
SPEED = 0.2  # Speed in m/s

def capture_images():
    try:
        while True:
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if not color_frame:
                continue

            color_image = np.asanyarray(color_frame.get_data())
            rotated_image = cv2.rotate(color_image, cv2.ROTATE_90_CLOCKWISE)
            cv2.imwrite(f'wasd.png', rotated_image)
            time.sleep(0.5)
    except Exception as e:
        print(f"Error capturing images: {e}")

def press(key):
    if key.lower() == 'w':  # Forward
        motor_controller.set_speed_mps(0, SPEED, left_dir)
        motor_controller.set_speed_mps(1, SPEED, right_dir)
    elif key.lower() == 's':  # Backward
        motor_controller.set_speed_mps(0, -SPEED, left_dir)
        motor_controller.set_speed_mps(1, -SPEED, right_dir)
    elif key.lower() == 'a':  # Left turn
        motor_controller.set_speed_mps(0, -SPEED, left_dir)
        motor_controller.set_speed_mps(1, SPEED, right_dir)
    elif key.lower() == 'd':  # Right turn
        motor_controller.set_speed_mps(0, SPEED, left_dir)
        motor_controller.set_speed_mps(1, -SPEED, right_dir)
    elif key.lower() == 'q':  # Quit
        stop_listening()

def release(key):
    # Stop motors when key is released
    motor_controller.set_speed_mps(0, 0, left_dir)
    motor_controller.set_speed_mps(1, 0, right_dir)

try:
    print("WASD to control, Q to quit")
    # image_thread = threading.Thread(target=capture_images)
    # image_thread.start()
    listen_keyboard(
        on_press=press,
        on_release=release,
        until='q'
    )

except Exception as e:
    print(f"Error: {e}")
finally:
    # Stop motors on exit
    motor_controller.stop_left()
    motor_controller.stop_right()
    # pipeline.stop()
