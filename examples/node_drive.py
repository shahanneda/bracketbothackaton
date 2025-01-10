#!/usr/bin/env python3

# Adds the lib directory to the Python path
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

import json
import time
import traceback
import paho.mqtt.client as mqtt

from lib.odrive_uart import ODriveUART

# ------------------------------------------------------------------------------------
# Constants
# ------------------------------------------------------------------------------------
MQTT_BROKER_ADDRESS = "localhost"
MQTT_TOPIC = "robot/drive"  # Adjust topic name as needed

# Default driving speed in m/s. Adjust to taste.
LINEAR_SPEED = 0.3  
ANGULAR_SPEED = 1.2

# Robot parameters
WHEEL_RADIUS = 0.0825   # meters (adjust based on your robot's wheel radius)
WHEEL_BASE = 0.4      # meters (distance between the wheels)

# ------------------------------------------------------------------------------------
# Motor Setup
# ------------------------------------------------------------------------------------
try:
    # Load motor directions from JSON file
    with open(os.path.expanduser('~/quickstart/lib/motor_dir.json'), 'r') as f:
        motor_dirs = json.load(f)
        left_dir = motor_dirs['left']
        right_dir = motor_dirs['right']
except Exception as e:
    raise Exception("Error reading motor_dir.json") from e

# Initialize the motor controller
motor_controller = ODriveUART(
    port='/dev/ttyAMA1', 
    left_axis=0, 
    right_axis=1,
    dir_left=left_dir, 
    dir_right=right_dir
)

# Start motors and enable velocity mode
motor_controller.start_left()
motor_controller.start_right()
motor_controller.enable_velocity_mode_left()
motor_controller.enable_velocity_mode_right()

# Optionally disable any watchdog to prevent unexpected stops
motor_controller.disable_watchdog_left()
motor_controller.disable_watchdog_right()

# Clear any existing errors
motor_controller.clear_errors_left()
motor_controller.clear_errors_right()

# ------------------------------------------------------------------------------------
# MQTT Callbacks
# ------------------------------------------------------------------------------------

def on_connect(client, userdata, flags, reason_code, properties):
    """Callback for when the client receives a CONNACK response from the server."""
    print(f"[MQTT] Connected with result code {reason_code}")
    client.subscribe(MQTT_TOPIC)
    print(f"[MQTT] Subscribed to topic: {MQTT_TOPIC}")

def on_message(client, userdata, msg):
    """
    Callback for when a PUBLISH message is received from the server.
    Now handles JSON messages containing 'linear_velocity' and 'angular_velocity',
    as well as simple text commands.
    """
    try:
        payload = msg.payload.decode().strip()
        print(f"[MQTT] Received message on {msg.topic}: {payload}")

        # Try to parse JSON
        try:
            data = json.loads(payload)
            # Handle data containing 'linear_velocity' and 'angular_velocity'
            if 'linear_velocity' in data and 'angular_velocity' in data:
                lin_vel = data['linear_velocity']
                ang_vel = data['angular_velocity']
                set_velocity(lin_vel, ang_vel)
                return
        except json.JSONDecodeError:
            pass  # Not JSON, treat as string command

        command = payload.lower()

        if command == "forward":
            forward()
        elif command == "back":
            back()
        elif command == "left":
            turn_left()
        elif command == "right":
            turn_right()
        elif command == "stop":
            stop()
        else:
            print(f"Unknown command: {command}")

    except Exception as e:
        print(f"Error handling message: {e}")
        traceback.print_exc()

# ------------------------------------------------------------------------------------
# Motor Control Functions
# ------------------------------------------------------------------------------------

def set_velocity(linear_velocity, angular_velocity):
    """
    Sets the motor speeds based on desired linear and angular velocities.
    Assumes differential drive robot.
    """
    # Compute wheel linear velocities
    v_left = linear_velocity - (WHEEL_BASE / 2.0) * angular_velocity
    v_right = linear_velocity + (WHEEL_BASE / 2.0) * angular_velocity

    # Set the speeds using motor_controller
    motor_controller.set_speed_mps(0, v_left, left_dir)
    motor_controller.set_speed_mps(1, v_right, right_dir)
    print(f"Setting velocities - Left: {v_left} m/s, Right: {v_right} m/s")

def forward():
    """
    Drive both motors forward at SPEED m/s.
    """
    set_velocity(LINEAR_SPEED, 0.0)

def back():
    """
    Drive both motors backward at SPEED m/s.
    """
    set_velocity(-LINEAR_SPEED, 0.0)

def turn_left():
    """
    Turn left by setting a positive angular velocity.
    """
    set_velocity(0.0, ANGULAR_SPEED)

def turn_right():
    """
    Turn right by setting a negative angular velocity.
    """
    set_velocity(0.0, -ANGULAR_SPEED)

def stop():
    """
    Stop both motors.
    """
    set_velocity(0.0, 0.0)
    print("Stopping.")

# ------------------------------------------------------------------------------------
# Main Entry Point
# ------------------------------------------------------------------------------------

if __name__ == "__main__":
    # Create MQTT client and set callbacks
    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    client.on_connect = on_connect
    client.on_message = on_message

    try:
        print("[MQTT] Connecting to broker...")
        client.connect(MQTT_BROKER_ADDRESS)
        client.loop_start()  # Start non-blocking loop

        print("[drive.py] Listening for commands. Press Ctrl+C to exit.")

        # Keep the script running
        while True:
            time.sleep(1)

    except KeyboardInterrupt:
        print("\n[drive.py] Interrupted by user.")

    except Exception as e:
        print(f"[drive.py] Error: {e}")
        traceback.print_exc()

    finally:
        # On exit, stop the motors for safety
        stop()

        # Clean up MQTT
        client.loop_stop()
        client.disconnect()

        # Optionally re-enable watchodg if you want
        # motor_controller.enable_watchdog_left()
        # motor_controller.enable_watchdog_right()

        # Clear errors or perform any other shutdown tasks
        motor_controller.clear_errors_left()
        motor_controller.clear_errors_right()

        print("[drive.py] Shutdown complete.")
