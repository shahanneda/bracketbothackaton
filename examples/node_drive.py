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
SPEED = 0.2  

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

def on_connect(client, userdata, flags, rc):
    """Callback for when the client receives a CONNACK response from the server."""
    print(f"[MQTT] Connected with result code {rc}")
    client.subscribe(MQTT_TOPIC)
    print(f"[MQTT] Subscribed to topic: {MQTT_TOPIC}")

def on_message(client, userdata, msg):
    """
    Callback for when a PUBLISH message is received from the server.
    We expect JSON messages or simple text commands:
        - forward
        - backward
        - left
        - right
        - stop
    """
    try:
        payload = msg.payload.decode().strip()
        print(f"[MQTT] Received message on {msg.topic}: {payload}")

        # If the payload might be JSON, you could parse it:
        # data = json.loads(payload)
        # command = data.get("command", "").lower()

        # But for simplicity, let's just treat the payload as a string command:
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

def forward():
    """
    Drive both motors forward at SPEED m/s.
    """
    motor_controller.set_speed_mps(0, SPEED, left_dir)
    motor_controller.set_speed_mps(1, SPEED, right_dir)
    print("Driving forward.")

def back():
    """
    Drive both motors backward at SPEED m/s.
    """
    motor_controller.set_speed_mps(0, -SPEED, left_dir)
    motor_controller.set_speed_mps(1, -SPEED, right_dir)
    print("Driving backward.")

def turn_left():
    """
    Turn left by driving left motor backward and right motor forward.
    """
    motor_controller.set_speed_mps(0, -SPEED, left_dir)
    motor_controller.set_speed_mps(1, SPEED, right_dir)
    print("Turning left.")

def turn_right():
    """
    Turn right by driving left motor forward and right motor backward.
    """
    motor_controller.set_speed_mps(0, SPEED, left_dir)
    motor_controller.set_speed_mps(1, -SPEED, right_dir)
    print("Turning right.")

def stop():
    """
    Stop both motors.
    """
    motor_controller.set_speed_mps(0, 0, left_dir)
    motor_controller.set_speed_mps(1, 0, right_dir)
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
