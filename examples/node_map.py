#!/usr/bin/env python3
from __future__ import annotations

# Adds the lib directory to the Python path
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

import time
import math
import json
import numpy as np

from RPi import GPIO
import smbus2
import paho.mqtt.client as mqtt

# VL53L5CX libraries
from lib.vl53l5cx_lib.vl53l5cx import VL53L5CX
from lib.vl53l5cx_lib.api import (
    VL53L5CX_RESOLUTION_4X4,
    VL53L5CX_RESOLUTION_8X8
)

# -----------------------------------------------------------------------------
# MQTT Setup
# -----------------------------------------------------------------------------
MQTT_BROKER = "localhost"    # Change if your broker is on a different machine
MQTT_PORT = 1883
MQTT_TOPIC = "robot/tof_map"  # Publish the map data here

client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)  # Update to use VERSION2 callbacks
client.connect(MQTT_BROKER, MQTT_PORT, keepalive=60)
client.loop_start()

# -----------------------------------------------------------------------------
# ToF Sensor Setup
# -----------------------------------------------------------------------------
GPIO.setmode(GPIO.BCM)
sensor_pins = [17, 22, 27]  # LPn pins for the three sensors
for pin in sensor_pins:
    GPIO.setup(pin, GPIO.OUT)

def scan_i2c_bus(bus_number=1):
    bus = smbus2.SMBus(bus_number)
    devices = []
    for address in range(128):
        try:
            bus.write_byte(address, 0)
            devices.append(address)
        except IOError:
            pass
    return devices

def set_sensor_address(sensor, pin, new_address):
    # Pull down all LPn pins
    for p in sensor_pins:
        GPIO.output(p, GPIO.LOW)
    # Power up the current sensor
    GPIO.output(pin, GPIO.HIGH)
    time.sleep(0.1)
    # Set new I2C address
    sensor.set_i2c_address(new_address)
    # Pull up all pins again
    for p in sensor_pins:
        GPIO.output(p, GPIO.HIGH)

print("Scanning I2C bus...")
existing_addresses = scan_i2c_bus()
print(f"Found devices at: {[hex(a) for a in existing_addresses]}")

# Example addresses for the three sensors
addresses = [0x52, 0x54, 0x56]
sensors = []

for i, pin in enumerate(sensor_pins):
    if addresses[i] not in existing_addresses:
        sensor = VL53L5CX()
        set_sensor_address(sensor, pin, addresses[i])
    else:
        sensor = VL53L5CX(i2c_address=addresses[i])
    sensors.append(sensor)

# Verify that all sensors are alive
for sensor in sensors:
    if not sensor.is_alive():
        raise IOError(f"VL53L5CX at address {hex(sensor.i2c_address)} is not alive.")

print("Initialising sensors...")
USE_8X8_MODE = True  # Change to False for 4x4

# Initialize sensors
for sensor in sensors:
    sensor.init()
    if USE_8X8_MODE:
        sensor.set_resolution(VL53L5CX_RESOLUTION_8X8)
    else:
        sensor.set_resolution(VL53L5CX_RESOLUTION_4X4)
    sensor.start_ranging()

print("Sensors initialized.")

# -----------------------------------------------------------------------------
# Helper Functions for 3D Points
# -----------------------------------------------------------------------------
# Field of View
FOV_DEG = 60
OFFSET_8X8 = 3.75
OFFSET_4X4 = 7.5

if USE_8X8_MODE:
    rows_deg = np.linspace(-FOV_DEG/2 + OFFSET_8X8, FOV_DEG/2 - OFFSET_8X8, 8)
    cols_deg = np.linspace(-FOV_DEG/2 + OFFSET_8X8, FOV_DEG/2 - OFFSET_8X8, 8)
    NUM_ZONES = 64
else:
    rows_deg = np.linspace(-FOV_DEG/2 + OFFSET_4X4, FOV_DEG/2 - OFFSET_4X4, 4)
    cols_deg = np.linspace(-FOV_DEG/2 + OFFSET_4X4, FOV_DEG/2 - OFFSET_4X4, 4)
    NUM_ZONES = 16

def deg2rad(deg: float) -> float:
    return deg * math.pi / 180.0

def get_3d_points(distances_mm: list[int], sensor_index: int) -> np.ndarray:
    """
    Convert the distance readings into (x, y, z) points in world coordinates.
    sensor_index: 0 => sensor facing +60 deg, 1 => forward, 2 => -60 deg
        (Adjust these if your physical setup is different.)
    """
    # Adjust if you want to vary the sensor's mounting height:
    sensor_height_m = 0.4  # Example: 0.4m from ground

    points = []
    grid_size = int(math.sqrt(NUM_ZONES))  # 8 or 4
    for i, dist_mm in enumerate(distances_mm):
        dist_m = dist_mm * 0.001
        row_idx = i // grid_size
        col_idx = i % grid_size

        # Angles for this zone
        vert_rad  = deg2rad(rows_deg[row_idx])
        horiz_rad = deg2rad(cols_deg[col_idx])

        # Spherical → Cartesian
        x = dist_m * math.cos(vert_rad) * math.cos(horiz_rad)
        y = dist_m * math.cos(vert_rad) * math.sin(horiz_rad)
        z = dist_m * math.sin(vert_rad)
        
        # Shift for sensor height
        z += sensor_height_m

        points.append([x, y, z])

    points_np = np.array(points)

    # Rotate points to account for sensor orientation:
    #   sensor_index 0 => rotated +60 deg
    #   sensor_index 2 => rotated -60 deg
    #   sensor_index 1 => no rotation (forward)
    if points_np.size > 0:
        if sensor_index == 0:
            angle = deg2rad(60.0)
        elif sensor_index == 2:
            angle = deg2rad(-60.0)
        else:
            angle = 0.0

        if angle != 0.0:
            rot_z = np.array([
                [ math.cos(angle), -math.sin(angle), 0 ],
                [ math.sin(angle),  math.cos(angle), 0 ],
                [ 0,               0,                1 ],
            ])
            points_np = points_np @ rot_z.T

    return points_np

# -----------------------------------------------------------------------------
# Main Loop
# -----------------------------------------------------------------------------
print("Starting ToF read + MQTT publish loop...")
try:
    while True:
        all_sensor_data = []

        for s_idx, sensor in enumerate(sensors):
            if sensor.check_data_ready():
                data = sensor.get_ranging_data()

                # Get the right slice for distance readings
                distances_mm = data.distance_mm[:NUM_ZONES]
                target_status = data.target_status[:NUM_ZONES]

                # Convert to 3D points in world coordinates
                points_3d = get_3d_points(distances_mm, s_idx)

                # Build a data structure for this sensor
                # We'll also separate valid vs invalid points if you want
                valid_points = []
                invalid_points = []
                for i, (dist_mm, status) in enumerate(zip(distances_mm, target_status)):
                    # Status code 5 typically means "valid" measurement on VL53L5CX
                    if status == 5 and dist_mm != 0:
                        valid_points.append(points_3d[i].tolist())
                    else:
                        invalid_points.append(points_3d[i].tolist())

                sensor_data = {
                    "sensor_address": hex(sensor.i2c_address),
                    "sensor_index": s_idx,
                    "valid_points": valid_points,
                    "invalid_points": invalid_points,
                }
                all_sensor_data.append(sensor_data)

        # Publish all sensor data to MQTT as one JSON structure
        if all_sensor_data:
            payload = json.dumps({"sensors": all_sensor_data})
            client.publish(MQTT_TOPIC, payload)

        time.sleep(0.05)

except KeyboardInterrupt:
    print("\nInterrupted by user.")

finally:
    # Clean up
    GPIO.cleanup()
    client.loop_stop()
    client.disconnect()
    print("GPIO cleaned up, MQTT disconnected. Exiting.")
