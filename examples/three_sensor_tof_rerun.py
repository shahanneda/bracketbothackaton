#!/usr/bin/env python3
from __future__ import annotations

# Adds the lib directory to the Python path
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

import time
import math
import numpy as np

from RPi import GPIO  # Import GPIO module
import smbus2        # Import smbus for I2C communication

# VL53L5CX libraries
from lib.vl53l5cx_lib.vl53l5cx import VL53L5CX
from lib.vl53l5cx_lib.api import VL53L5CX_RESOLUTION_4X4, VL53L5CX_RESOLUTION_8X8, VL53L5CX_RANGING_MODE_CONTINUOUS, VL53L5CX_RANGING_MODE_AUTONOMOUS

# Rerun SDK
import rerun as rr
import matplotlib

# -----------------------------------------------------------------------------
# Rerun Setup
# -----------------------------------------------------------------------------
rr.init("tof_rerun_example")              # Initialize your Rerun app name
rr.connect_tcp("192.168.2.24:9876")       # Connect to Rerun Viewer over TCP

# -----------------------------------------------------------------------------
# Logging a Simple Robot Box (Timeless)
# -----------------------------------------------------------------------------
# Define a rectangular box to represent your robot’s footprint
robot_center    = np.array([[0.0, 0.0, 0.9]])     # (x=0, y=0, z=0.5)
robot_half_size = np.array([[0.04, 0.04, 0.9]])     # half extents in x,y,z
robot_color     = np.array([[1.0, 0.0, 0.0, 0.4]])# RGBA: red, semi-transparent

rr.log(
    "world/robot",
    rr.Boxes3D(
        centers=robot_center,
        half_sizes=robot_half_size,
        colors=robot_color
    ),
    timeless=True,
)

# -----------------------------------------------------------------------------
# ToF Sensor Setup
# -----------------------------------------------------------------------------
GPIO.setmode(GPIO.BCM)
sensor_pins = [17, 22, 27]
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
    # Power up current sensor
    GPIO.output(pin, GPIO.HIGH)
    time.sleep(0.1)
    # Set new I2C address
    sensor.set_i2c_address(new_address)
    # Pull up all pins
    for p in sensor_pins:
        GPIO.output(p, GPIO.HIGH)

existing_addresses = scan_i2c_bus()
addresses = [0x52, 0x54, 0x56]  # Example addresses

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
        raise IOError("One of the VL53L5CX devices is not alive.")

print("Initialising...")
t = time.time()

# Configuration: Set resolution mode
USE_8X8_MODE = True  # Set to False for 4x4 mode

# Initialize sensors with the selected resolution
for sensor in sensors:
    sensor.init()
    if USE_8X8_MODE:
        sensor.set_resolution(VL53L5CX_RESOLUTION_8X8)
    else:
        sensor.set_resolution(VL53L5CX_RESOLUTION_4X4)
    # sensor.set_ranging_mode(VL53L5CX_RANGING_MODE_AUTONOMOUS)
    # sensor.set_ranging_frequency_hz(15)
    # sensor.set_integration_time_ms(10)
    # sensor.set_sharpener_percent(50)
    print(sensor.get_ranging_frequency_hz())
    print(sensor.get_integration_time_ms())
    
    sensor.start_ranging()
print(f"Initialised in {time.time() - t:.1f}s")

# -----------------------------------------------------------------------------
# Helper: Convert Distances to 3D Points
# -----------------------------------------------------------------------------
# Define FOV and angles for 4x4 and 8x8 modes
FOV_DEG = 60
OFFSET_8X8 = 3.75
OFFSET_4X4 = 7.5

# Define angles based on the mode
if USE_8X8_MODE:
    rows_deg = np.linspace(-FOV_DEG/2 + OFFSET_8X8, FOV_DEG/2 - OFFSET_8X8, 8)
    cols_deg = np.linspace(-FOV_DEG/2 + OFFSET_8X8, FOV_DEG/2 - OFFSET_8X8, 8)
else:
    rows_deg = np.linspace(-FOV_DEG/2 + OFFSET_4X4, FOV_DEG/2 - OFFSET_4X4, 4)
    cols_deg = np.linspace(-FOV_DEG/2 + OFFSET_4X4, FOV_DEG/2 - OFFSET_4X4, 4)

def deg2rad(deg: float) -> float:
    return deg * math.pi / 180.0

def get_3d_points_8x8(distances_mm: list[int], sensor_index: int) -> np.ndarray:
    """
    Convert the 64 distance readings (in mm) into 64 (x, y, z) points.
    Optionally rotate the entire set about Z for left/right sensors:
      sensor_index 0: facing forward
                   1: rotated +60 deg
                   2: rotated -60 deg
    """
    # Convert to meters and adjust for sensor height
    sensor_height_m = 0.4  # 800mm in meters
    points = []
    for i, dist_mm in enumerate(distances_mm):
        dist_m = dist_mm * 0.001
        row_idx = i // 8
        col_idx = i % 8

        # Angles for this zone
        vert_rad  = deg2rad(rows_deg[row_idx])
        horiz_rad = deg2rad(cols_deg[col_idx])

        # Spherical → Cartesian
        x = dist_m * math.cos(vert_rad) * math.cos(horiz_rad)
        y = dist_m * math.cos(vert_rad) * math.sin(horiz_rad)
        z = dist_m * math.sin(vert_rad)
        z += sensor_height_m
        points.append([x, y, z])

    points_np = np.array(points)

    # Rotate for left/right sensors
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

def get_3d_points_4x4(distances_mm: list[int], sensor_index: int) -> np.ndarray:
    """
    Convert the 16 distance readings (in mm) into 16 (x, y, z) points.
    Optionally rotate the entire set about Z for left/right sensors:
      sensor_index 0: facing forward
                   1: rotated +60 deg
                   2: rotated -60 deg
    """
    # Convert to meters and adjust for sensor height
    sensor_height_m = 0.5  # 800mm in meters
    points = []
    for i, dist_mm in enumerate(distances_mm):
        dist_m = dist_mm * 0.001
        row_idx = i // 4
        col_idx = i % 4

        # Angles for this zone
        vert_rad  = deg2rad(rows_deg[row_idx])
        horiz_rad = deg2rad(cols_deg[col_idx])

        # Spherical → Cartesian
        x = dist_m * math.cos(vert_rad) * math.cos(horiz_rad)
        y = dist_m * math.cos(vert_rad) * math.sin(horiz_rad)
        z = dist_m * math.sin(vert_rad)

        # Adjust for sensor height
        z -= sensor_height_m
        points.append([x, y, z])

    points_np = np.array(points)

    # Rotate for left/right sensors
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

def get_3d_points(distances_mm: list[int], sensor_index: int) -> np.ndarray:
    """
    Convert the distance readings into (x, y, z) points.
    Adjusts for the selected resolution mode.
    """
    if USE_8X8_MODE:
        return get_3d_points_8x8(distances_mm, sensor_index)
    else:
        return get_3d_points_4x4(distances_mm, sensor_index)

# -----------------------------------------------------------------------------
# Color Mapping Setup
# -----------------------------------------------------------------------------
# We'll color by distance. Adjust range as needed.
cmap = matplotlib.colormaps["coolwarm"]  # Use a colormap that transitions from blue to red
norm = matplotlib.colors.Normalize(vmin=0.0, vmax=4.0)  # 0–4 meters color range

# -----------------------------------------------------------------------------
# Main Loop: Log sensor data to Rerun
# -----------------------------------------------------------------------------
print("Starting sensor read + Rerun logging loop...")
try:
    loop_count = 0
    previous_time = 0.0

    # Initialize previous distances to None
    previous_distances = [None] * len(sensors)

    while True:  # Loop until interrupted
        for s_idx, sensor in enumerate(sensors):
            if sensor.check_data_ready():
                ranging_data = sensor.get_ranging_data()

                # Use the correct number of distances based on the mode
                if USE_8X8_MODE:
                    distances_mm = ranging_data.distance_mm[:64]
                    target_status = ranging_data.target_status[:64]
                else:
                    distances_mm = ranging_data.distance_mm[:16]
                    target_status = ranging_data.target_status[:16]

                # Count invalid points and track status codes
                status_counts = {}
                dead_points = 0
                for i, (dist_mm, status) in enumerate(zip(distances_mm, target_status)):
                    if status != 5:
                        if status not in status_counts:
                            status_counts[status] = 0
                        status_counts[status] += 1
                    elif previous_distances[s_idx] is not None and dist_mm == previous_distances[s_idx][i]:
                        dead_points += 1

                print(f"Sensor {hex(sensor.i2c_address)}:")
                if status_counts:
                    print("  Invalid status counts:")
                    for status, count in status_counts.items():
                        print(f"    Status {status}: {count} points")
                if dead_points > 0:
                    print(f"  Dead points: {dead_points}")
                if not status_counts and dead_points == 0:
                    print("  All points valid")

                # Convert all points to 3D
                points = get_3d_points(distances_mm, s_idx)
                
                if points.size > 0:
                    rr.set_time_seconds("stable_time", loop_count)  # time for Rerun
                    d_m = np.linalg.norm(points, axis=1)  # distances in meters
                    
                    # Split points into valid and invalid
                    valid_mask = [(status == 5 and (previous_distances[s_idx] is None or dist_mm != previous_distances[s_idx][i])) 
                                for i, (dist_mm, status) in enumerate(zip(distances_mm, target_status))]
                    
                    valid_points = points[valid_mask]
                    invalid_points = points[~np.array(valid_mask)]
                    
                    if valid_points.size > 0:
                        # Color valid points by distance
                        valid_d_m = np.linalg.norm(valid_points, axis=1)
                        valid_colors = cmap(norm(valid_d_m))
                        valid_radii = np.full(valid_points.shape[0], 0.05)
                        
                        # Log valid points
                        rr.log(
                            f"world/tof/sensor_{hex(sensor.i2c_address)}/valid",
                            rr.Points3D(valid_points, colors=valid_colors, radii=valid_radii)
                        )
                    
                    if invalid_points.size > 0:
                        # Color invalid points yellow with 50% transparency
                        invalid_colors = np.full((invalid_points.shape[0], 4), [1.0, 1.0, 0.0, 0.5])
                        invalid_radii = np.full(invalid_points.shape[0], 0.05)
                        
                        # Log invalid points under separate path
                        rr.log(
                            f"world/tof/sensor_{hex(sensor.i2c_address)}/invalid",
                            rr.Points3D(invalid_points, colors=invalid_colors, radii=invalid_radii)
                        )

                # Update previous distances
                previous_distances[s_idx] = distances_mm

                loop_count += 1

        time.sleep(0.01)

except KeyboardInterrupt:
    print("\nInterrupted by user.")

finally:
    # Clean up
    GPIO.cleanup()
    print("GPIO cleaned up. Exiting.")
