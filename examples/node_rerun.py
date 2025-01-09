#!/usr/bin/env python3
from __future__ import annotations

# Adds the lib directory to the Python path
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

import json
import numpy as np
import paho.mqtt.client as mqtt
import matplotlib
import rerun as rr

# -----------------------------------------------------------------------------
# Rerun Setup
# -----------------------------------------------------------------------------
rr.init("tof_rerun_example")              # Initialize your Rerun app name
rr.connect_tcp("192.168.2.24:9876")       # Connect to Rerun Viewer over TCP

# -----------------------------------------------------------------------------
# MQTT Setup
# -----------------------------------------------------------------------------
MQTT_BROKER = "localhost"    
MQTT_PORT = 1883
MQTT_TOPIC = "robot/tof_map"  # Subscribe to the map data topic

# -----------------------------------------------------------------------------
# Color Mapping Setup
# -----------------------------------------------------------------------------
cmap = matplotlib.colormaps["coolwarm"]  # Use a colormap that transitions from blue to red
norm = matplotlib.colors.Normalize(vmin=0.0, vmax=4.0)  # 0–4 meters color range

# -----------------------------------------------------------------------------
# Logging a Simple Robot Box (Timeless)
# -----------------------------------------------------------------------------
robot_center    = np.array([[0.0, 0.0, 0.9]])     # (x=0, y=0, z=0.9)
robot_half_size = np.array([[0.04, 0.04, 0.9]])   # half extents in x,y,z
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
# MQTT Callbacks
# -----------------------------------------------------------------------------
def on_connect(client, userdata, flags, reason_code, properties):
    print(f"Connected with reason code: {reason_code}")
    client.subscribe(MQTT_TOPIC)

def on_message(client, userdata, msg):
    try:
        data = json.loads(msg.payload)
        
        # Process each sensor's data
        for sensor_data in data["sensors"]:
            sensor_addr = sensor_data["sensor_address"]
            
            # Process valid points
            valid_points = sensor_data["valid_points"]
            if valid_points:
                points_np = np.array(valid_points)
                d_m = np.linalg.norm(points_np, axis=1)  # distances in meters
                colors = cmap(norm(d_m))
                radii = np.full(points_np.shape[0], 0.05)
                
                rr.log(
                    f"world/tof/sensor_{sensor_addr}/valid",
                    rr.Points3D(points_np, colors=colors, radii=radii)
                )
            
            # Process invalid points
            invalid_points = sensor_data["invalid_points"]
            if invalid_points:
                points_np = np.array(invalid_points)
                colors = np.full((points_np.shape[0], 4), [1.0, 1.0, 0.0, 0.5])  # Yellow, semi-transparent
                radii = np.full(points_np.shape[0], 0.05)
                
                rr.log(
                    f"world/tof/sensor_{sensor_addr}/invalid",
                    rr.Points3D(points_np, colors=colors, radii=radii)
                )

    except Exception as e:
        print(f"Error processing message: {e}")

# -----------------------------------------------------------------------------
# Main Loop
# -----------------------------------------------------------------------------
def main():
    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    client.on_connect = on_connect
    client.on_message = on_message

    try:
        client.connect(MQTT_BROKER, MQTT_PORT, 60)
        print("Starting MQTT loop...")
        client.loop_forever()
        
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    finally:
        client.disconnect()
        print("MQTT disconnected. Exiting.")

if __name__ == "__main__":
    main()
