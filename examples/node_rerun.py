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

# Import math for trigonometric functions
import math

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
MQTT_TOPIC = "robot/tof_map"         # Subscribe to the map data topic
PATH_PLAN_TOPIC = "robot/local_path"  # Subscribe to the path plan topic
ODOMETRY_TOPIC = "robot/odometry"     # Subscribe to the odometry data

# -----------------------------------------------------------------------------
# Color Mapping Setup
# -----------------------------------------------------------------------------
cmap = matplotlib.colormaps["coolwarm"]  # Use a colormap that transitions from blue to red
norm = matplotlib.colors.Normalize(vmin=0.0, vmax=4.0)  # 0–4 meters color range

# -----------------------------------------------------------------------------
# Logging a Simple Robot Box (Timeless)
# -----------------------------------------------------------------------------
robot_half_size = np.array([[0.04, 0.04, 0.9]])   # half extents in x,y,z
robot_color     = np.array([[1.0, 0.0, 0.0, 0.4]])# RGBA: red, semi-transparent

# -----------------------------------------------------------------------------
# MQTT Callbacks
# -----------------------------------------------------------------------------
def on_connect(client, userdata, flags, reason_code, properties=None):
    print(f"Connected with reason code: {reason_code}")
    client.subscribe([
        (MQTT_TOPIC, 0),
        (PATH_PLAN_TOPIC, 0),
        (ODOMETRY_TOPIC, 0),  # Subscribe to the odometry topic
    ])

def on_message(client, userdata, msg):
    try:
        if msg.topic == MQTT_TOPIC:
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

            # Add occupancy grid visualization
            if "occupancy_grid" in data:
                grid_info = data["occupancy_grid"]
                grid = np.array(grid_info["data"])
                resolution = grid_info["resolution"]
                min_x = grid_info["min_x"]
                min_y = grid_info["min_y"]
                
                # Create points for occupied cells (where grid == 0)
                occupied_y, occupied_x = np.where(grid == 0)
                
                if len(occupied_x) > 0:
                    # Convert grid coordinates to world coordinates
                    world_x = occupied_x * resolution + min_x + (resolution / 2)
                    world_y = occupied_y * resolution + min_y + (resolution / 2)
                    world_z = np.full_like(world_x, 0.1)  # Points at 0.1m height
                    
                    points = np.column_stack((world_x, world_y, world_z))
                    colors = np.full((len(points), 4), [0.2, 0.2, 0.2, 1.0])  # Dark gray, fully opaque
                    radii = np.full(len(points), resolution / 2)  # Half the cell size
                    
                    rr.log(
                        "world/occupancy_grid",
                        rr.Points3D(points, colors=colors, radii=radii)
                    )
        elif msg.topic == PATH_PLAN_TOPIC:
            # Parse the path plan message
            path_data = json.loads(msg.payload)
            path_xy = path_data["path_xy"]  # Get world coordinates

            # Convert path to numpy array for visualization
            path_points = np.array([[x, y, 0.1] for x, y in path_xy])  # Set Z to 0.1m
            
            # Create line segments between consecutive points
            if len(path_points) > 1:
                rr.log(
                    "world/path_plan",
                    rr.LineStrips3D(
                        [path_points],  # Wrap path_points in a list to create a single continuous strip
                        colors=[[0.0, 1.0, 0.0, 1.0]],  # Green path
                        radii=[0.02]  # Line thickness
                    )
                )
                print(f"Visualized path with {len(path_points)} points")

        elif msg.topic == ODOMETRY_TOPIC:
            # Process odometry data
            odom_data = json.loads(msg.payload)
            x = odom_data['x']
            y = odom_data['y']
            theta = odom_data['theta']

            # Update the robot's position in Rerun
            robot_center = np.array([[x, y, 0.45]])  # Update robot's center position

            # Convert theta (yaw) to quaternion
            # Quaternion representing rotation around Z-axis by angle theta
            sin_theta_half = math.sin(theta / 2.0)
            cos_theta_half = math.cos(theta / 2.0)
            quat = rr.Quaternion(xyzw=[0.0, 0.0, sin_theta_half, cos_theta_half])

            # Log the robot's position and orientation
            rr.log(
                "world/robot",
                rr.Boxes3D(
                    centers=robot_center,
                    half_sizes=robot_half_size,
                    quaternions=[quat],
                    colors=robot_color
                ),
                timeless=False,  # Allow the robot's state to update over time
            )
            print(f"Updated robot position: x={x}, y={y}, theta={theta}")

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
