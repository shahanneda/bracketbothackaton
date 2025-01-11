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
robot_half_size = np.array([[0.02, 0.02, 0.7]])   # half extents in x,y,z
robot_color     = np.array([[1.0, 0.0, 0.0, 0.4]])# RGBA: red, semi-transparent

# -----------------------------------------------------------------------------
# Global Variables to Store Robot Path
# -----------------------------------------------------------------------------
robot_path = []  # List to store robot positions over time

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
                    
                    # Log the points relative to the 'robot' frame
                    rr.log(
                        f"robot/tof/sensor_{sensor_addr}/valid",
                        rr.Points3D(points_np, colors=colors, radii=radii),
                        timeless=False,
                    )
                
                # Process invalid points
                invalid_points = sensor_data["invalid_points"]
                if invalid_points:
                    points_np = np.array(invalid_points)
                    colors = np.full((points_np.shape[0], 4), [1.0, 1.0, 0.0, 0.5])  # Yellow, semi-transparent
                    radii = np.full(points_np.shape[0], 0.05)
                    
                    # Log the points relative to the 'robot' frame
                    rr.log(
                        f"robot/tof/sensor_{sensor_addr}/invalid",
                        rr.Points3D(points_np, colors=colors, radii=radii),
                        timeless=False,
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
                    # Convert grid coordinates to local coordinates
                    local_x = occupied_x * resolution + min_x + (resolution / 2)
                    local_y = occupied_y * resolution + min_y + (resolution / 2)
                    local_z = np.full_like(local_x, 0.1)  # Points at 0.1m height
                    
                    points = np.column_stack((local_x, local_y, local_z))
                    colors = np.full((len(points), 4), [0.2, 0.2, 0.2, 1.0])  # Dark gray, fully opaque
                    radii = np.full(len(points), resolution / 2)  # Half the cell size
                    
                    # Log the occupancy grid relative to the 'robot' frame
                    rr.log(
                        "robot/occupancy_grid",
                        rr.Points3D(points, colors=colors, radii=radii),
                        timeless=False,
                    )
        elif msg.topic == PATH_PLAN_TOPIC:
            # Parse the path plan message
            path_data = json.loads(msg.payload)
            path_xy = path_data["path_xy"]  # Get path coordinates relative to the robot
    
            # Convert path to numpy array for visualization
            path_points = np.array([[x, y, 0.1] for x, y in path_xy])  # Set Z to 0.1m
            
            # Log the path plan relative to the 'robot' frame
            if len(path_points) > 1:
                rr.log(
                    "robot/path_plan",
                    rr.LineStrips3D(
                        [path_points],
                        colors=[[0.0, 1.0, 0.0, 1.0]],  # Green path
                        radii=[0.02]
                    ),
                    timeless=False,
                )
                print(f"Visualized path with {len(path_points)} points")
    
        elif msg.topic == ODOMETRY_TOPIC:
            # Process odometry data
            odom_data = json.loads(msg.payload)
            x = odom_data['x']
            y = odom_data['y']
            theta = odom_data['theta']
    
            # Update the robot's transform in Rerun
            sin_theta_half = math.sin(theta / 2.0)
            cos_theta_half = math.cos(theta / 2.0)
            quat = rr.Quaternion(xyzw=[0.0, 0.0, sin_theta_half, cos_theta_half])
    
            # Log the transform from 'world' to 'robot'
            rr.log(
                "robot",
                rr.Transform3D(
                    translation=[x, y, 0.0],
                    rotation=quat,
                ),
                timeless=False,
            )
    
            # Log the robot's visualization (optional)
            robot_center = np.array([[0.0, 0.0, 0.7]])  # Robot is at the origin of its own frame
    
            # Log the box shape
            rr.log(
                "robot/geometry/extrusion",
                rr.Boxes3D(
                    centers=robot_center,
                    half_sizes=robot_half_size,
                    colors=robot_color
                ),
                timeless=False,
            )
            
            # Add capsule base
            rr.log(
                "robot/geometry/base",
                rr.Boxes3D(
                    centers=[[0.0, 0.0, 0.0825]],  # Center at base
                    half_sizes=[[0.25, 0.25, 0.075]],  # Half width/length 0.5/2 = 0.25m, height 0.15/2 = 0.075m
                    colors=robot_color
                ),
                timeless=False,
            )
            print(f"Updated robot position: x={x:.2f}, y={y:.2f}, theta={theta:.2f}")
    
            # Append the current position to the robot path (in world frame)
            robot_path.append([x, y, 0.05])  # Z-coordinate is consistent with robot_center
    
            # Log the robot's path as a LineStrips3D in world frame
            if len(robot_path) > 1:
                rr.log(
                    "world/robot_path",
                    rr.LineStrips3D(
                        [np.array(robot_path)],
                        colors=[[0.0, 0.0, 1.0, 1.0]],  # Blue color for the path
                        radii=0.01
                    ),
                    timeless=False,
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
