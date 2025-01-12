#!/usr/bin/env python3

import json
import time
import math
import paho.mqtt.client as mqtt

# MQTT setup
MQTT_BROKER_ADDRESS = "localhost"

# Topics
MQTT_TOPIC_PATH_PLAN = "robot/local_path"
MQTT_TOPIC_DRIVE_CMD = "robot/drive"
MQTT_TOPIC_ODOMETRY = "robot/odometry"  # Assuming odometry data is published here
MQTT_TOPIC_PATH_COMPLETED = "robot/path_completed"  # New topic

# Robot parameters
MAX_LINEAR_SPEED = 0.2    # m/s
MAX_ANGULAR_SPEED = 1.0   # rad/s

# Control gains (tune these as needed)
K_LINEAR = 0.5
K_ANGULAR = 1.5

# State thresholds
ANGLE_THRESHOLD = 0.1   # radians (~5.7 degrees)
DISTANCE_THRESHOLD = 0.1  # meters

# Global variables
path_xy = []        # List of (x, y) waypoints
current_index = 0   # Index of current waypoint

robot_x = 0.0       # Robot's current x position
robot_y = 0.0       # Robot's current y position
robot_th = 0.0      # Robot's current heading in radians

# State variable
state = 'ROTATING'  # Initial state

def on_path_message(client, userdata, msg):
    """
    Callback for receiving new paths.
    """
    global path_xy, current_index, state
    payload = msg.payload.decode()
    data = json.loads(payload)
    new_path_xy = data.get('path_xy', [])

    path_xy = new_path_xy
    current_index = 0   # Reset current index
    state = 'ROTATING'  # Start with rotating to first waypoint
    print("[node_drivepath.py] Updated path with {} waypoints.".format(len(path_xy)))

def on_odometry_message(client, userdata, msg):
    """
    Callback for receiving robot's odometry data.
    """
    global robot_x, robot_y, robot_th
    payload = msg.payload.decode()
    data = json.loads(payload)
    robot_x = data.get('x', robot_x)
    robot_y = data.get('y', robot_y)
    robot_th = data.get('theta', robot_th)

def on_message(client, userdata, msg):
    """
    General message handler to dispatch based on topic.
    """
    if msg.topic == MQTT_TOPIC_PATH_PLAN:
        on_path_message(client, userdata, msg)
    elif msg.topic == MQTT_TOPIC_ODOMETRY:
        on_odometry_message(client, userdata, msg)

def main():
    global path_xy, current_index, state
    global robot_x, robot_y, robot_th

    # Initialize MQTT client
    client = mqtt.Client()
    client.on_message = on_message
    client.connect(MQTT_BROKER_ADDRESS)

    # Subscribe to path and odometry topics
    client.subscribe(MQTT_TOPIC_PATH_PLAN)
    client.subscribe(MQTT_TOPIC_ODOMETRY)
    print("[node_drivepath.py] Subscribed to topics.")

    client.loop_start()

    print("[node_drivepath.py] Started path follower.")

    try:
        while True:
            # Allow time for MQTT messages to be processed
            time.sleep(0.1)

            if path_xy and current_index < len(path_xy):
                # Get the next waypoint
                goal_x, goal_y = path_xy[current_index]

                # Get the robot's current position
                rx, ry, rth = robot_x, robot_y, robot_th

                # Compute the distance and angle to the waypoint
                dx = goal_x - rx
                dy = goal_y - ry

                distance = math.hypot(dx, dy)
                angle_to_goal = math.atan2(dy, dx)

                # Compute the angular error
                angle_error = angle_to_goal - rth
                # Normalize angle_error to [-pi, pi]
                angle_error = (angle_error + math.pi) % (2 * math.pi) - math.pi

                if state == 'ROTATING':
                    # Rotate in place to face the waypoint
                    angular_velocity = K_ANGULAR * angle_error

                    # Limit angular velocity
                    angular_velocity = max(-MAX_ANGULAR_SPEED, min(MAX_ANGULAR_SPEED, angular_velocity))

                    # Set linear velocity to zero
                    linear_velocity = 0.0

                    # Send velocity commands
                    cmd_msg = {
                        'linear_velocity': linear_velocity,
                        'angular_velocity': angular_velocity
                    }
                    client.publish(MQTT_TOPIC_DRIVE_CMD, json.dumps(cmd_msg))

                    print(f"[node_drivepath.py] ROTATING: angle_error {angle_error:.2f} rad, angular_velocity {angular_velocity:.2f} rad/s")

                    # Check if the robot is facing the waypoint within the threshold
                    if abs(angle_error) < ANGLE_THRESHOLD:
                        print("[node_drivepath.py] Rotation aligned. Switching to DRIVING state.")
                        state = 'DRIVING'

                elif state == 'DRIVING':
                    # Drive forward to the waypoint
                    linear_velocity = K_LINEAR * distance
                    linear_velocity = min(MAX_LINEAR_SPEED, linear_velocity)

                    # Optionally make small angular corrections
                    angular_velocity = K_ANGULAR * angle_error
                    # Limit angular velocity during driving
                    MAX_DRIVING_ANGULAR_SPEED = 0.5  # rad/s
                    angular_velocity = max(-MAX_DRIVING_ANGULAR_SPEED, min(MAX_DRIVING_ANGULAR_SPEED, angular_velocity))

                    # Send velocity commands
                    cmd_msg = {
                        'linear_velocity': linear_velocity,
                        'angular_velocity': angular_velocity
                    }
                    client.publish(MQTT_TOPIC_DRIVE_CMD, json.dumps(cmd_msg))

                    print(f"[node_drivepath.py] DRIVING: distance {distance:.2f} m, linear_velocity {linear_velocity:.2f} m/s, angular_velocity {angular_velocity:.2f} rad/s")

                    # Check if we have reached the waypoint
                    if distance < DISTANCE_THRESHOLD:
                        print(f"[node_drivepath.py] Reached waypoint {current_index}: ({goal_x:.2f}, {goal_y:.2f})")
                        current_index += 1
                        if current_index < len(path_xy):
                            state = 'ROTATING'  # Rotate to face the next waypoint
                        else:
                            # Path completed
                            print("[node_drivepath.py] Path completed. Notifying path planning node.")
                            # Publish to the path completed topic
                            client.publish(MQTT_TOPIC_PATH_COMPLETED, json.dumps({'status': 'completed'}))
                            # Reset path variables
                            path_xy = []
                            current_index = 0
                            state = 'IDLE'  # No active path

                else:
                    # State is 'IDLE', send stop command
                    cmd_msg = {
                        'linear_velocity': 0.0,
                        'angular_velocity': 0.0
                    }
                    client.publish(MQTT_TOPIC_DRIVE_CMD, json.dumps(cmd_msg))
                    print("[node_drivepath.py] IDLE: No path to follow.")
            else:
                # No path or path completed
                # Send stop command
                cmd_msg = {
                    'linear_velocity': 0.0,
                    'angular_velocity': 0.0
                }
                client.publish(MQTT_TOPIC_DRIVE_CMD, json.dumps(cmd_msg))
                time.sleep(0.1)

    except KeyboardInterrupt:
        print("\n[node_drivepath.py] Interrupted by user.")
    finally:
        # Send stop command on exit
        cmd_msg = {
            'linear_velocity': 0.0,
            'angular_velocity': 0.0
        }
        client.publish(MQTT_TOPIC_DRIVE_CMD, json.dumps(cmd_msg))
        client.loop_stop()
        client.disconnect()
        print("[node_drivepath.py] Shutdown complete.")

if __name__ == "__main__":
    main()