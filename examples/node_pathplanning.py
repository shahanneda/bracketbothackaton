#!/usr/bin/env python3
import numpy as np
import json
import time
import math
import random
import paho.mqtt.client as mqtt
from heapq import heappush, heappop

# -----------------------------------------------------------------------------
# MQTT Setup
# -----------------------------------------------------------------------------
MQTT_BROKER = "localhost"
MQTT_PORT   = 1883

# We read the occupancy grid from here
MQTT_TOPIC_OCC_GRID = "robot/tof_map"
# We publish paths here
MQTT_TOPIC_PATH_PLAN = "robot/local_path"

client = mqtt.Client()
client.connect(MQTT_BROKER, MQTT_PORT, keepalive=60)
client.loop_start()

# Global variables
occupancy_grid = None
grid_params = {}

# We'll keep track of the current path so we can see if it becomes obstructed
current_path_rc = None  # list of (r, c)
current_path_xy = None  # list of (x, y) in world coords

# -----------------------------------------------------------------------------
# MQTT Callback
# -----------------------------------------------------------------------------
def on_message(client, userdata, message):
    """
    Callback for occupancy grid updates.
    We'll parse the occupancy grid and store it for planning.
    """
    global occupancy_grid, grid_params

    payload = json.loads(message.payload)
    if "occupancy_grid" not in payload:
        return

    grid_info = payload["occupancy_grid"]

    # Parse metadata
    height     = grid_info["height"]
    width      = grid_info["width"]
    resolution = grid_info["resolution"]
    min_x      = grid_info["min_x"]
    max_x      = grid_info["max_x"]
    min_y      = grid_info["min_y"]
    max_y      = grid_info["max_y"]

    data = np.array(grid_info["data"], dtype=np.uint8).reshape((height, width))

    occupancy_grid = data
    grid_params = {
        "height": height,
        "width": width,
        "resolution": resolution,
        "min_x": min_x,
        "max_x": max_x,
        "min_y": min_y,
        "max_y": max_y
    }

client.subscribe(MQTT_TOPIC_OCC_GRID)
client.on_message = on_message

# -----------------------------------------------------------------------------
# A* Implementation
# -----------------------------------------------------------------------------
def heuristic(a, b):
    """
    Heuristic for A* (Euclidean distance).
    a, b are (row, col) in grid coordinates.
    """
    return math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)

def a_star(grid, start_rc, goal_rc):
    """
    Perform A* search on the occupancy grid (0=occupied, 1=free).
    start_rc: (row, col)
    goal_rc:  (row, col)
    Returns: list of (row, col) path or None if no path.
    """
    (start_r, start_c) = start_rc
    (goal_r,  goal_c)  = goal_rc

    if not in_bounds(grid, start_r, start_c) or not in_bounds(grid, goal_r, goal_c):
        return None
    if grid[start_r, start_c] == 0 or grid[goal_r, goal_c] == 0:
        return None

    # A* frontier
    frontier = []
    heappush(frontier, (0, start_rc))  # (priority, cell)

    came_from = {start_rc: None}
    cost_so_far = {start_rc: 0.0}

    # 8-directional movement
    neighbors_8 = [(-1, -1), (-1, 0), (-1, 1),
                   (0, -1),           (0, 1),
                   (1, -1),  (1, 0),  (1, 1)]

    while frontier:
        current_priority, current_cell = heappop(frontier)
        if current_cell == goal_rc:
            return reconstruct_path(came_from, start_rc, goal_rc)

        current_r, current_c = current_cell
        for dr, dc in neighbors_8:
            nr, nc = current_r + dr, current_c + dc
            if not in_bounds(grid, nr, nc):
                continue
            if grid[nr, nc] == 0:  # Occupied
                continue

            new_cost = cost_so_far[current_cell] + math.sqrt(dr*dr + dc*dc)
            if (nr, nc) not in cost_so_far or new_cost < cost_so_far[(nr, nc)]:
                cost_so_far[(nr, nc)] = new_cost
                priority = new_cost + heuristic((nr, nc), goal_rc)
                came_from[(nr, nc)] = current_cell
                heappush(frontier, (priority, (nr, nc)))
    return None

def reconstruct_path(came_from, start_rc, goal_rc):
    path = []
    current = goal_rc
    while current is not None:
        path.append(current)
        current = came_from[current]
    path.reverse()
    return path

def in_bounds(grid, r, c):
    return (0 <= r < grid.shape[0]) and (0 <= c < grid.shape[1])

# -----------------------------------------------------------------------------
# Conversions grid <-> world
# -----------------------------------------------------------------------------
def grid_to_world(r, c, params):
    """
    Convert grid cell (r, c) to (x, y) in meters.
    """
    resolution = params["resolution"]
    min_x      = params["min_x"]
    min_y      = params["min_y"]
    x = min_x + (c + 0.5) * resolution
    y = min_y + (r + 0.5) * resolution
    return (x, y)

def world_to_grid(x, y, params):
    """
    Convert (x, y) in meters to grid cell (r, c).
    """
    resolution = params["resolution"]
    min_x      = params["min_x"]
    min_y      = params["min_y"]
    c = int((x - min_x) / resolution)
    r = int((y - min_y) / resolution)
    return (r, c)

# -----------------------------------------------------------------------------
# "In Front" Check
# -----------------------------------------------------------------------------
def is_in_front(robot_x, robot_y, robot_th_deg, cand_x, cand_y, max_angle_deg=90.0):
    """
    Return True if the point (cand_x, cand_y) is within ±max_angle_deg
    of the robot's heading (robot_th_deg), relative to (robot_x, robot_y).
    
    We'll do this by computing:
        angle_to_candidate = atan2(cand_y - robot_y, cand_x - robot_x) [deg]
        angle_diff = minimal difference between angle_to_candidate and robot_th_deg
    Then check if |angle_diff| <= max_angle_deg.
    """
    dx = cand_x - robot_x
    dy = cand_y - robot_y
    angle_to_candidate_deg = math.degrees(math.atan2(dy, dx))

    # Normalize angles to (-180, +180]
    def wrap_angle_180(a):
        return (a + 180) % 360 - 180

    # difference in [-180, +180]
    diff = wrap_angle_180(angle_to_candidate_deg - robot_th_deg)
    return abs(diff) <= max_angle_deg

# -----------------------------------------------------------------------------
# Path Obstruction Check
# -----------------------------------------------------------------------------
def is_path_obstructed(path_rc, grid):
    """
    Returns True if any cell in the path is now occupied (grid==0).
    """
    for (r, c) in path_rc:
        if not in_bounds(grid, r, c):
            return True  # out of bounds => definitely obstructed
        if grid[r, c] == 0:
            return True
    return False

# -----------------------------------------------------------------------------
# Updated Random Goal Selection (In Front + nearest)
# -----------------------------------------------------------------------------
def pick_random_free_cell_in_front(grid, grid_params, robot_r, robot_c,
                                   robot_x, robot_y, robot_th_deg,
                                   num_samples=30, max_radius_m=1.5):
    """
    Sample up to `num_samples` random free cells within 'max_radius_m' 
    of the robot, but only those that lie in front of the robot 
    (±90 deg by default). Then pick the nearest one among those.

    Returns (r, c) or None.
    """
    resolution = grid_params["resolution"]
    height, width = grid.shape

    # We'll define a bounding box in grid coordinates for a circle of radius max_radius_m
    max_radius_cells = int(max_radius_m / resolution)
    r_min = max(0, robot_r - max_radius_cells)
    r_max = min(height - 1, robot_r + max_radius_cells)
    c_min = max(0, robot_c - max_radius_cells)
    c_max = min(width  - 1, robot_c + max_radius_cells)

    candidates = []
    for _ in range(num_samples):
        # random r, c in bounding box
        rand_r = random.randint(r_min, r_max)
        rand_c = random.randint(c_min, c_max)

        # must be free
        if grid[rand_r, rand_c] == 1:
            # check distance
            dist_cells = math.sqrt((rand_r - robot_r)**2 + (rand_c - robot_c)**2)
            if dist_cells <= max_radius_cells:
                # check if in front
                cand_x, cand_y = grid_to_world(rand_r, rand_c, grid_params)
                if is_in_front(robot_x, robot_y, robot_th_deg, cand_x, cand_y):
                    candidates.append((rand_r, rand_c))

    if not candidates:
        return None

    # pick the nearest candidate (in world distance)
    best_candidate = None
    best_dist = float('inf')
    for (cr, cc) in candidates:
        cx, cy = grid_to_world(cr, cc, grid_params)
        d = math.hypot(cx - robot_x, cy - robot_y)
        if d < best_dist:
            best_dist = d
            best_candidate = (cr, cc)

    return best_candidate

# -----------------------------------------------------------------------------
# Main Loop
# -----------------------------------------------------------------------------
def main():
    global occupancy_grid, grid_params
    global current_path_rc, current_path_xy

    # Example robot state:
    #   Suppose the robot is at (x=0, y=0) facing 0 deg in world frame.
    #   In a real system, you'd read these from odometry / IMU / etc.
    robot_x = 0.0
    robot_y = 0.0
    robot_th_deg = 0.0

    while True:
        if occupancy_grid is None:
            print("No occupancy grid yet. Waiting...")
            time.sleep(1)
            continue

        # If we have an existing path, check if it's obstructed
        if current_path_rc is not None:
            if is_path_obstructed(current_path_rc, occupancy_grid):
                print("Path became obstructed! We'll pick a new random goal...")
                current_path_rc = None  # force new path
            else:
                # Path is still good, keep using it
                print("Path is still valid. Continuing to follow it.")
                # in a real system, you might have code to follow the path here.
                # we'll just sleep and not choose a new path if it's not obstructed
                time.sleep(2)
                continue
        
        # If we get here, either we have no path or the old one was obstructed.
        # => pick a new random goal in front
        robot_r, robot_c = world_to_grid(robot_x, robot_y, grid_params)
        goal_rc = pick_random_free_cell_in_front(
            occupancy_grid, grid_params,
            robot_r, robot_c,
            robot_x, robot_y, robot_th_deg,
            num_samples=30, max_radius_m=1.5
        )
        if goal_rc is None:
            print("No front-facing free cell found. Retrying in 1s...")
            time.sleep(1)
            continue

        # Run A* to that goal
        path_rc = a_star(occupancy_grid, (robot_r, robot_c), goal_rc)
        if path_rc is None:
            print("No path found to the chosen random goal. Trying again...")
            time.sleep(1)
            continue

        # We have a path. Store it globally, publish it.
        current_path_rc = path_rc
        current_path_xy = [grid_to_world(r, c, grid_params) for (r, c) in path_rc]

        path_msg = {
            "path_rc": path_rc,
            "path_xy": current_path_xy
        }
        client.publish(MQTT_TOPIC_PATH_PLAN, json.dumps(path_msg))
        print(f"Published new path with {len(path_rc)} points. Goal cell = {goal_rc}")

        # Sleep a bit, then let the loop check if path becomes obstructed, etc.
        time.sleep(2)


try:
    main()
except KeyboardInterrupt:
    print("Interrupted by user.")
finally:
    client.loop_stop()
    client.disconnect()
    print("MQTT disconnected. Exiting.")
