import time
import math
import numpy as np
import matplotlib.pyplot as plt
from queue import PriorityQueue
from Common.sara_library import SaraRobot, ColorLed

# ===== CONFIG =====
GRID_SIZE_CM = 5
MAP_WIDTH = 100
MAP_HEIGHT = 100
OBSTACLE_THRESHOLD = 0.3
MAX_SENSOR_RANGE_CM = 100

FORWARD_SPEED = 30      # speed in cm/s approximation
ROTATION_SPEED = 40     # degrees/s approximation
TIME_STEP = 0.1         # seconds per loop

# ===== GRID FUNCTIONS =====
def world_to_grid(x_cm, y_cm):
    return int(x_cm / GRID_SIZE_CM), int(y_cm / GRID_SIZE_CM)

def grid_to_world(gx, gy):
    return gx * GRID_SIZE_CM + GRID_SIZE_CM/2, gy * GRID_SIZE_CM + GRID_SIZE_CM/2

def update_map(robot, x_cm, y_cm, theta_rad, occupancy_map):
    sensor_values = robot.body.distance_sensors.get_all_values() / 65536
    angles_bottom = robot.body.distance_sensors.sensor_angles_bottom.flatten()
    angles_mid = robot.body.distance_sensors.sensor_angles_mid.flatten()
    angles = np.concatenate([angles_bottom, angles_mid])

    for i, dist in enumerate(sensor_values):
        if dist < OBSTACLE_THRESHOLD:
            distance_cm = dist * MAX_SENSOR_RANGE_CM
            angle = theta_rad + math.radians(angles[i])
            obs_x = x_cm + distance_cm * math.cos(angle)
            obs_y = y_cm + distance_cm * math.sin(angle)
            gx, gy = world_to_grid(obs_x, obs_y)
            if 0 <= gx < MAP_WIDTH and 0 <= gy < MAP_HEIGHT:
                occupancy_map[gx, gy] = 1

def plot_map(occupancy_map, robot_pos=None, goal_pos=None, path=None):
    plt.clf()
    plt.imshow(occupancy_map.T, origin='lower', cmap='Greys', interpolation='nearest')
    if path:
        px, py = zip(*path)
        plt.scatter(px, py, c='yellow', s=20)
    if robot_pos:
        rx, ry = world_to_grid(*robot_pos)
        plt.scatter(rx, ry, c='blue', s=50)
    if goal_pos:
        gx, gy = world_to_grid(*goal_pos)
        plt.scatter(gx, gy, c='green', s=50)
    plt.pause(0.001)

# ===== PATHFINDING =====
def astar(start, goal, occupancy_map):
    open_set = PriorityQueue()
    open_set.put((0, start))
    came_from = {}
    g_score = {start:0}
    def h(a,b): return abs(a[0]-b[0]) + abs(a[1]-b[1])
    while not open_set.empty():
        _, current = open_set.get()
        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            return path[::-1]
        cx, cy = current
        for dx, dy in [(-1,0),(1,0),(0,-1),(0,1)]:
            nx, ny = cx+dx, cy+dy
            if 0<=nx<MAP_WIDTH and 0<=ny<MAP_HEIGHT and occupancy_map[nx,ny]==0:
                neighbor = (nx, ny)
                tentative_g = g_score[current] + 1
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    g_score[neighbor] = tentative_g
                    f = tentative_g + h(neighbor, goal)
                    open_set.put((f, neighbor))
                    came_from[neighbor] = current
    return None

# ===== MOVEMENT & ODOMETRY =====
def update_position(robot, x_cm, y_cm, theta_rad):
    """
    Update robot position using encoders + compass.
    """
    # Read encoders (assumes two main wheels: left=0, right=1)
    encoders = robot.body.encoders.encoders
    left_distance = encoders[0] / 10.0  # convert to cm, adjust scaling
    right_distance = encoders[1] / 10.0

    # Compute forward distance and rotation
    forward_cm = (left_distance + right_distance) / 2.0
    wheel_base_cm = 20  # approximate distance between wheels
    delta_theta = (right_distance - left_distance) / wheel_base_cm
    theta_rad = math.radians(robot.body.compass.read_abs_angle())

    # Update x, y in world coordinates
    x_cm += forward_cm * math.cos(theta_rad)
    y_cm += forward_cm * math.sin(theta_rad)
    return x_cm, y_cm, theta_rad

def move_towards(robot, x_cm, y_cm, theta_rad, target_cm):
    tx, ty = target_cm
    angle_to_target = math.atan2(ty - y_cm, tx - x_cm)
    distance = math.hypot(tx - x_cm, ty - y_cm)

    rotation = 0
    if abs(angle_to_target - theta_rad) > 0.1:
        rotation = ROTATION_SPEED if angle_to_target > theta_rad else -ROTATION_SPEED
        forward = 0
    else:
        forward = min(FORWARD_SPEED, distance)
        rotation = 0

    robot.base.move(Sideways_Velocity=0, Forward_Velocity=forward, Rotation_Velocity=rotation)
    x_cm += forward * TIME_STEP * math.cos(theta_rad)
    y_cm += forward * TIME_STEP * math.sin(theta_rad)
    theta_rad += math.radians(rotation * TIME_STEP)
    return x_cm, y_cm, theta_rad

# ===== MAIN =====
def main():
    robot = SaraRobot(logging=False)
    time.sleep(1)

    occupancy_map = np.zeros((MAP_WIDTH, MAP_HEIGHT), dtype=int)
    x_cm, y_cm = 250, 250
    theta_rad = math.radians(robot.body.compass.read_abs_angle())

    # Set goal in cm
    goal_cm = (400, 100)

    plt.ion()
    plt.figure(figsize=(8,8))
    path = None
    old_counter = robot.body.distance_sensors.get_rx_counter()

    try:
        while True:
            # Update map with sensor readings
            update_map(robot, x_cm, y_cm, theta_rad, occupancy_map)

            # Plan path if needed
            gx, gy = world_to_grid(*goal_cm)
            rx, ry = world_to_grid(x_cm, y_cm)
            if path is None or occupancy_map[gx, gy] == 1:
                path = astar((rx, ry), (gx, gy), occupancy_map)
                if path is None:
                    print("No path found!")
                    robot.base.move_stop()
                    break

            # Move along path
            next_cell = path[1] if len(path) > 1 else path[0]
            next_cm = grid_to_world(*next_cell)
            x_cm, y_cm, theta_rad = move_towards(robot, x_cm, y_cm, theta_rad, next_cm)

            # Update plot
            plot_map(occupancy_map, robot_pos=(x_cm, y_cm), goal_pos=goal_cm, path=path)

            # Wait for new sensor data
            while old_counter == robot.body.distance_sensors.get_rx_counter():
                time.sleep(0.05)
            old_counter = robot.body.distance_sensors.get_rx_counter()

    except KeyboardInterrupt:
        print("Stopping")
        robot.base.move_stop()
        robot.base.led.setcolor(ColorLed.WHITE, ColorLed.LED_ON)
        robot.stop()

if __name__ == "__main__":
    main()
