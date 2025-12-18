import pygame
import numpy as np
import math
import sys
import matplotlib.pyplot as plt
import time

# --- Simulation setup ---
WIDTH, HEIGHT = 900, 600
FPS = 60
SENSOR_COUNT = 13
SENSOR_ANGLE_RANGE = (-90, 90)  # degrees
SENSOR_LENGTH = 150
SENSOR_BEAM_WIDTH = 8      # degrees per sensor beam
SENSOR_SUBRAYS = 5         # how many sub-rays per beam
ROBOT_RADIUS = 20
MEMORY_DECAY = 0.5        # memory fade rate

# Colors
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
GREY = (180, 180, 180)
RED = (255, 80, 80)
BLUE = (100, 200, 255)
GREEN = (50, 255, 50)
PURPLE = (200, 100, 255)
YELLOW = (255, 255, 80)

pygame.init()
screen = pygame.display.set_mode((WIDTH, HEIGHT))
clock = pygame.time.Clock()
pygame.display.set_caption("SaraBot 2D Smart Avoidance Simulator")

# --- Virtual robot state ---
robot_pos = np.array([WIDTH // 2, HEIGHT // 2], dtype=float)
robot_angle = 0.0
obstacles = []
path_points = []
direction_memory = np.zeros(SENSOR_COUNT, dtype=float)

# --- Matplotlib setup ---
plt.ion()
fig, ax = plt.subplots(subplot_kw={"projection": "polar"})
fig.canvas.manager.set_window_title("Sensor Detection Graph")
ax.set_theta_offset(np.pi / 2)
ax.set_theta_direction(-1)
ax.set_rlim(0, 1)

# --- Map generator functions ---

def make_u_shape():
    """
    Creates a U-shaped arrangement of chair-leg obstacles with an inner 'island' walkway.
    Outer U (chair legs) and inner island (chair legs) spaced to look like chair-legs.
    """
    obs = []
    spacing = 20
    leg_size = 6

    # U dimensions
    u_width = 500
    u_height = 300
    offset_x = WIDTH//2 - u_width//2
    offset_y = HEIGHT//2 - u_height//2

    # Outer left & right vertical "walls" (chair legs)
    for y in range(0, u_height + spacing, spacing):
        obs.append(pygame.Rect(offset_x - leg_size//2, offset_y + y - leg_size//2, leg_size, leg_size))
        obs.append(pygame.Rect(offset_x + u_width - leg_size//2, offset_y + y - leg_size//2, leg_size, leg_size))

    # Outer bottom horizontal "floor" of the U
    for x in range(0, u_width + spacing, spacing):
        obs.append(pygame.Rect(offset_x + x - leg_size//2, offset_y + u_height - leg_size//2, leg_size, leg_size))

    # Inner island: create a smaller rounded rectangle (island) with chair-legs outlining it,
    # leaving a walkway between island and outer U.
    walkway_margin = 60  # distance between outer U and island
    island_w = u_width - walkway_margin * 2
    island_h = u_height - walkway_margin * 2
    island_x = offset_x + walkway_margin
    island_y = offset_y + walkway_margin

    # Create chair-leg outline for island
    for x in range(0, island_w + spacing, spacing):
        obs.append(pygame.Rect(island_x + x - leg_size//2, island_y - leg_size//2, leg_size, leg_size))
        obs.append(pygame.Rect(island_x + x - leg_size//2, island_y + island_h - leg_size//2, leg_size, leg_size))
    for y in range(0, island_h + spacing, spacing):
        obs.append(pygame.Rect(island_x - leg_size//2, island_y + y - leg_size//2, leg_size, leg_size))
        obs.append(pygame.Rect(island_x + island_w - leg_size//2, island_y + y - leg_size//2, leg_size, leg_size))

    return obs

def make_corner():
    """Creates a corner obstacle (L-shaped wall) made of spaced chair-legs."""
    obs = []
    spacing = 20
    leg_size = 8
    corner_size = 320
    offset_x = WIDTH//2 - corner_size//2
    offset_y = HEIGHT//2 - corner_size//2

    for x in range(0, corner_size + spacing, spacing):
        obs.append(pygame.Rect(offset_x + x - leg_size//2, offset_y - leg_size//2, leg_size, leg_size))
    for y in range(0, corner_size + spacing, spacing):
        obs.append(pygame.Rect(offset_x - leg_size//2, offset_y + y - leg_size//2, leg_size, leg_size))
    return obs

def make_corridor():
    """Creates a long corridor (parallel walls) made of chair-legs."""
    obs = []
    spacing = 20
    leg_size = 8
    corridor_width = 220
    wall_length = HEIGHT - 120
    offset_x = WIDTH//2 - corridor_width//2
    offset_y = 60

    for y in range(0, wall_length + spacing, spacing):
        obs.append(pygame.Rect(offset_x - leg_size//2, offset_y + y - leg_size//2, leg_size, leg_size))
        obs.append(pygame.Rect(offset_x + corridor_width - leg_size//2, offset_y + y - leg_size//2, leg_size, leg_size))
    return obs

def make_random_obstacles():
    """Random scattered 'chair-leg' obstacles."""
    obs = []
    leg_size = 8
    for _ in range(60):
        x = np.random.randint(40, WIDTH - 40)
        y = np.random.randint(40, HEIGHT - 40)
        obs.append(pygame.Rect(x - leg_size//2, y - leg_size//2, leg_size, leg_size))
    return obs

def choose_map(name):
    name = name.strip().lower()
    if name == "u_shape":
        return make_u_shape()
    elif name == "corner":
        return make_corner()
    elif name == "corridor":
        return make_corridor()
    elif name == "random":
        return make_random_obstacles()
    else:
        return []

# --- Helper functions ---

def cast_sensors(pos, angle_deg, obstacles):
    """
    Simulate 180° distance sensors with multiple sub-rays per beam.
    Returns:
      - distances: array of normalized distances (0..1)
      - sensor_angles_rad: array of sensor beam center angles in radians (relative to robot heading)
                         (i.e. angles you can add to math.radians(angle_deg))
    """
    angles_deg = np.linspace(SENSOR_ANGLE_RANGE[0], SENSOR_ANGLE_RANGE[1], SENSOR_COUNT)
    distances = np.ones(SENSOR_COUNT)

    for i, a_deg in enumerate(angles_deg):
        # beam subrays distributed across beam width
        ray_angles_deg = np.linspace(a_deg - SENSOR_BEAM_WIDTH/2, a_deg + SENSOR_BEAM_WIDTH/2, SENSOR_SUBRAYS)
        min_dist = 1.0
        for sub_a_deg in ray_angles_deg:
            ray_angle = math.radians(angle_deg + sub_a_deg)
            for d in range(1, SENSOR_LENGTH):
                x = pos[0] + d * math.cos(ray_angle)
                y = pos[1] - d * math.sin(ray_angle)
                # out of bounds -> treat as obstacle
                if x < 0 or x >= WIDTH or y < 0 or y >= HEIGHT:
                    min_dist = min(min_dist, d / SENSOR_LENGTH)
                    break
                # check collision with any chair-leg (rect)
                hit = False
                for rect in obstacles:
                    if rect.collidepoint(x, y):
                        min_dist = min(min_dist, d / SENSOR_LENGTH)
                        hit = True
                        break
                if hit:
                    break
        distances[i] = min_dist
    # provide sensor center angles in radians for drawing: convert degrees to radians
    sensor_angles_rad = np.deg2rad(angles_deg)
    return distances, sensor_angles_rad

def smart_avoidance(sensor_values, direction_memory):
    """Smart obstacle avoidance with short-term memory."""
    max_forward = 20.0
    max_turn = 20.0

    repulsion = (1 - sensor_values) ** 2
    sensor_angles = np.linspace(-90, 90, len(sensor_values)) * math.pi / 180
    direction_memory = direction_memory * MEMORY_DECAY + repulsion * (1 - MEMORY_DECAY)

    avoidance_strength = repulsion + 0.75 * direction_memory
    x_force = np.sum(-avoidance_strength * np.cos(sensor_angles))
    y_force = np.sum(-avoidance_strength * np.sin(sensor_angles))

    desired_vector = np.array([1.0, 0.0])
    avoidance_vector = np.array([x_force, y_force])
    combined_vector = desired_vector + avoidance_vector
    combined_angle = math.atan2(combined_vector[1], combined_vector[0])

    rotation = np.clip(math.degrees(combined_angle) * 0.5, -max_turn, max_turn)
    forward = max_forward * (1 - np.clip(np.max(repulsion), 0.2, 1))

    return forward, rotation, direction_memory

def draw_robot(screen, pos, angle_deg, sensor_values, sensor_angles_rad, memory):
    pygame.draw.circle(screen, BLUE, pos.astype(int), ROBOT_RADIUS)
    # draw heading line
    hx = pos[0] + (ROBOT_RADIUS + 10) * math.cos(math.radians(angle_deg))
    hy = pos[1] - (ROBOT_RADIUS + 10) * math.sin(math.radians(angle_deg))
    pygame.draw.line(screen, BLACK, pos, (hx, hy), 2)
    for i, a in enumerate(sensor_angles_rad):
        ray_angle = math.radians(angle_deg) + a
        length = SENSOR_LENGTH * sensor_values[i]
        end_x = pos[0] + length * math.cos(ray_angle)
        end_y = pos[1] - length * math.sin(ray_angle)
        mem_intensity = int(np.clip(memory[i], 0, 1) * 255)
        # sensor color: more memory -> stronger red; distant -> grey
        if sensor_values[i] < 0.9:
            color = (mem_intensity, max(0, 120 - mem_intensity//2), 0)
        else:
            c = mem_intensity // 2
            color = (c, c, c)
        pygame.draw.line(screen, color, pos, (end_x, end_y), 2)

def update_sensor_plot(ax, sensor_values):
    ax.clear()
    ax.set_theta_offset(np.pi / 2)
    ax.set_theta_direction(-1)
    ax.set_rlim(0, 1)
    sensor_angles = np.linspace(-90, 90, len(sensor_values)) * np.pi / 180
    for i, val in enumerate(sensor_values):
        start_angle = sensor_angles[i] - (np.pi / SENSOR_COUNT)
        stop_angle = sensor_angles[i] + (np.pi / SENSOR_COUNT)
        color = "lightgrey"
        if val < 0.6:
            color = "yellow"
        if val < 0.3:
            color = "red"
        ax.fill_between([start_angle, stop_angle], val, 1, color=color)
    fig.canvas.draw_idle()
    fig.canvas.flush_events()

# --- Choose a pre-shaped map before the simulation starts ---
print("Choose map: [u_shape, corner, corridor, random, empty]")
selected_map = input("Map type: ").strip().lower()
obstacles = choose_map(selected_map)

# optional: set robot start position/heading for certain maps
if selected_map == "corridor":
    robot_pos = np.array([WIDTH//2, HEIGHT - 80], dtype=float)
    robot_angle = -90.0  # facing up
elif selected_map == "corner":
    robot_pos = np.array([WIDTH//2 + 120, HEIGHT//2 + 120], dtype=float)
    robot_angle = 180.0
elif selected_map == "u_shape":
    robot_pos = np.array([WIDTH//2, HEIGHT//2 + 90], dtype=float)
    robot_angle = 90.0
else:
    robot_pos = np.array([WIDTH // 2, HEIGHT // 2], dtype=float)
    robot_angle = 0.0

# --- Main loop ---
running = True
last_update_time = time.time()

while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    # handle mouse pressed actions for adding/removing chair legs
    mouse_pressed = pygame.mouse.get_pressed()
    if mouse_pressed[0]:  # left click add small "chair leg" obstacle (10x10)
        mx, my = pygame.mouse.get_pos()
        # avoid adding too many duplicates at same spot: only add if no rect collides nearby
        nearby = False
        for r in obstacles:
            if r.collidepoint(mx, my):
                nearby = True
                break
        if not nearby:
            obstacles.append(pygame.Rect(mx - 5, my - 5, 10, 10))
    if mouse_pressed[2]:  # right click remove obstacle under mouse
        mx, my = pygame.mouse.get_pos()
        obstacles = [r for r in obstacles if not r.collidepoint(mx, my)]

    # Sensor simulation
    sensor_values, angles_rad = cast_sensors(robot_pos, robot_angle, obstacles)

    # Avoidance logic
    forward, rotation, direction_memory = smart_avoidance(sensor_values, direction_memory)

    # Update robot position with collision detection
    new_robot_pos = robot_pos.copy()
    new_robot_angle = robot_angle + rotation
    new_robot_pos[0] += forward * math.cos(math.radians(new_robot_angle))
    new_robot_pos[1] -= forward * math.sin(math.radians(new_robot_angle))

    # Create a rectangle for the robot's new position
    robot_rect = pygame.Rect(new_robot_pos[0] - ROBOT_RADIUS, new_robot_pos[1] - ROBOT_RADIUS, ROBOT_RADIUS * 2, ROBOT_RADIUS * 2)

    # Check for collisions with obstacles
    collision = False
    for rect in obstacles:
        if robot_rect.colliderect(rect):
            collision = True
            break

    # If no collision, update the robot's position and angle
    if not collision:
        robot_pos = new_robot_pos
        robot_angle = new_robot_angle

    # Update robot position
    robot_angle += rotation
    robot_pos[0] += forward * math.cos(math.radians(robot_angle))
    robot_pos[1] -= forward * math.sin(math.radians(robot_angle))
    robot_pos[0] = np.clip(robot_pos[0], ROBOT_RADIUS, WIDTH - ROBOT_RADIUS)
    robot_pos[1] = np.clip(robot_pos[1], ROBOT_RADIUS, HEIGHT - ROBOT_RADIUS)
    path_points.append(tuple(robot_pos))

    # Visualization
    screen.fill(WHITE)
    for rect in obstacles:
        pygame.draw.rect(screen, GREEN, rect)
    if len(path_points) > 2:
        pygame.draw.lines(screen, PURPLE, False, path_points, 1)
    draw_robot(screen, robot_pos, robot_angle, sensor_values, angles_rad, direction_memory)
    pygame.display.flip()

    if time.time() - last_update_time > 0.05:
        update_sensor_plot(ax, sensor_values)
        last_update_time = time.time()

    clock.tick(FPS)

pygame.quit()
plt.close(fig)
sys.exit()
