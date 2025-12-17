"""
Maze exploration - always flies forward (camera in front)
Uses right-hand rule with forward priority
"""

import pyhula_simulator as pyhula
from maze_config import ROWS, COLS, START, WALLS
import time
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import heapq

# Setup simulator
pyhula.set_maze_size(rows=ROWS, cols=COLS)
pyhula.set_walls(WALLS)

api = pyhula.UserApi()
if not api.connect():
    print("Connection failed")
    exit()

print("Connected successfully")

BLOCK_SIZE = 60
FLIGHT_HEIGHT = 80
BLOCK = 0.6

rows, cols = ROWS, COLS
start_x, start_y = START

print(f"\nMaze: {rows} rows x {cols} columns")
print(f"Start: ({start_x}, {start_y})")
print(f"Total blocks to visit: {rows * cols}")

# Direction: 0=North(+Y), 1=East(+X), 2=South(-Y), 3=West(-X)
DIR_DELTA = [(0, 1), (1, 0), (0, -1), (-1, 0)]
DIR_NAMES = ['North', 'East', 'South', 'West']

# Setup plot
plt.ion()
fig, ax = plt.subplots(figsize=(10, 8))
ax.set_xlim(0, cols * BLOCK)
ax.set_ylim(0, rows * BLOCK)
ax.set_aspect('equal')
ax.grid(True, alpha=0.3)
title = ax.set_title('Exploring (Forward Only) - 0/' + str(rows * cols))

# Draw grid
for i in range(cols + 1):
    ax.axvline(i * BLOCK, color='black', linewidth=2)
for i in range(rows + 1):
    ax.axhline(i * BLOCK, color='black', linewidth=2)

# Track visited blocks
visited_patches = {}
for x in range(cols):
    for y in range(rows):
        patch = ax.add_patch(Rectangle((x * BLOCK + 0.02, y * BLOCK + 0.02),
                            BLOCK - 0.04, BLOCK - 0.04, facecolor='white', alpha=0.3))
        visited_patches[(x, y)] = patch

ax.add_patch(Rectangle((start_x * BLOCK + 0.05, start_y * BLOCK + 0.05),
                        BLOCK - 0.1, BLOCK - 0.1, facecolor='green', alpha=0.5, zorder=2))

drone_pos, = ax.plot([], [], 'bo', markersize=15, zorder=5)
drone_arrow = None
path_line, = ax.plot([], [], 'b-', linewidth=2, alpha=0.5)
detected_walls = set()

def block_to_cm(bx, by):
    return 15 + (bx * BLOCK_SIZE), 15 + (by * BLOCK_SIZE)

def mark_visited(x, y):
    visited_patches[(x, y)].set_facecolor('lightgreen')
    visited_patches[(x, y)].set_alpha(0.5)

def update_plot(current_x, current_y, facing, new_walls_list):
    global drone_arrow

    cx = current_x * BLOCK + BLOCK/2
    cy = current_y * BLOCK + BLOCK/2
    drone_pos.set_data([cx], [cy])

    if drone_arrow:
        drone_arrow.remove()
    dx, dy = DIR_DELTA[facing]
    drone_arrow = ax.arrow(cx, cy, dx * 0.2, dy * 0.2, head_width=0.1, head_length=0.05,
                           fc='yellow', ec='black', zorder=6)

    path_line.set_data([px * BLOCK + BLOCK/2 for px in path_x], [py * BLOCK + BLOCK/2 for py in path_y])

    # Draw new walls
    for wall in new_walls_list:
        if wall not in detected_walls:
            detected_walls.add(wall)
            (x1, y1), (x2, y2) = wall
            if x1 == x2:  # Horizontal wall
                wx = x1 * BLOCK
                wy1, wy2 = min(y1, y2) * BLOCK + BLOCK, min(y1, y2) * BLOCK + BLOCK
                ax.plot([wx, wx + BLOCK], [wy1, wy2], 'r-', linewidth=5)
            else:  # Vertical wall
                wy = y1 * BLOCK
                wx1, wx2 = min(x1, x2) * BLOCK + BLOCK, min(x1, x2) * BLOCK + BLOCK
                ax.plot([wx1, wx2], [wy, wy + BLOCK], 'r-', linewidth=5)

    plt.draw()
    plt.pause(0.05)

def get_obstacles_absolute(obstacles, facing):
    """Convert relative obstacles to absolute directions blocked"""
    # obstacles: forward/back/left/right relative to drone
    # facing: 0=N, 1=E, 2=S, 3=W
    blocked = set()
    rel_to_offset = {'forward': 0, 'right': 1, 'back': 2, 'left': 3}

    for rel_dir, is_blocked in obstacles.items():
        if is_blocked and rel_dir in rel_to_offset:
            abs_dir = (facing + rel_to_offset[rel_dir]) % 4
            blocked.add(abs_dir)

    return blocked

def can_move(current, direction, blocked_abs):
    """Check if can move in absolute direction"""
    if direction in blocked_abs:
        return False
    dx, dy = DIR_DELTA[direction]
    nx, ny = current[0] + dx, current[1] + dy
    return 0 <= nx < cols and 0 <= ny < rows

# Start
api.single_fly_barrier_aircraft(True)
api.Plane_cmd_switch_QR(0)
time.sleep(0.5)
api.single_fly_takeoff()
time.sleep(0.5)

current_x, current_y = start_x, start_y
facing = 0  # Start facing North

abs_x, abs_y = block_to_cm(current_x, current_y)
api.single_fly_straight_flight(abs_x, abs_y, FLIGHT_HEIGHT)

all_blocks = {(x, y) for x in range(cols) for y in range(rows)}
visited = {(start_x, start_y)}
mark_visited(start_x, start_y)

path_x, path_y = [current_x], [current_y]
total_steps = 0
max_steps = rows * cols * 4  # Safety limit

print(f"\n=== Starting exploration (forward priority) ===")

while len(visited) < len(all_blocks) and total_steps < max_steps:
    obstacles = api.Plane_getBarrier_relative()
    blocked_abs = get_obstacles_absolute(obstacles, facing)

    # Record walls
    new_walls = []
    for abs_dir in blocked_abs:
        dx, dy = DIR_DELTA[abs_dir]
        neighbor = (current_x + dx, current_y + dy)
        wall = tuple(sorted([(current_x, current_y), neighbor]))
        new_walls.append(wall)

    update_plot(current_x, current_y, facing, new_walls)
    title.set_text(f'Exploring - {len(visited)}/{len(all_blocks)} | Facing: {DIR_NAMES[facing]}')

    print(f"Pos: ({current_x},{current_y}), Facing: {DIR_NAMES[facing]}, Visited: {len(visited)}/{len(all_blocks)}")

    # Priority: forward-unvisited > right-unvisited > left-unvisited > forward > right > left > back
    moved = False

    # Build move options: (priority, abs_direction, needs_turn)
    # Lower priority = better
    options = []

    for turn, abs_dir in [(0, facing), (1, (facing+1)%4), (3, (facing+3)%4), (2, (facing+2)%4)]:
        # turn: 0=forward, 1=right, 3=left, 2=back
        if not can_move((current_x, current_y), abs_dir, blocked_abs):
            continue

        dx, dy = DIR_DELTA[abs_dir]
        nx, ny = current_x + dx, current_y + dy
        is_unvisited = (nx, ny) not in visited

        # Priority: unvisited first (0), then visited (1), then by turn amount
        priority = (0 if is_unvisited else 1, turn)
        options.append((priority, abs_dir, nx, ny))

    if options:
        options.sort()
        _, new_facing, nx, ny = options[0]

        # Calculate turn needed: 0=none, 1=right 90°, 2=180°, 3=left 90°
        turn_amount = (new_facing - facing) % 4

        if turn_amount == 1:
            api.single_fly_turnright(90)
        elif turn_amount == 2:
            api.single_fly_turnright(180)
        elif turn_amount == 3:
            api.single_fly_turnleft(90)

        # Update facing
        facing = new_facing

        # Fly forward one block
        api.single_fly_forward(BLOCK_SIZE)

        # Update position
        current_x, current_y = nx, ny

        path_x.append(current_x)
        path_y.append(current_y)
        total_steps += 1

        if (current_x, current_y) not in visited:
            visited.add((current_x, current_y))
            mark_visited(current_x, current_y)

        moved = True

    if not moved:
        print("Stuck!")
        break

    time.sleep(0.1)

update_plot(current_x, current_y, facing, [])
title.set_text(f'Complete! {len(visited)}/{len(all_blocks)} in {total_steps} steps')

print(f"\n========== SUMMARY ==========")
print(f"Total blocks: {len(all_blocks)}")
print(f"Blocks visited: {len(visited)}")
print(f"Total steps: {total_steps}")
if len(visited) < len(all_blocks):
    print(f"Unreachable: {all_blocks - visited}")
print(f"=============================")

api.single_fly_touchdown()
plt.ioff()
plt.show()