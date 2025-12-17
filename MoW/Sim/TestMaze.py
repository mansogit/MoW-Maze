"""
Test maze navigation with simulator
"""

import pyhula_simulator as pyhula
from maze_config import ROWS, COLS, START, END, WALLS
import time
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

# Setup simulator from config
pyhula.set_maze_size(rows=ROWS, cols=COLS)
pyhula.set_walls(WALLS)

api = pyhula.UserApi()
if not api.connect():
    print("Connection failed")
    exit()

print("Connected successfully")

BLOCK_SIZE = 60
FLIGHT_HEIGHT = 80

def block_to_cm(block_x, block_y):
    return 15 + (block_x * BLOCK_SIZE), 15 + (block_y * BLOCK_SIZE)

def move_to_coordinates(x, y, h):
    api.single_fly_straight_flight(x, y, h)

rows, cols = ROWS, COLS
start_x, start_y = START
end_x, end_y = END

print(f"\nMaze: {rows} rows x {cols} columns")
print(f"Start: ({start_x}, {start_y}), End: ({end_x}, {end_y})")

BLOCK = 0.6

plt.ion()
fig, ax = plt.subplots(figsize=(8, 8))
ax.set_xlim(0, cols * BLOCK)
ax.set_ylim(0, rows * BLOCK)
ax.set_aspect('equal')
ax.grid(True, alpha=0.3)
ax.set_title('Drone Maze Navigation (SIMULATOR)')

for i in range(cols + 1):
    ax.axvline(i * BLOCK, color='black', linewidth=2)
for i in range(rows + 1):
    ax.axhline(i * BLOCK, color='black', linewidth=2)

ax.add_patch(Rectangle((start_x * BLOCK + 0.05, start_y * BLOCK + 0.05), BLOCK - 0.1, BLOCK - 0.1, facecolor='green', alpha=0.5))
ax.add_patch(Rectangle((end_x * BLOCK + 0.05, end_y * BLOCK + 0.05), BLOCK - 0.1, BLOCK - 0.1, facecolor='red', alpha=0.5))

drone_pos, = ax.plot([], [], 'bo', markersize=15)
path_line, = ax.plot([], [], 'b-', linewidth=2, alpha=0.5)
detected_walls = set()
obstacle_patches = []

api.single_fly_barrier_aircraft(True)
api.Plane_cmd_switch_QR(0)
time.sleep(0.5)

api.single_fly_takeoff()
time.sleep(0.5)

start_abs_x, start_abs_y = block_to_cm(start_x, start_y)
move_to_coordinates(start_abs_x, start_abs_y, FLIGHT_HEIGHT)

current_x, current_y = start_x, start_y
visited = {(current_x, current_y): 1}
last_move = None
stuck_counter = 0
path_x, path_y = [current_x], [current_y]

def update_plot(obstacles):
    drone_pos.set_data([current_x * BLOCK + BLOCK/2], [current_y * BLOCK + BLOCK/2])
    path_line.set_data([x * BLOCK + BLOCK/2 for x in path_x], [y * BLOCK + BLOCK/2 for y in path_y])
    for direction, has_obstacle in obstacles.items():
        if has_obstacle:
            if direction == 'forward':
                wall_key = (current_x, current_y + 1, 'h')
                x1, x2 = current_x * BLOCK, (current_x + 1) * BLOCK
                y1 = y2 = (current_y + 1) * BLOCK
            elif direction == 'back':
                wall_key = (current_x, current_y, 'h')
                x1, x2 = current_x * BLOCK, (current_x + 1) * BLOCK
                y1 = y2 = current_y * BLOCK
            elif direction == 'right':
                wall_key = (current_x + 1, current_y, 'v')
                x1 = x2 = (current_x + 1) * BLOCK
                y1, y2 = current_y * BLOCK, (current_y + 1) * BLOCK
            elif direction == 'left':
                wall_key = (current_x, current_y, 'v')
                x1 = x2 = current_x * BLOCK
                y1, y2 = current_y * BLOCK, (current_y + 1) * BLOCK
            else:
                continue
            if wall_key not in detected_walls:
                detected_walls.add(wall_key)
                line, = ax.plot([x1, x2], [y1, y2], 'r-', linewidth=5)
                obstacle_patches.append(line)
    plt.draw()
    plt.pause(0.1)

while current_x != end_x or current_y != end_y:
    obstacles = api.Plane_getBarrier()
    update_plot(obstacles)
    print(f"Position: ({current_x},{current_y}), Obstacles: {obstacles}")

    move_options = []
    for dx, dy, direction, blocked in [(0,1,'forward',obstacles.get('forward')),
                                        (0,-1,'back',obstacles.get('back')),
                                        (1,0,'right',obstacles.get('right')),
                                        (-1,0,'left',obstacles.get('left'))]:
        nx, ny = current_x + dx, current_y + dy
        if blocked or not (0 <= nx < cols and 0 <= ny < rows):
            continue
        dist = abs(nx - end_x) + abs(ny - end_y)
        vc = visited.get((nx, ny), 0)
        rev = last_move and dx == -last_move[0] and dy == -last_move[1]
        move_options.append((vc, dist, rev, dx, dy, direction, nx, ny))

    if not move_options:
        stuck_counter += 1
        if stuck_counter >= 15:
            print("Stuck!")
            break
        time.sleep(0.2)
        continue

    move_options.sort()
    vc, dist, rev, dx, dy, direction, nx, ny = move_options[0]

    if vc < 3:
        abs_x, abs_y = block_to_cm(nx, ny)
        move_to_coordinates(abs_x, abs_y, FLIGHT_HEIGHT)
        last_move = (dx, dy)
        current_x, current_y = nx, ny
        visited[(current_x, current_y)] = visited.get((current_x, current_y), 0) + 1
        path_x.append(current_x)
        path_y.append(current_y)
        stuck_counter = 0
    else:
        stuck_counter += 1
        if stuck_counter >= 15:
            break

    time.sleep(0.2)

update_plot({})
if current_x == end_x and current_y == end_y:
    print(f"\n✓ Reached destination! Explored {len(visited)} blocks")
else:
    print(f"\n✗ Stopped at ({current_x}, {current_y})")

api.single_fly_touchdown()
plt.ioff()
plt.show()