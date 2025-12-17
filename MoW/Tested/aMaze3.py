import pyhula
import time
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import matplotlib.patches as mpatches

# Initialize and connect
api = pyhula.UserApi()
if not api.connect():
    print("Connection failed")
    exit()

print("Connected successfully")

# Maze parameters
BLOCK_SIZE = 60  # cm per block
SPEED = 50
FLIGHT_HEIGHT = 80

def block_to_cm(block_x, block_y):
    """Convert block coordinates to absolute cm coordinates
    Block (0,0) is centered at (15cm, 15cm)
    """
    abs_x = 15 + (block_x * BLOCK_SIZE)
    abs_y = 15 + (block_y * BLOCK_SIZE)
    return abs_x, abs_y

def move_to_coordinates(x, y, h):
    api.single_fly_straight_flight(x, y, h)

# Get maze parameters
rows = int(input("Enter number of rows (Y-axis, forward): "))
cols = int(input("Enter number of columns (X-axis, left/right): "))
start_x = int(input("Enter start X (column 0 to {}): ".format(cols - 1)))
start_y = int(input("Enter start Y (row 0 to {}): ".format(rows - 1)))
end_x = int(input("Enter end X (column 0 to {}): ".format(cols - 1)))
end_y = int(input("Enter end Y (row 0 to {}): ".format(rows - 1)))

print(f"\nMaze: {rows} rows (Y) x {cols} columns (X)")
print(f"Start: (X={start_x}, Y={start_y}), End: (X={end_x}, Y={end_y})")

# Setup plot
plt.ion()
fig, ax = plt.subplots(figsize=(10, 10))
ax.set_xlim(-0.5, cols - 0.5)
ax.set_ylim(-0.5, rows - 0.5)
ax.set_aspect('equal')
ax.grid(True, alpha=0.3)
ax.set_xlabel('X (columns)')
ax.set_ylabel('Y (rows)')
ax.set_title('Drone Maze Navigation')

# Draw maze grid
for i in range(cols + 1):
    ax.axvline(i - 0.5, color='black', linewidth=2)
for i in range(rows + 1):
    ax.axhline(i - 0.5, color='black', linewidth=2)

# Draw start and end
ax.add_patch(Rectangle((start_x - 0.4, start_y - 0.4), 0.8, 0.8, 
                        facecolor='green', alpha=0.5, label='Start'))
ax.add_patch(Rectangle((end_x - 0.4, end_y - 0.4), 0.8, 0.8, 
                        facecolor='red', alpha=0.5, label='End'))

# Initialize plot elements
drone_pos, = ax.plot([], [], 'bo', markersize=15, label='Drone')
path_line, = ax.plot([], [], 'b-', linewidth=2, alpha=0.5, label='Path')
obstacle_patches = []

# Add wall legend entry as a line
wall_legend, = ax.plot([], [], 'r-', linewidth=5, label='Walls')

ax.legend()
plt.draw()
plt.pause(0.1)

# Enable obstacle avoidance and QR positioning
api.single_fly_barrier_aircraft(True)
print("Turning on QR code positioning...")
api.Plane_cmd_switch_QR(0)
time.sleep(2)

# Takeoff
print("\nTaking off...")
api.single_fly_takeoff()
time.sleep(3)

# Initialize to starting position
start_abs_x, start_abs_y = block_to_cm(start_x, start_y)
move_to_coordinates(start_abs_x, start_abs_y, FLIGHT_HEIGHT)
time.sleep(2)

current_x = start_x
current_y = start_y
visited = {(current_x, current_y): 1}
last_move = None
stuck_counter = 0
max_stuck = 15

# Track path for plotting
path_x = [current_x]
path_y = [current_y]

# Track detected walls permanently (as edges between blocks)
detected_walls = set()

# Update plot function
def update_plot(obstacles):
    # Update drone position
    drone_pos.set_data([current_x], [current_y])
    path_line.set_data(path_x, path_y)
    
    # Add newly detected walls as edges
    for direction, has_obstacle in obstacles.items():
        if has_obstacle:
            # Calculate wall position as edge between current block and obstacle
            if direction == 'forward':
                wall_key = (current_x, current_y + 0.5, 'h')  # horizontal wall above
                x1, x2 = current_x - 0.5, current_x + 0.5
                y1 = y2 = current_y + 0.5
            elif direction == 'back':
                wall_key = (current_x, current_y - 0.5, 'h')  # horizontal wall below
                x1, x2 = current_x - 0.5, current_x + 0.5
                y1 = y2 = current_y - 0.5
            elif direction == 'right':
                wall_key = (current_x + 0.5, current_y, 'v')  # vertical wall right
                x1 = x2 = current_x + 0.5
                y1, y2 = current_y - 0.5, current_y + 0.5
            elif direction == 'left':
                wall_key = (current_x - 0.5, current_y, 'v')  # vertical wall left
                x1 = x2 = current_x - 0.5
                y1, y2 = current_y - 0.5, current_y + 0.5
            else:
                continue
            
            if wall_key not in detected_walls:
                detected_walls.add(wall_key)
                # Draw wall as thick red line
                wall_line, = ax.plot([x1, x2], [y1, y2], 'r-', linewidth=5, solid_capstyle='butt')
                obstacle_patches.append(wall_line)
    
    plt.draw()
    plt.pause(0.01)

# Navigate to end point
while current_x != end_x or current_y != end_y:
    obstacles = api.Plane_getBarrier()
    
    # Update plot
    update_plot(obstacles)
    
    print(f"\nCurrent: Block (X={current_x}, Y={current_y}), Obstacles: {obstacles}")
    
    move_options = []
    all_moves = [
        (0, 1, 'forward', obstacles.get('forward', False)),
        (0, -1, 'back', obstacles.get('back', False)),
        (1, 0, 'right', obstacles.get('right', False)),
        (-1, 0, 'left', obstacles.get('left', False))
    ]
    
    for dx, dy, direction, has_obstacle in all_moves:
        new_x = current_x + dx
        new_y = current_y + dy
        
        if has_obstacle or not (0 <= new_x < cols and 0 <= new_y < rows):
            continue
        
        distance_to_goal = abs(new_x - end_x) + abs(new_y - end_y)
        visit_count = visited.get((new_x, new_y), 0)
        is_reverse = last_move and (dx == -last_move[0] and dy == -last_move[1])
        abs_x, abs_y = block_to_cm(new_x, new_y)
        
        move_options.append({
            'dx': dx, 'dy': dy, 'direction': direction,
            'new_x': new_x, 'new_y': new_y,
            'abs_x': abs_x, 'abs_y': abs_y,
            'distance': distance_to_goal,
            'visit_count': visit_count,
            'is_reverse': is_reverse
        })
    
    if not move_options:
        stuck_counter += 1
        print(f"No valid moves! Stuck counter: {stuck_counter}")
        if stuck_counter >= max_stuck:
            print("Drone is stuck!")
            break
        time.sleep(2)
        continue
    
    move_options.sort(key=lambda m: (m['visit_count'], m['distance'], m['is_reverse']))
    
    moved = False
    for move in move_options:
        if move['visit_count'] < 3:
            print(f"Flying {move['direction']} to Block ({move['new_x']}, {move['new_y']})")
            move_to_coordinates(move['abs_x'], move['abs_y'], FLIGHT_HEIGHT)
            
            last_move = (move['dx'], move['dy'])
            current_x = move['new_x']
            current_y = move['new_y']
            visited[(current_x, current_y)] = visited.get((current_x, current_y), 0) + 1
            
            path_x.append(current_x)
            path_y.append(current_y)
            
            moved = True
            stuck_counter = 0
            break
    
    if not moved:
        stuck_counter += 1
        if stuck_counter >= max_stuck:
            print("Unable to find path!")
            break
    
    time.sleep(2)

# Final plot update
update_plot({})

if current_x == end_x and current_y == end_y:
    print(f"\n✓ Reached destination! Explored {len(visited)} blocks")
else:
    print(f"\n✗ Stopped at Block ({current_x}, {current_y})")

print("\nLanding...")
api.single_fly_touchdown()

plt.ioff()
plt.show()
