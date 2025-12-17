import pyhula
import time

# Initialize and connect
api = pyhula.UserApi()
if not api.connect():
    print("Connection failed")
    exit()

print("Connected successfully")

# Maze parameters
BLOCK_SIZE = 60  # cm per block
SPEED = 50  # Reduced speed (0-100 cm/s)
FLIGHT_HEIGHT = 80  # cm

# First block center coordinates with offset
OFFSET_X = 0  # cm offset
OFFSET_Y = 0  # cm offset
FIRST_BLOCK_CENTER_X = 15 + OFFSET_X  # cm
FIRST_BLOCK_CENTER_Y = 15 + OFFSET_Y  # cm


# Function to convert block coordinates to absolute cm coordinates
def block_to_cm(block_x, block_y):
    """Convert block coordinates to absolute cm coordinates (center of block)"""
    abs_x = FIRST_BLOCK_CENTER_X + (block_x * BLOCK_SIZE)
    abs_y = FIRST_BLOCK_CENTER_Y + (block_y * BLOCK_SIZE)
    return abs_x, abs_y


def move_to_coordinates(x, y, h):
    """Move to coordinates using straight flight"""
    api.single_fly_straight_flight(x, y, h)


# Get maze parameters
rows = int(input("Enter number of rows (Y-axis, forward): "))
cols = int(input("Enter number of columns (X-axis, left/right): "))

start_x = int(input("Enter start X (column 0 to {}): ".format(cols - 1)))
start_y = int(input("Enter start Y (row 0 to {}): ".format(rows - 1)))

end_x = int(input("Enter end X (column 0 to {}): ".format(cols - 1)))
end_y = int(input("Enter end Y (row 0 to {}): ".format(rows - 1)))

print(f"\nMaze: {rows} rows (Y) x {cols} columns (X)")
print(f"Start: (X={start_x}, Y={start_y})")
print(f"End: (X={end_x}, Y={end_y})")
print(f"Offset: X={OFFSET_X}cm, Y={OFFSET_Y}cm")
print("Y-axis = Forward/Backward, X-axis = Left/Right")

# Enable obstacle avoidance
api.single_fly_barrier_aircraft(True)

# Turn on QR code positioning
print("Turning on QR code positioning...")
api.Plane_cmd_switch_QR(0)
time.sleep(2)

# Takeoff
print("\nTaking off...")
api.single_fly_takeoff()
time.sleep(3)

# Initialize to starting position
start_abs_x, start_abs_y = block_to_cm(start_x, start_y)
print(
    f"Moving to starting position: Block ({start_x}, {start_y}) = ({start_abs_x}cm, {start_abs_y}cm, {FLIGHT_HEIGHT}cm)")
move_to_coordinates(start_abs_x, start_abs_y, FLIGHT_HEIGHT)
time.sleep(2)

# Current position in blocks
current_x = start_x
current_y = start_y

# Track visited positions with visit count to detect loops
visited = {}
visited[(current_x, current_y)] = 1

# Track last move to avoid immediate reversal
last_move = None

# Track consecutive failed attempts
stuck_counter = 0
max_stuck = 15

# Navigate to end point
while current_x != end_x or current_y != end_y:
    obstacles = api.Plane_getBarrier()

    print(f"\nCurrent position: Block (X={current_x}, Y={current_y})")
    print(f"Obstacles detected: {obstacles}")

    # Calculate Manhattan distance to goal for each possible move
    move_options = []

    # All possible moves: (dx, dy, direction_name, has_obstacle)
    all_moves = [
        (0, 1, 'forward', obstacles.get('forward', False)),
        (0, -1, 'back', obstacles.get('back', False)),
        (1, 0, 'right', obstacles.get('right', False)),
        (-1, 0, 'left', obstacles.get('left', False))
    ]

    # Evaluate each move - ONLY add valid moves (no obstacle, in bounds)
    for dx, dy, direction, has_obstacle in all_moves:
        new_x = current_x + dx
        new_y = current_y + dy

        # Skip if obstacle detected
        if has_obstacle:
            print(f"  Skipping {direction}: obstacle detected")
            continue

        # Skip if out of bounds
        if not (0 <= new_x < cols and 0 <= new_y < rows):
            print(f"  Skipping {direction}: out of bounds")
            continue

        # Calculate Manhattan distance to goal
        distance_to_goal = abs(new_x - end_x) + abs(new_y - end_y)

        # Get visit count (0 if never visited)
        visit_count = visited.get((new_x, new_y), 0)

        # Check if this is reversing the last move
        is_reverse = False
        if last_move:
            is_reverse = (dx == -last_move[0] and dy == -last_move[1])

        # Calculate absolute coordinates for this block
        abs_x, abs_y = block_to_cm(new_x, new_y)

        move_options.append({
            'dx': dx,
            'dy': dy,
            'direction': direction,
            'new_x': new_x,
            'new_y': new_y,
            'abs_x': abs_x,
            'abs_y': abs_y,
            'distance': distance_to_goal,
            'visit_count': visit_count,
            'is_reverse': is_reverse
        })

        print(
            f"  Valid move {direction} to Block ({new_x}, {new_y}) = ({abs_x}cm, {abs_y}cm): visits={visit_count}, distance={distance_to_goal}, reverse={is_reverse}")

    if not move_options:
        stuck_counter += 1
        print(f"No valid moves available! Stuck counter: {stuck_counter}")

        if stuck_counter >= max_stuck:
            print("Unable to find path. Drone is stuck!")
            break

        time.sleep(2)
        continue

    # Sort moves by priority:
    # 1. Unvisited moves closer to goal
    # 2. Less visited moves closer to goal
    # 3. Avoid reversing last move unless necessary
    move_options.sort(key=lambda m: (
        m['visit_count'],  # Prioritize less visited
        m['distance'],  # Then closer to goal
        m['is_reverse'],  # Avoid reversing
    ))

    # Try to make a move
    moved = False
    for move in move_options:
        # Prefer unvisited or lightly visited cells
        if move['visit_count'] < 3:  # Allow revisiting up to 3 times
            print(
                f"\nExecuting: Flying {move['direction']} from Block (X={current_x}, Y={current_y}) to Block (X={move['new_x']}, Y={move['new_y']})")
            print(f"  -> Absolute coordinates: ({move['abs_x']}cm, {move['abs_y']}cm, {FLIGHT_HEIGHT}cm)")
            print(f"  -> Stats: visits={move['visit_count']}, distance to goal={move['distance']}")

            # Use straight_flight to target coordinates
            move_to_coordinates(move['abs_x'], move['abs_y'], FLIGHT_HEIGHT)

            # Update position
            last_move = (move['dx'], move['dy'])
            current_x = move['new_x']
            current_y = move['new_y']

            # Update visit count
            visited[(current_x, current_y)] = visited.get((current_x, current_y), 0) + 1

            moved = True
            stuck_counter = 0
            break

    if not moved:
        stuck_counter += 1
        print(f"All available moves too visited. Stuck counter: {stuck_counter}")

        if stuck_counter >= max_stuck:
            print("Unable to find path. Drone is stuck!")
            break

    time.sleep(2)  # Wait longer like in the example

if current_x == end_x and current_y == end_y:
    print(f"\n✓ Reached destination: Block (X={end_x}, Y={end_y})")
    print(f"  Explored {len(visited)} unique blocks")
else:
    print(f"\n✗ Stopped at: Block (X={current_x}, Y={current_y})")
    print(f"  Explored {len(visited)} unique blocks")

# Land
print("\nLanding...")
api.single_fly_touchdown()