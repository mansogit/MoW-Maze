"""
D* Lite maze navigation - visits every block
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


# D* Lite implementation
class DStarLite:
    def __init__(self, start, goal, rows, cols):
        self.start = start
        self.goal = goal
        self.rows = rows
        self.cols = cols
        self.km = 0
        self.g = {}
        self.rhs = {}
        self.U = []
        self.walls = set()

        for x in range(cols):
            for y in range(rows):
                self.g[(x, y)] = float('inf')
                self.rhs[(x, y)] = float('inf')

        self.rhs[goal] = 0
        heapq.heappush(self.U, (self._calculate_key(goal), goal))

    def _heuristic(self, a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def _calculate_key(self, s):
        return (min(self.g[s], self.rhs[s]) + self._heuristic(self.start, s) + self.km,
                min(self.g[s], self.rhs[s]))

    def _get_neighbors(self, s):
        neighbors = []
        for dx, dy in [(0, 1), (0, -1), (1, 0), (-1, 0)]:
            nx, ny = s[0] + dx, s[1] + dy
            if 0 <= nx < self.cols and 0 <= ny < self.rows:
                edge = tuple(sorted([s, (nx, ny)]))
                if edge not in self.walls:
                    neighbors.append((nx, ny))
        return neighbors

    def _cost(self, a, b):
        edge = tuple(sorted([a, b]))
        if edge in self.walls:
            return float('inf')
        return 1

    def _update_vertex(self, u):
        if u != self.goal:
            min_rhs = float('inf')
            for s in self._get_neighbors(u):
                min_rhs = min(min_rhs, self._cost(u, s) + self.g[s])
            self.rhs[u] = min_rhs

        self.U = [(k, v) for k, v in self.U if v != u]
        heapq.heapify(self.U)

        if self.g[u] != self.rhs[u]:
            heapq.heappush(self.U, (self._calculate_key(u), u))

    def compute_shortest_path(self):
        while self.U and (self.U[0][0] < self._calculate_key(self.start) or
                          self.rhs[self.start] != self.g[self.start]):
            k_old, u = heapq.heappop(self.U)
            k_new = self._calculate_key(u)

            if k_old < k_new:
                heapq.heappush(self.U, (k_new, u))
            elif self.g[u] > self.rhs[u]:
                self.g[u] = self.rhs[u]
                for s in self._get_neighbors(u):
                    self._update_vertex(s)
            else:
                self.g[u] = float('inf')
                self._update_vertex(u)
                for s in self._get_neighbors(u):
                    self._update_vertex(s)

    def get_next_move(self):
        if self.g[self.start] == float('inf'):
            return None

        best = None
        best_cost = float('inf')
        for s in self._get_neighbors(self.start):
            cost = self._cost(self.start, s) + self.g[s]
            if cost < best_cost:
                best_cost = cost
                best = s
        return best

    def update_walls(self, new_walls):
        self.km += self._heuristic(self.start, self.start)

        for wall in new_walls:
            if wall not in self.walls:
                self.walls.add(wall)
                a, b = wall
                if 0 <= a[0] < self.cols and 0 <= a[1] < self.rows:
                    self._update_vertex(a)
                if 0 <= b[0] < self.cols and 0 <= b[1] < self.rows:
                    self._update_vertex(b)

        self.compute_shortest_path()

    def move_to(self, new_start):
        self.km += self._heuristic(self.start, new_start)
        self.start = new_start

    def set_new_goal(self, new_goal):
        """Reset for new goal while keeping known walls"""
        old_walls = self.walls.copy()
        self.goal = new_goal
        self.km = 0
        self.U = []

        for x in range(self.cols):
            for y in range(self.rows):
                self.g[(x, y)] = float('inf')
                self.rhs[(x, y)] = float('inf')

        self.rhs[new_goal] = 0
        heapq.heappush(self.U, (self._calculate_key(new_goal), new_goal))
        self.walls = old_walls

        for wall in old_walls:
            a, b = wall
            if 0 <= a[0] < self.cols and 0 <= a[1] < self.rows:
                self._update_vertex(a)
            if 0 <= b[0] < self.cols and 0 <= b[1] < self.rows:
                self._update_vertex(b)

        self.compute_shortest_path()


# Setup plot
plt.ion()
fig, ax = plt.subplots(figsize=(10, 8))
ax.set_xlim(0, cols * BLOCK)
ax.set_ylim(0, rows * BLOCK)
ax.set_aspect('equal')
ax.grid(True, alpha=0.3)
title = ax.set_title('Visiting All Blocks - 0/' + str(rows * cols))

# Draw grid
for i in range(cols + 1):
    ax.axvline(i * BLOCK, color='black', linewidth=2)
for i in range(rows + 1):
    ax.axhline(i * BLOCK, color='black', linewidth=2)

# Track visited blocks with patches
visited_patches = {}
for x in range(cols):
    for y in range(rows):
        patch = ax.add_patch(Rectangle((x * BLOCK + 0.02, y * BLOCK + 0.02),
                                       BLOCK - 0.04, BLOCK - 0.04, facecolor='white', alpha=0.3))
        visited_patches[(x, y)] = patch

# Start marker
ax.add_patch(Rectangle((start_x * BLOCK + 0.05, start_y * BLOCK + 0.05),
                       BLOCK - 0.1, BLOCK - 0.1, facecolor='green', alpha=0.5, zorder=2))

drone_pos, = ax.plot([], [], 'bo', markersize=15, zorder=5)
path_line, = ax.plot([], [], 'b-', linewidth=2, alpha=0.5)
detected_walls = set()


def block_to_cm(block_x, block_y):
    return 15 + (block_x * BLOCK_SIZE), 15 + (block_y * BLOCK_SIZE)


def mark_visited(x, y):
    visited_patches[(x, y)].set_facecolor('lightgreen')
    visited_patches[(x, y)].set_alpha(0.5)


def update_plot(obstacles, current_x, current_y):
    drone_pos.set_data([current_x * BLOCK + BLOCK / 2], [current_y * BLOCK + BLOCK / 2])
    path_line.set_data([px * BLOCK + BLOCK / 2 for px in path_x], [py * BLOCK + BLOCK / 2 for py in path_y])

    new_walls = []
    for direction, has_obstacle in obstacles.items():
        if has_obstacle:
            if direction == 'forward':
                wall = tuple(sorted([(current_x, current_y), (current_x, current_y + 1)]))
                x1, x2 = current_x * BLOCK, (current_x + 1) * BLOCK
                y1 = y2 = (current_y + 1) * BLOCK
            elif direction == 'back':
                wall = tuple(sorted([(current_x, current_y), (current_x, current_y - 1)]))
                x1, x2 = current_x * BLOCK, (current_x + 1) * BLOCK
                y1 = y2 = current_y * BLOCK
            elif direction == 'right':
                wall = tuple(sorted([(current_x, current_y), (current_x + 1, current_y)]))
                x1 = x2 = (current_x + 1) * BLOCK
                y1, y2 = current_y * BLOCK, (current_y + 1) * BLOCK
            elif direction == 'left':
                wall = tuple(sorted([(current_x, current_y), (current_x - 1, current_y)]))
                x1 = x2 = current_x * BLOCK
                y1, y2 = current_y * BLOCK, (current_y + 1) * BLOCK
            else:
                continue

            if wall not in detected_walls:
                detected_walls.add(wall)
                new_walls.append(wall)
                ax.plot([x1, x2], [y1, y2], 'r-', linewidth=5)

    plt.draw()
    plt.pause(0.05)
    return new_walls


def get_nearest_unvisited(current, unvisited, dstar):
    """Find nearest unvisited block"""
    if not unvisited:
        return None

    best = None
    best_dist = float('inf')

    for target in unvisited:
        # Use D* Lite to check reachability and distance
        dstar.set_new_goal(target)
        dstar.move_to(current)
        dist = dstar.g[current]
        if dist < best_dist:
            best_dist = dist
            best = target

    return best


# Start navigation
api.single_fly_barrier_aircraft(True)
api.Plane_cmd_switch_QR(0)
time.sleep(0.5)

api.single_fly_takeoff()
time.sleep(0.5)

current_x, current_y = start_x, start_y
abs_x, abs_y = block_to_cm(current_x, current_y)
api.single_fly_straight_flight(abs_x, abs_y, FLIGHT_HEIGHT)

# Track all blocks and visited
all_blocks = {(x, y) for x in range(cols) for y in range(rows)}
visited = {(start_x, start_y)}
mark_visited(start_x, start_y)

path_x, path_y = [current_x], [current_y]
total_steps = 0

# Initialize D* Lite (will change goals as needed)
dstar = DStarLite((current_x, current_y), (current_x, current_y), rows, cols)
dstar.compute_shortest_path()
known_walls = set()

print(f"\n=== Starting exploration ===")

while len(visited) < len(all_blocks):
    # Scan current position
    obstacles = api.Plane_getBarrier()
    new_walls = update_plot(obstacles, current_x, current_y)

    if new_walls:
        for w in new_walls:
            known_walls.add(w)

    unvisited = all_blocks - visited
    title.set_text(f'Visiting All Blocks - {len(visited)}/{len(all_blocks)}')

    print(f"Position: ({current_x},{current_y}), Visited: {len(visited)}/{len(all_blocks)}")

    # Find nearest unvisited
    target = get_nearest_unvisited((current_x, current_y), unvisited, dstar)

    if target is None:
        print("Cannot reach remaining blocks!")
        break

    # Navigate to target
    dstar.set_new_goal(target)
    dstar.move_to((current_x, current_y))
    for w in known_walls:
        if w not in dstar.walls:
            dstar.walls.add(w)
    dstar.compute_shortest_path()

    while (current_x, current_y) != target:
        obstacles = api.Plane_getBarrier()
        new_walls = update_plot(obstacles, current_x, current_y)

        if new_walls:
            for w in new_walls:
                known_walls.add(w)
            dstar.update_walls(new_walls)

        next_pos = dstar.get_next_move()

        if next_pos is None:
            print(f"Cannot reach {target}, finding new target...")
            break

        current_x, current_y = next_pos
        dstar.move_to(next_pos)

        abs_x, abs_y = block_to_cm(current_x, current_y)
        api.single_fly_straight_flight(abs_x, abs_y, FLIGHT_HEIGHT)

        path_x.append(current_x)
        path_y.append(current_y)
        total_steps += 1

        if (current_x, current_y) not in visited:
            visited.add((current_x, current_y))
            mark_visited(current_x, current_y)
            title.set_text(f'Visiting All Blocks - {len(visited)}/{len(all_blocks)}')

        time.sleep(0.1)

update_plot({}, current_x, current_y)
title.set_text(f'Complete! Visited {len(visited)}/{len(all_blocks)} blocks in {total_steps} steps')

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