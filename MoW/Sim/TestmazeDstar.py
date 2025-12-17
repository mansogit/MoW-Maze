"""
D* Lite maze navigation with Manhattan distance display
"""

import pyhula_simulator as pyhula
from maze_config import ROWS, COLS, START, END, WALLS
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
end_x, end_y = END

print(f"\nMaze: {rows} rows x {cols} columns")
print(f"Start: ({start_x}, {start_y}), End: ({end_x}, {end_y})")

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

# Initialize D* Lite
dstar = DStarLite(START, END, rows, cols)
dstar.compute_shortest_path()

# Setup plot
plt.ion()
fig, ax = plt.subplots(figsize=(10, 8))
ax.set_xlim(0, cols * BLOCK)
ax.set_ylim(0, rows * BLOCK)
ax.set_aspect('equal')
ax.grid(True, alpha=0.3)
ax.set_title('D* Lite Maze Navigation')

# Draw grid
for i in range(cols + 1):
    ax.axvline(i * BLOCK, color='black', linewidth=2)
for i in range(rows + 1):
    ax.axhline(i * BLOCK, color='black', linewidth=2)

# Draw start/end
ax.add_patch(Rectangle((start_x * BLOCK + 0.05, start_y * BLOCK + 0.05),
                        BLOCK - 0.1, BLOCK - 0.1, facecolor='green', alpha=0.5))
ax.add_patch(Rectangle((end_x * BLOCK + 0.05, end_y * BLOCK + 0.05),
                        BLOCK - 0.1, BLOCK - 0.1, facecolor='red', alpha=0.5))

# Draw initial distances (will be updated)
distance_texts = {}
for x in range(cols):
    for y in range(rows):
        g_val = dstar.g[(x, y)]
        txt_str = str(int(g_val)) if g_val != float('inf') else '∞'
        txt = ax.text(x * BLOCK + BLOCK/2, y * BLOCK + BLOCK/2, txt_str,
                      ha='center', va='center', fontsize=20, fontweight='bold', color='black',
                      bbox=dict(boxstyle='round', facecolor='white', alpha=0.8, edgecolor='none'))
        distance_texts[(x, y)] = txt

drone_pos, = ax.plot([], [], 'bo', markersize=15, zorder=5)
path_line, = ax.plot([], [], 'b-', linewidth=2, alpha=0.5)
detected_walls = set()

def block_to_cm(block_x, block_y):
    return 15 + (block_x * BLOCK_SIZE), 15 + (block_y * BLOCK_SIZE)

def update_distance_display():
    """Update all distance numbers based on current D* Lite g values"""
    for x in range(cols):
        for y in range(rows):
            g_val = dstar.g[(x, y)]
            txt_str = str(int(g_val)) if g_val != float('inf') else '∞'
            distance_texts[(x, y)].set_text(txt_str)

def update_plot(obstacles, current_x, current_y):
    drone_pos.set_data([current_x * BLOCK + BLOCK/2], [current_y * BLOCK + BLOCK/2])
    path_line.set_data([x * BLOCK + BLOCK/2 for x in path_x], [y * BLOCK + BLOCK/2 for y in path_y])

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
    plt.pause(0.1)
    return new_walls

# Start navigation
api.single_fly_barrier_aircraft(True)
api.Plane_cmd_switch_QR(0)
time.sleep(0.5)

api.single_fly_takeoff()
time.sleep(0.5)

current_x, current_y = start_x, start_y
abs_x, abs_y = block_to_cm(current_x, current_y)
api.single_fly_straight_flight(abs_x, abs_y, FLIGHT_HEIGHT)

path_x, path_y = [current_x], [current_y]
steps = 0

while (current_x, current_y) != (end_x, end_y):
    obstacles = api.Plane_getBarrier()
    new_walls = update_plot(obstacles, current_x, current_y)

    print(f"Position: ({current_x},{current_y}), Obstacles: {obstacles}")

    # Update D* Lite with new walls
    if new_walls:
        dstar.update_walls(new_walls)
        update_distance_display()

    # Get next move
    next_pos = dstar.get_next_move()

    if next_pos is None:
        print("No path found!")
        break

    # Move
    current_x, current_y = next_pos
    dstar.move_to(next_pos)

    abs_x, abs_y = block_to_cm(current_x, current_y)
    api.single_fly_straight_flight(abs_x, abs_y, FLIGHT_HEIGHT)

    path_x.append(current_x)
    path_y.append(current_y)
    steps += 1

    time.sleep(0.3)

update_plot({}, current_x, current_y)

if (current_x, current_y) == (end_x, end_y):
    print(f"\n✓ Reached destination in {steps} steps!")
else:
    print(f"\n✗ Stopped at ({current_x}, {current_y})")

api.single_fly_touchdown()
plt.ioff()
plt.show()