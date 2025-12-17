"""
pyhula simulator - drop-in replacement for testing maze navigation
Matches API from Hula_Python_interface_specifier_V3_20250724.pdf
"""

MAZE_WALLS = set()

_state = {
    'x': 0, 'y': 0, 'z': 0,
    'yaw': 0, 'pitch': 0, 'roll': 0,  # degrees
    'flying': False,
    'facing': 0  # 0=N, 1=E, 2=S, 3=W (for grid navigation)
}
_config = {'rows': 5, 'cols': 5, 'block_size': 60}

_DIR_DELTA = [(0, 1), (1, 0), (0, -1), (-1, 0)]  # N, E, S, W

def get_version():
    return "1.1.7-simulator"

def set_walls(walls):
    global MAZE_WALLS
    MAZE_WALLS = set(tuple(sorted([tuple(a), tuple(b)])) for a, b in walls)

def set_maze_size(rows, cols):
    _config['rows'] = rows
    _config['cols'] = cols

class UserApi:
    def connect(self, server_ip=None):
        print(f"[SIM] Connected" + (f" to {server_ip}" if server_ip else ""))
        return True

    # === Takeoff / Landing ===
    def single_fly_takeoff(self, led=0):
        _state['flying'] = True
        _state['z'] = 80
        print("[SIM] Takeoff")

    def single_fly_touchdown(self, led=0):
        _state['flying'] = False
        _state['z'] = 0
        print("[SIM] Landing")

    def single_fly_hover_flight(self, time, led=0):
        print(f"[SIM] Hover for {time}s")

    # === Directional Flight (relative to drone facing) ===
    def single_fly_forward(self, distance, speed=100, led=0):
        dx, dy = _DIR_DELTA[_state['facing']]
        _state['x'] += dx * distance
        _state['y'] += dy * distance
        print(f"[SIM] Forward {distance}cm")

    def single_fly_back(self, distance, speed=100, led=0):
        dx, dy = _DIR_DELTA[_state['facing']]
        _state['x'] -= dx * distance
        _state['y'] -= dy * distance
        print(f"[SIM] Back {distance}cm")

    def single_fly_left(self, distance, speed=100, led=0):
        left_dir = (_state['facing'] + 3) % 4  # Left is -90 degrees
        dx, dy = _DIR_DELTA[left_dir]
        _state['x'] += dx * distance
        _state['y'] += dy * distance
        print(f"[SIM] Left {distance}cm")

    def single_fly_right(self, distance, speed=100, led=0):
        right_dir = (_state['facing'] + 1) % 4  # Right is +90 degrees
        dx, dy = _DIR_DELTA[right_dir]
        _state['x'] += dx * distance
        _state['y'] += dy * distance
        print(f"[SIM] Right {distance}cm")

    def single_fly_up(self, distance, speed=100, led=0):
        _state['z'] += distance
        print(f"[SIM] Up {distance}cm")

    def single_fly_down(self, distance, speed=100, led=0):
        _state['z'] -= distance
        print(f"[SIM] Down {distance}cm")

    # === Turning (yaw control) ===
    def single_fly_turnleft(self, angle, led=0):
        _state['yaw'] = (_state['yaw'] - angle) % 360
        # Update facing for grid navigation
        turns = round(angle / 90) % 4
        _state['facing'] = (_state['facing'] - turns) % 4
        print(f"[SIM] Turn left {angle}° (yaw={_state['yaw']}°)")

    def single_fly_turnright(self, angle, led=0):
        _state['yaw'] = (_state['yaw'] + angle) % 360
        # Update facing for grid navigation
        turns = round(angle / 90) % 4
        _state['facing'] = (_state['facing'] + turns) % 4
        print(f"[SIM] Turn right {angle}° (yaw={_state['yaw']}°)")

    # === Absolute Position Flight ===
    def single_fly_straight_flight(self, x, y, z, speed=100, led=0):
        old_x = _state['x']
        old_y = _state['y']

        _state['x'] = x
        _state['y'] = y
        _state['z'] = z

        # Calculate movement direction and update facing
        dx = x - old_x
        dy = y - old_y

        # Update facing based on primary movement direction
        if abs(dx) > abs(dy):
            if dx > 0:
                _state['facing'] = 1  # East (+X)
            elif dx < 0:
                _state['facing'] = 3  # West (-X)
        elif abs(dy) > abs(dx):
            if dy > 0:
                _state['facing'] = 0  # North (+Y)
            elif dy < 0:
                _state['facing'] = 2  # South (-Y)

        bx = round((x - 15) / _config['block_size'])
        by = round((y - 15) / _config['block_size'])

        print(f"[SIM] Move to ({x},{y},{z}) - Block ({bx},{by})")

    # === Special Maneuvers ===
    def single_fly_bounce(self, frequency, height, led=0):
        print(f"[SIM] Bounce {frequency}x at {height}cm")

    def single_fly_radius_around(self, radius, led=0):
        direction = "anticlockwise" if radius > 0 else "clockwise"
        print(f"[SIM] Circle radius {abs(radius)}cm {direction}")

    def single_fly_autogyration360(self, num, led=0):
        direction = "anticlockwise" if num > 0 else "clockwise"
        _state['yaw'] = (_state['yaw'] + num * 360) % 360
        print(f"[SIM] Spin {abs(num)} turns {direction}")

    def single_fly_somersault(self, direction, led=0):
        dirs = ['forward', 'back', 'left', 'right']
        print(f"[SIM] Somersault {dirs[direction]}")

    def single_fly_curvilinearFlight(self, x, y, z, direction=True, speed=100, led=0):
        d = "anticlockwise" if direction else "clockwise"
        print(f"[SIM] Curve to ({x},{y},{z}) {d}")

    # === Obstacle Avoidance ===
    def single_fly_barrier_aircraft(self, mode):
        print(f"[SIM] Obstacle avoidance: {mode}")

    def Plane_getBarrier(self):
        """Returns obstacles in absolute directions (forward=North/+Y) - matches real API"""
        bx = round((_state['x'] - 15) / _config['block_size'])
        by = round((_state['y'] - 15) / _config['block_size'])

        return {
            'forward': self._is_blocked(bx, by, bx, by + 1),  # North +Y
            'back': self._is_blocked(bx, by, bx, by - 1),     # South -Y
            'right': self._is_blocked(bx, by, bx + 1, by),    # East +X
            'left': self._is_blocked(bx, by, bx - 1, by)      # West -X
        }

    def Plane_getBarrier_relative(self):
        """Returns obstacles relative to drone's current facing direction"""
        bx = round((_state['x'] - 15) / _config['block_size'])
        by = round((_state['y'] - 15) / _config['block_size'])
        facing = _state['facing']

        abs_blocked = []
        for i in range(4):
            dx, dy = _DIR_DELTA[i]
            abs_blocked.append(self._is_blocked(bx, by, bx + dx, by + dy))

        return {
            'forward': abs_blocked[facing],
            'right': abs_blocked[(facing + 1) % 4],
            'back': abs_blocked[(facing + 2) % 4],
            'left': abs_blocked[(facing + 3) % 4]
        }

    def Plane_getBarrier_absolute(self):
        """Alias for Plane_getBarrier - returns absolute directions"""
        return self.Plane_getBarrier()

    def _is_blocked(self, x1, y1, x2, y2):
        if not (0 <= x2 < _config['cols'] and 0 <= y2 < _config['rows']):
            return True
        edge = tuple(sorted([(x1, y1), (x2, y2)]))
        return edge in MAZE_WALLS

    # === QR Code / Positioning ===
    def Plane_cmd_switch_QR(self, type):
        status = "on" if type == 0 else "off"
        print(f"[SIM] QR positioning: {status}")

    def single_fly_Optical_flow_alignment(self, qr_id, qr_size=20, angle=0):
        print(f"[SIM] Align to QR {qr_id}")
        return True

    def single_fly_Optical_flow_recognition(self, qr_id, qr_size=20):
        return {'result': True, 'x': 0, 'y': 0, 'z': 0, 'yaw': 0, 'qr_id': qr_id}

    def single_fly_Proactive_alignment(self, qr_id):
        print(f"[SIM] Front camera align QR {qr_id}")
        return True

    def single_fly_Anticipatory_recognition(self, qr_id):
        return {'result': True, 'x': 0, 'y': 0, 'z': 0, 'yaw': 0, 'qr_id': qr_id}

    def single_fly_track_Qrcode(self, qr_id, time):
        print(f"[SIM] Track QR {qr_id} for {time}s")
        return 1

    def single_fly_AiIdentifies(self, mode):
        return {'x': 0, 'y': 0, 'z': 0, 'angle': 0, 'result': True}

    # === Line Following ===
    def single_fly_Line_walking(self, fun_id, dist, way_color):
        print(f"[SIM] Line walk {dist}cm")
        return 1

    # === Camera / Video ===
    def Plane_cmd_swith_rtp(self, type):
        status = "on" if type == 0 else "off"
        print(f"[SIM] Video stream: {status}")

    def single_fly_flip_rtp(self):
        print("[SIM] Open video window")

    def Plane_fly_take_photo(self):
        print("[SIM] Photo taken")

    def Plane_cmd_switch_video(self, type):
        status = "start" if type == 0 else "stop"
        print(f"[SIM] Recording: {status}")

    def Plane_cmd_camera_angle(self, type, data):
        print(f"[SIM] Camera angle: {data}°")

    def single_fly_getColor(self):
        return {'r': 128, 'g': 128, 'b': 128, 'state': 1}

    # === LED / Lights ===
    def single_fly_lamplight(self, r, g, b, time, mode):
        print(f"[SIM] LED ({r},{g},{b}) mode={mode} for {time}s")

    # === Laser ===
    def plane_fly_generating(self, type, data, reserve):
        types = ['single shot', 'keep shooting', 'receiver on', 'receiver off', 'keep firing', 'laser off']
        print(f"[SIM] Laser: {types[type]}")

    def plane_fly_laser_receiving(self):
        return False

    # === Motor Control ===
    def plane_fly_arm(self):
        print("[SIM] Motors armed")

    def plane_fly_disarm(self):
        print("[SIM] Motors disarmed")

    # === Getters ===
    def get_battery(self):
        return 100

    def get_coordinate(self):
        return [_state['x'], _state['y'], _state['z']]

    def get_yaw(self):
        return [_state['yaw'], _state['pitch'], _state['roll']]

    def get_plane_speed(self):
        return [0, 0, 0]

    def get_plane_distance(self):
        return _state['z']

    def get_plane_id(self):
        return 1

    # === Electromagnet / Clamp ===
    def Plane_cmd_electromagnet(self, type):
        action = "attract" if type == 2 else "release"
        print(f"[SIM] Electromagnet: {action}")

    def Plane_cmd_clamp(self, type, angle=0):
        actions = ['disable', 'enable', f'angle {angle}°', 'magnet release', 'magnet attract']
        print(f"[SIM] Clamp: {actions[type]}")