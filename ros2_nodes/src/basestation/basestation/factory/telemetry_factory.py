"""
Default telemetry state factory.

Provides the initial/fallback Status payload that matches the web client's
Status type (statusTypes.ts). Used when ROS2 topics are unavailable, and as
the seed state for the factory (fake-data) thread started by `start()` below
when the app is run with `--factory` / `FACTORY_MODE=1`.
"""

import logging
import math
import random
import threading
import time
from collections import deque

log = logging.getLogger(__name__)

# Statuses cycled through by the factory loop — mirrors the subset ros2_bridge's
# _ACTION_TO_STATUS actually produces from real Task messages (the other two
# TaskValues, "lost connection" / "out of control", are error states the bridge
# never assigns on its own).
_TASK_STATUSES = ["standby", "autonomous", "remote"]

_LOG_MESSAGES = [
    "Heartbeat OK",
    "GPS fix acquired",
    "Waypoint reached",
    "Adjusting heading to course",
    "Battery nominal",
    "ZED odometry synced",
    "Obstacle cleared",
    "Holding station",
]

# How often the factory thread mutates the shared state, in seconds.
_TICK_S = 0.5


def make_default_state() -> dict:
    """
    Returns an idle Status dict with zero/empty values, mirroring
    visualizer/src/types/statusTypes.ts's InitialStatus.
    Fields are updated in-place by ros2_bridge as real data arrives.
    """
    return {
        "map": {
            "occupancyGrid": [],
            "navigationGrid": [],
            "courseTrail": [],
        },
        "planning": {
            "status": "idle",
            "course": [],
            "plan": [],
        },
        "task": {
            "log": [],
            "location": {"latitude": 0.0, "longitude": 0.0},
            "data": {"id": 0, "name": "", "status": "standby", "latitude": 0.0, "longitude": 0.0},
        },
        "rudder": {
            "angle": 0.0,
        },
        "battery": {
            "motors": 0.0,
            "primary": 0.0,
        },
        "motors": {
            "left": 0.0,
            "right": 0.0,
        },
        "asv": {
            "speed": 0.0,
            "heading": 0.0,
            "longitude": 0.0,
            "latitude": 0.0,
        },
        "signal": {
            "strength": 100.0,
        },
        "zed": {
            "odom": {
                "position": {"x": 0.0, "y": 0.0, "z": 0.0},
                "orientation": {"roll": 0.0, "pitch": 0.0, "yaw": 0.0},
            },
            "objects": [],
            "camera": {
                "active": False,
                "width": 0,
                "height": 0,
                "encoding": "",
            },
        },
    }


# ---------------------------------------------------------------------------
# Factory (fake-data) thread — mutates a shared state dict the same way
# ros2_bridge's node does, so app.py can swap one for the other based on
# --factory / FACTORY_MODE. Used when there's no ASV / ROS2 to talk to but
# the web client still needs something dynamic to render.
# ---------------------------------------------------------------------------

def _clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def _step(min_step: float, max_step: float) -> float:
    """Random +/- delta, magnitude in [min_step, max_step]."""
    return random.uniform(min_step, max_step) * random.choice((-1.0, 1.0))


def _append_log(log_list: list, entry: str) -> None:
    if not log_list or log_list[-1] != entry:
        log_list.append(entry)
    if len(log_list) > 50:
        del log_list[:-50]


# ---------------------------------------------------------------------------
# Fake occupancy/navigation grid + path — a Python port of the grid/path
# generation in visualizer/src/utils/telemetry/telemetryFactory.ts
# (createFineGrid / deriveCoarseGrid / createNavigationGrid / createPlanning),
# so the shapes and numeric cell types line up exactly with what the web
# client (react-isometric-engine's Grid/Path/CellTypes) expects on the wire.
#
# Regenerating this is not cheap (Gaussian blur over a 100x100 grid in pure
# Python), so it's only done once at factory start and occasionally
# thereafter (see _MAP_REGEN_CHANCE in _loop) — never every tick.
# ---------------------------------------------------------------------------

# CellTypes numeric values — must match react-isometric-engine's CellTypes
# (node_modules/react-isometric-engine/dist/index.js), not just the labels.
_CELL_EMPTY = 0
_CELL_OCCUPIED = 1
_CELL_PATH = 2
_CELL_CURRENT = 3
_CELL_OBJECTIVE = 4
_CELL_ERROR = 5

_GLOBAL_GRID_SIZE = 20              # react-isometric-engine GLOBAL_GRID_SIZE
_FINE_SCALE = 5                     # GLOBAL_CELL_SIZE / LOCAL_CELL_SIZE (40 / 8)
_FINE_GRID_SIZE = _GLOBAL_GRID_SIZE * _FINE_SCALE  # 100x100
_FINE_SIGMA = _FINE_SCALE * 2.0     # visually equivalent to sigma=2 on the 20x20 grid

# Chance, per _TICK_S tick, of regenerating the map/route — simulates a new
# task being issued without needing a "Next Task" button. ~1/50 ticks at the
# default 0.5s interval works out to roughly once every 25s on average.
_MAP_REGEN_CHANCE = 0.01


def _gaussian_kernel_1d(sigma: float) -> list:
    radius = math.ceil(sigma * 3)
    kernel = [math.exp(-(k * k) / (2 * sigma * sigma)) for k in range(-radius, radius + 1)]
    total = sum(kernel)
    return [v / total for v in kernel]


def _gaussian_blur_separable(noise: list, sigma: float) -> list:
    """Horizontal pass then vertical pass — same approach as the JS
    applyGaussianBlurSeparable, just without a fast typed-array path."""
    h = len(noise)
    w = len(noise[0])
    kernel = _gaussian_kernel_1d(sigma)
    r = len(kernel) // 2

    temp = [[0.0] * w for _ in range(h)]
    for row in range(h):
        for col in range(w):
            acc = 0.0
            wsum = 0.0
            for k, kv in enumerate(kernel):
                nc = col + k - r
                if 0 <= nc < w:
                    acc += noise[row][nc] * kv
                    wsum += kv
            temp[row][col] = acc / wsum if wsum > 0 else 0.0

    out = [[0.0] * w for _ in range(h)]
    for row in range(h):
        for col in range(w):
            acc = 0.0
            wsum = 0.0
            for k, kv in enumerate(kernel):
                nr = row + k - r
                if 0 <= nr < h:
                    acc += temp[nr][col] * kv
                    wsum += kv
            out[row][col] = acc / wsum if wsum > 0 else 0.0
    return out


def _make_cell(x: int, y: int, cell_type: int = _CELL_EMPTY, z: float = 0.0) -> dict:
    return {"type": cell_type, "x": x, "y": y, "z": z}


def _create_fine_grid() -> list:
    """High-resolution occupancy grid at LOCAL_CELL_SIZE precision — obstacle
    islands have the same footprint as the coarse grid once downsampled.
    Mirrors createFineGrid()."""
    size = _FINE_GRID_SIZE
    noise = [[random.random() for _ in range(size)] for _ in range(size)]
    blurred = _gaussian_blur_separable(noise, _FINE_SIGMA)

    lo = min(v for row in blurred for v in row)
    hi = max(v for row in blurred for v in row)
    span = (hi - lo) or 1.0
    threshold = 0.75

    grid = []
    for row in range(size):
        grid_row = []
        for col in range(size):
            normalized = (blurred[row][col] - lo) / span
            cell_type = _CELL_OCCUPIED if normalized > threshold else _CELL_EMPTY
            grid_row.append(_make_cell(col, row, cell_type, normalized))
        grid.append(grid_row)
    return grid


def _derive_coarse_grid(fine_grid: list) -> list:
    """Downsamples the fine grid to GLOBAL_GRID_SIZE — a coarse cell is occupied
    if any of its FINE_SCALE x FINE_SCALE fine cells is occupied. Mirrors
    deriveCoarseGrid()."""
    coarse_size = _GLOBAL_GRID_SIZE
    grid = []
    for coarse_row in range(coarse_size):
        row_cells = []
        fine_row_start = coarse_row * _FINE_SCALE
        for coarse_col in range(coarse_size):
            fine_col_start = coarse_col * _FINE_SCALE
            occupied = False
            for dr in range(_FINE_SCALE):
                for dc in range(_FINE_SCALE):
                    if fine_grid[fine_row_start + dr][fine_col_start + dc]["type"] == _CELL_OCCUPIED:
                        occupied = True
                        break
                if occupied:
                    break
            row_cells.append(_make_cell(coarse_col, coarse_row, _CELL_OCCUPIED if occupied else _CELL_EMPTY))
        grid.append(row_cells)
    return grid


def _bfs_path(occupancy_grid: list, start_x: int, start_y: int, goal_x: int, goal_y: int):
    """4-directional BFS from (start_x, start_y) to (goal_x, goal_y). Returns
    (path, closest) — path is None if unreachable; closest (the nearest cell
    actually reached) is always populated. Mirrors bfsPath()."""
    h = len(occupancy_grid)
    w = len(occupancy_grid[0])

    parent = {(start_x, start_y): None}
    queue = deque([(start_x, start_y)])

    def dist(x, y):
        return abs(x - goal_x) + abs(y - goal_y)

    closest = (start_x, start_y)
    closest_dist = dist(start_x, start_y)

    dirs = [(1, 0), (-1, 0), (0, 1), (0, -1)]

    while queue:
        x, y = queue.popleft()

        d = dist(x, y)
        if d < closest_dist:
            closest_dist = d
            closest = (x, y)

        if (x, y) == (goal_x, goal_y):
            path = []
            cur = (x, y)
            while cur is not None:
                path.append(cur)
                cur = parent[cur]
            path.reverse()
            return path, closest

        for dx, dy in dirs:
            nx, ny = x + dx, y + dy
            if nx < 0 or nx >= w or ny < 0 or ny >= h:
                continue
            if (nx, ny) in parent:
                continue
            if occupancy_grid[ny][nx]["type"] == _CELL_OCCUPIED:
                continue
            parent[(nx, ny)] = (x, y)
            queue.append((nx, ny))

    return None, closest


def _create_navigation_grid(occupancy_grid: list):
    """BFS a path from a start cell (top-left quadrant) to a goal cell
    (bottom-right quadrant); if unreachable, marks the closest reachable cell
    as an error instead. Mirrors createNavigationGrid()."""
    h = len(occupancy_grid)
    w = len(occupancy_grid[0])

    navigation_grid = [[_make_cell(cell["x"], cell["y"]) for cell in row] for row in occupancy_grid]

    free_cells = [
        (col, row)
        for row in range(h)
        for col in range(w)
        if occupancy_grid[row][col]["type"] != _CELL_OCCUPIED
    ]

    if len(free_cells) < 2:
        return navigation_grid, []

    q = max(1, len(free_cells) // 4)
    start = free_cells[random.randint(0, q - 1)]
    goal = free_cells[random.randint(len(free_cells) - q, len(free_cells) - 1)]

    bfs_result, closest = _bfs_path(occupancy_grid, start[0], start[1], goal[0], goal[1])

    path = []
    if bfs_result:
        for x, y in bfs_result:
            cell = _make_cell(x, y, _CELL_PATH)
            path.append(cell)
            navigation_grid[y][x] = dict(cell)
    else:
        cx, cy = closest
        navigation_grid[cy][cx] = _make_cell(cx, cy, _CELL_ERROR)
        start_cell = _make_cell(start[0], start[1], _CELL_PATH)
        path.append(start_cell)
        navigation_grid[start[1]][start[0]] = dict(start_cell)

    return navigation_grid, path


def _create_planning(navigation_grid: list, path: list):
    """Splits `path` into `course` (just the current position) and `plan`
    (the remaining route), and marks the current/objective cells on
    navigation_grid in place. Mirrors createPlanning() — currentIndex is
    always 0, since the factory has no task-progress concept, just a
    freshly-issued route."""
    current_index = 0
    course = [dict(path[i], type=_CELL_PATH) for i in range(current_index + 1)] if path else []
    plan = [dict(path[i], type=_CELL_PATH) for i in range(current_index, len(path))] if path else []

    current = path[current_index] if path else _make_cell(0, 0, _CELL_CURRENT)
    navigation_grid[current["y"]][current["x"]] = dict(current, type=_CELL_CURRENT)

    last_path_cell = path[-1] if path else current
    if last_path_cell["x"] != current["x"] or last_path_cell["y"] != current["y"]:
        navigation_grid[last_path_cell["y"]][last_path_cell["x"]] = dict(last_path_cell, type=_CELL_OBJECTIVE)

    return course, plan


def _generate_map_and_plan() -> dict:
    """Builds a brand-new fake occupancyGrid/navigationGrid/course/plan — the
    Python equivalent of the web client's generateRandomState() map
    generation. Pure computation, touches no shared state, so callers should
    run it *before* taking the state lock."""
    fine_grid = _create_fine_grid()
    occupancy_grid = _derive_coarse_grid(fine_grid)
    navigation_grid, path = _create_navigation_grid(occupancy_grid)
    course, plan = _create_planning(navigation_grid, path)
    return {
        "occupancyGrid": occupancy_grid,
        "navigationGrid": navigation_grid,
        "course": course,
        "plan": plan,
    }


def _apply_map(state: dict, generated: dict) -> None:
    """Splices a _generate_map_and_plan() result into `state`. Caller holds the lock."""
    state["map"]["occupancyGrid"] = generated["occupancyGrid"]
    state["map"]["navigationGrid"] = generated["navigationGrid"]
    state["planning"]["course"] = generated["course"]
    state["planning"]["plan"] = generated["plan"]


def _randomize_initial(state: dict) -> None:
    """Seed `state` with plausible non-zero starting values — make_default_state()'s
    zeros are fine as an idle placeholder, but factory mode is meant to look alive."""
    state["asv"]["heading"] = random.uniform(0, 360)
    state["asv"]["speed"] = random.uniform(0.5, 3.0)
    state["asv"]["latitude"] = random.uniform(43.6, 43.7)
    state["asv"]["longitude"] = random.uniform(-79.6, -79.5)
    state["task"]["location"]["latitude"] = state["asv"]["latitude"]
    state["task"]["location"]["longitude"] = state["asv"]["longitude"]
    state["rudder"]["angle"] = random.uniform(-10, 10)
    state["motors"]["left"] = random.uniform(40, 90)
    state["motors"]["right"] = random.uniform(40, 90)
    state["battery"]["motors"] = random.uniform(60, 100)
    state["battery"]["primary"] = random.uniform(60, 100)
    state["signal"]["strength"] = random.uniform(80, 100)
    state["planning"]["status"] = random.choice(_TASK_STATUSES)
    state["task"]["data"]["status"] = state["planning"]["status"]
    _append_log(state["task"]["log"], "[FACTORY] Factory telemetry stream started.")


def _tick(state: dict) -> None:
    """Mutate `state` in place with one fake-telemetry step. Caller holds the lock."""
    asv = state["asv"]
    asv["heading"] = (asv["heading"] + _step(0.5, 4.0)) % 360.0
    asv["speed"] = _clamp(asv["speed"] + _step(0.05, 0.3), 0.0, 5.0)

    heading_rad = math.radians(asv["heading"])
    dist = asv["speed"] * _TICK_S
    asv["latitude"] += math.cos(heading_rad) * dist * 0.00001
    asv["longitude"] += math.sin(heading_rad) * dist * 0.00001
    state["task"]["location"]["latitude"] = asv["latitude"]
    state["task"]["location"]["longitude"] = asv["longitude"]

    state["rudder"]["angle"] = _clamp(state["rudder"]["angle"] + _step(1.0, 5.0), -45.0, 45.0)

    state["motors"]["left"] = _clamp(state["motors"]["left"] + _step(0.5, 3.0), 0.0, 100.0)
    state["motors"]["right"] = _clamp(state["motors"]["right"] + _step(0.5, 3.0), 0.0, 100.0)

    # Batteries drift downward (a slow discharge), everything else random-walks.
    state["battery"]["motors"] = _clamp(state["battery"]["motors"] - random.uniform(0.0, 0.05), 0.0, 100.0)
    state["battery"]["primary"] = _clamp(state["battery"]["primary"] - random.uniform(0.0, 0.03), 0.0, 100.0)

    state["signal"]["strength"] = _clamp(state["signal"]["strength"] + _step(0.5, 3.0), 60.0, 100.0)

    zed = state["zed"]["odom"]
    zed["position"]["x"] = asv["longitude"]
    zed["position"]["y"] = asv["latitude"]
    zed["orientation"]["yaw"] = asv["heading"]

    if random.random() < 0.05:
        state["planning"]["status"] = random.choice(_TASK_STATUSES)
        state["task"]["data"]["status"] = state["planning"]["status"]

    if random.random() < 0.2:
        _append_log(state["task"]["log"], f"[FACTORY] {random.choice(_LOG_MESSAGES)}")


def _loop(state: dict, lock: threading.Lock, interval: float) -> None:
    log.info("Factory telemetry thread running (interval=%.2fs)", interval)
    while True:
        time.sleep(interval)
        # Generate outside the lock — the Gaussian blur is the one expensive
        # part of a tick and shouldn't hold up /telemetry's reader thread.
        new_map = _generate_map_and_plan() if random.random() < _MAP_REGEN_CHANCE else None
        with lock:
            _tick(state)
            if new_map is not None:
                _apply_map(state, new_map)
                _append_log(state["task"]["log"], "[FACTORY] New task issued — course replotted.")


def start(state: dict, lock: threading.Lock, interval: float = _TICK_S) -> threading.Thread:
    """
    Start the factory (fake) telemetry generator in a daemon thread, mutating
    `state` in place under `lock` — mirrors ros2_bridge.start()'s signature so
    app.py can pick one bridge or the other based on --factory / FACTORY_MODE.
    """
    initial_map = _generate_map_and_plan()
    with lock:
        _randomize_initial(state)
        _apply_map(state, initial_map)
    t = threading.Thread(target=_loop, args=(state, lock, interval), daemon=True, name="telemetry-factory")
    t.start()
    log.info("Telemetry factory thread started.")
    return t
