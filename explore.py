"""Frontier exploration (Yamauchi, 1997) and A* path planning (Hart, Nilsson & Raphael, 1968)."""
import heapq
from collections import deque
import numpy as np

from world import make_house, START, RES, to_cell, cell_center
from lidar import scan
from mapping import new_map, update_map, free_cells, occupied_cells, unknown_cells

SAFETY_M = 0.18        # robot radius (0.15 m) plus a small margin
ESCAPE_CELLS = 6       # 30 cm: near the robot we ignore the safety zone
MOVE_M = 0.5           # move up to 0.5 m (10 cells) per turn
IGNORE_HALF = 3        # ignore a 7 x 7 area (3 cells each side) around a reached target
MAX_SCANS = 400

# The 8 directions the robot can move: (row change, col change).
DIRS8 = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]


def inflate(mask, radius_m):
    """Grow the True cells outward by radius_m (makes the 'safety zone')."""
    r = radius_m / RES
    R = int(np.ceil(r))
    ny, nx = mask.shape
    padded = np.pad(mask, R)
    out = np.zeros_like(mask)
    for dr in range(-R, R + 1):
        for dc in range(-R, R + 1):
            if dr * dr + dc * dc <= r * r:   # only cells inside the circle
                out |= padded[R + dr:R + dr + ny, R + dc:R + dc + nx]
    return out


def safety_zone(logodds):
    """Map's occupied cells grown by 0.18 m. The robot can't plan through these."""
    return inflate(occupied_cells(logodds), SAFETY_M)


def find_frontiers(logodds, zone, ignored):
    """Frontier = free cell, outside the safety zone, touching an unknown cell (up/down/left/right)."""
    unknown = unknown_cells(logodds)
    touches = np.zeros_like(unknown)
    touches[1:, :] |= unknown[:-1, :]
    touches[:-1, :] |= unknown[1:, :]
    touches[:, 1:] |= unknown[:, :-1]
    touches[:, :-1] |= unknown[:, 1:]
    return free_cells(logodds) & ~zone & touches & ~ignored


def walkable_for_search(logodds, zone, robot_cell):
    """Cells the robot may drive through: free and not in the safety zone.
    Escape rule: within 30 cm of the robot, only the map's occupied cells block it."""
    free = free_cells(logodds)
    walk = free & ~zone
    r0, c0 = robot_cell
    rows, cols = np.ogrid[:logodds.shape[0], :logodds.shape[1]]
    near = (rows - r0) ** 2 + (cols - c0) ** 2 <= ESCAPE_CELLS ** 2
    walk[near] = free[near] & ~occupied_cells(logodds)[near]
    return walk


def bfs_to_frontier(walk, frontiers, start):
    """Breadth-first search (8 directions) for the nearest frontier.
    Returns the path as a list of cells (start first), or None if none can be reached."""
    ny, nx = walk.shape
    parent = {start: None}
    queue = deque([start])
    while queue:
        cell = queue.popleft()
        if frontiers[cell]:
            path = []
            while cell is not None:      # walk back to the start to get the path
                path.append(cell)
                cell = parent[cell]
            return path[::-1]
        r, c = cell
        for dr, dc in DIRS8:
            nb = (r + dr, c + dc)
            if 0 <= nb[0] < ny and 0 <= nb[1] < nx and nb not in parent and walk[nb]:
                parent[nb] = cell
                queue.append(nb)
    return None


def astar(blocked, start, goal):
    """A* search: 8-direction moves, straight-line distance as the guess (heuristic).
    Returns the path as a list of cells, or None if there is no path."""
    ny, nx = blocked.shape

    def h(cell):
        return np.hypot(cell[0] - goal[0], cell[1] - goal[1])

    cost = {start: 0.0}
    parent = {start: None}
    heap = [(h(start), start)]
    while heap:
        _, cell = heapq.heappop(heap)
        if cell == goal:
            path = []
            while cell is not None:
                path.append(cell)
                cell = parent[cell]
            return path[::-1]
        r, c = cell
        for dr, dc in DIRS8:
            nb = (r + dr, c + dc)
            if not (0 <= nb[0] < ny and 0 <= nb[1] < nx) or blocked[nb]:
                continue
            new_cost = cost[cell] + np.hypot(dr, dc)
            if new_cost < cost.get(nb, np.inf):
                cost[nb] = new_cost
                parent[nb] = cell
                heapq.heappush(heap, (new_cost + h(nb), nb))
    return None


def path_length_m(path):
    """Length of a path of cells, in metres."""
    p = np.array(path)
    return float(np.sum(np.hypot(*np.diff(p, axis=0).T)) * RES)


# ---------------- Measurements ----------------

def reachable_floor(house):
    """Flood fill from the start through the real house, with walls grown by 0.18 m."""
    open_ = ~inflate(house, SAFETY_M)
    start = to_cell(*START)
    seen = np.zeros_like(house)
    seen[start] = True
    queue = deque([start])
    while queue:
        r, c = queue.popleft()
        for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            nr, nc = r + dr, c + dc
            if 0 <= nr < house.shape[0] and 0 <= nc < house.shape[1] and open_[nr, nc] and not seen[nr, nc]:
                seen[nr, nc] = True
                queue.append((nr, nc))
    return seen


def wall_edges(house):
    """Real wall cells that touch open floor (up, down, left or right)."""
    floor = ~house
    touches = np.zeros_like(house)
    touches[1:, :] |= floor[:-1, :]
    touches[:-1, :] |= floor[1:, :]
    touches[:, 1:] |= floor[:, :-1]
    touches[:, :-1] |= floor[:, 1:]
    return house & touches


def coverage(logodds, reachable):
    """% of reachable floor that the map marks as free."""
    return 100.0 * np.sum(free_cells(logodds) & reachable) / np.sum(reachable)


def accuracy(logodds, house):
    """Of all cells the map marks free or occupied, the % that match the real house."""
    free, occ = free_cells(logodds), occupied_cells(logodds)
    correct = np.sum(free & ~house) + np.sum(occ & house)
    return 100.0 * correct / np.sum(free | occ)


def walls_found(logodds, edges):
    """% of real wall cells (that touch open floor) the map marks as occupied."""
    return 100.0 * np.sum(occupied_cells(logodds) & edges) / np.sum(edges)


# ---------------- The exploration loop ----------------

def explore(noise_sigma=0.02, seed=42, record=False):
    """Let the robot explore the house on its own.

    Returns a dict with the final map, the stats, and (if record=True) one
    snapshot per scan for making the GIF.
    """
    rng = np.random.default_rng(seed)
    house = make_house()
    reachable = reachable_floor(house)
    edges = wall_edges(house)

    logodds = new_map()
    ignored = np.zeros(house.shape, dtype=bool)
    x, y = START
    trail = [(x, y)]
    driven = 0.0
    bumps = 0
    scans = 0
    curve = []      # (metres driven, coverage %) after each scan
    frames = []     # snapshots for the GIF

    while scans < MAX_SCANS:
        # 1. Scan and update the map.
        angles, distances, hits = scan(house, x, y, noise_sigma, rng)
        update_map(logodds, x, y, angles, distances, hits)
        scans += 1
        cov = coverage(logodds, reachable)
        curve.append((driven, cov))
        if record:
            frames.append({
                "map": logodds.copy(), "x": x, "y": y, "trail": list(trail),
                "angles": angles, "distances": distances,
                "coverage": cov, "driven": driven,
            })

        # 2. Find the nearest frontier we can reach.
        robot = to_cell(x, y)
        zone = safety_zone(logodds)
        walk = walkable_for_search(logodds, zone, robot)
        walk[robot] = True                     # the robot can always stand where it is
        path = None
        while True:
            frontiers = find_frontiers(logodds, zone, ignored)
            path = bfs_to_frontier(walk, frontiers, robot)
            if path is None or len(path) > 1:
                break
            # The robot is standing on its target already: ignore it and look again.
            r, c = path[0]
            ignored[max(r - IGNORE_HALF, 0):r + IGNORE_HALF + 1, max(c - IGNORE_HALF, 0):c + IGNORE_HALF + 1] = True
        if path is None:
            break                              # nothing left to explore

        # 3. Move up to 0.5 m along the path.
        target = path[-1]
        moved = 0.0
        reached = False
        for cell in path[1:]:
            nx_, ny_ = cell_center(*cell)
            step = np.hypot(nx_ - x, ny_ - y)
            if moved + step > MOVE_M + 1e-9:   # this step would go past 0.5 m
                break
            if house[cell]:                    # bump check against the REAL house
                bumps += 1
                break
            moved += step
            x, y = nx_, ny_
            trail.append((x, y))
            reached = cell == target
        driven += moved
        if reached:
            # Ignore rule: we got there, so stop chasing this spot.
            r, c = target
            ignored[max(r - IGNORE_HALF, 0):r + IGNORE_HALF + 1, max(c - IGNORE_HALF, 0):c + IGNORE_HALF + 1] = True

    return {
        "map": logodds, "house": house, "trail": trail, "curve": curve, "frames": frames,
        "scans": scans, "driven": driven, "bumps": bumps,
        "coverage": coverage(logodds, reachable),
        "accuracy": accuracy(logodds, house),
        "walls_found": walls_found(logodds, edges),
    }
