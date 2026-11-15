"""The robot's map: an occupancy grid built from (angle, distance) readings.

Each cell holds a "log-odds" number:
  negative = probably free, positive = probably occupied, 0 = unknown.
This file does not know anything about the fake house, so next week it can be
used unchanged with readings from the real sensor.
"""
import numpy as np

RES = 0.05          # cell size in metres
NX, NY = 200, 160   # map size in cells (same as the house)
MAX_RANGE = 4.0     # metres

FREE_ADD = -0.4     # added to a cell a beam passes through
OCC_ADD = 0.85      # added to the cell where a beam hit something
CLAMP = 5.0         # values are kept between -5 and +5
THRESHOLD = 0.3     # below -0.3 = free, above +0.3 = occupied


def new_map():
    """A fresh map: every cell is 0 (unknown)."""
    return np.zeros((NY, NX))


def update_map(logodds, x, y, angles_rad, distances_m, hits):
    """Add one set of readings, taken from position (x, y), to the map.

    angles_rad  - beam angles in radians
    distances_m - measured distances in metres
    hits        - True if that beam hit something
    Works with any number of readings. Changes `logodds` in place and returns it.
    """
    angles_rad = np.asarray(angles_rad, dtype=float)
    distances_m = np.asarray(distances_m, dtype=float)
    hits = np.asarray(hits, dtype=bool)
    ny, nx = logodds.shape

    for angle, dist, hit in zip(angles_rad, distances_m, hits):
        dx, dy = np.cos(angle), np.sin(angle)

        # Where the beam ended, and which cell that is.
        end_col = int(np.floor((x + dx * dist) / RES))
        end_row = int(np.floor((y + dy * dist) / RES))

        # How far along the beam the cells are free.
        # For a hit: stop one cell short of the hit point, so noise can't erase walls.
        # For a miss: free all the way to max range.
        free_len = dist - RES if hit else MAX_RANGE

        # Walk along the beam in half-cell steps and collect the cells we pass.
        s = np.arange(0.0, max(free_len, 0.0) + 1e-9, RES / 2)
        cols = np.floor((x + dx * s) / RES).astype(int)
        rows = np.floor((y + dy * s) / RES).astype(int)
        keep = (cols >= 0) & (cols < nx) & (rows >= 0) & (rows < ny)
        if hit:
            keep &= ~((rows == end_row) & (cols == end_col))  # never free the hit cell
        cells = set(zip(rows[keep], cols[keep]))               # each cell only once per beam
        for r, c in cells:
            logodds[r, c] += FREE_ADD

        # The cell at the hit point is occupied.
        if hit and 0 <= end_row < ny and 0 <= end_col < nx:
            logodds[end_row, end_col] += OCC_ADD

    np.clip(logodds, -CLAMP, CLAMP, out=logodds)
    return logodds


def free_cells(logodds):
    """True where the map thinks the cell is free."""
    return logodds < -THRESHOLD


def occupied_cells(logodds):
    """True where the map thinks the cell is occupied."""
    return logodds > THRESHOLD


def unknown_cells(logodds):
    """True where the map does not know yet."""
    return ~free_cells(logodds) & ~occupied_cells(logodds)


def map_image(logodds):
    """Turn the map into a picture: gray = unknown, white = free, black = occupied."""
    img = np.full(logodds.shape, 0.5)
    img[free_cells(logodds)] = 1.0
    img[occupied_cells(logodds)] = 0.0
    return img
