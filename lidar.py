"""The fake LiDAR: it shoots laser beams all around the robot and measures distances."""
import numpy as np
from world import RES, NX, NY

NUM_BEAMS = 180        # 180 beams, one every 2 degrees, all the way around
MAX_RANGE = 4.0        # metres (same as the real VL53L1X sensor)
STEP = 0.025           # we walk along each beam in 2.5 cm steps
ROBOT_RADIUS = 0.15    # metres

# The angle of every beam, in radians: 0, 2, 4, ... 358 degrees.
ANGLES = np.deg2rad(np.arange(NUM_BEAMS) * 2.0)


def scan(house, x, y, noise_sigma=0.02, rng=None):
    """Do one 360-degree scan from position (x, y).

    Returns three arrays, one value per beam:
      angles    - beam angle in radians
      distances - measured distance in metres
      hits      - True if the beam hit something, False if it reached max range
    """
    if rng is None:
        rng = np.random.default_rng(42)

    # All the distances we check along a beam: 0.025, 0.050, ... 4.0 m
    steps = np.arange(1, int(round(MAX_RANGE / STEP)) + 1) * STEP

    # Points along every beam at once (rows = beams, columns = steps).
    px = x + np.cos(ANGLES)[:, None] * steps[None, :]
    py = y + np.sin(ANGLES)[:, None] * steps[None, :]
    cols = np.floor(px / RES).astype(int)
    rows = np.floor(py / RES).astype(int)

    # A point is "solid" if it is inside a wall, or outside the house.
    inside = (cols >= 0) & (cols < NX) & (rows >= 0) & (rows < NY)
    solid = np.ones_like(inside)
    solid[inside] = house[rows[inside], cols[inside]]

    # For each beam, find the first step that is solid.
    hits = solid.any(axis=1)
    first = np.argmax(solid, axis=1)
    distances = np.where(hits, steps[first], MAX_RANGE)

    # Add Gaussian noise to the hits only, then keep distances in 0.01..4.0 m.
    noise = rng.normal(0.0, noise_sigma, NUM_BEAMS) if noise_sigma > 0 else np.zeros(NUM_BEAMS)
    distances = np.where(hits, distances + noise, distances)
    distances = np.clip(distances, 0.01, MAX_RANGE)

    return ANGLES.copy(), distances, hits
