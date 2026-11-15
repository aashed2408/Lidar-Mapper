"""The house floor plan: the "real world" that the robot explores.

The house is stored as a grid of small square cells (5 cm each).
A cell is True if it is solid (a wall or furniture) and False if it is open floor.
grid[row, col] means grid[y, x]: rows go up the house, columns go across.
"""
import numpy as np

RES = 0.05            # size of one grid cell in metres (5 cm)
WIDTH_M = 10.0        # house width (x) in metres
HEIGHT_M = 8.0        # house height (y) in metres
NX = 200              # number of cells across (x)
NY = 160              # number of cells up (y)

START = (1.0, 3.0)    # where the robot starts, in metres (Living room)

# Every solid rectangle as (x0, y0, x1, y1) in metres.
RECTANGLES = [
    # Outer walls (10 cm thick)
    (0, 0, 10, 0.1), (0, 7.9, 10, 8), (0, 0, 0.1, 8), (9.9, 0, 10, 8),
    # Middle wall with 2 doorways
    (5, 0, 5.1, 1.5), (5, 2.5, 5.1, 5.5), (5, 6.5, 5.1, 8),
    # Left-side wall with 1 doorway
    (0, 4, 2, 4.1), (3, 4, 5, 4.1),
    # Right-side wall with 1 doorway
    (5, 3, 7, 3.1), (8, 3, 10, 3.1),
    # Furniture: couch, table, bed
    (1, 1, 2.2, 1.8), (2.5, 5.5, 3.5, 7), (7.5, 5, 9, 6),
]

# Room names and where to put their labels (in metres).
ROOMS = {
    "Living room": (2.5, 2.5),
    "Kitchen": (2.5, 7.5),
    "Office": (7.5, 1.5),
    "Bedroom": (7.5, 7.5),
}


def make_house():
    """Build the house grid. Returns a 160 x 200 array of True (solid) / False (floor)."""
    grid = np.zeros((NY, NX), dtype=bool)
    for x0, y0, x1, y1 in RECTANGLES:
        # Turn metres into cell numbers. The end number is not included.
        c0, c1 = round(x0 / RES), round(x1 / RES)
        r0, r1 = round(y0 / RES), round(y1 / RES)
        grid[r0:r1, c0:c1] = True
    return grid


def to_cell(x, y):
    """Turn a position in metres into (row, col) of the grid cell it is in."""
    return int(np.floor(y / RES)), int(np.floor(x / RES))


def cell_center(row, col):
    """Turn a grid cell (row, col) into the (x, y) position of its centre in metres."""
    return (col + 0.5) * RES, (row + 0.5) * RES
