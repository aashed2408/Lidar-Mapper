"""Step 4: plan a path across the house with A*, using the map the robot built in Step 2.

Makes results/path_plan.png
"""
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from world import WIDTH_M, HEIGHT_M, ROOMS, START, RES, to_cell, cell_center
from mapping import map_image, unknown_cells
from explore import explore, safety_zone, astar, path_length_m

GOAL = (9.0, 7.2)   # in the Bedroom


def main():
    # The final map from Step 2 (same noise and seed, so it's the same map).
    result = explore(noise_sigma=0.02, seed=42)
    logodds = result["map"]

    # Unknown cells and the safety zone are blocked.
    blocked = unknown_cells(logodds) | safety_zone(logodds)
    path = astar(blocked, to_cell(*START), to_cell(*GOAL))
    if path is None:
        print("No path found!")
        return

    length = path_length_m(path)
    straight = float(np.hypot(GOAL[0] - START[0], GOAL[1] - START[1]))

    # Check: how close does the path get to a real wall? (distance between cell centres)
    house = result["house"]
    wall_rc = np.argwhere(house)
    closest = min(np.min(np.hypot(*(wall_rc - np.array(c)).T)) for c in path) * RES
    print(f"A* path: {length:.2f} m, straight line: {straight:.2f} m, "
          f"closest approach to a real wall: {closest * 100:.0f} cm")

    px, py = zip(*[cell_center(*c) for c in path])
    fig, ax = plt.subplots(figsize=(9, 7.5), dpi=100)
    ax.imshow(map_image(logodds), cmap="gray", origin="lower",
              extent=[0, WIDTH_M, 0, HEIGHT_M], vmin=0, vmax=1)
    ax.plot(px, py, color="tab:blue", lw=2.5, label=f"A* path ({length:.1f} m)")
    ax.plot([START[0], GOAL[0]], [START[1], GOAL[1]], "--", color="red", lw=1.5,
            label=f"Straight line ({straight:.1f} m)")
    ax.plot(*START, "o", color="green", ms=10, label="Start")
    ax.plot(*GOAL, "*", color="orange", ms=16, label="Goal")
    for name, (lx, ly) in ROOMS.items():
        ax.text(lx, ly, name, ha="center", va="center", fontsize=11, color="purple",
                bbox=dict(facecolor="white", alpha=0.7, edgecolor="none"))
    ax.set_title(f"A* path: {length:.1f} m vs straight line: {straight:.1f} m")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.legend(loc="lower right")
    fig.tight_layout()
    fig.savefig("results/path_plan.png")
    plt.close(fig)
    print("Saved results/path_plan.png")


if __name__ == "__main__":
    main()
