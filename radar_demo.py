"""Step 1: one 360-degree LiDAR scan from the start position.

Makes:
  results/radar_scan.gif   - a radar-style sweep showing each hit as the beam passes it
  results/one_scan_map.png - the real house with the beams, and the robot's map after one scan
"""
import numpy as np
import matplotlib
matplotlib.use("Agg")  # draw to files, not to a window
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
import imageio

from world import make_house, START, WIDTH_M, HEIGHT_M, ROOMS
from lidar import scan, MAX_RANGE, ROBOT_RADIUS
from mapping import new_map, update_map, map_image

EXTENT = [0, WIDTH_M, 0, HEIGHT_M]  # so pictures use metres on the axes


def fig_to_array(fig):
    """Turn a matplotlib figure into an image array for the GIF."""
    fig.canvas.draw()
    return np.asarray(fig.canvas.buffer_rgba())[:, :, :3].copy()


def make_radar_gif(angles, distances, hits, path):
    """Radar view: a beam line sweeps around, and hits light up as it passes them."""
    # Hit points relative to the robot (robot is at 0, 0 in this picture).
    hx = distances * np.cos(angles)
    hy = distances * np.sin(angles)

    fig, ax = plt.subplots(figsize=(5, 5), dpi=80)
    fig.patch.set_facecolor("black")
    ax.set_facecolor("black")
    ax.set_xlim(-MAX_RANGE, MAX_RANGE)
    ax.set_ylim(-MAX_RANGE, MAX_RANGE)
    ax.set_aspect("equal")
    ax.axis("off")
    # Range rings every 1 m, like a real radar screen.
    for r in range(1, int(MAX_RANGE) + 1):
        ax.add_patch(Circle((0, 0), r, fill=False, color="#0a5a0a", lw=1))
        ax.text(r * 0.71, r * 0.71, f"{r} m", color="#0a8a0a", fontsize=8)
    ax.plot(0, 0, "o", color="cyan", ms=6)
    ax.set_title("LiDAR scan (radar view)", color="#3f3", fontsize=12)

    beam, = ax.plot([], [], color="#3f3", lw=2)
    dots = ax.scatter([], [], s=8, color="#3f3")

    frames = []
    n_frames = 60                        # 60 frames at 20 fps = 3 seconds
    for i in range(1, n_frames + 1):
        sweep = 2 * np.pi * i / n_frames  # how far the beam has turned so far
        beam.set_data([0, MAX_RANGE * np.cos(sweep)], [0, MAX_RANGE * np.sin(sweep)])
        shown = hits & (angles <= sweep)  # hits the beam has already passed
        dots.set_offsets(np.column_stack([hx[shown], hy[shown]]) if shown.any() else np.empty((0, 2)))
        frames.append(fig_to_array(fig))
    plt.close(fig)
    imageio.mimsave(path, frames, duration=50, loop=0)  # 50 ms per frame


def make_one_scan_png(house, logodds, angles, distances, path):
    """Two panels: real house with beams (left), robot's map after one scan (right)."""
    x, y = START
    fig, axes = plt.subplots(1, 2, figsize=(12, 5), dpi=100)

    ax = axes[0]
    ax.imshow(~house, cmap="gray", origin="lower", extent=EXTENT, vmin=0, vmax=1)
    for a, d in zip(angles, distances):
        ax.plot([x, x + d * np.cos(a)], [y, y + d * np.sin(a)], color="red", lw=0.5, alpha=0.5)
    ax.add_patch(Circle((x, y), ROBOT_RADIUS, color="blue"))
    for name, (lx, ly) in ROOMS.items():
        ax.text(lx, ly, name, ha="center", va="center", fontsize=9, color="green")
    ax.set_title("Real house + 180 laser beams")

    ax = axes[1]
    ax.imshow(map_image(logodds), cmap="gray", origin="lower", extent=EXTENT, vmin=0, vmax=1)
    ax.add_patch(Circle((x, y), ROBOT_RADIUS, color="blue"))
    ax.set_title("Robot's map after one scan")

    for ax in axes:
        ax.set_xlabel("x (m)")
        ax.set_ylabel("y (m)")
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)


def main():
    rng = np.random.default_rng(42)       # fixed seed so results repeat
    house = make_house()
    x, y = START
    angles, distances, hits = scan(house, x, y, noise_sigma=0.02, rng=rng)

    logodds = new_map()
    update_map(logodds, x, y, angles, distances, hits)

    make_radar_gif(angles, distances, hits, "results/radar_scan.gif")
    make_one_scan_png(house, logodds, angles, distances, "results/one_scan_map.png")
    print(f"Beams: {len(angles)}, hits: {hits.sum()}, misses: {(~hits).sum()}")
    print("Saved results/radar_scan.gif and results/one_scan_map.png")


if __name__ == "__main__":
    main()
