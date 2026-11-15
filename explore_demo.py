"""Step 2: the robot explores the whole house on its own.

Makes:
  results/exploration.gif       - real house (left) and the robot's map (right), one frame per scan
  results/coverage.png          - coverage % vs metres driven
  results/exploration_stats.md  - the final numbers
"""
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
from matplotlib.collections import LineCollection
import imageio

from world import WIDTH_M, HEIGHT_M
from lidar import ROBOT_RADIUS
from mapping import map_image
from explore import explore

EXTENT = [0, WIDTH_M, 0, HEIGHT_M]


def fig_to_array(fig):
    """Turn a matplotlib figure into an image array for the GIF."""
    fig.canvas.draw()
    return np.asarray(fig.canvas.buffer_rgba())[:, :, :3].copy()


def make_gif(result, path):
    house = result["house"]
    fig, (ax_l, ax_r) = plt.subplots(1, 2, figsize=(10, 4.6), dpi=72)

    # Left: the real house, faint laser beams, the robot and its trail.
    ax_l.imshow(~house, cmap="gray", origin="lower", extent=EXTENT, vmin=0, vmax=1)
    beams = LineCollection([], colors="red", linewidths=0.5, alpha=0.25)
    ax_l.add_collection(beams)
    trail_l, = ax_l.plot([], [], color="orange", lw=1.5)
    robot_l = Circle((0, 0), ROBOT_RADIUS, color="blue")
    ax_l.add_patch(robot_l)
    ax_l.set_title("Real house")

    # Right: the robot's map so far, the robot and its trail.
    map_img = ax_r.imshow(np.full(house.shape, 0.5), cmap="gray", origin="lower",
                          extent=EXTENT, vmin=0, vmax=1)
    trail_r, = ax_r.plot([], [], color="orange", lw=1.5)
    robot_r = Circle((0, 0), ROBOT_RADIUS, color="blue")
    ax_r.add_patch(robot_r)
    ax_r.set_title("Robot's map")

    for ax in (ax_l, ax_r):
        ax.set_xlim(0, WIDTH_M)
        ax.set_ylim(0, HEIGHT_M)
        ax.set_xticks([])
        ax.set_yticks([])
    caption = fig.text(0.5, 0.03, "", ha="center", fontsize=12)
    fig.tight_layout(rect=[0, 0.07, 1, 1])

    frames = []
    for f in result["frames"]:
        x, y = f["x"], f["y"]
        ends = np.column_stack([x + f["distances"] * np.cos(f["angles"]),
                                y + f["distances"] * np.sin(f["angles"])])
        beams.set_segments([[(x, y), tuple(e)] for e in ends])
        tx, ty = zip(*f["trail"])
        trail_l.set_data(tx, ty)
        trail_r.set_data(tx, ty)
        robot_l.center = (x, y)
        robot_r.center = (x, y)
        map_img.set_data(map_image(f["map"]))
        caption.set_text(f"Coverage: {f['coverage']:.1f}%    Driven: {f['driven']:.1f} m")
        frames.append(fig_to_array(fig))
    plt.close(fig)
    imageio.mimsave(path, frames, duration=125, loop=0)  # 125 ms per frame = 8 fps


def make_coverage_plot(result, path):
    curve = np.array(result["curve"])
    fig, ax = plt.subplots(figsize=(7, 4), dpi=100)
    ax.plot(curve[:, 0], curve[:, 1], color="tab:blue", lw=2)
    ax.set_xlabel("Metres driven")
    ax.set_ylabel("Coverage (%)")
    ax.set_ylim(0, 105)
    ax.set_title("Coverage vs distance driven")
    ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)


def write_stats(result, path):
    lines = [
        "# Exploration stats (noise 2 cm, seed 42)",
        "",
        "| Measure | Value |",
        "|---|---|",
        f"| Scans | {result['scans']} |",
        f"| Metres driven | {result['driven']:.1f} |",
        f"| Coverage | {result['coverage']:.1f}% |",
        f"| Accuracy | {result['accuracy']:.1f}% |",
        f"| Walls found | {result['walls_found']:.1f}% |",
        f"| Bumps | {result['bumps']} |",
        "",
    ]
    with open(path, "w") as f:
        f.write("\n".join(lines))
    print("\n".join(lines))


def main():
    result = explore(noise_sigma=0.02, seed=42, record=True)
    make_gif(result, "results/exploration.gif")
    make_coverage_plot(result, "results/coverage.png")
    write_stats(result, "results/exploration_stats.md")
    print("Saved results/exploration.gif, results/coverage.png, results/exploration_stats.md")


if __name__ == "__main__":
    main()
