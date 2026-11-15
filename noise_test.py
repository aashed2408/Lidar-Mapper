"""Step 3: how does sensor noise change the map?

Runs exploration with noise 0, 2, 5 and 10 cm (same seed) and makes:
  results/noise_test.png - the 4 final maps side by side
  results/noise_test.md  - a table of the numbers
"""
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from world import WIDTH_M, HEIGHT_M
from mapping import map_image
from explore import explore

NOISE_CM = [0, 2, 5, 10]


def main():
    results = []
    for cm in NOISE_CM:
        r = explore(noise_sigma=cm / 100.0, seed=42)
        results.append((cm, r))
        print(f"noise {cm} cm: scans {r['scans']}, {r['driven']:.1f} m, coverage {r['coverage']:.1f}%, "
              f"accuracy {r['accuracy']:.1f}%, walls found {r['walls_found']:.1f}%, bumps {r['bumps']}")

    # Picture: the 4 final maps side by side.
    fig, axes = plt.subplots(1, 4, figsize=(18, 4.2), dpi=100)
    for ax, (cm, r) in zip(axes, results):
        ax.imshow(map_image(r["map"]), cmap="gray", origin="lower",
                  extent=[0, WIDTH_M, 0, HEIGHT_M], vmin=0, vmax=1)
        ax.set_title(f"Noise {cm} cm\naccuracy {r['accuracy']:.1f}%, walls found {r['walls_found']:.1f}%")
        ax.set_xticks([])
        ax.set_yticks([])
    fig.tight_layout()
    fig.savefig("results/noise_test.png")
    plt.close(fig)

    # Table.
    lines = [
        "# Noise test (seed 42)",
        "",
        "| Noise (cm) | Scans | Metres | Coverage | Accuracy | Walls found | Bumps |",
        "|---|---|---|---|---|---|---|",
    ]
    for cm, r in results:
        lines.append(f"| {cm} | {r['scans']} | {r['driven']:.1f} | {r['coverage']:.1f}% | "
                     f"{r['accuracy']:.1f}% | {r['walls_found']:.1f}% | {r['bumps']} |")
    with open("results/noise_test.md", "w") as f:
        f.write("\n".join(lines) + "\n")
    print("Saved results/noise_test.png and results/noise_test.md")


if __name__ == "__main__":
    main()
