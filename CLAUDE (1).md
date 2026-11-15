# LiDAR Mapping Simulator — Project Spec

## What this project is
A Python simulation of a robot with a spinning LiDAR (a laser distance sensor) that explores a house on its own, builds a map of it, and then plans a path through it.
It replicates three classic robotics ideas: occupancy grid mapping, frontier-based exploration, and A* path planning.
The owner is a beginner. Deadline: finished in one day. Next week, the same mapping code will be used with a real sensor.

## Rules for Claude Code
- Keep code simple and readable. Add short comments a grade 10 student can follow.
- Dependencies: numpy, matplotlib, imageio ONLY.
- Use a virtual environment (`.venv`) and a `requirements.txt`.
- After every step, RUN the scripts and check the output matches the "Expected" notes below. If it doesn't, find out why and fix it before moving on.
- Use a fixed random seed (42) so results are repeatable.
- Do not add features that are not in this spec.
- All outputs go in `results/`.
- Keep each script's run time under about 2 minutes.
- The user works in Claude Code on the web, so this project lives in a GitHub repo and the user can't type into a terminal. Add a `.gitignore` for `.venv/` and `__pycache__/`. Keep every GIF under 5 MB so it shows on GitHub. Push your work when done.

## Files
```
world.py         # the house floor plan (the "real world")
lidar.py         # the fake LiDAR: casts laser rays and adds noise
mapping.py       # builds the robot's map from (angle, distance) readings. Reused next week with the real sensor.
explore.py       # frontier exploration and A* path planning
radar_demo.py    # Step 1
explore_demo.py  # Step 2
noise_test.py    # Step 3
path_test.py     # Step 4
README.md        # Step 5
results/         # all images, GIFs and tables
```

## The house (world.py)
- 10 m wide (x) by 8 m tall (y). Grid cells are 5 cm, so the grid is 200 x 160 cells. Origin at the bottom-left.
- Each rectangle below is solid (a wall or furniture), given as (x0, y0) to (x1, y1) in metres. Fill cells from round(y0/0.05) to round(y1/0.05) and round(x0/0.05) to round(x1/0.05), end exclusive.
- Outer walls (10 cm thick): (0,0)-(10,0.1), (0,7.9)-(10,8), (0,0)-(0.1,8), (9.9,0)-(10,8)
- Middle wall with 2 doorways: (5,0)-(5.1,1.5), (5,2.5)-(5.1,5.5), (5,6.5)-(5.1,8)
- Left-side wall with 1 doorway: (0,4)-(2,4.1), (3,4)-(5,4.1)
- Right-side wall with 1 doorway: (5,3)-(7,3.1), (8,3)-(10,3.1)
- Furniture: couch (1,1)-(2.2,1.8), table (2.5,5.5)-(3.5,7), bed (7.5,5)-(9,6)
- Room names for labels: Living room (bottom-left, where the robot starts), Kitchen (top-left), Office (bottom-right), Bedroom (top-right).

## The robot and the LiDAR (lidar.py)
- Robot: a circle with radius 0.15 m. Starts at (1.0, 3.0) in the Living room.
- The robot always knows its exact position (see Limitations).
- LiDAR: 180 laser beams, one every 2 degrees, all the way around (360 degrees). Max range 4.0 m (the same range as the real VL53L1X sensor planned for next week).
- Ray casting: step along each beam in 2.5 cm steps until it reaches a solid cell (a "hit") or the max range (a "miss").
- Noise: add Gaussian noise (default sigma = 0.02 m) to hit distances only. Clip distances to 0.01–4.0 m.

## The robot's map (mapping.py)
- An occupancy grid the same size as the house (200 x 160), stored as log-odds, all starting at 0 (unknown).
- Main function: `update_map(logodds, x, y, angles_rad, distances_m, hits)`. It must work with ANY list of readings, so next week's real sensor data can use it unchanged.
- For each beam:
  - Cells along the beam are free: add -0.4. For a hit, stop ONE CELL SHORT of the hit point, so noise can't erase walls. For a miss, mark free all the way to max range.
  - For a hit, the cell at the hit point is occupied: add +0.85.
  - Clamp every value to -5..+5.
- Reading the map: log-odds below -0.3 = free, above +0.3 = occupied, otherwise unknown.

## Exploration and planning (explore.py)
Frontier-based exploration (Yamauchi, 1997):
- Safety zone: grow the map's occupied cells outward by 0.18 m (the robot radius plus a margin). The robot can't plan through these cells.
- Escape rule: within 30 cm (6 cells) of the robot, use the map's occupied cells WITHOUT the safety zone. Otherwise the robot gets trapped when it discovers an obstacle right next to it.
- A frontier is a free cell outside the safety zone that touches an unknown cell (up, down, left or right).
- Each turn: scan, update the map, find the nearest frontier with a breadth-first search (8 directions) through free, safe cells, then move up to 0.5 m (10 cells) along that path, then scan again.
- Ignore rule: if the robot reaches its frontier target during a move, ignore a 7 x 7-cell area around that target from then on. Without this, the robot chases unknown cells inside walls forever.
- Bump check: before each cell of a move, if that cell is solid in the REAL house, stop the move and count a "bump".
- Stop when no frontier can be reached, or after 400 scans.
- A* path planning (Hart, Nilsson & Raphael, 1968): 8-direction moves, straight-line distance as the heuristic. Unknown cells and the safety zone count as blocked.

## Measurements
- Coverage: the % of reachable floor in the real house that the robot's map marks as free. "Reachable floor" = flood fill from the start position through the real house, with walls grown by 0.18 m.
- Accuracy: of all cells the map marks as free or occupied, the % that match the real house.
- Walls found: of the real wall cells that touch open floor, the % the map marks as occupied.
- Distance driven (metres), number of scans, and number of bumps.

## Step 1 — One scan, radar style (radar_demo.py)
- The robot does one 360-degree scan from the start position (noise 2 cm).
- `results/radar_scan.gif`: a radar-style view on a dark background. A beam line sweeps around the robot over about 3 seconds, and each hit appears as a dot when the beam passes it.
- `results/one_scan_map.png`: two panels. Left: the real house with the robot and its 180 beams. Right: the robot's map after one scan (gray = unknown, white = free, black = occupied).
- Expected: the Living room's walls and the couch show up as black outlines. Doorways show as gaps. The other rooms stay mostly gray, because the walls block the laser.

## Step 2 — Exploration (explore_demo.py)
- Run exploration with noise 2 cm and seed 42.
- `results/exploration.gif`: two panels side by side.
  - Left, titled "Real house": the floor plan, the robot, its trail, and faint laser beams.
  - Right, titled "Robot's map": the map so far, the robot, and its trail.
  - One frame per scan, 8 fps. Add a caption with coverage % and metres driven.
- `results/coverage.png`: coverage % vs metres driven.
- `results/exploration_stats.md`: scans, metres driven, coverage, accuracy, walls found, bumps.
- Expected (roughly): coverage reaches about 100% (at least 95%) after about 30–45 m of driving and 70–110 scans. Accuracy about 98% (at least 96%). 0 bumps. Coverage is roughly 40–50% after 5 m and over 90% by 20 m.

## Step 3 — Noise test (noise_test.py)
- Run exploration with noise 0, 2, 5 and 10 cm (same seed).
- `results/noise_test.png`: the 4 final maps side by side, each titled with its noise, accuracy and walls found.
- `results/noise_test.md`: a table of noise, scans, metres, coverage, accuracy, walls found and bumps.
- Expected (roughly): at 0 cm, walls are crisp, accuracy is about 100%, walls found is about 99%, and there are 0 bumps. As noise grows, walls look thicker and fuzzier, and the robot drives farther and bumps more. At 10 cm: accuracy is about 97–98%, walls found drops to roughly 75%, the robot drives about 50 m, and there are several bumps. Coverage stays near 100% at every noise level.

## Step 4 — Path planning (path_test.py)
- Use the final map from Step 2. Plan an A* path from the start (1.0, 3.0) to the Bedroom at (9.0, 7.2).
- `results/path_plan.png`: the robot's map with the A* path drawn on it, a dashed straight line from start to goal, and room labels. The title shows the path length vs the straight-line distance.
- Expected: the path goes through doorways and stays at least 18 cm from walls. It is about 9.5–10.5 m long, vs 9.0 m in a straight line.

## Step 5 — README.md
Sections: title, one-line summary, the exploration GIF, how it works (5 short bullets: LiDAR, map, frontiers, A*, noise), results (all images + a "Findings" section left as `TODO: write in your own words`), limitations, next week (hardware), how to run, credits.

Limitations to list honestly:
- The robot always knows exactly where it is. Real SLAM also has to work out the robot's position from its sensors (localization), which is the hardest part. This project does the mapping half.
- 2D only. Real drones and self-driving cars map in 3D.
- Perfect movement: no wheel slip or drift.
- Simple sensor model: Gaussian noise only, with no reflections, glass, sunlight or dark surfaces.
- Nothing in the house moves.

Next week (hardware): a VL53L1X laser distance sensor on a servo, connected to an ESP32 or Arduino, sends (angle, distance) readings to a laptop over USB. `update_map` in mapping.py turns them into a real map of a room. The servo sweeps 180 degrees instead of 360.

Credits: Moravec & Elfes (1985), occupancy grid maps; Yamauchi (1997), frontier-based exploration; Hart, Nilsson & Raphael (1968), the A* algorithm.
