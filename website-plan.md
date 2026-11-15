# LiDAR Mapper — Website Project Spec

## What this project is
A website where you load ANY floor plan, click where a robot should start, and watch it explore on its own with a simulated spinning LiDAR (a laser distance sensor), building its own map live. When it's done, you click a goal and it plans the shortest safe path there.
It replicates three classic robotics ideas: occupancy grid maps (Moravec & Elfes, 1985), frontier-based exploration (Yamauchi, 1997) and A* path planning (Hart, Nilsson & Raphael, 1968).
The owner is a beginner. Deadline: finished in one day. Next week, the same ideas will be used with a real sensor.

## Rules for Claude Code
- Plain HTML, CSS and JavaScript (ES modules). No frameworks, no libraries, no build step, no server. It must run on GitHub Pages as static files.
- Keep code simple and readable. Add short comments a grade 10 student can follow.
- Put all the robot logic (floor plans, LiDAR, mapping, exploration, A*) in modules that use NO browser APIs, so Node can test them. Only `js/app.js` touches the page.
- After every step, RUN `node tests/test.mjs` and check the output matches the "Expected" notes below. If it doesn't, find out why and fix it before moving on.
- Use a seeded random number generator (for example mulberry32, seed 42) and Box-Muller for Gaussian noise, so results are repeatable.
- Do not add features that are not in this spec.
- The animation must stay smooth on a normal laptop. Keep the grid at most 400 cells on its long side.
- The user works in Claude Code on the web, so this project lives in a GitHub repo and the user can't type into a terminal. If you can load the page in a headless browser, check that it has no console errors. Push your work when done.

## Files
```
index.html        # the page
style.css         # the look
js/rng.js         # seeded random numbers
js/world.js       # the built-in example house + turning an uploaded image into a grid
js/lidar.js       # the fake LiDAR: casts laser rays and adds noise
js/mapping.js     # builds the robot's map from (angle, distance) readings
js/explore.js     # frontier exploration, A* path planning, and the measurements
js/app.js         # page logic: controls, drawing, animation
tests/test.mjs    # Node tests for the robot logic
results/test_results.md   # written by the tests
README.md
```

## Floor plans (js/world.js)
A floor plan is a grid of cells: solid (wall or furniture) or open floor. Cells are 5 cm by default.

Built-in "Example house":
- 10 m wide (x) by 8 m tall (y): 200 x 160 cells. Origin at the bottom-left.
- Each rectangle below is solid, given as (x0, y0) to (x1, y1) in metres. Fill cells from round(y0/0.05) to round(y1/0.05) and round(x0/0.05) to round(x1/0.05), end exclusive.
- Outer walls: (0,0)-(10,0.1), (0,7.9)-(10,8), (0,0)-(0.1,8), (9.9,0)-(10,8)
- Middle wall with 2 doorways: (5,0)-(5.1,1.5), (5,2.5)-(5.1,5.5), (5,6.5)-(5.1,8)
- Left-side wall with 1 doorway: (0,4)-(2,4.1), (3,4)-(5,4.1)
- Right-side wall with 1 doorway: (5,3)-(7,3.1), (8,3)-(10,3.1)
- Furniture: couch (1,1)-(2.2,1.8), table (2.5,5.5)-(3.5,7), bed (7.5,5)-(9,6)
- Room labels: Living room (bottom-left), Kitchen (top-left), Office (bottom-right), Bedroom (top-right).
- Default start: (1.0, 3.0).

Uploaded floor plan (any PNG or JPG):
- A pure function `imageToGrid(rgba, imgWidth, imgHeight, realWidthMetres)` turns pixels into a grid, so it can be tested in Node.
- Dark pixels (brightness below 128) are walls. Transparent pixels count as white.
- Cell size is 5 cm. If that would make the long side more than 400 cells, use a bigger cell size so it fits, and show the cell size on the page.
- When shrinking the image, a cell is solid if ANY pixel inside it is dark, so thin lines don't disappear.
- Add a 1-cell solid border around the whole grid, so the robot can't leave the picture.
- The user sets the plan's real width in metres (default 10).
- Show this tip next to the upload button: "Works best with simple black-on-white plans. Door-swing arcs, text and furniture symbols become walls, so erase them and leave doorways as open gaps."

## The robot and the LiDAR (js/lidar.js)
- Robot: a circle with radius 0.15 m. It always knows its exact position (see Limitations).
- LiDAR: 180 beams, one every 2 degrees, all the way around. Max range set by a slider (default 4.0 m, the same as the real VL53L1X sensor planned for next week).
- Ray casting: step along each beam in half-cell steps until it reaches a solid cell (a "hit") or the max range (a "miss").
- Noise: Gaussian noise (sigma from a slider, default 2 cm) added to hit distances only. Clip to 0.01 m up to the max range.

## The robot's map (js/mapping.js)
- An occupancy grid the same size as the floor plan, stored as log-odds, all starting at 0 (unknown).
- Main function: `updateMap(logodds, x, y, anglesRad, distancesM, hits)`. It must work with ANY list of readings.
- For each beam:
  - Cells along the beam are free: add -0.4. For a hit, stop ONE CELL SHORT of the hit point, so noise can't erase walls. For a miss, mark free all the way to max range.
  - For a hit, the cell at the hit point is occupied: add +0.85.
  - Clamp every value to -5..+5.
- Reading the map: log-odds below -0.3 = free, above +0.3 = occupied, otherwise unknown.

## Exploration and planning (js/explore.js)
Frontier-based exploration:
- Safety zone: grow the map's occupied cells outward by 0.18 m (the robot radius plus a margin). The robot can't plan through these cells.
- Escape rule: within 30 cm of the robot, use the map's occupied cells WITHOUT the safety zone. Otherwise the robot gets trapped when it discovers an obstacle right next to it.
- A frontier is a free cell outside the safety zone that touches an unknown cell (up, down, left or right).
- Each turn: scan, update the map, find the nearest frontier with a breadth-first search (8 directions) through free, safe cells, then move up to 0.5 m along that path, then scan again.
- Ignore rule: if the robot reaches its frontier target during a move, ignore a 7 x 7-cell area around that target from then on. Without this, the robot chases unknown cells inside walls forever.
- Bump check: before each cell of a move, if that cell is solid in the REAL floor plan, stop the move and count a "bump".
- Stop when no frontier can be reached ("Done: map complete"), or after 400 scans ("Stopped: scan limit").
- Write exploration as a step-by-step object (for example `explorer.step()` does one turn) so the page can animate it and the tests can run it to the end.
- A* path planning: 8-direction moves, straight-line distance as the heuristic. Unknown cells and the safety zone count as blocked.

Measurements:
- Coverage: the % of reachable floor that the robot's map marks as free. "Reachable floor" = flood fill from the start through the REAL floor plan, with walls grown by 0.18 m.
- Accuracy: of all cells the map marks as free or occupied, the % that match the real floor plan.
- Walls found: of the real solid cells that touch open floor, the % the map marks as occupied.
- Also count metres driven, scans and bumps.

## The page (index.html, style.css, js/app.js)
- Header: "LiDAR Mapper" and one line: "Load any floor plan and watch a robot map it with a laser."
- Controls:
  - Floor plan: "Example house" or "Upload your own" (file picker), the real width in metres, and the tip text.
  - Sliders: sensor noise (0–10 cm, default 2), LiDAR range (1–8 m, default 4), speed (1–30 scans per second, default 8).
  - Buttons: Start, Pause, Reset, and "Download example floor plan" (saves the example house as a black-on-white PNG, so people can see the format and test uploading).
- Click on the real floor plan to set the start point. If the spot is a wall or too close to one, show a short message and don't move the start.
- Two canvases side by side (stacked on narrow screens):
  - "Real floor plan": the plan, the robot, its trail, and faint laser beams from the latest scan.
  - "Robot's map": gray = unknown, white = free, black = occupied, plus the robot and its trail.
- Live stats: scans, metres driven, coverage %, accuracy %, walls found %, bumps, and status.
- A small live chart of coverage % vs metres driven, drawn on a canvas.
- When exploring is done, show "Click the robot's map to plan a path". On a click, draw the A* path and a dashed straight line, and show "Path: X m (straight line: Y m)". If there's no path, say so.
- A short "How it works" section at the bottom: 5 bullets (LiDAR, map, frontiers, A*, noise), written for a grade 10 reader.

## Step 1 — Robot logic and tests
Build rng.js, world.js, lidar.js, mapping.js and explore.js, plus tests/test.mjs. The tests print results and write `results/test_results.md`:
1. Example house, noise 2 cm, seed 42, start (1.0, 3.0), run to the end.
   - Expected (roughly): coverage about 100% (at least 95%) after about 30–45 m of driving and 70–110 scans. Accuracy about 98% (at least 96%). 0 bumps.
2. Noise test: the same run with noise 0, 2, 5 and 10 cm, as a table of noise, scans, metres, coverage, accuracy, walls found and bumps.
   - Expected (roughly): at 0 cm, accuracy about 100%, walls found about 99%, 0 bumps. At 10 cm, accuracy about 97–98%, walls found roughly 75%, about 50 m driven, several bumps. Coverage stays near 100% at every noise level.
3. A* on the final map from test 1, from (1.0, 3.0) to the Bedroom at (9.0, 7.2).
   - Expected: about 9.5–10.5 m, vs 9.0 m in a straight line, staying at least 18 cm from walls.
4. imageToGrid with a made-up pixel array: a 1-pixel-wide dark line in a large image must still be solid after shrinking, and the border must be solid.

## Step 2 — The page with the example house
- Build the page so the example house can be explored with the live animation, stats and chart.
- Expected: the robot's map fills in room by room, and the final stats are close to Step 1's test results.

## Step 3 — Any floor plan
- Add uploading, the real-width input, clicking to set the start, and "Download example floor plan".
- Expected: downloading the example floor plan and uploading it again gives a grid that matches the built-in example (add this check to the tests by comparing imageToGrid on the example's own pixels).

## Step 4 — Path planning on click
- Add click-to-plan on the robot's map after exploration.
- Expected: paths go through doorways and never through unknown areas.

## Step 5 — README.md
Sections: title, one-line summary, a link placeholder for the live site (`TODO: add GitHub Pages link`), a screenshot placeholder (`TODO: add screenshot`), how it works (5 bullets), test results (copy the table from results/test_results.md), a "Findings" section left as `TODO: write in your own words`, limitations, next week (hardware), how to run, and credits.

How to run: "Open the GitHub Pages link. To run it from your own computer, use a local web server, because browsers block JavaScript modules opened straight from a file."

Limitations to list honestly:
- The robot always knows exactly where it is. Real SLAM also has to work out the robot's position from its sensors (localization), which is the hardest part. This project does the mapping half.
- 2D only. Real drones and self-driving cars map in 3D.
- Perfect movement: no wheel slip or drift.
- Simple sensor model: Gaussian noise only, with no reflections, glass, sunlight or dark surfaces.
- Nothing in the floor plan moves.
- Uploaded plans are read pixel by pixel, so door arcs, text and symbols become walls.

Next week (hardware): a VL53L1X laser distance sensor on a servo, connected to an ESP32 or Arduino, sends (angle, distance) readings to a laptop over USB, and the same `updateMap` logic turns them into a real map of a room. The servo sweeps 180 degrees instead of 360.

Credits: Moravec & Elfes (1985), occupancy grid maps; Yamauchi (1997), frontier-based exploration; Hart, Nilsson & Raphael (1968), the A* algorithm.
