// Node tests for the robot logic. Run with:  node tests/test.mjs
// Prints the results and writes them to results/test_results.md
import { writeFileSync, mkdirSync } from "node:fs";
import { makeExampleHouse, imageToGrid, worldToRgba, toCell, cellCenter } from "../js/world.js";
import { Explorer, planPath, pathLength } from "../js/explore.js";
import { isUnknown } from "../js/mapping.js";

const lines = [];      // the lines of the results file
let failures = 0;

function log(text = "") {
  console.log(text);
  lines.push(text);
}

// Print a check and remember if it failed.
function check(name, ok) {
  log(`- ${ok ? "PASS" : "FAIL"}: ${name}`);
  if (!ok) failures++;
}

// Run a full exploration of the example house.
function runExample(noise) {
  const world = makeExampleHouse();
  const ex = new Explorer(world, world.start, { noise, maxRange: 4.0, seed: 42 });
  while (!ex.done) ex.step();
  return ex;
}

const f1 = (v) => v.toFixed(1);

log("# Test results");
log();

// ---------- Test 1: full exploration, noise 2 cm ----------
log("## Test 1: Example house, noise 2 cm, seed 42");
log();
const t0 = Date.now();
const ex = runExample(0.02);
const secs = (Date.now() - t0) / 1000;
log("| Measure | Value |");
log("|---|---|");
log(`| Scans | ${ex.scans} |`);
log(`| Metres driven | ${f1(ex.driven)} |`);
log(`| Coverage | ${f1(ex.stats.coverage)}% |`);
log(`| Accuracy | ${f1(ex.stats.accuracy)}% |`);
log(`| Walls found | ${f1(ex.stats.wallsFound)}% |`);
log(`| Bumps | ${ex.bumps} |`);
log(`| Status | ${ex.status} |`);
log();
console.log(`(took ${secs.toFixed(1)} s)`);
check("coverage at least 95%", ex.stats.coverage >= 95);
check("accuracy at least 96%", ex.stats.accuracy >= 96);
check("0 bumps", ex.bumps === 0);
check("finished with 'Done: map complete'", ex.status === "Done: map complete");
log();

// ---------- Test 2: noise test ----------
log("## Test 2: Noise test (seed 42)");
log();
log("| Noise (cm) | Scans | Metres | Coverage | Accuracy | Walls found | Bumps |");
log("|---|---|---|---|---|---|---|");
for (const cm of [0, 2, 5, 10]) {
  const r = cm === 2 ? ex : runExample(cm / 100);
  log(`| ${cm} | ${r.scans} | ${f1(r.driven)} | ${f1(r.stats.coverage)}% | ` +
      `${f1(r.stats.accuracy)}% | ${f1(r.stats.wallsFound)}% | ${r.bumps} |`);
}
log();

// ---------- Test 3: A* on the final map from test 1 ----------
log("## Test 3: A* from (1.0, 3.0) to the Bedroom (9.0, 7.2)");
log();
{
  const world = ex.world;
  const start = { x: 1.0, y: 3.0 }, goal = { x: 9.0, y: 7.2 };
  const path = planPath(ex.map, toCell(world, start.x, start.y), toCell(world, goal.x, goal.y));
  check("a path was found", path !== null);
  if (path) {
    const length = pathLength(path, world.cell);
    const straight = Math.hypot(goal.x - start.x, goal.y - start.y);
    // Closest distance from any path cell to any real solid cell (centre to centre).
    let closest = Infinity;
    for (const p of path) {
      for (let i = 0; i < world.solid.length; i++) {
        if (!world.solid[i]) continue;
        const d = Math.hypot(p.r - Math.floor(i / world.nx), p.c - (i % world.nx));
        if (d < closest) closest = d;
      }
    }
    closest *= world.cell;
    const throughUnknown = path.some((p) => isUnknown(ex.map.data[p.r * world.nx + p.c]));
    log(`Path: ${length.toFixed(2)} m (straight line: ${straight.toFixed(2)} m). ` +
        `Closest to a real wall: ${(closest * 100).toFixed(0)} cm.`);
    log();
    check("path is 9.5-10.5 m", length >= 9.5 && length <= 10.5);
    check("path stays at least 18 cm from walls", closest >= 0.18 - 1e-9);
    check("path never goes through unknown cells", !throughUnknown);
    // Which doorways does it use? A doorway is crossed when the path passes a wall line.
    const pts = path.map((p) => cellCenter(world, p.r, p.c));
    const crossesWallLines = pts.every((p) => !world.solid[toCell(world, p.x, p.y).r * world.nx + toCell(world, p.x, p.y).c]);
    check("path never enters a solid cell", crossesWallLines);
  }
}
log();

// ---------- Test 4: imageToGrid ----------
log("## Test 4: imageToGrid");
log();
{
  // A made-up white image, 2000 x 1600 pixels, with a 1-pixel-wide dark vertical line.
  const w = 2000, h = 1600, lineX = 1001;
  const rgba = new Uint8ClampedArray(w * h * 4).fill(255);
  for (let y = 0; y < h; y++) {
    const i = (y * w + lineX) * 4;
    rgba[i] = rgba[i + 1] = rgba[i + 2] = 0;
  }
  // 50 m wide: 5 cm cells would be 1000 cells, so the cells must grow to fit 400.
  const g = imageToGrid(rgba, w, h, 50);
  log(`Image ${w} x ${h} px, 50 m wide -> grid ${g.nx} x ${g.ny} cells, cell size ${(g.cell * 100).toFixed(1)} cm`);
  log();
  check("long side is at most 400 cells", Math.max(g.nx, g.ny) <= 400);
  const lineCol = Math.floor(lineX / (w / g.nx));
  let lineSolid = true;
  for (let r = 0; r < g.ny; r++) if (!g.solid[r * g.nx + lineCol]) lineSolid = false;
  check("the 1-pixel line is still solid after shrinking", lineSolid);
  let border = true;
  for (let c = 0; c < g.nx; c++) if (!g.solid[c] || !g.solid[(g.ny - 1) * g.nx + c]) border = false;
  for (let r = 0; r < g.ny; r++) if (!g.solid[r * g.nx] || !g.solid[r * g.nx + g.nx - 1]) border = false;
  check("the 1-cell border is solid", border);
  check("open white areas stay open", !g.solid[Math.floor(g.ny / 2) * g.nx + 10]);
  // Transparent pixels count as white.
  const clear = new Uint8ClampedArray(20 * 20 * 4); // all zeros = black but fully transparent
  const gc = imageToGrid(clear, 20, 20, 1);
  check("transparent pixels count as white", !gc.solid[10 * gc.nx + 10]);
}
log();

// ---------- Test 5: download the example floor plan, then upload it again ----------
log("## Test 5: Example floor plan PNG -> imageToGrid matches the built-in example");
log();
{
  const house = makeExampleHouse();
  // The same pixels that "Download example floor plan" saves (4 pixels per cell).
  const png = worldToRgba(house, 4);
  const g = imageToGrid(png.rgba, png.width, png.height, 10);
  let different = 0;
  for (let i = 0; i < house.solid.length; i++) if (g.solid[i] !== house.solid[i]) different++;
  log(`Image ${png.width} x ${png.height} px -> grid ${g.nx} x ${g.ny} cells, ${different} cells different`);
  log();
  check("same grid size and cell size", g.nx === house.nx && g.ny === house.ny && Math.abs(g.cell - house.cell) < 1e-12);
  check("every cell matches", different === 0);
}
log();

log(failures === 0 ? "All checks passed." : `${failures} check(s) failed.`);
mkdirSync(new URL("../results/", import.meta.url), { recursive: true });
writeFileSync(new URL("../results/test_results.md", import.meta.url), lines.join("\n") + "\n");
console.log("Saved results/test_results.md");
process.exitCode = failures === 0 ? 0 : 1;
