// Page logic: controls, drawing and animation. This is the only file that touches the page.
import { makeExampleHouse, imageToGrid, worldToRgba, toCell } from "./world.js";
import { ROBOT_RADIUS } from "./lidar.js";
import { isFree, isOccupied } from "./mapping.js";
import { Explorer, inflate, SAFETY_M, planPath, pathLength } from "./explore.js";

const $ = (id) => document.getElementById(id);
const realCanvas = $("realCanvas");
const mapCanvas = $("mapCanvas");
const chart = $("chart");
const CANVAS_WIDTH = 600; // drawing size of each canvas in pixels (the page scales it to fit)

// ---------- The state of the app ----------
let world = makeExampleHouse(); // the real floor plan
let start = world.start;        // where the robot starts (metres)
let explorer = null;            // the exploration, one turn at a time
let running = false;            // is the animation playing?
let planImage = null;           // a pre-drawn picture of the real plan
let tooClose = null;            // real walls grown by 0.18 m: the robot can't start here
let uploaded = null;            // the pixels of the uploaded picture, if any
let plan = null;                // the A* path the user asked for: { path, from, goal }

// ---------- Helpers ----------

// Turn metres into canvas pixels. y is flipped because canvases count down from the top.
function toPx(canvas, x, y) {
  const s = canvas.width / (world.nx * world.cell);
  return [x * s, canvas.height - y * s];
}

// Draw a grid of cells onto a canvas. colourOf(i) returns a grey level 0..255 for cell i.
function gridToCanvas(colourOf) {
  const off = document.createElement("canvas");
  off.width = world.nx;
  off.height = world.ny;
  const ctx = off.getContext("2d");
  const img = ctx.createImageData(world.nx, world.ny);
  for (let r = 0; r < world.ny; r++) {
    for (let c = 0; c < world.nx; c++) {
      const v = colourOf(r * world.nx + c);
      const p = ((world.ny - 1 - r) * world.nx + c) * 4; // image rows go top-down
      img.data[p] = img.data[p + 1] = img.data[p + 2] = v;
      img.data[p + 3] = 255;
    }
  }
  ctx.putImageData(img, 0, 0);
  return off;
}

// Set the canvas sizes to match the plan's shape.
function sizeCanvases() {
  for (const cv of [realCanvas, mapCanvas]) {
    cv.width = CANVAS_WIDTH;
    cv.height = Math.round((CANVAS_WIDTH * world.ny) / world.nx);
  }
  planImage = gridToCanvas((i) => (world.solid[i] ? 0 : 255));
  $("cellSize").textContent = (world.cell * 100).toFixed(1);
}

// Switch to a new floor plan.
function loadWorld(newWorld) {
  world = newWorld;
  start = world.start;
  tooClose = inflate(world.solid, world.nx, world.ny, world.cell, SAFETY_M);
  sizeCanvases();
  reset();
  if (!start) $("message").textContent = "Click the real floor plan to set the start point";
}

// ---------- Drawing ----------

function drawRobotAndTrail(ctx, canvas) {
  if (!start) return; // no start point chosen yet
  const trail = explorer ? explorer.trail : [start];
  const pos = trail[trail.length - 1];
  // The trail: everywhere the robot has been.
  ctx.strokeStyle = "orange";
  ctx.lineWidth = 2;
  ctx.beginPath();
  trail.forEach((p, k) => {
    const [px, py] = toPx(canvas, p.x, p.y);
    if (k === 0) ctx.moveTo(px, py); else ctx.lineTo(px, py);
  });
  ctx.stroke();
  // The robot: a blue circle with its real size.
  const [rx, ry] = toPx(canvas, pos.x, pos.y);
  const radius = Math.max(3, (ROBOT_RADIUS / (world.nx * world.cell)) * canvas.width);
  ctx.fillStyle = "#2f6fde";
  ctx.beginPath();
  ctx.arc(rx, ry, radius, 0, 2 * Math.PI);
  ctx.fill();
}

function drawReal() {
  const ctx = realCanvas.getContext("2d");
  ctx.imageSmoothingEnabled = false;
  ctx.drawImage(planImage, 0, 0, realCanvas.width, realCanvas.height);

  // Room labels (only the example house has them).
  ctx.fillStyle = "green";
  ctx.font = "14px system-ui, sans-serif";
  ctx.textAlign = "center";
  for (const room of world.rooms) {
    const [px, py] = toPx(realCanvas, room.x, room.y);
    ctx.fillText(room.name, px, py);
  }

  // Faint laser beams from the latest scan.
  if (explorer && explorer.lastScan) {
    const s = explorer.lastScan;
    const last = explorer.trail[explorer.trail.length - 1];
    // The scan was taken before the move, so draw it from where the robot scanned.
    const from = explorer.scanFrom || last;
    ctx.strokeStyle = "rgba(220, 0, 0, 0.25)";
    ctx.lineWidth = 1;
    ctx.beginPath();
    for (let b = 0; b < s.angles.length; b++) {
      const [x0, y0] = toPx(realCanvas, from.x, from.y);
      const [x1, y1] = toPx(realCanvas,
        from.x + s.distances[b] * Math.cos(s.angles[b]),
        from.y + s.distances[b] * Math.sin(s.angles[b]));
      ctx.moveTo(x0, y0);
      ctx.lineTo(x1, y1);
    }
    ctx.stroke();
  }
  drawRobotAndTrail(ctx, realCanvas);
}

function drawMap() {
  const ctx = mapCanvas.getContext("2d");
  ctx.imageSmoothingEnabled = false;
  // gray = unknown, white = free, black = occupied
  const data = explorer ? explorer.map.data : null;
  const img = gridToCanvas((i) => {
    if (!data) return 128;
    if (isFree(data[i])) return 255;
    if (isOccupied(data[i])) return 0;
    return 128;
  });
  ctx.drawImage(img, 0, 0, mapCanvas.width, mapCanvas.height);
  drawRobotAndTrail(ctx, mapCanvas);

  if (plan) {
    const [fx, fy] = toPx(mapCanvas, plan.from.x, plan.from.y);
    const [gx, gy] = toPx(mapCanvas, plan.goal.x, plan.goal.y);
    // Dashed straight line from the robot to the goal.
    ctx.strokeStyle = "red";
    ctx.lineWidth = 2;
    ctx.setLineDash([8, 6]);
    ctx.beginPath(); ctx.moveTo(fx, fy); ctx.lineTo(gx, gy); ctx.stroke();
    ctx.setLineDash([]);
    // The A* path.
    if (plan.path) {
      ctx.strokeStyle = "#00a0c8";
      ctx.lineWidth = 3;
      ctx.beginPath();
      plan.path.forEach((p, k) => {
        const [px, py] = toPx(mapCanvas, (p.c + 0.5) * world.cell, (p.r + 0.5) * world.cell);
        if (k === 0) ctx.moveTo(px, py); else ctx.lineTo(px, py);
      });
      ctx.stroke();
    }
    // The goal: a green dot.
    ctx.fillStyle = "green";
    ctx.beginPath(); ctx.arc(gx, gy, 6, 0, 2 * Math.PI); ctx.fill();
  }
}

function drawChart() {
  const ctx = chart.getContext("2d");
  const W = chart.width, H = chart.height;
  const left = 44, bottom = 30, top = 10, right = 12;
  ctx.clearRect(0, 0, W, H);
  const curve = explorer ? explorer.curve : [];
  const maxX = Math.max(10, ...curve.map((p) => p[0]));
  const sx = (x) => left + (x / maxX) * (W - left - right);
  const sy = (y) => H - bottom - (y / 100) * (H - bottom - top);

  // Axes and grid lines
  ctx.strokeStyle = "#ddd";
  ctx.fillStyle = "#555";
  ctx.font = "11px system-ui, sans-serif";
  ctx.lineWidth = 1;
  ctx.textAlign = "right";
  for (let y = 0; y <= 100; y += 25) {
    ctx.beginPath(); ctx.moveTo(left, sy(y)); ctx.lineTo(W - right, sy(y)); ctx.stroke();
    ctx.fillText(y + "%", left - 4, sy(y) + 4);
  }
  ctx.textAlign = "center";
  // Pick a label step so there are at most about 8 labels.
  let stepX = 5;
  while (maxX / stepX > 8) stepX *= 2; // 5, 10, 20, 40, 80, ...
  for (let x = 0; x <= maxX; x += stepX) ctx.fillText(x + " m", sx(x), H - bottom + 14);
  ctx.fillText("metres driven", (left + W - right) / 2, H - 2);

  // The coverage line
  ctx.strokeStyle = "#2f6fde";
  ctx.lineWidth = 2;
  ctx.beginPath();
  curve.forEach(([x, y], k) => (k === 0 ? ctx.moveTo(sx(x), sy(y)) : ctx.lineTo(sx(x), sy(y))));
  ctx.stroke();
}

function updateStats() {
  const e = explorer;
  $("sScans").textContent = e ? e.scans : 0;
  $("sDriven").textContent = (e ? e.driven : 0).toFixed(1);
  $("sCoverage").textContent = (e ? e.stats.coverage : 0).toFixed(1) + "%";
  $("sAccuracy").textContent = (e ? e.stats.accuracy : 0).toFixed(1) + "%";
  $("sWalls").textContent = (e ? e.stats.wallsFound : 0).toFixed(1) + "%";
  $("sBumps").textContent = e ? e.bumps : 0;
  $("sStatus").textContent = e ? (running || e.done ? e.status : "Paused") : "Ready";
}

function drawAll() {
  drawReal();
  drawMap();
  drawChart();
  updateStats();
}

// ---------- Exploration controls ----------

function newExplorer() {
  return new Explorer(world, start, {
    noise: Number($("noise").value) / 100,  // slider is in cm
    maxRange: Number($("range").value),
    seed: 42,
  });
}

function reset() {
  running = false;
  explorer = null;
  plan = null;
  $("message").textContent = "";
  drawAll();
}

// One turn of the robot. Remember where it scanned from, so the beams are drawn from there.
function stepOnce() {
  explorer.scanFrom = { x: explorer.x, y: explorer.y };
  explorer.step();
  if (explorer.done) {
    running = false;
    $("message").textContent = "Click the robot's map to plan a path";
  }
}

// The animation loop: runs as many turns as the speed slider asks for.
let lastTime = null;
let owed = 0; // how many turns we owe since the last frame
function frame(time) {
  if (running && explorer && !explorer.done) {
    const dt = lastTime === null ? 0 : (time - lastTime) / 1000;
    owed += dt * Number($("speed").value);
    let n = 0;
    while (owed >= 1 && !explorer.done && n < 5) { // at most 5 turns per frame, to stay smooth
      stepOnce();
      owed -= 1;
      n++;
    }
    if (n === 5) owed = 0;
    drawAll();
  } else {
    owed = 0;
  }
  lastTime = time;
  requestAnimationFrame(frame);
}

$("start").addEventListener("click", () => {
  if (!start) {
    $("message").textContent = "Click the real floor plan to set the start point";
    return;
  }
  if (!explorer) explorer = newExplorer();
  if (explorer.done) return;
  running = true;
  owed = 1; // take the first scan right away
  drawAll();
});
$("pause").addEventListener("click", () => {
  running = false;
  drawAll();
});
$("reset").addEventListener("click", reset);

// Sliders: show their values. Changing noise or range starts a fresh run.
for (const [id, label] of [["noise", "noiseVal"], ["range", "rangeVal"], ["speed", "speedVal"]]) {
  $(id).addEventListener("input", () => {
    $(label).textContent = $(id).value;
    if (id !== "speed") reset();
  });
}

// ---------- Floor plans ----------

// Where on the plan (in metres) did the mouse click?
function clickToMetres(canvas, event) {
  const rect = canvas.getBoundingClientRect();
  const px = ((event.clientX - rect.left) / rect.width) * canvas.width;
  const py = ((event.clientY - rect.top) / rect.height) * canvas.height;
  const s = canvas.width / (world.nx * world.cell);
  return { x: px / s, y: (canvas.height - py) / s };
}

// Click on the real floor plan to set the start point.
realCanvas.addEventListener("click", (event) => {
  const p = clickToMetres(realCanvas, event);
  const { r, c } = toCell(world, p.x, p.y);
  if (r < 0 || r >= world.ny || c < 0 || c >= world.nx || tooClose[r * world.nx + c]) {
    $("message").textContent = "That spot is a wall or too close to one. Pick an open spot.";
    return;
  }
  start = p;
  reset();
});

// After exploring, click the robot's map to plan a path from the robot to that spot.
mapCanvas.addEventListener("click", (event) => {
  if (!explorer || !explorer.done) return;
  const goal = clickToMetres(mapCanvas, event);
  const g = toCell(world, goal.x, goal.y);
  if (g.r < 0 || g.r >= world.ny || g.c < 0 || g.c >= world.nx) return;
  const from = { x: explorer.x, y: explorer.y };
  const path = planPath(explorer.map, toCell(world, from.x, from.y), g);
  plan = { path, from, goal };
  if (path) {
    const length = pathLength(path, world.cell);
    const straight = Math.hypot(goal.x - from.x, goal.y - from.y);
    $("message").textContent = `Path: ${length.toFixed(1)} m (straight line: ${straight.toFixed(1)} m)`;
  } else {
    $("message").textContent = "No path to that spot: it is unknown, a wall, or too close to a wall.";
  }
  drawAll();
});

// Turn the uploaded picture into a floor plan, using the real width box.
function useUploaded() {
  const width = Number($("realWidth").value);
  if (!(width > 0)) {
    $("message").textContent = "Please enter a real width above 0 m.";
    return;
  }
  loadWorld(imageToGrid(uploaded.data, uploaded.width, uploaded.height, width));
}

// Read the chosen picture file and get its pixels.
$("file").addEventListener("change", () => {
  const file = $("file").files[0];
  if (!file) return;
  const img = new Image();
  img.onload = () => {
    const off = document.createElement("canvas");
    off.width = img.naturalWidth;
    off.height = img.naturalHeight;
    const ctx = off.getContext("2d");
    ctx.drawImage(img, 0, 0);
    uploaded = ctx.getImageData(0, 0, off.width, off.height);
    URL.revokeObjectURL(img.src);
    document.querySelector('input[name="plan"][value="upload"]').checked = true;
    useUploaded();
  };
  img.onerror = () => { $("message").textContent = "Sorry, that file could not be read as a picture."; };
  img.src = URL.createObjectURL(file);
});

// Switching between "Example house" and "Upload your own".
for (const radio of document.querySelectorAll('input[name="plan"]')) {
  radio.addEventListener("change", () => {
    if (radio.value === "example") loadWorld(makeExampleHouse());
    else if (uploaded) useUploaded();
    else $("file").click(); // no picture yet: open the file picker
  });
}

// Changing the real width rebuilds the uploaded plan.
$("realWidth").addEventListener("change", () => {
  const uploadChosen = document.querySelector('input[name="plan"][value="upload"]').checked;
  if (uploadChosen && uploaded) useUploaded();
});

// Save the example house as a black-on-white PNG (4 pixels per cell).
$("download").addEventListener("click", () => {
  const png = worldToRgba(makeExampleHouse(), 4);
  const off = document.createElement("canvas");
  off.width = png.width;
  off.height = png.height;
  off.getContext("2d").putImageData(new ImageData(png.rgba, png.width, png.height), 0, 0);
  off.toBlob((blob) => {
    const a = document.createElement("a");
    a.href = URL.createObjectURL(blob);
    a.download = "example_floor_plan.png";
    a.click();
    setTimeout(() => URL.revokeObjectURL(a.href), 1000);
  }, "image/png");
});

// ---------- Start up ----------
loadWorld(makeExampleHouse());
requestAnimationFrame(frame);
