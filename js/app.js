// Page logic: controls, drawing and animation. This is the only file that touches the page.
import { makeExampleHouse, imageToGrid, worldToRgba, toCell } from "./world.js";
import { ROBOT_RADIUS } from "./lidar.js";
import { isFree, isOccupied, updateMap } from "./mapping.js";
import { Explorer, inflate, SAFETY_M, planPath, pathLength } from "./explore.js";
import { parseLine, LineSplitter, readingToBeam, beamEnd, newRoomMap, mapToRgba, ROOM_M, SENSOR_MAX_M } from "./sensor.js";

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
function gridToCanvas(colourOf, nx = world.nx, ny = world.ny) {
  const off = document.createElement("canvas");
  off.width = nx;
  off.height = ny;
  const ctx = off.getContext("2d");
  const img = ctx.createImageData(nx, ny);
  for (let r = 0; r < ny; r++) {
    for (let c = 0; c < nx; c++) {
      const v = colourOf(r * nx + c);
      const p = ((ny - 1 - r) * nx + c) * 4; // image rows go top-down
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
  savePng(worldToRgba(makeExampleHouse(), 4), "example_floor_plan.png");
});

// Save pixels ({ rgba, width, height }) as a PNG file the user downloads.
function savePng(png, fileName) {
  const off = document.createElement("canvas");
  off.width = png.width;
  off.height = png.height;
  off.getContext("2d").putImageData(new ImageData(png.rgba, png.width, png.height), 0, 0);
  off.toBlob((blob) => {
    const a = document.createElement("a");
    a.href = URL.createObjectURL(blob);
    a.download = fileName;
    a.click();
    setTimeout(() => URL.revokeObjectURL(a.href), 1000);
  }, "image/png");
}

// ---------- Real sensor mode ----------
// Readings come from the ESP32 over USB (the Web Serial API, Chrome or Edge only).

const sensorCanvas = $("sensorCanvas");
let sensorMap = newRoomMap(); // the map of the room, all unknown at first
let scanner = null;           // where the scanner is (metres), or null if not set yet
let placing = true;           // true = the next click on the map sets the scanner's position
let headingDeg = 90;          // the way the scanner points when the servo is at 90 degrees
let flipped = false;          // true = the servo turns the other way (map came out mirrored)
let dots = [];                // recent hit points, for the radar dots
let latest = null;            // the latest reading and its beam
let readingCount = 0, sweepCount = 0;
let port = null;              // the USB serial port, while connected
let drawPending = false;
const MAX_DOTS = 182;         // about two sweeps of dots

function sensorMessage(text) {
  $("sensorMessage").textContent = text;
}

// Metres -> canvas pixels, for the 8 m x 8 m room.
function roomToPx(x, y) {
  const s = sensorCanvas.width / ROOM_M;
  return [x * s, sensorCanvas.height - y * s];
}

// Redraw at the next animation frame (readings can arrive faster than we need to draw).
function requestSensorDraw() {
  if (drawPending) return;
  drawPending = true;
  requestAnimationFrame(() => {
    drawPending = false;
    drawSensor();
  });
}

function drawSensor() {
  const ctx = sensorCanvas.getContext("2d");
  const W = sensorCanvas.width, H = sensorCanvas.height;
  ctx.imageSmoothingEnabled = false;

  // The map: gray = unknown, white = free, black = occupied.
  const data = sensorMap.data;
  const img = gridToCanvas((i) => (isFree(data[i]) ? 255 : isOccupied(data[i]) ? 0 : 128),
    sensorMap.nx, sensorMap.ny);
  ctx.drawImage(img, 0, 0, W, H);

  if (scanner) {
    const [sx, sy] = roomToPx(scanner.x, scanner.y);
    const rangePx = (SENSOR_MAX_M / ROOM_M) * W;
    // Canvas angles go clockwise because y points down, so we use minus signs.
    const h = (headingDeg * Math.PI) / 180;

    // The half circle the sensor can see (servo 0 to 180 degrees).
    ctx.strokeStyle = "rgba(0, 160, 60, 0.6)";
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.moveTo(sx, sy);
    ctx.arc(sx, sy, rangePx, -h - Math.PI / 2, -h + Math.PI / 2);
    ctx.closePath();
    ctx.stroke();

    // The radar sweep line, from the scanner to the latest reading.
    if (latest) {
      const end = beamEnd(scanner.x, scanner.y, latest.beam);
      const [ex, ey] = roomToPx(end.x, end.y);
      ctx.strokeStyle = "rgba(0, 200, 80, 0.9)";
      ctx.lineWidth = 2;
      ctx.beginPath(); ctx.moveTo(sx, sy); ctx.lineTo(ex, ey); ctx.stroke();
    }

    // The scanner: a blue circle with an arrow showing which way it faces.
    ctx.fillStyle = "#2f6fde";
    ctx.beginPath(); ctx.arc(sx, sy, 7, 0, 2 * Math.PI); ctx.fill();
    ctx.strokeStyle = "#2f6fde";
    ctx.lineWidth = 3;
    ctx.beginPath(); ctx.moveTo(sx, sy); ctx.lineTo(sx + 25 * Math.cos(h), sy - 25 * Math.sin(h)); ctx.stroke();
  }

  // The radar dots: newer dots are brighter.
  dots.forEach((d, k) => {
    const [px, py] = roomToPx(d.x, d.y);
    const alpha = 0.25 + 0.75 * ((k + 1) / dots.length);
    ctx.fillStyle = `rgba(0, 230, 90, ${alpha})`;
    ctx.beginPath(); ctx.arc(px, py, 3, 0, 2 * Math.PI); ctx.fill();
  });

  // The text next to the map.
  $("rAngle").textContent = latest ? `${latest.reading.angleDeg}°` : "-";
  $("rDistance").textContent = !latest ? "-"
    : latest.reading.ok ? `${(latest.reading.distanceMm / 1000).toFixed(2)} m` : "no reading";
  $("rCount").textContent = readingCount;
  $("rSweeps").textContent = sweepCount;
  $("headingVal").textContent = headingDeg;
  $("heading").value = headingDeg;
  $("scannerPos").textContent = scanner
    ? `(${scanner.x.toFixed(2)} m, ${scanner.y.toFixed(2)} m)` : "not set";
}

// Handle one line from the ESP32.
function handleLine(line) {
  const msg = parseLine(line);
  if (!msg) return;
  if (msg.type === "message") {
    sensorMessage("ESP32 says: " + msg.text);
  } else if (msg.type === "sweepEnd") {
    sweepCount++;
  } else {
    readingCount++;
    const beam = readingToBeam(msg, headingDeg, flipped);
    latest = { reading: msg, beam };
    if (!scanner) {
      sensorMessage("Readings are arriving. Click the map to show where the scanner is.");
    } else {
      // Add this one reading to the map with the same updateMap the simulation uses.
      updateMap(sensorMap, scanner.x, scanner.y, [beam.angleRad], [beam.distanceM], [beam.hit]);
      if (beam.hit) {
        dots.push(beamEnd(scanner.x, scanner.y, beam));
        if (dots.length > MAX_DOTS) dots.shift();
      }
    }
  }
  requestSensorDraw();
}

// Keep reading text from the USB port until it is unplugged or closed.
async function readFromPort() {
  const decoder = new TextDecoderStream();
  const piped = port.readable.pipeTo(decoder.writable).catch(() => {});
  const reader = decoder.readable.getReader();
  const splitter = new LineSplitter();
  try {
    while (true) {
      const { value, done } = await reader.read();
      if (done) break;
      for (const line of splitter.push(value)) handleLine(line);
    }
  } catch (err) {
    sensorMessage("The sensor stopped sending (was the USB cable unplugged?). Click Connect sensor to try again.");
  } finally {
    reader.releaseLock();
    await piped;
    try { await port.close(); } catch (err) { /* already closed */ }
    port = null;
    $("rConnection").textContent = "Not connected";
    $("connect").disabled = false;
  }
}

$("connect").addEventListener("click", async () => {
  if (!("serial" in navigator)) return;
  try {
    // The browser shows a list of USB devices. Pick the ESP32 (CP2102 / "USB to UART").
    port = await navigator.serial.requestPort();
    await port.open({ baudRate: 115200 });
  } catch (err) {
    port = null;
    if (err.name === "NotFoundError") sensorMessage("No device was chosen.");
    else sensorMessage("Could not open the USB port. Close the Arduino Serial Monitor (only one program can use it) and try again.");
    return;
  }
  $("rConnection").textContent = "Connected";
  $("connect").disabled = true;
  sensorMessage(scanner ? "Connected. The ESP32 restarts, waits 5 seconds, then starts sweeping."
    : "Connected. Now click the map to show where the scanner is.");
  readFromPort();
});

// Click the map to set (or move) the scanner's position.
sensorCanvas.addEventListener("click", (event) => {
  if (!placing) return;
  const rect = sensorCanvas.getBoundingClientRect();
  const x = ((event.clientX - rect.left) / rect.width) * ROOM_M;
  const y = (1 - (event.clientY - rect.top) / rect.height) * ROOM_M;
  if (x <= 0 || x >= ROOM_M || y <= 0 || y >= ROOM_M) return;
  scanner = { x, y };
  placing = false;
  sensorMessage("Scanner placed. Set which way it faces with the slider.");
  requestSensorDraw();
});

$("heading").addEventListener("input", () => {
  headingDeg = Number($("heading").value);
  requestSensorDraw();
});

$("clearMap").addEventListener("click", () => {
  sensorMap = newRoomMap();
  dots = [];
  latest = null;
  readingCount = 0;
  sweepCount = 0;
  sensorMessage("Map cleared.");
  requestSensorDraw();
});

$("moveScanner").addEventListener("click", () => {
  placing = true;
  sensorMessage("Click the map where the scanner is now.");
});

$("turn180").addEventListener("click", () => {
  headingDeg = (headingDeg + 180) % 360;
  sensorMessage(`Turned 180°. The scanner now faces ${headingDeg}°. Turn the real scanner around too.`);
  requestSensorDraw();
});

$("flip").addEventListener("click", () => {
  flipped = !flipped;
  sensorMessage(flipped ? "Flipped: the servo now counts the other way. Click Clear map to start again."
    : "Flipped back to normal. Click Clear map to start again.");
  requestSensorDraw();
});

$("downloadMap").addEventListener("click", () => {
  savePng(mapToRgba(sensorMap, 4), "lidar_room_map.png");
});

// Switching between Simulation and Real sensor.
for (const radio of document.querySelectorAll('input[name="mode"]')) {
  radio.addEventListener("change", () => {
    const sensorMode = radio.value === "sensor";
    $("simMode").hidden = sensorMode;
    $("sensorMode").hidden = !sensorMode;
    if (sensorMode) {
      running = false; // pause the simulation while we use the real sensor
      drawAll();
      if (!scanner) sensorMessage("Click the map to show where the scanner is, then click Connect sensor.");
      requestSensorDraw();
    }
  });
}

// No Web Serial (Firefox, Safari, phones): explain, and turn off the Connect button.
if (!("serial" in navigator)) {
  $("noSerial").hidden = false;
  $("connect").disabled = true;
}

// ---------- Start up ----------
loadWorld(makeExampleHouse());
requestAnimationFrame(frame);
