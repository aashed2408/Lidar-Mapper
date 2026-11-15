// Frontier exploration (Yamauchi, 1997), A* path planning (Hart, Nilsson & Raphael, 1968),
// and the measurements.
import { scan } from "./lidar.js";
import { newMap, updateMap, isFree, isOccupied, isUnknown } from "./mapping.js";
import { mulberry32 } from "./rng.js";
import { toCell, cellCenter } from "./world.js";

export const SAFETY_M = 0.18;  // robot radius (0.15 m) plus a small margin
export const ESCAPE_M = 0.30;  // near the robot we ignore the safety zone
export const MOVE_M = 0.5;     // move up to 0.5 m per turn
export const IGNORE_HALF = 3;  // ignore a 7 x 7 area (3 cells each side) around a reached target
export const MAX_SCANS = 400;

// The 8 directions the robot can move: [row change, col change].
const DIRS8 = [[-1, -1], [-1, 0], [-1, 1], [0, -1], [0, 1], [1, -1], [1, 0], [1, 1]];
const DIRS4 = [[-1, 0], [1, 0], [0, -1], [0, 1]];

// Grow the "on" cells of `mask` outward by radiusM metres (makes the safety zone).
export function inflate(mask, nx, ny, cell, radiusM) {
  const r = radiusM / cell;
  const R = Math.ceil(r);
  // All the offsets that fit inside a circle of radius r.
  const offsets = [];
  for (let dr = -R; dr <= R; dr++) {
    for (let dc = -R; dc <= R; dc++) {
      if (dr * dr + dc * dc <= r * r) offsets.push([dr, dc]);
    }
  }
  const out = new Uint8Array(nx * ny);
  for (let row = 0; row < ny; row++) {
    for (let col = 0; col < nx; col++) {
      if (!mask[row * nx + col]) continue;
      // Stamp a circle around every "on" cell.
      for (const [dr, dc] of offsets) {
        const rr = row + dr, cc = col + dc;
        if (rr >= 0 && rr < ny && cc >= 0 && cc < nx) out[rr * nx + cc] = 1;
      }
    }
  }
  return out;
}

// The map's occupied cells grown by 0.18 m. The robot can't plan through these.
export function safetyZone(map) {
  const occ = map.data.map((v) => (isOccupied(v) ? 1 : 0));
  return inflate(occ, map.nx, map.ny, map.cell, SAFETY_M);
}

// Walk back through the `parent` links to build the path (start first).
function buildPath(parent, end, nx) {
  const path = [];
  for (let i = end; i !== -1; i = parent[i]) path.push({ r: Math.floor(i / nx), c: i % nx });
  return path.reverse();
}

// A* search on a grid: 8-direction moves, straight-line distance as the guess (heuristic).
// `blocked` is an array of 0/1. Returns a list of cells, or null if there is no path.
export function astar(blocked, nx, ny, start, goal) {
  const n = nx * ny;
  const cost = new Float64Array(n).fill(Infinity);
  const parent = new Int32Array(n).fill(-1);
  const closed = new Uint8Array(n);
  const s = start.r * nx + start.c;
  const g = goal.r * nx + goal.c;
  const h = (r, c) => Math.hypot(r - goal.r, c - goal.c);
  cost[s] = 0;

  // A small "binary heap" that always gives back the cell with the lowest score.
  const heap = [[h(start.r, start.c), s]];
  const push = (item) => {
    heap.push(item);
    let i = heap.length - 1;
    while (i > 0) {
      const p = (i - 1) >> 1;
      if (heap[p][0] <= heap[i][0]) break;
      [heap[p], heap[i]] = [heap[i], heap[p]];
      i = p;
    }
  };
  const pop = () => {
    const top = heap[0];
    const last = heap.pop();
    if (heap.length > 0) {
      heap[0] = last;
      let i = 0;
      for (;;) {
        const a = 2 * i + 1, b = a + 1;
        let m = i;
        if (a < heap.length && heap[a][0] < heap[m][0]) m = a;
        if (b < heap.length && heap[b][0] < heap[m][0]) m = b;
        if (m === i) break;
        [heap[m], heap[i]] = [heap[i], heap[m]];
        i = m;
      }
    }
    return top;
  };

  while (heap.length > 0) {
    const [, i] = pop();
    if (i === g) return buildPath(parent, g, nx);
    if (closed[i]) continue;
    closed[i] = 1;
    const r = Math.floor(i / nx), c = i % nx;
    for (const [dr, dc] of DIRS8) {
      const rr = r + dr, cc = c + dc;
      if (rr < 0 || rr >= ny || cc < 0 || cc >= nx) continue;
      const j = rr * nx + cc;
      if (blocked[j]) continue;
      const newCost = cost[i] + Math.hypot(dr, dc);
      if (newCost < cost[j]) {
        cost[j] = newCost;
        parent[j] = i;
        push([newCost + h(rr, cc), j]);
      }
    }
  }
  return null;
}

// Plan a path on the robot's map. Unknown cells and the safety zone count as blocked.
export function planPath(map, start, goal) {
  const zone = safetyZone(map);
  const blocked = new Uint8Array(map.nx * map.ny);
  for (let i = 0; i < blocked.length; i++) blocked[i] = zone[i] || isUnknown(map.data[i]) ? 1 : 0;
  return astar(blocked, map.nx, map.ny, start, goal);
}

// Length of a path of cells, in metres.
export function pathLength(path, cell) {
  let total = 0;
  for (let i = 1; i < path.length; i++) {
    total += Math.hypot(path[i].r - path[i - 1].r, path[i].c - path[i - 1].c);
  }
  return total * cell;
}

// ---------------- Measurements ----------------

// Flood fill from the start through the REAL floor plan, with walls grown by 0.18 m.
export function reachableFloor(world, start) {
  const { nx, ny, cell, solid } = world;
  const grown = inflate(solid, nx, ny, cell, SAFETY_M);
  const seen = new Uint8Array(nx * ny);
  const s = toCell(world, start.x, start.y);
  if (s.r < 0 || s.r >= ny || s.c < 0 || s.c >= nx || grown[s.r * nx + s.c]) return seen;
  seen[s.r * nx + s.c] = 1;
  const queue = [s.r * nx + s.c];
  for (let q = 0; q < queue.length; q++) {
    const r = Math.floor(queue[q] / nx), c = queue[q] % nx;
    for (const [dr, dc] of DIRS4) {
      const rr = r + dr, cc = c + dc;
      if (rr < 0 || rr >= ny || cc < 0 || cc >= nx) continue;
      const j = rr * nx + cc;
      if (!grown[j] && !seen[j]) { seen[j] = 1; queue.push(j); }
    }
  }
  return seen;
}

// Real solid cells that touch open floor (up, down, left or right).
export function wallEdges(world) {
  const { nx, ny, solid } = world;
  const edges = new Uint8Array(nx * ny);
  for (let r = 0; r < ny; r++) {
    for (let c = 0; c < nx; c++) {
      if (!solid[r * nx + c]) continue;
      for (const [dr, dc] of DIRS4) {
        const rr = r + dr, cc = c + dc;
        if (rr >= 0 && rr < ny && cc >= 0 && cc < nx && !solid[rr * nx + cc]) {
          edges[r * nx + c] = 1;
          break;
        }
      }
    }
  }
  return edges;
}

// Coverage, accuracy and walls found, all as percentages.
export function measure(map, world, reachable, edges) {
  let reach = 0, reachFree = 0, known = 0, correct = 0, edgeCount = 0, edgeFound = 0;
  for (let i = 0; i < map.data.length; i++) {
    const free = isFree(map.data[i]), occ = isOccupied(map.data[i]);
    const solid = world.solid[i] === 1;
    if (reachable[i]) { reach++; if (free) reachFree++; }
    if (free || occ) { known++; if ((free && !solid) || (occ && solid)) correct++; }
    if (edges[i]) { edgeCount++; if (occ) edgeFound++; }
  }
  return {
    coverage: reach ? (100 * reachFree) / reach : 0,
    accuracy: known ? (100 * correct) / known : 0,
    wallsFound: edgeCount ? (100 * edgeFound) / edgeCount : 0,
  };
}

// ---------------- The exploration, one turn at a time ----------------

// Usage: const ex = new Explorer(world, start, { noise, maxRange, seed });
//        while (!ex.done) ex.step();
export class Explorer {
  constructor(world, start, { noise = 0.02, maxRange = 4.0, seed = 42 } = {}) {
    this.world = world;
    this.noise = noise;
    this.maxRange = maxRange;
    this.rand = mulberry32(seed);
    this.map = newMap(world.nx, world.ny, world.cell);
    this.ignored = new Uint8Array(world.nx * world.ny);
    this.reachable = reachableFloor(world, start);
    this.edges = wallEdges(world);
    this.x = start.x;
    this.y = start.y;
    this.trail = [{ x: start.x, y: start.y }];
    this.driven = 0;
    this.scans = 0;
    this.bumps = 0;
    this.curve = [];         // [metres driven, coverage %] after each scan
    this.lastScan = null;    // the latest scan, for drawing the laser beams
    this.stats = { coverage: 0, accuracy: 0, wallsFound: 0 };
    this.done = false;
    this.status = "Ready";
  }

  // Mark a 7 x 7 area around cell (r, c) as "ignore from now on".
  ignoreAround(r, c) {
    const { nx, ny } = this.world;
    for (let rr = Math.max(r - IGNORE_HALF, 0); rr <= Math.min(r + IGNORE_HALF, ny - 1); rr++) {
      for (let cc = Math.max(c - IGNORE_HALF, 0); cc <= Math.min(c + IGNORE_HALF, nx - 1); cc++) {
        this.ignored[rr * nx + cc] = 1;
      }
    }
  }

  // Frontier = free cell, outside the safety zone, touching an unknown cell (up/down/left/right).
  isFrontier(i, zone) {
    const { nx, ny } = this.world;
    const d = this.map.data;
    if (!isFree(d[i]) || zone[i] || this.ignored[i]) return false;
    const r = Math.floor(i / nx), c = i % nx;
    for (const [dr, dc] of DIRS4) {
      const rr = r + dr, cc = c + dc;
      if (rr >= 0 && rr < ny && cc >= 0 && cc < nx && isUnknown(d[rr * nx + cc])) return true;
    }
    return false;
  }

  // Breadth-first search (8 directions) from the robot to the nearest frontier,
  // through cells the robot may drive on. Returns the path, or null.
  findFrontierPath(walk, zone, robot) {
    const { nx, ny } = this.world;
    const start = robot.r * nx + robot.c;
    const parent = new Int32Array(nx * ny).fill(-2); // -2 = not visited yet
    parent[start] = -1;
    const queue = [start];
    for (let q = 0; q < queue.length; q++) {
      const i = queue[q];
      if (this.isFrontier(i, zone)) return buildPath(parent, i, nx);
      const r = Math.floor(i / nx), c = i % nx;
      for (const [dr, dc] of DIRS8) {
        const rr = r + dr, cc = c + dc;
        if (rr < 0 || rr >= ny || cc < 0 || cc >= nx) continue;
        const j = rr * nx + cc;
        if (parent[j] === -2 && walk[j]) { parent[j] = i; queue.push(j); }
      }
    }
    return null;
  }

  // One turn: scan, update the map, find the nearest frontier, move up to 0.5 m.
  step() {
    if (this.done) return;
    const world = this.world;
    const { nx, ny, cell } = world;

    // 1. Scan and update the map.
    const s = scan(world, this.x, this.y, this.maxRange, this.noise, this.rand);
    updateMap(this.map, this.x, this.y, s.angles, s.distances, s.hits);
    this.lastScan = s;
    this.scans++;
    this.stats = measure(this.map, world, this.reachable, this.edges);
    this.curve.push([this.driven, this.stats.coverage]);

    // 2. Which cells can the robot drive through? Free and outside the safety zone.
    //    Escape rule: within 30 cm of the robot, only the map's occupied cells block it.
    const robot = toCell(world, this.x, this.y);
    const zone = safetyZone(this.map);
    const esc = Math.round(ESCAPE_M / cell);
    const walk = new Uint8Array(nx * ny);
    for (let r = 0; r < ny; r++) {
      for (let c = 0; c < nx; c++) {
        const i = r * nx + c;
        const v = this.map.data[i];
        const near = (r - robot.r) ** 2 + (c - robot.c) ** 2 <= esc * esc;
        walk[i] = isFree(v) && (near ? !isOccupied(v) : !zone[i]) ? 1 : 0;
      }
    }
    walk[robot.r * nx + robot.c] = 1; // the robot can always stand where it is

    // 3. Find the nearest frontier.
    let path = null;
    for (;;) {
      path = this.findFrontierPath(walk, zone, robot);
      if (path === null || path.length > 1) break;
      // The robot is standing on its target already: ignore it and look again.
      this.ignoreAround(path[0].r, path[0].c);
    }
    if (path === null) {
      this.done = true;
      this.status = "Done: map complete";
      return;
    }

    // 4. Move up to 0.5 m along the path.
    const target = path[path.length - 1];
    let moved = 0;
    let reached = false;
    for (let k = 1; k < path.length; k++) {
      const p = cellCenter(world, path[k].r, path[k].c);
      const stepLen = Math.hypot(p.x - this.x, p.y - this.y);
      if (moved + stepLen > MOVE_M + 1e-9) break;          // this step would go past 0.5 m
      if (world.solid[path[k].r * nx + path[k].c]) {      // bump check against the REAL plan
        this.bumps++;
        break;
      }
      moved += stepLen;
      this.x = p.x;
      this.y = p.y;
      this.trail.push({ x: p.x, y: p.y });
      reached = path[k].r === target.r && path[k].c === target.c;
    }
    this.driven += moved;
    // Ignore rule: we got there, so stop chasing this spot.
    if (reached) this.ignoreAround(target.r, target.c);

    if (this.scans >= MAX_SCANS) {
      this.done = true;
      this.status = "Stopped: scan limit";
    } else {
      this.status = "Exploring";
    }
  }
}
