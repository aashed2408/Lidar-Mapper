// The robot's map: an occupancy grid built from (angle, distance) readings.
//
// Each cell holds a "log-odds" number: negative = probably free,
// positive = probably occupied, 0 = unknown.
// This file knows nothing about the fake floor plan, so the same logic can be used
// with a real sensor.

export const FREE_ADD = -0.4;  // added to a cell a beam passes through
export const OCC_ADD = 0.85;   // added to the cell where a beam hit something
export const CLAMP = 5;        // values are kept between -5 and +5
export const THRESHOLD = 0.3;  // below -0.3 = free, above +0.3 = occupied

// A fresh map: every cell is 0 (unknown).
export function newMap(nx, ny, cell) {
  return { nx, ny, cell, data: new Float64Array(nx * ny) };
}

// Add one set of readings, taken from position (x, y), to the map.
//   anglesRad: beam angles in radians
//   distancesM: measured distances in metres
//   hits: true (or 1) if that beam hit something
// Works with any number of readings. Changes `logodds` and returns it.
export function updateMap(logodds, x, y, anglesRad, distancesM, hits) {
  const { nx, ny, cell, data } = logodds;

  for (let b = 0; b < anglesRad.length; b++) {
    const dx = Math.cos(anglesRad[b]), dy = Math.sin(anglesRad[b]);
    const dist = distancesM[b];
    const hit = Boolean(hits[b]);

    // Where the beam ended, and which cell that is.
    const endC = Math.floor((x + dx * dist) / cell);
    const endR = Math.floor((y + dy * dist) / cell);

    // How far along the beam the cells are free.
    // For a hit: stop one cell short of the hit point, so noise can't erase walls.
    // For a miss: free all the way to the end (max range).
    const freeLen = hit ? dist - cell : dist;

    // Walk along the beam in half-cell steps and collect the cells we pass.
    const seen = new Set(); // each cell only once per beam
    for (let k = 0; k * (cell / 2) <= Math.max(freeLen, 0) + 1e-9; k++) {
      const s = k * (cell / 2);
      const c = Math.floor((x + dx * s) / cell);
      const r = Math.floor((y + dy * s) / cell);
      if (c < 0 || c >= nx || r < 0 || r >= ny) continue;
      if (hit && r === endR && c === endC) continue; // never free the hit cell
      seen.add(r * nx + c);
    }
    for (const i of seen) data[i] += FREE_ADD;

    // The cell at the hit point is occupied.
    if (hit && endR >= 0 && endR < ny && endC >= 0 && endC < nx) {
      data[endR * nx + endC] += OCC_ADD;
    }
  }

  // Clamp every value to -5..+5.
  for (let i = 0; i < data.length; i++) {
    if (data[i] > CLAMP) data[i] = CLAMP;
    else if (data[i] < -CLAMP) data[i] = -CLAMP;
  }
  return logodds;
}

// Reading the map, one cell at a time.
export const isFree = (v) => v < -THRESHOLD;
export const isOccupied = (v) => v > THRESHOLD;
export const isUnknown = (v) => !isFree(v) && !isOccupied(v);
