// Real sensor: turning the lines the ESP32 sends into readings for the map.
//
// The ESP32 sends one line per reading:   angle_degrees,distance_mm,ok
// and "SWEEP_END" after each sweep. Anything else (like a wiring warning) is a message.
// This file uses NO browser APIs, so Node can test it.

import { newMap, isFree, isOccupied } from "./mapping.js";

export const ROOM_M = 8;           // the real-sensor map is an empty 8 m x 8 m room
export const ROOM_CELL = 0.05;     // 5 cm cells, so 160 x 160 cells
export const SENSOR_MAX_M = 4.0;   // the VL53L1X reaches about 4 m

// A fresh, empty (all unknown) map of the room.
export function newRoomMap() {
  const n = Math.round(ROOM_M / ROOM_CELL);
  return newMap(n, n, ROOM_CELL);
}

// Read one line from the ESP32. Returns one of:
//   { type: "reading", angleDeg, distanceMm, ok }   for "90,1234,1"
//   { type: "sweepEnd" }                            for "SWEEP_END"
//   { type: "message", text }                       for any other text
//   null                                            for an empty line
export function parseLine(line) {
  const text = String(line).trim();
  if (text === "") return null;
  if (text === "SWEEP_END") return { type: "sweepEnd" };

  const parts = text.split(",");
  if (parts.length === 3) {
    const angleDeg = Number(parts[0]);
    const distanceMm = Number(parts[1]);
    const okText = parts[2].trim();
    // All three must be proper numbers, and ok must be 0 or 1.
    if (parts.every((p) => p.trim() !== "") && Number.isFinite(angleDeg) &&
        Number.isFinite(distanceMm) && (okText === "0" || okText === "1")) {
      return { type: "reading", angleDeg, distanceMm, ok: okText === "1" };
    }
  }
  return { type: "message", text };
}

// Serial data arrives in random-sized chunks, not neat lines.
// A LineSplitter collects the chunks and hands back only complete lines.
export class LineSplitter {
  constructor() {
    this.leftover = ""; // the start of a line we haven't seen the end of yet
  }

  // Add a chunk of text. Returns a list of the complete lines in it.
  push(chunk) {
    const text = this.leftover + chunk;
    const lines = text.split(/\r?\n/);
    this.leftover = lines.pop(); // the last piece has no newline yet
    return lines.map((l) => l.replace(/\r$/, ""));
  }
}

// Turn one reading into a beam for updateMap.
//   headingDeg: which way the scanner points when the servo is at 90 degrees
//               (0 = right on the map, 90 = up, 180 = left, 270 = down)
//   flipped:    true if the servo turns the other way (the map came out mirrored)
// Returns { angleRad, distanceM, hit }.
export function readingToBeam(reading, headingDeg, flipped = false) {
  // Servo angles go counterclockwise from the heading. 90 = straight ahead.
  const turn = (reading.angleDeg - 90) * (flipped ? -1 : 1);
  const angleRad = ((headingDeg + turn) * Math.PI) / 180;
  const distanceM = reading.distanceMm / 1000;

  // A hit is a valid reading closer than 4 m. Anything else is a miss:
  // the beam found nothing, so the cells are free up to the max range.
  const hit = reading.ok && distanceM > 0 && distanceM < SENSOR_MAX_M;
  return { angleRad, distanceM: hit ? distanceM : SENSOR_MAX_M, hit };
}

// Where a hit landed on the map, in metres (for drawing the radar dots).
export function beamEnd(x, y, beam) {
  return { x: x + beam.distanceM * Math.cos(beam.angleRad), y: y + beam.distanceM * Math.sin(beam.angleRad) };
}

// Draw the map as pixels (top row first), `scale` pixels per cell.
// Gray = unknown, white = free, black = occupied. Used for "Download map as PNG".
export function mapToRgba(map, scale) {
  const w = map.nx * scale, h = map.ny * scale;
  const rgba = new Uint8ClampedArray(w * h * 4);
  for (let py = 0; py < h; py++) {
    const r = map.ny - 1 - Math.floor(py / scale); // image top = highest row
    for (let px = 0; px < w; px++) {
      const v = map.data[r * map.nx + Math.floor(px / scale)];
      const grey = isFree(v) ? 255 : isOccupied(v) ? 0 : 128;
      const i = (py * w + px) * 4;
      rgba[i] = rgba[i + 1] = rgba[i + 2] = grey;
      rgba[i + 3] = 255;
    }
  }
  return { rgba, width: w, height: h };
}
