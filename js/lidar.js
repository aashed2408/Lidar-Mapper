// The fake LiDAR: it shoots laser beams all around the robot and measures distances.
import { gaussian } from "./rng.js";

export const NUM_BEAMS = 180;    // one beam every 2 degrees, all the way around
export const ROBOT_RADIUS = 0.15; // metres

// Do one 360-degree scan from position (x, y) in the floor plan `world`.
//   maxRange: how far the laser reaches (metres)
//   noiseSigma: size of the random error added to hits (metres)
//   rand: a seeded random number function (from rng.js)
// Returns { angles, distances, hits }, one value per beam.
export function scan(world, x, y, maxRange, noiseSigma, rand) {
  const { nx, ny, cell, solid } = world;
  const step = cell / 2;                        // walk along each beam in half-cell steps
  const nSteps = Math.round(maxRange / step);
  const angles = new Float64Array(NUM_BEAMS);
  const distances = new Float64Array(NUM_BEAMS);
  const hits = new Uint8Array(NUM_BEAMS);

  for (let b = 0; b < NUM_BEAMS; b++) {
    const angle = (b * 2 * Math.PI) / NUM_BEAMS;
    const dx = Math.cos(angle), dy = Math.sin(angle);
    angles[b] = angle;
    distances[b] = maxRange;                    // a "miss" unless we find something

    for (let k = 1; k <= nSteps; k++) {
      const d = k * step;
      const c = Math.floor((x + dx * d) / cell);
      const r = Math.floor((y + dy * d) / cell);
      // Outside the plan counts as solid too.
      if (c < 0 || c >= nx || r < 0 || r >= ny || solid[r * nx + c]) {
        distances[b] = d;
        hits[b] = 1;
        break;
      }
    }

    // Random error, added to hits only. Keep the distance between 0.01 m and max range.
    const noise = noiseSigma > 0 ? gaussian(rand, noiseSigma) : 0;
    if (hits[b]) distances[b] += noise;
    distances[b] = Math.min(maxRange, Math.max(0.01, distances[b]));
  }
  return { angles, distances, hits };
}
