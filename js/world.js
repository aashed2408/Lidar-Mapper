// Floor plans: the "real world" the robot explores.
//
// A floor plan (a "world") is a grid of cells. Each cell is solid (1) or open floor (0).
// Cells are stored row by row in one long array: index = row * nx + col.
// Row 0 is at the BOTTOM of the plan (y = 0), like a maths graph.

export const DEFAULT_CELL = 0.05; // 5 cm cells
export const MAX_CELLS = 400;     // the long side of the grid is at most 400 cells

// Every solid rectangle of the example house as [x0, y0, x1, y1] in metres.
const EXAMPLE_RECTS = [
  // Outer walls
  [0, 0, 10, 0.1], [0, 7.9, 10, 8], [0, 0, 0.1, 8], [9.9, 0, 10, 8],
  // Middle wall with 2 doorways
  [5, 0, 5.1, 1.5], [5, 2.5, 5.1, 5.5], [5, 6.5, 5.1, 8],
  // Left-side wall with 1 doorway
  [0, 4, 2, 4.1], [3, 4, 5, 4.1],
  // Right-side wall with 1 doorway
  [5, 3, 7, 3.1], [8, 3, 10, 3.1],
  // Furniture: couch, table, bed
  [1, 1, 2.2, 1.8], [2.5, 5.5, 3.5, 7], [7.5, 5, 9, 6],
];

// Make the built-in example house (10 m x 8 m, 200 x 160 cells).
export function makeExampleHouse() {
  const cell = DEFAULT_CELL;
  const nx = 200, ny = 160;
  const solid = new Uint8Array(nx * ny);
  for (const [x0, y0, x1, y1] of EXAMPLE_RECTS) {
    // Turn metres into cell numbers. The end number is not included.
    const c0 = Math.round(x0 / cell), c1 = Math.round(x1 / cell);
    const r0 = Math.round(y0 / cell), r1 = Math.round(y1 / cell);
    for (let r = r0; r < r1; r++) {
      for (let c = c0; c < c1; c++) solid[r * nx + c] = 1;
    }
  }
  return {
    nx, ny, cell, solid,
    start: { x: 1.0, y: 3.0 },
    rooms: [
      { name: "Living room", x: 2.5, y: 2.5 },
      { name: "Kitchen", x: 2.5, y: 7.5 },
      { name: "Office", x: 7.5, y: 1.5 },
      { name: "Bedroom", x: 7.5, y: 7.5 },
    ],
  };
}

// Turn an image's pixels into a floor plan grid.
//   rgba: the pixels, 4 numbers each (red, green, blue, alpha), top row first
//   imgWidth, imgHeight: image size in pixels
//   realWidthMetres: how wide the plan is in real life
export function imageToGrid(rgba, imgWidth, imgHeight, realWidthMetres) {
  const realHeight = realWidthMetres * imgHeight / imgWidth;

  // Use 5 cm cells, unless that makes the grid too big. Then use bigger cells.
  let cell = DEFAULT_CELL;
  if (Math.max(realWidthMetres, realHeight) / cell > MAX_CELLS) {
    cell = Math.max(realWidthMetres, realHeight) / MAX_CELLS;
  }
  const nx = Math.max(1, Math.ceil(realWidthMetres / cell - 1e-9));
  const ny = Math.max(1, Math.ceil(realHeight / cell - 1e-9));
  const pxPerCell = imgWidth / realWidthMetres * cell; // how many pixels fit in one cell

  // Is the pixel at (px, py) dark? (py counts from the top of the image.)
  function isDark(px, py) {
    const i = (py * imgWidth + px) * 4;
    const a = rgba[i + 3] / 255;
    // Mix the colour with white by its transparency, so see-through pixels count as white.
    const brightness = ((rgba[i] + rgba[i + 1] + rgba[i + 2]) / 3) * a + 255 * (1 - a);
    return brightness < 128;
  }

  // The range of pixels [first, last) that lie inside cell number k.
  function pixelRange(k, size) {
    const first = Math.min(size - 1, Math.floor(k * pxPerCell + 1e-6));
    const last = Math.min(size, Math.max(first + 1, Math.ceil((k + 1) * pxPerCell - 1e-6)));
    return [first, last];
  }

  const solid = new Uint8Array(nx * ny);
  for (let r = 0; r < ny; r++) {
    // Row r counts from the bottom, but image rows count from the top.
    const [b0, b1] = pixelRange(r, imgHeight);
    for (let c = 0; c < nx; c++) {
      const [x0, x1] = pixelRange(c, imgWidth);
      // A cell is solid if ANY pixel inside it is dark, so thin lines don't disappear.
      let dark = false;
      for (let pb = b0; pb < b1 && !dark; pb++) {
        const py = imgHeight - 1 - pb;
        for (let px = x0; px < x1; px++) {
          if (isDark(px, py)) { dark = true; break; }
        }
      }
      if (dark) solid[r * nx + c] = 1;
    }
  }

  // A 1-cell solid border, so the robot can't leave the picture.
  for (let c = 0; c < nx; c++) { solid[c] = 1; solid[(ny - 1) * nx + c] = 1; }
  for (let r = 0; r < ny; r++) { solid[r * nx] = 1; solid[r * nx + nx - 1] = 1; }

  return { nx, ny, cell, solid, start: null, rooms: [] };
}

// Draw a floor plan as black-on-white pixels (top row first), `scale` pixels per cell.
// Used for "Download example floor plan".
export function worldToRgba(world, scale) {
  const w = world.nx * scale, h = world.ny * scale;
  const rgba = new Uint8ClampedArray(w * h * 4);
  for (let py = 0; py < h; py++) {
    const r = world.ny - 1 - Math.floor(py / scale); // image top = highest row
    for (let px = 0; px < w; px++) {
      const c = Math.floor(px / scale);
      const v = world.solid[r * world.nx + c] ? 0 : 255;
      const i = (py * w + px) * 4;
      rgba[i] = rgba[i + 1] = rgba[i + 2] = v;
      rgba[i + 3] = 255;
    }
  }
  return { rgba, width: w, height: h };
}

// Which cell is the point (x, y) metres in? Returns { r, c }.
export function toCell(world, x, y) {
  return { r: Math.floor(y / world.cell), c: Math.floor(x / world.cell) };
}

// The centre of cell (r, c) in metres.
export function cellCenter(world, r, c) {
  return { x: (c + 0.5) * world.cell, y: (r + 0.5) * world.cell };
}
