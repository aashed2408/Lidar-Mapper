// Seeded random numbers, so every run with the same seed gives the same results.

// mulberry32: a tiny random number generator. Returns a function that gives
// a new number between 0 and 1 each time you call it.
export function mulberry32(seed) {
  let a = seed >>> 0;
  return function () {
    a = (a + 0x6d2b79f5) >>> 0;
    let t = a;
    t = Math.imul(t ^ (t >>> 15), t | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}

// Box-Muller: turns two even random numbers into one bell-curve (Gaussian) number
// with average 0 and spread `sigma`.
export function gaussian(rand, sigma) {
  const u1 = 1 - rand(); // 1 - rand() is never 0, so the log below is safe
  const u2 = rand();
  return sigma * Math.sqrt(-2 * Math.log(u1)) * Math.cos(2 * Math.PI * u2);
}
