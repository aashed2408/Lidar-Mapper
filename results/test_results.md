# Test results

## Test 1: Example house, noise 2 cm, seed 42

| Measure | Value |
|---|---|
| Scans | 87 |
| Metres driven | 41.7 |
| Coverage | 100.0% |
| Accuracy | 99.3% |
| Walls found | 98.7% |
| Bumps | 0 |
| Status | Done: map complete |

- PASS: coverage at least 95%
- PASS: accuracy at least 96%
- PASS: 0 bumps
- PASS: finished with 'Done: map complete'

## Test 2: Noise test (seed 42)

| Noise (cm) | Scans | Metres | Coverage | Accuracy | Walls found | Bumps |
|---|---|---|---|---|---|---|
| 0 | 87 | 41.8 | 100.0% | 100.0% | 99.5% | 0 |
| 2 | 87 | 41.7 | 100.0% | 99.3% | 98.7% | 0 |
| 5 | 88 | 42.2 | 100.0% | 98.6% | 92.9% | 0 |
| 10 | 118 | 52.4 | 100.0% | 97.1% | 60.9% | 9 |

## Test 3: A* from (1.0, 3.0) to the Bedroom (9.0, 7.2)

- PASS: a path was found
Path: 9.74 m (straight line: 9.04 m). Closest to a real wall: 21 cm.

- PASS: path is 9.5-10.5 m
- PASS: path stays at least 18 cm from walls
- PASS: path never goes through unknown cells
- PASS: path never enters a solid cell

## Test 4: imageToGrid

Image 2000 x 1600 px, 50 m wide -> grid 400 x 320 cells, cell size 12.5 cm

- PASS: long side is at most 400 cells
- PASS: the 1-pixel line is still solid after shrinking
- PASS: the 1-cell border is solid
- PASS: open white areas stay open
- PASS: transparent pixels count as white

## Test 5: Example floor plan PNG -> imageToGrid matches the built-in example

Image 800 x 640 px -> grid 200 x 160 cells, 0 cells different

- PASS: same grid size and cell size
- PASS: every cell matches

All checks passed.
