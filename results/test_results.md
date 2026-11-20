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

## Test 6: Real sensor: reading the ESP32's lines

- PASS: '90,1234,1' is a valid reading
- PASS: '178,0,0' is a reading with ok = 0
- PASS: 'SWEEP_END' ends a sweep
- PASS: an empty line is ignored
- PASS: the wiring warning is a message
- PASS: broken lines are messages, not readings
- PASS: chunks are joined back into whole lines
- PASS: 90 degrees points the way the scanner faces
- PASS: 180 degrees is 90 degrees to the left
- PASS: flipped: 180 degrees is 90 degrees to the right
- PASS: turned 180: 90 degrees points down
- PASS: ok = 0 is a miss out to 4 m
- PASS: 4 m or more is a miss
- PASS: the room map is 8 m x 8 m (160 x 160 cells)
- PASS: 91 readings and 1 sweep end were read
- PASS: the wall in front (y = 5.5 m) is occupied
- PASS: the wall on the left (x = 2.5 m) is occupied
- PASS: the wall on the right (x = 5.5 m) is occupied
- PASS: the floor in front of the scanner is free
- PASS: behind the scanner is still unknown
- PASS: PNG colours: white free, black wall, gray unknown

All checks passed.
