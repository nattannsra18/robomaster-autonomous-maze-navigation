# Classwork 8 - SLAM / Explore the Unknown World

This branch adds a standalone unknown-world exploration mode without changing the original fixed-grid pickup/drop mission.

## Hardware wiring

| Sensor | CAN hub | Port | Role |
|---|---:|---:|---|
| Front-left digital IR | 2 (left) | 1 | collision/safety guard |
| Left Sharp | 2 (left) | 2 | left-wall range + mapping |
| Front-right digital IR | 1 (right) | 1 | collision/safety guard |
| Right Sharp | 1 (right) | 2 | right-wall range + mapping |
| Front ToF | gimbal | RoboMaster distance sensor | front range + mapping |

The robot is switched to `CHASSIS_LEAD` mode so the gimbal follows the chassis, then the gimbal is recentered. This keeps the ToF aligned with chassis-forward while the chassis turns.

## What it does

- Starts with an empty occupancy grid; no maze walls are preloaded.
- Uses `sub_position` for x/y odometry.
- Uses `sub_attitude` plus the existing closed-loop 90-degree turn controller.
- Reuses the existing junction/Trémaux-style explorer to visit unexplored exits.
- Projects front ToF, left Sharp and right Sharp measurements into the map.
- Uses the two front digital IR sensors as collision guards only.
- Records start pose, end pose, sensor/pose log, exploration events and trajectory.
- Exports a map for Ground Truth comparison.

This is odometry-assisted occupancy-grid mapping for the classwork. It does not claim scan-matching or probabilistic loop-closure SLAM.

## Run

```powershell
python classwork8_main.py
```

Outputs:

```text
classwork8_output/run_YYYYMMDD_HHMMSS/
├── map.csv
├── map.svg
├── trajectory.csv
├── sensor_and_pose_log.csv
├── exploration_log.csv
└── summary.json
```

Map encoding:

- `-1` = UNKNOWN
- `0` = FREE
- `100` = OCCUPIED/WALL

## Accuracy and Coverage

After aligning/cropping the generated map to the same evaluation grid as the Ground Truth:

```powershell
python -m classwork8.evaluate classwork8_output\run_...\map.csv ground_truth.csv
```

The evaluator prints:

```text
Map Accuracy = correct cells / total cells * 100
Coverage = explored cells / total cells * 100
```

The 8 m × 8 m software canvas is only an initially unknown workspace, not prior maze knowledge. Therefore its raw working-canvas coverage is not the final assignment Coverage unless the Ground Truth evaluation area is also exactly that canvas.

## Before the final accuracy run

Measure the physical lens offsets from the chassis centre and update `Classwork8Config`:

- `tof_forward_offset_m`
- `sharp_lateral_offset_m`

The defaults are only integration-test approximations.

Also verify the Sharp ADC-to-cm calibration against the actual wall surface.

## First physical test sequence

1. Lift the drive wheels or use a clear area and confirm all sensor values.
2. Test left/right IR independently.
3. Test each Sharp at 10, 20, 30, 40 and 50 cm.
4. Confirm the gimbal follows chassis rotation and ToF remains forward.
5. Test a simple straight corridor, then an L-shaped corridor.
6. Only after those pass, test the full unknown maze.
