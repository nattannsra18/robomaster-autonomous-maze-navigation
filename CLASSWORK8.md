# Classwork 8 - ToF-Only Explore the Unknown World

This branch provides an unknown-world exploration mode for the RoboMaster EP.

## Final hardware/runtime design

Classwork 8 now uses only:

| Source | Purpose |
|---|---|
| Front ToF on gimbal | obstacle distance + occupancy-grid mapping |
| RoboMaster odometry (`sub_position`) | x/y localization |
| RoboMaster attitude (`sub_attitude`) | chassis yaw monitoring |
| Gimbal angle (`sub_angle`) | verify actual ToF scan direction |

IR, Sharp sensors and Sensor Adapter/CAN hubs are not used.

## Physical maze cell

The real maze cell is:

```text
60 cm x 60 cm
```

The exploration planner therefore uses:

```text
1 logical move = 1 full cell = 0.60 m
```

The occupancy map remains finer:

```text
map resolution = 0.05 m = 5 cm per occupancy cell
```

The 5 cm occupancy pixels and the 60 cm physical maze cells are different concepts.

## No 90-degree chassis scan turns

The robot runs in RoboMaster `FREE` mode so gimbal yaw and chassis yaw are independent.

At each maze-cell centre the chassis stays in the same general orientation while the gimbal scans:

```text
            FRONT
              ^
              |
LEFT  <---- ROBOT ----> RIGHT
              |
              v
             BACK
```

The gimbal scan uses closed-loop `sub_angle()` feedback. A scan is accepted only after the measured gimbal yaw reaches the requested direction within tolerance.

## Moving between cells without turning the chassis

The RoboMaster EP mecanum chassis is used directly:

```text
FRONT -> drive forward
RIGHT -> strafe right
BACK  -> reverse
LEFT  -> strafe left
```

Before and during a one-cell move, the gimbal points the ToF in the travel direction.

Therefore a RIGHT move looks conceptually like:

```text
        ToF --->
      [ GIMBAL ]

      [ ROBOT ]  ==================>  next cell
                  strafe 60 cm
```

The chassis does not need to rotate 90 degrees first.

A small yaw-hold correction may still be applied during translation to prevent chassis drift. It is not a scan turn.

## Exploration algorithm

1. Start at logical cell (0, 0).
2. Keep the chassis stopped.
3. Scan LEFT, FRONT, RIGHT and BACK using gimbal yaw only.
4. Project each ToF ray into the occupancy grid.
5. Select an open neighboring 60 cm cell that has not been visited.
6. Point the gimbal/ToF toward that travel direction.
7. Move one full cell using mecanum translation.
8. Read ToF continuously while moving.
9. Stop immediately if the travel direction becomes blocked.
10. Repeat.
11. If there are no new neighbors, DFS backtracks to the previous cell.
12. Finish when all reachable logical cells are exhausted or STOP is requested.

## Realtime GUI

Normal run:

```powershell
python classwork8_main.py
```

This opens a Tkinter GUI showing:

- occupancy map in realtime
- 60 cm physical-cell grid
- robot position
- trajectory
- current logical cell
- current Gimbal/ToF direction
- current ToF distance
- move count
- map coverage
- mission status
- STOP & SAVE button

Terminal-only mode:

```powershell
python classwork8_main.py --no-gui
```

## IMPORTANT: verify gimbal directions before the first moving test

Run:

```powershell
python classwork8_gimbal_test.py
```

The chassis does not move.

Visually confirm this exact sequence:

```text
FRONT -> physical front
RIGHT -> physical robot right
BACK  -> physical rear
LEFT  -> physical robot left
FRONT -> physical front
```

Do not run the full exploration until those four labels match the actual ToF direction.

## Recommended first full test

Use a wide open test area first.

1. Pull the latest branch.
2. Run `classwork8_gimbal_test.py`.
3. Confirm all four Gimbal directions.
4. Place the robot near the centre of a 60 cm reference grid.
5. Run `classwork8_main.py`.
6. Watch the realtime GUI.
7. Be ready to press **STOP & SAVE**.
8. Test one forward cell first.
9. Then test right/left strafe.
10. Only then use the real maze.

## Outputs

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

After aligning/cropping the generated map to the same evaluation grid as Ground Truth:

```powershell
python -m classwork8.evaluate classwork8_output\run_...\map.csv ground_truth.csv
```

The evaluator prints:

```text
Map Accuracy = correct cells / total cells * 100
Coverage = explored cells / total cells * 100
```

The 8 m x 8 m software canvas is only an initially unknown working canvas; it is not prior maze knowledge.
