# Classwork 8 - ToF-Only Explore the Unknown World

This branch adds a standalone unknown-world exploration mode without changing the original fixed-grid pickup/drop mission.

## Hardware used

Only these signals are used by Classwork 8:

| Source | Purpose |
|---|---|
| Front ToF on gimbal | obstacle distance + occupancy-grid mapping |
| RoboMaster odometry (`sub_position`) | x/y position estimate |
| RoboMaster attitude (`sub_attitude`) | yaw / cardinal heading control |

IR, Sharp sensors and Sensor Adapter/CAN hubs are not used by the Classwork 8 runtime.

The robot is switched to `CHASSIS_LEAD`, then the gimbal is recentered. Therefore the ToF stays aligned with chassis-forward while the chassis rotates.

## Exploration method

The robot has only one forward range sensor, so it actively scans the world:

1. Stop.
2. Rotate the chassis to four cardinal headings.
3. Measure ToF in each direction.
4. Project the four rays into an occupancy grid.
5. Choose an open direction leading to an unvisited logical node.
6. Move forward one short step using yaw hold.
7. Repeat.
8. When no new direction exists, backtrack using DFS.
9. Finish when the reachable DFS search is exhausted or a safety limit is reached.

Default exploration step is 0.25 m and default forward speed is intentionally low because there are no side/corner sensors.

This is odometry-assisted occupancy-grid mapping with active ToF scanning. It does not claim scan matching or probabilistic loop-closure SLAM.

## Run

```powershell
git pull origin classwork8-slam-exploration
python classwork8_main.py
```

Press `Ctrl+C` at any time. The chassis is stopped and the run artifacts are exported.

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

After aligning/cropping the generated map to the same evaluation grid as the Ground Truth:

```powershell
python -m classwork8.evaluate classwork8_output\run_...\map.csv ground_truth.csv
```

The evaluator prints:

```text
Map Accuracy = correct cells / total cells * 100
Coverage = explored cells / total cells * 100
```

The 8 m × 8 m software canvas is only an initially unknown workspace, not prior maze knowledge. Raw working-canvas coverage is therefore not the final assignment Coverage unless Ground Truth uses the same area.

## Before the final accuracy run

Measure the physical ToF lens offset from the chassis centre and update:

```python
tof_forward_offset_m
```

The default value is only an integration-test approximation.

## Recommended physical test sequence

1. Run the stationary ToF/pose test first.
2. Confirm ToF changes at known distances such as 20, 50 and 100 cm.
3. Confirm gimbal stays chassis-forward while the chassis turns.
4. Run ToF-only exploration in a large clear area.
5. Test a straight corridor.
6. Test an L-shaped corridor.
7. Only then test the full unknown maze.

Because there are no side sensors, use low speed and keep enough corridor width for odometry drift.
