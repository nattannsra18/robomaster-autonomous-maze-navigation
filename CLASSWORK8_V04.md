# Classwork 8 V04: choose the workflow

Use the same virtual environment and RoboMaster Wi-Fi connection for both commands.
Run these commands from the repository root.

## SLAM only (no Ground Truth)

```powershell
python -u classwork8_slam_04.py
```

The configuration window opens directly. Set motion and sensor values, then
click **Apply & Connect**. The robot explores an unknown maze, and **STOP &
SAVE** exports the discovered map, trajectory, sensor log, exploration log,
GUI image, and `summary.json` under `classwork8_output/run_YYYYMMDD_HHMMSS/`.
There is no Ground Truth editor or automatic accuracy evaluation. The working
canvas coverage in `summary.json` is diagnostic, not the assignment's field
coverage score.

## Optional Ground Truth and automatic evaluation

```powershell
python -u classwork8_slam_04_ground_truth.py
```

This opens the Ground Truth editor in the configuration window. Save a
measured reference layout and start SLAM. **STOP & SAVE** also exports
`evaluation/` and computes accuracy and field coverage if the reference is
valid and correctly aligned. The robot's planner never receives this map.

For either command, `--no-gui` starts with defaults and bypasses the
configuration window. The SLAM-only command keeps automatic evaluation off
in this mode as well.
