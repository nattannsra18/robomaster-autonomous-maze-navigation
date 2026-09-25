# Classwork 8 - ToF + Camera Explore the Unknown World

This branch provides an unknown-world exploration mode for the RoboMaster EP.

## Final hardware/runtime design

Classwork 8 uses:

| Source | Purpose |
|---|---|
| Front ToF on gimbal | authoritative obstacle distance + occupancy-grid mapping |
| RoboMaster camera on gimbal | visual corridor centering while moving |
| RoboMaster odometry (`sub_position`) | x/y localization and metric cell targets |
| RoboMaster attitude (`sub_attitude`) | chassis yaw hold |
| Gimbal angle (`sub_angle`) | verify actual ToF/camera scan direction |

IR, Sharp sensors and Sensor Adapter/CAN hubs are not used.

The camera is **assistive**, not authoritative. A camera frame never declares a
wall/open edge and never replaces the ToF emergency stop. When corridor lines
cannot be detected reliably, camera correction becomes zero and navigation
falls back to ToF + odometry.

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

The main GUI is an auto-fit **discovered 60 cm cell map**, visually matching
the fixed-grid mission GUI style but without preloading field dimensions.

It shows:

- thin gray borders for discovered 60 cm cells
- thick black lines for ToF-confirmed walls
- a light-blue logical DFS cell-to-cell path
- a solid blue odometry trajectory that grows continuously while the robot moves
- green `S` at the unknown-world local origin `(0,0)`
- red `R` at the robot's current continuous position
- orange arrow for the live Gimbal/ToF direction
- cell coordinates, ToF, Gimbal yaw, moves and occupancy coverage
- STOP & SAVE button

The display uses:

```text
mission-start FRONT = screen up
mission-start RIGHT = screen right
```

It does **not** know the real field width/height or which physical corner the
robot started from.  The displayed grid automatically expands and recentres as
new neighboring cells are discovered.

The separate 5 cm occupancy grid still runs in the background and is exported
as `map.csv` / `map.svg` for accuracy and coverage analysis.

Terminal-only mode:

```powershell
python classwork8_main.py --no-gui
```

Disable camera assistance at any time:

```powershell
python classwork8_main.py --no-vision
```

or:

```powershell
python classwork8_main.py --no-gui --no-vision
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


## Camera-assisted corridor centering

RoboMaster's camera moves with the same gimbal as the ToF. During a cell move,
the gimbal points along the travel direction, so the image is also looking in
the direction of motion.

The vision pipeline uses:

```text
360p H.264 camera stream
  -> OpenCV/FFmpeg direct TCP decode
  -> lower-image ROI
  -> grayscale + blur
  -> Canny edges
  -> Hough line segments
  -> left/right corridor boundary candidates
  -> corridor centre error
  -> confidence gate
  -> small lateral velocity correction
```

Camera steering is **disabled by default** (`vision_steering_enabled=False`).
The camera still runs in MONITOR ONLY mode, shows candidate boundaries and logs
confidence. This is intentional: foam walls, distant inner-wall edges and floor
tile seams have produced false corridor centres in real tests. Do not enable
steering before calibrating on captured raw frames and verifying the correction
sign in a controlled open test.

The analyser rejects short distant segments, excessive extrapolation to the
image bottom and incompatible left/right boundary candidates. Even after
calibration, ToF remains the sole authority for obstacle distance and stops.
Camera guidance is not a replacement for side-distance sensors.

The video backend intentionally avoids DJI's optional native
`libmedia_codec` on Windows: the SDK sends the stream-control commands and
OpenCV/FFmpeg directly decodes RoboMaster's TCP H.264 stream on port 40921.

### Stationary camera test

Before allowing camera steering, run:

```powershell
python classwork8_camera_test.py
```

The chassis will not move. A window shows the camera overlay:

- blue/orange = side-boundary candidates
- green vertical line = estimated corridor centre
- `confidence` = heuristic quality indicator, not a calibrated collision-risk probability
- `MONITOR` = the visual estimate cannot command chassis motion
- Press `S` to save raw and annotated PNG frames in
  `classwork8_output/camera_samples/` (press `Q` or Esc to quit).

Capture centre, left-offset and right-offset views before calibrating. A green
line over a foam wall is an invalid centre estimate even if confidence is high.

Put the stationary robot in the real foam-wall corridor and move it manually
left/right. The sign of the displayed error should change consistently and the
green centre estimate should follow the visual corridor centre.

If the stream cannot open or the room produces unreliable lines, keep the
assignment operational with `--no-vision`.

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


## V03: configurable GUI + nearest-frontier exploration

Run the current field-tuning version with:

\`\`\`powershell
python -u classwork8_slam_03.py
\`\`\`

Before the robot connects, V03 opens a configuration window with tabs for
motion, ToF/safety, mapping and vision. Important 60 cm calibration parameters
(\`step_tolerance_m\`, \`odom_scale_x\`, \`odom_scale_y\`, travel speed and
heading gains) can be changed without editing Python source.

V03 uses **nearest-frontier BFS**, not the V01/V02 DFS parent stack. A frontier
is a visited cell with a confirmed OPEN edge into an unvisited cell. The
planner:

1. enters an adjacent unvisited open cell immediately when one exists;
2. otherwise searches confirmed-open visited cells with BFS;
3. routes to the nearest reachable frontier;
4. prefers frontiers with more unvisited open edges when path lengths tie;
5. replans after a confirmed blocked edge.

The realtime GUI shows the selected frontier and a purple dashed planned route.
Returning toward the start is therefore only expected when the nearest/only
remaining frontier is actually in that direction.

V03 field defaults use a neutral odometry scale (1.00/1.00) and a 5 mm logical
stop tolerance. Calibrate X and Y separately with a tape measure. If one
commanded 60 cm move physically travels \`D\` cm, a useful next estimate is:

\`\`\`text
new_scale = old_scale * D / 60
\`\`\`

For example, if scale 1.00 moves only 54 cm, try about 0.90.


## V04: closed-maze auto stop + GUI map export

Run:

\`\`\`powershell
python -u classwork8_slam_04.py
\`\`\`

V04 keeps the V03 nearest-frontier BFS planner and adds a second completion
criterion for the closed rectangular classwork arena.  A run may finish when:

- every logical cell inside the currently discovered bounding rectangle has
  actually been visited; and
- each of the four outer sides has enough WALL evidence.

The default perimeter ratio is 0.70.  This tolerates an occasional ToF miss on
a low foam outer wall without chasing a phantom frontier outside the real
arena.  Disable **Closed-maze auto stop** in the configuration GUI for a
non-rectangular/open-ended environment.

The realtime GUI now has **SAVE GUI MAP NOW**.  When automatic export is
enabled, the final logical GUI map is also written to the run directory as:

\`\`\`text
classwork8_output/run_<timestamp>/gui_map.png
\`\`\`

The original assignment outputs (\`map.csv\`, \`map.svg\`,
\`sensor_and_pose_log.csv\`, \`exploration_log.csv\`, \`trajectory.csv\`, and
\`summary.json\`) are still exported normally.
