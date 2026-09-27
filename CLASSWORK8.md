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

## Accuracy and Coverage (assignment item 4)

The evaluator compares the saved robot occupancy map against a separately drawn
**Ground Truth** map of the real maze. This is an offline step; no robot or
RoboMaster SDK connection is required.

- Map encoding: \`-1\` = UNKNOWN, \`0\` = FREE, \`100\` = WALL.
- Ground Truth must use only \`0\` or \`100\` within the evaluated area.
- The two CSV files must represent **the same physical resolution and orientation**.
  The default occupancy \`map.csv\` is 5 cm per cell, NOT a 60 cm-per-cell
  logical maze diagram. Rasterize a hand-drawn Ground Truth to 5 cm per
  cell (including wall thickness), or compare two equally sized fine grids.
- Rows in \`map.csv\` are top-to-bottom (positive map Y first), and columns
  are left-to-right (negative to positive map X). Check orientation against
  the actual start direction before choosing the crop.
- The initial 8 m x 8 m working canvas is larger than many real fields.
  Do **not** use its full area as the assignment denominator unless the
  Ground Truth covers the same full area.
- Do not generate the Ground Truth from the robot's predicted map; measure
  the physical maze or use the lecturer's reference plan independently.

If the two maps already have identical dimensions and orientation:

\`\`\`powershell
python -m classwork8.evaluate "classwork8_output\run_YYYYMMDD_HHMMSS\map.csv" ground_truth.csv
\`\`\`

If Ground Truth covers only a rectangle within the exported canvas, provide
its top-left **0-based row/column in the exported map.csv**, after any
explicit rotation (do not blindly centre-crop):

\`\`\`powershell
python -m classwork8.evaluate "classwork8_output\run_YYYYMMDD_HHMMSS\map.csv" ground_truth.csv --top 38 --left 44
\`\`\`

The numbers above are only command syntax examples; determine your real
\`--top\`/\`--left\` from the physical frame. If the map orientation is known
to differ, \`--rotate 90\`, \`180\` or \`270\` rotates the predicted map
clockwise *before* cropping.

Outputs in \`run_...\evaluation\`:

- \`evaluation.txt\`: assignment equations and numeric counts/percentages.
- \`evaluation.json\`: machine-readable results and the chosen alignment.
- \`aligned_map.csv\`: exactly the evaluated robot-map region.
- \`comparison.svg\`: robot map, Ground Truth and a third agreement map
  (green correct, yellow unexplored, red wrong prediction).

Assignment formulas:

\`\`\`text
Map Accuracy = number of correctly classified cells / all GT cells * 100
Coverage     = number of non-UNKNOWN predicted cells / all GT cells * 100
\`\`\`

An UNKNOWN robot cell counts as **incorrect** for assignment accuracy, and
**unexplored** for coverage. The additional observed-cell accuracy is for
diagnostics only and is *not* substituted for the assignment score.
The mission's existing \`working_canvas_coverage_percent\` in \`summary.json\`
is not the assignment Coverage; use \`evaluation.json\` after matching the
physical field's evaluation area.

Run the evaluator's offline regression tests:

\`\`\`powershell
python -m unittest tests.test_classwork8_evaluation -v
\`\`\`

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


## Final Assignment Round 1 V05 - ToF + camera baseline

Entry point:

\`\`\`powershell
python -u final_round1_tof_camera_01.py
\`\`\`

This version intentionally starts with only the gimbal ToF, chassis odometry /
attitude, and the RoboMaster camera.  It keeps the proven V04 nearest-frontier
mapping/motion controller and adds camera target surveying during the same
gimbal scan.

The camera is opened exactly once by \`classwork8/camera_service.py\`.  The
target detector consumes copies of the latest frame, so later vision modules
must reuse this service rather than opening a second DJI video stream.

Target processing is OpenCV-only:

\`\`\`text
BGR
 -> Lab-L CLAHE lighting normalization
 -> HSV broad candidate masks
 -> morphology
 -> contours
 -> color + shape confidence
 -> temporal verification
 -> TargetRegistry
\`\`\`

Targets are surveyed only when the ToF direction is classified as a nearby
wall.  This prevents a distant sign seen through an open corridor from being
registered against the wrong approach cell and reduces camera-processing time.

Round 1 adds two navigation-ready exports:

\`\`\`text
topology.json
targets.json
\`\`\`

\`topology.json\` stores the logical visited cells, OPEN/WALL edges, traversed
edges, start-cell scan signature and final cell.  \`targets.json\` stores every
verified color/shape target, its confidence, observing cell, gimbal direction,
ToF range and an estimated metric target position.

The V05 GUI shows the shared camera target overlay and draws detected targets
as colored Txx markers on the map.  The final \`gui_map.png\` includes those
markers as well.

The V05 baseline does not aim or fire the blaster.  It is the Round-1 mapping
and target-database foundation for the later Round-2 route planner.


### V05 camera tuning from the 27 Sep real foam-wall sample

The real 640x360 sample contained seven physical signs. The original detector
reported eight because a yellow floor object was incorrectly accepted as a
YELLOW RECTANGLE. The rightmost green rectangle was also called a square.

The current V05 detector now:
- limits the target band to configurable fractions of image height
  (\`target_roi_top_ratio=0.18\`, \`target_roi_bottom_ratio=0.82\` by default);
- rejects clipped objects touching ROI/image borders;
- tightens square aspect ratio to 0.85-1.16 and keeps elongated signs as
  rectangles;
- rejects tiny contours below 300 px by default;
- verifies 4 of 6 distinct fresh camera frames, never counting one cached
  frame repeatedly;
- keeps separate adjacent same-color/same-shape targets rather than merging
  them solely because the shared ToF ray gives the same distance;
- draws small numbered boxes to prevent camera labels overlapping.

The ROI must be tuned in the configuration GUI if the camera pitch, mounting,
target height, or maze geometry changes; it is not a universal field constant.

**Offline saved-image check (does not connect to the robot):**

\`\`\`powershell
python -u final_target_image_test.py
\`\`\`

The script automatically finds the newest
\`classwork8_output/target_camera_samples/target_raw_*.png\`, prints each
detection's color, shape, confidence, bounding box and aspect ratio, and
writes a matching \`target_retuned_*.png\`. An explicit path is also accepted:

\`\`\`powershell
python -u final_target_image_test.py "C:\\path\\to\\target_raw_sample.png"
\`\`\`

**Offline regression tests:**

\`\`\`powershell
python -m unittest tests.test_final_target_detection -v
\`\`\`

The displayed confidence is a detection heuristic, not a calibrated
probability of correctness. Cross-cell target association still requires
camera geometry or additional validation; V05 deliberately does not merge
targets from different approach cells based on one ToF distance alone.


### V05 runtime GUI footer + horizontal gimbal pitch guard

The runtime GUI keeps **SAVE GUI MAP NOW (Ctrl+S)** and **STOP & SAVE**
visible in a fixed lower-right action area; long diagnostics/legend text is
scrollable. Closing the window still requests safe stop/export.

The ToF/camera gimbal is now controlled in both axes, not yaw alone:
\`gimbal.sub_angle()\` provides relative pitch/yaw feedback. The mission
defaults to horizontal \`gimbal_scan_pitch_deg=0.0\`. Both axes must remain
inside their configured tolerances before a scan is accepted. If pitch drifts
while a ToF range is sampled, the robot retries while stationary; if pitch
moves beyond the motion safety threshold during translation, chassis motion
stops before re-levelling. Failed orientation recovery aborts instead of
silently mapping an upward/downward ToF ray as a wall.

The config GUI exposes pitch target, correction gain, feedback/command sign,
pitch tolerance and movement-safety threshold. The live GUI displays the
actual relative pitch and yaw; a non-level warning is shown when pitch differs
by over 2 degrees.

**Test physical pitch/yaw without any chassis travel:**

\`\`\`powershell
python -u final_gimbal_pitch_test.py
\`\`\`

If the reported pitch error grows in the wrong direction during this stationary
test, stop with Ctrl+C and re-run with \`--pitch-sign -1\`. Change the GUI sign
only after confirming the hardware feedback response. If a gimbal cannot
maintain its commanded horizontal position, inspect mounting/cables and do not
start full maze exploration.

**Offline tests without RoboMaster:**

\`\`\`powershell
python -m unittest tests.test_final_gimbal_pitch_v05 -v
\`\`\`

### V05 revised pitch damping after stationary hardware log

The 27 Sep stationary test showed yaw reached every requested direction, but
pitch alternated between about -1.7 and +1.7 degrees. This was **not** a
passing demonstration of steady level pitch: the old +/-2 degree tolerance
accepted all these endpoints, and the 4 deg/s minimum correction could add
visible nodding. The updated scan pitch defaults are:

```text
Target pitch       0.0 deg
Pitch Kp           0.9
Minimum pitch rate 1.5 deg/s
Maximum pitch rate 10.0 deg/s
Pitch tolerance    0.8 deg
Unsafe pitch drift 6.0 deg (stop motion)
```

The test now logs `[SWEEP]` pitch min/max/peak error using actual angle
subscription samples during each yaw turn, in addition to `[TEST]` and
`[HOLD]` endpoints. It also checks that the pitch is close to the target
after the 0.30 s stationary hold. This is necessary because an endpoint can
appear level even when the camera nods substantially during the sweep.

Run the offline regression first, then the **stationary** hardware diagnostic:

```powershell
python -m unittest tests.test_final_gimbal_pitch_v05 -v
python -u final_gimbal_pitch_test.py
```

Do not flip the pitch drive sign merely because a small residual pitch error
remains. Reverse it only if feedback consistently moves away from the target
when an explicit pitch correction is commanded. If the software times out,
keep the chassis stopped and inspect the emitted pitch history / gimbal
mechanics. Optical level may require calibration because the SDK reports
relative gimbal angle, not the visual horizon of a particular camera mount.


### V05 yaw-only scan isolation after 24-degree transient pitch log

The subsequent real RoboMaster stationary test showed pitch reaching about +24.4
degrees during FRONT -> RIGHT and -23.7 degrees during RIGHT -> FRONT, while
settling to less than +/-0.8 degrees at the endpoints. Therefore the old
simultaneous pitch/yaw control, and/or a physical gimbal/yaw coupling, must be
isolated rather than only tightening endpoint tolerance.

The V05 scanner now uses **three sequential stages**:

1. While yaw is stopped, level pitch to the configured scan angle.
2. Turn yaw **with pitch_speed fixed at exactly zero** (default max yaw 40
   deg/s). Monitor the actual pitch subscriber during the whole sweep.
3. Stop yaw, then re-level pitch with yaw_speed=0. Check both axes after
   settling before accepting ToF or camera observations.

If pitch departs from the intended scan plane by more than
\`gimbal_yaw_pitch_guard_deg=6.0\` *during the yaw-only stage*, the scanner
stops and returns failure. This is intentional: a persistent excursion with
no concurrent pitch commands suggests hardware/firmware/cable coupling, not
merely the previous dual-axis software controller. Avoid a full maze run until
the stationary diagnostic demonstrates stable yaw-only operation. GUI shows
the configurable pitch tolerance rather than an outdated hardcoded +/-2 deg.

Run:

\`\`\`powershell
git pull origin classwork8-slam-exploration
python -m py_compile classwork8\tof_camera_round1_v05.py final_gimbal_pitch_test.py
python -m unittest tests.test_final_gimbal_pitch_v05 -v
python -u final_gimbal_pitch_test.py
\`\`\`

The stationary diagnostic prints the pitch min/max and peak error even if a
yaw-only drift guard stops the sequence; no chassis travel occurs. Do not
increase the guard threshold just to make a failed test pass.


### V05 low floor target survey and genuinely live GUI

The 27 Sep V05 field experiment completed a 3x3 logical map but exported
\`target_count=0\` despite physical signs on the floor. The previous camera
survey used pitch zero and only ran when horizontal ToF read <55 cm; the Tk
preview was an old frame copied into a map status snapshot.

**Updated flow at each stopped cell/direction:**

1. Level gimbal pitch at the normal horizontal mapping angle (default 0).
2. Record and validate the ToF ray for navigation/map evidence.
3. If target survey is enabled, change pitch with **yaw stopped** to the
   independently adjustable camera survey angle (default -10 deg).
4. Perform multi-frame color + shape verification; record verified targets.
5. Restore the horizontal ToF angle, checking actual pitch/yaw feedback.
6. Only then can another ToF scan or cell movement happen.

The V05 GUI shows **live annotated target detection** at a separate
\`target_preview_fps\` cadence (8 FPS by default). Target preview runs in its
own worker and never opens an extra video stream. Navigation map snapshots
no longer carry the unused 160x160 matrix or a stale image, and Tk redraws
the map only when new navigation data arrives.

Runtime controls include:
- camera observation pitch slider, -20 through +10 degrees;
- Apply camera pitch at next scan;
- Rescan current cell: safely finishes the active scan, then repeats it from
  the same logical cell before moving, using the currently selected pitch;
- Enlarge Live Camera;
- Save Raw + Detected Camera Frame (for diagnosing missing target candidates);
- Save GUI Map Now / Ctrl+S and Stop & Save remain fixed in the footer.

The GUI NEVER commands gimbal motors directly: a requested pitch is queued
and physically applied only by the explorer worker while chassis motion is
stopped. Negative pitch is intended to look lower, but check the actual
direction on the physical RoboMaster camera before starting the maze.

\`target_survey_open_directions=True\` enables looking for low signs even when
a horizontal ToF ray reports an open corridor. Such observations are exported
with \`status=NEEDS_RANGE_REVIEW\` and \`range_confirmed_wall=false\`, since the
camera alone does not prove the target's physical wall range. The GUI adds
a \`?\` suffix to these estimated target markers. A target with a confirmed
near-wall ToF ray retains \`status=DETECTED\`.

The survey log now records \`TARGET_SURVEY\` for each attempted camera
direction, including current candidates, verified target count, pitch and
whether a wall range was confirmed. This differentiates missed field-of-view
targets from temporal verification failures. Full-map autonomous target
navigation must not trust \`NEEDS_RANGE_REVIEW\` positions without another
observation.

**Pull and check offline syntax/tests:**

\`\`\`powershell
git pull origin classwork8-slam-exploration
python -m py_compile classwork8\live_survey.py classwork8\tof_camera_round1_v05.py classwork8\gui_v05.py classwork8\config.py classwork8\config_gui_v05.py classwork8\target_detection.py
python -m unittest tests.test_final_live_survey tests.test_final_target_detection -v
\`\`\`

**Small maze hardware test:** keep the original navigation settings, use a
camera pitch of -10 degrees initially, and test one floor sign. Open the
enlarged live camera window. If the sign is still near/below the ROI, adjust
the slider, apply, then press Rescan Current Cell. Review \`TARGET_SURVEY\`
events and \`targets.json\` before starting a full 7x7 run.

Full four-direction camera survey adds physical pitch movements and multiple
vision samples; measure its impact on the total 15-minute limit before the
exam and disable survey of open directions if the actual signs can be
reliably observed at nearby walls.


### V05: ground-level signs, live annotated preview, runtime ROI

The 27 Sep Round-1 run completed its 3x3 mapping mission but produced an empty
\`targets.json\`. The physical signs had been mounted near the floor. The
original ROI stopped at image-height ratio 0.82 and the normal horizontal
ToF ray could not by itself prove or reject a low sign.

The target-observation default now uses pitch -10 degrees **only at stopped
camera survey checkpoints**, and its ROI is 0.18..0.94 by default (previously
0.18..0.82). A broader ROI can also detect floor reflections; it must be
calibrated from the live preview on the real field, not treated as a guaranteed
improvement for every scene. The horizontal ToF scan and driving orientation
are restored before navigation proceeds. The mapping planner, motor control
and emergency stop thresholds are unchanged.

The GUI's target panel now includes:
- a live camera overlay with numbered contours plus a matching color/shape/
  confidence legend, rather than an occasional snapshot from map publication;
- a **Camera look-down pitch** slider, which queues a new pitch for the next
  stopped survey; it never directly moves the gimbal while driving;
- a **Live detection region bottom** slider (0.70-0.99), immediately updating
  both live preview and scan-time target detection without commanding motors;
- **RESCAN CURRENT CELL** to recheck the current cell with the updated pitch
  and ROI before the next navigation decision;
- a larger camera popup and **SAVE RAW + DETECTED CAMERA FRAME** button;
- independent preview refresh, actual processing FPS and preview-frame age.
  The map drawing is limited to ~4Hz so it does not block video redraws.

The GUI shows **LIVE CANDIDATES (unverified)** separately from **Targets: N**.
Only stationary survey observations that pass 4 of 6 *distinct new* frames
after the camera pitch has settled are added to \`targets.json\`.

Start at pitch -10 degrees and ROI bottom 0.94. Inspect the overlay; if a
floor-level sign is cropped by the horizontal ROI line, raise the bottom ROI
toward 0.96-0.99 or slightly adjust the look-down pitch, then press RESCAN.
If a floor object or reflection gets detected, narrow the ROI again and
capture raw/debug samples. Not all lighting or sign placement is solved by
one set of thresholds.

Offline tests (do not connect to RoboMaster):

\`\`\`powershell
python -m unittest tests.test_final_target_detection tests.test_final_live_survey -v
python -u final_target_image_test.py --roi-bottom 0.82
python -u final_target_image_test.py --roi-bottom 0.94
\`\`\`

The old seven-sign *wall* sample may acquire an extra floor-object candidate
under the broader floor-sign profile; compare both ROI outputs, rather than
interpreting the difference as a navigation failure. For a genuine floor-sign
test, capture a NEW raw image at the look-down angle and compare both ROIs.


### V05 correction: transient yaw pitch must not prematurely end Round 1

The 27 Sep 20:04 run exported prematurely while RIGHT yaw was near its
endpoint (actual +84.4 deg) because the interim pitch reading was +6.1 deg,
just above the former 6.0 deg *in-motion* cutoff. During the earlier
stationary test, the same hardware had reached correct pitch after far
larger **transient** excursions. The in-motion limit alone was therefore an
over-sensitive stop condition; it was not evidence that maze exploration
completed or failed to detect any actual target.

The scanner still uses staged pitch -> yaw-only -> pitch, with the same speed,
gain, target, and tolerances. The old \`gimbal_yaw_pitch_guard_deg\` setting
now triggers \`TRANSIENT_PITCH_WARNING\` during stationary yaw motion rather
than aborting immediately. No ToF mapping ray is recorded during a yaw turn,
and navigation is not resumed unless final yaw/pitch both settle and
independent checks after the ToF sampling period also pass. Failure to
re-level still stops exploration and exports a partial result.

Use the offline regression and then a short real run:

\`\`\`powershell
python -m unittest tests.test_final_gimbal_pitch_v05 -v
python -u final_round1_tof_camera_01.py
\`\`\`

The first interrupted run surveyed LEFT and FRONT only, both reporting zero
camera candidates. That does not establish that all other directions had no
targets; RIGHT/BACK were not completed. Before the next run, use the
independent annotated preview and **SAVE RAW + DETECTED CAMERA FRAME** at a
direction with a visible real low sign, and inspect the 0.94 ROI boundary,
camera pitch and confidence.


### V05 follow-up: distant sign location, yaw speed, visited-cell scan reuse

The real-field screenshot showed a red sign at the far end of a corridor, but
the first camera-only observation was plotted near the start cell. A monocular
camera detection provides an observing cell, viewing direction and pixel
centroid, **not** a measured sign position. A horizontal ToF ray can end on a
different plane from the low sign.

V05 now keeps these different kinds of evidence separate in \`targets.json\`:
- \`observation_cells\`: robot cells from which the sign was seen;
- \`line_of_sight_end_xy_m\`: the horizontal ToF ray endpoint when available;
- \`sighting_cell_hint\`: the *unconfirmed* last cell before the distant ToF
  wall reading; e.g. 136 cm along LEFT from (0,0) suggests (0,2), but does
  not establish a target location;
- \`estimated_target_xy_m: null\`, \`approach_cells: []\`,
  \`status: SIGHTING_ONLY\`, and \`round2_position_ready: false\` for distant
  sightings;
- \`NEAR_WALL_ESTIMATE\` for the conservative projection after a later
  wall-range-confirmed observation; these projections still require close
  verification before round 2.

The map displays distant observations as **hollow Txx? LOS** markers, with a
dashed line from the observer to the tentative ray cell. Only tentative
near-wall estimates are solid markers. Neither is a guaranteed ground-truth
target coordinate, and neither is automatically marked Round-2-ready.
Duplicate observations from different cells are retained for later geometric
association rather than silently merging two same-colored signs.

The first tab of the pre-mission Configuration GUI is now **Mission Settings**
with the frequent controls (robot speed, maximum Gimbal yaw speed, visited
cell scan reuse, target survey, camera pitch/ROI and export). The advanced
tabs share the same Tk variables; no duplicate settings diverge.
The runtime GUI also has a max-yaw slider (15-90 deg/s) and visited-cell scan
checkbox. These controls update configuration at scan/checkpoints and never
directly command motors.

**Skip repeat scan** reuses a 4-way cached OPEN/WALL topology only if that
exact visited cell has previously completed all four physical directions,
every edge is known, and no operator rescan is pending. On a revisit it records
\`SCAN_REUSED\` and skips the time-consuming gimbal sweep and camera survey.
New cells, ambiguous/unknown edges, explicit RESCAN, and a disabled option
still get the full scan. Before EVERY translation, and THROUGHOUT motion,
fresh ToF in the actual travel direction remains mandatory; cached distances
are never reused for collision protection or temporary side centering.

Offline regression (no RoboMaster required):

\`\`\`powershell
python -m unittest tests.test_final_target_detection tests.test_final_scan_reuse tests.test_final_live_survey -v
\`\`\`

If tests pass, run \`python -u final_round1_tof_camera_01.py\` and first verify
that a far sign produces \`SIGHTING_ONLY\` / \`sighting_cell_hint\` rather than
a physical target at the start, and that returning to a fully scanned cell
produces \`[SCAN_REUSED]\` while fresh move-direction ToF remains active.


### V05 safer mecanum straight-line motion with ToF + odometry only

The motion controller previously corrected lateral error from wheel odometry,
plus a small side-distance bias taken only at the start of a cell and faded
away after about 35 cm. That alone is not proof the chassis is parallel to
a wall: mecanum wheel slip can be absent from wheel odometry, and the one
gimbal ToF cannot continuously face both forward and sideways.

The V05 movement addition applies only to a leg with a **confirmed mapped
wall at its side**. By default, after 0.48 of a 60 cm cell (~29 cm), it:
1. Sends a full chassis stop.
2. Points the existing ToF at the known wall side(s) and samples each while
   stationary at horizontal pitch.
3. If any known side return is <=22 cm, stops the mission with a
   \`SIDE_CLEARANCE_LOW_LEFT/RIGHT\` reason. This threshold is a
   **sensor-to-wall range**, not actual body clearance; calibrate it with
   the specific RoboMaster / foam wall installation.
4. With sufficiently reliable wall returns, makes only a bounded
   right-relative lateral bias; unknown/distant returns cannot command
   automatic sideways motion.
5. Restores the ToF to the actual travel direction, waits for a new forward
   reading, and **only then** resumes the remaining fraction of the 60 cm
   cell. There is at most one such mid-cell side checkpoint per move.

Wheel odometry has an independent cross-track guard: slowdown after reported
3.0 cm lateral error, and a stop at 8.5 cm. The total lateral command near
mapped side walls is capped at 0.028 m/s, even if scan and odometry terms
otherwise add. Loss of yaw feedback while heading hold is enabled also stops
the chassis rather than disabling yaw correction silently. These movement
events are saved as \`MIDCELL_SIDE_CHECK\`, \`SIDE_CLEARANCE_LOW\`,
\`CROSS_TRACK_LIMIT\`, and \`CELL_MOTION_QUALITY\` in exploration logs.

The original 60 cm logical cell goal and frontier planner are unchanged.
The optional mid-cell physical checks may add several seconds per wall-side
move, which matters for the 15-minute Final time budget. Switch them off
only after field testing establishes that the increased risk is acceptable.

The pre-mission **Mission Settings** tab includes "Check side-wall clearance
mid-cell", and **ToF / Safety** exposes progress ratio, side hard/soft
thresholds, maximum mid-cell bias, combined lateral limit, and cross-track
slow/stop thresholds. The new defaults are conservative field-testing
starting points, NOT certified collision-avoidance limits.

A single gimbal ToF cannot guarantee continuous side collision detection or
independently measure a robot's full wall-parallel orientation. Start with
the chassis placed parallel to the actual maze wall, test a short straight
route with a hand ready to STOP, and compare true tape-measured lateral
clearance to the ToF readings and odometry log. Low foam can be missed by
an unlevel sensor. Sharp/IR side hardware would later improve continuous
protection; no such hardware is assumed here.

Offline tests (no robot connection):

\`\`\`powershell
python -m py_compile classwork8\motion_safety_v05.py classwork8\tof_camera_round1_v05.py classwork8\config.py classwork8\config_gui_v05.py
python -m unittest tests.test_final_motion_safety_v05 tests.test_final_scan_reuse tests.test_final_gimbal_pitch_v05 -v
\`\`\`

On a real test, look for \`[SIDE_CHECK]\` at a wall-adjacent cell,
\`[MOVE] Reached ... peak cross=... yaw_error=...\`, and export
\`exploration_log.csv\` to compare physical and reported drift.


### V05 27 September hotfix: scan NameError, faster default scan, lower signs

A prior automated edit inserted the *movement-only* mid-cell checkpoint inside
\`_scan_four_directions\`, even though that function has no
\`checkpoint_enabled\` variable. This terminated Round 1 at its first
stationary scan with \`NameError\` and still exported partial files. The
checkpoint now exists exclusively in \`_drive_one_cell\`; the scan only surveys
ToF/camera and updates topology. Offline test
\`test_checkpoint_code_is_only_inside_drive_not_four_way_scan\` prevents that
scope mistake recurring.

Requested new initial settings: yaw max 90 deg/s, yaw Kp 2.0, minimum yaw
12 deg/s, final yaw tolerance 2.5 deg; pitch Kp 1.5, pitch rate 3..24 deg/s.
The staged pitch -> yaw-only -> pitch sequence and final level safety guard
remain. The previously attached run ended with yaw +2.2 deg, just above the
old +/-2.0 deg final tolerance; the slightly wider 2.5 deg threshold avoids
that specific marginal stop, while still requiring final angle feedback.

For low floor signs, the camera survey now defaults to -15 degrees (formerly
-10), ROI bottom 0.96, and eight *new* frames with four consecutive required
for verification. A target that passes four consecutive frames is preserved
if later samples are lost to glare. These changes can also admit more floor
reflections, so compare live raw/annotated images and tune HSV/ROI under the
exam lighting. No direct autonomous chassis camera steering was enabled.

The requested 0.25 m/s value is the **open-route travel ceiling**, not a
blanket speed in narrow corridors. A leg with a confirmed wall at either side
is independently capped at 0.12 m/s, then front-range and lateral-drift
slowdown can lower it further. Mid-cell side-check and emergency front stop
remain enabled. For the very first physical trial after this change, lower
travel speed to 0.08-0.10 m/s in Mission Settings and use a short unobstructed
test; do not begin with a full 7x7 run.

Test offline on the user's Windows/Python 3.8 environment:

\`\`\`powershell
git pull origin classwork8-slam-exploration
python -m py_compile classwork8\config.py classwork8\config_gui_v05.py classwork8\gui_v05.py classwork8\target_detection.py classwork8\tof_camera_round1_v05.py
python -m unittest tests.test_final_motion_safety_v05 tests.test_final_target_detection tests.test_final_gimbal_pitch_v05 tests.test_final_scan_reuse tests.test_final_live_survey -v
\`\`\`


### V05 21:02 mid-cell side false stop: compare with the start-side baseline

The real run scanned the RIGHT wall at **14.5 cm** while still at (0,0).
At ~0.29 m into the first forward cell, it read **14.7 cm** on that same
side. The old hard-stop default of 22 cm incorrectly treated a nearly
unchanged reading as dangerous and ended the mission after exporting three
target records. The stop did not indicate mapping completion or a Python
exception.

The baseline-aware correction now:
- uses 10 cm as an **uncalibrated starting critical ToF-to-wall limit**;
- checks for a 4 cm-or-greater REDUCTION from the side reading measured in
  that cell's four-way stationary scan;
- ignores side changes up to 1.5 cm for lateral centering;
- does not try to achieve a fabricated 28 cm clearance from a single wall;
- does not apply automatic side bias without a valid fresh start-of-leg
  baseline (for example when the revisit scan was intentionally skipped);
- requires a full chassis stop for the mid-cell side scan, then returns ToF
  to travel direction and waits for a fresh forward reading before resuming;
- logs the baseline, mid-cell range, and explicit \`[SIDE_CHECK] STOP\` reason
  whenever motion is blocked. Every run prints \`[MISSION] Finish reason:\`.

These are **sensor readings, not physical chassis-side clearance**. Measure
the actual sensor-to-body offset and side gap before relying on any 10 cm
absolute threshold. A foam edge can be missed or reflect badly. Recheck a
single wall-adjacent cell at 0.08-0.10 m/s with someone ready to STOP, then
adjust the independent calibrated critical range and maximum drop in the
ToF / Safety tab. Never simply disable ToF front-stop or lateral-drift safety
to make the run continue.

The new offline regression includes the exact 14.5 -> 14.7 cm RIGHT-wall
case (must continue with zero bias), a 14.5 -> 10.4 cm reduction (must stop),
absolute critical range, missing baseline and one-wall no-blind-strafe tests.

\`\`\`powershell
git pull origin classwork8-slam-exploration
python -m py_compile classwork8\motion_safety_v05.py classwork8\tof_camera_round1_v05.py classwork8\config.py classwork8\config_gui_v05.py
python -m unittest tests.test_final_motion_safety_v05 -v
\`\`\`


### V05 21:14 field stop: LEFT ToF = 6.5 cm on frontier relocation

This run was **not** a 30 cm checkpoint stop or an exception. Four full
directional surveys completed at the start, the robot drove/accepted several
cells, and the run reported:

\`\`\`text
[SCAN] LEFT ToF = 6.5 cm
[MOVE] SIDE_CRITICAL_AT_START LEFT: 6.5 cm <= 10.0 cm.
[MISSION] Finish reason: FRONTIER_RELOCATE_SIDE_CRITICAL_AT_START_LEFT
\`\`\`

The previous move was accepted at progress 0.536 m due to a confirmed FRONT
obstacle at 15.6 cm; the physical chassis may therefore be about 6.4 cm
short of the logical centre of (1,-2). This *may contribute* to an awkward
side/corner reading; the log alone cannot establish whether there is
physical contact, a wall edge/reflection or odometry error.

Hotfix: when a start-side reading is critical, the stopped robot temporarily
looks at that side and obtains **two independent fresh ToF returns**, clearing
the filter before each. Both must be consistent within 2 cm and each must
be at least 12 cm (10 cm critical + 2 cm hysteresis). Only then will it
restore the Gimbal to the actual travel direction, require new forward ToF,
and consider resuming. Persistently low readings (e.g. 6.4 and 6.6 cm),
inconsistent readings, absent fresh updates or pitch misalignment keep
the chassis stopped and print the precise \`[SIDE_START]\` reason.

The 10 cm critical distance is a SENSOR-to-wall threshold, not a certified
clearance of the RoboMaster body. Before changing it, physically measure the
gap between the body/wheels and foam wall at the paused cell, compare the
measured offset from the mounted ToF, and correct the starting alignment if
needed. Never lower the guard just to make the mission finish. This update
does not change the gimbal controller, frontier planner, travel speed,
target-detection thresholds or heading control.

Offline regression:

\`\`\`powershell
python -m py_compile classwork8\motion_safety_v05.py classwork8\tof_camera_round1_v05.py classwork8\config.py classwork8\config_gui_v05.py
python -m unittest tests.test_final_motion_safety_v05 -v
\`\`\`
