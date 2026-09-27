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
