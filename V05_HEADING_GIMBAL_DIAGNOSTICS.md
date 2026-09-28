# V05 gimbal timeout and gradual left-yaw investigation

This patch is based on the BASIC Motion branch. No physical run logs from the
new version were supplied; the following **verified code causes** and
**field hypotheses** are intentionally distinguished.

## Confirmed in the previous code

1. `_point_gimbal` allocated one `gimbal_turn_timeout_sec` deadline before
   PRE_YAW pitch, YAW, and POST_YAW pitch. All three spent that same budget.
   A slow pitch operation could make a later phase fail even when the gimbal
   remained responsive.
2. At the final endpoint it returned False immediately if pitch or yaw
   settled outside tolerance, without a bounded retry.
3. Gimbal yaw max was 90 in `Classwork8Config`, and additionally hard-capped
   at 90 by `LiveSurveyBridge.set_yaw_speed()` and the live slider.
4. Motion used the mission-start *chassis* attitude as heading target and
   continuous yaw correction, but had no post-scan chassis heading checkpoint.
   `heading_drive_sign=+1` was not physically reconfirmed in BASIC Motion.
5. `GimbalTracker.get_yaw()` is relative to the chassis, not the chassis's
   actual yaw. The SDK also provides `yaw_ground` in sub_angle's fourth
   feedback field. The difference is **diagnostic only** and may share
   onboard estimation; do not treat it as an independent IMU.

## Changes

- Every gimbal stage now starts its **own** timeout; a single bounded
  retry is made if final orientation is outside tolerance.
- Default max yaw speed 140 deg/s, min 7 deg/s, yaw Kp 2.8,
  per-stage timeout 8 seconds. Lower minimum is intended to reduce overshoot
  while speeding up the large-angle sweeps.
- Default camera survey look-down -20 degrees; ToF scan pitch stays at 0.
- Default requested chassis travel speed 0.30 m/s, unchanged by ToF/walls.
- `[HEADING_TRACE]` reports chassis yaw/reference/error and gimbal
  relative/ground yaw at PRE_SCAN, POST_SCAN, PRE_DEPARTURE, POST_MOVE.
- At PRE_DEPARTURE, if chassis heading error exceeds 1.5 degrees, do
  **one bounded yaw-only alignment** with max z=10 deg/s and 3.5 s timeout.
  Errors above 12 degrees stop for inspection instead of attempting an
  unexpected large stationary spin.
- If alignment *increases* yaw error or moving correction diverges, stop
  with `HEADING_SIGN_MISMATCH` or `HEADING_CORRECTION_DIVERGED`.
  Never auto-flip the sign or auto-recover. An operator must inspect the
  trace and verify the physical sign before changing the GUI field.
- Existing camera, target detector, gimbal mechanical scan order, ToF
  mapping, SLAM and export remain intact. No wall/ToF speed caps are restored.

## Distinguish likely field causes

- Chassis heading shifts between PRE_SCAN and POST_SCAN while no chassis
  movement was commanded: inspect actual wheels, FREE mode, mechanical
  movement, gimbal reaction, attitude drift and stationary camera video.
- Chassis heading stable during scan but changes during movement:
  inspect `[HEADING_MOVE]` measured diff and commanded z. If error grows
  systematically in the direction of commanded z, investigate
  `heading_drive_sign`, steering feedback, wheel slip and bias.
- Chassis attitude changes while the *physical* heading does not:
  possible attitude/IMU drift. Compare video/physical alignment; gimbal
  ground-minus-relative is supporting telemetry, not independent ground truth.
- Gimbal fails: `[GIMBAL_FAIL]` now prints PRE_YAW / POST_YAW pitch timeout,
  YAW timeout, or FINAL endpoint values. Compare these with feedback and the
  motor response before changing sign or tolerance.

## Verification

From the repository root on the user's RoboMaster Python environment:

```powershell
git pull --ff-only origin refactor/v05-basic-motion-91fa792
python -m py_compile classwork8\tof_camera_round1_v05.py classwork8\config.py classwork8\config_gui_v05.py classwork8\gui_v05.py classwork8\live_survey.py tests\test_basic_heading_v05.py
python -m unittest tests.test_basic_heading_v05 tests.test_final_gimbal_pitch_v05 tests.test_final_motion_safety_v05 tests.test_basic_gui_config_v05 -v
```

First hardware test: physically supervised, unobstructed area, independent
emergency stop. For initial diagnosis explicitly override 0.30 default
to 0.10 m/s (or launch with no camera to isolate motion):

```powershell
python -u final_round1_tof_camera_01.py --travel-speed 0.10
```

A Python compile or offline test is not proof of physical yaw sign, motor
polarity, gimbal response, safe clearance, or IMU stability.
