# V05 yaw isolation: distinguish commanded turn, scan coupling, and stale IMU

The previous bounded post-scan correction can mask a persistent sign error:
small errors may accumulate without exceeding its short divergence threshold.
This is a **diagnostic experiment**, not a claim that the physical root cause
has been determined without new robot telemetry.

## Confirmed code-level issues repaired

- FREE mode previously logged `mode_ok` but continued even if False. A failed
  mode transition could leave chassis and gimbal coupled; now fail closed.
- The four-direction scan previously had an unused `stop_chassis_fn_called`
  placeholder but no explicit chassis zero at scan entry. The caller now sends
  x=y=z=0 before **every** full sweep.
- Previous diagnostics did not distinguish an old non-None chassis yaw from
  a recently received attitude sample. V05 now stamps callback arrival time,
  reports `yaw_age_sec`, and stops a motion command after >1.5s of stale yaw
  while heading is in use or isolation is active.
- The previous post-scan correction and the translation loop both generated
  chassis yaw commands, so looking at accumulated diff alone cannot identify
  their contribution. `--yaw-isolation` disables both and hard-enforces z=0
  at the final movement command producer.

## Run one isolated experiment

Connect the robot to PC, turn off other RoboMaster controller apps/scripts,
and ensure the field is open and supervised. **Do not place the robot in the
maze** for this first experiment. Put a tape arrow aligned with its physical
nose as an external reference (not the GUI map or another IMU).

```powershell
git pull --ff-only origin refactor/v05-basic-motion-91fa792
python -u final_round1_tof_camera_01.py --yaw-isolation --travel-speed 0.10 2>&1 | Tee-Object -FilePath yaw_isolation_v05.log
```

The GUI still opens and the rest of the scanning/mapping/target detector
remain enabled. The CLI flag is enforced **again after the GUI** to ensure it
cannot accidentally be reset. With this flag, every translation is x/y only;
the program will never intentionally command nonzero chassis z, including
at the post-scan alignment checkpoint. No automatic sign-flipping or repeated
turning tests occur. Ctrl+C/manual stop and stopping at the normal cell
endpoint are preserved.

Important trace markers:

- `[INIT] FREE mode result` must be True; otherwise no scan begins.
- `[HEADING_TRACE] phase=STATIONARY_0..5` samples 3 seconds with the gimbal
  held front and chassis explicitly stopped.
- `PRE_GIMBAL_* / POST_GIMBAL_*` show each individual sweep's chassis yaw.
- `SCAN_STOP_SENT_*`, `PRE_SCAN_*`, `POST_SCAN_*`, `PRE_DEPARTURE_*`,
  `POST_MOVE_*` distinguish stationary vs translation.
- `yaw_age_sec` shows freshness; `gimbal_relative` and `gimbal_ground`
  are correlated camera/gimbal telemetry, **not** a substitute for an
  independent physical heading reference.
- `[MOTION] ... yaw_correction=+0.00` must show zero during the isolation test.

How to interpret:

| Evidence | Useful next investigation |
| --- | --- |
| Physical robot rotates during STATIONARY with no gimbal movement and z=0 | other controller, firmware/mode, wheel movement or hardware; compare yaw-age and physical tape |
| Chassis shifts mainly from PRE_GIMBAL to POST_GIMBAL despite z=0 | FREE mode response and gimbal/chassis physical or control coupling |
| Physical nose stays fixed but SDK chassis yaw drifts | attitude estimator/IMU rather than physical wheel rotation |
| No yaw drift while z=0, but drift returns in ordinary mode | wrong z-command sign/gain/noise; calibrate physical response before changing heading_drive_sign |
| Movement-only left drift with z=0 | wheel asymmetry/traction/mecanum/SDK translation control rather than programmed heading correction |
| yaw_age_sec rises >1.5 | lost/stale attitude telemetry, not true live yaw |

Please send the full `yaw_isolation_v05.log` and describe whether the tape
arrow/physical chassis turned to the left in STATIONARY, during scan or
only when traveling. This is required before attributing the persistent
drift to the motor/IMU/gimbal/sign.

This is NOT a physical robot safety certification; use a separate accessible
E-stop. BASIC motion still has no wall collision avoidance.
