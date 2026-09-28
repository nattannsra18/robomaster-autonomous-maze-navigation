# V05 BASIC motion baseline

Base commit: `91fa79257cbb0f1053cbe4748ff170645e5693d5`. Isolated V05 change only; `classwork8_slam_04.py` and
`classwork8/tof_only_v04.py` are unchanged. The V05 entrypoint
`final_round1_tof_camera_01.py` no longer overrides the requested travel
speed to 0.10: use the configuration value or explicit `--travel-speed MPS`.

Pipeline: `run -> _drive_one_cell -> _basic_motion_command -> chassis.drive_speed`.
For FRONT/RIGHT/BACK/LEFT, the requested longitudinal speed is passed through
exactly. `_fixed_heading_control_v02` produces continuous yaw P steering only.
No wall, ToF, cross-track, heading, corner or obstacle condition reduces speed.

## Audit of removed runtime behavior

| File / function | Old trigger | Old effect | BASIC change |
|---|---|---|---|
| tof_camera_round1_v05.py / _drive_one_cell | wall_sides | cap at 0.12 | removed |
| same | cross-track >= 0.030m | cap at 0.065 | removed |
| same | front < slow_front_cm | scale as low as 0.04 | removed |
| same / _confirm_front_blocked | front <= stop_front_cm | stop/confirm/block | removed |
| same | cross-track >= abort | stop | removed |
| same | critical starting side | stop/recheck/abort | removed |
| same / _midcell_wall_checkpoint | mapped side wall halfway | stop/scan/bias/abort | removed |
| same / _fixed_heading_control_v02 | yaw error | set x=y=0 recovery | removed |
| same | gimbal pitch drift | stop/reorient during leg | removed from motion |
| same | deadline | CELL_TIMEOUT stop | removed |
| same / _scan_side_guidance_v02 | scan-side distances | temporary lateral bias | removed |
| same / run | FRONT_BLOCKED | mark blocked / auto replan | removed |
| same | odometry/vision errors | further lateral speed layers | removed |

Legacy fields remain in `config.py` for serialized config compatibility but
are absent from the V05 GUI. The standalone historical helpers in
`motion_safety_v05.py` are not consulted by BASIC motion; only its
`adjacent_wall_sides()` pure topology helper remains imported.
ToF observations still update the grid and logs; missing or close ToF never
changes chassis commands. Cross-track is logged and never gates completion.

**Unchanged:** camera stream, target detector, image processing, target scan
pauses, GimbalTracker and gimbal control, sensor collection, odometry,
occupancy mapping, existing frontier exploration, reporting and export.

**Remaining lifecycle stops:** planned cell completion, explicit GUI/manual
stop, Ctrl+C, mission completion, missing pose/heading/gimbal feedback, and
`finally` shutdown. SDK per-command timeout remains in force. There is
no stuck/stationary motion watchdog and no automatic recovery.

This is an experimental non-collision-avoiding motion layer. Test only in an
unobstructed, physically supervised area with working independent E-stop.

Verification on your RoboMaster PC:

```powershell
python -m py_compile classwork8\tof_camera_round1_v05.py classwork8\config.py classwork8\config_gui_v05.py tests\test_final_motion_safety_v05.py
python -m unittest tests.test_final_motion_safety_v05 tests.test_final_gimbal_pitch_v05 tests.test_final_target_detection -v
```

Example controlled, supervised headless test (choose desired speed):

```powershell
python final_round1_tof_camera_01.py --no-gui --travel-speed 0.10
```
