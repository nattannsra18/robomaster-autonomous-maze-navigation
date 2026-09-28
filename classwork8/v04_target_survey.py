"""Camera-only adapter for the unchanged V04 navigation/SLAM controller.

Never imports the V05 motion/SLAM module. The shared camera is only sampled
while the chassis is stopped, after V04 has completed its normal ToF scan.
No camera result is allowed to change V04 occupancy, route, drive or recovery.
"""
from __future__ import annotations

import math
import time
from dataclasses import replace

from robomaster_mission.mission import stop_chassis
from .camera_service import CameraService
from .live_survey import LiveSurveyBridge
from .target_detection import TargetDetector, TargetRegistry, survey_targets_with_hold


class V04TargetSurvey:
    def __init__(self, config, survey_bridge=None):
        self.config = config
        self.bridge = survey_bridge if survey_bridge is not None else LiveSurveyBridge(config)
        self.camera = None
        self.detector = TargetDetector(config)
        self.registry = TargetRegistry(config)

    def start(self, ep_robot):
        if not self.config.target_detection_enabled:
            self.bridge.set_status("Camera survey disabled; pure V04 ToF mapping")
            return False
        self.camera = CameraService(
            ep_robot, resolution=self.config.target_camera_resolution,
            start_timeout_sec=self.config.target_camera_start_timeout_sec,
        )
        if not self.camera.start():
            self.camera = None
            self.bridge.set_status("Camera unavailable; pure V04 ToF mapping")
            return False
        self.bridge.attach_camera(self.camera)
        return True

    def stop(self):
        # The preview thread must release the shared frames BEFORE stream shutdown.
        self.bridge.stop()
        if self.camera is not None:
            self.camera.stop()

    @staticmethod
    def _wait(seconds, stop_event):
        end = time.monotonic() + max(0.0, seconds)
        while time.monotonic() < end:
            if stop_event.is_set():
                return False
            stop_event.wait(min(0.025, end - time.monotonic()))
        return not stop_event.is_set()

    def _pitch(self, gimbal, tracker, desired, stop_event):
        """Camera-only pitch positioning. Yaw speed is ALWAYS zero."""
        c = self.config
        desired = float(desired)
        deadline = time.monotonic() + float(c.target_camera_pitch_timeout_sec)
        stable = 0
        try:
            while time.monotonic() < deadline:
                if stop_event.is_set():
                    return False
                measured = tracker.get_pitch()
                if measured is None:
                    stop_event.wait(0.025)
                    continue
                error = desired - float(measured)
                tol = (float(c.gimbal_pitch_tolerance_deg)
                       if abs(desired - float(c.gimbal_scan_pitch_deg)) < 0.01
                       else float(c.target_camera_pitch_tolerance_deg))
                if abs(error) <= tol:
                    stable += 1
                    gimbal.drive_speed(pitch_speed=0.0, yaw_speed=0.0)
                    if stable >= int(c.gimbal_stable_samples):
                        return self._wait(float(c.target_camera_settle_sec), stop_event) and (
                            tracker.get_pitch() is not None and
                            abs(float(tracker.get_pitch()) - desired) <= tol
                        )
                else:
                    stable = 0
                    speed = max(float(c.gimbal_pitch_min_speed_dps), min(
                        float(c.gimbal_pitch_max_speed_dps),
                        abs(error) * float(c.gimbal_pitch_kp)))
                    gimbal.drive_speed(
                        pitch_speed=math.copysign(speed, error) * float(c.gimbal_pitch_drive_sign),
                        yaw_speed=0.0,
                    )
                stop_event.wait(0.03)
            return False
        finally:
            gimbal.drive_speed(pitch_speed=0.0, yaw_speed=0.0)

    def _camera_yaw(self, gimbal, tracker, desired, stop_event):
        """Only optional off-axis CAMERA views; never samples ToF or changes map."""
        c = self.config
        deadline = time.monotonic() + float(c.gimbal_turn_timeout_sec)
        stable = 0
        try:
            while time.monotonic() < deadline:
                if stop_event.is_set():
                    return False
                pitch, yaw = tracker.get_angles()
                if pitch is None or yaw is None:
                    stop_event.wait(0.03)
                    continue
                # A real yaw-only sweep may transiently perturb physical pitch.
                # No ToF is sampled here; restore and verify pitch AFTER yaw settles.
                error = float(desired) - float(yaw)  # mechanical axis, do NOT wrap
                if abs(error) <= float(c.gimbal_tolerance_deg):
                    stable += 1
                    gimbal.drive_speed(pitch_speed=0.0, yaw_speed=0.0)
                    if stable >= int(c.gimbal_stable_samples):
                        return self._wait(float(c.gimbal_settle_sec), stop_event)
                else:
                    stable = 0
                    speed = max(float(c.gimbal_min_yaw_speed_dps), min(
                        float(c.gimbal_yaw_speed_dps),
                        abs(error) * float(c.gimbal_yaw_kp),
                    ))
                    gimbal.drive_speed(
                        pitch_speed=0.0, yaw_speed=math.copysign(speed, error))
                stop_event.wait(0.03)
            return False
        finally:
            gimbal.drive_speed(pitch_speed=0.0, yaw_speed=0.0)

    def _align_camera_view(self, gimbal, tracker, desired_yaw, desired_pitch, stop_event):
        """Bounded camera-only yaw/pitch alignment BEFORE recording any image.

        Correct yaw with the camera level, then reposition camera pitch. On
        real hardware a pitch-only adjustment can perturb yaw, so verify BOTH
        axes from feedback after a fresh settling interval. Never command the
        chassis or alter the V04 map/navigation scan.
        """
        c = self.config
        for attempt in range(1, 4):
            if stop_event.is_set():
                return False
            measured_pitch, measured_yaw = tracker.get_angles()
            if (measured_yaw is None or
                    abs(float(measured_yaw) - float(desired_yaw)) >
                    float(c.gimbal_tolerance_deg)):
                if not self._pitch(
                    gimbal, tracker, float(c.gimbal_scan_pitch_deg), stop_event
                ):
                    return False
                if not self._camera_yaw(gimbal, tracker, desired_yaw, stop_event):
                    return False
            if not self._pitch(gimbal, tracker, desired_pitch, stop_event):
                return False
            if not self._wait(0.12, stop_event):
                return False
            measured_pitch, measured_yaw = tracker.get_angles()
            if (measured_pitch is not None and measured_yaw is not None and
                    abs(float(measured_pitch) - float(desired_pitch)) <=
                    float(c.target_camera_pitch_tolerance_deg) and
                    abs(float(measured_yaw) - float(desired_yaw)) <=
                    float(c.gimbal_tolerance_deg)):
                return True
            print(
                "[TARGET_V04] View alignment retry {}/3: desired yaw={:+.1f}, "
                "pitch={:+.1f}; actual yaw={}, pitch={}.".format(
                    attempt, float(desired_yaw), float(desired_pitch),
                    "---" if measured_yaw is None else "{:+.1f}".format(float(measured_yaw)),
                    "---" if measured_pitch is None else "{:+.1f}".format(float(measured_pitch)),
                ),
                flush=True,
            )
        return False

    def _view(self, chassis, gimbal, tracker, stop_event, cell,
              direction, tof_cm, *, side_offset=0.0, recorder=None):
        """Capture at a fully stopped pose and restore horizontal pitch ALWAYS."""
        c = self.config
        stop_chassis(chassis)
        pitch = float(self.bridge.get_pitch())
        is_side = bool(side_offset)
        view_config = c
        detector = self.detector
        if is_side:
            view_config = replace(
                c,
                target_sample_frames=min(int(c.target_sample_frames),
                                         int(c.target_camera_side_sample_frames)),
                target_hold_max_sec=min(float(c.target_hold_max_sec),
                                        float(c.target_camera_side_hold_max_sec)),
                target_hold_max_windows=1,
            )
            detector = TargetDetector(view_config)
        desired_yaw = float(c.gimbal_yaw_for_direction(direction)) + float(side_offset)
        try:
            # Do not begin capturing at a yaw that has drifted after camera pitch.
            # An optional camera view can be skipped; its ToF-level restoration
            # below remains mandatory before V04 navigation is permitted.
            if not self._align_camera_view(
                gimbal, tracker, desired_yaw, pitch, stop_event
            ):
                print(
                    "[TARGET_V04] {} view skipped: camera yaw/pitch not stable "
                    "(requested yaw={:+.1f}, pitch={:+.1f}).".format(
                        "SIDE" if is_side else "CARDINAL", desired_yaw, pitch
                    ), flush=True,
                )
                if recorder is not None:
                    recorder.event(
                        time.monotonic(), "V04_CAMERA_VIEW_SKIPPED",
                        "UNSTABLE_CAMERA_POSE",
                        logical_node=cell, direction=direction, side_offset_deg=side_offset,
                    )
                return 0
            epoch = time.monotonic()  # NEVER verify frames captured before pose settled
            verified, pending, debug, windows = survey_targets_with_hold(
                detector, self.camera, view_config,
                not_before=epoch, stop_event=stop_event)
            measured, measured_yaw = tracker.get_angles()
            if (measured is None or measured_yaw is None or
                    abs(float(measured) - pitch) >
                    float(c.target_camera_pitch_tolerance_deg) or
                    abs(float(measured_yaw) - desired_yaw) >
                    float(c.gimbal_tolerance_deg)):
                # Image evidence must match a known camera bearing. Discard it
                # instead of inventing a target coordinate or ending V04 SLAM.
                verified, pending = [], []
                print(
                    "[TARGET_V04] Discarded camera evidence: pose changed "
                    "during capture (desired yaw={:+.1f}, actual yaw={}).".format(
                        desired_yaw,
                        "---" if measured_yaw is None else
                        "{:+.1f}".format(float(measured_yaw))
                    ), flush=True,
                )
            if is_side:
                for item in verified:
                    self.registry.add_side_view_sighting(
                        item.detection, cell, direction,
                        camera_yaw_deg=float(measured_yaw),
                        camera_pitch_deg=pitch,
                        side_yaw_offset_deg=side_offset,
                        confidence=float(item.confidence),
                        verified_frames=int(item.verified_frames), verified=True)
                for item in pending:
                    d = item["detection"]
                    self.registry.add_side_view_sighting(
                        d, cell, direction,
                        camera_yaw_deg=float(measured_yaw),
                        camera_pitch_deg=pitch,
                        side_yaw_offset_deg=side_offset,
                        confidence=float(item["confidence_sum"]) / max(1, int(item["frames"])),
                        verified_frames=int(item["frames"]), verified=False)
            else:
                near_wall = tof_cm is not None and 0.0 < float(tof_cm) < float(c.tof_open_cm)
                for item in verified:
                    saved = self.registry.add_verified(
                        item, cell, direction, tof_cm,
                        range_confirmed_wall=near_wall, camera_pitch_deg=pitch)
                    print("[TARGET_V04] {} {} -> {} ({})".format(
                        item.detection.color, item.detection.shape,
                        saved["target_id"], saved["localization_status"]), flush=True)
                for item in pending:
                    self.registry.add_pending(item, cell, direction, tof_cm)
            if recorder is not None:
                recorder.event(
                    time.monotonic(), "V04_CAMERA_SURVEY",
                    "side-view" if is_side else "cardinal",
                    logical_node=cell, direction=direction, side_offset_deg=side_offset,
                    verified=len(verified), pending=len(pending), windows=windows)
            return len(verified)
        finally:
            # If restore fails, the caller MUST abort the mission: horizontal
            # ToF and V04 movement cannot use a downward-tilted sensor.
            restored = self._pitch(gimbal, tracker, float(c.gimbal_scan_pitch_deg), stop_event)
            if not restored and not stop_event.is_set():
                raise RuntimeError("CAMERA_HORIZONTAL_TOF_RESTORE_FAILED")

    def survey_cell(self, chassis, gimbal, tracker, sensors, stop_event,
                    cell, ranges, point_v04, recorder):
        if self.camera is None or not self.camera.running:
            return
        c = self.config
        # Survey AFTER V04 has finished mapping this cell. This extra camera
        # sweep does not call any V05 planner, mapper, recovery or drive function.
        here = tracker.get_yaw()
        directions = sorted(range(4), key=lambda d: abs(
            float(c.gimbal_yaw_for_direction(d)) - float(here or 0.0)))
        for direction in directions:
            if stop_event.is_set():
                return
            distance = ranges.get(direction)
            if (not c.target_survey_open_directions and
                    (distance is None or float(distance) >= float(c.tof_open_cm))):
                continue
            stop_chassis(chassis)
            if not self._pitch(gimbal, tracker, float(c.gimbal_scan_pitch_deg), stop_event):
                raise RuntimeError("CAMERA_PRE_SCAN_LEVEL_FAILED")
            if not point_v04(gimbal, sensors, tracker, direction, c, stop_event):
                raise RuntimeError("V04_CAMERA_DIRECTION_FAILED")
            # V04 yaw-only gimbal motion can physically perturb pitch.
            if not self._pitch(gimbal, tracker, float(c.gimbal_scan_pitch_deg), stop_event):
                raise RuntimeError("CAMERA_PRE_VIEW_LEVEL_FAILED")
            self._view(chassis, gimbal, tracker, stop_event, cell,
                       direction, distance, recorder=recorder)
            if c.target_camera_multi_angle_enabled:
                nominal = float(c.gimbal_yaw_for_direction(direction))
                for sign in (-1, 1):
                    if stop_event.is_set():
                        return
                    offset = sign * float(c.target_camera_side_yaw_offset_deg)
                    try:
                        if not self._camera_yaw(gimbal, tracker, nominal + offset, stop_event):
                            print("[TARGET_V04] Skipping optional side view: yaw not reached.", flush=True)
                            continue
                        if not self._pitch(gimbal, tracker, float(c.gimbal_scan_pitch_deg), stop_event):
                            print("[TARGET_V04] Skipping optional side view: horizontal pitch not reached.", flush=True)
                            continue
                        self._view(chassis, gimbal, tracker, stop_event, cell,
                                   direction, distance, side_offset=offset,
                                   recorder=recorder)
                    finally:
                        # An unsuccessful restoration never authorizes a V04
                        # movement, even if the camera detection was successful.
                        if not stop_event.is_set():
                            if not self._pitch(gimbal, tracker, float(c.gimbal_scan_pitch_deg), stop_event):
                                raise RuntimeError("CAMERA_SIDE_RESTORE_PITCH_FAILED")
                            if not self._camera_yaw(gimbal, tracker, nominal, stop_event):
                                raise RuntimeError("CAMERA_SIDE_RESTORE_YAW_FAILED")
                            # Yaw-only motion can disturb pitch. Re-level ToF
                            # before the next view or any V04 movement.
                            if not self._pitch(gimbal, tracker, float(c.gimbal_scan_pitch_deg), stop_event):
                                raise RuntimeError("CAMERA_SIDE_RESTORE_FINAL_LEVEL_FAILED")
        if not self._pitch(gimbal, tracker, float(c.gimbal_scan_pitch_deg), stop_event):
            if not stop_event.is_set():
                raise RuntimeError("CAMERA_FINAL_LEVEL_FAILED")
