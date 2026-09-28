"""V04-only resilient live preview: the V05 detector never gates raw video.

This subclass changes presentation only. It does not alter the shared camera
stream, detector thresholds, robot motion, Gimbal mapping or V05 original GUI.
"""
from __future__ import annotations

import time
import cv2

from .live_survey import LiveSurveyBridge as _OriginalLiveSurveyBridge
from .target_detection import TargetDetector


class LiveSurveyBridge(_OriginalLiveSurveyBridge):
    def _preview_loop(self) -> None:
        detector = None
        reported_error = None
        try:
            detector = TargetDetector(self._config)
        except Exception as exc:
            reported_error = "Detector startup: {}".format(exc)
            self.set_status("Preview detector error: {}".format(exc))
            print("[CAMERA_PREVIEW] {}".format(reported_error), flush=True)

        previous_capture = None
        previous_processed = None
        period = 1.0 / max(1.0, float(self._config.target_preview_fps))
        first_frame_reported = False
        last_frame_seen = time.monotonic()
        stale_reported = False

        while not self._stop.is_set():
            started = time.monotonic()
            camera = self._camera
            if camera is None or not camera.running:
                self._stop.wait(0.10)
                continue

            try:
                sample = camera.latest_with_timestamp(
                    max_age_sec=float(self._config.target_max_frame_age_sec)
                )
            except Exception as exc:
                sample = None
                message = "Camera frame read: {}".format(exc)
                if reported_error != message:
                    print("[CAMERA_PREVIEW] {}".format(message), flush=True)
                    self.set_status(message)
                    reported_error = message

            if sample is not None:
                frame, capture_time = sample
                if previous_capture is None or capture_time > previous_capture:
                    previous_capture = capture_time
                    last_frame_seen = time.monotonic()
                    stale_reported = False

                    # Always preserve the raw decoded frame, even when the
                    # detector raises. This is the key preview isolation.
                    debug = frame.copy()
                    candidates = []
                    error_message = None
                    if detector is not None:
                        try:
                            candidates, debug = detector.detect(frame)
                            debug = self.annotate_live_candidates(
                                debug, candidates, self.get_roi_bottom()
                            )
                        except Exception as exc:
                            error_message = "Detector: {}".format(exc)
                    else:
                        error_message = reported_error or "Detector unavailable"

                    if error_message:
                        cv2.putText(
                            debug, "RAW CAMERA - DETECTOR ERROR",
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                            0.65, (0, 0, 255), 2, cv2.LINE_AA,
                        )
                        self.set_status("Preview detector error: {}".format(error_message))
                        if reported_error != error_message:
                            print("[CAMERA_PREVIEW] {}; raw fallback active".format(
                                error_message
                            ), flush=True)
                            reported_error = error_message
                    elif reported_error is not None:
                        reported_error = None
                        self.set_status("Live color/shape preview active")

                    now = time.monotonic()
                    fps = (0.0 if previous_processed is None else
                           1.0 / max(1e-3, now - previous_processed))
                    previous_processed = now
                    with self._lock:
                        self._raw_frame = frame.copy()
                        self._preview = debug.copy()
                        self._preview_timestamp = now
                        self._capture_timestamp = float(capture_time)
                        self._candidate_count = len(candidates)
                        self._fps = fps
                    if not first_frame_reported:
                        h, w = frame.shape[:2]
                        print("[CAMERA_PREVIEW] First live frame visible: {}x{}".format(
                            w, h
                        ), flush=True)
                        first_frame_reported = True

            if not stale_reported and time.monotonic() - last_frame_seen > 2.0:
                print(
                    "[CAMERA_PREVIEW] Waiting for NEW decoded frame; stream may be stalled.",
                    flush=True,
                )
                self.set_status("Camera stream opened, but no recent decoded frames")
                stale_reported = True

            self._stop.wait(max(0.005, period - (time.monotonic() - started)))
