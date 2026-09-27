"""Shared live target overlay + safe runtime camera-survey angle requests.

CameraService alone owns the RoboMaster stream. This worker only reads fresh
frames and annotates them, independently of the mapping GUI publish cadence.
The GUI NEVER sends gimbal commands: its angle is applied by the mission
worker only while the chassis is stationary at a scan checkpoint.
"""

from __future__ import annotations

import threading
import time
from datetime import datetime
from pathlib import Path
from typing import Optional

import cv2

from .target_detection import TargetDetector


class LiveSurveyBridge:
    def __init__(self, config) -> None:
        self._lock = threading.Lock()
        self._config = config
        self._camera = None
        self._preview = None
        self._raw_frame = None
        self._preview_timestamp = 0.0
        self._capture_timestamp = 0.0
        self._candidate_count = 0
        self._fps = 0.0
        self._pitch_deg = float(config.target_camera_pitch_deg)
        self._status = "Waiting for camera"
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._rescan = threading.Event()

    def set_pitch(self, value: float) -> float:
        value = max(
            float(self._config.target_camera_pitch_min_deg),
            min(float(self._config.target_camera_pitch_max_deg), float(value)),
        )
        with self._lock:
            self._pitch_deg = float(value)
            self._status = "Camera angle queued for next stopped scan"
        return value

    def get_pitch(self) -> float:
        with self._lock:
            return float(self._pitch_deg)

    def request_rescan(self) -> None:
        self._rescan.set()
        self.set_status("Rescan requested at the current cell")

    def consume_rescan(self) -> bool:
        if self._rescan.is_set():
            self._rescan.clear()
            return True
        return False

    def set_status(self, status: str) -> None:
        with self._lock:
            self._status = str(status)

    def attach_camera(self, camera) -> None:
        if camera is None or not camera.running:
            self.set_status("Camera unavailable")
            return
        self._camera = camera
        self._stop.clear()
        self._thread = threading.Thread(
            target=self._preview_loop,
            name="final-live-target-preview",
            daemon=True,
        )
        self._thread.start()
        self.set_status("Live color/shape preview active")

    def _preview_loop(self) -> None:
        # Separate detector avoids sharing a mutable OpenCV CLAHE object with
        # the mapping thread's temporal verification.
        detector = TargetDetector(self._config)
        previous_capture = None
        previous_processed = None
        period = 1.0 / max(1.0, float(self._config.target_preview_fps))

        while not self._stop.is_set():
            started = time.monotonic()
            camera = self._camera
            if camera is None or not camera.running:
                self._stop.wait(0.10)
                continue

            sample = camera.latest_with_timestamp(
                max_age_sec=float(self._config.target_max_frame_age_sec)
            )
            if sample is not None:
                frame, capture_time = sample
                if previous_capture is None or capture_time > previous_capture:
                    try:
                        candidates, debug = detector.detect(frame)
                        now = time.monotonic()
                        fps = (
                            0.0 if previous_processed is None
                            else 1.0 / max(1e-3, now - previous_processed)
                        )
                        previous_processed = now
                        previous_capture = capture_time
                        with self._lock:
                            self._raw_frame = frame.copy()
                            self._preview = debug
                            self._preview_timestamp = now
                            self._capture_timestamp = float(capture_time)
                            self._candidate_count = len(candidates)
                            self._fps = fps
                    except Exception as exc:
                        self.set_status("Preview detector error: {}".format(exc))

            self._stop.wait(max(0.005, period - (time.monotonic() - started)))

    def latest_preview(self):
        """Return one immutable-by-convention snapshot to the Tk main thread."""
        with self._lock:
            age = (
                float("inf")
                if self._preview_timestamp <= 0.0
                else time.monotonic() - self._preview_timestamp
            )
            if age > 1.25 or self._preview is None:
                frame = None
            else:
                frame = self._preview.copy()
            return {
                "frame": frame,
                "timestamp": float(self._preview_timestamp),
                "age_sec": age,
                "candidate_count": int(self._candidate_count),
                "preview_fps": float(self._fps),
                "pitch_deg": float(self._pitch_deg),
                "status": self._status,
            }

    def save_camera_sample(self, output_dir) -> Optional[tuple]:
        """Save a matching raw/annotated frame for real-lighting calibration."""
        with self._lock:
            if self._raw_frame is None or self._preview is None:
                return None
            raw = self._raw_frame.copy()
            debug = self._preview.copy()

        folder = Path(output_dir) / "target_camera_samples"
        folder.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        raw_path = folder / "target_raw_{}.png".format(stamp)
        debug_path = folder / "target_debug_{}.png".format(stamp)
        if not cv2.imwrite(str(raw_path), raw):
            raise RuntimeError("Could not save {}".format(raw_path))
        if not cv2.imwrite(str(debug_path), debug):
            raise RuntimeError("Could not save {}".format(debug_path))
        return str(raw_path), str(debug_path)

    def stop(self) -> None:
        self._stop.set()
        thread = self._thread
        if thread is not None and thread.is_alive() and (
            threading.current_thread() is not thread
        ):
            thread.join(timeout=2.0)
        self._thread = None
        with self._lock:
            self._preview = None
            self._raw_frame = None
