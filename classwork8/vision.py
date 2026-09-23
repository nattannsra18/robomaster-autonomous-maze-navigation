"""Camera-assisted corridor centering for Classwork 8.

This module intentionally does not make topology decisions. ToF remains the
authoritative obstacle/wall sensor. Vision only provides a small lateral
correction while moving when both corridor boundaries are visible.

The RoboMaster SDK normally decodes video through libmedia_codec.  Classwork 8
can instead ask the SDK camera module to enable its H.264 stream and let
OpenCV/FFmpeg decode the raw TCP stream directly, avoiding the optional DJI
native codec on Windows.
"""

from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass
from typing import Optional, Tuple

import cv2
import numpy as np


@dataclass
class VisionEstimate:
    timestamp: float
    error_norm: float
    confidence: float
    left_x: Optional[float]
    right_x: Optional[float]
    corridor_center_x: Optional[float]
    frame_width: int
    frame_height: int
    debug_frame: Optional[np.ndarray]


class CorridorVision:
    def __init__(self, ep_robot, config) -> None:
        self.robot = ep_robot
        self.config = config
        self.camera = ep_robot.camera

        self._lock = threading.Lock()
        self._estimate: Optional[VisionEstimate] = None
        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()
        self._cap = None
        self._stream_enabled = False
        self._running = False
        self._last_error = 0.0

    @property
    def running(self) -> bool:
        return self._running

    def start(self) -> bool:
        if not bool(self.config.vision_enabled):
            print("[VISION] Disabled by config.", flush=True)
            return False

        resolution = str(self.config.vision_resolution)
        robot_ip = getattr(self.robot, "ip", None)
        if not robot_ip:
            robot_ip = "192.168.2.1"

        print(
            "[VISION] Enabling RoboMaster H.264 stream at {}:{} ({})...".format(
                robot_ip,
                self.camera.conf.video_stream_port,
                resolution,
            ),
            flush=True,
        )

        try:
            # Use the SDK only for stream-control commands. Do not call
            # start_video_stream(), because that starts DJI's libmedia decoder.
            if not self.camera._stream_sdk(1, resolution):
                print("[VISION] SDK stream mode command failed.", flush=True)
                return False
            if not self.camera._video_stream(1, resolution):
                print("[VISION] SDK video stream command failed.", flush=True)
                try:
                    self.camera._stream_sdk(0, resolution)
                except Exception:
                    pass
                return False
            self._stream_enabled = True

            url = "tcp://{}:{}".format(
                robot_ip,
                int(self.camera.conf.video_stream_port),
            )

            params = []
            if hasattr(cv2, "CAP_PROP_OPEN_TIMEOUT_MSEC"):
                params.extend([cv2.CAP_PROP_OPEN_TIMEOUT_MSEC, 3000])
            if hasattr(cv2, "CAP_PROP_READ_TIMEOUT_MSEC"):
                params.extend([cv2.CAP_PROP_READ_TIMEOUT_MSEC, 1000])

            if params:
                cap = cv2.VideoCapture(url, cv2.CAP_FFMPEG, params)
            else:
                cap = cv2.VideoCapture(url, cv2.CAP_FFMPEG)

            if not cap.isOpened():
                cap.release()
                print(
                    "[VISION] OpenCV could not open {}. Falling back to ToF+odometry.".format(
                        url
                    ),
                    flush=True,
                )
                self.stop()
                return False

            try:
                cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            except Exception:
                pass

            # Confirm that actual decoded frames arrive before declaring vision
            # active. A successful TCP open alone is not enough.
            first_frame = None
            deadline = time.monotonic() + float(self.config.vision_start_timeout_sec)
            while time.monotonic() < deadline:
                ok, frame = cap.read()
                if ok and frame is not None and frame.size:
                    first_frame = frame
                    break

            if first_frame is None:
                cap.release()
                print(
                    "[VISION] Stream opened but no decoded frame arrived. "
                    "Falling back to ToF+odometry.",
                    flush=True,
                )
                self.stop()
                return False

            self._cap = cap
            self._stop.clear()
            self._running = True

            first_h, first_w = first_frame.shape[:2]
            print(
                "[VISION] First decoded camera frame: {}x{}; analyzing lines...".format(
                    first_w, first_h
                ),
                flush=True,
            )
            self._process_and_store(first_frame)
            print("[VISION] First frame analysis OK.", flush=True)

            self._thread = threading.Thread(
                target=self._capture_loop,
                name="classwork8-camera-vision",
                daemon=True,
            )
            self._thread.start()

            h, w = first_frame.shape[:2]
            print(
                "[VISION] Camera active: {}x{} OpenCV/FFmpeg direct TCP decode.".format(
                    w,
                    h,
                ),
                flush=True,
            )
            return True

        except Exception as exc:
            # Analysis errors happen after frames have already decoded. Include
            # the traceback so they cannot be mistaken for camera connection
            # or H.264 decoder failures.
            import traceback
            print(
                "[VISION] Startup/analysis failed: {}. Falling back to ToF+odometry.".format(
                    exc
                ),
                flush=True,
            )
            traceback.print_exc()
            self.stop()
            return False

    def stop(self) -> None:
        self._stop.set()
        self._running = False

        cap = self._cap
        self._cap = None
        if cap is not None:
            try:
                cap.release()
            except Exception:
                pass

        thread = self._thread
        self._thread = None
        if (
            thread is not None
            and thread.is_alive()
            and threading.current_thread() is not thread
        ):
            thread.join(timeout=1.5)

        if self._stream_enabled:
            try:
                self.camera._video_stream(0, str(self.config.vision_resolution))
            except Exception:
                pass
            try:
                self.camera._stream_sdk(0, str(self.config.vision_resolution))
            except Exception:
                pass
            self._stream_enabled = False

    def latest(self, max_age_sec: Optional[float] = None) -> Optional[VisionEstimate]:
        with self._lock:
            estimate = self._estimate

        if estimate is None:
            return None

        max_age = (
            float(self.config.vision_max_age_sec)
            if max_age_sec is None
            else float(max_age_sec)
        )
        if time.monotonic() - estimate.timestamp > max_age:
            return None
        return estimate

    def correction_mps(self) -> Tuple[float, float, float]:
        """Return (right-relative correction m/s, error_norm, confidence)."""
        estimate = self.latest()
        if estimate is None:
            return 0.0, 0.0, 0.0
        if estimate.confidence < float(self.config.vision_min_confidence):
            return 0.0, estimate.error_norm, estimate.confidence

        correction = (
            float(self.config.vision_kp_mps)
            * float(estimate.error_norm)
        )
        limit = float(self.config.vision_max_correction_mps)
        correction = max(-limit, min(limit, correction))
        return correction, estimate.error_norm, estimate.confidence

    def latest_debug_frame(self) -> Optional[np.ndarray]:
        estimate = self.latest(max_age_sec=1.0)
        if estimate is None or estimate.debug_frame is None:
            return None
        return estimate.debug_frame.copy()

    def _capture_loop(self) -> None:
        while not self._stop.is_set():
            cap = self._cap
            if cap is None:
                break

            try:
                ok, frame = cap.read()
            except Exception:
                ok, frame = False, None

            if not ok or frame is None or not frame.size:
                time.sleep(0.02)
                continue

            self._process_and_store(frame)

        self._running = False

    @staticmethod
    def _line_x_at_y(line, y_ref: float) -> Optional[float]:
        x1, y1, x2, y2 = [float(v) for v in line]
        dy = y2 - y1
        if abs(dy) < 1e-6:
            return None
        t = (float(y_ref) - y1) / dy
        return x1 + t * (x2 - x1)

    def _process_and_store(self, frame: np.ndarray) -> None:
        estimate = self._analyse(frame)
        with self._lock:
            self._estimate = estimate

    def _analyse(self, frame: np.ndarray) -> VisionEstimate:
        h, w = frame.shape[:2]
        roi_top = int(max(0, min(h - 1, h * float(self.config.vision_roi_top_ratio))))
        roi = frame[roi_top:h, :]

        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(
            gray,
            (int(self.config.vision_blur_kernel), int(self.config.vision_blur_kernel)),
            0,
        )
        edges = cv2.Canny(
            gray,
            int(self.config.vision_canny_low),
            int(self.config.vision_canny_high),
        )

        lines = cv2.HoughLinesP(
            edges,
            1,
            np.pi / 180.0,
            threshold=int(self.config.vision_hough_threshold),
            minLineLength=int(self.config.vision_min_line_length_px),
            maxLineGap=int(self.config.vision_max_line_gap_px),
        )

        y_ref = float(h - 1)
        image_center = float(w) / 2.0
        left_candidates = []
        right_candidates = []
        debug = frame.copy()

        if lines is not None:
            for packed in lines:
                # OpenCV versions differ here:
                #   HoughLinesP -> (N, 1, 4) on some builds
                #   HoughLinesP -> (N, 4)    on others
                # Flatten one result so both layouts are accepted.
                values = np.asarray(packed).reshape(-1)
                if values.size < 4:
                    continue
                x1, y1r, x2, y2r = [int(v) for v in values[:4]]
                y1 = y1r + roi_top
                y2 = y2r + roi_top
                dx = float(x2 - x1)
                dy = float(y2 - y1)

                length = math.hypot(dx, dy)
                if length < float(self.config.vision_min_line_length_px):
                    continue

                # Ignore nearly-horizontal floor/foam seams. Corridor side
                # boundaries should have a meaningful vertical component.
                angle_from_horizontal = abs(
                    math.degrees(math.atan2(dy, dx))
                )
                if angle_from_horizontal > 90.0:
                    angle_from_horizontal = 180.0 - angle_from_horizontal
                if angle_from_horizontal < float(self.config.vision_min_side_angle_deg):
                    continue

                x_bottom = self._line_x_at_y((x1, y1, x2, y2), y_ref)
                if x_bottom is None:
                    continue
                if x_bottom < -0.35 * w or x_bottom > 1.35 * w:
                    continue

                if x_bottom < image_center:
                    left_candidates.append(float(x_bottom))
                    colour = (255, 170, 0)
                else:
                    right_candidates.append(float(x_bottom))
                    colour = (0, 170, 255)

                cv2.line(debug, (x1, y1), (x2, y2), colour, 2)

        left_x = (
            float(np.median(left_candidates))
            if left_candidates
            else None
        )
        right_x = (
            float(np.median(right_candidates))
            if right_candidates
            else None
        )

        confidence = 0.0
        center_x = None
        raw_error = 0.0

        if left_x is not None and right_x is not None and right_x > left_x:
            corridor_width = right_x - left_x
            width_ratio = corridor_width / float(w)

            if (
                float(self.config.vision_min_corridor_width_ratio)
                <= width_ratio
                <= float(self.config.vision_max_corridor_width_ratio)
            ):
                center_x = (left_x + right_x) / 2.0
                raw_error = (center_x - image_center) / max(1.0, image_center)

                count_score = min(
                    1.0,
                    (len(left_candidates) + len(right_candidates)) / 8.0,
                )
                symmetry_score = min(
                    1.0,
                    min(len(left_candidates), len(right_candidates)) / 3.0,
                )
                confidence = 0.55 * count_score + 0.45 * symmetry_score

        alpha = float(self.config.vision_error_ema_alpha)
        if confidence >= float(self.config.vision_min_confidence):
            self._last_error = (
                alpha * raw_error
                + (1.0 - alpha) * self._last_error
            )
        else:
            # Decay old steering information quickly when boundaries disappear.
            self._last_error *= 0.75

        error_norm = float(self._last_error)

        cv2.rectangle(
            debug,
            (0, roi_top),
            (w - 1, h - 1),
            (110, 110, 110),
            1,
        )
        cv2.line(
            debug,
            (int(image_center), roi_top),
            (int(image_center), h - 1),
            (180, 180, 180),
            1,
        )

        if left_x is not None:
            cv2.circle(debug, (int(left_x), int(y_ref)), 6, (255, 170, 0), -1)
        if right_x is not None:
            cv2.circle(debug, (int(right_x), int(y_ref)), 6, (0, 170, 255), -1)
        if center_x is not None:
            cv2.line(
                debug,
                (int(center_x), roi_top),
                (int(center_x), h - 1),
                (0, 255, 0),
                2,
            )

        cv2.putText(
            debug,
            "vision err={:+.3f} conf={:.2f}".format(error_norm, confidence),
            (10, 24),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (0, 255, 0) if confidence >= float(self.config.vision_min_confidence)
            else (0, 180, 255),
            2,
            cv2.LINE_AA,
        )

        return VisionEstimate(
            timestamp=time.monotonic(),
            error_norm=error_norm,
            confidence=float(confidence),
            left_x=left_x,
            right_x=right_x,
            corridor_center_x=center_x,
            frame_width=w,
            frame_height=h,
            debug_frame=debug,
        )
