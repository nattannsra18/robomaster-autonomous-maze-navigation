"""Single-owner RoboMaster camera stream for the Final Assignment.

Only this service opens the H.264 TCP stream.  Other vision modules consume
copies of the newest decoded frame so they never compete for the DJI stream.
"""

from __future__ import annotations

import threading
import time
from typing import Optional

import cv2
import numpy as np


class CameraService:
    def __init__(self, ep_robot, resolution: str = "360p", start_timeout_sec: float = 5.0):
        self.robot = ep_robot
        self.camera = ep_robot.camera
        self.resolution = str(resolution)
        self.start_timeout_sec = float(start_timeout_sec)

        self._lock = threading.Lock()
        self._latest_frame: Optional[np.ndarray] = None
        self._latest_timestamp = 0.0

        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._cap = None
        self._stream_enabled = False
        self._running = False

    @property
    def running(self) -> bool:
        return bool(self._running)

    def start(self) -> bool:
        robot_ip = getattr(self.robot, "ip", None) or "192.168.2.1"
        port = int(self.camera.conf.video_stream_port)

        print(
            "[CAMERA] Opening shared RoboMaster stream {}:{} ({})...".format(
                robot_ip,
                port,
                self.resolution,
            ),
            flush=True,
        )

        try:
            if not self.camera._stream_sdk(1, self.resolution):
                print("[CAMERA] SDK stream-mode command failed.", flush=True)
                return False

            if not self.camera._video_stream(1, self.resolution):
                try:
                    self.camera._stream_sdk(0, self.resolution)
                except Exception:
                    pass
                print("[CAMERA] SDK video-stream command failed.", flush=True)
                return False

            self._stream_enabled = True
            url = "tcp://{}:{}".format(robot_ip, port)

            params = []
            if hasattr(cv2, "CAP_PROP_OPEN_TIMEOUT_MSEC"):
                params.extend([cv2.CAP_PROP_OPEN_TIMEOUT_MSEC, 3000])
            if hasattr(cv2, "CAP_PROP_READ_TIMEOUT_MSEC"):
                params.extend([cv2.CAP_PROP_READ_TIMEOUT_MSEC, 1000])

            try:
                if params:
                    cap = cv2.VideoCapture(url, cv2.CAP_FFMPEG, params)
                else:
                    cap = cv2.VideoCapture(url, cv2.CAP_FFMPEG)
            except TypeError:
                cap = cv2.VideoCapture(url, cv2.CAP_FFMPEG)

            if not cap.isOpened():
                cap.release()
                self.stop()
                print("[CAMERA] OpenCV could not open {}.".format(url), flush=True)
                return False

            try:
                cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            except Exception:
                pass

            first_frame = None
            deadline = time.monotonic() + self.start_timeout_sec

            while time.monotonic() < deadline:
                ok, frame = cap.read()
                if ok and frame is not None and frame.size:
                    first_frame = frame
                    break

            if first_frame is None:
                cap.release()
                self.stop()
                print("[CAMERA] Stream opened but no decoded frame arrived.", flush=True)
                return False

            self._cap = cap
            self._stop.clear()
            self._running = True

            with self._lock:
                self._latest_frame = first_frame.copy()
                self._latest_timestamp = time.monotonic()

            self._thread = threading.Thread(
                target=self._capture_loop,
                name="final-shared-camera",
                daemon=True,
            )
            self._thread.start()

            h, w = first_frame.shape[:2]
            print("[CAMERA] Shared stream active: {}x{}.".format(w, h), flush=True)
            return True

        except Exception as exc:
            print("[CAMERA] Startup failed: {}".format(exc), flush=True)
            self.stop()
            return False

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
                time.sleep(0.005)
                continue

            with self._lock:
                self._latest_frame = frame.copy()
                self._latest_timestamp = time.monotonic()

        self._running = False

    def latest(self, max_age_sec: float = 0.6) -> Optional[np.ndarray]:
        with self._lock:
            frame = self._latest_frame
            timestamp = self._latest_timestamp

        if frame is None:
            return None

        if time.monotonic() - timestamp > float(max_age_sec):
            return None

        return frame.copy()

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
                self.camera._video_stream(0, self.resolution)
            except Exception:
                pass
            try:
                self.camera._stream_sdk(0, self.resolution)
            except Exception:
                pass
            self._stream_enabled = False
