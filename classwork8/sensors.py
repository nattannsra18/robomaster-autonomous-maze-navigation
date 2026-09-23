from __future__ import annotations

import statistics
from collections import deque
from typing import Optional, Tuple

from robomaster_mission.mission import (
    CALIBRATION_SHARP_LEFT,
    CALIBRATION_SHARP_RIGHT,
    SensorManager as LegacySensorManager,
)

from .config import Classwork8Config


class Classwork8SensorManager(LegacySensorManager):
    """Legacy filtering with the actual Classwork 8 CAN-hub wiring."""

    def __init__(self, sensor_adapter, config: Classwork8Config):
        super().__init__(sensor_adapter)
        self.config = config
        self.ir_left_hist = deque(maxlen=config.ir_confirm_samples)
        self.ir_right_hist = deque(maxlen=config.ir_confirm_samples)

    def _read_sharp_port(self, hub_id: int, *, left: bool):
        try:
            raw = float(self.sensor_adapter.get_adc(id=hub_id, port=self.config.sharp_port))
        except Exception as exc:
            print(f"Sharp hub {hub_id} read error: {exc}")
            return 0, None

        buf = self.left_adc_buf if left else self.right_adc_buf
        buf.append(raw)
        med = statistics.median(buf)
        table = CALIBRATION_SHARP_LEFT if left else CALIBRATION_SHARP_RIGHT
        cm = self.adc_to_cm(med, table)
        return int(round(raw)), float(cm)

    def read_left(self):
        return self._read_sharp_port(self.config.left_hub_id, left=True)

    def read_right(self):
        return self._read_sharp_port(self.config.right_hub_id, left=False)

    def _read_ir_port(self, hub_id: int, history: deque) -> Optional[bool]:
        try:
            level = self.sensor_adapter.get_io(id=hub_id, port=self.config.ir_port)
        except Exception as exc:
            print(f"IR hub {hub_id} read error: {exc}")
            return None
        if level is None:
            return None
        blocked = int(level) == int(self.config.ir_blocked_level)
        history.append(bool(blocked))
        if len(history) < history.maxlen:
            return None
        return all(history)

    def read_front_corner_ir(self) -> Tuple[Optional[bool], Optional[bool]]:
        left = self._read_ir_port(self.config.left_hub_id, self.ir_left_hist)
        right = self._read_ir_port(self.config.right_hub_id, self.ir_right_hist)
        return left, right
