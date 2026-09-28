"""V05 stationary four-direction clearance planner (pure, testable).

Directions: 0 FRONT, 1 RIGHT, 2 BACK, 3 LEFT in ROBOT frame.
Ranges are horizontal ToF sensor-to-wall centimetres, not chassis-edge gaps.
One gimbal ToF cannot see four directions simultaneously. Call only with
the current scan direction and a fresh opposite-direction safety range
(from this unmoved scan epoch or an immediate short opposite probe).
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, Optional


DIRECTIONS = ("FRONT", "RIGHT", "BACK", "LEFT")


@dataclass(frozen=True)
class ClearancePlan:
    wall_direction: int
    away_direction: int
    measured_cm: float
    target_cm: float
    opposite_cm: float
    shift_cm: float


def clearance_target(config, direction: int) -> float:
    return float(getattr(config, "wall_clearance_{}_cm".format(
        DIRECTIONS[int(direction) % 4].lower()
    )))


def _valid_wall_reading(value, config) -> bool:
    return (
        value is not None
        and math.isfinite(float(value))
        and float(config.mapping_min_cm) <= float(value)
        and float(value) < float(config.tof_open_cm)
    )


def choose_clearance_plan(
    ranges: Dict[int, Optional[float]], config
) -> Optional[ClearancePlan]:
    """Choose a feasible bounded shift away from a near wall.

    Caller supplies just the current direction and its opposite. A valid
    open-range opposite side is spacious but still bounded by max_step.
    Missing/invalid opposite range means no safe motion. A narrow pair
    (both too close) is skipped, never forced.
    """
    candidates = []
    tol = float(config.wall_clearance_deadband_cm)
    max_step = float(config.wall_clearance_max_step_cm)
    for side in (0, 1, 2, 3):
        opposite = (side + 2) % 4
        near = ranges.get(side)
        far = ranges.get(opposite)
        if not _valid_wall_reading(near, config):
            continue
        if far is None or not math.isfinite(float(far)):
            continue
        if float(far) < float(config.mapping_min_cm):
            continue
        desired = clearance_target(config, side)
        deficit = desired - float(near)
        if deficit <= tol:
            continue
        # All directions share the one-cell snapshot. Reserve the opposing
        # target plus tolerance, even if it is currently classified OPEN.
        opposing_headroom = float(far) - clearance_target(config, opposite) - tol
        if opposing_headroom <= tol:
            continue
        shift = min(deficit, opposing_headroom, max_step)
        if shift <= tol:
            continue
        candidates.append((
            deficit,
            ClearancePlan(side, opposite, float(near), desired,
                          float(far), shift)
        ))
    if not candidates:
        return None
    return max(candidates, key=lambda pair: pair[0])[1]
