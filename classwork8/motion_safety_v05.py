"""Conservative ToF-only lateral checks for a 60 cm mecanum maze cell.

This module has no RoboMaster imports and does not issue motor commands.
It reports a safe/unsafe checkpoint decision and a *small* travel-frame
right-positive bias from wall observations made only while the chassis stops.

A single forward-facing ToF is not a continuous side-clearance sensor.
"""

from __future__ import annotations

from typing import Dict, Optional, Set, Tuple


def adjacent_wall_sides(
    direction: int,
    current_cell: Tuple[int, int],
    target_cell: Tuple[int, int],
    edge_states: Dict[Tuple[int, int, int], str],
) -> Set[int]:
    """Identify only confirmed wall sides beside the current one-cell leg."""
    left_dir = (int(direction) - 1) % 4
    right_dir = (int(direction) + 1) % 4
    return {
        side
        for side in (left_dir, right_dir)
        if any(
            edge_states.get((cell[0], cell[1], side)) == "WALL"
            for cell in (current_cell, target_cell)
        )
    }


def side_checkpoint_decision(
    direction: int,
    wall_sides: Set[int],
    readings_cm: Dict[int, Optional[float]],
    *,
    hard_stop_cm: float,
    soft_margin_cm: float,
    wall_max_cm: float,
    gain_mps_per_cm: float,
    max_bias_mps: float,
) -> Tuple[bool, str, float]:
    """Evaluate a stationary side scan: (may_continue, label, right_bias).

    A dangerously short return makes further travel unsafe. Missing or long
    side returns are not blindly treated as clearance and never generate
    an automatic lateral steering command.
    """
    left_dir = (int(direction) - 1) % 4
    right_dir = (int(direction) + 1) % 4

    valid = {}
    for side in wall_sides:
        value = readings_cm.get(side)
        if value is None or value <= 0:
            continue
        if float(value) <= float(hard_stop_cm):
            return False, "SIDE_CLEARANCE_LOW_{}".format(
                "LEFT" if side == left_dir else "RIGHT"
            ), 0.0
        if float(value) <= float(wall_max_cm):
            valid[side] = float(value)

    if not valid:
        return True, "SIDE_WALL_NOT_VISIBLE_NO_AUTO_STEER", 0.0

    max_bias = max(0.0, float(max_bias_mps))
    gain = max(0.0, float(gain_mps_per_cm))
    bias = 0.0

    if left_dir in valid and right_dir in valid:
        # Left wall closer -> move right (positive travel-relative correction).
        error_cm = valid[right_dir] - valid[left_dir]
        bias = error_cm * gain
        label = "MIDCELL_BETWEEN_WALLS"
    elif left_dir in valid and valid[left_dir] < float(soft_margin_cm):
        bias = (float(soft_margin_cm) - valid[left_dir]) * gain
        label = "MIDCELL_BIAS_AWAY_LEFT"
    elif right_dir in valid and valid[right_dir] < float(soft_margin_cm):
        bias = -(float(soft_margin_cm) - valid[right_dir]) * gain
        label = "MIDCELL_BIAS_AWAY_RIGHT"
    else:
        label = "MIDCELL_CLEARANCE_OK"

    return True, label, max(-max_bias, min(max_bias, bias))
