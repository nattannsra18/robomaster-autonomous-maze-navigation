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
    baseline_cm: Optional[Dict[int, Optional[float]]] = None,
    max_baseline_drop_cm: float = 4.0,
    recenter_deadband_cm: float = 1.5,
) -> Tuple[bool, str, float]:
    """Evaluate a stationary side scan: (may_continue, label, right_bias).

    ToF-to-wall distance is not the robot body's wall clearance. The old
    default absolute stop of 22 cm exceeded the user's physically normal
    right reading (14.5 cm), and killed a run at a stable 14.7 cm.

    Stop on an independently calibrated CRITICAL sensor return or when a
    wall has become meaningfully closer relative to its freshly scanned
    start-of-leg baseline. Never infer a centering target of 28 cm from a
    single wall, and never steer using a missing/stale baseline.
    """
    left_dir = (int(direction) - 1) % 4
    right_dir = (int(direction) + 1) % 4
    baseline_cm = baseline_cm or {}
    valid = {}
    reference = {}

    for side in wall_sides:
        value = readings_cm.get(side)
        if value is None or float(value) <= 0.0:
            continue
        distance = float(value)
        name = "LEFT" if side == left_dir else "RIGHT"
        if distance <= float(hard_stop_cm):
            return False, "SIDE_CRITICAL_RANGE_{}".format(name), 0.0

        if distance > float(wall_max_cm):
            # A missing wall echo cannot justify lateral correction.
            continue

        valid[side] = distance
        baseline = baseline_cm.get(side)
        if baseline is not None and 0.0 < float(baseline) <= float(wall_max_cm):
            baseline = float(baseline)
            reference[side] = baseline
            if baseline - distance >= float(max_baseline_drop_cm):
                return False, "SIDE_RANGE_DROP_{}".format(name), 0.0

    if not valid:
        return True, "SIDE_WALL_NOT_VISIBLE_NO_AUTO_STEER", 0.0

    max_bias = max(0.0, float(max_bias_mps))
    gain = max(0.0, float(gain_mps_per_cm))
    deadband = max(0.0, float(recenter_deadband_cm))
    bias = 0.0

    if left_dir in reference and right_dir in reference:
        # Preserve the *starting* side-distance balance, not a made-up
        # absolute 28 cm clearance on each side.
        left_change = valid[left_dir] - reference[left_dir]
        right_change = valid[right_dir] - reference[right_dir]
        error_cm = right_change - left_change
        if abs(error_cm) <= deadband:
            return True, "MIDCELL_SIDE_BASELINE_STABLE", 0.0
        bias = error_cm * gain
        label = "MIDCELL_RESTORE_SIDE_BALANCE"
    elif left_dir in reference:
        loss_cm = reference[left_dir] - valid[left_dir]
        if loss_cm <= deadband:
            return True, "MIDCELL_SIDE_BASELINE_STABLE", 0.0
        bias = loss_cm * gain
        label = "MIDCELL_BIAS_AWAY_LEFT"
    elif right_dir in reference:
        loss_cm = reference[right_dir] - valid[right_dir]
        if loss_cm <= deadband:
            return True, "MIDCELL_SIDE_BASELINE_STABLE", 0.0
        bias = -loss_cm * gain
        label = "MIDCELL_BIAS_AWAY_RIGHT"
    else:
        return True, "MIDCELL_NO_BASELINE_NO_AUTO_STEER", 0.0

    return True, label, max(-max_bias, min(max_bias, bias))


def critical_start_side_recheck(
    values_cm: Tuple[Optional[float], Optional[float]],
    *,
    hard_stop_cm: float,
    release_margin_cm: float,
    max_spread_cm: float,
) -> Tuple[bool, str, Optional[float]]:
    """Require two independent stationary fresh returns before clearing a stop.

    Returns (cleared, diagnostic, confirmed_range_cm). A single unexpectedly
    high reflection must NOT override an earlier low wall reading. The guard
    remains active whenever samples are absent, disagree or stay too close.
    """
    first, second = values_cm
    if first is None or second is None:
        return False, "SIDE_START_RECHECK_NO_FRESH_TOF", None
    a, b = float(first), float(second)
    if a <= 0.0 or b <= 0.0:
        return False, "SIDE_START_RECHECK_INVALID_TOF", None
    if abs(a - b) > float(max_spread_cm):
        return False, "SIDE_START_RECHECK_INCONSISTENT", None
    confirmed = (a + b) / 2.0
    if min(a, b) < float(hard_stop_cm) + float(release_margin_cm):
        return False, "SIDE_START_CRITICAL_CONFIRMED", confirmed
    return True, "SIDE_START_TRANSIENT_CLEARED", confirmed


def bound_travel_lateral(
    x_cmd: float,
    y_cmd: float,
    travel_right: Tuple[float, float],
    max_lateral_mps: float,
) -> Tuple[float, float]:
    """Limit combined odometry+scan lateral command without altering travel.

    Unit travel-right vectors in DIR_RIGHT_VEC_DRIVE are orthogonal to the
    commanded longitudinal direction for FRONT/RIGHT/BACK/LEFT.
    """
    rx, ry = travel_right
    lateral = x_cmd * rx + y_cmd * ry
    bounded = max(
        -float(max_lateral_mps),
        min(float(max_lateral_mps), float(lateral)),
    )
    difference = bounded - lateral
    return x_cmd + rx * difference, y_cmd + ry * difference
