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
    allow_soft_recovery: bool = False,
    opposite_clearance_cm: Optional[float] = None,
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
    approaching_side = None

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
                if approaching_side is not None:
                    return False, "SIDE_RANGE_DROP_BOTH", 0.0
                approaching_side = side

    # A stationary scan of the OPPOSITE direction is essential. Never recover
    # from a short side return using cached data or only the approaching wall.
    if approaching_side is not None:
        name = "LEFT" if approaching_side == left_dir else "RIGHT"
        clearance = opposite_clearance_cm
        if (
            not allow_soft_recovery or clearance is None
            or float(clearance) <= float(soft_margin_cm)
        ):
            return False, "SIDE_RANGE_DROP_{}".format(name), 0.0
        # Hard stop above was checked first; recovery is deliberately a slow
        # bias while advancing, not a blind lateral strafe into another wall.
        away_sign = 1.0 if approaching_side == left_dir else -1.0
        return (
            True,
            "MIDCELL_SOFT_WALL_RECOVERY_AWAY_{}".format(name),
            away_sign * max(0.0, float(max_bias_mps)),
        )

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


def side_start_recovery_preflight(
    confirmed_side_cm: Optional[float],
    fresh_opposite_cm: Optional[float],
    *,
    hard_stop_cm: float,
    release_margin_cm: float,
    opposite_min_cm: float,
    step_m: float,
) -> Tuple[bool, str]:
    """Authorize ONLY an away-from-wall nudge with a fresh opposite-direction ray.

    This checks sensor-to-wall distances, not body clearance. Physical sensor
    offsets and low obstacles must be checked at the real robot. Never move
    on a missing reading, or without additional room beyond the small step.
    """
    if confirmed_side_cm is None or float(confirmed_side_cm) <= 0.0:
        return False, "RECOVERY_NO_CONFIRMED_SIDE_RANGE"
    if fresh_opposite_cm is None or float(fresh_opposite_cm) <= 0.0:
        return False, "RECOVERY_NO_FRESH_OPPOSITE_RANGE"
    if float(confirmed_side_cm) >= float(hard_stop_cm) + float(release_margin_cm):
        return False, "RECOVERY_NOT_NEEDED"
    if float(step_m) <= 0.0:
        return False, "RECOVERY_INVALID_STEP"
    required = max(
        float(opposite_min_cm),
        float(hard_stop_cm) + float(release_margin_cm) +
        float(step_m) * 100.0 + 3.0,
    )
    if float(fresh_opposite_cm) < required:
        return False, "RECOVERY_OPPOSITE_TOO_CLOSE"
    return True, "RECOVERY_SAFE_OPPOSITE_RAY"


def heading_alignment_preflight(
    yaw_error_deg: Optional[float],
    side_ranges_cm: Dict[int, Optional[float]],
    critical_side: int,
    *,
    min_error_deg: float,
    max_step_deg: float,
    max_initial_error_deg: float,
    critical_side_min_cm: float,
    other_side_min_cm: float,
) -> Tuple[bool, str, float]:
    """Plan only a small correction toward mission-start heading, never a guess.

    These are *central ToF rays*, not physical chassis-corner clearance. A
    verified chassis-swept envelope and attended field test are also required
    before enabling automatic chassis rotation in a narrow real maze.
    """
    if yaw_error_deg is None:
        return False, "HEADING_RECOVERY_NO_YAW", 0.0
    error = float(yaw_error_deg)
    if abs(error) < float(min_error_deg):
        return False, "HEADING_RECOVERY_ALREADY_ALIGNED", 0.0
    if abs(error) > float(max_initial_error_deg):
        return False, "HEADING_RECOVERY_YAW_TOO_LARGE", 0.0
    for direction in range(4):
        value = side_ranges_cm.get(direction)
        if value is None or float(value) <= 0.0:
            return False, "HEADING_RECOVERY_RANGE_UNAVAILABLE", 0.0
        required = (
            float(critical_side_min_cm)
            if direction == int(critical_side) % 4
            else float(other_side_min_cm)
        )
        if float(value) < required:
            return False, "HEADING_RECOVERY_NO_ROTATION_CLEARANCE_" + str(direction), 0.0
    correction = max(-float(max_step_deg), min(float(max_step_deg), error))
    return True, "HEADING_RECOVERY_SMALL_CORRECTION", correction
