"""Conservative target plotting for the Final Round-1 discovered GUI map.

A distant camera observation is a bearing, not a confirmed target location.
Show its ToF-ray endpoint only as a hollow '?' hint with a line of sight from
the observing cell. Never place it at the observing cell as a physical sign.
"""

from __future__ import annotations

from typing import Optional, Tuple


_DIR_VEC = {
    0: (1.0, 0.0),   # FRONT
    1: (0.0, -1.0),  # RIGHT
    2: (-1.0, 0.0),  # BACK
    3: (0.0, 1.0),   # LEFT
}


def target_plot_geometry(target: dict, cell_size_m: float):
    """Return (plot_xy, origin_xy_or_none, sighting_only).

    A nearby wall-surface projection is still an estimate, not verified
    target depth. For camera-only detections, the plot is an explicitly
    provisional line-of-sight hint: a measured ToF end point, or one-cell
    direction indicator if ToF was unavailable.
    """
    sighting_only = (
        target.get("localization_status") == "SIGHTING_ONLY"
        or not target.get("range_confirmed_wall", False)
    )
    xy = target.get("estimated_target_xy_m")
    if not sighting_only and xy is not None and len(xy) == 2:
        return (float(xy[0]), float(xy[1])), None, False

    cells = target.get("observation_cells") or target.get("approach_cells") or []
    if not cells:
        return None, None, True

    source = cells[0]
    origin = (
        float(source[0]) * float(cell_size_m),
        float(source[1]) * float(cell_size_m),
    )
    direction_list = target.get("view_directions") or [0]
    direction = int(direction_list[0]) % 4
    # A SIGHTING_ONLY bearing has NO physical position. Low foam walls can
    # let ToF see far beyond the arena, so drawing its distant range-derived
    # cell hint as a target creates a false exterior map marker. Keep the
    # original hint in targets.json, but put a hollow '?' inside the observed
    # cell at its visible edge. A real close-wall recheck can localize it later.
    dx, dy = _DIR_VEC[direction]
    endpoint = (
        origin[0] + dx * float(cell_size_m) * 0.45,
        origin[1] + dy * float(cell_size_m) * 0.45,
    )
    return endpoint, origin, True


def confirmed_map_targets(targets):
    """Return only camera-confirmed signs for map rendering/export.

    PENDING_RECHECK is useful for internal revisit logic and targets.json,
    but is not a confirmed sign and must not clutter the logical map.
    Confirmed SIGHTING_ONLY observations remain visible as hollow bearing
    markers; their position is not represented as a verified coordinate.
    """
    return [
        target for target in (targets or [])
        if target.get("status") != "PENDING_RECHECK"
        and target.get("confirmed") is not False
    ]


def target_marker_offsets(targets, cell_size_m: float):
    """Pure display-only offsets, in logical-cell widths, for shared ToF rays.

    Multiple signs on the same wall can have exactly the same 2-D ToF range
    estimate. Spread badges so none disappears. Never alter targets.json or
    imply these badge offsets are triangulated physical coordinates.
    """
    import math

    entries = list(targets or [])
    offsets = [(0.0, 0.0) for _ in entries]
    groups = {}
    for index, target in enumerate(entries):
        xy, _, _ = target_plot_geometry(target, cell_size_m)
        if xy is None:
            continue
        # Only symbols at nearly identical physical plot positions collide.
        key = (round(xy[0], 3), round(xy[1], 3))
        groups.setdefault(key, []).append(index)

    for indices in groups.values():
        n = len(indices)
        if n <= 1:
            continue
        columns = int(math.ceil(math.sqrt(n)))
        rows = int(math.ceil(n / columns))
        for slot, index in enumerate(indices):
            col, row = slot % columns, slot // columns
            offsets[index] = (
                (col - (columns - 1) / 2.0) * 0.36,
                (row - (rows - 1) / 2.0) * 0.36,
            )
    return offsets
