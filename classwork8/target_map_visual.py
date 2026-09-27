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
    # A distant wall's candidate cell is the last cell before its measured
    # range plane. Show an OPEN circle at its centre (not as a confirmed target).
    # Retain actual ToF ray end in targets.json for later geometric validation.
    cell_hint = target.get("sighting_cell_hint")
    if cell_hint is not None and len(cell_hint) == 2:
        endpoint = (
            float(cell_hint[0]) * float(cell_size_m),
            float(cell_hint[1]) * float(cell_size_m),
        )
    else:
        dx, dy = _DIR_VEC[direction]
        endpoint = (
            origin[0] + dx * float(cell_size_m),
            origin[1] + dy * float(cell_size_m),
        )
    return endpoint, origin, True
