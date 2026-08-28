"""Run artifact exporters: JSON, final map SVG, and sensor-history graph SVG."""
from __future__ import annotations

import json
import math
from datetime import datetime
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Tuple

from .configuration import DIR_DELTA, HybridConfig
from .grid_map import GridMazeMap
from .version import PROGRAM_VERSION


def _edge_coordinates(
    r: int,
    c: int,
    direction: int,
    margin: float,
    cell: float,
) -> Tuple[float, float, float, float]:
    x0, y0 = margin + c * cell, margin + r * cell
    if direction == 0:
        return x0, y0, x0 + cell, y0
    if direction == 1:
        return x0 + cell, y0, x0 + cell, y0 + cell
    if direction == 2:
        return x0, y0 + cell, x0 + cell, y0 + cell
    return x0, y0, x0, y0 + cell


def _physical_edge_key(
    maze: GridMazeMap,
    edge: Iterable[int],
) -> Tuple[Tuple[int, int], Tuple[int, int]]:
    r, c, direction = (int(value) for value in edge)
    return tuple(sorted(((r, c), maze.neighbour((r, c), direction))))


def _write_map_svg(path: Path, maze: GridMazeMap, snapshot: dict) -> None:
    cell = 70
    margin = 42
    width = margin * 2 + maze.cols * cell
    height = margin * 2 + maze.rows * cell
    svg = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        '<rect width="100%" height="100%" fill="#f8fafc"/>',
        '<style>text{font-family:Arial,sans-serif}.grid{stroke:#cbd5e1;stroke-width:1}.wall{stroke:#111827;stroke-width:7;stroke-linecap:round}.seen{stroke:#dc2626;stroke-width:7;stroke-linecap:round}.route{stroke:#2563eb;stroke-width:4;fill:none;stroke-linejoin:round;stroke-linecap:round}</style>',
    ]
    for r in range(maze.rows + 1):
        y = margin + r * cell
        svg.append(
            f'<line class="grid" x1="{margin}" y1="{y}" '
            f'x2="{margin + maze.cols * cell}" y2="{y}"/>'
        )
    for c in range(maze.cols + 1):
        x = margin + c * cell
        svg.append(
            f'<line class="grid" x1="{x}" y1="{margin}" '
            f'x2="{x}" y2="{margin + maze.rows * cell}"/>'
        )

    if snapshot["travel_path"]:
        points = " ".join(
            f"{margin + (c + 0.5) * cell},{margin + (r + 0.5) * cell}"
            for r, c in snapshot["travel_path"]
        )
        svg.append(f'<polyline class="route" points="{points}"/>')

    # Merge by physical edge. Sensor evidence wins visually over a drawn wall,
    # so a confirmed red wall never becomes black merely because both exist.
    walls: Dict[Tuple[Tuple[int, int], Tuple[int, int]], Tuple[List[int], bool]] = {}
    for edge in snapshot["manual_walls"]:
        walls[_physical_edge_key(maze, edge)] = (edge, False)
    for edge in snapshot["sensor_walls"]:
        walls[_physical_edge_key(maze, edge)] = (edge, True)
    for edge, sensed in walls.values():
        r, c, direction = edge
        x1, y1, x2, y2 = _edge_coordinates(r, c, direction, margin, cell)
        css = "seen" if sensed else "wall"
        svg.append(
            f'<line class="{css}" x1="{x1}" y1="{y1}" x2="{x2}" y2="{y2}"/>'
        )

    markers = (
        ("start", "S", "#16a34a"),
        ("drop", "D", "#f59e0b"),
        ("exit", "E", "#7c3aed"),
    )
    for name, label, colour in markers:
        value = snapshot[name]
        if value is None:
            continue
        r, c = value
        cx, cy = margin + (c + 0.5) * cell, margin + (r + 0.5) * cell
        svg.append(f'<circle cx="{cx}" cy="{cy}" r="16" fill="{colour}"/>')
        svg.append(
            f'<text x="{cx}" y="{cy + 6}" font-size="17" text-anchor="middle" '
            f'fill="white" font-weight="bold">{label}</text>'
        )
    svg.append(
        f'<text x="{margin}" y="26" font-size="16" fill="#0f172a">'
        f'Status: {snapshot["status"]}</text>'
    )
    svg.append("</svg>")
    path.write_text("\n".join(svg), encoding="utf-8")


def _polyline_points(
    history: List[dict],
    key: str,
    x_of,
    y_of,
) -> List[str]:
    groups: List[str] = []
    current: List[str] = []
    for index, row in enumerate(history):
        value = row.get(key)
        if value is None:
            if len(current) >= 2:
                groups.append(" ".join(current))
            current = []
            continue
        current.append(f"{x_of(index):.2f},{y_of(float(value)):.2f}")
    if len(current) >= 2:
        groups.append(" ".join(current))
    return groups


def _write_sensor_graph_svg(
    path: Path,
    config: HybridConfig,
    snapshot: dict,
) -> None:
    history = list(snapshot.get("sensor_history", []))
    width, height = 1100, 600
    left, right, top, bottom = 82, 34, 62, 92
    plot_w, plot_h = width - left - right, height - top - bottom
    values = [
        float(row[key])
        for row in history
        for key in ("front_cm", "left_cm", "right_cm")
        if row.get(key) is not None
    ]
    thresholds = [
        float(config.front_wall_cm),
        float(config.side_topology_open_cm),
        float(config.hard_stop_front_cm),
    ]
    y_max = max([60.0, *values, *thresholds])
    y_max = math.ceil((y_max * 1.10) / 10.0) * 10.0
    sample_max = max(1, len(history) - 1)

    def x_of(index: int) -> float:
        return left + (index / sample_max) * plot_w

    def y_of(value: float) -> float:
        return top + plot_h - (max(0.0, min(value, y_max)) / y_max) * plot_h

    svg = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        '<rect width="100%" height="100%" fill="#ffffff"/>',
        '<style>text{font-family:Arial,sans-serif}.axis{stroke:#334155;stroke-width:1.5}.grid{stroke:#e2e8f0;stroke-width:1}.label{fill:#334155;font-size:13px}.title{fill:#0f172a;font-size:20px;font-weight:bold}.note{fill:#64748b;font-size:12px}</style>',
        f'<text class="title" x="{left}" y="30">Sensor distance history after mission</text>',
        f'<text class="note" x="{left}" y="49">Each sample is a centred cell scan used to update the red-wall map.</text>',
    ]

    for tick in range(0, int(y_max) + 1, 10):
        y = y_of(float(tick))
        svg.append(f'<line class="grid" x1="{left}" y1="{y}" x2="{left + plot_w}" y2="{y}"/>')
        svg.append(f'<text class="label" x="{left - 12}" y="{y + 5}" text-anchor="end">{tick}</text>')
    svg.append(f'<line class="axis" x1="{left}" y1="{top}" x2="{left}" y2="{top + plot_h}"/>')
    svg.append(f'<line class="axis" x1="{left}" y1="{top + plot_h}" x2="{left + plot_w}" y2="{top + plot_h}"/>')
    svg.append(f'<text class="label" x="22" y="{top + plot_h / 2}" transform="rotate(-90 22 {top + plot_h / 2})" text-anchor="middle">Distance (cm)</text>')
    svg.append(f'<text class="label" x="{left + plot_w / 2}" y="{height - 22}" text-anchor="middle">Cell scan sequence</text>')

    threshold_specs = (
        (config.front_wall_cm, "#f59e0b", "Front wall threshold"),
        (config.side_topology_open_cm, "#a855f7", "Side open threshold"),
        (config.hard_stop_front_cm, "#dc2626", "Hard stop"),
    )
    for value, colour, label in threshold_specs:
        y = y_of(float(value))
        svg.append(f'<line x1="{left}" y1="{y}" x2="{left + plot_w}" y2="{y}" stroke="{colour}" stroke-width="1.5" stroke-dasharray="7 5" opacity="0.7"/>')
        svg.append(f'<text x="{left + plot_w - 4}" y="{y - 5}" text-anchor="end" fill="{colour}" font-size="11">{label} {float(value):.1f} cm</text>')

    series = (
        ("front_cm", "#2563eb", "Front ToF"),
        ("left_cm", "#16a34a", "Left Sharp"),
        ("right_cm", "#ef4444", "Right Sharp"),
    )
    for key, colour, _label in series:
        for points in _polyline_points(history, key, x_of, y_of):
            svg.append(f'<polyline points="{points}" fill="none" stroke="{colour}" stroke-width="3" stroke-linejoin="round" stroke-linecap="round"/>')
        for index, row in enumerate(history):
            if row.get(key) is not None:
                svg.append(f'<circle cx="{x_of(index):.2f}" cy="{y_of(float(row[key])):.2f}" r="3" fill="{colour}"/>')

    legend_x = left
    for _key, colour, label in series:
        svg.append(f'<line x1="{legend_x}" y1="{height - 52}" x2="{legend_x + 28}" y2="{height - 52}" stroke="{colour}" stroke-width="4"/>')
        svg.append(f'<text class="label" x="{legend_x + 35}" y="{height - 47}">{label}</text>')
        legend_x += 180

    if not history:
        svg.append(f'<text x="{left + plot_w / 2}" y="{top + plot_h / 2}" text-anchor="middle" fill="#64748b" font-size="18">No sensor snapshots were recorded.</text>')
    else:
        tick_step = max(1, len(history) // 12)
        for index, row in enumerate(history):
            if index % tick_step != 0 and index != len(history) - 1:
                continue
            cell = row.get("cell", ["?", "?"])
            label = f'{index} ({cell[0]},{cell[1]})'
            x = x_of(index)
            svg.append(f'<line x1="{x}" y1="{top + plot_h}" x2="{x}" y2="{top + plot_h + 6}" stroke="#334155"/>')
            svg.append(f'<text class="note" x="{x}" y="{top + plot_h + 20}" text-anchor="middle" transform="rotate(35 {x} {top + plot_h + 20})">{label}</text>')

    svg.append("</svg>")
    path.write_text("\n".join(svg), encoding="utf-8")


def export_run_artifacts(
    config: HybridConfig,
    maze: GridMazeMap,
) -> Tuple[Path, Path, Path]:
    """Save JSON, the final maze map, and a sensor-distance graph."""
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    prefix = Path(f"{config.output_prefix}_{stamp}").resolve()
    json_path = prefix.with_suffix(".json")
    map_svg_path = prefix.with_name(prefix.name + "_map.svg")
    graph_svg_path = prefix.with_name(prefix.name + "_sensor_graph.svg")
    snapshot = maze.to_dict()
    payload = {
        "program_version": PROGRAM_VERSION,
        "created_at": datetime.now().isoformat(timespec="seconds"),
        "config": config.to_dict(),
        "map": snapshot,
    }
    json_path.write_text(
        json.dumps(payload, indent=2, ensure_ascii=False),
        encoding="utf-8",
    )
    _write_map_svg(map_svg_path, maze, snapshot)
    _write_sensor_graph_svg(graph_svg_path, config, snapshot)
    return json_path, map_svg_path, graph_svg_path
