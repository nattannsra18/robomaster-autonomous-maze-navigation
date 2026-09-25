"""Realtime discovered-grid GUI for Final Round 1 V05.

V04 also exports the logical GUI map as a PNG automatically when a run ends.


The visual map is intentionally local/unknown-world:
- no field dimensions are known in advance
- start is logical cell (0, 0)
- FRONT at mission start is drawn upward
- RIGHT is drawn to the right
- cells/walls appear only as the robot discovers them

The fine 5 cm occupancy grid still runs in the mapper/export pipeline, but this
GUI presents the 60 cm logical maze cells in the same visual style as the main
fixed-grid GUI.
"""

from __future__ import annotations

import math
import queue
import threading
import traceback
from datetime import datetime
from pathlib import Path
from typing import Callable


class RealtimeMapGUI:
    COLOURS = {
        "background": "#f8fafc",
        "panel": "#ffffff",
        "canvas": "#eef2f7",
        "cell": "#ffffff",
        "grid": "#cbd5e1",
        "wall": "#111827",
        "travel": "#2563eb",
        "route": "#93c5fd",
        "planner": "#7c3aed",
        "start": "#16a34a",
        "robot": "#dc2626",
        "gimbal": "#f59e0b",
        "unknown_text": "#94a3b8",
        "text": "#0f172a",
        "muted": "#64748b",
    }

    def __init__(
        self,
        stop_event: threading.Event,
        canvas_px: int = 720,
        refresh_ms: int = 150,
        output_dir: str = "classwork8_output",
        auto_save_map: bool = True,
        export_width_px: int = 1200,
        export_height_px: int = 900,
    ):
        import tkinter as tk
        from tkinter import ttk
        from PIL import Image, ImageDraw, ImageFont, ImageTk

        self.tk = tk
        self.ttk = ttk
        self.Image = Image
        self.ImageDraw = ImageDraw
        self.ImageFont = ImageFont
        self.ImageTk = ImageTk
        self.stop_event = stop_event
        self.refresh_ms = int(refresh_ms)
        self.output_dir = Path(output_dir)
        self.auto_save_map = bool(auto_save_map)
        self.export_width_px = int(export_width_px)
        self.export_height_px = int(export_height_px)
        self._auto_saved_run_dirs = set()
        self._last_saved_path = None
        self._queue = queue.Queue()
        self._latest = None
        self._vision_photo = None

        self.root = tk.Tk()
        self.root.title("Final Round 1 V05 - Frontier Exploration")
        self.root.geometry("1120x840")
        self.root.minsize(920, 700)
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

        self.root.configure(background=self.COLOURS["background"])

        outer = ttk.Frame(self.root, padding=10)
        outer.pack(fill="both", expand=True)

        left = ttk.Frame(outer)
        left.pack(side="left", fill="both", expand=True)

        right = ttk.Frame(outer, width=340)
        right.pack(side="right", fill="y", padx=(14, 0))
        right.pack_propagate(False)

        self.canvas_px = int(canvas_px)
        self.canvas = tk.Canvas(
            left,
            width=self.canvas_px,
            height=self.canvas_px,
            bg=self.COLOURS["canvas"],
            highlightthickness=1,
            highlightbackground="#94a3b8",
        )
        self.canvas.pack(fill="both", expand=True)

        ttk.Label(
            right,
            text="Final Round 1 V05",
            font=("Segoe UI", 16, "bold"),
        ).pack(anchor="w")
        ttk.Label(
            right,
            text="ToF map + camera color/shape target survey",
            font=("Segoe UI", 10),
        ).pack(anchor="w", pady=(0, 14))

        self.status_var = tk.StringVar(value="Starting...")
        self.pose_var = tk.StringVar(value="Pose: --")
        self.cell_var = tk.StringVar(value="Cell: --")
        self.tof_var = tk.StringVar(value="ToF: --")
        self.gimbal_var = tk.StringVar(value="Gimbal: --")
        self.vision_var = tk.StringVar(value="Camera: starting...")
        self.target_var = tk.StringVar(value="Targets: 0")
        self.moves_var = tk.StringVar(value="Moves: 0")
        self.discovered_var = tk.StringVar(value="Discovered cells: 1")
        self.coverage_var = tk.StringVar(value="Occupancy coverage: 0.00%")
        self.planner_var = tk.StringVar(value="Planner: FRONTIER_BFS")
        self.frontier_var = tk.StringVar(value="Frontiers: 0")
        self.planner_target_var = tk.StringVar(value="Planner target: --")
        self.completion_var = tk.StringVar(value="Completion: evaluating...")
        self.reason_var = tk.StringVar(value="")

        for variable in (
            self.status_var,
            self.pose_var,
            self.cell_var,
            self.tof_var,
            self.gimbal_var,
            self.vision_var,
            self.target_var,
            self.moves_var,
            self.discovered_var,
            self.coverage_var,
            self.planner_var,
            self.frontier_var,
            self.planner_target_var,
            self.completion_var,
        ):
            ttk.Label(
                right,
                textvariable=variable,
                wraplength=280,
            ).pack(anchor="w", pady=3)

        ttk.Separator(right, orient="horizontal").pack(fill="x", pady=12)

        ttk.Label(
            right,
            text="Camera target detection",
            font=("Segoe UI", 10, "bold"),
        ).pack(anchor="w", pady=(0, 5))

        self.vision_preview = ttk.Label(
            right,
            text="Waiting for camera...",
            anchor="center",
        )
        self.vision_preview.pack(fill="x", pady=(0, 8))

        ttk.Separator(right, orient="horizontal").pack(fill="x", pady=12)

        ttk.Label(
            right,
            text=(
                "Map legend\n"
                "Thin gray = discovered 60 cm cell\n"
                "Thick black = wall detected by ToF\n"
                "Blue = realtime odometry trajectory\n"
                "Light blue = travelled logical path\n"
                "Purple dashed = planned route to nearest frontier\n"
                "Colored Txx = detected target position estimate\n"
                "Green S = mission start\n"
                "Red R = robot\n"
                "Orange arrow = current ToF/Gimbal direction"
            ),
            justify="left",
            wraplength=280,
        ).pack(anchor="w")

        ttk.Separator(right, orient="horizontal").pack(fill="x", pady=12)

        ttk.Label(
            right,
            text=(
                "Map frame\n"
                "FRONT at mission start = ↑\n"
                "RIGHT at mission start = →\n"
                "No field size/start corner is preloaded."
            ),
            justify="left",
            wraplength=280,
        ).pack(anchor="w")

        ttk.Label(
            right,
            textvariable=self.reason_var,
            wraplength=280,
            foreground="#8b0000",
        ).pack(anchor="w", pady=(14, 8))

        self.save_map_button = ttk.Button(
            right,
            text="SAVE GUI MAP NOW",
            command=self._save_gui_map_now,
        )
        self.save_map_button.pack(fill="x", pady=(8, 4))

        self.stop_button = ttk.Button(
            right,
            text="STOP & SAVE",
            command=self._request_stop,
        )
        self.stop_button.pack(fill="x", pady=(4, 6))

        ttk.Label(
            right,
            text="Closing this window requests a safe stop and result export.",
            wraplength=280,
            justify="left",
        ).pack(anchor="w", pady=(8, 0))

        self.root.after(self.refresh_ms, self._poll)

    def publish(self, snapshot: dict) -> None:
        try:
            while self._queue.qsize() > 2:
                self._queue.get_nowait()
        except queue.Empty:
            pass
        self._queue.put(snapshot)

    def _request_stop(self) -> None:
        self.stop_event.set()
        self.status_var.set("Status: Stopping safely and saving results...")
        self.stop_button.state(["disabled"])

    def _on_close(self) -> None:
        if not self.stop_event.is_set():
            self._request_stop()
            self.root.after(600, self._close_if_finished)
        else:
            self.root.destroy()

    def _close_if_finished(self) -> None:
        if self._latest and self._latest.get("finished"):
            self.root.destroy()
        else:
            self.root.after(300, self._close_if_finished)

    def _poll(self) -> None:
        try:
            while True:
                self._latest = self._queue.get_nowait()
        except queue.Empty:
            pass

        if self._latest is not None:
            self._render(self._latest)

        self.root.after(self.refresh_ms, self._poll)

    @staticmethod
    def _display_cell(cell):
        """Logical +X FRONT/+Y LEFT -> screen row/column."""
        x, y = int(cell[0]), int(cell[1])
        return -x, -y

    def _render(self, snapshot: dict) -> None:
        if snapshot.get("known_cells"):
            self._draw_grid_map(snapshot)

        self.status_var.set("Status: " + str(snapshot.get("status", "")))

        robot_xy = snapshot.get("robot_xy")
        if robot_xy is None:
            self.pose_var.set("Pose: --")
        else:
            self.pose_var.set(
                "Pose: x={:+.2f} m  y={:+.2f} m".format(
                    float(robot_xy[0]),
                    float(robot_xy[1]),
                )
            )

        cell = snapshot.get("logical_cell")
        self.cell_var.set(
            "Cell: --"
            if cell is None
            else "Cell: ({}, {})".format(cell[0], cell[1])
        )

        tof_cm = snapshot.get("tof_cm")
        self.tof_var.set(
            "ToF: --"
            if tof_cm is None
            else "ToF: {:.1f} cm".format(float(tof_cm))
        )

        direction_name = snapshot.get("gimbal_direction_name", "--")
        gimbal_yaw = snapshot.get("gimbal_yaw_deg")
        if gimbal_yaw is None:
            self.gimbal_var.set("Gimbal: {}".format(direction_name))
        else:
            self.gimbal_var.set(
                "Gimbal: {} ({:+.1f}°)".format(
                    direction_name,
                    float(gimbal_yaw),
                )
            )

        vision_active = bool(snapshot.get("vision_active"))
        target_active = bool(snapshot.get("target_detection_active"))
        if target_active:
            self.vision_var.set("Camera: TARGET SURVEY ACTIVE")
        elif vision_active:
            self.vision_var.set("Camera: stream active")
        else:
            self.vision_var.set("Camera: unavailable; ToF mapping continues")

        self.target_var.set(
            "Targets: {}".format(int(snapshot.get("target_count", 0)))
        )

        self._render_vision_preview(snapshot.get("vision_frame"), vision_active)

        self.moves_var.set("Moves: {}".format(snapshot.get("moves", 0)))
        self.discovered_var.set(
            "Discovered cells: {}".format(len(snapshot.get("known_cells") or []))
        )
        self.coverage_var.set(
            "Occupancy coverage: {:.2f}%".format(
                float(snapshot.get("coverage", 0.0))
            )
        )

        self.planner_var.set(
            "Planner: {}".format(snapshot.get("planner_mode") or "FRONTIER_BFS")
        )
        self.frontier_var.set(
            "Reachable frontiers: {}".format(
                int(snapshot.get("planner_frontier_count", 0))
            )
        )
        frontier_cell = snapshot.get("planner_frontier_cell")
        frontier_target = snapshot.get("planner_frontier_target")
        if frontier_cell is None:
            self.planner_target_var.set("Planner target: --")
        else:
            self.planner_target_var.set(
                "Planner target: {} -> {}".format(
                    tuple(frontier_cell),
                    "--" if frontier_target is None else tuple(frontier_target),
                )
            )

        ratios = snapshot.get("completion_perimeter_ratios") or {}
        if snapshot.get("completion_enabled"):
            ratio_text = " ".join(
                "{}:{:.0f}%".format(str(name)[0], float(value) * 100.0)
                for name, value in sorted(ratios.items())
            )
            self.completion_var.set(
                "Completion: {}  rectangle={}x{}  {}".format(
                    "READY" if snapshot.get("completion_ready") else "checking",
                    int(snapshot.get("completion_rows", 0)),
                    int(snapshot.get("completion_cols", 0)),
                    ratio_text or "perimeter pending",
                )
            )
        else:
            self.completion_var.set("Completion: frontier exhaustion only")

        reason_text = snapshot.get("reason") or ""

        if snapshot.get("finished"):
            self.stop_button.state(["disabled"])

            run_dir = snapshot.get("run_dir")
            if (
                self.auto_save_map
                and run_dir
                and str(run_dir) not in self._auto_saved_run_dirs
            ):
                try:
                    target = Path(str(run_dir)) / "gui_map.png"
                    self._export_map_png(snapshot, target)
                    self._auto_saved_run_dirs.add(str(run_dir))
                    self._last_saved_path = target
                    reason_text = (
                        reason_text
                        + "\nGUI map saved: {}".format(target)
                    ).strip()
                except Exception as exc:
                    reason_text = (
                        reason_text
                        + "\nGUI map save failed: {}".format(exc)
                    ).strip()

        self.reason_var.set(reason_text)

    def _save_gui_map_now(self) -> None:
        snapshot = self._latest
        if not snapshot or not snapshot.get("known_cells"):
            self.reason_var.set("GUI map is not available yet.")
            return

        try:
            folder = self.output_dir / "gui_snapshots"
            folder.mkdir(parents=True, exist_ok=True)
            stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            target = folder / "gui_map_{}.png".format(stamp)
            self._export_map_png(snapshot, target)
            self._last_saved_path = target
            self.reason_var.set("GUI map saved: {}".format(target))
        except Exception as exc:
            self.reason_var.set("GUI map save failed: {}".format(exc))


    def _export_map_png(self, snapshot: dict, target_path: Path) -> None:
        """Render the same logical information as the Tk canvas into a PNG."""
        target_path = Path(target_path)
        target_path.parent.mkdir(parents=True, exist_ok=True)

        width = max(640, int(self.export_width_px))
        height = max(480, int(self.export_height_px))

        image = self.Image.new(
            "RGB",
            (width, height),
            self.COLOURS["background"],
        )
        draw = self.ImageDraw.Draw(image)
        font = self.ImageFont.load_default()

        known_cells = {
            (int(cell[0]), int(cell[1]))
            for cell in (snapshot.get("known_cells") or [(0, 0)])
        }
        current_cell = snapshot.get("logical_cell")
        if current_cell is not None:
            known_cells.add((int(current_cell[0]), int(current_cell[1])))
        known_cells.add((0, 0))

        display_cells = [self._display_cell(cell) for cell in known_cells]
        min_row = min(r for r, _ in display_cells) - 1
        max_row = max(r for r, _ in display_cells) + 1
        min_col = min(c for _, c in display_cells) - 1
        max_col = max(c for _, c in display_cells) + 1

        rows = max_row - min_row + 1
        cols = max_col - min_col + 1

        header_h = 70
        outer_pad = 55

        size = min(
            (width - 2 * outer_pad) / max(1, cols),
            (height - header_h - 2 * outer_pad) / max(1, rows),
        )
        size = max(24.0, min(130.0, size))

        map_w = cols * size
        map_h = rows * size
        ox = (width - map_w) / 2.0
        oy = header_h + (height - header_h - map_h) / 2.0

        def logical_center(cell):
            row, col = self._display_cell(cell)
            return (
                ox + (col - min_col + 0.5) * size,
                oy + (row - min_row + 0.5) * size,
            )

        def logical_rect(cell):
            row, col = self._display_cell(cell)
            x0 = ox + (col - min_col) * size
            y0 = oy + (row - min_row) * size
            return x0, y0, x0 + size, y0 + size

        def metric_to_image(x_m, y_m):
            cell_size_m = float(snapshot.get("cell_size_m", 0.60))
            display_row = -float(x_m) / cell_size_m
            display_col = -float(y_m) / cell_size_m
            return (
                ox + (display_col - min_col + 0.5) * size,
                oy + (display_row - min_row + 0.5) * size,
            )

        def line_points(cells):
            return [logical_center(cell) for cell in cells]

        title = "Final Round 1 V05 - GUI Logical Map"
        status = str(snapshot.get("status", ""))
        draw.text((24, 16), title, fill=self.COLOURS["text"], font=font)
        draw.text((24, 36), status, fill=self.COLOURS["muted"], font=font)

        for cell in sorted(known_cells):
            x0, y0, x1, y1 = logical_rect(cell)
            draw.rectangle(
                (x0, y0, x1, y1),
                fill=self.COLOURS["cell"],
                outline=self.COLOURS["grid"],
                width=1,
            )
            draw.text(
                (x0 + 5, y0 + 4),
                "{:+d},{:+d}".format(cell[0], cell[1]),
                fill=self.COLOURS["unknown_text"],
                font=font,
            )

        planner_route = [
            (int(cell[0]), int(cell[1]))
            for cell in (snapshot.get("planner_route") or [])
        ]
        if len(planner_route) >= 2:
            draw.line(
                line_points(planner_route),
                fill=self.COLOURS["planner"],
                width=max(2, int(size * 0.05)),
            )

        logical_path = [
            (int(cell[0]), int(cell[1]))
            for cell in (snapshot.get("logical_path") or [])
        ]
        if len(logical_path) >= 2:
            draw.line(
                line_points(logical_path),
                fill=self.COLOURS["route"],
                width=max(2, int(size * 0.055)),
            )

        trajectory = snapshot.get("trajectory") or []
        if len(trajectory) >= 2:
            draw.line(
                [
                    metric_to_image(float(x_m), float(y_m))
                    for x_m, y_m in trajectory
                ],
                fill=self.COLOURS["travel"],
                width=max(3, int(size * 0.075)),
            )

        drawn_lines = set()
        for x, y, direction in (snapshot.get("wall_edges") or []):
            cell = (int(x), int(y))
            if cell not in known_cells:
                continue

            x0, y0, x1, y1 = logical_rect(cell)
            direction = int(direction) % 4

            if direction == 0:
                line = (x0, y0, x1, y0)
            elif direction == 1:
                line = (x1, y0, x1, y1)
            elif direction == 2:
                line = (x0, y1, x1, y1)
            else:
                line = (x0, y0, x0, y1)

            key = tuple(round(v, 3) for v in line)
            reverse = (key[2], key[3], key[0], key[1])
            if key in drawn_lines or reverse in drawn_lines:
                continue
            drawn_lines.add(key)

            draw.line(
                (line[0], line[1], line[2], line[3]),
                fill=self.COLOURS["wall"],
                width=max(4, int(size * 0.075)),
            )

        target_colours = {
            "red": "#dc2626",
            "green": "#16a34a",
            "blue": "#2563eb",
            "yellow": "#ca8a04",
            "orange": "#ea580c",
        }
        for target in snapshot.get("targets") or []:
            xy = target.get("estimated_target_xy_m")
            if not xy or len(xy) < 2:
                continue
            tx, ty = metric_to_image(float(xy[0]), float(xy[1]))
            colour = target_colours.get(
                str(target.get("color", "")).lower(),
                "#7c3aed",
            )
            radius = max(8.0, size * 0.12)
            draw.ellipse(
                (tx - radius, ty - radius, tx + radius, ty + radius),
                fill=colour,
                outline="white",
                width=2,
            )
            draw.text(
                (tx - radius * 0.6, ty - 5),
                str(target.get("target_id", "T")),
                fill="white",
                font=font,
            )

        sx, sy = logical_center((0, 0))
        r = max(9.0, size * 0.16)
        draw.ellipse(
            (sx - r, sy - r, sx + r, sy + r),
            fill=self.COLOURS["start"],
            outline="white",
            width=2,
        )
        draw.text((sx - 4, sy - 6), "S", fill="white", font=font)

        robot_xy = snapshot.get("robot_xy")
        if robot_xy is None:
            if current_cell is None:
                rx, ry = sx, sy
            else:
                rx, ry = logical_center(
                    (int(current_cell[0]), int(current_cell[1]))
                )
        else:
            rx, ry = metric_to_image(
                float(robot_xy[0]),
                float(robot_xy[1]),
            )

        rr = max(10.0, size * 0.18)
        draw.ellipse(
            (rx - rr, ry - rr, rx + rr, ry + rr),
            fill=self.COLOURS["robot"],
            outline="white",
            width=2,
        )
        draw.text((rx - 4, ry - 6), "R", fill="white", font=font)

        direction = int(snapshot.get("gimbal_direction", 0)) % 4
        dx, dy = {
            0: (0.0, -1.0),
            1: (1.0, 0.0),
            2: (0.0, 1.0),
            3: (-1.0, 0.0),
        }[direction]
        arrow_len = max(24.0, size * 0.42)
        ex = rx + dx * arrow_len
        ey = ry + dy * arrow_len
        draw.line(
            (rx, ry, ex, ey),
            fill=self.COLOURS["gimbal"],
            width=max(3, int(size * 0.055)),
        )

        draw.text(
            (max(8, ox), max(header_h, oy - 20)),
            "FRONT ^",
            fill=self.COLOURS["text"],
            font=font,
        )
        draw.text(
            (max(8, ox + map_w - 70), max(header_h, oy - 20)),
            "RIGHT >",
            fill=self.COLOURS["text"],
            font=font,
        )

        ratios = snapshot.get("completion_perimeter_ratios") or {}
        completion = " ".join(
            "{}={:.0f}%".format(name, float(value) * 100.0)
            for name, value in sorted(ratios.items())
        )
        if completion:
            draw.text(
                (24, height - 24),
                "Perimeter: " + completion,
                fill=self.COLOURS["muted"],
                font=font,
            )

        image.save(str(target_path), format="PNG")


    def _render_vision_preview(self, frame, active: bool) -> None:
        if frame is None:
            self._vision_photo = None
            self.vision_preview.configure(
                image="",
                text="Camera unavailable" if not active else "Waiting for frame...",
            )
            return

        try:
            rgb = frame[:, :, ::-1]
            image = self.Image.fromarray(rgb)
            target_w = 310
            target_h = 174

            if hasattr(self.Image, "Resampling"):
                resample = self.Image.Resampling.LANCZOS
            else:
                resample = self.Image.LANCZOS

            image.thumbnail((target_w, target_h), resample)
            photo = self.ImageTk.PhotoImage(image=image)
            self._vision_photo = photo
            self.vision_preview.configure(image=photo, text="")
        except Exception:
            self._vision_photo = None
            self.vision_preview.configure(image="", text="Camera preview error")

    def _draw_grid_map(self, snapshot: dict) -> None:
        canvas = self.canvas
        colours = self.COLOURS
        canvas.delete("all")

        known_cells = {
            (int(cell[0]), int(cell[1]))
            for cell in (snapshot.get("known_cells") or [(0, 0)])
        }
        current_cell = snapshot.get("logical_cell")
        if current_cell is not None:
            known_cells.add((int(current_cell[0]), int(current_cell[1])))
        known_cells.add((0, 0))

        display_cells = [self._display_cell(cell) for cell in known_cells]
        min_row = min(r for r, _ in display_cells)
        max_row = max(r for r, _ in display_cells)
        min_col = min(c for _, c in display_cells)
        max_col = max(c for _, c in display_cells)

        # One-cell visual padding around the discovered map. It is only display
        # space; it does not imply known field extent.
        pad_cells = 1
        min_row -= pad_cells
        max_row += pad_cells
        min_col -= pad_cells
        max_col += pad_cells

        rows = max_row - min_row + 1
        cols = max_col - min_col + 1

        canvas_w = max(100, int(canvas.winfo_width()))
        canvas_h = max(100, int(canvas.winfo_height()))
        outer_pad = 42

        size = min(
            (canvas_w - 2 * outer_pad) / max(1, cols),
            (canvas_h - 2 * outer_pad) / max(1, rows),
        )
        size = max(28.0, min(110.0, size))

        map_w = cols * size
        map_h = rows * size
        ox = (canvas_w - map_w) / 2.0
        oy = (canvas_h - map_h) / 2.0

        def logical_center(cell):
            row, col = self._display_cell(cell)
            return (
                ox + (col - min_col + 0.5) * size,
                oy + (row - min_row + 0.5) * size,
            )

        def logical_rect(cell):
            row, col = self._display_cell(cell)
            x0 = ox + (col - min_col) * size
            y0 = oy + (row - min_row) * size
            return x0, y0, x0 + size, y0 + size

        def metric_to_canvas(x_m, y_m):
            # 1 logical cell = cell_size_m. +X is FRONT/up, +Y is LEFT.
            cell_size_m = float(snapshot.get("cell_size_m", 0.60))
            display_row = -float(x_m) / cell_size_m
            display_col = -float(y_m) / cell_size_m
            px = ox + (display_col - min_col + 0.5) * size
            py = oy + (display_row - min_row + 0.5) * size
            return px, py

        # Draw discovered cells only. Unknown field outside this set stays gray.
        for cell in sorted(known_cells):
            x0, y0, x1, y1 = logical_rect(cell)
            canvas.create_rectangle(
                x0,
                y0,
                x1,
                y1,
                fill=colours["cell"],
                outline=colours["grid"],
                width=1,
            )
            canvas.create_text(
                x0 + 5,
                y0 + 4,
                text="{:+d},{:+d}".format(cell[0], cell[1]),
                anchor="nw",
                fill=colours["unknown_text"],
                font=("Segoe UI", max(7, int(size * 0.10))),
            )

        # Planned nearest-frontier route. This shows why the robot is
        # relocating through already-visited cells instead of making the route
        # look like unexplained backtracking.
        planner_route = [
            (int(cell[0]), int(cell[1]))
            for cell in (snapshot.get("planner_route") or [])
        ]
        if len(planner_route) >= 2:
            coords = []
            for cell in planner_route:
                coords.extend(logical_center(cell))
            canvas.create_line(
                *coords,
                fill=colours["planner"],
                width=max(2, int(size * 0.050)),
                dash=(4, 4),
                joinstyle="round",
                capstyle="round",
            )

        # Logical cell-centre path: actual exploration/relocation history.
        logical_path = [
            (int(cell[0]), int(cell[1]))
            for cell in (snapshot.get("logical_path") or [])
        ]
        if len(logical_path) >= 2:
            coords = []
            for cell in logical_path:
                coords.extend(logical_center(cell))
            canvas.create_line(
                *coords,
                fill=colours["route"],
                width=max(2, int(size * 0.055)),
                dash=(7, 5),
                joinstyle="round",
                capstyle="round",
            )

        # Actual odometry path grows continuously while the robot is moving.
        trajectory = snapshot.get("trajectory") or []
        if len(trajectory) >= 2:
            coords = []
            for x_m, y_m in trajectory:
                coords.extend(metric_to_canvas(float(x_m), float(y_m)))
            canvas.create_line(
                *coords,
                fill=colours["travel"],
                width=max(3, int(size * 0.075)),
                joinstyle="round",
                capstyle="round",
            )

        # Sensor-confirmed walls overlay the thin cell grid.
        wall_edges = snapshot.get("wall_edges") or []
        drawn_lines = set()
        for x, y, direction in wall_edges:
            cell = (int(x), int(y))
            if cell not in known_cells:
                # A mirrored wall entry can belong to an undiscovered neighbour.
                # The same physical edge will be drawn from the discovered side.
                continue

            x0, y0, x1, y1 = logical_rect(cell)
            direction = int(direction) % 4
            if direction == 0:      # FRONT = screen top
                line = (x0, y0, x1, y0)
            elif direction == 1:    # RIGHT
                line = (x1, y0, x1, y1)
            elif direction == 2:    # BACK
                line = (x0, y1, x1, y1)
            else:                   # LEFT
                line = (x0, y0, x0, y1)

            key = tuple(round(v, 3) for v in line)
            reverse_key = (key[2], key[3], key[0], key[1])
            if key in drawn_lines or reverse_key in drawn_lines:
                continue
            drawn_lines.add(key)

            canvas.create_line(
                *line,
                fill=colours["wall"],
                width=max(4, int(size * 0.075)),
                capstyle="round",
            )

        # Camera-confirmed targets.  Use the estimated metric wall position
        # derived from the observing cell + gimbal direction + ToF distance.
        target_colours = {
            "red": "#dc2626",
            "green": "#16a34a",
            "blue": "#2563eb",
            "yellow": "#ca8a04",
            "orange": "#ea580c",
        }
        for target in snapshot.get("targets") or []:
            xy = target.get("estimated_target_xy_m")
            if not xy or len(xy) < 2:
                continue
            tx, ty = metric_to_canvas(float(xy[0]), float(xy[1]))
            colour = target_colours.get(
                str(target.get("color", "")).lower(),
                "#7c3aed",
            )
            radius = max(8.0, size * 0.12)
            canvas.create_oval(
                tx - radius,
                ty - radius,
                tx + radius,
                ty + radius,
                fill=colour,
                outline="white",
                width=2,
            )
            canvas.create_text(
                tx,
                ty,
                text=str(target.get("target_id", "T")),
                fill="white",
                font=("Segoe UI", max(7, int(size * 0.09)), "bold"),
            )

        # Start marker.
        sx, sy = logical_center((0, 0))
        start_radius = max(9, size * 0.16)
        canvas.create_oval(
            sx - start_radius,
            sy - start_radius,
            sx + start_radius,
            sy + start_radius,
            fill=colours["start"],
            outline="white",
            width=2,
        )
        canvas.create_text(
            sx,
            sy,
            text="S",
            fill="white",
            font=("Segoe UI", max(9, int(size * 0.18)), "bold"),
        )

        # Robot marker follows continuous odometry, not only logical-cell jumps.
        robot_xy = snapshot.get("robot_xy")
        if robot_xy is None:
            if current_cell is None:
                robot_px, robot_py = sx, sy
            else:
                robot_px, robot_py = logical_center(
                    (int(current_cell[0]), int(current_cell[1]))
                )
        else:
            robot_px, robot_py = metric_to_canvas(
                float(robot_xy[0]),
                float(robot_xy[1]),
            )

        robot_radius = max(10, size * 0.18)
        canvas.create_oval(
            robot_px - robot_radius,
            robot_py - robot_radius,
            robot_px + robot_radius,
            robot_py + robot_radius,
            fill=colours["robot"],
            outline="white",
            width=2,
        )
        canvas.create_text(
            robot_px,
            robot_py,
            text="R",
            fill="white",
            font=("Segoe UI", max(9, int(size * 0.17)), "bold"),
        )

        # ToF/gimbal direction arrow.
        direction = int(snapshot.get("gimbal_direction", 0)) % 4
        dx, dy = {
            0: (0.0, -1.0),   # FRONT/up
            1: (1.0, 0.0),    # RIGHT
            2: (0.0, 1.0),    # BACK/down
            3: (-1.0, 0.0),   # LEFT
        }[direction]
        arrow_len = max(24.0, size * 0.42)
        canvas.create_line(
            robot_px,
            robot_py,
            robot_px + dx * arrow_len,
            robot_py + dy * arrow_len,
            fill=colours["gimbal"],
            width=max(3, int(size * 0.055)),
            arrow="last",
        )

        # Local orientation labels. These are relative, not global compass data.
        canvas.create_text(
            ox,
            max(14, oy - 24),
            text="FRONT ↑",
            anchor="w",
            fill=colours["text"],
            font=("Segoe UI", 11, "bold"),
        )
        canvas.create_text(
            ox + map_w,
            max(14, oy - 24),
            text="RIGHT →",
            anchor="e",
            fill=colours["text"],
            font=("Segoe UI", 11, "bold"),
        )

        # Small note prevents the auto-fit display from being mistaken for a
        # preloaded field boundary.
        canvas.create_text(
            ox,
            min(canvas_h - 8, oy + map_h + 18),
            text="Auto-fit discovered area — outer gray space is still unknown.",
            anchor="w",
            fill=colours["muted"],
            font=("Segoe UI", 9),
        )

    def run(self) -> None:
        self.root.mainloop()


def run_with_gui(
    run_function: Callable,
    config,
    ep_robot,
) -> None:
    stop_event = threading.Event()
    gui = RealtimeMapGUI(
        stop_event,
        canvas_px=config.gui_canvas_px,
        refresh_ms=config.gui_refresh_ms,
        output_dir=config.output_dir,
        auto_save_map=config.gui_auto_save_map,
        export_width_px=config.gui_export_width_px,
        export_height_px=config.gui_export_height_px,
    )

    def worker():
        try:
            run_function(
                config=config,
                publish=gui.publish,
                stop_event=stop_event,
                ep_robot=ep_robot,
            )
        except Exception as exc:
            gui.publish({
                "status": "ERROR",
                "reason": "{}\n{}".format(
                    exc,
                    traceback.format_exc(limit=4),
                ),
                "finished": True,
            })

    thread = threading.Thread(
        target=worker,
        name="classwork8-explorer",
        daemon=True,
    )
    thread.start()
    gui.run()
