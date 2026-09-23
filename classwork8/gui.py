"""Realtime Tkinter GUI for Classwork 8 exploration."""

from __future__ import annotations

import math
import queue
import threading
import traceback
from typing import Callable, Optional


class RealtimeMapGUI:
    def __init__(self, stop_event: threading.Event, canvas_px: int = 720, refresh_ms: int = 150):
        import tkinter as tk
        from tkinter import ttk

        self.tk = tk
        self.ttk = ttk
        self.stop_event = stop_event
        self.refresh_ms = int(refresh_ms)
        self._queue = queue.Queue()
        self._latest = None
        self._photo = None
        self._scaled_photo = None

        self.root = tk.Tk()
        self.root.title("Classwork 8 - ToF Realtime Exploration")
        self.root.geometry("1040x820")
        self.root.minsize(900, 700)
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

        outer = ttk.Frame(self.root, padding=10)
        outer.pack(fill="both", expand=True)

        left = ttk.Frame(outer)
        left.pack(side="left", fill="both", expand=True)

        right = ttk.Frame(outer, width=280)
        right.pack(side="right", fill="y", padx=(12, 0))

        self.canvas_px = int(canvas_px)
        self.canvas = tk.Canvas(
            left,
            width=self.canvas_px,
            height=self.canvas_px,
            bg="#b8b8b8",
            highlightthickness=1,
            highlightbackground="#666666",
        )
        self.canvas.pack(fill="both", expand=True)

        ttk.Label(right, text="Classwork 8", font=("Segoe UI", 16, "bold")).pack(anchor="w")
        ttk.Label(right, text="ToF-only realtime mapping", font=("Segoe UI", 10)).pack(anchor="w", pady=(0, 14))

        self.status_var = tk.StringVar(value="Starting...")
        self.pose_var = tk.StringVar(value="Pose: --")
        self.cell_var = tk.StringVar(value="Cell: --")
        self.tof_var = tk.StringVar(value="ToF: --")
        self.gimbal_var = tk.StringVar(value="Gimbal: --")
        self.moves_var = tk.StringVar(value="Moves: 0")
        self.coverage_var = tk.StringVar(value="Coverage: 0.00%")
        self.reason_var = tk.StringVar(value="")

        for variable in (
            self.status_var,
            self.pose_var,
            self.cell_var,
            self.tof_var,
            self.gimbal_var,
            self.moves_var,
            self.coverage_var,
        ):
            ttk.Label(right, textvariable=variable, wraplength=260).pack(anchor="w", pady=3)

        ttk.Separator(right, orient="horizontal").pack(fill="x", pady=12)
        ttk.Label(
            right,
            text="Map legend\nGray = unknown\nWhite = free\nBlack = wall\nBlue = trajectory\nRed = robot",
            justify="left",
        ).pack(anchor="w")

        ttk.Label(
            right,
            textvariable=self.reason_var,
            wraplength=260,
            foreground="#8b0000",
        ).pack(anchor="w", pady=(14, 8))

        self.stop_button = ttk.Button(right, text="STOP & SAVE", command=self._request_stop)
        self.stop_button.pack(fill="x", pady=(12, 6))

        ttk.Label(
            right,
            text="Closing the window also requests a safe stop and export.",
            wraplength=260,
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
        self.status_var.set("Stopping safely and saving results...")
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

    def _render(self, snapshot: dict) -> None:
        matrix = snapshot.get("matrix")
        if matrix:
            self._draw_map(snapshot)

        status = snapshot.get("status", "")
        self.status_var.set("Status: " + status)

        robot_xy = snapshot.get("robot_xy")
        if robot_xy is None:
            self.pose_var.set("Pose: --")
        else:
            self.pose_var.set("Pose: x={:+.2f} m  y={:+.2f} m".format(robot_xy[0], robot_xy[1]))

        cell = snapshot.get("logical_cell")
        self.cell_var.set("Cell: --" if cell is None else "Cell: ({}, {})".format(cell[0], cell[1]))

        tof_cm = snapshot.get("tof_cm")
        self.tof_var.set("ToF: --" if tof_cm is None else "ToF: {:.1f} cm".format(tof_cm))

        direction_name = snapshot.get("gimbal_direction_name", "--")
        gimbal_yaw = snapshot.get("gimbal_yaw_deg")
        if gimbal_yaw is None:
            self.gimbal_var.set("Gimbal: {}".format(direction_name))
        else:
            self.gimbal_var.set("Gimbal: {} ({:+.1f}°)".format(direction_name, gimbal_yaw))

        self.moves_var.set("Moves: {}".format(snapshot.get("moves", 0)))
        self.coverage_var.set("Coverage: {:.2f}%".format(float(snapshot.get("coverage", 0.0))))

        reason = snapshot.get("reason") or ""
        self.reason_var.set(reason)

        if snapshot.get("finished"):
            self.stop_button.state(["disabled"])

    def _draw_map(self, snapshot: dict) -> None:
        matrix = snapshot["matrix"]
        rows = len(matrix)
        cols = len(matrix[0]) if rows else 0
        if rows == 0 or cols == 0:
            return

        canvas_w = max(1, int(self.canvas.winfo_width()))
        canvas_h = max(1, int(self.canvas.winfo_height()))
        scale = max(1, min(canvas_w // cols, canvas_h // rows))
        image_w = cols * scale
        image_h = rows * scale
        left = (canvas_w - image_w) / 2.0
        top = (canvas_h - image_h) / 2.0

        base = self.tk.PhotoImage(width=cols, height=rows)
        color = {-1: "#b8b8b8", 0: "#ffffff", 100: "#111111"}
        # Matrix row 0 is the minimum world-y row, so reverse for screen top-down.
        for display_row, source_row in enumerate(reversed(matrix)):
            row_colors = " ".join(color.get(int(v), "#b8b8b8") for v in source_row)
            base.put("{" + row_colors + "}", to=(0, display_row))

        scaled = base.zoom(scale, scale)
        self._photo = base
        self._scaled_photo = scaled

        self.canvas.delete("all")
        self.canvas.create_image(left, top, anchor="nw", image=scaled)

        resolution = float(snapshot.get("resolution_m", 0.05))
        origin_x = float(snapshot.get("origin_x_m", -cols * resolution / 2.0))
        origin_y = float(snapshot.get("origin_y_m", -rows * resolution / 2.0))
        width_m = cols * resolution
        height_m = rows * resolution
        cell_size_m = float(snapshot.get("cell_size_m", 0.60))

        def world_to_canvas(x_m: float, y_m: float):
            col = (x_m - origin_x) / resolution
            row = (y_m - origin_y) / resolution
            px = left + col * scale
            py = top + (rows - row) * scale
            return px, py

        # Physical 60 cm cell boundaries, centered around the start cell.
        k_min = int(math.floor((-width_m / 2.0 - cell_size_m / 2.0) / cell_size_m)) - 1
        k_max = int(math.ceil((width_m / 2.0 - cell_size_m / 2.0) / cell_size_m)) + 1
        for k in range(k_min, k_max + 1):
            x = (k + 0.5) * cell_size_m
            if origin_x <= x <= origin_x + width_m:
                px, _ = world_to_canvas(x, 0.0)
                self.canvas.create_line(px, top, px, top + image_h, fill="#d0d0d0", dash=(3, 5))
            y = (k + 0.5) * cell_size_m
            if origin_y <= y <= origin_y + height_m:
                _, py = world_to_canvas(0.0, y)
                self.canvas.create_line(left, py, left + image_w, py, fill="#d0d0d0", dash=(3, 5))

        trajectory = snapshot.get("trajectory") or []
        if len(trajectory) >= 2:
            coords = []
            for x_m, y_m in trajectory:
                px, py = world_to_canvas(float(x_m), float(y_m))
                coords.extend((px, py))
            self.canvas.create_line(*coords, fill="#2563eb", width=3, smooth=False)

        robot_xy = snapshot.get("robot_xy")
        if robot_xy is not None:
            px, py = world_to_canvas(float(robot_xy[0]), float(robot_xy[1]))
            radius = max(5, int(0.12 / resolution * scale))
            self.canvas.create_oval(
                px - radius,
                py - radius,
                px + radius,
                py + radius,
                fill="#dc2626",
                outline="#7f1d1d",
                width=2,
            )

            direction = int(snapshot.get("gimbal_direction", 0)) % 4
            dx, dy = {
                0: (1.0, 0.0),
                1: (0.0, -1.0),
                2: (-1.0, 0.0),
                3: (0.0, 1.0),
            }[direction]
            length = max(28, radius * 2)
            self.canvas.create_line(
                px,
                py,
                px + dx * length,
                py - dy * length,
                fill="#f59e0b",
                width=4,
                arrow="last",
            )

        # Mark map origin/start.
        sx, sy = world_to_canvas(0.0, 0.0)
        self.canvas.create_oval(sx - 5, sy - 5, sx + 5, sy + 5, fill="#16a34a", outline="")

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
                "reason": "{}\n{}".format(exc, traceback.format_exc(limit=4)),
                "finished": True,
            })

    thread = threading.Thread(target=worker, name="classwork8-explorer", daemon=True)
    thread.start()
    gui.run()
