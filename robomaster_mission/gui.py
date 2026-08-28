"""Tkinter map editor and mission monitor."""
from __future__ import annotations

import json
import queue
import threading
import traceback
from pathlib import Path
from typing import Dict, Optional, Tuple

from .configuration import DIR_DELTA, DIR_FROM_NAME, HEADINGS, HybridConfig
from .grid_map import GridMazeMap
from .mission import run_hybrid_robot, simulate_mission
from .planning import mission_route_preview

class HybridMazeGUI:
    COLOURS = {
        "background": "#f8fafc",
        "panel": "#ffffff",
        "primary": "#2563eb",
        "primary_dark": "#1e3a8a",
        "success_soft": "#dcfce7",
        "warning_soft": "#fef3c7",
        "info_soft": "#dbeafe",
        "danger_soft": "#fee2e2",
        "muted": "#64748b",
        "grid": "#cbd5e1",
        "wall": "#111827",
        "sensor_wall": "#dc2626",
        "route": "#60a5fa",
        "travel": "#2563eb",
        "start": "#16a34a",
        "drop": "#f59e0b",
        "drop_wall": "#f97316",
        "exit": "#7c3aed",
        "robot": "#0f172a",
    }

    FIELD_SECTIONS = {
        "1  Mission": [
            (
                "Map dimensions",
                "ค่าหลักที่ต้องตรงกับสนามจริงก่อนวาดกำแพง",
                [
                    ("rows", "Rows / จำนวนแถว", int),
                    ("cols", "Columns / จำนวนคอลัมน์", int),
                    ("cell_size_cm", "Cell size / ขนาดช่อง (cm)", float),
                ],
            ),
            (
                "Robot orientation",
                "N อยู่ด้านบนของแผนที่ และ E อยู่ด้านขวา",
                [
                    ("connection", "Connection", str),
                    ("start_heading", "Start heading / ทิศเริ่มต้น", str),
                    ("exit_heading", "Exit heading / ทิศออก", str),
                    ("exit_drive_cm", "Drive past exit / ระยะพ้นทางออก (cm)", float),
                ],
            ),
        ],
        "2  Pickup": [
            (
                "Pickup mission",
                "ค่าที่ใช้ตรวจวัตถุและยืนยันว่าหยิบติดก่อนเข้าเขาวงกต",
                [
                    ("pickup_enabled", "Run pickup before maze", bool),
                    ("pickup_target_cm", "Pickup ToF target (cm)", float),
                    ("pickup_tolerance_cm", "Pickup tolerance (cm)", float),
                    ("object_detect_max_cm", "Object detection max (cm)", float),
                    ("pickup_attempts", "Maximum pickup attempts", int),
                ],
            ),
            (
                "Approach and verification",
                "ปกติไม่ต้องปรับ ถ้ารถเข้าหาวัตถุเร็วหรือช้าเกินไปจึงค่อยแก้",
                [
                    ("pickup_fast_speed_mps", "Fast approach (m/s)", float),
                    ("pickup_slow_speed_mps", "Slow approach (m/s)", float),
                    ("pickup_crawl_speed_mps", "Crawl approach (m/s)", float),
                    ("pickup_timeout_sec", "Approach timeout (s)", float),
                    ("tof_lost_abort_sec", "ToF lost timeout (s)", float),
                    ("lift_clear_delta_cm", "Lift verification delta (cm)", float),
                    ("verify_window_sec", "Verification window (s)", float),
                    ("retry_backoff_cm", "Retry reverse distance (cm)", float),
                ],
            ),
            (
                "Arm and gripper calibration",
                "แก้เฉพาะเมื่อท่าแขนหรือกริปเปอร์ของรถจริงไม่ตรง",
                [
                    ("gripper_power", "Gripper power (%)", int),
                    ("gripper_open_sec", "Gripper open time (s)", float),
                    ("gripper_close_sec", "Gripper close time (s)", float),
                    ("arm_action_timeout_sec", "Arm timeout (s)", float),
                    ("arm_settle_sec", "Arm settle time (s)", float),
                    ("carry_x", "Carry arm X", int),
                    ("carry_y", "Carry arm Y", int),
                    ("pickup_x", "Pickup arm X", int),
                    ("pickup_y", "Pickup arm Y", int),
                    ("drop_x", "Drop arm X", int),
                    ("drop_y", "Drop arm Y", int),
                ],
            ),
        ],
        "3  Drop 40 cm": [
            (
                "Wall orientation",
                "สนามนี้ใช้ W + RIGHT: รถหันตะวันตก กำแพงอีกด้านอยู่ขวา",
                [
                    ("drop_heading", "Drop heading / ทิศรถขณะวาง", str),
                    ("drop_side", "Side wall sensor / กำแพงด้านข้าง", str),
                ],
            ),
            (
                "Object requirement",
                "กรอกระยะจากศูนย์กลางวัตถุถึงกำแพง ไม่ใช่ค่าที่เซนเซอร์อ่าน",
                [
                    ("drop_object_front_wall_cm", "Object → front wall (cm)", float),
                    ("drop_object_side_wall_cm", "Object → side wall (cm)", float),
                ],
            ),
            (
                "Sensor mounting offsets",
                "วัดจากรถจริง: วัตถุยื่นหน้า ToF เท่าไร และ Sharp ห่างแนวกลางวัตถุเท่าไร",
                [
                    ("drop_tof_to_object_forward_cm", "Object ahead of ToF (cm)", float),
                    ("drop_sharp_to_object_lateral_cm", "Object centre from Sharp (cm)", float),
                ],
            ),
            (
                "Acceptance and safety",
                "Tolerance 3–5 cm เหมาะกับงานจริง; 10 cm ใช้ทดสอบแบบหลวม",
                [
                    ("drop_distance_tolerance_cm", "Allowed error ± (cm)", float),
                    ("drop_stable_samples", "Stable sensor samples", int),
                    ("drop_align_timeout_sec", "Alignment timeout (s)", float),
                    ("drop_align_max_travel_cm", "Maximum alignment travel (cm)", float),
                    ("drop_align_max_speed_mps", "Maximum alignment speed (m/s)", float),
                ],
            ),
        ],
        "4  Motion & Safety": [
            (
                "Normal driving",
                "ความเร็วต่ำช่วยลด odometry slip และการชนมุมกำแพง",
                [
                    ("forward_speed_mps", "Forward speed (m/s)", float),
                    ("minimum_speed_mps", "Minimum speed (m/s)", float),
                    ("lateral_max_mps", "Maximum lateral correction (m/s)", float),
                    ("cell_tolerance_cm", "Cell distance tolerance (cm)", float),
                ],
            ),
            (
                "Sensor safety",
                "Hard stop ต้องต่ำกว่า Front wall threshold เสมอ",
                [
                    ("hard_stop_front_cm", "Emergency front stop (cm)", float),
                    ("front_wall_cm", "Front topology threshold (cm)", float),
                    ("side_wall_cm", "Side wall-control threshold (cm)", float),
                    ("side_topology_open_cm", "Sharp opening threshold (cm)", float),
                ],
            ),
            (
                "Sharp calibration",
                "ว่างไว้เพื่อใช้ตารางที่วัดแล้ว 450→10, 360→20, 300→30, 240→40, 200→50 cm หรือเลือก JSON เพื่อ override",
                [
                    (
                        "sharp_left_calibration_file",
                        "LEFT Sharp calibration JSON (optional)",
                        str,
                    ),
                    (
                        "sharp_right_calibration_file",
                        "RIGHT Sharp calibration JSON (optional)",
                        str,
                    ),
                ],
            ),
        ],
        "5  Advanced": [
            (
                "Planner",
                "ค่าเหล่านี้มีค่าเริ่มต้นที่ปลอดภัย ไม่จำเป็นต้องปรับในการทดสอบปกติ",
                [
                    ("turn_cost", "A* turn cost", float),
                    ("max_replans", "Maximum replans", int),
                    ("explore_max_steps", "Fallback maximum steps", int),
                    ("fallback_edge_limit", "Trémaux edge traversal limit", int),
                    ("sensor_overrides_drawing", "Use live sensor walls in A*", bool),
                ],
            ),
            (
                "Control loop",
                "ปรับเฉพาะเมื่อกำลังวิเคราะห์การเคลื่อนที่ระดับล่าง",
                [
                    ("end_wall_arrival_min_ratio", "End-wall arrival ratio", float),
                    ("drive_timeout_sec", "Drive command timeout (s)", float),
                    ("control_period_sec", "Control loop period (s)", float),
                    ("drop_align_kp_mps_per_cm", "DROP alignment Kp", float),
                ],
            ),
            (
                "Output",
                "ชื่อไฟล์ JSON/SVG ที่บันทึกผลหลังจบภารกิจ",
                [("output_prefix", "Result filename prefix", str)],
            ),
        ],
    }

    def __init__(self, initial_config: Optional[HybridConfig] = None):
        try:
            import tkinter as tk
            from tkinter import filedialog, messagebox, ttk
        except ModuleNotFoundError as exc:
            raise RuntimeError("Tkinter is required for the map editor") from exc

        self.tk, self.ttk = tk, ttk
        self.filedialog, self.messagebox = filedialog, messagebox
        # Tk variables need an existing Tcl/Tk interpreter.  Python 3.8 raises
        # "Too early to create variable" when StringVar is constructed first.
        self.root = tk.Tk()
        self.config = initial_config or HybridConfig()
        self.config.update_drop_sensor_targets()
        self.maze = GridMazeMap(self.config.rows, self.config.cols)
        self.events: queue.Queue = queue.Queue()
        self.stop_event = threading.Event()
        self.worker: Optional[threading.Thread] = None
        self.tool = tk.StringVar(master=self.root, value="wall")
        self.status_var = tk.StringVar(
            master=self.root,
            value="Draw walls and place START, DROP, EXIT",
        )
        self.drop_target_summary_var = tk.StringVar(master=self.root, value="")
        self.drop_orientation_summary_var = tk.StringVar(master=self.root, value="")
        self.vars: Dict[str, object] = {}
        self.robot_visual: Optional[Tuple[float, float]] = None

        self.root.title("RoboMaster Basic Fixed-Grid Pickup & Drop")
        self.root.geometry("1480x900")
        self.root.minsize(1120, 720)
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        self._build_ui()
        self._load_vars_from_config()
        self.redraw()
        self.root.after(60, self.poll_events)

    def _build_ui(self) -> None:
        tk, ttk = self.tk, self.ttk
        colours = self.COLOURS
        self.root.configure(background=colours["background"])
        style = ttk.Style(self.root)
        if "clam" in style.theme_names():
            style.theme_use("clam")
        style.configure("TFrame", background=colours["background"])
        style.configure("Panel.TFrame", background=colours["panel"])
        style.configure("TLabel", background=colours["background"], foreground="#0f172a")
        style.configure("Panel.TLabel", background=colours["panel"], foreground="#0f172a")
        style.configure("Muted.TLabel", background=colours["panel"], foreground=colours["muted"])
        style.configure("Section.TLabelframe", background=colours["panel"], borderwidth=1)
        style.configure(
            "Section.TLabelframe.Label",
            background=colours["panel"],
            foreground=colours["primary_dark"],
            font=("Segoe UI", 10, "bold"),
        )
        style.configure("TNotebook", background=colours["background"], borderwidth=0)
        style.configure("TNotebook.Tab", padding=(10, 7), font=("Segoe UI", 9))
        style.map(
            "TNotebook.Tab",
            background=[("selected", colours["panel"])],
            foreground=[("selected", colours["primary_dark"])],
        )
        style.configure(
            "Primary.TButton",
            background=colours["primary"],
            foreground="white",
            font=("Segoe UI", 10, "bold"),
            padding=8,
        )
        style.map(
            "Primary.TButton",
            background=[("active", "#1d4ed8"), ("disabled", "#94a3b8")],
        )
        style.configure(
            "Danger.TButton",
            background="#dc2626",
            foreground="white",
            font=("Segoe UI", 10, "bold"),
            padding=8,
        )
        style.map(
            "Danger.TButton",
            background=[("active", "#b91c1c"), ("disabled", "#cbd5e1")],
        )
        style.configure("Action.TButton", padding=6)

        outer = tk.Frame(self.root, bg=colours["background"], padx=10, pady=10)
        outer.pack(fill="both", expand=True)
        outer.columnconfigure(1, weight=1)
        outer.rowconfigure(0, weight=1)

        left = tk.Frame(outer, width=480, bg=colours["background"])
        left.grid(row=0, column=0, sticky="nsw", padx=(0, 10))
        left.grid_propagate(False)
        left.rowconfigure(1, weight=1)
        left.columnconfigure(0, weight=1)

        title_card = tk.Frame(left, bg=colours["primary_dark"], padx=14, pady=11)
        title_card.grid(row=0, column=0, sticky="ew", pady=(0, 8))
        tk.Label(
            title_card,
            text="RoboMaster Fixed-Grid Mission",
            bg=colours["primary_dark"],
            fg="white",
            font=("Segoe UI", 15, "bold"),
        ).pack(anchor="w")
        tk.Label(
            title_card,
            text="Setup → Pickup → DROP 40 cm → Exit",
            bg=colours["primary_dark"],
            fg="#bfdbfe",
            font=("Segoe UI", 9),
        ).pack(anchor="w", pady=(2, 0))

        notebook = ttk.Notebook(left)
        notebook.grid(row=1, column=0, sticky="nsew")

        def make_scroll_page(tab_title: str):
            page = ttk.Frame(notebook, style="Panel.TFrame")
            notebook.add(page, text=tab_title)
            page.rowconfigure(0, weight=1)
            page.columnconfigure(0, weight=1)
            page_canvas = tk.Canvas(
                page,
                bg=colours["panel"],
                highlightthickness=0,
                borderwidth=0,
            )
            scrollbar = ttk.Scrollbar(page, orient="vertical", command=page_canvas.yview)
            page_canvas.configure(yscrollcommand=scrollbar.set)
            page_canvas.grid(row=0, column=0, sticky="nsew")
            scrollbar.grid(row=0, column=1, sticky="ns")
            body = ttk.Frame(page_canvas, style="Panel.TFrame", padding=(10, 10, 8, 14))
            window_id = page_canvas.create_window((0, 0), window=body, anchor="nw")
            body.columnconfigure(0, weight=1)
            body.bind(
                "<Configure>",
                lambda _event, c=page_canvas: c.configure(scrollregion=c.bbox("all")),
            )
            page_canvas.bind(
                "<Configure>",
                lambda event, c=page_canvas, item=window_id: c.itemconfigure(
                    item, width=event.width
                ),
            )
            page_canvas.bind(
                "<MouseWheel>",
                lambda event, c=page_canvas: c.yview_scroll(
                    int(-event.delta / 120), "units"
                ),
            )
            return body

        for tab_title, sections in self.FIELD_SECTIONS.items():
            body = make_scroll_page(tab_title)
            body_row = 0
            for section_title, section_note, fields in sections:
                card = ttk.Labelframe(
                    body,
                    text=section_title,
                    style="Section.TLabelframe",
                    padding=10,
                )
                card.grid(row=body_row, column=0, sticky="ew", pady=(0, 10))
                card.columnconfigure(1, weight=1)
                ttk.Label(
                    card,
                    text=section_note,
                    style="Muted.TLabel",
                    wraplength=405,
                    justify="left",
                ).grid(row=0, column=0, columnspan=2, sticky="ew", pady=(0, 7))
                for field_row, (name, label, kind) in enumerate(fields, start=1):
                    default = getattr(self.config, name)
                    if kind is bool:
                        var = tk.BooleanVar(master=self.root, value=bool(default))
                        widget = ttk.Checkbutton(card, text=label, variable=var)
                        widget.grid(
                            row=field_row,
                            column=0,
                            columnspan=2,
                            sticky="w",
                            pady=3,
                        )
                    else:
                        ttk.Label(card, text=label, style="Panel.TLabel").grid(
                            row=field_row, column=0, sticky="w", pady=3
                        )
                        var = tk.StringVar(master=self.root, value=str(default))
                        if name in ("start_heading", "exit_heading", "drop_heading"):
                            widget = ttk.Combobox(
                                card,
                                textvariable=var,
                                values=HEADINGS,
                                state="readonly",
                                width=13,
                            )
                        elif name == "drop_side":
                            widget = ttk.Combobox(
                                card,
                                textvariable=var,
                                values=("LEFT", "RIGHT"),
                                state="readonly",
                                width=13,
                            )
                        elif name == "connection":
                            widget = ttk.Combobox(
                                card,
                                textvariable=var,
                                values=("ap", "sta", "rndis"),
                                state="readonly",
                                width=13,
                            )
                        elif name in (
                            "sharp_left_calibration_file",
                            "sharp_right_calibration_file",
                        ):
                            holder = ttk.Frame(card, style="Panel.TFrame")
                            holder.columnconfigure(0, weight=1)
                            ttk.Entry(holder, textvariable=var).grid(
                                row=0, column=0, sticky="ew"
                            )
                            ttk.Button(
                                holder,
                                text="Browse…",
                                command=lambda selected_var=var: self._choose_calibration_file(
                                    selected_var
                                ),
                            ).grid(row=0, column=1, padx=(5, 0))
                            widget = holder
                        else:
                            widget = ttk.Entry(card, textvariable=var, width=17)
                        widget.grid(
                            row=field_row,
                            column=1,
                            sticky="ew",
                            pady=3,
                            padx=(10, 0),
                        )
                    self.vars[name] = (var, kind)
                body_row += 1

            if tab_title.startswith("3"):
                summary = tk.Frame(
                    body,
                    bg=colours["info_soft"],
                    highlightbackground="#93c5fd",
                    highlightthickness=1,
                    padx=12,
                    pady=10,
                )
                summary.grid(row=body_row, column=0, sticky="ew", pady=(0, 8))
                tk.Label(
                    summary,
                    text="Calculated sensor targets",
                    bg=colours["info_soft"],
                    fg=colours["primary_dark"],
                    font=("Segoe UI", 10, "bold"),
                ).pack(anchor="w")
                tk.Label(
                    summary,
                    textvariable=self.drop_target_summary_var,
                    bg=colours["info_soft"],
                    fg="#0f172a",
                    font=("Consolas", 11, "bold"),
                    justify="left",
                ).pack(anchor="w", pady=(5, 2))
                tk.Label(
                    summary,
                    textvariable=self.drop_orientation_summary_var,
                    bg=colours["info_soft"],
                    fg="#334155",
                    wraplength=405,
                    justify="left",
                ).pack(anchor="w")

        for field_name in (
            "drop_object_front_wall_cm",
            "drop_object_side_wall_cm",
            "drop_tof_to_object_forward_cm",
            "drop_sharp_to_object_lateral_cm",
            "drop_heading",
            "drop_side",
        ):
            self.vars[field_name][0].trace_add(
                "write", self._update_drop_target_preview
            )

        buttons = ttk.Frame(left)
        buttons.grid(row=2, column=0, sticky="ew", pady=(8, 0))
        for column in range(2):
            buttons.columnconfigure(column, weight=1)
        ttk.Button(
            buttons,
            text="Apply map size",
            command=self.apply_map_size,
            style="Action.TButton",
        ).grid(row=0, column=0, sticky="ew", padx=(0, 3))
        ttk.Button(
            buttons,
            text="Preview A* route",
            command=self.preview_route,
            style="Action.TButton",
        ).grid(row=0, column=1, sticky="ew", padx=(3, 0))
        ttk.Button(buttons, text="Save setup", command=self.save_setup).grid(
            row=1, column=0, sticky="ew", padx=(0, 3), pady=(5, 0)
        )
        ttk.Button(buttons, text="Load setup", command=self.load_setup).grid(
            row=1, column=1, sticky="ew", padx=(3, 0), pady=(5, 0)
        )
        self.start_button = ttk.Button(
            buttons,
            text="▶  START MISSION",
            command=self.start_mission,
            style="Primary.TButton",
        )
        self.start_button.grid(
            row=2, column=0, columnspan=2, sticky="ew", pady=(8, 0)
        )
        self.stop_button = ttk.Button(
            buttons,
            text="■  EMERGENCY STOP",
            command=self.request_stop,
            state="disabled",
            style="Danger.TButton",
        )
        self.stop_button.grid(
            row=3, column=0, columnspan=2, sticky="ew", pady=(5, 0)
        )
        ttk.Button(
            buttons,
            text="Clear red sensor observations",
            command=self.clear_sensor_map,
        ).grid(row=4, column=0, columnspan=2, sticky="ew", pady=(5, 0))

        right = ttk.Frame(outer, style="Panel.TFrame")
        right.grid(row=0, column=1, sticky="nsew")
        right.columnconfigure(0, weight=1)
        right.rowconfigure(2, weight=1)

        tools = tk.Frame(right, bg=colours["panel"], padx=10, pady=8)
        tools.grid(row=0, column=0, sticky="ew")
        tk.Label(
            tools,
            text="Map tools",
            bg=colours["panel"],
            fg="#0f172a",
            font=("Segoe UI", 10, "bold"),
        ).pack(side="left")
        for value, label in (
            ("wall", "Wall"),
            ("start", "Start"),
            ("drop", "Drop"),
            ("exit", "Exit"),
        ):
            ttk.Radiobutton(
                tools, text=label, value=value, variable=self.tool
            ).pack(side="left", padx=5)
        legend = tk.Frame(tools, bg=colours["panel"])
        legend.pack(side="right")
        for text_value, colour in (
            ("Drawn", colours["wall"]),
            ("Sensor", colours["sensor_wall"]),
            ("DROP wall", colours["drop_wall"]),
            ("Travel", colours["travel"]),
            ("A*", colours["route"]),
        ):
            tk.Label(
                legend,
                text="● " + text_value,
                bg=colours["panel"],
                fg=colour,
                font=("Segoe UI", 9, "bold"),
            ).pack(side="left", padx=4)

        tk.Label(
            right,
            textvariable=self.status_var,
            anchor="w",
            bg="#e2e8f0",
            fg="#0f172a",
            padx=10,
            pady=7,
            font=("Segoe UI", 9),
        ).grid(row=1, column=0, sticky="ew", pady=(0, 6))
        self.canvas = tk.Canvas(
            right,
            bg=self.COLOURS["background"],
            highlightthickness=1,
            highlightbackground="#94a3b8",
        )
        self.canvas.grid(row=2, column=0, sticky="nsew")
        self.canvas.bind("<Button-1>", self.on_canvas_click)
        self.canvas.bind("<Button-3>", self.on_canvas_right_click)
        self.canvas.bind("<Configure>", lambda _event: self.redraw())

    def _choose_calibration_file(self, target_var) -> None:
        selected = self.filedialog.askopenfilename(
            title="Select Sharp calibration JSON",
            filetypes=(("JSON calibration", "*.json"), ("All files", "*.*")),
        )
        if selected:
            target_var.set(selected)

    def _update_drop_target_preview(self, *_args) -> None:
        try:
            object_front = float(self.vars["drop_object_front_wall_cm"][0].get())
            object_side = float(self.vars["drop_object_side_wall_cm"][0].get())
            front_offset = float(
                self.vars["drop_tof_to_object_forward_cm"][0].get()
            )
            side_offset = float(
                self.vars["drop_sharp_to_object_lateral_cm"][0].get()
            )
            front_sensor = object_front + front_offset
            side_sensor = object_side - side_offset
            self.drop_target_summary_var.set(
                f"ToF   = {object_front:.1f} + {front_offset:.1f} = "
                f"{front_sensor:.1f} cm\n"
                f"Sharp = {object_side:.1f} - {side_offset:.1f} = "
                f"{side_sensor:.1f} cm"
            )
            heading_name = str(self.vars["drop_heading"][0].get())
            side_name = str(self.vars["drop_side"][0].get())
            heading_index = DIR_FROM_NAME[heading_name]
            side_direction = (
                (heading_index - 1) % 4
                if side_name == "LEFT"
                else (heading_index + 1) % 4
            )
            self.drop_orientation_summary_var.set(
                f"รถหัน {heading_name}: ToF ตรวจผนัง {heading_name} และ "
                f"Sharp {side_name} ตรวจผนัง {HEADINGS[side_direction]}"
            )
        except (KeyError, TypeError, ValueError):
            self.drop_target_summary_var.set("กรอกตัวเลขให้ครบเพื่อคำนวณ")
            self.drop_orientation_summary_var.set(
                "เลือก Drop heading และ LEFT/RIGHT ให้ตรงกับกำแพงจริง"
            )
        if hasattr(self, "canvas"):
            self.redraw()

    def _load_vars_from_config(self) -> None:
        for name, (var, _kind) in self.vars.items():
            var.set(getattr(self.config, name))
        self._update_drop_target_preview()

    def read_config(self) -> HybridConfig:
        # Preserve advanced values that are intentionally not shown in the GUI.
        values = self.config.to_dict()
        for name, (var, kind) in self.vars.items():
            raw = var.get()
            if kind is bool:
                values[name] = bool(raw)
            elif kind is int:
                values[name] = int(float(raw))
            elif kind is float:
                values[name] = float(raw)
            else:
                values[name] = str(raw).strip()
        values["simulation"] = False
        config = HybridConfig.from_dict(values)
        config.validate()
        return config

    def apply_map_size(self) -> None:
        try:
            config = self.read_config()
            if (config.rows, config.cols) != (self.maze.rows, self.maze.cols):
                self.maze.resize(config.rows, config.cols)
            self.config = config
            self.status_var.set(f"Map size: {config.rows} x {config.cols}, cell {config.cell_size_cm:.1f} cm")
            self.redraw()
        except Exception as exc:
            self.messagebox.showerror("Invalid settings", str(exc))

    def _canvas_geometry(self):
        width = max(300, self.canvas.winfo_width())
        height = max(300, self.canvas.winfo_height())
        margin = 44.0
        cell = min((width - 2 * margin) / self.maze.cols, (height - 2 * margin) / self.maze.rows)
        origin_x = (width - cell * self.maze.cols) / 2.0
        origin_y = (height - cell * self.maze.rows) / 2.0
        return origin_x, origin_y, cell

    def _event_cell_and_edge(self, event):
        ox, oy, size = self._canvas_geometry()
        c = int((event.x - ox) // size)
        r = int((event.y - oy) // size)
        cell = (r, c)
        if not self.maze.in_bounds(cell):
            return None, None
        local_x = (event.x - (ox + c * size)) / size
        local_y = (event.y - (oy + r * size)) / size
        distances = {0: local_y, 1: 1.0 - local_x, 2: 1.0 - local_y, 3: local_x}
        edge = min(distances, key=distances.get)
        return cell, edge

    def on_canvas_click(self, event) -> None:
        if self.worker is not None and self.worker.is_alive():
            return
        cell, edge = self._event_cell_and_edge(event)
        if cell is None:
            return
        tool = self.tool.get()
        if tool == "wall":
            neighbour = self.maze.neighbour(cell, edge)
            if self.maze.in_bounds(neighbour):
                # A deliberate operator edit supersedes stale evidence from a
                # previous run on this exact edge.
                self.maze.clear_observed_edge(cell, edge)
                self.maze.toggle_manual_wall(cell, edge)
        else:
            self.maze.set_marker(tool, cell)
        self.maze.planned_path.clear()
        self.redraw()

    def on_canvas_right_click(self, event) -> None:
        if self.worker is not None and self.worker.is_alive():
            return
        cell, edge = self._event_cell_and_edge(event)
        if cell is not None and self.maze.in_bounds(self.maze.neighbour(cell, edge)):
            self.maze.clear_observed_edge(cell, edge)
            self.maze.toggle_manual_wall(cell, edge)
            self.maze.planned_path.clear()
            self.redraw()

    def clear_sensor_map(self) -> None:
        if self.worker is not None and self.worker.is_alive():
            self.messagebox.showinfo(
                "Mission running",
                "Stop the mission before clearing sensor observations.",
            )
            return
        self.maze.clear_sensor_map()
        self.status_var.set("Cleared all red sensor walls; drawn walls were preserved")
        self.redraw()

    def preview_route(self) -> None:
        try:
            self.config = self.read_config()
            route = mission_route_preview(
                self.maze,
                DIR_FROM_NAME[self.config.start_heading],
                self.config.turn_cost,
                self.config.cell_size_cm,
            )
            self.maze.planned_path = route or []
            if route is None:
                self.status_var.set("No complete A* route. Robot can still use online fallback.")
            else:
                distance = max(0, len(route) - 1) * self.config.cell_size_cm
                self.status_var.set(f"A* preview: {len(route) - 1} cells, approximately {distance:.0f} cm")
            self.redraw()
        except Exception as exc:
            self.messagebox.showerror("Cannot preview", str(exc))

    def save_setup(self) -> None:
        try:
            self.config = self.read_config()
            filename = self.filedialog.asksaveasfilename(
                title="Save maze setup",
                defaultextension=".json",
                filetypes=(("JSON", "*.json"),),
            )
            if not filename:
                return
            payload = {"config": self.config.to_dict(), "map": self.maze.to_dict()}
            Path(filename).write_text(json.dumps(payload, indent=2, ensure_ascii=False), encoding="utf-8")
            self.status_var.set(f"Saved setup: {Path(filename).name}")
        except Exception as exc:
            self.messagebox.showerror("Save failed", str(exc))

    def load_setup(self) -> None:
        try:
            filename = self.filedialog.askopenfilename(
                title="Load maze setup",
                filetypes=(("JSON", "*.json"), ("All files", "*.*")),
            )
            if not filename:
                return
            payload = json.loads(Path(filename).read_text(encoding="utf-8"))
            self.config = HybridConfig.from_dict(payload["config"])
            self.config.validate()
            self.maze = GridMazeMap.from_dict(payload["map"])
            self._load_vars_from_config()
            self.robot_visual = None
            self.status_var.set(f"Loaded setup: {Path(filename).name}")
            self.redraw()
        except Exception as exc:
            self.messagebox.showerror("Load failed", str(exc))

    def validate_mission(self) -> None:
        self.config = self.read_config()
        if (self.config.rows, self.config.cols) != (self.maze.rows, self.maze.cols):
            raise ValueError("Press 'Apply map size' after changing rows or columns")
        missing = [name.upper() for name in ("start", "drop", "exit") if getattr(self.maze, name) is None]
        if missing:
            raise ValueError("Place all required markers: " + ", ".join(missing))
        exit_direction = DIR_FROM_NAME[self.config.exit_heading]
        if self.maze.in_bounds(self.maze.neighbour(self.maze.exit, exit_direction)):
            raise ValueError("EXIT marker must be on the border and Exit heading must point outside")

        route = mission_route_preview(
            self.maze,
            DIR_FROM_NAME[self.config.start_heading],
            self.config.turn_cost,
            self.config.cell_size_cm,
        )
        if route is None:
            raise ValueError(
                "The fixed grid has no complete A* route START -> DROP -> EXIT"
            )

        drop_heading = DIR_FROM_NAME[self.config.drop_heading]
        side_direction = (
            (drop_heading - 1) % 4
            if self.config.drop_side == "LEFT"
            else (drop_heading + 1) % 4
        )
        missing_drop_walls = []
        def has_drawn_drop_wall(direction: int) -> bool:
            return (
                not self.maze.in_bounds(self.maze.neighbour(self.maze.drop, direction))
                or (self.maze.drop[0], self.maze.drop[1], direction)
                in self.maze.manual_walls
            )

        if not has_drawn_drop_wall(drop_heading):
            missing_drop_walls.append("FRONT")
        if not has_drawn_drop_wall(side_direction):
            missing_drop_walls.append(self.config.drop_side)
        if missing_drop_walls:
            raise ValueError(
                "DROP alignment points to a missing "
                + "/".join(missing_drop_walls)
                + " wall. Orange solid edges are valid; dashed red-orange "
                "edges require a wall or a different Drop heading/side."
            )
        self.status_var.set(
            f"Fixed-grid route ready: {len(route) - 1} cell moves; DROP faces "
            f"{self.config.drop_heading} with {self.config.drop_side} wall; "
            f"sensor targets ToF={self.config.drop_front_sensor_target_cm:.1f}cm, "
            f"Sharp={self.config.drop_side_sensor_target_cm:.1f}cm."
        )

    def start_mission(self) -> None:
        try:
            self.validate_mission()
        except Exception as exc:
            self.messagebox.showerror("Mission not ready", str(exc))
            return
        answer = self.messagebox.askyesnocancel(
            "Start mode",
            f"DROP configuration\n"
            f"• Robot faces {self.config.drop_heading}; uses "
            f"{self.config.drop_side} Sharp\n"
            f"• Object goal: front {self.config.drop_object_front_wall_cm:.1f} cm, "
            f"side {self.config.drop_object_side_wall_cm:.1f} cm\n"
            f"• Sensor targets: ToF {self.config.drop_front_sensor_target_cm:.1f} cm, "
            f"Sharp {self.config.drop_side_sensor_target_cm:.1f} cm\n"
            f"• Allowed error: ±{self.config.drop_distance_tolerance_cm:.1f} cm\n\n"
            "Yes = connect to the real RoboMaster\n"
            "No = run simulation only\n"
            "Cancel = return to editor",
        )
        if answer is None:
            return
        self.config.simulation = not answer
        self.stop_event.clear()
        self.maze.reset_run_data()
        self.robot_visual = None
        self.start_button.configure(state="disabled")
        self.stop_button.configure(state="normal")

        def worker() -> None:
            try:
                if self.config.simulation:
                    simulate_mission(self.config, self.maze, self.events, self.stop_event)
                else:
                    run_hybrid_robot(self.config, self.maze, self.events, self.stop_event)
            except Exception as exc:
                traceback.print_exc()
                self.events.put(("error", f"{type(exc).__name__}: {exc}"))

        self.worker = threading.Thread(target=worker, name="robomaster-mission", daemon=True)
        self.worker.start()

    def request_stop(self) -> None:
        self.stop_event.set()
        self.status_var.set("STOP requested; waiting for safe shutdown...")

    def poll_events(self) -> None:
        try:
            while True:
                kind, payload = self.events.get_nowait()
                if kind == "status":
                    self.status_var.set(str(payload))
                elif kind == "pose":
                    cell = payload["cell"]
                    direction = payload.get("direction", payload["heading"])
                    fraction = payload.get("fraction", 0.0)
                    dr, dc = DIR_DELTA[direction]
                    self.robot_visual = (cell[0] + dr * fraction, cell[1] + dc * fraction)
                elif kind == "done":
                    self.status_var.set(str(payload))
                    self.start_button.configure(state="normal")
                    self.stop_button.configure(state="disabled")
                elif kind == "error":
                    self.status_var.set(str(payload))
                    self.start_button.configure(state="normal")
                    self.stop_button.configure(state="disabled")
                    self.messagebox.showerror("Mission stopped", str(payload))
                self.redraw()
        except queue.Empty:
            pass
        self.root.after(60, self.poll_events)

    def _edge_coordinates(self, r: int, c: int, direction: int, ox: float, oy: float, size: float):
        x0, y0 = ox + c * size, oy + r * size
        if direction == 0:
            return x0, y0, x0 + size, y0
        if direction == 1:
            return x0 + size, y0, x0 + size, y0 + size
        if direction == 2:
            return x0, y0 + size, x0 + size, y0 + size
        return x0, y0, x0, y0 + size

    def redraw(self) -> None:
        canvas = self.canvas
        canvas.delete("all")
        ox, oy, size = self._canvas_geometry()
        colours = self.COLOURS

        for r in range(self.maze.rows):
            for c in range(self.maze.cols):
                x0, y0 = ox + c * size, oy + r * size
                canvas.create_rectangle(x0, y0, x0 + size, y0 + size, outline=colours["grid"], width=1)
                canvas.create_text(x0 + 5, y0 + 5, text=f"{r},{c}", anchor="nw", fill="#94a3b8", font=("Segoe UI", max(7, int(size * 0.12))))

        # Outer border, with a visible gap at the selected exit edge.
        exit_edge = None
        if self.maze.exit is not None:
            exit_edge = (self.maze.exit[0], self.maze.exit[1], DIR_FROM_NAME.get(str(self.vars["exit_heading"][0].get()), 0))
        for r in range(self.maze.rows):
            for c in range(self.maze.cols):
                for direction in range(4):
                    neighbour = self.maze.neighbour((r, c), direction)
                    if self.maze.in_bounds(neighbour):
                        continue
                    if exit_edge == (r, c, direction):
                        coords = self._edge_coordinates(r, c, direction, ox, oy, size)
                        canvas.create_line(*coords, fill=colours["exit"], width=3, dash=(6, 5))
                    else:
                        canvas.create_line(*self._edge_coordinates(r, c, direction, ox, oy, size), fill=colours["wall"], width=5)

        drawn = set()
        for edge_set, colour, width in (
            (self.maze.manual_walls, colours["wall"], 5),
            (self.maze.sensor_walls, colours["sensor_wall"], 7),
        ):
            for r, c, direction in list(edge_set):
                other = self.maze.neighbour((r, c), direction)
                key = tuple(sorted(((r, c), other)))
                if (key, colour) in drawn:
                    continue
                drawn.add((key, colour))
                canvas.create_line(*self._edge_coordinates(r, c, direction, ox, oy, size), fill=colour, width=width, capstyle="round")

        # Highlight the two walls selected for DROP alignment. Solid orange
        # means that wall exists in the drawn map; dashed red-orange means the
        # orientation points at a missing wall and must be corrected.
        if self.maze.drop is not None:
            heading_name = str(self.vars["drop_heading"][0].get())
            side_name = str(self.vars["drop_side"][0].get())
            if heading_name in DIR_FROM_NAME and side_name in ("LEFT", "RIGHT"):
                drop_heading = DIR_FROM_NAME[heading_name]
                side_direction = (
                    (drop_heading - 1) % 4
                    if side_name == "LEFT"
                    else (drop_heading + 1) % 4
                )
                for direction in (drop_heading, side_direction):
                    neighbour = self.maze.neighbour(self.maze.drop, direction)
                    wall_exists = (
                        not self.maze.in_bounds(neighbour)
                        or (
                            self.maze.drop[0],
                            self.maze.drop[1],
                            direction,
                        )
                        in self.maze.manual_walls
                    )
                    canvas.create_line(
                        *self._edge_coordinates(
                            self.maze.drop[0],
                            self.maze.drop[1],
                            direction,
                            ox,
                            oy,
                            size,
                        ),
                        fill=colours["drop_wall"] if wall_exists else "#ef4444",
                        width=4,
                        dash=None if wall_exists else (5, 4),
                        capstyle="round",
                    )

        def draw_polyline(path, colour, width, dash=None):
            if len(path) < 2:
                return
            points = []
            for r, c in path:
                points.extend((ox + (c + 0.5) * size, oy + (r + 0.5) * size))
            canvas.create_line(*points, fill=colour, width=width, dash=dash, joinstyle="round", capstyle="round")

        draw_polyline(self.maze.planned_path, colours["route"], 3, (7, 5))
        draw_polyline(self.maze.travel_path, colours["travel"], 5)

        for marker, label, colour in (("start", "S", colours["start"]), ("drop", "D", colours["drop"]), ("exit", "E", colours["exit"])):
            cell = getattr(self.maze, marker)
            if cell is None:
                continue
            r, c = cell
            cx, cy = ox + (c + 0.5) * size, oy + (r + 0.5) * size
            radius = max(10, size * 0.20)
            canvas.create_oval(cx - radius, cy - radius, cx + radius, cy + radius, fill=colour, outline="white", width=2)
            canvas.create_text(cx, cy, text=label, fill="white", font=("Segoe UI", max(9, int(size * 0.20)), "bold"))

        if self.robot_visual is not None or self.maze.robot_cell is not None:
            if self.robot_visual is None:
                rr, cc = self.maze.robot_cell
            else:
                rr, cc = self.robot_visual
            cx, cy = ox + (cc + 0.5) * size, oy + (rr + 0.5) * size
            direction = self.maze.robot_heading
            dr, dc = DIR_DELTA[direction]
            radius = max(8, size * 0.16)
            canvas.create_oval(cx - radius, cy - radius, cx + radius, cy + radius, fill=colours["robot"], outline="white", width=2)
            canvas.create_line(cx, cy, cx + dc * radius * 1.7, cy + dr * radius * 1.7, fill="white", width=3, arrow="last")

        canvas.create_text(ox, max(12, oy - 22), text="N ↑", anchor="w", fill="#0f172a", font=("Segoe UI", 11, "bold"))
        canvas.create_text(ox + self.maze.cols * size, max(12, oy - 22), text="E →", anchor="e", fill="#0f172a", font=("Segoe UI", 11, "bold"))

    def on_close(self) -> None:
        if self.worker is not None and self.worker.is_alive():
            if not self.messagebox.askyesno("Mission running", "Request stop and close the window?"):
                return
            self.stop_event.set()
        self.root.destroy()

    def run(self) -> None:
        self.root.mainloop()
