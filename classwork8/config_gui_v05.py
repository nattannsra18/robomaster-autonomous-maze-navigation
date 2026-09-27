"""Pre-mission configuration GUI for Final Round 1 V05.

The user can tune the important robot/mapping parameters without editing source
code.  Values are applied to the provided Classwork8Config instance only for
the current run.
"""

from __future__ import annotations

from typing import Dict, List, Tuple


def configure_before_run(config) -> bool:
    import tkinter as tk
    from tkinter import messagebox, ttk

    root = tk.Tk()
    root.title("Classwork 8 V05 - Mission Configuration")
    # Fit short laptop screens and Windows display scaling. The action bar
    # stays anchored below a scrollable parameter notebook.
    screen_w = root.winfo_screenwidth()
    screen_h = root.winfo_screenheight()
    window_w = max(560, min(920, screen_w - 64))
    window_h = max(460, min(720, screen_h - 100))
    root.geometry("{}x{}".format(window_w, window_h))
    root.minsize(min(650, window_w), min(460, window_h))

    accepted = {"value": False}
    variables: Dict[str, object] = {}

    outer = ttk.Frame(root, padding=12)
    outer.pack(fill="both", expand=True)

    ttk.Label(
        outer,
        text="Classwork 8 V05 - Mission Configuration",
        font=("Segoe UI", 17, "bold"),
    ).pack(anchor="w")

    ttk.Label(
        outer,
        text=(
            "Planner: Nearest-Frontier BFS + closed-maze auto completion. "
            "Configure the run here; no source-code editing is required."
        ),
        wraplength=800,
    ).pack(anchor="w", pady=(2, 10))

    notebook = ttk.Notebook(outer)

    # Each tab has its own vertical scrollbar. A long Target Detection tab
    # must never push the bottom Start/Cancel buttons outside the screen.
    tabs = {}
    tab_canvases = {}
    tab_names = (
        "Motion", "ToF / Safety", "Mapping",
        "Target Detection", "Completion / Export",
    )
    for name in tab_names:
        tab_container = ttk.Frame(notebook)
        notebook.add(tab_container, text=name)
        canvas = tk.Canvas(tab_container, highlightthickness=0, borderwidth=0)
        scrollbar = ttk.Scrollbar(
            tab_container, orient="vertical", command=canvas.yview
        )
        canvas.configure(yscrollcommand=scrollbar.set)
        scrollbar.pack(side="right", fill="y")
        canvas.pack(side="left", fill="both", expand=True)

        content = ttk.Frame(canvas, padding=12)
        content_id = canvas.create_window((0, 0), window=content, anchor="nw")

        def _update_scrollregion(_event, active_canvas=canvas):
            active_canvas.configure(scrollregion=active_canvas.bbox("all"))

        def _fit_content(event, active_canvas=canvas, item_id=content_id):
            active_canvas.itemconfigure(item_id, width=event.width)

        content.bind("<Configure>", _update_scrollregion)
        canvas.bind("<Configure>", _fit_content)
        tabs[name] = content
        tab_canvases[name] = canvas

    def _scroll_current_tab(event):
        try:
            name = tab_names[notebook.index(notebook.select())]
            delta = getattr(event, "delta", 0)
            if delta:
                steps = -max(1, abs(int(delta / 120))) if delta > 0 else max(1, abs(int(delta / 120)))
            else:
                steps = -1 if getattr(event, "num", 0) == 4 else 1
            tab_canvases[name].yview_scroll(steps, "units")
        except Exception:
            pass

    root.bind("<MouseWheel>", _scroll_current_tab, add="+")
    root.bind("<Button-4>", _scroll_current_tab, add="+")
    root.bind("<Button-5>", _scroll_current_tab, add="+")

    field_specs: Dict[str, List[Tuple[str, str, str, str]]] = {
        "Motion": [
            ("cell_size_m", "Cell size (m)", "float", "Physical maze cell; assignment default = 0.60"),
            ("step_tolerance_m", "Cell stop tolerance (m)", "float", "0.005 means stop around 59.5 cm in calibrated odometry"),
            ("travel_speed_mps", "Travel speed (m/s)", "float", "Use 0.08-0.12 while tuning"),
            ("odom_scale_x", "Odometry scale X", "float", "Start at 1.00; tune with a measured 60 cm forward test"),
            ("odom_scale_y", "Odometry scale Y", "float", "Start at 1.00; tune with a measured 60 cm strafe test"),
            ("cross_track_kp", "Cross-track Kp", "float", "Correct cell-centre drift"),
            ("cross_track_max_mps", "Max cross-track correction (m/s)", "float", "Limit lateral correction"),
            ("cell_center_tolerance_m", "Cell-centre tolerance (m)", "float", "Allowed perpendicular error at the end of a cell"),
            ("heading_kp_z", "Heading Kp", "float", "Yaw correction gain"),
            ("heading_deadband_deg", "Heading deadband (deg)", "float", "Ignore tiny yaw noise"),
            ("heading_recover_release_deg", "Pause-translation yaw error (deg)", "float", "Above this, V05 corrects yaw before translating"),
            ("heading_recover_trigger_deg", "Hard yaw recovery trigger (deg)", "float", "Use stronger correction above this error"),
            ("heading_max_z_dps", "Max yaw correction speed", "float", "Normal heading correction limit"),
            ("heading_recover_max_z_dps", "Max hard-recovery yaw speed", "float", "Recovery limit"),
        ],
        "ToF / Safety": [
            ("tof_open_cm", "Open direction threshold (cm)", "float", ">= this value is an open candidate"),
            ("scan_hard_wall_cm", "Hard-wall threshold (cm)", "float", "<= this is confidently a wall"),
            ("stop_front_cm", "Emergency front stop (cm)", "float", "Movement stops before collision"),
            ("slow_front_cm", "Slow-down threshold (cm)", "float", "Start reducing travel speed"),
            ("tof_recovery_wait_sec", "ToF recovery wait (s)", "float", "Wait for fresh ToF after gimbal/filter reset"),
            ("tof_recovery_retries", "ToF recovery retries", "int", "Extra attempts before ending the run"),
            ("scan_samples", "Scan samples", "int", "Median samples per gimbal direction"),
            ("scan_sample_interval_sec", "Scan sample interval (s)", "float", "Delay between ToF samples"),
            ("front_block_confirm_samples", "Blocked confirmation samples", "int", "Consecutive low readings before declaring blocked"),
            ("scan_side_guidance_enabled", "Use scan-side centering", "bool", "Small temporary correction from the stopped 4-way scan"),
            ("scan_side_max_correction_mps", "Max scan-side correction (m/s)", "float", "Keep conservative without live side sensors"),
            ("gimbal_yaw_speed_dps", "Yaw max speed (deg/s)", "float", "V05 staged yaw-only default 40; slower for smooth diagnostic turns"),
            ("gimbal_min_yaw_speed_dps", "Yaw minimum speed (deg/s)", "float", "Low-speed correction near a requested scan direction"),
            ("gimbal_yaw_kp", "Yaw correction Kp", "float", "Smooth proportional yaw-only controller"),
            ("gimbal_turn_timeout_sec", "Gimbal orientation timeout (s)", "float", "Whole pitch -> yaw -> pitch sequence"),
            ("gimbal_yaw_pitch_guard_deg", "Pitch guard during yaw (deg)", "float", "Stop scan if yaw-only rotation physically tilts the gimbal too far"),
            ("gimbal_scan_pitch_deg", "Scan pitch target (deg)", "float", "Horizontal relative gimbal pitch; start with 0 degrees"),
            ("gimbal_pitch_kp", "Pitch correction Kp", "float", "Use a gentle gain to avoid nodding during yaw sweeps"),
            ("gimbal_pitch_min_speed_dps", "Minimum pitch speed (deg/s)", "float", "Lower than the previous 4 deg/s to reduce overshoot near level"),
            ("gimbal_pitch_max_speed_dps", "Maximum pitch speed (deg/s)", "float", "Limit visible pitch movement"),
            ("gimbal_pitch_drive_sign", "Pitch direction sign (+1/-1)", "float", "Only reverse after a stationary feedback test confirms pitch moves opposite"),
            ("gimbal_pitch_tolerance_deg", "Pitch tolerance (deg)", "float", "0.8 deg default; earlier 2 deg allowed noticeable nodding"),
            ("gimbal_pitch_unsafe_deg", "Unsafe pitch drift (deg)", "float", "Movement stops and re-levels above this error"),
        ],
        "Mapping": [
            ("resolution_m", "Occupancy resolution (m)", "float", "Assignment map resolution; default = 0.05"),
            ("map_width_m", "Working canvas width (m)", "float", "Internal export canvas, not prior field knowledge"),
            ("map_height_m", "Working canvas height (m)", "float", "Internal export canvas, not prior field knowledge"),
            ("max_moves", "Maximum cell moves", "int", "Safety cap"),
            ("free_delta", "Free evidence delta", "int", "Occupancy evidence update"),
            ("occupied_delta", "Occupied evidence delta", "int", "Occupancy evidence update"),
        ],
        "Completion / Export": [
            ("closed_maze_auto_stop", "Closed-maze auto stop", "bool", "Stop when the discovered rectangular arena is fully visited and its outer perimeter is wall-confirmed"),
            ("closed_maze_perimeter_wall_ratio", "Perimeter wall ratio", "float", "0.70 tolerates one missed low-foam wall reading on a short side"),
            ("closed_maze_min_rows", "Minimum rows before auto stop", "int", "Prevents tiny early rectangles from completing the mission"),
            ("closed_maze_min_cols", "Minimum columns before auto stop", "int", "Prevents tiny early rectangles from completing the mission"),
            ("gui_auto_save_map", "Auto-save GUI map PNG", "bool", "Save gui_map.png in the same run output folder when the mission finishes"),
            ("gui_export_width_px", "GUI export width (px)", "int", "PNG export width"),
            ("gui_export_height_px", "GUI export height (px)", "int", "PNG export height"),
        ],
        "Target Detection": [
            ("target_detection_enabled", "Enable camera target survey", "bool", "Round 1 detects color + shape while the gimbal already scans ToF"),
            ("target_camera_resolution", "Camera resolution", "choice", "360p is recommended for low latency"),
            ("target_camera_pitch_deg", "Target observation pitch (deg)", "float", "Default -10 looks lower during stationary camera survey; horizontal ToF remains 0"),
            ("target_preview_fps", "Live preview FPS", "float", "Independent annotated camera preview; default 8"),
            ("target_survey_open_directions", "Survey OPEN directions too", "bool", "Find low signs even if horizontal ToF says the path ahead is open (range marked unconfirmed)"),
            ("target_min_confidence", "Candidate confidence", "float", "Reject weak single-frame detections below this value"),
            ("target_save_confidence", "Save confidence", "float", "Temporal track must exceed this value before entering targets.json"),
            ("target_sample_frames", "Frames per scan direction", "int", "How many latest frames are sampled while gimbal is stationary"),
            ("target_verify_frames", "Required matching frames", "int", "Minimum repeated detections before a target is verified"),
            ("target_frame_interval_sec", "Frame interval (s)", "float", "Small delay between temporal verification samples"),
            ("target_verify_max_jump_px", "Max centroid jump (px)", "float", "Keeps temporal verification on the same object"),
            ("target_merge_centroid_px", "Same-view merge threshold (px)", "float", "Keep adjacent same-color signs separate; default 18px"),
            ("target_clahe_clip_limit", "CLAHE clip limit", "float", "Lighting normalization strength on Lab-L"),
            ("target_roi_top_ratio", "Target ROI top (0-1)", "float", "Exclude non-target ceiling/background; tune using the camera debug frame"),
            ("target_roi_bottom_ratio", "Target ROI bottom (0-1)", "float", "Exclude reflective floor; current field default = 0.82"),
            ("target_roi_border_margin_px", "ROI border margin (px)", "int", "Reject cropped contours touching the detection region"),
            ("target_min_contour_area_px", "Minimum contour area (px)", "float", "Reject small tape and floor noise"),
            ("target_rectangularity_min", "Rectangle fill minimum", "float", "Square/rectangle geometry threshold"),
            ("target_square_aspect_min", "Square aspect min", "float", "Lower W/H bound for square"),
            ("target_square_aspect_max", "Square aspect max", "float", "Upper W/H bound for square"),
            ("target_circle_circularity_min", "Circle circularity min", "float", "Circle geometry threshold"),
        ],
    }

    help_labels = []

    def add_field(parent, attr, label, kind, help_text, row):
        ttk.Label(parent, text=label).grid(
            row=row, column=0, sticky="w", padx=(0, 10), pady=5
        )

        current = getattr(config, attr)

        if kind == "bool":
            var = tk.BooleanVar(value=bool(current))
            widget = ttk.Checkbutton(parent, variable=var)
        elif kind == "choice":
            var = tk.StringVar(value=str(current))
            widget = ttk.Combobox(
                parent,
                textvariable=var,
                values=("360p", "540p", "720p"),
                state="readonly",
                width=18,
            )
        else:
            var = tk.StringVar(value=str(current))
            widget = ttk.Entry(parent, textvariable=var, width=20)

        variables[attr] = (var, kind)
        widget.grid(row=row, column=1, sticky="ew", pady=5)

        help_label = ttk.Label(
            parent,
            text=help_text,
            foreground="#64748b",
            wraplength=390,
        )
        help_label.grid(row=row, column=2, sticky="w", padx=(12, 0), pady=5)
        help_labels.append(help_label)

    for tab_name, specs in field_specs.items():
        parent = tabs[tab_name]
        parent.columnconfigure(1, weight=0)
        parent.columnconfigure(2, weight=1)

        for row, spec in enumerate(specs):
            add_field(parent, *spec, row=row)

    info = ttk.LabelFrame(outer, text="60 cm calibration", padding=10)
    # Pack the footer after the action bar is created so the Start button
    # remains on the bottom edge of the window.

    calibration_var = tk.StringVar()
    ttk.Label(
        info,
        textvariable=calibration_var,
        wraplength=790,
        justify="left",
    ).pack(anchor="w")

    def refresh_calibration_text(*_args):
        try:
            cell = float(variables["cell_size_m"][0].get())
            tol = float(variables["step_tolerance_m"][0].get())
            sx = float(variables["odom_scale_x"][0].get())
            sy = float(variables["odom_scale_y"][0].get())
            calibration_var.set(
                "Controller target: {:.1f} cm before tolerance; "
                "completion threshold ≈ {:.1f} cm in calibrated map coordinates. "
                "Current odom scale X/Y = {:.3f}/{:.3f}. "
                "For a tape-measure test, if a commanded 60 cm move physically "
                "travels D cm, adjust scale approximately to old_scale × D/60.".format(
                    cell * 100.0,
                    max(0.0, cell - tol) * 100.0,
                    sx,
                    sy,
                )
            )
        except Exception:
            calibration_var.set("Enter valid numeric motion values to see calibration guidance.")

    for attr in ("cell_size_m", "step_tolerance_m", "odom_scale_x", "odom_scale_y"):
        variables[attr][0].trace_add("write", refresh_calibration_text)

    refresh_calibration_text()

    button_row = ttk.Frame(outer)
    button_row.pack(side="bottom", fill="x", pady=(8, 0))
    info.pack(side="bottom", fill="x", pady=(8, 0))

    def set_v05_defaults():
        defaults = {
            "cell_size_m": 0.60,
            "step_tolerance_m": 0.005,
            "travel_speed_mps": 0.10,
            "odom_scale_x": 1.00,
            "odom_scale_y": 1.00,
            "cross_track_kp": 0.70,
            "cross_track_max_mps": 0.045,
            "cell_center_tolerance_m": 0.035,
            "heading_kp_z": 2.4,
            "heading_deadband_deg": 0.35,
            "heading_recover_release_deg": 1.0,
            "heading_recover_trigger_deg": 4.0,
            "heading_max_z_dps": 18.0,
            "heading_recover_max_z_dps": 24.0,
            "tof_open_cm": 55.0,
            "scan_hard_wall_cm": 25.0,
            "stop_front_cm": 18.0,
            "slow_front_cm": 35.0,
            "tof_recovery_wait_sec": 1.20,
            "tof_recovery_retries": 2,
            "scan_samples": 5,
            "scan_sample_interval_sec": 0.06,
            "front_block_confirm_samples": 4,
            "scan_side_guidance_enabled": True,
            "scan_side_max_correction_mps": 0.015,
            "gimbal_yaw_speed_dps": 40.0,
            "gimbal_min_yaw_speed_dps": 8.0,
            "gimbal_yaw_kp": 1.0,
            "gimbal_turn_timeout_sec": 10.0,
            "gimbal_yaw_pitch_guard_deg": 6.0,
            "gimbal_scan_pitch_deg": 0.0,
            "gimbal_pitch_kp": 0.9,
            "gimbal_pitch_min_speed_dps": 1.5,
            "gimbal_pitch_max_speed_dps": 10.0,
            "gimbal_pitch_drive_sign": 1.0,
            "gimbal_pitch_tolerance_deg": 0.8,
            "gimbal_pitch_unsafe_deg": 6.0,
            "resolution_m": 0.05,
            "map_width_m": 8.0,
            "map_height_m": 8.0,
            "max_moves": 500,
            "free_delta": -2,
            "occupied_delta": 5,
            "closed_maze_auto_stop": True,
            "closed_maze_perimeter_wall_ratio": 0.70,
            "closed_maze_min_rows": 2,
            "closed_maze_min_cols": 2,
            "gui_auto_save_map": True,
            "gui_export_width_px": 1200,
            "gui_export_height_px": 900,
            "target_detection_enabled": True,
            "target_camera_resolution": "360p",
            "target_camera_pitch_deg": -10.0,
            "target_preview_fps": 8.0,
            "target_survey_open_directions": True,
            "target_min_confidence": 0.50,
            "target_save_confidence": 0.60,
            "target_sample_frames": 6,
            "target_verify_frames": 4,
            "target_frame_interval_sec": 0.045,
            "target_verify_max_jump_px": 50.0,
            "target_merge_centroid_px": 18.0,
            "target_merge_distance_m": 0.40,
            "target_clahe_clip_limit": 2.0,
            "target_roi_top_ratio": 0.18,
            "target_roi_bottom_ratio": 0.82,
            "target_roi_border_margin_px": 3,
            "target_min_contour_area_px": 300.0,
            "target_rectangularity_min": 0.58,
            "target_square_aspect_min": 0.85,
            "target_square_aspect_max": 1.16,
            "target_circle_circularity_min": 0.70,
        }

        for attr, value in defaults.items():
            if attr not in variables:
                continue
            var, kind = variables[attr]
            if kind == "bool":
                var.set(bool(value))
            else:
                var.set(str(value))

    def apply_and_start():
        try:
            for attr, (var, kind) in variables.items():
                if kind == "bool":
                    value = bool(var.get())
                elif kind == "int":
                    value = int(var.get())
                elif kind == "float":
                    value = float(var.get())
                else:
                    value = str(var.get())

                setattr(config, attr, value)

            # One logical step is always exactly one physical cell.
            config.exploration_step_m = float(config.cell_size_m)
            config.validate()

        except Exception as exc:
            messagebox.showerror(
                "Invalid configuration",
                str(exc),
                parent=root,
            )
            return

        accepted["value"] = True
        root.destroy()

    def cancel():
        accepted["value"] = False
        root.destroy()

    ttk.Button(
        button_row,
        text="Reset V05 defaults",
        command=set_v05_defaults,
    ).pack(side="left")

    ttk.Button(
        button_row,
        text="Cancel",
        command=cancel,
    ).pack(side="right", padx=(8, 0))

    ttk.Button(
        button_row,
        text="Apply & Connect",
        command=apply_and_start,
    ).pack(side="right")

    # Packing this last leaves the button and calibration footer permanently
    # visible, while the selected tab receives the remaining height.
    notebook.pack(side="top", fill="both", expand=True)
    root.protocol("WM_DELETE_WINDOW", cancel)
    root.mainloop()

    return bool(accepted["value"])
