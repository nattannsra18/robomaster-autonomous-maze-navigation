from dataclasses import dataclass, asdict


@dataclass
class Classwork8Config:
    """Runtime configuration for ToF-only unknown-world exploration."""

    connection: str = "ap"

    # Empty working canvas only; no maze layout is preloaded.
    map_width_m: float = 8.0
    map_height_m: float = 8.0
    resolution_m: float = 0.05

    # Physical maze cell size. One exploration move = one whole cell.
    cell_size_m: float = 0.60
    exploration_step_m: float = 0.60
    step_tolerance_m: float = 0.03
    cross_track_kp: float = 0.70
    cross_track_max_mps: float = 0.045

    # V02 motion calibration.
    #
    # The 2026-09-25 field log showed that three nominal RIGHT cell moves
    # changed the same forward-wall ToF range by about 215 cm while wheel
    # odometry reported only about 172 cm.  Scale wheel odometry into the
    # physical maze frame so one logical 60 cm cell does not overshoot.
    # Tune these two values with a tape-measure test if the floor changes.
    odom_scale_x: float = 1.25
    odom_scale_y: float = 1.25

    # V02 fixed-heading controller.  The chassis is supposed to keep the
    # mission-start yaw while mecanum-translating; do not wait for an 8-degree
    # error before pausing translation and recovering.
    heading_kp_z: float = 2.4
    heading_max_z_dps: float = 18.0
    heading_deadband_deg: float = 0.35
    heading_recover_trigger_deg: float = 4.0
    heading_recover_release_deg: float = 1.0
    heading_recover_max_z_dps: float = 24.0
    heading_drive_sign: float = 1.0

    # V02 ToF robustness.  Gimbal direction changes clear the ToF filter, so
    # wait briefly for a genuinely fresh sample instead of aborting instantly.
    tof_recovery_wait_sec: float = 0.90
    tof_recovery_retries: int = 2
    front_block_confirm_samples: int = 4
    front_block_confirm_interval_sec: float = 0.05
    front_block_release_margin_cm: float = 3.0

    # V02 scan-derived corridor guidance.  The single ToF cannot look forward
    # and sideways simultaneously, so use the four-way scan made while stopped
    # to add only a small lateral bias during the next 60 cm move.
    scan_side_guidance_enabled: bool = True
    scan_side_wall_max_cm: float = 45.0
    scan_side_danger_cm: float = 22.0
    scan_side_kp_mps_per_cm: float = 0.0020
    scan_side_max_correction_mps: float = 0.015

    # V05 lateral protection using only the existing gimbal ToF. The robot
    # stops halfway along a 60 cm leg ONLY when a mapped side has a confirmed
    # wall, measures that wall while stationary, and then returns ToF to the
    # travel direction before driving resumes. Never scan sideways in motion.
    # These ranges are ToF sensor-to-wall, not chassis-side physical clearance.
    wall_follow_recovery_enabled: bool = False  # opt in through Final Round-1 GUI
    midcell_side_check_enabled: bool = False
    midcell_side_check_ratio: float = 0.48
    # These are sensor-to-wall distances, NOT physical chassis clearance.
    # Field log: RIGHT was 14.5 cm at start and 14.7 cm mid-leg; an old 22 cm
    # absolute stop falsely aborted normal travel. Calibrate the critical
    # range using measured sensor-to-body offset before a full maze trial.
    midcell_side_hard_stop_cm: float = 10.0
    # Only an independently repeated stationary SIDE measurement can clear
    # one suspicious low range. Sustained 6.5 cm must still halt the chassis.
    side_start_recheck_enabled: bool = True
    side_start_release_margin_cm: float = 2.0
    side_start_recheck_max_spread_cm: float = 2.0
    midcell_side_soft_margin_cm: float = 18.0
    midcell_side_max_baseline_drop_cm: float = 4.0
    midcell_side_recenter_deadband_cm: float = 1.5
    midcell_side_max_bias_mps: float = 0.012
    motion_total_lateral_max_mps: float = 0.028
    # Explicit cap for wall-adjacent legs: 0.25 m/s is NOT used next to foam walls.
    motion_wall_adjacent_speed_cap_mps: float = 0.12
    # Abort if wheel odometry reports excessive departure from the planned
    # logical cell centreline; it cannot detect unreported wheel slip.
    motion_cross_track_abort_m: float = 0.085
    motion_cross_track_slow_m: float = 0.030
    motion_slow_cross_track_speed_mps: float = 0.065

    # If a confirmed wall appears only after most of a cell has already been
    # traversed, keep the logical DFS state synchronized with the physical
    # robot instead of pretending it never left the previous cell.
    blocked_near_target_accept_ratio: float = 0.82
    cell_center_tolerance_m: float = 0.035

    # ToF is mounted on the gimbal. The chassis stays at its initial heading
    # during scanning; the gimbal points ToF toward the scan/travel direction.
    tof_forward_offset_m: float = 0.12
    tof_max_mapping_cm: float = 300.0
    mapping_min_cm: float = 3.0

    # Gimbal yaw positions relative to the chassis.
    gimbal_front_yaw_deg: float = 0.0
    gimbal_right_yaw_deg: float = 90.0
    gimbal_back_yaw_deg: float = 180.0
    gimbal_left_yaw_deg: float = -90.0

    # Closed-loop gimbal scan tuning. The actual relative yaw is read from
    # gimbal.sub_angle(), so the mapper does not assume the gimbal reached target.
    # Turn slowly with YAW ONLY; restore level PITCH after yaw finishes.
    # The observed transient yaw/pitch excursion is logged, but ToF is sampled
    # only AFTER both axes settle to their configured scan angles.
    gimbal_yaw_speed_dps: float = 110.0
    gimbal_min_yaw_speed_dps: float = 14.0
    gimbal_yaw_kp: float = 2.4
    gimbal_tolerance_deg: float = 2.5
    gimbal_stable_samples: int = 3
    gimbal_turn_timeout_sec: float = 10.0
    gimbal_settle_sec: float = 0.20
    gimbal_yaw_pitch_guard_deg: float = 6.0

    # V05 ToF/camera both share the gimbal: yaw-only control does not keep
    # pitch level. Hold the horizontal mission scan pitch using feedback.
    # A positive pitch speed should increase SDK pitch feedback; reverse
    # gimbal_pitch_drive_sign only if a stationary hardware test proves otherwise.
    gimbal_scan_pitch_deg: float = 0.0
    # The 27 Sep stationary log alternated around -1.7 / +1.7 deg because
    # the old 2 deg tolerance accepted both endpoints. Slow correction near
    # level and use a tighter acceptance window; do not force a 4 deg/s pulse.
    gimbal_pitch_kp: float = 2.0
    gimbal_pitch_min_speed_dps: float = 4.0
    gimbal_pitch_max_speed_dps: float = 35.0
    gimbal_pitch_tolerance_deg: float = 0.8
    gimbal_pitch_unsafe_deg: float = 6.0
    gimbal_pitch_drive_sign: float = 1.0

    # Occupancy evidence
    free_delta: int = -2
    occupied_delta: int = 5
    min_score: int = -20
    max_score: int = 20
    occupied_threshold: int = 3
    free_threshold: int = -2

    # ToF-only exploration. A wall in the current cell is typically about
    # 30 cm from chassis centre. This threshold only marks candidate directions;
    # movement still continuously checks ToF and stops at stop_front_cm.
    tof_open_cm: float = 55.0
    # V02 topology classification: a very close return is confidently a wall.
    # Mid-range returns are re-sampled and biased toward OPEN because a false
    # open is still protected by the continuous front-stop guard, whereas a
    # false wall can permanently hide an unexplored branch.
    scan_hard_wall_cm: float = 25.0
    scan_ambiguous_retries: int = 1
    scan_ambiguous_retry_settle_sec: float = 0.10
    scan_samples: int = 5
    scan_sample_interval_sec: float = 0.06
    max_moves: int = 500

    # Conservative mecanum translation. No 90-degree chassis scan turns.
    # Requested open-route ceiling. Mapped wall-adjacent legs remain speed-capped.
    travel_speed_mps: float = 0.20
    stop_front_cm: float = 18.0
    slow_front_cm: float = 35.0
    drive_timeout_sec: float = 0.15
    loop_delay_sec: float = 0.04

    # Tiny yaw corrections are allowed while translating so the chassis keeps
    # its original orientation. Set False if absolutely no z correction is wanted.
    heading_hold_enabled: bool = True

    # Camera-assisted corridor centering. ToF remains authoritative for walls
    # and stopping; vision only adds a small lateral correction when reliable.
    vision_enabled: bool = True
    # Default to camera diagnostics only until the direction and wall/floor
    # boundaries have been verified on the actual foam-wall maze.
    vision_steering_enabled: bool = False
    vision_resolution: str = "360p"
    vision_start_timeout_sec: float = 4.0
    vision_max_age_sec: float = 0.40
    vision_min_confidence: float = 0.70
    vision_kp_mps: float = 0.035
    vision_max_correction_mps: float = 0.025
    vision_error_ema_alpha: float = 0.35
    vision_roi_top_ratio: float = 0.42
    vision_blur_kernel: int = 5
    vision_canny_low: int = 45
    vision_canny_high: int = 130
    vision_hough_threshold: int = 32
    vision_min_line_length_px: int = 35
    vision_max_line_gap_px: int = 24
    vision_min_side_angle_deg: float = 24.0
    # Reject short distant segments and long extrapolations to image bottom.
    # The previous image included a distant inner-wall edge as a left wall.
    vision_min_side_vertical_ratio: float = 0.20
    vision_max_bottom_gap_ratio: float = 0.13
    vision_center_deadband_ratio: float = 0.10
    vision_max_candidate_spread_ratio: float = 0.12
    vision_min_corridor_width_ratio: float = 0.25
    vision_max_corridor_width_ratio: float = 1.10

    # Final Assignment Round 1 - camera target survey.
    skip_scanned_visited_cells: bool = True
    target_detection_enabled: bool = True
    target_camera_resolution: str = "360p"
    target_camera_start_timeout_sec: float = 5.0
    target_max_frame_age_sec: float = 0.60
    # Camera looks down only while stopped, after the horizontal ToF ray has
    # been sampled. Mapping/driving ALWAYS restore gimbal_scan_pitch_deg.
    target_camera_pitch_deg: float = -15.0
    target_camera_pitch_min_deg: float = -20.0
    target_camera_pitch_max_deg: float = 10.0
    target_camera_pitch_tolerance_deg: float = 1.5
    target_camera_pitch_timeout_sec: float = 4.5
    target_camera_settle_sec: float = 0.15
    target_preview_fps: float = 10.0
    target_survey_open_directions: bool = True

    # Lighting-robust OpenCV detector.
    target_clahe_clip_limit: float = 2.0
    target_clahe_grid: int = 8
    target_morph_kernel: int = 3
    # Ground-level signs may appear in the lower camera frame after looking
    # down. Keep the visible band adjustable in the running GUI: extending it
    # can also admit reflective floor clutter, so inspect the live overlay.
    # These are camera-image ratios, never a preloaded maze layout.
    target_roi_top_ratio: float = 0.18
    target_roi_bottom_ratio: float = 0.96
    target_roi_border_margin_px: int = 3
    target_min_contour_area_px: float = 300.0
    target_min_contour_area_ratio: float = 0.0010
    target_max_contour_area_ratio: float = 0.35
    target_approx_epsilon_ratio: float = 0.030
    target_rectangularity_min: float = 0.58
    target_square_aspect_min: float = 0.85
    target_square_aspect_max: float = 1.16
    target_circle_circularity_min: float = 0.70
    target_circle_min_vertices: int = 6

    # Candidate confidence + temporal verification.
    target_min_confidence: float = 0.50
    target_save_confidence: float = 0.60
    target_sample_frames: int = 8
    target_verify_frames: int = 3
    # Hold a stationary target view for additional independent-frame checks
    # if some visible candidates were not yet verified. Bound the delay.
    target_hold_max_sec: float = 3.0
    target_hold_max_windows: int = 3
    target_pending_min_frames: int = 2
    target_frame_interval_sec: float = 0.040
    target_verify_max_jump_px: float = 50.0
    target_merge_centroid_px: float = 18.0

    # Metric wall-face coordinates from one ToF ray cannot resolve lateral
    # target offsets: never merge different signs based on ToF range alone.
    # This radius is reserved for later cross-view registration/calibration.
    target_merge_distance_m: float = 0.40

    # V04 closed-maze completion.
    #
    # The classwork arena is a closed rectangular maze.  A single ToF miss on
    # a low foam boundary can leave a phantom OPEN frontier forever.  V04 may
    # therefore finish when all cells inside the discovered bounding rectangle
    # have been visited and each outer side is sufficiently wall-confirmed.
    closed_maze_auto_stop: bool = True
    closed_maze_perimeter_wall_ratio: float = 0.70
    closed_maze_min_rows: int = 2
    closed_maze_min_cols: int = 2

    # GUI / export
    gui_refresh_ms: int = 150
    gui_canvas_px: int = 720
    gui_auto_save_map: bool = True
    gui_export_width_px: int = 1200
    gui_export_height_px: int = 900

    output_dir: str = "classwork8_output"

    def validate(self) -> None:
        if self.connection not in ("ap", "sta", "rndis"):
            raise ValueError("connection must be ap, sta, or rndis")
        if self.resolution_m <= 0.0:
            raise ValueError("resolution_m must be positive")
        if self.map_width_m <= self.resolution_m or self.map_height_m <= self.resolution_m:
            raise ValueError("map dimensions must be larger than one cell")
        if self.cell_size_m <= 0.0 or self.exploration_step_m <= 0.0:
            raise ValueError("cell/step size must be positive")
        if abs(self.cell_size_m - self.exploration_step_m) > 1e-9:
            raise ValueError("this classwork mode expects one move per physical cell")
        if not -20.0 <= self.gimbal_scan_pitch_deg <= 20.0:
            raise ValueError("gimbal_scan_pitch_deg must be between -20 and 20")
        if self.gimbal_pitch_drive_sign not in (-1.0, 1.0):
            raise ValueError("gimbal_pitch_drive_sign must be -1.0 or +1.0")
        if self.gimbal_pitch_min_speed_dps <= 0.0 or (
            self.gimbal_pitch_max_speed_dps < self.gimbal_pitch_min_speed_dps
        ):
            raise ValueError("gimbal pitch speeds are invalid")
        if self.gimbal_pitch_kp <= 0.0 or self.gimbal_pitch_tolerance_deg <= 0.0:
            raise ValueError("gimbal pitch gain/tolerance must be positive")
        if self.gimbal_pitch_unsafe_deg <= self.gimbal_pitch_tolerance_deg:
            raise ValueError("gimbal pitch unsafe angle must exceed tolerance")
        if self.gimbal_yaw_pitch_guard_deg <= self.gimbal_pitch_tolerance_deg:
            raise ValueError("gimbal yaw pitch guard must exceed pitch tolerance")
        if not 0.0 < self.gimbal_min_yaw_speed_dps <= self.gimbal_yaw_speed_dps <= 120.0:
            raise ValueError("gimbal yaw speed must be above minimum and at most 120 deg/s")
        if self.gimbal_turn_timeout_sec <= 0.0:
            raise ValueError("gimbal turn timeout must be positive")
        if self.tof_max_mapping_cm <= self.mapping_min_cm:
            raise ValueError("ToF mapping range is invalid")
        if self.tof_open_cm <= self.stop_front_cm:
            raise ValueError("tof_open_cm must be greater than stop_front_cm")
        if self.travel_speed_mps <= 0.0:
            raise ValueError("travel_speed_mps must be positive")
        if self.odom_scale_x <= 0.0 or self.odom_scale_y <= 0.0:
            raise ValueError("odometry scale factors must be positive")
        if self.heading_drive_sign == 0.0:
            raise ValueError("heading_drive_sign must be non-zero")
        if self.tof_recovery_wait_sec <= 0.0:
            raise ValueError("tof_recovery_wait_sec must be positive")
        if self.tof_recovery_retries < 0:
            raise ValueError("tof_recovery_retries must be >= 0")
        if self.cell_center_tolerance_m <= 0.0:
            raise ValueError("cell_center_tolerance_m must be positive")
        if not 0.10 <= self.midcell_side_check_ratio <= 0.85:
            raise ValueError("midcell_side_check_ratio must be between 0.10 and 0.85")
        if not 0.0 < self.midcell_side_hard_stop_cm < self.midcell_side_soft_margin_cm <= self.scan_side_wall_max_cm:
            raise ValueError("midcell side clearance thresholds must increase up to scan_side_wall_max_cm")
        if not 0.0 < self.side_start_release_margin_cm <= 5.0:
            raise ValueError("side start release margin must be 0..5 cm")
        if not 0.0 < self.side_start_recheck_max_spread_cm <= 5.0:
            raise ValueError("side start recheck max spread must be 0..5 cm")
        if not 0.0 < self.midcell_side_max_baseline_drop_cm <= 20.0:
            raise ValueError("midcell_side_max_baseline_drop_cm must be 0..20 cm")
        if not 0.0 <= self.midcell_side_recenter_deadband_cm < self.midcell_side_max_baseline_drop_cm:
            raise ValueError("midcell side recenter deadband must be less than baseline-drop stop")
        if not 0.0 <= self.midcell_side_max_bias_mps <= 0.03:
            raise ValueError("midcell_side_max_bias_mps must be 0..0.03")
        if not 0.0 < self.motion_total_lateral_max_mps <= 0.05:
            raise ValueError("motion_total_lateral_max_mps must be 0..0.05")
        if not 0.0 < self.motion_wall_adjacent_speed_cap_mps <= 0.25:
            raise ValueError("wall-adjacent speed cap must be >0 and <=0.25 m/s")
        if not 0.0 < self.motion_cross_track_slow_m < self.motion_cross_track_abort_m < self.cell_size_m / 2.0:
            raise ValueError("cross-track limits must satisfy 0 < slow < abort < half cell size")
        if not 0.0 < self.motion_slow_cross_track_speed_mps <= self.travel_speed_mps:
            raise ValueError("motion_slow_cross_track_speed_mps must not exceed travel speed")
        if self.front_block_confirm_samples < 1:
            raise ValueError("front_block_confirm_samples must be >= 1")
        if not 0.0 < self.blocked_near_target_accept_ratio <= 1.0:
            raise ValueError("blocked_near_target_accept_ratio must be in (0, 1]")
        if self.max_moves <= 0:
            raise ValueError("max_moves must be positive")
        if self.target_camera_resolution not in ("360p", "540p", "720p"):
            raise ValueError("target_camera_resolution must be 360p, 540p, or 720p")
        if not (
            self.target_camera_pitch_min_deg <= self.target_camera_pitch_deg
            <= self.target_camera_pitch_max_deg
        ):
            raise ValueError("Camera observation pitch is outside its allowed range")
        if not -25.0 <= self.target_camera_pitch_min_deg < self.target_camera_pitch_max_deg <= 20.0:
            raise ValueError("Camera pitch limits are invalid")
        if self.target_camera_pitch_timeout_sec <= 0.0 or self.target_camera_settle_sec < 0.0:
            raise ValueError("Camera pitch timing is invalid")
        if self.target_camera_pitch_tolerance_deg <= 0.0 or self.target_preview_fps <= 0.0:
            raise ValueError("Camera pitch tolerance/preview FPS must be positive")
        if self.target_clahe_clip_limit <= 0.0 or self.target_clahe_grid < 2:
            raise ValueError("target CLAHE configuration is invalid")
        if self.target_morph_kernel < 3:
            raise ValueError("target_morph_kernel must be >= 3")
        if not 0.0 <= self.target_roi_top_ratio < self.target_roi_bottom_ratio <= 1.0:
            raise ValueError("target ROI ratios must satisfy 0 <= top < bottom <= 1")
        if self.target_roi_border_margin_px < 0:
            raise ValueError("target ROI border margin must be >= 0")
        if not 0.0 < self.target_square_aspect_min <= 1.0 <= self.target_square_aspect_max:
            raise ValueError("invalid target square aspect range")
        if self.target_merge_centroid_px <= 0.0:
            raise ValueError("target_merge_centroid_px must be positive")
        if not 0.0 <= self.target_min_confidence <= 1.0:
            raise ValueError("target_min_confidence must be between 0 and 1")
        if not 0.0 <= self.target_save_confidence <= 1.0:
            raise ValueError("target_save_confidence must be between 0 and 1")
        if self.target_save_confidence < self.target_min_confidence:
            raise ValueError("target_save_confidence must be >= target_min_confidence")
        if self.target_sample_frames < 1 or self.target_verify_frames < 1:
            raise ValueError("target frame counts must be positive")
        if self.target_verify_frames > self.target_sample_frames:
            raise ValueError("target_verify_frames cannot exceed target_sample_frames")
        if self.target_hold_max_sec <= 0 or not 1 <= self.target_hold_max_windows <= 8:
            raise ValueError("target hold time/windows must be positive and bounded")
        if not 1 <= self.target_pending_min_frames <= self.target_verify_frames:
            raise ValueError("target pending frames must be within verify frame count")
        if self.target_merge_distance_m <= 0.0:
            raise ValueError("target_merge_distance_m must be positive")
        if not 0.0 <= self.closed_maze_perimeter_wall_ratio <= 1.0:
            raise ValueError("closed_maze_perimeter_wall_ratio must be between 0 and 1")
        if self.closed_maze_min_rows < 1 or self.closed_maze_min_cols < 1:
            raise ValueError("closed_maze_min_rows/cols must be >= 1")
        if self.gui_export_width_px < 320 or self.gui_export_height_px < 240:
            raise ValueError("GUI export size is too small")
        if self.vision_resolution not in ("360p", "540p", "720p"):
            raise ValueError("vision_resolution must be 360p, 540p, or 720p")
        if self.vision_blur_kernel < 3 or self.vision_blur_kernel % 2 == 0:
            raise ValueError("vision_blur_kernel must be an odd integer >= 3")
        if not 0.0 <= self.vision_min_confidence <= 1.0:
            raise ValueError("vision_min_confidence must be between 0 and 1")
        if not 0.0 <= self.vision_min_side_vertical_ratio <= 1.0:
            raise ValueError("vision_min_side_vertical_ratio must be between 0 and 1")
        if not 0.0 <= self.vision_max_bottom_gap_ratio <= 1.0:
            raise ValueError("vision_max_bottom_gap_ratio must be between 0 and 1")
        if not 0.0 <= self.vision_center_deadband_ratio < 0.5:
            raise ValueError("vision_center_deadband_ratio must be between 0 and 0.5")
        if not 0.0 <= self.vision_max_candidate_spread_ratio <= 1.0:
            raise ValueError("vision_max_candidate_spread_ratio must be between 0 and 1")

    def gimbal_yaw_for_direction(self, direction: int) -> float:
        return {
            0: self.gimbal_front_yaw_deg,
            1: self.gimbal_right_yaw_deg,
            2: self.gimbal_back_yaw_deg,
            3: self.gimbal_left_yaw_deg,
        }[int(direction) % 4]

    def to_dict(self) -> dict:
        return asdict(self)
