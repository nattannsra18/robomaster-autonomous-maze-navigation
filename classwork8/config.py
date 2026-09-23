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
    cross_track_kp: float = 0.60
    cross_track_max_mps: float = 0.04

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
    gimbal_yaw_speed_dps: float = 90.0
    gimbal_min_yaw_speed_dps: float = 20.0
    gimbal_yaw_kp: float = 1.6
    gimbal_tolerance_deg: float = 2.0
    gimbal_stable_samples: int = 3
    gimbal_turn_timeout_sec: float = 7.0
    gimbal_settle_sec: float = 0.20

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
    scan_samples: int = 5
    scan_sample_interval_sec: float = 0.06
    max_moves: int = 500

    # Conservative mecanum translation. No 90-degree chassis scan turns.
    travel_speed_mps: float = 0.12
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

    # GUI
    gui_refresh_ms: int = 150
    gui_canvas_px: int = 720

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
        if self.tof_max_mapping_cm <= self.mapping_min_cm:
            raise ValueError("ToF mapping range is invalid")
        if self.tof_open_cm <= self.stop_front_cm:
            raise ValueError("tof_open_cm must be greater than stop_front_cm")
        if self.travel_speed_mps <= 0.0:
            raise ValueError("travel_speed_mps must be positive")
        if self.max_moves <= 0:
            raise ValueError("max_moves must be positive")
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
