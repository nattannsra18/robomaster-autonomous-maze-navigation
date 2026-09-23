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

    # ToF is mounted on the gimbal. The chassis stays at its initial heading
    # during the whole mission; the gimbal scans/matches the travel direction.
    tof_forward_offset_m: float = 0.12
    tof_max_mapping_cm: float = 300.0
    mapping_min_cm: float = 3.0

    # Gimbal absolute yaw positions relative to the chassis.
    # DJI moveto yaw accepts approximately -250..250 degrees.
    gimbal_front_yaw_deg: float = 0.0
    gimbal_right_yaw_deg: float = 90.0
    gimbal_back_yaw_deg: float = 180.0
    gimbal_left_yaw_deg: float = -90.0
    gimbal_yaw_speed_dps: float = 100.0
    gimbal_settle_sec: float = 0.20

    # Occupancy evidence
    free_delta: int = -2
    occupied_delta: int = 5
    min_score: int = -20
    max_score: int = 20
    occupied_threshold: int = 3
    free_threshold: int = -2

    # ToF-only exploration. A wall in the current cell is typically about
    # 30 cm from chassis centre. The threshold below treats a direction as a
    # candidate cell only when it looks substantially more open than that.
    tof_open_cm: float = 55.0
    scan_samples: int = 5
    scan_sample_interval_sec: float = 0.06
    max_moves: int = 500

    # Conservative mecanum translation. No chassis rotation is used.
    travel_speed_mps: float = 0.12
    stop_front_cm: float = 18.0
    slow_front_cm: float = 35.0
    drive_timeout_sec: float = 0.15
    loop_delay_sec: float = 0.04

    # Keep chassis yaw close to its initial heading while translating.
    heading_hold_enabled: bool = True

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

    def gimbal_yaw_for_direction(self, direction: int) -> float:
        return {
            0: self.gimbal_front_yaw_deg,
            1: self.gimbal_right_yaw_deg,
            2: self.gimbal_back_yaw_deg,
            3: self.gimbal_left_yaw_deg,
        }[int(direction) % 4]

    def to_dict(self) -> dict:
        return asdict(self)
