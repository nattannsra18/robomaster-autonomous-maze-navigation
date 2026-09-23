from dataclasses import dataclass, asdict


@dataclass
class Classwork8Config:
    """Runtime configuration for ToF-only unknown-world exploration."""

    connection: str = "ap"

    # Empty working canvas only; no maze layout is preloaded.
    map_width_m: float = 8.0
    map_height_m: float = 8.0
    resolution_m: float = 0.05

    # ToF is mounted on the gimbal. The robot runs CHASSIS_LEAD and keeps the
    # gimbal recentered, so ToF always points chassis-forward.
    tof_forward_offset_m: float = 0.12
    tof_max_mapping_cm: float = 300.0
    mapping_min_cm: float = 3.0

    # Occupancy evidence
    free_delta: int = -2
    occupied_delta: int = 5
    min_score: int = -20
    max_score: int = 20
    occupied_threshold: int = 3
    free_threshold: int = -2

    # ToF-only exploration. The robot stops every step, scans four cardinal
    # directions by rotating the chassis, then uses DFS/backtracking.
    exploration_step_m: float = 0.25
    tof_open_cm: float = 50.0
    scan_samples: int = 5
    scan_sample_interval_sec: float = 0.06
    scan_settle_sec: float = 0.18
    max_moves: int = 500

    # Conservative motion because there are no side/corner sensors.
    forward_speed_mps: float = 0.12
    stop_front_cm: float = 18.0
    slow_front_cm: float = 35.0
    drive_timeout_sec: float = 0.15
    loop_delay_sec: float = 0.04

    # Closed-loop cardinal heading control.
    turn_tolerance_deg: float = 2.0
    turn_stable_samples: int = 3
    turn_timeout_sec: float = 5.0

    output_dir: str = "classwork8_output"

    def validate(self) -> None:
        if self.connection not in ("ap", "sta", "rndis"):
            raise ValueError("connection must be ap, sta, or rndis")
        if self.resolution_m <= 0.0:
            raise ValueError("resolution_m must be positive")
        if self.map_width_m <= self.resolution_m or self.map_height_m <= self.resolution_m:
            raise ValueError("map dimensions must be larger than one cell")
        if self.tof_max_mapping_cm <= self.mapping_min_cm:
            raise ValueError("ToF mapping range is invalid")
        if self.exploration_step_m <= 0.0:
            raise ValueError("exploration_step_m must be positive")
        if self.tof_open_cm <= self.stop_front_cm:
            raise ValueError("tof_open_cm must be greater than stop_front_cm")
        if self.forward_speed_mps <= 0.0:
            raise ValueError("forward_speed_mps must be positive")
        if self.max_moves <= 0:
            raise ValueError("max_moves must be positive")

    def to_dict(self) -> dict:
        return asdict(self)
