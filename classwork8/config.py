from dataclasses import dataclass, asdict


@dataclass
class Classwork8Config:
    """Runtime configuration for Classwork 8 unknown-world exploration.

    The sensor IDs/ports below match the physical wiring described for the
    current RoboMaster:
      - right CAN sensor hub: ID 1
      - left CAN sensor hub: ID 2
      - digital IR on port 1 of each hub
      - Sharp analog distance sensor on port 2 of each hub
      - ToF mounted on the gimbal and kept facing chassis-forward
    """

    connection: str = "ap"

    # Sensor hubs / ports
    right_hub_id: int = 1
    left_hub_id: int = 2
    ir_port: int = 1
    sharp_port: int = 2
    ir_blocked_level: int = 0
    ir_confirm_samples: int = 2

    # Mapping canvas. This is only an initially empty working area; it is NOT
    # prior maze knowledge. Start is placed at the centre.
    map_width_m: float = 8.0
    map_height_m: float = 8.0
    resolution_m: float = 0.05

    # Sensor mounting offsets from chassis centre. Measure lens-to-centre and
    # update these before the final accuracy run. Defaults are conservative
    # approximations for first integration tests.
    tof_forward_offset_m: float = 0.12
    sharp_lateral_offset_m: float = 0.16

    # Usable mapping range. A reading near/above max range marks free space but
    # does not create an occupied endpoint.
    tof_max_mapping_cm: float = 200.0
    sharp_max_mapping_cm: float = 50.0
    mapping_min_cm: float = 3.0

    # Occupancy evidence scores
    free_delta: int = -2
    occupied_delta: int = 5
    min_score: int = -20
    max_score: int = 20
    occupied_threshold: int = 3
    free_threshold: int = -2

    # Motion/safety values, mostly inherited from the working legacy explorer.
    forward_speed_mps: float = 0.20
    drive_timeout_sec: float = 0.15
    loop_delay_sec: float = 0.05
    stop_front_cm: float = 15.0
    side_warning_cm: float = 11.0
    ir_escape_y_mps: float = 0.15

    # Avoid writing rays while the robot is substantially between cardinal
    # headings. The legacy controller performs closed-loop 90-degree turns.
    mapping_heading_tolerance_deg: float = 8.0

    stop_on_open_exit: bool = False
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
        if self.sharp_max_mapping_cm <= self.mapping_min_cm:
            raise ValueError("Sharp mapping range is invalid")

    def to_dict(self) -> dict:
        return asdict(self)
