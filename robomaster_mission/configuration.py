"""Typed mission configuration and grid direction helpers."""
from dataclasses import dataclass
from typing import Any, Dict

HEADINGS = ("N", "E", "S", "W")

# ============================================================
# Hybrid known-map layer: editable grid + A* + run recording
# ============================================================

DIR_DELTA = {
    0: (-1, 0),  # N: canvas up
    1: (0, 1),   # E: canvas right
    2: (1, 0),   # S: canvas down
    3: (0, -1),  # W: canvas left
}
DIR_FROM_NAME = {name: index for index, name in enumerate(HEADINGS)}


@dataclass
class HybridConfig:
    """Values edited in the GUI before the robot is connected."""

    rows: int = 6
    cols: int = 8
    cell_size_cm: float = 60.0
    connection: str = "ap"
    simulation: bool = False
    start_heading: str = "N"
    exit_heading: str = "N"
    sensor_overrides_drawing: bool = True

    forward_speed_mps: float = 0.20
    minimum_speed_mps: float = 0.05
    lateral_max_mps: float = 0.05
    hard_stop_front_cm: float = 15.0
    front_wall_cm: float = 45.0
    side_wall_cm: float = 28.0
    side_topology_open_cm: float = 18.0
    # Optional JSON overrides. Empty values use the measured built-in table.
    sharp_left_calibration_file: str = ""
    sharp_right_calibration_file: str = ""
    cell_tolerance_cm: float = 3.0
    end_wall_arrival_min_ratio: float = 0.70
    drive_timeout_sec: float = 0.18
    exit_drive_cm: float = 65.0
    turn_cost: float = 0.18
    control_period_sec: float = 0.05
    max_replans: int = 80
    explore_max_steps: int = 300
    corridor_min_progress_cm: float = 18.0
    corridor_min_length_ratio: float = 0.30
    corridor_max_length_ratio: float = 1.80
    corridor_timeout_sec: float = 30.0
    junction_signature_score: float = 0.60
    learned_length_alpha: float = 0.35
    fallback_edge_limit: int = 2
    localization_guard_cm: float = 10.0

    pickup_enabled: bool = True
    pickup_target_cm: float = 8.0
    pickup_tolerance_cm: float = 0.7
    object_detect_max_cm: float = 100.0
    pickup_fast_speed_mps: float = 0.08
    pickup_slow_speed_mps: float = 0.04
    pickup_crawl_speed_mps: float = 0.02
    pickup_timeout_sec: float = 25.0
    pickup_attempts: int = 5
    tof_lost_abort_sec: float = 2.0
    lift_clear_delta_cm: float = 8.0
    verify_window_sec: float = 0.8
    gripper_power: int = 50
    gripper_open_sec: float = 1.5
    gripper_close_sec: float = 1.5
    arm_action_timeout_sec: float = 8.0
    arm_settle_sec: float = 0.35
    carry_x: int = 0
    carry_y: int = 100
    pickup_x: int = 180
    pickup_y: int = -50
    drop_x: int = 180
    drop_y: int = -50
    retry_backoff_cm: float = 5.0

    # DROP geometry. The competition target is measured from the released
    # object, while the controller reads wall distance from ToF/Sharp. Positive
    # front offset means the object is ahead of the ToF (closer to front wall).
    # Positive side offset means the side Sharp is closer to its wall than the
    # object's centreline. validate() converts these four operator-friendly
    # measurements into the two sensor targets used by the controller.
    drop_heading: str = "W"
    drop_side: str = "RIGHT"
    drop_object_front_wall_cm: float = 40.0
    drop_object_side_wall_cm: float = 40.0
    drop_tof_to_object_forward_cm: float = 0.0
    drop_sharp_to_object_lateral_cm: float = 12.0
    drop_front_sensor_target_cm: float = 40.0
    drop_side_sensor_target_cm: float = 28.0
    drop_distance_tolerance_cm: float = 5.0
    drop_align_timeout_sec: float = 20.0
    drop_align_max_travel_cm: float = 60.0
    drop_align_max_speed_mps: float = 0.07
    drop_align_kp_mps_per_cm: float = 0.006
    drop_stable_samples: int = 6

    output_prefix: str = "robomaster_basic_maze_run"

    def update_drop_sensor_targets(self) -> None:
        """Translate object-to-wall requirements into sensor readings."""
        self.drop_front_sensor_target_cm = (
            float(self.drop_object_front_wall_cm)
            + float(self.drop_tof_to_object_forward_cm)
        )
        self.drop_side_sensor_target_cm = (
            float(self.drop_object_side_wall_cm)
            - float(self.drop_sharp_to_object_lateral_cm)
        )

    def validate(self) -> None:
        self.update_drop_sensor_targets()
        if not 1 <= int(self.rows) <= 30 or not 1 <= int(self.cols) <= 30:
            raise ValueError("Rows and columns must be between 1 and 30")
        if not 20.0 <= float(self.cell_size_cm) <= 200.0:
            raise ValueError("Cell size must be between 20 and 200 cm")
        if self.start_heading not in HEADINGS or self.exit_heading not in HEADINGS:
            raise ValueError("Heading must be N, E, S, or W")
        if self.drop_heading not in HEADINGS:
            raise ValueError("Drop heading must be N, E, S, or W")
        if self.drop_side not in ("LEFT", "RIGHT"):
            raise ValueError("Drop side must be LEFT or RIGHT")
        if str(self.connection).strip().lower() not in ("ap", "sta", "rndis"):
            raise ValueError("Connection must be ap, sta, or rndis")
        if not 0.01 <= self.forward_speed_mps <= 1.0:
            raise ValueError("Forward speed must be 0.01-1.00 m/s")
        if self.hard_stop_front_cm >= self.front_wall_cm:
            raise ValueError("Front-wall threshold must be greater than hard-stop distance")
        if not 0.50 <= self.end_wall_arrival_min_ratio <= 0.95:
            raise ValueError("End-wall arrival ratio must be 0.50-0.95")
        if not 5.0 <= self.side_topology_open_cm <= 80.0:
            raise ValueError("Side topology open threshold must be 5-80 cm")
        if self.side_topology_open_cm >= self.side_wall_cm:
            raise ValueError(
                "Side topology open threshold must be lower than side-wall control threshold"
            )
        if self.pickup_target_cm <= 0.0:
            raise ValueError("Pickup target must be greater than zero")
        if not 10.0 <= self.drop_object_front_wall_cm <= 150.0:
            raise ValueError("Object front-wall target must be 10-150 cm")
        if not 10.0 <= self.drop_object_side_wall_cm <= 150.0:
            raise ValueError("Object side-wall target must be 10-150 cm")
        if not 0.0 <= self.drop_tof_to_object_forward_cm <= 60.0:
            raise ValueError("ToF-to-object forward offset must be 0-60 cm")
        if not 0.0 <= self.drop_sharp_to_object_lateral_cm <= 60.0:
            raise ValueError("Sharp-to-object lateral offset must be 0-60 cm")
        if not 10.0 <= self.drop_front_sensor_target_cm <= 150.0:
            raise ValueError("Drop front sensor target must be 10-150 cm")
        if not 5.0 <= self.drop_side_sensor_target_cm <= 80.0:
            raise ValueError("Drop side sensor target must be 5-80 cm")
        if not 0.5 <= self.drop_distance_tolerance_cm <= 20.0:
            raise ValueError("Drop alignment tolerance must be 0.5-20 cm")
        if self.drop_align_timeout_sec <= 0.0:
            raise ValueError("Drop alignment timeout must be greater than zero")
        if not 10.0 <= self.drop_align_max_travel_cm <= 200.0:
            raise ValueError("Drop alignment max travel must be 10-200 cm")
        if not 0.01 <= self.drop_align_max_speed_mps <= 0.20:
            raise ValueError("Drop alignment max speed must be 0.01-0.20 m/s")
        if int(self.drop_stable_samples) < 2:
            raise ValueError("Drop stable samples must be at least 2")
        if not 0.0 < self.corridor_min_length_ratio < 1.0:
            raise ValueError("Corridor minimum length ratio must be between 0 and 1")
        if self.corridor_max_length_ratio <= 1.0:
            raise ValueError("Corridor maximum length ratio must be greater than 1")
        if not 0.0 <= self.junction_signature_score <= 1.0:
            raise ValueError("Junction signature score must be between 0 and 1")
        if self.corridor_min_progress_cm < 0.0:
            raise ValueError("Corridor minimum progress cannot be negative")
        if self.corridor_timeout_sec <= 0.0:
            raise ValueError("Corridor timeout must be greater than zero")
        if not 0.0 < self.learned_length_alpha <= 1.0:
            raise ValueError("Learned length alpha must be in (0, 1]")
        if int(self.fallback_edge_limit) < 1:
            raise ValueError("Tremaux edge limit must be at least 1")
        if not 2.0 <= self.localization_guard_cm <= 100.0:
            raise ValueError("Localization guard must be 2-100 cm")

    def to_dict(self) -> dict:
        return dict(self.__dict__)

    @classmethod
    def from_dict(cls, data: dict) -> "HybridConfig":
        allowed = cls.__dataclass_fields__.keys()
        values = {key: value for key, value in data.items() if key in allowed}
        # Preserve the effective sensor targets of setup files created before
        # the object/offset GUI existed. New files store all fields explicitly.
        if "drop_object_front_wall_cm" not in data:
            old_front = float(data.get("drop_front_sensor_target_cm", 40.0))
            front_offset = float(values.get("drop_tof_to_object_forward_cm", 0.0))
            values["drop_object_front_wall_cm"] = old_front - front_offset
        if "drop_object_side_wall_cm" not in data:
            old_side = float(data.get("drop_side_sensor_target_cm", 28.0))
            side_offset = float(values.get("drop_sharp_to_object_lateral_cm", 12.0))
            values["drop_object_side_wall_cm"] = old_side + side_offset
        result = cls(**values)
        result.update_drop_sensor_targets()
        return result
