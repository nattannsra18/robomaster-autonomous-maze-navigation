"""Thread-safe fixed-grid map and runtime wall evidence."""
from __future__ import annotations

import threading
from typing import Dict, Iterable, List, Optional, Set, Tuple

from .configuration import DIR_DELTA


SENSOR_WALL_OPEN_CONFIRMATIONS = 3


class GridMazeMap:
    """Editable prior map plus sensor evidence collected during a run."""

    def __init__(self, rows: int, cols: int):
        self.rows = int(rows)
        self.cols = int(cols)
        self.manual_walls: Set[Tuple[int, int, int]] = set()
        self.sensor_walls: Set[Tuple[int, int, int]] = set()
        self.observed_edges: Set[Tuple[int, int, int]] = set()
        # An edge the chassis physically crossed is stronger evidence than a
        # later oblique side-Sharp snapshot taken near a corner.
        self.traversed_open_edges: Set[Tuple[int, int, int]] = set()
        # A blocked median is accepted immediately. A later open median must be
        # repeated before it is allowed to erase the red wall. This prevents a
        # single corner/offset Sharp reading from destroying confirmed data.
        self._blocked_count: Dict[Tuple[int, int, int], int] = {}
        self._open_count: Dict[Tuple[int, int, int], int] = {}
        self._open_streak: Dict[Tuple[int, int, int], int] = {}
        self.sensor_history: List[dict] = []
        self.start: Optional[Tuple[int, int]] = None
        self.drop: Optional[Tuple[int, int]] = None
        self.exit: Optional[Tuple[int, int]] = None
        self.robot_cell: Optional[Tuple[int, int]] = None
        self.robot_heading = 0
        self.planned_path: List[Tuple[int, int]] = []
        self.travel_path: List[Tuple[int, int]] = []
        self.topology_memory: dict = {}
        self.status = "EDIT MAP"
        self._lock = threading.RLock()

    def in_bounds(self, cell: Tuple[int, int]) -> bool:
        r, c = cell
        return 0 <= r < self.rows and 0 <= c < self.cols

    @staticmethod
    def opposite(direction: int) -> int:
        return (int(direction) + 2) % 4

    def neighbour(self, cell: Tuple[int, int], direction: int) -> Tuple[int, int]:
        dr, dc = DIR_DELTA[int(direction) % 4]
        return cell[0] + dr, cell[1] + dc

    def _both_sides(self, cell: Tuple[int, int], direction: int):
        direction %= 4
        yield cell[0], cell[1], direction
        neighbour = self.neighbour(cell, direction)
        if self.in_bounds(neighbour):
            yield neighbour[0], neighbour[1], self.opposite(direction)

    def set_manual_wall(self, cell: Tuple[int, int], direction: int, blocked: bool) -> None:
        with self._lock:
            changed = False
            for edge in self._both_sides(cell, direction):
                if blocked:
                    changed = changed or edge not in self.manual_walls
                    self.manual_walls.add(edge)
                else:
                    changed = changed or edge in self.manual_walls
                    self.manual_walls.discard(edge)
            if changed:
                # Learned lengths belong to the topology that produced them.
                # A wall edit may split or join corridors, so discard stale
                # associations instead of silently applying them to a new map.
                self.topology_memory = {}

    def toggle_manual_wall(self, cell: Tuple[int, int], direction: int) -> None:
        blocked = (cell[0], cell[1], direction % 4) not in self.manual_walls
        self.set_manual_wall(cell, direction, blocked)

    def observe_edge(
        self,
        cell: Tuple[int, int],
        direction: int,
        blocked: bool,
        force: bool = False,
    ) -> None:
        with self._lock:
            mirrored = list(self._both_sides(cell, direction))
            if blocked and not force and any(
                edge in self.traversed_open_edges for edge in mirrored
            ):
                # Do not close a corridor that the robot has already crossed
                # because of one side-facing reading at a nearby corner.
                return
            for edge in mirrored:
                self.observed_edges.add(edge)
                if blocked:
                    self._blocked_count[edge] = self._blocked_count.get(edge, 0) + 1
                    self._open_streak[edge] = 0
                    self.sensor_walls.add(edge)
                    if force:
                        self.traversed_open_edges.discard(edge)
                else:
                    self._open_count[edge] = self._open_count.get(edge, 0) + 1
                    self._open_streak[edge] = self._open_streak.get(edge, 0) + 1
                    if self._open_streak[edge] >= SENSOR_WALL_OPEN_CONFIRMATIONS:
                        self.sensor_walls.discard(edge)

    def confirm_traversed_open_edge(
        self,
        cell: Tuple[int, int],
        direction: int,
    ) -> None:
        with self._lock:
            for edge in self._both_sides(cell, direction):
                self.traversed_open_edges.add(edge)
                self.observed_edges.add(edge)
                self.sensor_walls.discard(edge)
                self._open_count[edge] = self._open_count.get(edge, 0) + 1
                self._open_streak[edge] = SENSOR_WALL_OPEN_CONFIRMATIONS

    def is_traversed_open(self, cell: Tuple[int, int], direction: int) -> bool:
        with self._lock:
            return (
                cell[0], cell[1], direction % 4
            ) in self.traversed_open_edges

    def clear_observed_edge(self, cell: Tuple[int, int], direction: int) -> None:
        """Remove old sensor evidence when the operator corrects one edge."""
        with self._lock:
            for edge in self._both_sides(cell, direction):
                self.observed_edges.discard(edge)
                self.sensor_walls.discard(edge)
                self.traversed_open_edges.discard(edge)
                self._blocked_count.pop(edge, None)
                self._open_count.pop(edge, None)
                self._open_streak.pop(edge, None)

    def clear_sensor_map(self) -> None:
        with self._lock:
            self.observed_edges.clear()
            self.sensor_walls.clear()
            self.traversed_open_edges.clear()
            self._blocked_count.clear()
            self._open_count.clear()
            self._open_streak.clear()
            self.sensor_history.clear()

    def reset_run_data(self) -> None:
        """Clear evidence and paths before a new real or simulated mission."""
        with self._lock:
            self.clear_sensor_map()
            self.travel_path.clear()
            self.planned_path.clear()
            self.robot_cell = self.start

    def record_sensor_snapshot(
        self,
        cell: Tuple[int, int],
        heading: int,
        front_cm: Optional[float],
        left_cm: Optional[float],
        right_cm: Optional[float],
    ) -> None:
        """Record the filtered values actually used for a cell-map update."""
        with self._lock:
            self.sensor_history.append(
                {
                    "sequence": len(self.sensor_history),
                    "cell": [int(cell[0]), int(cell[1])],
                    "heading": int(heading) % 4,
                    "front_cm": None if front_cm is None else round(float(front_cm), 3),
                    "left_cm": None if left_cm is None else round(float(left_cm), 3),
                    "right_cm": None if right_cm is None else round(float(right_cm), 3),
                }
            )

    def has_wall(
        self,
        cell: Tuple[int, int],
        direction: int,
        sensor_overrides: bool = True,
    ) -> bool:
        edge = (cell[0], cell[1], direction % 4)
        neighbour = self.neighbour(cell, direction)
        if not self.in_bounds(neighbour):
            return True
        with self._lock:
            # In the fixed-maze mode, a wall explicitly drawn by the operator
            # is authoritative.  A single oblique Sharp reading must never
            # open that wall and send A* through it.  Live sensors may still
            # add an unexpected safety wall on an edge drawn as open.
            if edge in self.manual_walls:
                return True
            if sensor_overrides and edge in self.observed_edges:
                return edge in self.sensor_walls
            return edge in self.sensor_walls

    def set_marker(self, marker: str, cell: Tuple[int, int]) -> None:
        if not self.in_bounds(cell):
            raise ValueError(f"Cell {cell} is outside the map")
        with self._lock:
            if marker == "start":
                self.start = cell
            elif marker == "drop":
                self.drop = cell
            elif marker == "exit":
                self.exit = cell
            else:
                raise ValueError(f"Unknown marker {marker}")

    def resize(self, rows: int, cols: int) -> None:
        with self._lock:
            self.rows, self.cols = int(rows), int(cols)
            self.manual_walls = {
                edge for edge in self.manual_walls
                if 0 <= edge[0] < self.rows and 0 <= edge[1] < self.cols
            }
            self.sensor_walls.clear()
            self.observed_edges.clear()
            self.traversed_open_edges.clear()
            self._blocked_count.clear()
            self._open_count.clear()
            self._open_streak.clear()
            self.sensor_history.clear()
            for name in ("start", "drop", "exit"):
                cell = getattr(self, name)
                if cell is not None and not self.in_bounds(cell):
                    setattr(self, name, None)
            self.planned_path.clear()
            self.travel_path.clear()
            self.topology_memory = {}

    def record_pose(self, cell: Tuple[int, int], heading: int) -> None:
        with self._lock:
            self.robot_cell = cell
            self.robot_heading = heading % 4
            if not self.travel_path or self.travel_path[-1] != cell:
                self.travel_path.append(cell)

    @staticmethod
    def _unique_edges(edges: Iterable[Tuple[int, int, int]]) -> List[List[int]]:
        unique = []
        seen = set()
        for r, c, direction in edges:
            dr, dc = DIR_DELTA[direction]
            other = (r + dr, c + dc)
            key = tuple(sorted(((r, c), other)))
            if key in seen:
                continue
            seen.add(key)
            unique.append([r, c, direction])
        return sorted(unique)

    def _edge_evidence_rows(self) -> List[dict]:
        rows = []
        seen = set()
        all_edges = set(self._blocked_count) | set(self._open_count)
        for edge in sorted(all_edges):
            r, c, direction = edge
            dr, dc = DIR_DELTA[direction]
            key = tuple(sorted(((r, c), (r + dr, c + dc))))
            if key in seen:
                continue
            seen.add(key)
            mirrored = list(self._both_sides((r, c), direction))
            rows.append(
                {
                    "edge": [r, c, direction],
                    "blocked_count": max(
                        (self._blocked_count.get(item, 0) for item in mirrored),
                        default=0,
                    ),
                    "open_count": max(
                        (self._open_count.get(item, 0) for item in mirrored),
                        default=0,
                    ),
                    "open_streak": max(
                        (self._open_streak.get(item, 0) for item in mirrored),
                        default=0,
                    ),
                    "state": "WALL" if edge in self.sensor_walls else "OPEN",
                }
            )
        return rows

    def to_dict(self) -> dict:
        with self._lock:
            return {
                "rows": self.rows,
                "cols": self.cols,
                "manual_walls": self._unique_edges(self.manual_walls),
                "sensor_walls": self._unique_edges(self.sensor_walls),
                "observed_edges": self._unique_edges(self.observed_edges),
                "traversed_open_edges": self._unique_edges(self.traversed_open_edges),
                "edge_evidence": self._edge_evidence_rows(),
                "sensor_history": list(self.sensor_history),
                "start": list(self.start) if self.start is not None else None,
                "drop": list(self.drop) if self.drop is not None else None,
                "exit": list(self.exit) if self.exit is not None else None,
                "robot_cell": list(self.robot_cell) if self.robot_cell is not None else None,
                "robot_heading": self.robot_heading,
                "planned_path": [list(cell) for cell in self.planned_path],
                "travel_path": [list(cell) for cell in self.travel_path],
                "topology_memory": self.topology_memory,
                "status": self.status,
            }

    @classmethod
    def from_dict(cls, data: dict) -> "GridMazeMap":
        result = cls(int(data["rows"]), int(data["cols"]))
        for edge in data.get("manual_walls", []):
            result.set_manual_wall((int(edge[0]), int(edge[1])), int(edge[2]), True)
        for edge in data.get("sensor_walls", []):
            result.observe_edge((int(edge[0]), int(edge[1])), int(edge[2]), True)
        for edge in data.get("observed_edges", []):
            for mirrored in result._both_sides((int(edge[0]), int(edge[1])), int(edge[2])):
                result.observed_edges.add(mirrored)
        for edge in data.get("traversed_open_edges", []):
            cell = (int(edge[0]), int(edge[1]))
            result.confirm_traversed_open_edge(cell, int(edge[2]))
        for row in data.get("edge_evidence", []):
            edge_data = row.get("edge", [])
            if len(edge_data) != 3:
                continue
            cell = (int(edge_data[0]), int(edge_data[1]))
            direction = int(edge_data[2])
            for edge in result._both_sides(cell, direction):
                result._blocked_count[edge] = int(row.get("blocked_count", 0))
                result._open_count[edge] = int(row.get("open_count", 0))
                result._open_streak[edge] = int(row.get("open_streak", 0))
        result.sensor_history = [
            dict(row) for row in data.get("sensor_history", []) if isinstance(row, dict)
        ]
        for marker in ("start", "drop", "exit"):
            value = data.get(marker)
            if value is not None:
                setattr(result, marker, (int(value[0]), int(value[1])))
        result.topology_memory = dict(data.get("topology_memory", {}))
        return result
