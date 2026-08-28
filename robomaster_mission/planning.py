"""Orientation-aware A* and topological graph planning."""
from __future__ import annotations

import heapq
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Set, Tuple

from .configuration import DIR_DELTA, HybridConfig
from .grid_map import GridMazeMap

def turn_distance(from_heading: int, to_heading: int) -> int:
    difference = (to_heading - from_heading) % 4
    return min(difference, 4 - difference)


def relative_turn(from_heading: int, to_heading: int) -> str:
    difference = (to_heading - from_heading) % 4
    return {0: "FRONT", 1: "RIGHT", 2: "BACK", 3: "LEFT"}[difference]


def direction_between(a: Tuple[int, int], b: Tuple[int, int]) -> int:
    delta = b[0] - a[0], b[1] - a[1]
    for direction, expected in DIR_DELTA.items():
        if delta == expected:
            return direction
    raise ValueError(f"Cells {a} and {b} are not adjacent")


@dataclass
class TopologyNode:
    node_id: str
    cell: Tuple[int, int]
    edge_ids: List[str] = field(default_factory=list)


@dataclass
class TopologyEdge:
    edge_id: str
    node_a: str
    node_b: str
    path_a_to_b: List[Tuple[int, int]]
    direction_from_a: int
    direction_from_b: int
    estimated_length_m: float
    learned_length_m: Optional[float] = None
    state: str = "UNSEEN"
    attempts: int = 0
    traversals: int = 0
    blocked_reason: Optional[str] = None

    def other(self, node_id: str) -> str:
        if node_id == self.node_a:
            return self.node_b
        if node_id == self.node_b:
            return self.node_a
        raise ValueError(f"{node_id} is not connected to {self.edge_id}")

    def direction_from(self, node_id: str) -> int:
        if node_id == self.node_a:
            return self.direction_from_a
        if node_id == self.node_b:
            return self.direction_from_b
        raise ValueError(f"{node_id} is not connected to {self.edge_id}")

    def arrival_heading_from(self, node_id: str) -> int:
        target = self.other(node_id)
        return (self.direction_from(target) + 2) % 4

    def path_from(self, node_id: str) -> List[Tuple[int, int]]:
        if node_id == self.node_a:
            return list(self.path_a_to_b)
        if node_id == self.node_b:
            return list(reversed(self.path_a_to_b))
        raise ValueError(f"{node_id} is not connected to {self.edge_id}")

    @property
    def planning_length_m(self) -> float:
        return self.learned_length_m or self.estimated_length_m


@dataclass(frozen=True)
class TopologyRouteStep:
    edge_id: str
    from_node: str
    to_node: str
    departure_direction: int
    arrival_heading: int


class TopologicalMazeGraph:
    """Compress the drawn grid into detectable junction/corner/dead-end nodes.

    Straight degree-2 cells disappear into edges.  The physical robot therefore
    does not need to travel one exact cell length at a time; it follows a
    corridor until the expected node pattern is observed.
    """

    def __init__(self, maze: GridMazeMap, cell_size_cm: float):
        self.maze = maze
        self.cell_size_m = float(cell_size_cm) / 100.0
        self.nodes: Dict[str, TopologyNode] = {}
        self.edges: Dict[str, TopologyEdge] = {}
        self.node_by_cell: Dict[Tuple[int, int], str] = {}
        self.marker_nodes: Dict[str, str] = {}
        self._compile()
        self._restore_learned_lengths()

    def open_directions(self, cell: Tuple[int, int]) -> Set[int]:
        result = set()
        for direction in range(4):
            neighbour = self.maze.neighbour(cell, direction)
            if not self.maze.in_bounds(neighbour):
                continue
            if (cell[0], cell[1], direction) in self.maze.manual_walls:
                continue
            result.add(direction)
        return result

    @staticmethod
    def _is_decision_cell(open_dirs: Set[int]) -> bool:
        if len(open_dirs) != 2:
            return True
        first, second = sorted(open_dirs)
        return (first + 2) % 4 != second

    @staticmethod
    def _step_key(a: Tuple[int, int], b: Tuple[int, int]):
        return tuple(sorted((a, b)))

    @staticmethod
    def _path_key(path: List[Tuple[int, int]]) -> str:
        forward = ";".join(f"{r},{c}" for r, c in path)
        reverse = ";".join(f"{r},{c}" for r, c in reversed(path))
        return min(forward, reverse)

    def _compile(self) -> None:
        forced = {cell for cell in (self.maze.start, self.maze.drop, self.maze.exit) if cell is not None}
        node_cells = set(forced)
        for r in range(self.maze.rows):
            for c in range(self.maze.cols):
                cell = (r, c)
                if self._is_decision_cell(self.open_directions(cell)):
                    node_cells.add(cell)

        for index, cell in enumerate(sorted(node_cells)):
            node_id = f"N{index}"
            self.nodes[node_id] = TopologyNode(node_id, cell)
            self.node_by_cell[cell] = node_id

        used_steps = set()
        edge_index = 0
        for start_cell in sorted(node_cells):
            start_id = self.node_by_cell[start_cell]
            for initial_direction in sorted(self.open_directions(start_cell)):
                first = self.maze.neighbour(start_cell, initial_direction)
                if self._step_key(start_cell, first) in used_steps:
                    continue
                path = [start_cell]
                previous = start_cell
                current = first
                direction = initial_direction
                safety = self.maze.rows * self.maze.cols * 4 + 4
                while safety > 0:
                    safety -= 1
                    path.append(current)
                    used_steps.add(self._step_key(previous, current))
                    if current in node_cells:
                        break
                    candidates = self.open_directions(current) - {(direction + 2) % 4}
                    if len(candidates) != 1:
                        raise ValueError(
                            f"Topology compilation failed at {current}: exits={sorted(candidates)}"
                        )
                    next_direction = next(iter(candidates))
                    previous, current = current, self.maze.neighbour(current, next_direction)
                    direction = next_direction
                else:
                    raise ValueError("Topology contains an unbounded corridor loop")

                if current not in node_cells:
                    raise ValueError(f"Corridor from {start_cell} did not reach a node")
                target_id = self.node_by_cell[current]
                edge_id = f"E{edge_index}"
                edge_index += 1
                direction_from_a = direction_between(path[0], path[1])
                direction_from_b = direction_between(path[-1], path[-2])
                edge = TopologyEdge(
                    edge_id=edge_id,
                    node_a=start_id,
                    node_b=target_id,
                    path_a_to_b=path,
                    direction_from_a=direction_from_a,
                    direction_from_b=direction_from_b,
                    estimated_length_m=max(0.05, (len(path) - 1) * self.cell_size_m),
                )
                self.edges[edge_id] = edge
                self.nodes[start_id].edge_ids.append(edge_id)
                self.nodes[target_id].edge_ids.append(edge_id)

        for marker in ("start", "drop", "exit"):
            cell = getattr(self.maze, marker)
            if cell is not None:
                self.marker_nodes[marker] = self.node_by_cell[cell]

    def _restore_learned_lengths(self) -> None:
        saved = self.maze.topology_memory.get("edges", {})
        by_path = {
            self._path_key(edge.path_a_to_b): edge
            for edge in self.edges.values()
        }
        for saved_edge in saved.values() if isinstance(saved, dict) else []:
            try:
                path = [(int(cell[0]), int(cell[1])) for cell in saved_edge["path"]]
                learned = saved_edge.get("learned_length_m")
                if learned is not None and self._path_key(path) in by_path:
                    by_path[self._path_key(path)].learned_length_m = float(learned)
            except (KeyError, TypeError, ValueError):
                continue

    def node_open_directions(self, node_id: str) -> Set[int]:
        result = set()
        node = self.nodes[node_id]
        for edge_id in node.edge_ids:
            edge = self.edges[edge_id]
            if edge.state != "BLOCKED":
                result.add(edge.direction_from(node_id))
        return result

    def route(
        self,
        start_node: str,
        start_heading: int,
        goal_node: str,
        turn_cost: float,
    ) -> Optional[List[TopologyRouteStep]]:
        if start_node == goal_node:
            return []
        start_state = (start_node, start_heading % 4)
        frontier = [(0.0, start_state)]
        best = {start_state: 0.0}
        parent: Dict[Tuple[str, int], Tuple[Tuple[str, int], str]] = {}
        final_state = None
        while frontier:
            cost, state = heapq.heappop(frontier)
            if cost > best.get(state, float("inf")) + 1e-9:
                continue
            node_id, heading = state
            if node_id == goal_node:
                final_state = state
                break
            for edge_id in self.nodes[node_id].edge_ids:
                edge = self.edges[edge_id]
                if edge.state == "BLOCKED":
                    continue
                direction = edge.direction_from(node_id)
                target = edge.other(node_id)
                arrival = edge.arrival_heading_from(node_id)
                new_cost = (
                    cost
                    + edge.planning_length_m
                    + turn_cost * turn_distance(heading, direction)
                )
                next_state = (target, arrival)
                if new_cost + 1e-9 >= best.get(next_state, float("inf")):
                    continue
                best[next_state] = new_cost
                parent[next_state] = (state, edge_id)
                heapq.heappush(frontier, (new_cost, next_state))
        if final_state is None:
            return None

        reversed_steps = []
        state = final_state
        while state != start_state:
            previous, edge_id = parent[state]
            edge = self.edges[edge_id]
            reversed_steps.append(
                TopologyRouteStep(
                    edge_id=edge_id,
                    from_node=previous[0],
                    to_node=state[0],
                    departure_direction=edge.direction_from(previous[0]),
                    arrival_heading=edge.arrival_heading_from(previous[0]),
                )
            )
            state = previous
        return list(reversed(reversed_steps))

    def expanded_cells(self, route: List[TopologyRouteStep]) -> List[Tuple[int, int]]:
        if not route:
            return []
        cells = []
        for step in route:
            path = self.edges[step.edge_id].path_from(step.from_node)
            if cells:
                path = path[1:]
            cells.extend(path)
        return cells

    def confirm_edge(self, edge_id: str, measured_length_m: float, alpha: float) -> None:
        edge = self.edges[edge_id]
        edge.state = "CONFIRMED"
        edge.traversals += 1
        edge.blocked_reason = None
        if measured_length_m > 0.02:
            if edge.learned_length_m is None:
                edge.learned_length_m = measured_length_m
            else:
                edge.learned_length_m = (
                    (1.0 - alpha) * edge.learned_length_m
                    + alpha * measured_length_m
                )
        self.maze.topology_memory = self.to_dict()

    def block_edge(self, edge_id: str, reason: str) -> None:
        edge = self.edges[edge_id]
        edge.state = "BLOCKED"
        edge.blocked_reason = reason
        self.maze.topology_memory = self.to_dict()

    def to_dict(self) -> dict:
        return {
            "version": 2,
            "nodes": {
                node_id: {
                    "cell": list(node.cell),
                    "open_directions": sorted(self.node_open_directions(node_id)),
                }
                for node_id, node in self.nodes.items()
            },
            "edges": {
                edge_id: {
                    "node_a": edge.node_a,
                    "node_b": edge.node_b,
                    "path": [list(cell) for cell in edge.path_a_to_b],
                    "estimated_length_m": edge.estimated_length_m,
                    "learned_length_m": edge.learned_length_m,
                    "state": edge.state,
                    "attempts": edge.attempts,
                    "traversals": edge.traversals,
                    "blocked_reason": edge.blocked_reason,
                }
                for edge_id, edge in self.edges.items()
            },
        }


def astar_oriented(
    maze: GridMazeMap,
    start: Tuple[int, int],
    start_heading: int,
    goal: Tuple[int, int],
    turn_cost: float = 0.18,
    sensor_overrides: bool = True,
) -> Optional[List[Tuple[int, int]]]:
    """Shortest cell route; heading is included so excessive turns cost more."""
    if start == goal:
        return [start]
    if not maze.in_bounds(start) or not maze.in_bounds(goal):
        return None

    start_state = (start[0], start[1], start_heading % 4)
    frontier = [(0.0, 0.0, start_state)]
    best = {start_state: 0.0}
    parent: Dict[Tuple[int, int, int], Tuple[int, int, int]] = {}
    final_state = None

    while frontier:
        _, cost, state = heapq.heappop(frontier)
        if cost > best.get(state, float("inf")) + 1e-9:
            continue
        r, c, heading = state
        if (r, c) == goal:
            final_state = state
            break

        for direction in range(4):
            if maze.has_wall((r, c), direction, sensor_overrides):
                continue
            neighbour = maze.neighbour((r, c), direction)
            if not maze.in_bounds(neighbour):
                continue
            new_cost = cost + 1.0 + turn_cost * turn_distance(heading, direction)
            next_state = (neighbour[0], neighbour[1], direction)
            if new_cost + 1e-9 >= best.get(next_state, float("inf")):
                continue
            best[next_state] = new_cost
            parent[next_state] = state
            heuristic = abs(neighbour[0] - goal[0]) + abs(neighbour[1] - goal[1])
            heapq.heappush(frontier, (new_cost + heuristic, new_cost, next_state))

    if final_state is None:
        return None

    states = [final_state]
    while states[-1] != start_state:
        states.append(parent[states[-1]])
    states.reverse()
    return [(state[0], state[1]) for state in states]


def mission_route_preview(
    maze: GridMazeMap,
    start_heading: int,
    turn_cost: float,
    cell_size_cm: float = 60.0,
) -> Optional[List[Tuple[int, int]]]:
    if maze.start is None or maze.drop is None or maze.exit is None:
        return None
    # Preview only the operator-drawn fixed grid. Red observations from an old
    # run must not silently alter a new setup preview.
    planning = GridMazeMap(maze.rows, maze.cols)
    planning.manual_walls = set(maze.manual_walls)
    first = astar_oriented(
        planning,
        maze.start,
        start_heading,
        maze.drop,
        turn_cost,
        sensor_overrides=False,
    )
    if first is None:
        return None
    heading_at_drop = (
        direction_between(first[-2], first[-1])
        if len(first) >= 2
        else start_heading
    )
    second = astar_oriented(
        planning,
        maze.drop,
        heading_at_drop,
        maze.exit,
        turn_cost,
        sensor_overrides=False,
    )
    if second is None:
        return None
    return first + second[1:]
