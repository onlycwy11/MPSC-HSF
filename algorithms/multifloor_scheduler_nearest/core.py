"""Nearest-vertical-first (水平近路优先) conflict-resolution variant.

Algorithm (user-specified, single-pass, no iterative re-routing)
----------------------------------------------------------------
1. Hungarian assignment as in the main algorithm (unconstrained shortest
   travel time enters the cost matrix).
2. Build the full route for every assigned (robot, task) pair as two
   segments:  seg1 = start -> pickup, seg2 = pickup -> delivery.
3. Run conflict detection ONCE over the initial assignment.
4. For every interval that overlaps an earlier one on the same vertical
   resource (the "later" robot is judged to conflict before the elevator /
   stairs and must wait for the earlier user), the robot's routing is
   switched once as follows:
     a. locate the segment (seg1 or seg2) that contains the conflicting
        vertical resource edge;
     b. take the *start node of that segment* (A for the A->B segment,
        B for the B->C segment);
     c. for every reachable elevator/stair resource, compare the HORIZONTAL
        cost only: the sum of two floor-only shortest walks,
          ① plane walk segment start -> entry landing,
          ③ plane walk exit landing -> segment end;
        the vertical ride (②) does NOT enter the selection;
        wheeled robots may only choose elevators;
     d. if the horizontally-nearest resource differs from the one currently
        used, re-route that segment through it (still built as ① plane +
        ② vertical + ③ plane); if it is the current one, keep it (the robot
        queues and waits).
   No second conflict re-detection is performed: every conflicted robot /
   segment is processed at most once.
5. Remaining contention is resolved by a pure FIFO queue per resource
   (first arrival to the resource entrance goes first; ties broken by robot
   id; final tie-break by shorter vertical ride time). Plans are fitted to
   the reservation schedule without any further route switching.

Objective: minimise the sum of completion times over all robot-task pairs.
"""

from __future__ import annotations

import heapq
import json
import math
import random
import sqlite3
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Sequence, Set, Tuple


INF = 10**15
EPS = 1e-9
# Tolerance for "interval has ended" in sliding-window conflict detection
# (see the comment in the main algorithm's core.py): a slightly larger expiry
# tolerance prevents ~1e-9 float residues at physically-touching junctions
# from being reported as conflicts.
SLIDING_EPS = 1e-7


@dataclass(frozen=True)
class Node:
    id: str
    floor: int
    x: float
    y: float
    z: float = 0.0
    node_type: str = "corridor"


@dataclass(frozen=True)
class Edge:
    src: str
    dst: str
    length: float
    kind: str = "floor"
    capacity: int = 1
    resource: Optional[str] = None


@dataclass
class BuildingMap:
    nodes: Dict[str, Node]
    adjacency: Dict[str, List[Edge]]
    resources: Dict[str, int] = field(default_factory=dict)
    _graph: Optional["_IntGraph"] = field(default=None, repr=False, compare=False)

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "BuildingMap":
        nodes = {
            str(n["id"]): Node(
                str(n["id"]), int(n.get("floor", 0)), float(n.get("x", 0)), float(n.get("y", 0)),
                float(n.get("z", float(n.get("floor", 0)) * 3.5)), str(n.get("node_type", "corridor")),
            )
            for n in data.get("nodes", [])
        }
        adjacency: Dict[str, List[Edge]] = {key: [] for key in nodes}
        resources: Dict[str, int] = {str(k): int(v) for k, v in data.get("resources", {}).items()}
        for raw in data.get("edges", []):
            edge = Edge(
                src=str(raw["src"]),
                dst=str(raw["dst"]),
                length=float(raw.get("length", 1.0)),
                kind=str(raw.get("kind", "floor")),
                capacity=int(raw.get("capacity", resources.get(str(raw.get("resource", "")), 1))),
                resource=(str(raw["resource"]) if raw.get("resource") is not None else None),
            )
            if edge.src not in adjacency:
                adjacency[edge.src] = []
            adjacency[edge.src].append(edge)
            if edge.resource:
                resources.setdefault(edge.resource, edge.capacity)
        elevator_edges: Dict[str, List[Edge]] = {}
        for out_edges in adjacency.values():
            for edge in out_edges:
                if edge.kind == "elevator" and edge.resource:
                    elevator_edges.setdefault(edge.resource, []).append(edge)
        for resource, shaft_edges in elevator_edges.items():
            resources[resource] = 1
            stops = {edge.src for edge in shaft_edges} | {edge.dst for edge in shaft_edges}
            by_floor: Dict[int, List[str]] = {}
            for node_id in stops:
                if node_id in nodes:
                    by_floor.setdefault(nodes[node_id].floor, []).append(node_id)
            capacity = resources.get(resource, 1)
            floors = sorted(by_floor)
            for src_floor in floors:
                for dst_floor in floors:
                    if src_floor == dst_floor:
                        continue
                    for src in by_floor[src_floor]:
                        for dst in by_floor[dst_floor]:
                            if not any(edge.dst == dst and edge.kind == "elevator" and edge.resource == resource
                                       for edge in adjacency.get(src, [])):
                                adjacency.setdefault(src, []).append(
                                    Edge(src, dst, 0.0, "elevator", capacity, resource)
                                )
        return cls(nodes, adjacency, resources)

    def graph(self) -> "_IntGraph":
        if self._graph is None:
            self._graph = _IntGraph(self)
        return self._graph


class _IntGraph:
    __slots__ = ("building", "ids", "idx_to_id", "id_to_idx", "adj", "radj", "edges", "n")

    def __init__(self, building: BuildingMap) -> None:
        self.building = building
        self.ids = list(building.nodes.keys())
        self.n = len(self.ids)
        self.id_to_idx = {nid: i for i, nid in enumerate(self.ids)}
        self.idx_to_id = self.ids
        self.adj: List[List[Tuple[int, int]]] = [[] for _ in range(self.n)]
        self.radj: List[List[Tuple[int, int]]] = [[] for _ in range(self.n)]
        self.edges: List[Edge] = []
        for src, out_edges in building.adjacency.items():
            si = self.id_to_idx.get(src)
            if si is None:
                continue
            for edge in out_edges:
                di = self.id_to_idx.get(edge.dst)
                if di is None:
                    continue
                ei = len(self.edges)
                self.edges.append(edge)
                self.adj[si].append((di, ei))
                self.radj[di].append((si, ei))

    def idx(self, node_id: str) -> int:
        return self.id_to_idx[node_id]

    def nid(self, idx: int) -> str:
        return self.idx_to_id[idx]


def _robot_key(robot: Robot) -> Tuple[float, float, bool]:
    return (
        round(float(robot.speed_xy), 6),
        round(float(robot.speed_z), 6),
        robot.robot_type.lower() in {"wheeled", "wheel", "agv"},
    )


class DistanceCache:
    __slots__ = ("building", "graph", "config", "columns", "_paths", "_weights", "_can_use")

    _ELEV_STAIR = ("elevator", "stair")

    def __init__(self, building: BuildingMap, config: "SchedulerConfig") -> None:
        self.building = building
        self.graph = building.graph()
        self.config = config
        self.columns: Dict[Tuple[Any, ...], List[float]] = {}
        self._paths: Dict[Tuple[Any, ...], Optional[RouteCandidate]] = {}
        self._weights: Dict[Tuple[Any, ...], List[float]] = {}
        self._can_use: Dict[Tuple[Any, ...], List[bool]] = {}

    def _restriction_key(self, banned_resources: Optional[Set[str]], allowed_vertical: Optional[str]) -> Tuple[Any, ...]:
        return (tuple(sorted(banned_resources or ())), allowed_vertical)

    def _robot_config_key(self, robot: Robot, config: "SchedulerConfig") -> Tuple[Any, ...]:
        return (
            _robot_key(robot),
            round(float(config.elevator_board_time), 9),
            round(float(config.elevator_floor_time), 9),
            round(float(config.stair_speed), 9),
            round(float(config.floor_height), 9),
            round(float(config.default_speed_xy), 9),
            bool(config.allow_stair_for_non_wheeled),
        )

    def _weights_for(self, robot: Robot, config: "SchedulerConfig") -> List[float]:
        key = self._robot_config_key(robot, config)
        weights = self._weights.get(key)
        if weights is None:
            building = self.building
            weights = [edge_time(edge, robot, building, config) for edge in self.graph.edges]
            self._weights[key] = weights
            wheeled = robot.robot_type.lower() in {"wheeled", "wheel", "agv"}
            allow_stair = bool(config.allow_stair_for_non_wheeled)
            self._can_use[key] = [
                not (edge.kind == "stair" and (wheeled or not allow_stair))
                for edge in self.graph.edges
            ]
        return weights

    def _can_use_for(self, robot: Robot, config: "SchedulerConfig") -> List[bool]:
        key = self._robot_config_key(robot, config)
        if key not in self._can_use:
            self._weights_for(robot, config)
        return self._can_use[key]

    def _column_key(self, goal_idx: int, robot: Robot, banned_resources: Optional[Set[str]], allowed_vertical: Optional[str]) -> Tuple[Any, ...]:
        return (_robot_key(robot), goal_idx) + self._restriction_key(banned_resources, allowed_vertical)

    def _build_column(self, goal_idx: int, robot: Robot, banned_resources: Optional[Set[str]], allowed_vertical: Optional[str]) -> List[float]:
        graph = self.graph
        config = self.config
        banned = banned_resources or set()
        weights = self._weights_for(robot, config)
        can_use = self._can_use_for(robot, config)
        edges = graph.edges
        dist = [INF] * graph.n
        dist[goal_idx] = 0.0
        heap: List[Tuple[float, int]] = [(0.0, goal_idx)]
        while heap:
            cost, node = heapq.heappop(heap)
            if cost > dist[node] + EPS:
                continue
            for back_idx, ei in graph.radj[node]:
                if not can_use[ei]:
                    continue
                edge = edges[ei]
                if edge.resource in banned:
                    continue
                if allowed_vertical and edge.kind in self._ELEV_STAIR and edge.kind != allowed_vertical:
                    continue
                next_cost = cost + weights[ei]
                if next_cost + EPS < dist[back_idx]:
                    dist[back_idx] = next_cost
                    heapq.heappush(heap, (next_cost, back_idx))
        return dist

    def dist(
        self,
        start: str,
        goal: str,
        robot: Robot,
        banned_resources: Optional[Set[str]] = None,
        allowed_vertical: Optional[str] = None,
    ) -> float:
        goal_idx = self.graph.idx(goal)
        key = self._column_key(goal_idx, robot, banned_resources, allowed_vertical)
        column = self.columns.get(key)
        if column is None:
            column = self._build_column(goal_idx, robot, banned_resources, allowed_vertical)
            self.columns[key] = column
        return column[self.graph.idx(start)]

    def shortest(
        self,
        start: str,
        goal: str,
        robot: Robot,
        banned_resources: Optional[Set[str]] = None,
        allowed_vertical: Optional[str] = None,
    ) -> Optional[RouteCandidate]:
        banned = banned_resources or set()
        key = (_robot_key(robot), start, goal, self._restriction_key(banned, allowed_vertical))
        cached = self._paths.get(key)
        if cached is not None:
            return cached if cached.travel_time < INF else None
        graph = self.graph
        config = self.config
        weights = self._weights_for(robot, config)
        can_use = self._can_use_for(robot, config)
        edges = graph.edges
        start_idx = graph.idx(start)
        goal_idx = graph.idx(goal)
        distance = [INF] * graph.n
        previous = [None] * graph.n
        distance[start_idx] = 0.0
        heap: List[Tuple[float, str, int]] = [(0.0, start, start_idx)]
        while heap:
            cost, node_id, node = heapq.heappop(heap)
            if cost > distance[node] + EPS:
                continue
            if node == goal_idx:
                break
            for nxt, ei in graph.adj[node]:
                if not can_use[ei]:
                    continue
                edge = edges[ei]
                if edge.resource in banned:
                    continue
                if allowed_vertical and edge.kind in self._ELEV_STAIR and edge.kind != allowed_vertical:
                    continue
                next_cost = cost + weights[ei]
                if next_cost + EPS < distance[nxt]:
                    distance[nxt] = next_cost
                    previous[nxt] = (node, edge)
                    heapq.heappush(heap, (next_cost, graph.nid(nxt), nxt))
        if distance[goal_idx] >= INF:
            self._paths[key] = RouteCandidate([], [], INF, "none")
            return None
        edges_rev: List[Edge] = []
        nodes: List[str] = [goal]
        current = goal_idx
        while current != start_idx:
            parent, edge = previous[current]
            edges_rev.append(edge)
            current = parent
            nodes.append(graph.nid(current))
        edges_rev.reverse()
        nodes.reverse()
        vertical = "none"
        kinds = {edge.kind for edge in edges_rev}
        if "elevator" in kinds:
            vertical = "elevator"
        elif "stair" in kinds:
            vertical = "stair"
        result = RouteCandidate(nodes, edges_rev, distance[goal_idx], vertical)
        self._paths[key] = result
        return result


@dataclass(frozen=True)
class Robot:
    id: str
    robot_type: str
    start_node: str
    speed_xy: float = 1.5
    speed_z: float = 0.5
    task_type: str = "general"
    available_at: float = 0.0


@dataclass(frozen=True)
class Task:
    id: str
    pickup: str
    delivery: str
    task_type: str = "general"
    release_time: float = 0.0


@dataclass
class RouteCandidate:
    nodes: List[str]
    edges: List[Edge]
    travel_time: float
    vertical_mode: str


@dataclass
class Plan:
    robot: Robot
    task: Task
    candidates: List[RouteCandidate]
    candidate_index: int = 0
    start_time: float = 0.0
    wait_time: float = 0.0
    # 由电梯真实服务层回填：实际完成时刻与每段乘梯占用窗口 [ready, alight]。
    real_completion: Optional[float] = None
    elevator_windows: List[Tuple[str, float, float]] = field(default_factory=list)

    @property
    def candidate(self) -> RouteCandidate:
        return self.candidates[self.candidate_index]

    @property
    def travel_time(self) -> float:
        return self.candidate.travel_time

    @property
    def end_time(self) -> float:
        return self.start_time + self.travel_time

    def resource_intervals(self, building: Optional[BuildingMap] = None, config: Optional[SchedulerConfig] = None) -> List[Tuple[str, float, float, int, Optional[int]]]:
        intervals: List[Tuple[str, float, float, int, Optional[int]]] = []
        now = self.start_time
        for edge in self.candidate.edges:
            duration = edge_time(edge, self.robot, building, config)
            resource = _edge_resource_key(edge, building)
            if resource:
                direction = None
                if edge.kind == "stair":
                    floor_delta = 0
                    if building and edge.src in building.nodes and edge.dst in building.nodes:
                        floor_delta = building.nodes[edge.dst].floor - building.nodes[edge.src].floor
                    direction = 1 if floor_delta > 0 or (floor_delta == 0 and edge.src < edge.dst) else -1
                intervals.append((resource, now, now + duration, edge.capacity, direction))
            now += duration
        return intervals


@dataclass(frozen=True)
class Conflict:
    resource: str
    robot_ids: Tuple[str, ...]
    start: float
    end: float


@dataclass
class SchedulerConfig:
    # 载客一次 = 开门接1.5 + 关门接1.5 + 运行1.75*n + 开门离1.5 = board4.5 + 1.75*n
    elevator_board_time: float = 4.5
    elevator_floor_time: float = 1.75
    # 离馆关门：与当前人无关，仅当后面还有排队请求时顺延给下一位，队列空则忽略。
    elevator_door_close: float = 1.5
    # 电梯初始停靠层（0 基楼层编号，即第 1 层）。空闲停在上一趟目标层。
    elevator_initial_floor: int = 0
    stair_speed: float = 0.5
    floor_height: float = 3.5
    default_speed_xy: float = 1.5
    max_iterations: int = 1000
    epsilon: float = 1e-6
    unmatched_penalty: float = 1e9
    allow_stair_for_non_wheeled: bool = True


def _edge_resource_key(edge: Edge, building: Optional[BuildingMap] = None) -> str:
    if edge.kind == "elevator":
        return "elevator:" + (edge.resource or "|".join(sorted((edge.src, edge.dst))))
    if edge.kind == "stair":
        segment = "|".join(sorted((edge.src, edge.dst)))
        if building and edge.src in building.nodes and edge.dst in building.nodes:
            floor_a = building.nodes[edge.src].floor
            floor_b = building.nodes[edge.dst].floor
            segment = f"{min(floor_a, floor_b)}-{max(floor_a, floor_b)}"
        return "stair:" + (edge.resource or "stair") + ":" + segment
    if edge.kind != "floor" or not edge.resource:
        return ""
    return f"floor:{edge.resource}:{edge.src}->{edge.dst}"


def edge_time(edge: Edge, robot: Robot, building: Optional[BuildingMap], config: Optional[SchedulerConfig] = None) -> float:
    config = config or SchedulerConfig()
    if edge.kind == "elevator":
        floors = 1
        if building and edge.src in building.nodes and edge.dst in building.nodes:
            floors = max(1, abs(building.nodes[edge.src].floor - building.nodes[edge.dst].floor))
        return config.elevator_board_time + config.elevator_floor_time * floors
    if edge.kind == "stair":
        floors = 1
        if building and edge.src in building.nodes and edge.dst in building.nodes:
            floors = max(1, abs(building.nodes[edge.src].floor - building.nodes[edge.dst].floor))
        return config.floor_height * floors / max(config.stair_speed, EPS)
    return edge.length / max(robot.speed_xy, EPS)


def robot_can_use_edge(robot: Robot, edge: Edge, config: SchedulerConfig) -> bool:
    if edge.kind == "stair" and robot.robot_type.lower() in {"wheeled", "wheel", "agv"}:
        return False
    if edge.kind == "stair" and not config.allow_stair_for_non_wheeled:
        return False
    return True


def dijkstra(
    building: BuildingMap,
    robot: Robot,
    start: str,
    goal: str,
    config: SchedulerConfig,
    banned_resources: Optional[Set[str]] = None,
    allowed_vertical: Optional[str] = None,
) -> Optional[RouteCandidate]:
    cache = getattr(building, "_dijkstra_cache", None)
    if cache is None:
        cache = DistanceCache(building, config)
        building._dijkstra_cache = cache
    return cache.shortest(start, goal, robot, banned_resources, allowed_vertical)


def _distance(
    building: BuildingMap,
    robot: Robot,
    start: str,
    goal: str,
    config: SchedulerConfig,
    banned_resources: Optional[Set[str]] = None,
    allowed_vertical: Optional[str] = None,
) -> float:
    cache = getattr(building, "_dijkstra_cache", None)
    if cache is None:
        cache = DistanceCache(building, config)
        building._dijkstra_cache = cache
    return cache.dist(start, goal, robot, banned_resources, allowed_vertical)


def join_routes(first: RouteCandidate, second: RouteCandidate) -> RouteCandidate:
    return RouteCandidate(
        nodes=first.nodes + second.nodes[1:],
        edges=first.edges + second.edges,
        travel_time=first.travel_time + second.travel_time,
        vertical_mode=(
            first.vertical_mode
            if first.vertical_mode == second.vertical_mode
            else ("mixed" if first.vertical_mode != "none" and second.vertical_mode != "none" else first.vertical_mode if first.vertical_mode != "none" else second.vertical_mode)
        ),
    )


def _enumerate_modes_full(building: BuildingMap, robot: Robot, config: SchedulerConfig, pickup: str, delivery: str) -> List[Tuple[Optional[str], Optional[str]]]:
    modes: List[Tuple[Optional[str], Optional[str]]] = [(None, None)]
    start_floor = building.nodes[robot.start_node].floor
    pickup_floor = building.nodes[pickup].floor
    delivery_floor = building.nodes[delivery].floor
    if start_floor != pickup_floor or pickup_floor != delivery_floor:
        vertical_resources: Dict[str, Set[str]] = {"elevator": set(), "stair": set()}
        for edges in building.adjacency.values():
            for edge in edges:
                if edge.kind in vertical_resources:
                    vertical_resources[edge.kind].add(edge.resource or _edge_resource_key(edge))
        for resource in sorted(vertical_resources["elevator"]):
            modes.append(("elevator", resource))
        if robot.robot_type.lower() not in {"wheeled", "wheel", "agv"} and config.allow_stair_for_non_wheeled:
            for resource in sorted(vertical_resources["stair"]):
                modes.append(("stair", resource))
    return modes


def _banned_for(mode: Optional[str], selected_resource: Optional[str], building: BuildingMap) -> Set[str]:
    banned: Set[str] = set()
    if selected_resource:
        for edges in building.adjacency.values():
            for edge in edges:
                if edge.kind == mode:
                    resource = edge.resource or _edge_resource_key(edge)
                    if resource != selected_resource:
                        banned.add(resource)
    return banned


def _candidate_times(
    building: BuildingMap, robot: Robot, task: Task, config: SchedulerConfig
) -> List[Tuple[Optional[str], Optional[str], float]]:
    t1 = _distance(building, robot, robot.start_node, task.pickup, config)
    if t1 >= INF:
        return []
    t2 = _distance(building, robot, task.pickup, task.delivery, config)
    if t2 >= INF:
        return []
    return [(None, None, t1 + t2)]


def route_candidates(
    building: BuildingMap, robot: Robot, task: Task, config: SchedulerConfig
) -> List[RouteCandidate]:
    modes = _enumerate_modes_full(building, robot, config, task.pickup, task.delivery)
    candidates: List[RouteCandidate] = []
    seen: Set[Tuple[str, ...]] = set()
    for mode, selected_resource in modes:
        banned = _banned_for(mode, selected_resource, building)
        first = dijkstra(building, robot, robot.start_node, task.pickup, config, banned_resources=banned, allowed_vertical=mode)
        if not first:
            continue
        second = dijkstra(building, robot, task.pickup, task.delivery, config, banned_resources=banned, allowed_vertical=mode)
        if not second:
            continue
        merged = join_routes(first, second)
        key = tuple(edge.src + "->" + edge.dst + ":" + edge.kind for edge in merged.edges)
        if key not in seen:
            seen.add(key)
            candidates.append(merged)
    candidates.sort(key=lambda c: c.travel_time)
    return candidates


def hungarian(cost: Sequence[Sequence[float]]) -> List[int]:
    n = len(cost)
    if n == 0:
        return []
    u = [0.0] * (n + 1)
    v = [0.0] * (n + 1)
    p = [0] * (n + 1)
    way = [0] * (n + 1)
    for i in range(1, n + 1):
        p[0] = i
        j0 = 0
        minv = [INF] * (n + 1)
        used = [False] * (n + 1)
        while True:
            used[j0] = True
            i0 = p[j0]
            delta = INF
            j1 = 0
            for j in range(1, n + 1):
                if not used[j]:
                    cur = cost[i0 - 1][j - 1] - u[i0] - v[j]
                    if cur < minv[j]:
                        minv[j] = cur
                        way[j] = j0
                    if minv[j] < delta:
                        delta = minv[j]
                        j1 = j
            for j in range(n + 1):
                if used[j]:
                    u[p[j]] += delta
                    v[j] -= delta
                else:
                    minv[j] -= delta
            j0 = j1
            if p[j0] == 0:
                break
        while True:
            j1 = way[j0]
            p[j0] = p[j1]
            j0 = j1
            if j0 == 0:
                break
    assignment = [-1] * n
    for j in range(1, n + 1):
        if p[j]:
            assignment[p[j] - 1] = j - 1
    return assignment


def build_initial_plans(
    building: BuildingMap,
    robots: Sequence[Robot],
    tasks: Sequence[Task],
    config: SchedulerConfig,
) -> Tuple[List[Plan], List[str]]:
    times: Dict[Tuple[int, int], List[Tuple[Optional[str], Optional[str], float]]] = {}
    for i, robot in enumerate(robots):
        for j, task in enumerate(tasks):
            if robot.task_type == task.task_type:
                times[(i, j)] = _candidate_times(building, robot, task, config)
    size = max(len(robots), len(tasks))
    matrix = [[0.0 for _ in range(size)] for _ in range(size)]
    for i in range(size):
        for j in range(size):
            if i < len(robots) and j < len(tasks):
                values = times.get((i, j))
                matrix[i][j] = values[0][2] if values else config.unmatched_penalty
            elif i >= len(robots) and j < len(tasks):
                matrix[i][j] = config.unmatched_penalty
            else:
                matrix[i][j] = 0.0
    assignment = hungarian(matrix)
    plans: List[Plan] = []
    unmatched: List[str] = []
    assigned_tasks: Set[int] = set()
    for i, j in enumerate(assignment[: len(robots)]):
        if 0 <= j < len(tasks) and times.get((i, j)) and matrix[i][j] < config.unmatched_penalty:
            candidates = route_candidates(building, robots[i], tasks[j], config)
            plans.append(
                Plan(
                    robots[i], tasks[j], candidates,
                    start_time=max(robots[i].available_at, tasks[j].release_time),
                )
            )
            assigned_tasks.add(j)
    unmatched.extend(tasks[j].id for j in range(len(tasks)) if j not in assigned_tasks)
    return plans, unmatched


def _interval_overlap(a_start: float, a_end: float, b_start: float, b_end: float) -> bool:
    return a_start < b_end - EPS and b_start < a_end - EPS


def detect_conflicts(plans: Sequence[Plan], building: BuildingMap, config: Optional[SchedulerConfig] = None) -> List[Conflict]:
    by_resource: Dict[str, List[Tuple[str, float, float, int, Optional[int]]]] = {}
    for plan in plans:
        for resource, start, end, capacity, direction in plan.resource_intervals(building, config):
            by_resource.setdefault(resource, []).append((plan.robot.id, start, end, capacity, direction))
    conflicts: List[Conflict] = []
    for resource, intervals in by_resource.items():
        # Elevators serialized by the real elevator service layer; skip overlap
        # detection here (free-flow windows would overlap before queueing).
        if resource.startswith("elevator:"):
            continue
        if resource.startswith("stair:"):
            resource_id = resource.split(":", 1)[1]
        elif resource.startswith("floor:"):
            resource_id = resource.split(":", 2)[1]
        else:
            resource_id = resource
        capacity = max(1, int(building.resources.get(resource_id, intervals[0][3] if intervals else 1)))
        intervals.sort(key=lambda item: (item[1], item[2], item[0]))
        active: List[Tuple[float, Tuple[str, float, float, int, Optional[int]]]] = []
        is_stair = resource.startswith("stair:")
        for current in intervals:
            while active and active[0][0] <= current[1] + SLIDING_EPS:
                heapq.heappop(active)
            dir_active = [
                item for _, item in active
                if current[4] is None or item[4] is None or item[4] != current[4]
            ]
            if (dir_active and is_stair) or (len(dir_active) >= capacity and not is_stair):
                members = tuple(sorted({current[0], *(item[0] for item in dir_active)}))
                starts = [current[1], *(item[1] for item in dir_active)]
                ends = [current[2], *(item[2] for item in dir_active)]
                conflicts.append(Conflict(resource, members, max(starts), min(ends)))
            heapq.heappush(active, (current[2], current))
    unique: Dict[Tuple[str, Tuple[str, ...]], Conflict] = {}
    for conflict in conflicts:
        key = (conflict.resource, conflict.robot_ids)
        old = unique.get(key)
        if old is None:
            unique[key] = conflict
        else:
            unique[key] = Conflict(old.resource, old.robot_ids, min(old.start, conflict.start), max(old.end, conflict.end))
    return list(unique.values())


def _build_schedule(
    fixed_reservations: Sequence[Dict[str, Any]],
) -> Dict[str, List[Tuple[float, float, str, Optional[int]]]]:
    schedule: Dict[str, List[Tuple[float, float, str, Optional[int]]]] = {}
    for r in fixed_reservations:
        resource = str(r.get("resource", ""))
        if not resource:
            continue
        schedule.setdefault(resource, []).append(
            (float(r["start"]), float(r["end"]), str(r.get("robot_id", "")), r.get("direction"))
        )
    for intervals in schedule.values():
        intervals.sort(key=lambda item: item[0])
    return schedule


def _compute_delay_against_schedule(
    plan: Plan,
    building: BuildingMap,
    config: SchedulerConfig,
    schedule: Dict[str, List[Tuple[float, float, str, Optional[int]]]],
) -> float:
    intervals = plan.resource_intervals(building, config)
    delay = 0.0
    # Iterate delay to convergence.  In crowded single-elevator cases the
    # queueing chain can be hundreds of plans long (each iteration moves past
    # one queued user), so the iteration cap must exceed the longest possible
    # chain; the loop breaks as soon as nothing updates.  (A fixed cap of 100
    # used to leave real overlaps in R256-scale scenarios.)
    for _ in range(10000):
        updated = False
        for resource, own_start, own_end, capacity, own_dir in intervals:
            # Elevators are serialized by the real elevator service layer.
            if resource.startswith("elevator:"):
                continue
            shifted_start = own_start + delay
            shifted_end = own_end + delay
            for ex_start, ex_end, ex_robot, ex_dir in schedule.get(resource, []):
                if ex_robot == plan.robot.id:
                    continue
                if resource.startswith("stair:") and own_dir is not None and ex_dir is not None:
                    if own_dir == ex_dir:
                        continue
                if shifted_start < ex_end - EPS and ex_start < shifted_end - EPS:
                    required_delay = ex_end - own_start + EPS
                    if required_delay > delay:
                        delay = required_delay
                        updated = True
        if not updated:
            break
    return max(0.0, delay)


# ---------------------------------------------------------------------------
# Nearest-vertical-first specifics
# ---------------------------------------------------------------------------


def _vertical_landings(building: BuildingMap) -> Dict[str, Dict[int, List[str]]]:
    """kind -> resource-key -> {floor: [landing node ids]}."""
    result: Dict[str, Dict[str, Dict[int, Set[str]]]] = {"elevator": {}, "stair": {}}
    for edges in building.adjacency.values():
        for edge in edges:
            if edge.kind not in result:
                continue
            resource = edge.resource or _edge_resource_key(edge)
            floors = result[edge.kind].setdefault(resource, {})
            f_src = building.nodes[edge.src].floor
            f_dst = building.nodes[edge.dst].floor
            floors.setdefault(f_src, set()).add(edge.src)
            floors.setdefault(f_dst, set()).add(edge.dst)
    return {kind: {res: {f: sorted(nodes) for f, nodes in floors.items()} for res, floors in res_map.items()}
            for kind, res_map in result.items()}


def _all_vertical_resources(building: BuildingMap) -> Set[str]:
    resources: Set[str] = set()
    for edges in building.adjacency.values():
        for edge in edges:
            if edge.kind in {"elevator", "stair"}:
                resources.add(edge.resource or _edge_resource_key(edge))
    return resources


def _split_segments(plan: Plan, building: BuildingMap, config: SchedulerConfig) -> Tuple[RouteCandidate, RouteCandidate]:
    """Return (seg1 = start->pickup, seg2 = pickup->delivery)."""
    candidate = plan.candidate
    pickup = plan.task.pickup
    split = None
    for index, node in enumerate(candidate.nodes):
        if node == pickup:
            split = index
            break
    if split is None or split == 0:
        seg1 = RouteCandidate([plan.robot.start_node, pickup], [], 0.0, "none")
    else:
        seg1 = RouteCandidate(candidate.nodes[: split + 1], candidate.edges[:split], 0.0, "none")
    if split is None:
        seg2 = RouteCandidate([pickup, plan.task.delivery], [], 0.0, "none")
    else:
        seg2 = RouteCandidate(candidate.nodes[split:], candidate.edges[split:], 0.0, "none")
    seg1.travel_time = sum(edge_time(edge, plan.robot, building, config) for edge in seg1.edges)
    seg2.travel_time = sum(edge_time(edge, plan.robot, building, config) for edge in seg2.edges)
    return seg1, seg2


def _replan_segment_through_resource(
    building: BuildingMap,
    robot: Robot,
    segment_start: str,
    segment_end: str,
    kind: str,
    resource: str,
    config: SchedulerConfig,
) -> Optional[RouteCandidate]:
    """Plane walk to landing -> vertical ride -> plane walk to segment end.

    Floor-only walking uses allowed_vertical='floor'; the vertical ride is a
    single edge for elevators (direct arc) or the stair shaft edges.
    """
    start_floor = building.nodes[segment_start].floor
    end_floor = building.nodes[segment_end].floor
    landings = _vertical_landings(building)[kind].get(resource, {})
    if start_floor not in landings or end_floor not in landings:
        return None
    entry = _nearest_landing(building, robot, segment_start, landings[start_floor], config)
    exit_ = _nearest_landing(building, robot, segment_end, landings[end_floor], config)
    if entry is None or exit_ is None:
        return None
    plane1 = dijkstra(building, robot, segment_start, entry, config, banned_resources=None, allowed_vertical="floor")
    if not plane1:
        return None
    plane2 = dijkstra(building, robot, exit_, segment_end, config, banned_resources=None, allowed_vertical="floor")
    if not plane2:
        return None
    vertical = dijkstra(building, robot, entry, exit_, config, banned_resources=None, allowed_vertical=kind)
    if not vertical:
        return None
    return join_routes(join_routes(plane1, vertical), plane2)


def _nearest_landing(
    building: BuildingMap,
    robot: Robot,
    source: str,
    landings: Sequence[str],
    config: SchedulerConfig,
) -> Optional[str]:
    best = None
    best_cost = INF
    for landing in landings:
        cost = _distance(building, robot, source, landing, config, banned_resources=None, allowed_vertical="floor")
        if cost < best_cost - EPS:
            best_cost = cost
            best = landing
    return best


def apply_nearest_vertical_reroute(
    plans: List[Plan],
    building: BuildingMap,
    config: SchedulerConfig,
) -> Tuple[int, int]:
    """Single-pass nearest-vertical reroute; returns (rerouted_count, processed_count).

    Only intervals that overlap an *earlier* interval on the same vertical
    resource are rerouted (the later robot is the one judged to conflict and
    must wait for the earlier user).  Each (robot, segment) is processed at
    most once; no iterative re-detection is performed.
    """
    vertical_landings = _vertical_landings(building)
    # segment index for every plan: 0 = seg1 (start->pickup), 1 = seg2 (pickup->delivery)
    by_resource: Dict[str, List[Tuple[str, float, float, int, Optional[int], int]]] = {}
    for plan in plans:
        seg1, seg2 = _split_segments(plan, building, config)
        segments = [seg1, seg2]
        for seg_index, segment in enumerate(segments):
            now = plan.start_time
            for edge in segment.edges:
                resource = _edge_resource_key(edge, building)
                if resource and (resource.startswith("elevator:") or resource.startswith("stair:")):
                    duration = edge_time(edge, plan.robot, building, config)
                    direction = None
                    if edge.kind == "stair":
                        floor_delta = building.nodes[edge.dst].floor - building.nodes[edge.src].floor
                        direction = 1 if floor_delta > 0 or (floor_delta == 0 and edge.src < edge.dst) else -1
                    by_resource.setdefault(resource, []).append(
                        (plan.robot.id, now, now + duration, edge.capacity, direction, seg_index)
                    )
                now += edge_time(edge, plan.robot, building, config)
    # mark later overlapping intervals on each vertical resource
    todo: List[Tuple[str, int, str]] = []  # (robot.id, segment_index, resource)
    for resource, intervals in by_resource.items():
        if not resource.startswith(("elevator:", "stair:")):
            continue
        is_stair = resource.startswith("stair:")
        intervals.sort(key=lambda item: (item[1], item[2], item[0]))
        active: List[Tuple[float, Tuple[str, float, float, int, Optional[int], int]]] = []
        for current in intervals:
            while active and active[0][0] <= current[1] + SLIDING_EPS:
                heapq.heappop(active)
            dir_active = [
                item for _, item in active
                if current[4] is None or item[4] is None or item[4] != current[4]
            ]
            capacity = max(1, int(building.resources.get(resource.split(":", 1)[1], current[3])))
            if (dir_active and is_stair) or (len(dir_active) >= capacity and not is_stair):
                todo.append((current[0], current[5], resource))
            heapq.heappush(active, (current[2], current))
    # process each conflicted (robot, segment) once
    rerouted = 0
    processed = 0
    for robot_id, seg_index, resource in todo:
        plan = next((p for p in plans if p.robot.id == robot_id), None)
        if plan is None:
            continue
        seg1, seg2 = _split_segments(plan, building, config)
        segment = seg1 if seg_index == 0 else seg2
        if not segment.edges:
            continue
        segment_start = plan.robot.start_node if seg_index == 0 else plan.task.pickup
        segment_end = plan.task.pickup if seg_index == 0 else plan.task.delivery
        start_floor = building.nodes[segment_start].floor
        if segment_start == segment_end:
            continue
        # determine current vertical kind/resource used by this segment
        current_kind = None
        current_resource = None
        for edge in segment.edges:
            if edge.kind in {"elevator", "stair"}:
                current_kind = edge.kind
                current_resource = edge.resource or _edge_resource_key(edge)
                break
        end_floor = building.nodes[segment_end].floor
        # candidate vertical resources: selection cost = HORIZONTAL ONLY, the
        # sum of two floor-only shortest walks: ① start -> entry landing and
        # ③ exit landing -> segment end.  The vertical ride (②) does NOT enter
        # the selection (horizontal-nearest-first); the chosen route itself is
        # still built as the full ①+②+③ three-leg segment.
        choices: List[Tuple[float, str, str, RouteCandidate]] = []  # (horiz_cost, kind, resource, segment)
        for kind in ("elevator", "stair"):
            if kind == "stair" and plan.robot.robot_type.lower() in {"wheeled", "wheel", "agv"}:
                continue
            for res, floors in vertical_landings[kind].items():
                if start_floor not in floors or end_floor not in floors:
                    continue
                entry = _nearest_landing(building, plan.robot, segment_start, floors[start_floor], config)
                exit_ = _nearest_landing(building, plan.robot, segment_end, floors[end_floor], config)
                if entry is None or exit_ is None:
                    continue
                d_in = _distance(building, plan.robot, segment_start, entry, config,
                                 banned_resources=None, allowed_vertical="floor")
                d_out = _distance(building, plan.robot, exit_, segment_end, config,
                                 banned_resources=None, allowed_vertical="floor")
                if d_in >= INF or d_out >= INF:
                    continue
                candidate_segment = _replan_segment_through_resource(
                    building, plan.robot, segment_start, segment_end, kind, res, config
                )
                if candidate_segment is None:
                    continue
                choices.append((d_in + d_out, kind, res, candidate_segment))
        if not choices:
            continue
        choices.sort(key=lambda item: item[0])
        best_cost, best_kind, best_resource, best_segment = choices[0]
        processed += 1
        if best_kind == current_kind and best_resource == current_resource:
            continue  # cheapest resource is the one already used -> keep and queue
        # rebuild the plan candidate from the two segments
        other = seg2 if seg_index == 0 else seg1
        merged = join_routes(best_segment, other) if seg_index == 0 else join_routes(other, best_segment)
        plan.candidates.insert(0, merged)
        plan.candidate_index = 0
        rerouted += 1
    return rerouted, processed


def _apply_elevator_service(
    plans: Sequence[Plan], building: BuildingMap, config: SchedulerConfig
) -> None:
    """真实电梯串行服务模型：轿厢停层递推 + 空驶接驾 + 载客开关门 + 离馆关门顺延。

    只负责电梯资源；楼梯/平面排队已由 FIFO 完成。服务顺序按
    (start_time, robot.id)。每部电梯维护 free_at（可派遣时刻）与 floor（停靠层）。
    一次乘梯：空驶接驾 t_pickup=free_at+|from-floor|*per_floor（与步行并行），
    ready=max(t_pickup, arrive)，ride=board(开接+关接+开离)+per_floor*n，
    alight=ready+ride。离馆关门 door_close 仅当后面还有排队请求时顺延给下一位。
    """
    ordered = sorted(plans, key=lambda p: (p.start_time, p.robot.id))
    queues: Dict[str, List[Tuple[int, int]]] = {}
    for p in ordered:
        for seg_idx, edge in enumerate(p.candidate.edges):
            if edge.kind == "elevator":
                res = _edge_resource_key(edge, building)
                queues.setdefault(res, []).append((id(p), seg_idx))
    last_request: Set[Tuple[int, int]] = {items[-1] for items in queues.values()}

    state: Dict[str, Dict[str, float]] = {}
    for p in ordered:
        p.elevator_windows = []
        t = p.start_time
        for seg_idx, edge in enumerate(p.candidate.edges):
            if edge.kind != "elevator":
                t += edge_time(edge, p.robot, building, config)
                continue
            res = _edge_resource_key(edge, building)
            st = state.setdefault(
                res, {"free_at": 0.0, "floor": float(config.elevator_initial_floor)}
            )
            from_floor = building.nodes[edge.src].floor
            to_floor = building.nodes[edge.dst].floor
            n = abs(to_floor - from_floor)
            arrive = t
            t_pickup = st["free_at"] + abs(from_floor - st["floor"]) * config.elevator_floor_time
            ready = max(t_pickup, arrive)
            ride = config.elevator_board_time + config.elevator_floor_time * n
            alight = ready + ride
            p.wait_time += max(0.0, ready - arrive)
            is_last = (id(p), seg_idx) in last_request
            st["free_at"] = alight if is_last else alight + config.elevator_door_close
            st["floor"] = float(to_floor)
            p.elevator_windows.append((res, ready, alight))
            t = alight
        p.real_completion = t


def resolve_wait_only(
    plans: List[Plan],
    building: BuildingMap,
    config: SchedulerConfig,
    fixed_reservations: Sequence[Dict[str, Any]] = (),
) -> Dict[str, Any]:
    """Pure FIFO queueing: no route switching at all.

    Plans are fitted to the reservation schedule in order of
    (ideal start time, robot id, shorter ride time first).  Robots that share
    a start node and start time are staggered by 1 s each in robot-id order,
    exactly like the main algorithm.
    """
    fixed = _build_schedule(fixed_reservations)
    initial = detect_conflicts(plans, building, config)
    iterations = 0
    prev_signature = None
    for iteration in range(config.max_iterations):
        iterations = iteration + 1
        schedule = {resource: list(intervals) for resource, intervals in fixed.items()}
        for plan in plans:
            plan.start_time = max(plan.robot.available_at, plan.task.release_time)
            plan.wait_time = 0.0
        start_groups: Dict[Tuple[str, float], List[Plan]] = {}
        for plan in plans:
            key = (plan.robot.start_node, round(plan.start_time, 3))
            start_groups.setdefault(key, []).append(plan)
        for group in start_groups.values():
            if len(group) > 1:
                group.sort(key=lambda p: p.robot.id)
                for i, plan in enumerate(group[1:], 1):
                    plan.start_time += 1.0 * i
                    plan.wait_time += 1.0 * i
        # FIFO per resource entrance: (start time, robot id, ride time)
        sorted_plans = sorted(plans, key=lambda p: (p.start_time, p.robot.id, p.travel_time))
        for plan in sorted_plans:
            delay = _compute_delay_against_schedule(plan, building, config, schedule)
            if delay > EPS:
                plan.start_time += delay
                plan.wait_time += delay
            for resource, start, end, capacity, direction in plan.resource_intervals(building, config):
                schedule.setdefault(resource, []).append((start, end, plan.robot.id, direction))
                schedule[resource].sort(key=lambda item: item[0])
        signature = tuple(sorted(
            (plan.robot.id, round(plan.start_time, 3))
            for plan in plans
        ))
        if signature == prev_signature:
            break
        prev_signature = signature
    final = detect_conflicts(plans, building, config)
    return {
        "initial_conflict_count": len(initial),
        "final_conflict_count": len(final),
        "iterations": iterations,
        "reroutes": 0,
    }


def schedule(
    building: BuildingMap,
    robots: Sequence[Robot],
    tasks: Sequence[Task],
    config: Optional[SchedulerConfig] = None,
    reservations: Sequence[Dict[str, Any]] = (),
) -> Dict[str, Any]:
    config = config or SchedulerConfig()
    plans, unmatched = build_initial_plans(building, robots, tasks, config)
    fixed_reservations = [dict(item) for item in reservations]
    rerouted, processed = apply_nearest_vertical_reroute(plans, building, config)
    queue_stats = resolve_wait_only(plans, building, config, fixed_reservations)
    _apply_elevator_service(plans, building, config)

    def _final_end(plan: Plan) -> float:
        return plan.real_completion if plan.real_completion is not None else plan.end_time

    results: List[Dict[str, Any]] = []
    for plan in sorted(plans, key=lambda item: item.task.id):
        final_end = _final_end(plan)
        results.append(
            {
                "task_id": plan.task.id,
                "robot_id": plan.robot.id,
                "robot_type": plan.robot.robot_type,
                "task_type": plan.task.task_type,
                "projected_node": plan.task.delivery,
                "available_at": round(final_end, 6),
                "start_time": round(plan.start_time, 6),
                "completion_time": round(final_end, 6),
                "travel_time": round(plan.travel_time, 6),
                "wait_time": round(plan.wait_time, 6),
                "vertical_mode": plan.candidate.vertical_mode,
                "path": plan.candidate.nodes,
                "edges": [
                    {"src": edge.src, "dst": edge.dst, "kind": edge.kind, "resource": edge.resource}
                    for edge in plan.candidate.edges
                ],
                "resource_reservations": [
                    {"resource": resource, "start": round(start, 6), "end": round(end, 6)}
                    for resource, start, end in plan.elevator_windows
                ] + [
                    {"resource": resource, "start": round(start, 6), "end": round(end, 6)}
                    for resource, start, end, _, _ in plan.resource_intervals(building, config)
                    if not resource.startswith("elevator:")
                ],
            }
        )
    completion = [item["completion_time"] for item in results]
    travel = [item["travel_time"] for item in results]
    waits = [item["wait_time"] for item in results]
    metrics = {
        "assigned_count": len(results),
        "unassigned_count": len(unmatched),
        "makespan": max(completion, default=0.0),
        "sum_completion_time": round(sum(completion), 6),
        "average_travel_time": round(sum(travel) / len(travel), 6) if travel else 0.0,
        "total_wait_time": round(sum(waits), 6),
        "rerouted": rerouted,
        "reroute_candidates_processed": processed,
        **queue_stats,
    }
    elevator_reservations = [
            {
                "resource": resource,
                "robot_id": plan.robot.id,
                "task_id": plan.task.id,
                "start": round(start, 6),
                "end": round(end, 6),
                "direction": None,
            }
            for plan in plans
            for resource, start, end in plan.elevator_windows
        ]
    other_reservations = [
            {
                "resource": resource,
                "robot_id": plan.robot.id,
                "task_id": plan.task.id,
                "start": round(start, 6),
                "end": round(end, 6),
                "direction": direction,
            }
            for plan in plans
            for resource, start, end, _, direction in plan.resource_intervals(building, config)
            if not resource.startswith("elevator:")
        ]
    current_reservations = elevator_reservations + other_reservations
    resource_schedule = sorted(
        fixed_reservations + current_reservations,
        key=lambda item: (item["resource"], float(item["start"]), str(item.get("robot_id", ""))),
    )
    return {
        "assignments": results,
        "unassigned_tasks": unmatched,
        "resource_schedule": resource_schedule,
        "metrics": metrics,
    }


def load_json(path: str | Path) -> Dict[str, Any]:
    return json.loads(Path(path).read_text(encoding="utf-8"))


def read_sqlite_inputs(db_path: str | Path) -> Tuple[List[Robot], List[Task]]:
    connection = sqlite3.connect(str(db_path))
    try:
        robot_columns = {row[1] for row in connection.execute("PRAGMA table_info(robots)")}
        robot_type_expr = "task_type" if "task_type" in robot_columns else "'general'"
        available_expr = "available_at" if "available_at" in robot_columns else "0"
        robots_rows = connection.execute(
            f"SELECT id, robot_type, start_node, speed_xy, speed_z, {robot_type_expr}, {available_expr} FROM robots"
        ).fetchall()
        tasks_rows = connection.execute(
            "SELECT id, pickup, delivery, task_type, release_time FROM tasks WHERE status IS NULL OR status='pending'"
        ).fetchall()
    finally:
        connection.close()
    robots = [
        Robot(str(row[0]), str(row[1]), str(row[2]), float(row[3] or 1.5),
              float(row[4] or 0.5), str(row[5] or "general"), float(row[6] or 0.0))
        for row in robots_rows
    ]
    tasks = [Task(str(row[0]), str(row[1]), str(row[2]), str(row[3] or "general"), float(row[4] or 0.0)) for row in tasks_rows]
    return robots, tasks


def read_sqlite_reservations(db_path: str | Path) -> List[Dict[str, Any]]:
    connection = sqlite3.connect(str(db_path))
    connection.row_factory = sqlite3.Row
    try:
        exists = connection.execute(
            "SELECT 1 FROM sqlite_master WHERE type='table' AND name='resource_schedule'"
        ).fetchone()
        if not exists:
            return []
        columns = {row[1] for row in connection.execute("PRAGMA table_info(resource_schedule)")}
        required = {"resource", "start", "end"}
        if not required.issubset(columns):
            raise ValueError("resource_schedule must contain resource, start, and end columns")
        return [dict(row) for row in connection.execute("SELECT * FROM resource_schedule")]
    finally:
        connection.close()


def parse_payload(payload: Dict[str, Any]) -> Tuple[BuildingMap, List[Robot], List[Task], SchedulerConfig]:
    building = BuildingMap.from_dict(payload["map"])
    config = SchedulerConfig(**{k: v for k, v in payload.get("config", {}).items() if k in SchedulerConfig.__dataclass_fields__})
    robots = [
        Robot(
            str(r["id"]), str(r.get("robot_type", "wheeled")), str(r["start_node"]),
            float(r.get("speed_xy", config.default_speed_xy)), float(r.get("speed_z", config.stair_speed)),
            str(r.get("task_type", "general")), float(r.get("available_at", 0.0)),
        )
        for r in payload.get("robots", [])
    ]
    tasks = [
        Task(str(t["id"]), str(t["pickup"]), str(t["delivery"]),
             str(t.get("task_type", "general")), float(t.get("release_time", 0.0)))
        for t in payload.get("tasks", [])
    ]
    return building, robots, tasks, config


def load_real_inputs(map_path: str | Path, db_path: str | Path) -> Tuple[BuildingMap, List[Robot], List[Task], SchedulerConfig]:
    building = BuildingMap.from_dict(load_json(map_path))
    robots, tasks = read_sqlite_inputs(db_path)
    return building, robots, tasks, SchedulerConfig()


def generate_virtual(config_data: Dict[str, Any]) -> Dict[str, Any]:
    floors = int(config_data.get("floors", 3))
    width = int(config_data.get("width", 6))
    height = int(config_data.get("height", 6))
    elevator_count = int(config_data.get("elevators", 1))
    stair_count = int(config_data.get("stairs", 1))
    robot_count = int(config_data.get("robots", 6))
    task_count = int(config_data.get("tasks", robot_count))
    floor_height = float(config_data.get("floor_height", 3.5))
    grid_spacing = float(config_data.get("grid_spacing", 5.0))
    seed = int(config_data.get("seed", 7))
    if floors < 1 or width < 1 or height < 1:
        raise ValueError("floors, width, and height must all be positive")
    if min(elevator_count, stair_count, robot_count, task_count) < 0:
        raise ValueError("elevators, stairs, robots, and tasks cannot be negative")
    connector_count = elevator_count + stair_count
    if connector_count > width * height:
        raise ValueError("width * height must be at least elevators + stairs")
    if task_count and connector_count == width * height:
        raise ValueError("at least one grid position must remain for task pickup/delivery nodes")
    if floor_height <= 0 or grid_spacing <= 0:
        raise ValueError("floor_height and grid_spacing must be positive")
    rng = random.Random(seed)
    robot_types = list(config_data.get("robot_types", ["wheeled", "legged"]))
    task_types = list(config_data.get("task_types", ["general"]))
    if not robot_types or not task_types:
        raise ValueError("robot_types and task_types must each contain at least one value")
    positions = [(x, y) for x in range(width) for y in range(height)]
    rng.shuffle(positions)
    elevator_positions = positions[:elevator_count]
    stair_positions = positions[elevator_count:connector_count]
    connector_types = {position: "elevator_lobby" for position in elevator_positions}
    connector_types.update({position: "stair_lobby" for position in stair_positions})
    nodes: List[Dict[str, Any]] = []
    edges: List[Dict[str, Any]] = []
    for floor in range(floors):
        for x in range(width):
            for y in range(height):
                node_id = f"f{floor}_x{x}_y{y}"
                nodes.append({
                    "id": node_id,
                    "floor": floor,
                    "x": x * grid_spacing,
                    "y": y * grid_spacing,
                    "z": floor * floor_height,
                    "node_type": connector_types.get((x, y), "corridor"),
                })
                if x > 0:
                    left = f"f{floor}_x{x-1}_y{y}"
                    edges += [{"src": left, "dst": node_id, "length": grid_spacing, "kind": "floor"}, {"src": node_id, "dst": left, "length": grid_spacing, "kind": "floor"}]
                if y > 0:
                    down = f"f{floor}_x{x}_y{y-1}"
                    edges += [{"src": down, "dst": node_id, "length": grid_spacing, "kind": "floor"}, {"src": node_id, "dst": down, "length": grid_spacing, "kind": "floor"}]
    resources: Dict[str, int] = {}
    for index in range(elevator_count):
        x, y = elevator_positions[index]
        resource = f"elevator-{index}"
        resources[resource] = 1
        for floor in range(floors - 1):
            a, b = f"f{floor}_x{x}_y{y}", f"f{floor+1}_x{x}_y{y}"
            edges += [{"src": a, "dst": b, "length": floor_height, "kind": "elevator", "resource": resource}, {"src": b, "dst": a, "length": floor_height, "kind": "elevator", "resource": resource}]
    for index in range(stair_count):
        x, y = stair_positions[index]
        resource = f"stair-{index}"
        resources[resource] = int(config_data.get("stair_capacity", 1))
        for floor in range(floors - 1):
            a, b = f"f{floor}_x{x}_y{y}", f"f{floor+1}_x{x}_y{y}"
            edges += [{"src": a, "dst": b, "length": floor_height, "kind": "stair", "resource": resource}, {"src": b, "dst": a, "length": floor_height, "kind": "stair", "resource": resource}]
    node_ids = [n["id"] for n in nodes]
    task_node_ids = [n["id"] for n in nodes if n["node_type"] == "corridor"] or node_ids
    robots: List[Dict[str, Any]] = []
    for index in range(robot_count):
        robots.append({
            "id": f"r{index}", "robot_type": robot_types[index % len(robot_types)],
            "task_type": task_types[index % len(task_types)], "start_node": rng.choice(node_ids),
            "speed_xy": float(config_data.get("speed_xy", 1.5)),
            "speed_z": float(config_data.get("speed_z", config_data.get("stair_speed", 0.5))),
        })
    tasks: List[Dict[str, Any]] = []
    for index in range(task_count):
        tasks.append({
            "id": f"t{index}", "pickup": rng.choice(task_node_ids), "delivery": rng.choice(task_node_ids),
            "task_type": task_types[index % len(task_types)],
        })
    return {
        "map": {"nodes": nodes, "edges": edges, "resources": resources},
        "robots": robots,
        "tasks": tasks,
        "config": config_data,
        "resource_schedule": config_data.get("resource_schedule", []),
    }
