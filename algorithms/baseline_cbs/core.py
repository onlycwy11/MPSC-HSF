"""Faithful Conflict-Based Search (CBS) for the unified continuous-time
multi-floor scheduling model.

This is a re-implementation of Sharon et al. (2015), *Conflict-based search
for optimal multi-agent pathfinding* (AIJ 219), adapted only in the motion
model: the paper assumes a discrete grid with unit time steps; here the low
level plans over a continuous-time graph with floor-transfer edges (one
elevator ride = one transfer with cost 4.5 + 1.75*|df|; one stair transfer =
3.5*|df| / v_stair).  The algorithmic structure is faithful: a high-level
Constraint Tree (CT) whose nodes hold per-robot resource-time constraints,
a low-level constrained shortest-path solver, best-first expansion by sum of
completion times, and conflict splitting until a conflict-free plan is found
(CBS optimality argument carries over because constraints partition the
solution space and expansion is by increasing cost).

Assignment is not part of CBS: tasks are pre-assigned to type-matching robots
by a min-cost (Hungarian) assignment on unconstrained path times, then CBS
resolves all resource conflicts for that fixed assignment.  This matches the
experiment protocol (one robot, one task per batch; wheeled robots use only
elevators).
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


@dataclass(frozen=True)
class Node:
    id: str
    floor: int
    x: float
    y: float


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
    # resource -> {node_id: floor} for elevator/stair connectors
    connectors: Dict[str, Dict[str, int]] = field(default_factory=dict)

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "BuildingMap":
        nodes = {
            str(n["id"]): Node(
                str(n["id"]), int(n.get("floor", 0)), float(n.get("x", 0)), float(n.get("y", 0))
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
        obj = cls(nodes, adjacency, resources)
        obj._postprocess()
        return obj

    def _postprocess(self) -> None:
        """Mark wide (two-way) corridors as non-conflicting and collect
        elevator/stair connector nodes per resource."""
        # A floor edge with a reverse floor edge is a wide two-way corridor:
        # per the agreed model it does not create head-on conflicts.  A
        # one-way corridor (no reverse edge) is a narrow single-lane resource.
        reverse = {
            (e.dst, e.src)
            for out in self.adjacency.values()
            for e in out
            if e.kind == "floor"
        }
        for src, out in list(self.adjacency.items()):
            new_out: List[Edge] = []
            for e in out:
                if e.kind == "floor" and (e.src, e.dst) in reverse:
                    e = Edge(e.src, e.dst, e.length, e.kind, 10**9, None)
                new_out.append(e)
            self.adjacency[src] = new_out
        # Collect connectors: for every elevator/stair resource, map node -> floor.
        connectors: Dict[str, Dict[str, int]] = {}
        for out in self.adjacency.values():
            for e in out:
                if e.kind in {"elevator", "stair"} and e.resource:
                    d = connectors.setdefault(e.resource, {})
                    for nid in (e.src, e.dst):
                        if nid in self.nodes:
                            d[nid] = self.nodes[nid].floor
        self.connectors = connectors

    def to_dict(self) -> Dict[str, Any]:
        edges: List[Dict[str, Any]] = []
        for out in self.adjacency.values():
            for edge in out:
                edges.append(
                    {
                        "src": edge.src,
                        "dst": edge.dst,
                        "length": edge.length,
                        "kind": edge.kind,
                        "capacity": edge.capacity,
                        "resource": edge.resource,
                    }
                )
        return {
            "nodes": [vars(node) for node in self.nodes.values()],
            "edges": edges,
            "resources": self.resources,
        }


@dataclass(frozen=True)
class Robot:
    id: str
    robot_type: str
    start_node: str
    speed_xy: float = 1.0
    speed_z: float = 1.0
    capability_type: str = "general"


@dataclass(frozen=True)
class Task:
    id: str
    pickup: str
    delivery: str
    task_type: Optional[str] = None
    release_time: float = 0.0


@dataclass
class RouteCandidate:
    nodes: List[str]
    edges: List[Edge]
    travel_time: float
    vertical_mode: str
    edge_windows: Optional[List[Tuple[float, float]]] = None  # 相对 t0 的 [start,end)


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

    def resource_intervals(self, building: Optional[BuildingMap] = None, config: Optional[SchedulerConfig] = None) -> List[Tuple[str, float, float, int]]:
        """Return (resource, start, end, capacity) intervals for this plan.

        Uses the per-edge occupancy windows recorded by the planner (which
        include waiting), shifted by the plan start time.  Only
        capacity-limited resources produce intervals: elevators, stairs and
        one-way (narrow) corridors.  Wide two-way corridors carry no resource.
        """
        if self.candidate.edge_windows:
            intervals: List[Tuple[str, float, float, int]] = []
            for (ws, we), edge in zip(self.candidate.edge_windows, self.candidate.edges):
                resource = edge.resource
                capacity = edge.capacity if edge.resource else 0
                if resource and capacity < INF:
                    intervals.append((resource, self.start_time + ws, self.start_time + we, capacity))
            return intervals
        intervals = []
        now = self.start_time
        for edge in self.candidate.edges:
            duration = edge_time(edge, self.robot, building, config)
            resource = edge.resource
            capacity = edge.capacity if edge.resource else 0
            if resource and capacity < INF:
                intervals.append((resource, now, now + duration, capacity))
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


def _edge_resource_key(edge: Edge) -> str:
    """Undirected key for ordinary capacity-one corridor occupancy."""
    if edge.kind not in {"floor", "stair", "elevator"}:
        return ""
    return "edge:" + "|".join(sorted((edge.src, edge.dst)))


def edge_time(edge: Edge, robot: Robot, building: Optional[BuildingMap], config: Optional[SchedulerConfig] = None) -> float:
    """Duration of a single edge in the unified model.

    - elevator transfer edge: one ride = board/unload 4.5 s + 1.75 s per floor
      (length of a transfer edge is the number of floors crossed);
    - stair transfer edge: vertical distance / stair speed
      (length is the vertical distance in meters);
    - floor edge: length / horizontal speed.
    """
    config = config or SchedulerConfig()
    if edge.kind == "elevator":
        floors = max(1, int(round(edge.length)))
        return config.elevator_board_time + config.elevator_floor_time * floors
    if edge.kind == "stair":
        return edge.length / max(config.stair_speed, EPS)
    return edge.length / max(robot.speed_xy, EPS)


def robot_can_use_edge(robot: Robot, edge: Edge, config: SchedulerConfig) -> bool:
    if edge.kind == "stair" and robot.robot_type.lower() in {"wheeled", "wheel", "agv"}:
        return False
    if edge.kind == "stair" and not config.allow_stair_for_non_wheeled:
        return False
    return True


def _earliest_free(start_t: float, duration: float, intervals: Sequence[Tuple[float, float]]) -> Tuple[float, float]:
    """Earliest t >= start_t such that [t, t+duration) overlaps no interval.

    Returns (t, accumulated_wait).
    """
    t = start_t
    wait = 0.0
    for _ in range(256):
        end = None
        for ts, te in intervals:
            if t < te - EPS and ts < t + duration - EPS:
                if end is None or te > end:
                    end = te
        if end is None:
            return t, wait
        t = max(t, end)
        wait = t - start_t
    return t, wait


def plan_path(
    building: BuildingMap,
    robot: Robot,
    start: str,
    goal: str,
    config: SchedulerConfig,
    forbidden: Optional[Dict[str, List[Tuple[float, float]]]] = None,
    t0: float = 0.0,
) -> Optional[Tuple[RouteCandidate, float, float]]:
    """Constrained shortest path from start to goal under resource-time
    constraints.  Waiting at nodes is allowed.  Elevator/stair resources act
    as direct transfer edges between any two of their connector nodes with the
    agreed single-ride duration.  Returns (route, arrival_time, wait_time)."""
    forbidden = forbidden or {}
    best: Dict[str, float] = {}
    prev: Dict[str, Tuple[str, Edge, float, float]] = {}
    heap: List[Tuple[float, str, int]] = [(t0, start, 0)]
    counter = 0
    while heap:
        t, node, _ = heapq.heappop(heap)
        if t > best.get(node, INF) + EPS:
            continue
        if node == goal:
            break
        for edge in building.adjacency.get(node, []):
            if not robot_can_use_edge(robot, edge, config):
                continue
            if edge.kind == "elevator" or edge.kind == "stair":
                res = edge.resource
                if not res:
                    continue
                src_floor = building.nodes[node].floor
                for ng, ng_floor in building.connectors.get(res, {}).items():
                    if ng == node:
                        continue
                    d = abs(ng_floor - src_floor)
                    if d < EPS:
                        continue
                    if edge.kind == "elevator":
                        dur = config.elevator_board_time + config.elevator_floor_time * d
                        length = float(d)
                    else:
                        dur = d * config.floor_height / max(config.stair_speed, EPS)
                        length = d * config.floor_height
                    t_ok, wait = _earliest_free(t, dur, forbidden.get(res, ()))
                    if t_ok + dur < best.get(ng, INF) - EPS:
                        best[ng] = t_ok + dur
                        prev[ng] = (node, Edge(node, ng, length, edge.kind, 1, res), t_ok, t_ok + dur)
                        counter += 1
                        heapq.heappush(heap, (t_ok + dur, ng, counter))
            else:
                dur = edge.length / max(robot.speed_xy, EPS)
                res = edge.resource
                if res:
                    t_ok, wait = _earliest_free(t, dur, forbidden.get(res, ()))
                else:
                    t_ok, wait = t, 0.0
                if t_ok + dur < best.get(edge.dst, INF) - EPS:
                    best[edge.dst] = t_ok + dur
                    prev[edge.dst] = (node, edge, t_ok, t_ok + dur)
                    counter += 1
                    heapq.heappush(heap, (t_ok + dur, edge.dst, counter))
    if goal not in best:
        return None
    edges: List[Edge] = []
    edge_windows: List[Tuple[float, float]] = []
    nodes: List[str] = [goal]
    current = goal
    while current != start:
        parent, edge, ts, te = prev[current]
        edges.append(edge)
        edge_windows.append((ts - t0, te - t0))
        current = parent
        nodes.append(current)
    edges.reverse()
    edge_windows.reverse()
    nodes.reverse()
    vertical = "none"
    kinds = {e.kind for e in edges}
    if "elevator" in kinds:
        vertical = "elevator"
    elif "stair" in kinds:
        vertical = "stair"
    pure = sum(edge_time(e, robot, building, config) for e in edges)
    wait = max(0.0, best[goal] - t0 - pure)
    route = RouteCandidate(nodes, edges, best[goal] - t0, vertical, edge_windows)
    return route, best[goal], wait


def plan_task(
    building: BuildingMap,
    robot: Robot,
    task: Task,
    config: SchedulerConfig,
    forbidden: Optional[Dict[str, List[Tuple[float, float]]]] = None,
    t0: float = 0.0,
) -> Optional[Tuple[RouteCandidate, float, float]]:
    """Plan start -> pickup -> delivery under constraints.  Returns
    (route, completion_time, total_wait)."""
    forbidden = forbidden or {}
    first = plan_path(building, robot, robot.start_node, task.pickup, config, forbidden, t0)
    if first is None:
        return None
    r1, t1, w1 = first
    if task.pickup == task.delivery:
        return r1, t1, w1
    second = plan_path(building, robot, task.pickup, task.delivery, config, forbidden, t1)
    if second is None:
        return None
    r2, t2, w2 = second
    shift = t1 - t0
    merged = RouteCandidate(
        nodes=r1.nodes + r2.nodes[1:],
        edges=r1.edges + r2.edges,
        travel_time=t2 - t0,
        vertical_mode=(r1.vertical_mode if r1.vertical_mode == r2.vertical_mode else "mixed"),
        edge_windows=(r1.edge_windows or []) + ([(s + shift, e + shift) for s, e in (r2.edge_windows or [])]),
    )
    return merged, t2, w1 + w2


def dijkstra(
    building: BuildingMap,
    robot: Robot,
    start: str,
    goal: str,
    config: SchedulerConfig,
    banned_resources: Optional[Set[str]] = None,
    allowed_vertical: Optional[str] = None,
) -> Optional[RouteCandidate]:
    """Unconstrained shortest path used only for the initial assignment cost."""
    banned_resources = banned_resources or set()
    distance: Dict[str, float] = {start: 0.0}
    previous: Dict[str, Tuple[str, Edge]] = {}
    queue: List[Tuple[float, str]] = [(0.0, start)]
    while queue:
        cost, node = heapq.heappop(queue)
        if cost > distance.get(node, INF) + EPS:
            continue
        if node == goal:
            break
        for edge in building.adjacency.get(node, []):
            if edge.resource in banned_resources or not robot_can_use_edge(robot, edge, config):
                continue
            if allowed_vertical and edge.kind in {"elevator", "stair"} and edge.kind != allowed_vertical:
                continue
            next_cost = cost + edge_time(edge, robot, building, config)
            if next_cost + EPS < distance.get(edge.dst, INF):
                distance[edge.dst] = next_cost
                previous[edge.dst] = (node, edge)
                heapq.heappush(queue, (next_cost, edge.dst))
    if goal not in distance:
        return None
    edges: List[Edge] = []
    nodes: List[str] = [goal]
    current = goal
    while current != start:
        parent, edge = previous[current]
        edges.append(edge)
        current = parent
        nodes.append(current)
    edges.reverse()
    nodes.reverse()
    vertical = "none"
    kinds = {edge.kind for edge in edges}
    if "elevator" in kinds:
        vertical = "elevator"
    elif "stair" in kinds:
        vertical = "stair"
    return RouteCandidate(nodes, edges, distance[goal], vertical)


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
        edge_windows=(first.edge_windows or []) + ([(s + first.travel_time, e + first.travel_time) for s, e in (second.edge_windows or [])]),
    )


def route_candidates(
    building: BuildingMap, robot: Robot, task: Task, config: SchedulerConfig
) -> List[RouteCandidate]:
    """Unconstrained candidates under the unified model (elevator rides are
    single direct transfers).  Used for the initial assignment; the CBS low
    level replans under constraints."""
    planned = plan_task(building, robot, task, config, {}, 0.0)
    if planned is None:
        return []
    route, _completion, _wait = planned
    return [route]


def hungarian(cost: Sequence[Sequence[float]]) -> List[int]:
    """Minimum-cost assignment for a square matrix; returns column per row."""
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
    """Type-aware min-cost assignment on unconstrained path times."""
    candidates: Dict[Tuple[int, int], List[RouteCandidate]] = {}
    for i, robot in enumerate(robots):
        for j, task in enumerate(tasks):
            if task.task_type and robot.capability_type != task.task_type:
                candidates[(i, j)] = []
            else:
                candidates[(i, j)] = route_candidates(building, robot, task, config)
    size = max(len(robots), len(tasks))
    matrix = [[0.0 for _ in range(size)] for _ in range(size)]
    for i in range(size):
        for j in range(size):
            if i < len(robots) and j < len(tasks):
                values = candidates[(i, j)]
                matrix[i][j] = values[0].travel_time if values else config.unmatched_penalty
            elif i >= len(robots) and j < len(tasks):
                matrix[i][j] = config.unmatched_penalty
            else:
                matrix[i][j] = 0.0
    assignment = hungarian(matrix)
    plans: List[Plan] = []
    unmatched: List[str] = []
    assigned_tasks: Set[int] = set()
    for i, j in enumerate(assignment[: len(robots)]):
        if 0 <= j < len(tasks) and candidates[(i, j)] and matrix[i][j] < config.unmatched_penalty:
            plans.append(Plan(robots[i], tasks[j], candidates[(i, j)]))
            assigned_tasks.add(j)
    unmatched.extend(tasks[j].id for j in range(len(tasks)) if j not in assigned_tasks)
    return plans, unmatched


def _interval_overlap(a_start: float, a_end: float, b_start: float, b_end: float) -> bool:
    return a_start < b_end - EPS and b_start < a_end - EPS


def detect_conflicts(plans: Sequence[Plan], building: BuildingMap, config: Optional[SchedulerConfig] = None) -> List[Conflict]:
    """Resource-time conflict detection.  Only capacity-limited resources
    (elevator rides, stair transfers, one-way corridors) are checked."""
    by_resource: Dict[str, List[Tuple[str, float, float, int]]] = {}
    for plan in plans:
        for resource, start, end, capacity in plan.resource_intervals(building, config):
            by_resource.setdefault(resource, []).append((plan.robot.id, start, end, capacity))
    conflicts: List[Conflict] = []
    for resource, intervals in by_resource.items():
        capacity = max(1, int(building.resources.get(resource, intervals[0][3] if intervals else 1)))
        intervals.sort(key=lambda item: (item[1], item[2], item[0]))
        for index, current in enumerate(intervals):
            active = [item for item in intervals[:index] if item[2] > current[1] + EPS]
            if len(active) >= capacity:
                members = tuple(sorted({current[0], *(item[0] for item in active)}))
                starts = [current[1], *(item[1] for item in active)]
                ends = [current[2], *(item[2] for item in active)]
                conflicts.append(Conflict(resource, members, max(starts), min(ends)))
    unique: Dict[Tuple[str, Tuple[str, ...]], Conflict] = {}
    for conflict in conflicts:
        key = (conflict.resource, conflict.robot_ids)
        old = unique.get(key)
        if old is None:
            unique[key] = conflict
        else:
            unique[key] = Conflict(old.resource, old.robot_ids, min(old.start, conflict.start), max(old.end, conflict.end))
    return list(unique.values())


# ---------------------------------------------------------------------------
# CBS: high-level Constraint Tree
# ---------------------------------------------------------------------------

@dataclass
class CTNode:
    constraints: Dict[str, List[Tuple[str, float, float]]]
    plans: List[Plan]
    cost: float
    order: int


def _replan_robot(
    building: BuildingMap,
    plan: Plan,
    config: SchedulerConfig,
    constraints: Sequence[Tuple[str, float, float]],
) -> Optional[Tuple[RouteCandidate, float, float]]:
    forbidden: Dict[str, List[Tuple[float, float]]] = {}
    for res, ts, te in constraints:
        forbidden.setdefault(res, []).append((ts, te))
    return plan_task(building, plan.robot, plan.task, config, forbidden, t0=0.0)


def cbs_schedule(
    building: BuildingMap,
    robots: Sequence[Robot],
    tasks: Sequence[Task],
    config: Optional[SchedulerConfig] = None,
) -> Dict[str, Any]:
    """Faithful CBS on the continuous-time model.

    1. Hungarian assignment on unconstrained travel times (type-aware).
    2. CT search: root = no constraints; each node = per-robot constraint
       sets + low-level constrained paths; expand by smallest sum of
       completion times; split the earliest conflict by constraining one of
       the two robots and replanning only that robot; first conflict-free
       node popped is optimal for the given assignment.
    """
    config = config or SchedulerConfig()
    plans, unmatched = build_initial_plans(building, robots, tasks, config)
    if not plans:
        return {
            "assignments": [],
            "unassigned_tasks": list(unmatched),
            "metrics": {"assigned_count": 0, "unassigned_count": len(unmatched), "cbs_nodes": 0, "iterations": 0},
        }
    # Root: no constraints; use unconstrained candidates (already computed).
    root_plans = [Plan(p.robot, p.task, p.candidates) for p in plans]
    root_cost = sum(p.end_time for p in root_plans)
    heap: List[Tuple[float, int, CTNode]] = []
    counter = 0
    heapq.heappush(heap, (root_cost, counter, CTNode({}, root_plans, root_cost, counter)))
    counter += 1
    expanded = 0
    reroutes = 0
    best_solution: Optional[List[Plan]] = None
    best_cost = INF
    while heap:
        _, _, node = heapq.heappop(heap)
        expanded += 1
        if node.cost >= best_cost - EPS:
            continue
        conflicts = detect_conflicts(node.plans, building, config)
        if not conflicts:
            # Conflict-free => optimal for this assignment (best-first by cost).
            best_solution = node.plans
            best_cost = node.cost
            break
        # Split the earliest conflict into two constraint branches.
        conflict = sorted(conflicts, key=lambda c: (c.start, c.resource, c.robot_ids))[0]
        for robot_id in conflict.robot_ids:
            plan = next(p for p in node.plans if p.robot.id == robot_id)
            existing = node.constraints.get(robot_id, [])
            new_constraint = (conflict.resource, conflict.start, conflict.end)
            new_constraints = existing + [new_constraint] if new_constraint not in existing else list(existing)
            replanned = _replan_robot(building, plan, config, new_constraints)
            if replanned is None:
                continue  # infeasible branch: prune
            route, completion, wait = replanned
            reroutes += 1
            new_plans: List[Plan] = []
            for p in node.plans:
                if p.robot.id == robot_id:
                    new_plans.append(Plan(p.robot, p.task, [route], start_time=0.0, wait_time=wait))
                else:
                    new_plans.append(p)
            new_constraints_map = {k: list(v) for k, v in node.constraints.items()}
            new_constraints_map[robot_id] = new_constraints
            new_cost = sum(p.end_time for p in new_plans)
            heapq.heappush(heap, (new_cost, counter, CTNode(new_constraints_map, new_plans, new_cost, counter)))
            counter += 1
        if expanded >= config.max_iterations:
            break
    if best_solution is None and heap:
        # Fall back: cheapest node currently in the heap (best effort, still
        # conflict-resolved by serialization below).
        _, _, node = heapq.heappop(heap)
        best_solution = node.plans
    final_plans = best_solution if best_solution is not None else root_plans
    final_conflicts = detect_conflicts(final_plans, building, config)
    if final_conflicts:
        # Deterministic FIFO serialization as a safety net (should not fire
        # when CBS terminates optimally; only when the iteration cap hits).
        _serialize_conflicts(final_plans, building, config)
        final_conflicts = detect_conflicts(final_plans, building, config)
    # 真实电梯服务模型（轿厢停层递推 + 空驶接驾 + 关门顺延）。
    _apply_elevator_service(final_plans, building, config)

    def _final_end(plan: Plan) -> float:
        return plan.real_completion if plan.real_completion is not None else plan.end_time

    results: List[Dict[str, Any]] = []
    for plan in sorted(final_plans, key=lambda item: item.task.id):
        final_end = _final_end(plan)
        results.append(
            {
                "task_id": plan.task.id,
                "robot_id": plan.robot.id,
                "robot_type": plan.robot.robot_type,
                "start_time": round(plan.start_time, 6),
                "completion_time": round(final_end, 6),
                "travel_time": round(plan.travel_time, 6),
                "wait_time": round(plan.wait_time, 6),
                "vertical_mode": plan.candidate.vertical_mode,
                "path": plan.candidate.nodes,
                "edges": [
                    {"src": e.src, "dst": e.dst, "kind": e.kind, "resource": e.resource}
                    for e in plan.candidate.edges
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
        "initial_conflict_count": len(detect_conflicts(root_plans, building, config)),
        "final_conflict_count": len(final_conflicts),
        "iterations": expanded,
        "cbs_nodes": expanded,
        "reroutes": reroutes,
    }
    return {"assignments": results, "unassigned_tasks": unmatched, "metrics": metrics}


def _required_delay(plan: Plan, resource: str, others: Sequence[Plan], building: Optional[BuildingMap] = None, config: Optional[SchedulerConfig] = None) -> float:
    delay = 0.0
    own_intervals = [item for item in plan.resource_intervals(building, config) if item[0] == resource]
    for _, own_start, own_end, _ in own_intervals:
        for other in others:
            for other_resource, other_start, other_end, _ in other.resource_intervals(building, config):
                if other_resource != resource:
                    continue
                if _interval_overlap(own_start + delay, own_end + delay, other_start, other_end):
                    delay = max(delay, other_end - own_start + EPS)
    return max(0.0, delay)


def _shift(plan: Plan, delay: float) -> None:
    if delay > 0:
        plan.start_time += delay
        plan.wait_time += delay


def _apply_elevator_service(
    plans: Sequence[Plan], building: BuildingMap, config: SchedulerConfig
) -> None:
    """真实电梯串行服务模型：轿厢停层递推 + 空驶接驾 + 载客开关门 + 离馆关门顺延。

    只负责电梯资源；其余避让由 CBS/序列化完成。服务顺序按 (start_time, robot.id)。
    每部电梯维护 free_at（可派遣时刻）与 floor（停靠层）。空驶接驾
    t_pickup=free_at+|from-floor|*per_floor（与步行并行），ready=max(t_pickup,arrive)，
    ride=board(开接+关接+开离)+per_floor*n，alight=ready+ride。离馆关门 door_close
    仅当后面还有排队请求时顺延给下一位。资源键使用 edge.resource（如 elevator-0）。
    """
    ordered = sorted(plans, key=lambda p: (p.start_time, p.robot.id))
    queues: Dict[str, List[Tuple[int, int]]] = {}
    for p in ordered:
        for si, e in enumerate(p.candidate.edges):
            if e.kind == "elevator" and e.resource:
                queues.setdefault(e.resource, []).append((id(p), si))
    last_request: Set[Tuple[int, int]] = {items[-1] for items in queues.values()}

    state: Dict[str, Dict[str, float]] = {}
    for p in ordered:
        p.elevator_windows = []
        t = p.start_time
        for si, e in enumerate(p.candidate.edges):
            if e.kind != "elevator":
                t += edge_time(e, p.robot, building, config)
                continue
            res = e.resource
            st = state.setdefault(res, {"free_at": 0.0, "floor": float(config.elevator_initial_floor)})
            fr = building.nodes[e.src].floor
            to = building.nodes[e.dst].floor
            n = abs(to - fr)
            arrive = t
            pickup = st["free_at"] + abs(fr - st["floor"]) * config.elevator_floor_time
            ready = max(pickup, arrive)
            ride = config.elevator_board_time + config.elevator_floor_time * n
            alight = ready + ride
            p.wait_time += max(0.0, ready - arrive)
            is_last = (id(p), si) in last_request
            st["free_at"] = alight if is_last else alight + config.elevator_door_close
            st["floor"] = float(to)
            p.elevator_windows.append((res, ready, alight))
            t = alight
        p.real_completion = t


def _serialize_conflicts(plans: Sequence[Plan], building: BuildingMap, config: Optional[SchedulerConfig] = None) -> None:
    """Deterministic FIFO serialization (safety net, not part of CBS)."""
    changed = True
    guard = 0
    while changed and guard < 10000:
        guard += 1
        changed = False
        for conflict in detect_conflicts(plans, building):
            members = [plan for plan in plans if plan.robot.id in conflict.robot_ids]
            members.sort(key=lambda plan: (plan.start_time, plan.robot.id))
            for index, plan in enumerate(members[1:], 1):
                delay = _required_delay(plan, conflict.resource, members[:index], building, config)
                if delay > EPS:
                    _shift(plan, delay)
                    changed = True


def schedule(
    building: BuildingMap,
    robots: Sequence[Robot],
    tasks: Sequence[Task],
    config: Optional[SchedulerConfig] = None,
) -> Dict[str, Any]:
    return cbs_schedule(building, robots, tasks, config)


def load_json(path: str | Path) -> Dict[str, Any]:
    return json.loads(Path(path).read_text(encoding="utf-8"))


def read_sqlite_inputs(db_path: str | Path) -> Tuple[List[Robot], List[Task]]:
    connection = sqlite3.connect(str(db_path))
    try:
        robots_rows = connection.execute(
            "SELECT id, robot_type, start_node, speed_xy, speed_z, task_type FROM robots"
        ).fetchall()
        tasks_rows = connection.execute(
            "SELECT id, pickup, delivery, task_type, release_time FROM tasks WHERE status IS NULL OR status='pending'"
        ).fetchall()
    finally:
        connection.close()
    robots = [Robot(str(row[0]), str(row[1]), str(row[2]), float(row[3] or 1.0), float(row[4] or 1.0), str(row[5] or "general")) for row in robots_rows]
    tasks = [Task(str(row[0]), str(row[1]), str(row[2]), row[3], float(row[4] or 0.0)) for row in tasks_rows]
    return robots, tasks


def parse_payload(payload: Dict[str, Any]) -> Tuple[BuildingMap, List[Robot], List[Task], SchedulerConfig]:
    building = BuildingMap.from_dict(payload["map"])
    robots = [Robot(str(r["id"]), str(r.get("robot_type", "wheeled")), str(r["start_node"]), float(r.get("speed_xy", 1.5)), float(r.get("speed_z", 0.5)), str(r.get("task_type", "general"))) for r in payload.get("robots", [])]
    tasks = [Task(str(t["id"]), str(t["pickup"]), str(t["delivery"]), t.get("task_type"), float(t.get("release_time", 0.0))) for t in payload.get("tasks", [])]
    config = SchedulerConfig(**{k: v for k, v in payload.get("config", {}).items() if k in SchedulerConfig.__dataclass_fields__})
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
    seed = int(config_data.get("seed", 7))
    rng = random.Random(seed)
    nodes: List[Dict[str, Any]] = []
    edges: List[Dict[str, Any]] = []
    for floor in range(floors):
        for x in range(width):
            for y in range(height):
                node_id = f"f{floor}_x{x}_y{y}"
                nodes.append({"id": node_id, "floor": floor, "x": x, "y": y})
                if x > 0:
                    left = f"f{floor}_x{x-1}_y{y}"
                    edges += [{"src": left, "dst": node_id, "length": 1, "kind": "floor"}, {"src": node_id, "dst": left, "length": 1, "kind": "floor"}]
                if y > 0:
                    down = f"f{floor}_x{x}_y{y-1}"
                    edges += [{"src": down, "dst": node_id, "length": 1, "kind": "floor"}, {"src": node_id, "dst": down, "length": 1, "kind": "floor"}]
    connector_positions = [(0, 0), (width - 1, height - 1), (width // 2, height // 2), (0, height - 1)]
    resources: Dict[str, int] = {}
    for index in range(elevator_count):
        x, y = connector_positions[index % len(connector_positions)]
        resource = f"elevator-{index}"
        resources[resource] = int(config_data.get("elevator_capacity", 1))
        for floor in range(floors - 1):
            a, b = f"f{floor}_x{x}_y{y}", f"f{floor+1}_x{x}_y{y}"
            edges += [{"src": a, "dst": b, "length": 1, "kind": "elevator", "resource": resource}, {"src": b, "dst": a, "length": 1, "kind": "elevator", "resource": resource}]
    for index in range(stair_count):
        x, y = connector_positions[(index + elevator_count) % len(connector_positions)]
        resource = f"stair-{index}"
        resources[resource] = int(config_data.get("stair_capacity", 1))
        for floor in range(floors - 1):
            a, b = f"f{floor}_x{x}_y{y}", f"f{floor+1}_x{x}_y{y}"
            edges += [{"src": a, "dst": b, "length": 1, "kind": "stair", "resource": resource}, {"src": b, "dst": a, "length": 1, "kind": "stair", "resource": resource}]
    node_ids = [n["id"] for n in nodes]
    robots: List[Dict[str, Any]] = []
    robot_count = int(config_data.get("robots", 6))
    types = list(config_data.get("robot_types", ["wheeled", "legged"]))
    for index in range(robot_count):
        robots.append({"id": f"r{index}", "robot_type": types[index % len(types)], "start_node": rng.choice(node_ids), "speed_xy": float(config_data.get("speed_xy", 1.5)), "speed_z": float(config_data.get("speed_z", 0.5))})
    tasks: List[Dict[str, Any]] = []
    for index in range(int(config_data.get("tasks", robot_count))):
        tasks.append({"id": f"t{index}", "pickup": rng.choice(node_ids), "delivery": rng.choice(node_ids)})
    return {"map": {"nodes": nodes, "edges": edges, "resources": resources}, "robots": robots, "tasks": tasks, "config": config_data}
