"""MuNES-based baseline: faithful single-robot multi-floor path planning
(elevator-waiting-aware cost) with the simplest possible multi-robot layer
(greedy assignment + global first-come-first-served waiting).

Correspondence with the paper
-----------------------------
Donghwi Jung, Chan Kim, Jae-Kyung Cho, Seong-Woo Kim, *MuNES: Multifloor
Navigation Including Elevators and Stairs*, arXiv:2402.04535, 2024.  Official
code: https://github.com/donghwijung/MuNES (ROS package; planning/munes_wo_map.py
+ astar.py + utils.py).

The paper's algorithmic contribution is a *single-robot* trajectory planner on
a multi-floor graph whose A* edge cost explicitly accounts for elevator
waiting time.  The core formula (utils.calcCost in the official repo) is:

    cost(elevator edge) = run_time / elv_speed
                        + |elevator_current_floor - robot_floor| / elv_speed

i.e. the elevator must first travel from its current location to the robot's
floor ("come to pick you up"), then carry the robot.  The elevator location is
a global state updated after each planned trajectory.  MuNES contains NO
multi-robot mechanism (no assignment, no conflict resolution).

This baseline therefore keeps MuNES faithful in the layer that exists and uses
the most primitive possible mechanisms in the layers that the paper does not
cover, so that the comparison against our algorithm measures real gains:

    - assignment: greedy (tasks in release order, each given to the
      type-matching idle robot with the smallest static travel time) -- no
      global optimization;
    - path planning: MuNES elevator-waiting-aware cost (structure faithful to
      calcCost; parameters unified with the rest of the experiment: one ride =
      4.5 + 1.75*|df| s per elevator edge, come-to-pick-up waiting =
      1.75*|df| s of elevator travel per floor);
    - conflict resolution: waiting only (first-come-first-served).  Later
      robots never change path, never switch elevator/stair; they just start
      later.  Elevators are FIFO through the elevator-location state plus a
      per-ride occupancy window; corridors/stairs are FIFO through
      occupancy windows.
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
        """Wide two-way corridors are non-conflicting; collect elevator/stair
        connector nodes per resource."""
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
        connectors: Dict[str, Dict[str, int]] = {}
        for out in self.adjacency.values():
            for e in out:
                if e.kind in {"elevator", "stair"} and e.resource:
                    d = connectors.setdefault(e.resource, {})
                    for nid in (e.src, e.dst):
                        if nid in self.nodes:
                            d[nid] = self.nodes[nid].floor
        self.connectors = connectors


@dataclass(frozen=True)
class Robot:
    id: str
    robot_type: str
    start_node: str
    speed_xy: float = 1.5
    speed_z: float = 0.5
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
    edge_windows: Optional[List[Tuple[float, float]]] = None


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
    if edge.kind not in {"floor", "stair", "elevator"}:
        return ""
    return "edge:" + "|".join(sorted((edge.src, edge.dst)))


def edge_time(edge: Edge, robot: Robot, building: Optional[BuildingMap], config: Optional[SchedulerConfig] = None) -> float:
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


_STATIC_CACHE: Dict[Tuple[str, str, str], float] = {}


def _static_dijkstra(
    building: BuildingMap,
    robot: Robot,
    start: str,
    goal: str,
    config: SchedulerConfig,
) -> Tuple[Optional[List[str]], Optional[List[Edge]], float]:
    """Plain shortest path with unified edge costs (no elevator waiting).
    Used only for the greedy assignment estimate."""
    ck = (start, goal, robot.id)
    hit = _STATIC_CACHE.get(ck)
    if hit is not None:
        return None, None, hit
    dist: Dict[str, float] = {start: 0.0}
    prev: Dict[str, Tuple[str, Edge]] = {}
    heap: List[Tuple[float, str]] = [(0.0, start)]
    while heap:
        d, u = heapq.heappop(heap)
        if d > dist.get(u, INF) + EPS:
            continue
        if u == goal:
            break
        for e in building.adjacency.get(u, []):
            if not robot_can_use_edge(robot, e, config):
                continue
            cost = edge_time(e, robot, building, config)
            nd = d + cost
            if nd < dist.get(e.dst, INF) - EPS:
                dist[e.dst] = nd
                prev[e.dst] = (u, e)
                heapq.heappush(heap, (nd, e.dst))
    if goal not in prev and goal != start:
        return None, None, INF
    nodes: List[str] = [goal]
    edges: List[Edge] = []
    u = goal
    while u != start:
        pu, pe = prev[u]
        nodes.append(pu)
        edges.append(pe)
        u = pu
    nodes.reverse()
    edges.reverse()
    _STATIC_CACHE[ck] = dist.get(goal, 0.0)
    return nodes, edges, dist.get(goal, 0.0)


def _munes_leg(
    building: BuildingMap,
    robot: Robot,
    start: str,
    goal: str,
    config: SchedulerConfig,
    elv_floor: int,
) -> Tuple[Optional[List[str]], Optional[List[Edge]], float, float, int]:
    """MuNES elevator-waiting-aware shortest path (paper utils.calcCost).

    Edge costs:
      - floor: length / v_xy
      - stair: length / v_stair (legged only)
      - elevator (one direct ride): come-to-pick-up waiting
        1.75*|departure_floor - elv_floor|  plus  one ride
        4.5 + 1.75*|arrival_floor - departure_floor|.

    Returns (nodes, edges, arrival_time, elevator_wait, new_elv_floor,
    edge_windows).  edge_windows are the per-edge [t_start, t_end] windows of
    the nominal (conflict-free) profile used by the FIFO waiting layer."""
    dist: Dict[str, float] = {start: 0.0}
    prev: Dict[str, Tuple[str, Edge]] = {}
    heap: List[Tuple[float, str]] = [(0.0, start)]
    while heap:
        d, u = heapq.heappop(heap)
        if d > dist.get(u, INF) + EPS:
            continue
        if u == goal:
            break
        for e in building.adjacency.get(u, []):
            if not robot_can_use_edge(robot, e, config):
                continue
            if e.kind == "elevator":
                fa = building.nodes[u].floor
                fb = building.nodes[e.dst].floor
                ride = config.elevator_board_time + config.elevator_floor_time * abs(fb - fa)
                # The elevator location is fixed at its planning-time state
                # for the whole A* expansion (paper semantics: the elevator
                # "comes to pick you up" from its current floor).
                wait = config.elevator_floor_time * abs(fa - elv_floor)
                cost = wait + ride
            elif e.kind == "stair":
                cost = e.length / max(config.stair_speed, EPS)
            else:
                cost = e.length / max(robot.speed_xy, EPS)
            nd = d + cost
            if nd < dist.get(e.dst, INF) - EPS:
                dist[e.dst] = nd
                prev[e.dst] = (u, e)
                heapq.heappush(heap, (nd, e.dst))
    if goal not in prev and goal != start:
        return None, None, INF, 0.0, elv_floor, []
    nodes: List[str] = [goal]
    edges: List[Edge] = []
    u = goal
    while u != start:
        pu, pe = prev[u]
        nodes.append(pu)
        edges.append(pe)
        u = pu
    nodes.reverse()
    edges.reverse()
    elv_wait = 0.0
    edge_windows: List[Tuple[float, float]] = []
    t = 0.0
    new_elv_floor = elv_floor
    for e in edges:
        if e.kind == "elevator":
            fa = building.nodes[e.src].floor
            fb = building.nodes[e.dst].floor
            # Waiting is evaluated against the planning-time elevator location
            # (fixed for the whole A* expansion), not updated per edge.  The
            # elevator service (come-to-pick-up + ride) is one indivisible
            # occupancy, so the window covers the whole edge.
            wait = config.elevator_floor_time * abs(fa - elv_floor)
            ride = config.elevator_board_time + config.elevator_floor_time * abs(fb - fa)
            elv_wait += wait
            edge_windows.append((t, t + wait + ride))
            t = t + wait + ride
            new_elv_floor = fb
        else:
            dur = e.length / (config.stair_speed if e.kind == "stair" else robot.speed_xy)
            edge_windows.append((t, t + dur))
            t = t + dur
    return nodes, edges, dist.get(goal, 0.0), elv_wait, new_elv_floor, edge_windows


def _occupancy(edge: Edge, t_start: float, t_end: float, robot: Robot, building: BuildingMap, config: SchedulerConfig) -> Tuple[Optional[str], Tuple[float, float]]:
    """Resource occupancy window of one edge (None resource = non-conflicting).
    An elevator service (come-to-pick-up + ride) is one indivisible
    occupancy, so the whole edge window is reserved."""
    if edge.resource and edge.capacity < INF:
        return edge.resource, (t_start, t_end)
    if edge.kind == "floor" and edge.capacity < INF:
        return _edge_resource_key(edge), (t_start, t_end)
    return None, (t_start, t_end)


def greedy_assign(
    building: BuildingMap,
    robots: Sequence[Robot],
    tasks: Sequence[Task],
    config: SchedulerConfig,
) -> Tuple[List[Tuple[Robot, Task]], List[str]]:
    """Greedy assignment: tasks in release order, each task is given to the
    type-matching unassigned robot with the smallest static travel time
    (start -> pickup -> delivery).  No global optimization at all."""
    order = sorted(enumerate(tasks), key=lambda it: (it[1].release_time, it[1].id))
    used: Set[int] = set()
    assigned: List[Tuple[Robot, Task]] = []
    unmatched: List[str] = []
    for _, task in order:
        best: Optional[Tuple[float, int]] = None
        for i, robot in enumerate(robots):
            if i in used:
                continue
            if task.task_type and robot.capability_type != task.task_type:
                continue
            _, _, t1 = _static_dijkstra(building, robot, robot.start_node, task.pickup, config)
            if t1 >= INF:
                continue
            _, _, t2 = _static_dijkstra(building, robot, task.pickup, task.delivery, config)
            if t2 >= INF:
                continue
            total = t1 + t2
            if best is None or total < best[0]:
                best = (total, i)
        if best is None:
            unmatched.append(task.id)
        else:
            used.add(best[1])
            assigned.append((robots[best[1]], task))
    return assigned, unmatched


def munes_schedule(
    building: BuildingMap,
    robots: Sequence[Robot],
    tasks: Sequence[Task],
    config: Optional[SchedulerConfig] = None,
) -> Dict[str, Any]:
    """MuNES baseline: greedy assignment + MuNES elevator-waiting-aware
    planning + global FIFO waiting (first-come-first-served)."""
    config = config or SchedulerConfig()
    assigned, unmatched = greedy_assign(building, robots, tasks, config)
    if not assigned:
        return {
            "assignments": [],
            "unassigned_tasks": list(unmatched),
            "metrics": {"assigned_count": 0, "unassigned_count": len(unmatched), "makespan": 0.0, "sum_completion_time": 0.0, "average_travel_time": 0.0, "total_wait_time": 0.0, "initial_conflict_count": 0, "final_conflict_count": 0, "iterations": 0, "reroutes": 0},
        }

    windows: Dict[str, List[Tuple[float, float]]] = {}
    # 每部电梯真实服务状态：free_at 可派遣时刻、floor 当前停靠层（空驶接驾 + 关门顺延）。
    elv_state: Dict[str, Dict[str, float]] = {}
    elv_floor = 0  # elevator location state (paper: CURR_ELV_FLOOR_FROM_BASE = 0)

    results: List[Dict[str, Any]] = []
    total_wait = 0.0

    for robot, task in assigned:
        n1, e1, arr1, w1, elv_floor, ew1 = _munes_leg(building, robot, robot.start_node, task.pickup, config, elv_floor)
        if n1 is None:
            unmatched.append(task.id)
            continue
        n2, e2, arr2, w2, elv_floor, ew2 = _munes_leg(building, robot, task.pickup, task.delivery, config, elv_floor)
        if n2 is None:
            unmatched.append(task.id)
            continue

        nodes = n1 + n2[1:]
        edges = e1 + e2
        # Nominal per-edge windows from the MuNES arrival profile (leg 2 is
        # offset by the leg-1 completion time).
        leg1_end = ew1[-1][1] if ew1 else 0.0
        nominal: List[Tuple[float, float]] = ew1 + [(s + leg1_end, e + leg1_end) for s, e in ew2]
        # FIFO: shift the remaining profile whenever an occupancy window
        # collides with an already-planned robot (first-come-first-served).
        shifted = nominal[:]
        wait_total = 0.0
        for i, edge in enumerate(edges):
            key, _ = _occupancy(edge, shifted[i][0], shifted[i][1], robot, building, config)
            # First-come-first-served: delay this edge past every already
            # reserved window that overlaps it (the whole elevator service is
            # one indivisible occupancy, so a later robot waits for it to end).
            delay = 0.0
            if key is not None:
                for rs, re in windows.get(key, []):
                    if shifted[i][0] < re - EPS and rs < shifted[i][1] - EPS:
                        delay = max(delay, re - shifted[i][0])
            if delay > 0.0:
                wait_total += delay
                for j in range(i, len(shifted)):
                    ws_j, we_j = shifted[j]
                    shifted[j] = (ws_j + delay, we_j + delay)
            if edge.kind == "elevator" and edge.resource:
                # 容量1串行：状态机在平移后的真实时刻更新轿厢停层与离馆关门顺延。
                st = elv_state.setdefault(
                    edge.resource, {"free_at": 0.0, "floor": float(config.elevator_initial_floor)}
                )
                st["free_at"] = shifted[i][1] + config.elevator_door_close
                st["floor"] = float(building.nodes[edge.dst].floor)
            if key is not None:
                _, occ = _occupancy(edge, shifted[i][0], shifted[i][1], robot, building, config)
                windows.setdefault(key, []).append(occ)
        completion = shifted[-1][1] if shifted else 0.0
        total_wait += wait_total + w1 + w2

        vertical_mode = "none"
        kinds = {e.kind for e in edges}
        if "elevator" in kinds:
            vertical_mode = "elevator"
        elif "stair" in kinds:
            vertical_mode = "stair"
        elif "elevator" in kinds and "stair" in kinds:
            vertical_mode = "mixed"
        results.append(
            {
                "task_id": task.id,
                "robot_id": robot.id,
                "robot_type": robot.robot_type,
                "start_time": 0.0,
                "completion_time": round(completion, 6),
                "travel_time": round(completion, 6),
                "wait_time": round(wait_total + w1 + w2, 6),
                "vertical_mode": vertical_mode,
                "path": nodes,
                "edges": [{"src": e.src, "dst": e.dst, "kind": e.kind, "resource": e.resource} for e in edges],
            }
        )

    completions = [x["completion_time"] for x in results]
    metrics = {
        "assigned_count": len(results),
        "unassigned_count": len(unmatched),
        "makespan": max(completions, default=0.0),
        "sum_completion_time": round(sum(completions), 6),
        "average_travel_time": round(sum(x["travel_time"] for x in results) / len(results), 6) if results else 0.0,
        "total_wait_time": round(total_wait, 6),
        "initial_conflict_count": 0,
        "final_conflict_count": 0,
        "iterations": len(results),
        "reroutes": 0,
    }
    return {"assignments": results, "unassigned_tasks": list(unmatched), "metrics": metrics}


def schedule(
    building: BuildingMap,
    robots: Sequence[Robot],
    tasks: Sequence[Task],
    config: Optional[SchedulerConfig] = None,
) -> Dict[str, Any]:
    return munes_schedule(building, robots, tasks, config)


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
    robots = [Robot(str(row[0]), str(row[1]), str(row[2]), float(row[3] or 1.5), float(row[4] or 0.5), str(row[5] or "general")) for row in robots_rows]
    tasks = [Task(str(row[0]), str(row[1]), str(row[2]), row[3], float(row[4] or 0.0)) for row in tasks_rows]
    return robots, tasks


def parse_payload(payload: Dict[str, Any]) -> Tuple[BuildingMap, List[Robot], List[Task], SchedulerConfig]:
    building = BuildingMap.from_dict(payload["map"])
    robots = [Robot(str(r["id"]), str(r.get("robot_type", "wheeled")), str(r["start_node"]), float(r.get("speed_xy", 1.5)), float(r.get("speed_z", 0.5)), str(r.get("task_type", "general"))) for r in payload.get("robots", [])]
    tasks = [Task(str(t["id"]), str(t["pickup"]), str(t["delivery"]), t.get("task_type"), float(t.get("release_time", 0.0))) for t in payload.get("tasks", [])]
    config = SchedulerConfig(**{k: v for k, v in payload.get("config", {}).items() if k in SchedulerConfig.__dataclass_fields__})
    return building, robots, tasks, config


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
