"""Strip-based route planning baseline — faithful single-floor SRP core.

Faithful reproduction of Shi et al. (ICDE 2023), "Collision-Aware Route
Planning in Warehouses Made Efficient: A Strip-based Framework", adapted to
the unified continuous-time multi-floor interface:

- Strip aggregation (paper Algorithm 1): each regular grid layer is
  aggregated into horizontal strips (one per aisle row).
- Inter-strip planning: shortest strip sequence on the strip graph,
  collision-free (paper Sec. IV).
- Intra-strip planning (paper Sec. V): a route inside a strip is a
  1-D time-space segment (slope +v / -v / 0).  Collision = geometric
  segment intersection (paper Eq. 2 / Eq. 3).  On collision, the route
  stops right before the collision point and waits, then retries
  (paper Algorithm 2 backtracking).
- Slope-based indexing (paper Sec. V-D / Algorithm 3): segments are
  bucketed by slope (+v, -v, 0), each bucket keeps a time-ordered set,
  same-slope candidates are filtered by rotated coordinate.

Cross-floor part is the user-approved simplification: floors are separate
strip graphs connected by direct transfer edges whose cost is the elevator /
stair passage time; cross-floor conflicts use FIFO first-come-first-served
waiting (elevator/stair capacity one).
"""

from __future__ import annotations

import bisect
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
MIN_STEP = 0.05  # minimum time step for backtracking waiting


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
        return cls(nodes, adjacency, resources)

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


@dataclass
class Plan:
    robot: Robot
    task: Task
    candidates: List[RouteCandidate]
    candidate_index: int = 0
    start_time: float = 0.0
    wait_time: float = 0.0

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
        intervals: List[Tuple[str, float, float, int]] = []
        now = self.start_time
        for edge in self.candidate.edges:
            duration = edge_time(edge, self.robot, building, config)
            if edge.resource:
                intervals.append((edge.resource, now, now + duration, edge.capacity))
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
    default_speed_z: float = 0.5
    max_iterations: int = 1000
    epsilon: float = 1e-6
    unmatched_penalty: float = 1e9
    allow_stair_for_non_wheeled: bool = True


def edge_time(edge: Edge, robot: Robot, building: Optional[BuildingMap], config: Optional[SchedulerConfig] = None) -> float:
    config = config or SchedulerConfig()
    if edge.kind == "elevator":
        floors = 1
        if building and edge.src in building.nodes and edge.dst in building.nodes:
            floors = max(1, abs(building.nodes[edge.src].floor - building.nodes[edge.dst].floor))
        return config.elevator_board_time + config.elevator_floor_time * floors
    if edge.kind == "stair":
        if building and edge.src in building.nodes and edge.dst in building.nodes:
            delta_floor = abs(building.nodes[edge.src].floor - building.nodes[edge.dst].floor)
            return config.floor_height * delta_floor / config.stair_speed
        return edge.length / max(config.stair_speed, EPS)
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
    )


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


# =====================================================================
# Strip-based representation (faithful to Shi et al. ICDE 2023)
# =====================================================================

@dataclass(frozen=True)
class Strip:
    floor: int
    row: int          # latitudinal strip = aisle row index
    x0: int           # westernmost grid x
    x1: int           # easternmost grid x


@dataclass
class Segment:
    """1-D time-space segment inside a strip.

    (x0, t0) -> (x1, t1); x is the position along the strip direction.
    Slope +v / -v / 0 (waiting).  A point (x0 == x1 and t0 == t1) is a
    transient crossing event (lane change or strip transfer).
    """
    strip_key: str
    x0: float
    t0: float
    x1: float
    t1: float

    def slope(self) -> float:
        dt = self.t1 - self.t0
        if abs(dt) < EPS:
            return 0.0
        return (self.x1 - self.x0) / dt

    def pos(self, t: float) -> float:
        """Position along the strip at absolute time t."""
        dt = self.t1 - self.t0
        if abs(dt) < EPS:
            return self.x0
        return self.x0 + (self.x1 - self.x0) * (t - self.t0) / dt


def _segments_intersect(a: Segment, b: Segment, eps: float = EPS) -> bool:
    """Geometric intersection of two 1-D time-space segments (paper Eq. 2).

    Open-interval semantics in time: touching only at an endpoint is treated
    as non-colliding (a robot that arrives exactly when another one leaves
    the location is allowed -- "arrival releases").  Both segments are
    affine in t, so their position difference is affine; intersection iff
    the difference changes sign strictly inside the overlap window.
    """
    t_lo = max(a.t0, b.t0)
    t_hi = min(a.t1, b.t1)
    if t_hi - t_lo <= 2 * eps:
        return False
    span = t_hi - t_lo
    probe = max(eps, span / 8.0)
    d1 = a.pos(t_lo + probe) - b.pos(t_lo + probe)
    d2 = a.pos(t_hi - probe) - b.pos(t_hi - probe)
    if abs(d1) < 1e-6 or abs(d2) < 1e-6:
        return True
    return (d1 > 0) != (d2 > 0)


class StripIndex:
    """Slope-based indexing (paper Algorithm 3).

    Segments are bucketed by slope (+v, -v, 0); each bucket is a
    time-ordered list supporting binary search on start time; same-slope
    candidates are filtered by the orthogonal (rotated) coordinate.
    """

    def __init__(self) -> None:
        self._buckets: Dict[str, List[Segment]] = {"pos": [], "neg": [], "zero": []}

    def _bucket(self, seg: Segment) -> str:
        s = seg.slope()
        if s > EPS:
            return "pos"
        if s < -EPS:
            return "neg"
        return "zero"

    def insert(self, seg: Segment) -> None:
        bucket = self._buckets[self._bucket(seg)]
        bisect.insort(bucket, seg, key=lambda s: (s.t0, s.x0))

    def _rotated(self, seg: Segment) -> Tuple[float, float]:
        """Orthogonal coordinate after rotating the time-space plane by the
        segment slope (paper Eq. 4): for slope +v rotate by -pi/4, for
        slope -v by +pi/4; for slope 0 the orthogonal axis is the spatial one.
        """
        s = seg.slope()
        if abs(s) < EPS:
            return seg.x0, seg.x1
        if s > 0:
            c = math.sqrt(0.5)
            return (seg.x0 - seg.t0) * c, (seg.x1 - seg.t1) * c
        c = math.sqrt(0.5)
        return (seg.x0 + seg.t0) * c, (seg.x1 + seg.t1) * c

    def candidate_segments(self, seg: Segment) -> List[Segment]:
        """All stored segments that may intersect `seg` (time-overlap first,
        then same-slope spatial filtering via rotated coordinate)."""
        out: List[Segment] = []
        my_bucket = self._bucket(seg)
        for name, bucket in self._buckets.items():
            if not bucket:
                continue
            if name == my_bucket:
                o0, o1 = self._rotated(seg)
                lo = min(o0, o1) - 1e-6
                hi = max(o0, o1) + 1e-6
                for other in bucket:
                    if other.t1 < seg.t0 - EPS or seg.t1 < other.t0 - EPS:
                        continue
                    p0, p1 = self._rotated(other)
                    if max(p0, p1) < lo or min(p0, p1) > hi:
                        continue
                    out.append(other)
            else:
                for other in bucket:
                    if other.t1 < seg.t0 - EPS or seg.t1 < other.t0 - EPS:
                        continue
                    out.append(other)
        return out

    def earliest_collision(self, seg: Segment) -> Optional[Tuple[float, List[Segment]]]:
        tc: Optional[float] = None
        colliders: List[Segment] = []
        for other in self.candidate_segments(seg):
            if _segments_intersect(seg, other):
                c = max(seg.t0, other.t0)
                if tc is None or c < tc:
                    tc = c
                    colliders = [other]
                elif abs(c - tc) < EPS:
                    colliders.append(other)
        if tc is None:
            return None
        return tc, colliders


class GridLayer:
    """A regular grid floor: nodes on an equally-spaced axis-aligned grid
    with 4-neighbour floor edges.  Detected from the BuildingMap; strips are
    built per row (paper Algorithm 1, aisle rows).  Coordinates are physical
    metres; the grid spacing is derived from the map."""

    def __init__(self, floor: int, building: BuildingMap) -> None:
        self.floor = floor
        by_xy: Dict[Tuple[float, float], str] = {}
        for node in building.nodes.values():
            if node.floor != floor:
                continue
            by_xy[(float(node.x), float(node.y))] = node.id
        if not by_xy:
            raise ValueError(f"floor {floor} has no nodes")
        xs = sorted({p[0] for p in by_xy})
        ys = sorted({p[1] for p in by_xy})

        def spacing(values: List[float], axis: str) -> float:
            if len(values) < 2:
                return 1.0
            d = values[1] - values[0]
            if d <= 0 or any(abs(values[i + 1] - values[i] - d) > 1e-6 for i in range(len(values) - 1)):
                raise ValueError(f"strip baseline requires an equally-spaced grid on floor {floor} ({axis} axis)")
            return d

        self.dx = spacing(xs, "x")
        self.dy = spacing(ys, "y")
        self.xs = xs
        self.ys = ys
        self.x0, self.x1 = xs[0], xs[-1]
        self.y0, self.y1 = ys[0], ys[-1]
        self.by_xy = by_xy
        for (x, y), nid in by_xy.items():
            for ox, oy in ((x + self.dx, y), (x, y + self.dy)):
                if (ox, oy) in by_xy:
                    oid = by_xy[(ox, oy)]
                    if not any(e.dst == oid and e.kind == "floor" for e in building.adjacency.get(nid, [])):
                        raise ValueError(f"floor {floor}: grid adjacency missing {nid}->{oid}")

    def _nearest(self, values: List[float], v: float) -> float:
        return min(values, key=lambda a: abs(a - v))

    def node(self, x: float, y: float) -> str:
        nx = self._nearest(self.xs, x)
        ny = self._nearest(self.ys, y)
        if (nx, ny) not in self.by_xy:
            raise ValueError(f"grid node ({nx},{ny}) not found on floor {self.floor}")
        return self.by_xy[(nx, ny)]

    def strips(self) -> Dict[str, Strip]:
        return {f"f{self.floor}:r{int(round((y - self.y0) / self.dy))}": Strip(self.floor, int(round((y - self.y0) / self.dy)), self.x0, self.x1) for y in self.ys}


class StripContext:
    """Shared state for online strip-based route planning: per-strip segment
    indexes (all previously planned routes) and elevator/stair reservations."""

    def __init__(self, building: BuildingMap, layers: Dict[int, GridLayer], config: SchedulerConfig) -> None:
        self.building = building
        self.layers = layers
        self.config = config
        self.index: Dict[str, StripIndex] = {}
        for floor, layer in layers.items():
            for key in layer.strips():
                self.index[key] = StripIndex()
        self.reservations: Dict[str, List[Tuple[float, float]]] = {}
        # 每部电梯的真实服务状态：free_at 可派遣时刻、floor 当前停靠层。
        self.elv: Dict[str, Dict[str, float]] = {}

    def _strip_index(self, floor: int, row: int) -> StripIndex:
        return self.index[f"f{floor}:r{row}"]

    def elevator_free(
        self, resource: str, start: float, duration: float,
        from_floor: int, to_floor: int,
    ) -> Tuple[float, float]:
        """真实电梯服务：轿厢停层递推 + 空驶接驾 + 载客 + 离馆关门顺延。

        First-come-first-served。返回 (ready, wait)。空驶接驾
        t_pickup=free_at+|from-floor|*per_floor（与步行并行），ready=max(t_pickup,start)；
        载客 duration；离馆关门 door_close 顺延给后续请求（在线逐个规划，每趟计入）。
        """
        st = self.elv.setdefault(
            resource, {"free_at": 0.0, "floor": float(self.config.elevator_initial_floor)}
        )
        pickup = st["free_at"] + abs(from_floor - st["floor"]) * self.config.elevator_floor_time
        ready = max(pickup, start)
        alight = ready + duration
        st["free_at"] = alight + self.config.elevator_door_close
        st["floor"] = float(to_floor)
        self.reservations.setdefault(resource, []).append((ready, alight))
        return ready, ready - start

    def plan_layer_move(
        self, floor: int, row: int, x0: float, t0: float, x1: float, t1: float
    ) -> Tuple[List[Segment], float, float]:
        """Plan a straight move inside a strip with waiting.

        Waiting strategy (paper Algorithm 2, simplified): try to move
        directly; on the earliest collision with an already-planned segment,
        delay the whole departure (first-come-first-served) until the
        delayed segment no longer intersects any existing segment.  The
        waiting is recorded in the returned arrival time.  This is the
        user-approved "waiting method": later robots yield to earlier ones.
        Returns (segments, arrival, wait)."""
        key = f"f{floor}:r{row}"
        index = self._strip_index(floor, row)
        wait = 0.0
        guard = 0
        while guard < self.config.max_iterations:
            guard += 1
            seg = Segment(key, x0, t0 + wait, x1, t1 + wait)
            coll = index.earliest_collision(seg)
            if coll is None:
                index.insert(seg)
                return [seg], t1 + wait, wait
            blocker_end = max(s.t1 for s in coll[1])
            candidate = blocker_end - t0 + MIN_STEP
            wait = candidate if candidate > wait + MIN_STEP else wait + MIN_STEP
        raise RuntimeError(f"strip waiting did not converge on {key}")

    def plan_layer_path(self, floor: int, x0: float, y0: float, t0: float, x1: float, y1: float) -> Tuple[List[Segment], float, List[str], List[Edge], float]:
        """Plan a shortest path on one floor using the strip graph: monotone
        row sequence (strip-graph shortest path) with greedy lane changes,
        each row-internal move planned as 1-D segments.  Rows are indexed by
        grid row (0-based); x coordinates are physical metres.
        Returns (segments, arrival, nodes, edges, wait)."""
        layer = self.layers[floor]
        ri0 = int(round((y0 - layer.y0) / layer.dy))
        ri1 = int(round((y1 - layer.y0) / layer.dy))
        if ri0 == ri1:
            segs, arr, wait = self.plan_layer_move(floor, ri0, float(x0), t0, float(x1), t0 + abs(x1 - x0) / self.config.default_speed_xy)
            nodes = _manhattan_nodes(layer, x0, y0, x1, y1)
            edges = [Edge(a, b, 1.0, "floor") for a, b in zip(nodes[:-1], nodes[1:])]
            return segs, arr, nodes, edges, wait
        step = 1 if ri1 > ri0 else -1
        segs_all: List[Segment] = []
        nodes: List[str] = []
        edges: List[Edge] = []
        wait_total = 0.0
        # Move along x inside the starting row first (monotone).
        segs, arr, wait = self.plan_layer_move(floor, ri0, float(x0), t0, float(x1), t0 + abs(x1 - x0) / self.config.default_speed_xy)
        segs_all += segs
        wait_total += wait
        nodes = _manhattan_nodes(layer, x0, y0, x1, y0)
        edges = [Edge(a, b, 1.0, "floor") for a, b in zip(nodes[:-1], nodes[1:])]
        cur_t = arr
        ri = ri0
        while ri != ri1:
            next_ri = ri + step
            # Lane change: transient segment in both strips at (x1, depart)
            # covering the transversal move of one grid row (duration dy/v).
            # The departure time advances by `dur` each row (nominal), and
            # is pushed further only when a collision is detected.
            key_a, key_b = f"f{floor}:r{ri}", f"f{floor}:r{next_ri}"
            dur = layer.dy / self.config.default_speed_xy
            depart = cur_t
            initial = cur_t
            point = Segment(key_a, float(x1), depart, float(x1), depart + dur)
            for key in (key_a, key_b):
                idx = self.index[key]
                c = idx.earliest_collision(point)
                if c is not None:
                    blocker_end = max(s.t1 for s in c[1])
                    depart = max(depart + dur, blocker_end + MIN_STEP)
                    point = Segment(key, float(x1), depart, float(x1), depart + dur)
            wait_total += depart - initial
            for key in (key_a, key_b):
                self.index[key].insert(point)
            segs_all.append(point)
            cur_t = depart + dur
            a = layer.node(x1, layer.y0 + ri * layer.dy)
            b = layer.node(x1, layer.y0 + next_ri * layer.dy)
            nodes.append(b)
            edges.append(Edge(a, b, 1.0, "floor"))
            ri = next_ri
        return segs_all, cur_t, nodes, edges, wait_total


def _build_layers(building: BuildingMap) -> Dict[int, GridLayer]:
    floors = sorted({n.floor for n in building.nodes.values()})
    layers: Dict[int, GridLayer] = {}
    for f in floors:
        layers[f] = GridLayer(f, building)
    return layers


def _vertical_resources(building: BuildingMap) -> List[Tuple[str, str, Set[int], Tuple[float, float]]]:
    """All vertical resources as (kind, resource, covered floors, anchor xy)."""
    by_resource: Dict[str, List[Any]] = {}
    for nid, node in building.nodes.items():
        for edge in building.adjacency.get(nid, []):
            if edge.kind in {"elevator", "stair"} and edge.resource:
                dst = building.nodes[edge.dst]
                rec = by_resource.setdefault(edge.resource, [edge.kind, set(), None])
                rec[1].update((node.floor, dst.floor))
                if rec[2] is None:
                    rec[2] = (node.x, node.y)
    return [(rec[0], res, rec[1], rec[2]) for res, rec in by_resource.items()]


def _layer_manhattan(config: SchedulerConfig, xa: int, ya: int, xb: int, yb: int) -> float:
    return (abs(xa - xb) + abs(ya - yb)) / max(config.default_speed_xy, EPS)


def _manhattan_nodes(layer: GridLayer, xa: float, ya: float, xb: float, yb: float) -> List[str]:
    path: List[str] = []
    cx, cy = xa, ya
    path.append(layer.node(cx, cy))
    sx = 1 if xb > xa else -1
    sy = 1 if yb > ya else -1
    guard = 0
    while (abs(cx - xb) > 1e-9 or abs(cy - yb) > 1e-9) and guard < 10000:
        guard += 1
        if abs(cx - xb) > 1e-9:
            cx += sx * layer.dx
        else:
            cy += sy * layer.dy
        path.append(layer.node(cx, cy))
    if guard >= 10000:
        raise RuntimeError(f"manhattan node walk did not converge from ({xa},{ya}) to ({xb},{yb}) on floor {layer.floor}")
    return path


def _static_leg(
    layers: Dict[int, GridLayer],
    vertical: List[Tuple[str, str, Set[int], Tuple[float, float]]],
    config: SchedulerConfig,
    robot: Robot,
    fa: int, xa: int, ya: int,
    fb: int, xb: int, yb: int,
) -> Optional[Tuple[float, List[str], List[Edge], str]]:
    """Static (collision-free) shortest leg; vertical transfers are one-ride."""
    if fa == fb:
        path = _manhattan_nodes(layers[fa], xa, ya, xb, yb)
        edges = [Edge(a, b, 1.0, "floor") for a, b in zip(path[:-1], path[1:])]
        return _layer_manhattan(config, xa, ya, xb, yb), path, edges, "none"
    best: Optional[Tuple[float, List[str], List[Edge], str]] = None
    for kind, resource, floors, anchor in vertical:
        if fa not in floors or fb not in floors:
            continue
        if not robot_can_use_edge(robot, Edge("", "", 1.0, kind), config):
            continue
        ax, ay = float(anchor[0]), float(anchor[1])
        if kind == "elevator":
            vdur = config.elevator_board_time + config.elevator_floor_time * abs(fa - fb)
        else:
            vdur = config.floor_height * abs(fa - fb) / config.stair_speed
        dur = _layer_manhattan(config, xa, ya, ax, ay) + vdur + _layer_manhattan(config, ax, ay, xb, yb)
        if best is None or dur < best[0]:
            a_node = layers[fa].node(ax, ay)
            b_node = layers[fb].node(ax, ay)
            edge = Edge(a_node, b_node, 1.0, kind, 1, resource)
            # start-layer approach + transfer edge + arrival anchor node
            path = _manhattan_nodes(layers[fa], xa, ya, ax, ay) + [b_node]
            best = (dur, path, [edge], kind)
    return best


def _static_route(building: BuildingMap, robot: Robot, task: Task, config: SchedulerConfig, layers: Dict[int, GridLayer]) -> Optional[RouteCandidate]:
    """Collision-free geometric shortest path with one-ride elevator/stair
    transfers (assignment cost)."""

    def xy(node_id: str) -> Tuple[int, float, float]:
        node = building.nodes[node_id]
        return node.floor, float(node.x), float(node.y)

    f0, x0, y0 = xy(robot.start_node)
    fp, xp, yp = xy(task.pickup)
    fd, xd, yd = xy(task.delivery)
    vertical = _vertical_resources(building)

    leg1 = _static_leg(layers, vertical, config, robot, f0, x0, y0, fp, xp, yp)
    if leg1 is None:
        return None
    leg2 = _static_leg(layers, vertical, config, robot, fp, xp, yp, fd, xd, yd)
    if leg2 is None:
        return None
    dur1, nodes1, edges1, mode1 = leg1
    dur2, nodes2, edges2, mode2 = leg2
    nodes = nodes1 + nodes2[1:]
    edges = edges1 + edges2
    mode = mode1 if mode1 == mode2 else ("mixed" if mode1 != "none" and mode2 != "none" else mode1 if mode1 != "none" else mode2)
    return RouteCandidate(nodes, edges, dur1 + dur2, mode)


def build_initial_plans(
    building: BuildingMap,
    robots: Sequence[Robot],
    tasks: Sequence[Task],
    config: SchedulerConfig,
) -> Tuple[List[Plan], List[str]]:
    """Hungarian assignment with capability-type matching (one robot per
    task per batch); cost = collision-free shortest travel time."""
    layers = _build_layers(building)
    candidates: Dict[Tuple[int, int], Optional[RouteCandidate]] = {}
    for i, robot in enumerate(robots):
        for j, task in enumerate(tasks):
            if task.task_type and robot.capability_type != task.task_type:
                candidates[(i, j)] = None
            else:
                candidates[(i, j)] = _static_route(building, robot, task, config, layers)
    size = max(len(robots), len(tasks))
    matrix = [[0.0 for _ in range(size)] for _ in range(size)]
    for i in range(size):
        for j in range(size):
            if i < len(robots) and j < len(tasks):
                values = candidates[(i, j)]
                matrix[i][j] = values.travel_time if values else config.unmatched_penalty
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
            plans.append(Plan(robots[i], tasks[j], [candidates[(i, j)]]))
            assigned_tasks.add(j)
    unmatched.extend(tasks[j].id for j in range(len(tasks)) if j not in assigned_tasks)
    return plans, unmatched


def _interval_overlap(a_start: float, a_end: float, b_start: float, b_end: float) -> bool:
    return a_start < b_end - EPS and b_start < a_end - EPS


def detect_conflicts(plans: Sequence[Plan], building: BuildingMap, config: Optional[SchedulerConfig] = None) -> List[Conflict]:
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


def strip_schedule(
    building: BuildingMap,
    robots: Sequence[Robot],
    tasks: Sequence[Task],
    config: Optional[SchedulerConfig] = None,
) -> Dict[str, Any]:
    config = config or SchedulerConfig()
    plans, unmatched = build_initial_plans(building, robots, tasks, config)
    if not plans:
        return {"assignments": [], "unassigned_tasks": list(unmatched), "metrics": {"assigned_count": 0, "unassigned_count": len(unmatched), "makespan": 0.0, "sum_completion_time": 0.0, "average_travel_time": 0.0, "total_wait_time": 0.0, "initial_conflict_count": 0, "final_conflict_count": 0, "iterations": 0, "reroutes": 0}}
    layers = _build_layers(building)
    ctx = StripContext(building, layers, config)
    results: List[Dict[str, Any]] = []
    total_wait = 0.0
    for plan in sorted(plans, key=lambda p: (p.task.release_time, p.task.id)):
        route, wait = _plan_online(ctx, plan.robot, plan.task, config)
        plan.candidates = [route]
        plan.start_time = 0.0
        plan.wait_time = wait
        total_wait += wait
        results.append(
            {
                "task_id": plan.task.id,
                "robot_id": plan.robot.id,
                "robot_type": plan.robot.robot_type,
                "start_time": round(0.0, 6),
                "completion_time": round(route.travel_time, 6),
                "travel_time": round(route.travel_time, 6),
                "wait_time": round(wait, 6),
                "vertical_mode": route.vertical_mode,
                "path": route.nodes,
                "edges": [{"src": e.src, "dst": e.dst, "kind": e.kind, "resource": e.resource} for e in route.edges],
            }
        )
    completion = [x["completion_time"] for x in results]
    metrics = {
        "assigned_count": len(results),
        "unassigned_count": len(unmatched),
        "makespan": max(completion, default=0.0),
        "sum_completion_time": round(sum(completion), 6),
        "average_travel_time": round(sum(x["travel_time"] for x in results) / len(results), 6) if results else 0.0,
        "total_wait_time": round(total_wait, 6),
        "initial_conflict_count": 0,
        "final_conflict_count": 0,
        "iterations": len(results),
        "reroutes": 0,
    }
    return {"assignments": results, "unassigned_tasks": list(unmatched), "metrics": metrics}


def _plan_online(ctx: StripContext, robot: Robot, task: Task, config: SchedulerConfig) -> Tuple[RouteCandidate, float]:
    """Online strip-based route planning for one robot-task pair.

    The route is a sequence of: intra-strip 1-D segments (with backtracking
    waiting against previously planned routes), strip-transfer points, and
    elevator/stair direct transfers with FIFO waiting.
    """
    building = ctx.building
    layers = ctx.layers
    nodes: List[str] = []
    edges: List[Edge] = []
    all_segments: List[Segment] = []
    t = 0.0
    wait_total = 0.0

    def to_xy(node_id: str) -> Tuple[int, float, float]:
        node = building.nodes[node_id]
        return node.floor, float(node.x), float(node.y)

    f0, x0, y0 = to_xy(robot.start_node)
    fp, xp, yp = to_xy(task.pickup)
    fd, xd, yd = to_xy(task.delivery)

    vertical = _vertical_resources(building)  # (kind, resource, floors, anchor xy)

    def elevator_earliest(resource: str, start: float, duration: float) -> float:
        """Earliest free start time given current reservations (read-only)."""
        reserved = sorted(ctx.reservations.get(resource, []))
        t = start
        while True:
            ok = True
            for rs, re in reserved:
                if t < re - EPS and rs < t + duration - EPS:
                    t = re
                    ok = False
                    break
            if ok:
                break
        return t

    def transfer(f_from: int, f_to: int, xa: float, ya: float, ta: float) -> Tuple[float, List[str], List[Edge], float, str, Tuple[float, float]]:
        """Walk (intra-strip) to the best feasible elevator/stair landing,
        then a one-ride transfer with FIFO waiting.  Returns
        (arrival, nodes, edges, wait, kind, anchor xy)."""
        best: Optional[Tuple[float, str, str, float, Tuple[float, float]]] = None
        for kind, resource, floors, anchor in vertical:
            if f_from not in floors or f_to not in floors:
                continue
            if not robot_can_use_edge(robot, Edge("", "", 1.0, kind), config):
                continue
            ax, ay = float(anchor[0]), float(anchor[1])
            # Snap the landing to the nearest grid node (elevator/stair
            # shafts may sit off the regular grid).
            landing = building.nodes[layers[f_from].node(ax, ay)]
            ax, ay = float(landing.x), float(landing.y)
            if kind == "elevator":
                vdur = config.elevator_board_time + config.elevator_floor_time * abs(f_from - f_to)
            else:
                vdur = config.floor_height * abs(f_from - f_to) / config.stair_speed
            t_est = ta + _layer_manhattan(config, xa, ya, ax, ay)
            est = elevator_earliest(resource, t_est, vdur) + vdur
            if best is None or est < best[0]:
                best = (est, kind, resource, vdur, (ax, ay))
        if best is None:
            raise ValueError(f"no feasible vertical transfer {f_from}->{f_to} for robot {robot.id}")
        _, kind, resource, vdur, (ax, ay) = best
        segs, t_anchor, nids, edgs, w_layer = ctx.plan_layer_path(f_from, xa, ya, ta, ax, ay)
        start_ok, w = ctx.elevator_free(resource, t_anchor, vdur, f_from, f_to)
        a_node = layers[f_from].node(ax, ay)
        b_node = layers[f_to].node(ax, ay)
        return start_ok + vdur, nids + [b_node], edgs + [Edge(a_node, b_node, 1.0, kind, 1, resource)], w + w_layer, kind, (ax, ay)

    # start -> pickup
    nodes.append(layers[f0].node(x0, y0))
    cur_f, cur_x, cur_y = f0, x0, y0
    if f0 != fp:
        t, nids, edgs, w, _, (cur_x, cur_y) = transfer(f0, fp, cur_x, cur_y, t)
        cur_f = fp
        nodes.extend(nids[1:])
        edges.extend(edgs)
        wait_total += w
    segs, arr, nids, edgs, w_layer = ctx.plan_layer_path(cur_f, cur_x, cur_y, t, xp, yp)
    nodes.extend(nids[1:])
    edges.extend(edgs)
    wait_total += w_layer
    t = arr

    # pickup -> delivery
    cur_f, cur_x, cur_y = fp, xp, yp
    if fp != fd:
        t, nids, edgs, w, _, (cur_x, cur_y) = transfer(fp, fd, cur_x, cur_y, t)
        cur_f = fd
        nodes.extend(nids[1:])
        edges.extend(edgs)
        wait_total += w
    segs, arr, nids, edgs, w_layer = ctx.plan_layer_path(cur_f, cur_x, cur_y, t, xd, yd)
    nodes.extend(nids[1:])
    edges.extend(edgs)
    wait_total += w_layer
    t = arr

    vertical_mode = "none"
    kinds = {e.kind for e in edges}
    if "elevator" in kinds:
        vertical_mode = "elevator"
    elif "stair" in kinds:
        vertical_mode = "stair"
    elif "elevator" in kinds and "stair" in kinds:
        vertical_mode = "mixed"
    route = RouteCandidate(nodes, edges, t, vertical_mode)
    return route, wait_total


def load_json(path: str | Path) -> Dict[str, Any]:
    return json.loads(Path(path).read_text(encoding="utf-8"))


def read_sqlite_inputs(db_path: str | Path) -> Tuple[List[Robot], List[Task]]:
    connection = sqlite3.connect(str(db_path))
    try:
        robots_rows = connection.execute(
            "SELECT id, robot_type, start_node, speed_xy, speed_z FROM robots"
        ).fetchall()
        tasks_rows = connection.execute(
            "SELECT id, pickup, delivery, task_type, release_time FROM tasks WHERE status IS NULL OR status='pending'"
        ).fetchall()
    finally:
        connection.close()
    robots = [Robot(str(row[0]), str(row[1]), str(row[2]), float(row[3] or 1.5), float(row[4] or 0.5)) for row in robots_rows]
    tasks = [Task(str(row[0]), str(row[1]), str(row[2]), row[3], float(row[4] or 0.0)) for row in tasks_rows]
    return robots, tasks


def parse_payload(payload: Dict[str, Any]) -> Tuple[BuildingMap, List[Robot], List[Task], SchedulerConfig]:
    building = BuildingMap.from_dict(payload["map"])
    robots = [
        Robot(
            str(r["id"]),
            str(r.get("robot_type", "wheeled")),
            str(r["start_node"]),
            float(r.get("speed_xy", 1.5)),
            float(r.get("speed_z", 0.5)),
            str(r.get("task_type", r.get("capability_type", "general"))),
        )
        for r in payload.get("robots", [])
    ]
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
        robots.append({"id": f"r{index}", "robot_type": types[index % len(types)], "start_node": rng.choice(node_ids), "speed_xy": float(config_data.get("speed_xy", 1.5)), "speed_z": float(config_data.get("speed_z", 0.5)), "task_type": "general"})
    tasks: List[Dict[str, Any]] = []
    for index in range(int(config_data.get("tasks", robot_count))):
        tasks.append({"id": f"t{index}", "pickup": rng.choice(node_ids), "delivery": rng.choice(node_ids), "task_type": "general"})
    return {"map": {"nodes": nodes, "edges": edges, "resources": resources}, "robots": robots, "tasks": tasks, "config": config_data}


# keep `schedule` name for run.py
schedule = strip_schedule
