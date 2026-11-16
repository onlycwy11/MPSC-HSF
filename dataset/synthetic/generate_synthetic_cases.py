#!/usr/bin/env python3
"""Create the free-combination ICDE simulation dataset v3.

Unlike v2 (which fixes one dimension per group, control-variable style),
v3 is the FULL CARTESIAN grid the user wants:

    robots   : 8 levels  = 1, 16, 64, 256, 512, 1024, 2048, 4096
    task load: 4 ratios  = 0.5, 1.0, 1.5, 2.0   ->  tasks = max(1, round(R * ratio))
    floors   : 7 levels  = 2, 4, 8, 16, 32, 64, 100
    seeds    : 3         = 17, 29, 43

    total    : 8 * 4 * 7 * 3 = 672 cases

Map family rule (kept from v2's implied usage, disk-friendly):
    floors <= 8  -> L (40x40 grid, finer planar detail)
    floors  > 8  -> M (32x32 grid, controls file size on tall buildings)

Determinism: same code + same config + same seed -> identical case on any machine.
"""

from __future__ import annotations

import argparse
import json
import math
import random
from pathlib import Path
from typing import Any, Dict, Iterable, List, Sequence, Tuple


ROOT = Path(__file__).resolve().parent
SEEDS = (17, 29, 43)
ROBOT_LEVELS = [1, 16, 64, 256, 512, 1024, 2048, 4096]
LOAD_RATIOS = (0.5, 1.0, 1.5, 2.0)
FLOORS = [2, 4, 8, 16, 32, 64, 100]


def write_json(path: Path, value: Dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def facility_counts(floors: int, profile: str) -> Tuple[int, int]:
    balanced = max(1, math.ceil(floors / 8))
    if profile == "balanced":
        return balanced, balanced
    if profile == "elevator_scarce":
        return 1, balanced
    if profile == "stair_scarce":
        return balanced, 1
    raise ValueError(f"unknown facility profile: {profile}")


def connector_positions(width: int, height: int, count: int, offset: int = 0) -> List[Tuple[int, int]]:
    candidates: List[Tuple[int, int]] = list((
        (1, 1), (width - 2, height - 2), (width // 2, height // 2),
        (width - 2, 1), (1, height - 2), (width // 2, 1),
        (width // 2, height - 2), (1, height // 2), (width - 2, height // 2),
        (width // 4, height // 4), (3 * width // 4, 3 * height // 4),
        (width // 4, 3 * height // 4), (3 * width // 4, height // 4),
        (width // 2, height // 4), (width // 2, 3 * height // 4),
    ))
    candidates.extend((x, y) for x in range(1, width - 1) for y in range(1, height - 1))
    unique: List[Tuple[int, int]] = []
    for item in candidates:
        if item not in unique:
            unique.append(item)
    if offset + count > len(unique):
        raise ValueError(f"map {width}x{height} cannot place {count} connectors")
    return [unique[offset + i] for i in range(count)]


def node_id(floor: int, x: int, y: int) -> str:
    return f"f{floor}_x{x}_y{y}"


def make_map(floors: int, width: int, height: int, profile: str, grid_spacing: float = 2.5) -> Dict[str, Any]:
    elevators, stairs = facility_counts(floors, profile)
    elevator_xy = connector_positions(width, height, elevators, 0)
    stair_xy = connector_positions(width, height, stairs, elevators)
    elevator_set = set(elevator_xy)
    stair_set = set(stair_xy)
    if elevator_set & stair_set:
        raise ValueError("elevator and stair connector locations overlap")

    nodes: List[Dict[str, Any]] = []
    by_id: Dict[str, Dict[str, Any]] = {}
    for floor in range(floors):
        for x in range(width):
            for y in range(height):
                xy = (x, y)
                node_type = "elevator_lobby" if xy in elevator_set else "stair_lobby" if xy in stair_set else ("room" if 1 <= x < width - 1 and 1 <= y < height - 1 else "corridor")
                item = {"id": node_id(floor, x, y), "floor": floor, "x": x * grid_spacing, "y": y * grid_spacing, "z": floor * 3.5, "node_type": node_type}
                nodes.append(item)
                by_id[item["id"]] = item

    edges: List[Dict[str, Any]] = []
    for floor in range(floors):
        for x in range(width):
            for y in range(height):
                src = node_id(floor, x, y)
                for dx, dy in ((1, 0), (0, 1)):
                    nx, ny = x + dx, y + dy
                    if nx >= width or ny >= height:
                        continue
                    dst = node_id(floor, nx, ny)
                    edge = {"src": src, "dst": dst, "length": grid_spacing, "kind": "floor", "resource": None}
                    reverse = {"src": dst, "dst": src, "length": grid_spacing, "kind": "floor", "resource": None}
                    edges.extend((edge, reverse))

    resources: Dict[str, int] = {}
    resource_metadata: Dict[str, Dict[str, Any]] = {}
    for index, (x, y) in enumerate(elevator_xy):
        resource = f"elevator-{index}"
        resources[resource] = 1
        resource_metadata[resource] = {"kind": "elevator", "capacity": 1, "x": x, "y": y}
        for floor in range(floors - 1):
            a, b = node_id(floor, x, y), node_id(floor + 1, x, y)
            edges.extend((
                {"src": a, "dst": b, "length": 3.5, "kind": "elevator", "resource": resource},
                {"src": b, "dst": a, "length": 3.5, "kind": "elevator", "resource": resource},
            ))
    for index, (x, y) in enumerate(stair_xy):
        resource = f"stair-{index}"
        resources[resource] = 1
        resource_metadata[resource] = {"kind": "stair", "capacity": 1, "x": x, "y": y}
        for floor in range(floors - 1):
            a, b = node_id(floor, x, y), node_id(floor + 1, x, y)
            edges.extend((
                {"src": a, "dst": b, "length": 3.5, "kind": "stair", "resource": resource},
                {"src": b, "dst": a, "length": 3.5, "kind": "stair", "resource": resource},
            ))
    return {
        "nodes": nodes,
        "edges": edges,
        "resources": resources,
        "resource_metadata": resource_metadata,
        "meta": {"floors": floors, "width": width, "height": height, "grid_spacing_m": grid_spacing, "floor_height_m": 3.5, "facility_profile": profile, "elevators": elevators, "stairs": stairs},
    }


def floor_weights(floors: int, distribution: str) -> List[float]:
    if distribution in ("uniform", "floor_balanced"):
        raw = [1.0] * floors
    elif distribution == "entrance_hotspot":
        raw = [3.0 if f in (0, floors - 1) else 1.0 for f in range(floors)]
    elif distribution == "mid_floor_hotspot":
        center = (floors - 1) / 2
        raw = [1.0 + max(0.0, 4.0 - abs(f - center)) for f in range(floors)]
    else:
        raw = [1.0] * floors
    total = sum(raw)
    return [x / total for x in raw]


def weighted_choice(rng: random.Random, values: Sequence[int], weights: Sequence[float]) -> int:
    point = rng.random()
    cumulative = 0.0
    for value, weight in zip(values, weights):
        cumulative += weight
        if point <= cumulative:
            return value
    return values[-1]


def make_case(spec: Dict[str, Any], map_data: Dict[str, Any]) -> Dict[str, Any]:
    rng = random.Random(int(spec["seed"]))
    nodes = map_data["nodes"]
    by_floor: Dict[int, List[Dict[str, Any]]] = {}
    task_nodes = [node for node in nodes if node["node_type"] not in {"elevator_lobby", "stair_lobby"}]
    for node in task_nodes:
        by_floor.setdefault(int(node["floor"]), []).append(node)
    floors = sorted(by_floor)
    task_distribution = spec.get("task_distribution", "uniform")
    robot_distribution = spec.get("robot_distribution", "floor_balanced")
    floor_prob = floor_weights(len(floors), task_distribution)
    robot_floor_prob = floor_weights(len(floors), "uniform")

    def choose_node(distribution: str) -> Dict[str, Any]:
        if distribution == "floor_balanced":
            floor = floors[(choose_node.robot_index) % len(floors)]
            choose_node.robot_index += 1
        elif distribution == "entrance":
            floor = 0 if rng.random() < 0.5 else weighted_choice(rng, floors, robot_floor_prob)
        else:
            floor = weighted_choice(rng, floors, robot_floor_prob)
        return rng.choice(by_floor[floor])

    choose_node.robot_index = 0  # type: ignore[attr-defined]
    robots_count = int(spec["robots"])
    tasks_count = int(spec["tasks"])
    wheeled_count = round(robots_count * float(spec.get("wheeled_fraction", 0.75)))
    capability_types = list(spec.get("capability_types", ["general"]))
    robots: List[Dict[str, Any]] = []
    for index in range(robots_count):
        start = choose_node(robot_distribution)
        robots.append({
            "id": f"r{index}",
            "robot_type": "wheeled" if index < wheeled_count else "legged",
            "task_type": capability_types[index % len(capability_types)],
            "start_node": start["id"],
            "speed_xy": 1.5,
            "speed_z": 0.5,
            "available_at": 0.0,
        })

    cluster = rng.sample(task_nodes, min(4, len(task_nodes))) if task_distribution == "clustered_rooms" else []

    def task_endpoint() -> Dict[str, Any]:
        if cluster and rng.random() < 0.7:
            return rng.choice(cluster)
        floor = weighted_choice(rng, floors, floor_prob)
        return rng.choice(by_floor[floor])

    cross_floor_ratio = float(spec.get("cross_floor_ratio", 0.5))
    tasks: List[Dict[str, Any]] = []
    for index in range(tasks_count):
        pickup = task_endpoint()
        if len(floors) > 1 and rng.random() < cross_floor_ratio:
            other_floors = [f for f in floors if f != pickup["floor"]]
            delivery = rng.choice(by_floor[rng.choice(other_floors)])
        else:
            delivery = task_endpoint()
        while delivery["id"] == pickup["id"]:
            delivery = task_endpoint()
        tasks.append({
            "id": f"t{index}",
            "pickup": pickup["id"],
            "delivery": delivery["id"],
            "pickup_floor": pickup["floor"],
            "delivery_floor": delivery["floor"],
            "task_type": capability_types[index % len(capability_types)],
            "release_time": 0.0,
        })
    return {
        "schema_version": "multifloor-scheduler-generic-v2",
        "case": spec,
        "map": map_data,
        "robots": robots,
        "tasks": tasks,
        "config": {"speed_xy": 1.5, "stair_speed": 0.5, "floor_height": 3.5, "elevator_board_time": 4.5, "elevator_floor_time": 1.75, "max_iterations": 1000},
    }


def map_family_for_floors(floors: int) -> str:
    return "L" if floors <= 8 else "M"


def map_key_for(spec: Dict[str, Any]) -> str:
    return f"{spec['map_family']}-{spec['floors']}F-{spec.get('facility', 'balanced')}"


def build_manifest() -> Dict[str, Any]:
    cases: List[Dict[str, Any]] = []
    for r in ROBOT_LEVELS:
        for ratio in LOAD_RATIOS:
            m = max(1, round(r * ratio))
            ratio_code = str(ratio).replace(".", "p")
            for floors in FLOORS:
                family = map_family_for_floors(floors)
                for seed in SEEDS:
                    case_id = f"R{r}_M{m}_rho{ratio_code}_F{floors}_S{seed}"
                    cases.append({
                        "case_id": case_id,
                        "group": "free",
                        "robots": r,
                        "tasks": m,
                        "load_ratio": ratio,
                        "floors": floors,
                        "seed": seed,
                        "map_family": family,
                        "facility": "balanced",
                        "map_key": map_key_for({"map_family": family, "floors": floors, "facility": "balanced"}),
                    })
    ids = [c["case_id"] for c in cases]
    if len(cases) != len(ROBOT_LEVELS) * len(LOAD_RATIOS) * len(FLOORS) * len(SEEDS):
        raise AssertionError("unexpected case count")
    if len(set(ids)) != len(ids):
        raise AssertionError("duplicate case ids")
    return {
        "dataset_version": "icde-simulation-v3-free-combination",
        "robots_levels": ROBOT_LEVELS,
        "load_ratios": list(LOAD_RATIOS),
        "floors": FLOORS,
        "seeds": list(SEEDS),
        "notes": "Free cartesian grid R x load_ratio x floors x seeds. Materialize on demand.",
        "cases": cases,
    }


def make_map_for_spec(spec: Dict[str, Any]) -> Dict[str, Any]:
    dimensions = {"S": (20, 20), "M": (32, 32), "L": (40, 40)}
    width, height = dimensions[spec["map_family"]]
    return make_map(spec["floors"], width, height, spec.get("facility", "balanced"))


def materialize(root: Path, item: Dict[str, Any]) -> Path:
    maps_dir = root / "maps"
    cases_dir = root / "cases"
    map_path = maps_dir / f"{item['map_key']}.json"
    if not map_path.exists():
        write_json(map_path, make_map_for_spec(item))
    map_data = json.loads(map_path.read_text(encoding="utf-8"))
    output = cases_dir / f"{item['case_id']}.json"
    spec = dict(item)
    if "task_distribution" not in spec:
        spec["task_distribution"] = "uniform"
    if "robot_distribution" not in spec:
        spec["robot_distribution"] = "floor_balanced"
    write_json(output, make_case(spec, map_data))
    return output


def main() -> None:
    parser = argparse.ArgumentParser(description="Generate ICDE simulation dataset v3 (free combination)")
    parser.add_argument("--root", default=str(ROOT), help="dataset_v3 directory")
    parser.add_argument("--write-manifest", action="store_true", help="write maps and the 672-case manifest")
    parser.add_argument("--materialize-case", help="write one case JSON by case_id")
    parser.add_argument("--materialize-all", action="store_true", help="write every case JSON; may require substantial disk space")
    args = parser.parse_args()
    root = Path(args.root).resolve()
    manifest = build_manifest()
    if args.write_manifest or args.materialize_case or args.materialize_all:
        for item in manifest["cases"]:
            map_path = root / "maps" / f"{item['map_key']}.json"
            if not map_path.exists():
                write_json(map_path, make_map_for_spec(item))
        manifest_path = root / "manifests" / "case_manifest_v3.json"
        write_json(manifest_path, manifest)
    if args.materialize_case:
        item = next((x for x in manifest["cases"] if x["case_id"] == args.materialize_case), None)
        if item is None:
            raise SystemExit(f"unknown case_id: {args.materialize_case}")
        print(materialize(root, item))
    if args.materialize_all:
        for item in manifest["cases"]:
            materialize(root, item)
        print(f"materialized {len(manifest['cases'])} cases")
    if args.write_manifest:
        print(json.dumps({"cases": len(manifest["cases"]), "maps": len(list((root / 'maps').glob('*.json')))}, ensure_ascii=False, indent=2))
    if not (args.write_manifest or args.materialize_case or args.materialize_all):
        parser.print_help()


if __name__ == "__main__":
    main()
