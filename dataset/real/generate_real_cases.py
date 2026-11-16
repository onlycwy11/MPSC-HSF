#!/usr/bin/env python3
"""Generate the ICDE real-dataset case set (skeleton cases + manifest).

Skeleton case = {schema_version, case, map_ref, robots, tasks, config}.
The full map JSON is referenced (map_ref) instead of embedded, because the real
maps are large (18-60 MB); run_all_real.py assembles the single-file case on
the fly (map + robots + tasks) and deletes the temp file afterwards.

Scenes & workload design (scaled to real floor area, no unrealistic densities):
  - real (real_1-7F hospital): R in {16, 64, 128, 256, 512}
      max 512 robots over 7 floors (~73 robots/floor, ~72 m^2 per robot)
  - wh8 (movingai_warehouse_8F): R in {16, 64, 128, 256, 512, 1024}
  - wh10(movingai_warehouse_10F): R in {16, 64, 128, 256, 512, 1024}
  - task load: 4 ratios {0.5, 1.0, 1.5, 2.0} -> tasks = max(1, round(R*ratio))
  - seeds: 3 {17, 29, 43}
Total: 60 + 72 + 72 = 204 cases.

Case behaviour (aligned with v3 simulator dataset):
  - robots: floor-balanced starts, 75% wheeled / 25% legged
  - tasks: uniform floor choice, 50% cross-floor (pickup floor != delivery floor),
    endpoints prefer real topo navigation points, fall back to corridor cells
  - elevator/stair lobbies excluded from pickups/deliveries

Usage:
  python generate_real_cases.py            # generate manifest + all skeleton cases
"""

from __future__ import annotations

import argparse
import json
import math
import random
from pathlib import Path
from typing import Any, Dict, List, Tuple

ROOT = Path(__file__).resolve().parent
MAPS = ROOT / "maps"
CASES = ROOT / "cases"

SEEDS = (17, 29, 43)
LOAD_RATIOS = (0.5, 1.0, 1.5, 2.0)
SCENES = {
    "real": {"map": "real_1-7F.json", "robots": [16, 64, 128, 256, 512]},
    "wh8": {"map": "movingai_warehouse_8F.json", "robots": [16, 64, 128, 256, 512, 1024]},
    "wh10": {"map": "movingai_warehouse_10F.json", "robots": [16, 64, 128, 256, 512, 1024]},
}
WHEELED_FRACTION = 0.75
CROSS_FLOOR_RATIO = 0.5
CONFIG = {
    "speed_xy": 1.5,
    "stair_speed": 0.5,
    "floor_height": 3.5,
    "elevator_board_time": 4.5,
    "elevator_floor_time": 1.75,
    "max_iterations": 1000,
}


def write_json(path: Path, value: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, ensure_ascii=False, indent=1), encoding="utf-8")


def load_map(map_name: str) -> Dict[str, Any]:
    return json.loads((MAPS / map_name).read_text(encoding="utf-8"))


def load_topo(map_name: str) -> Dict[str, List[str]]:
    """topo_points.json: {floorN: {topo_name: node_id}} -> {floor: [node_ids]}."""
    topo_file = MAPS / "topo_points.json"
    if not topo_file.exists():
        return {}
    data = json.loads(topo_file.read_text(encoding="utf-8"))
    out: Dict[int, List[str]] = {}
    for fid, mapping in data.items():
        if mapping:
            out[int(fid.replace("floor", ""))] = list(mapping.values())
    return out


def floor_balanced_start(rng: random.Random, by_floor: Dict[int, List[str]], index: int) -> str:
    floors = sorted(by_floor)
    floor = floors[index % len(floors)]
    pool = by_floor[floor]
    return rng.choice(pool)


def task_pool_for(floor: int, by_floor: Dict[int, List[str]], topo: Dict[int, List[str]]) -> List[str]:
    """Prefer real topo navigation points; fall back to corridor cells (RAR/benchmark floors)."""
    pool = topo.get(floor, [])
    if pool:
        return [n for n in pool if n in by_floor.get(floor, [])] or by_floor.get(floor, [])
    return by_floor.get(floor, [])


def make_case_skeleton(spec: Dict[str, Any], map_data: Dict[str, Any], topo: Dict[int, List[str]]) -> Dict[str, Any]:
    rng = random.Random(int(spec["seed"]))
    lobby = {"elevator_lobby", "stair_lobby"}
    by_floor: Dict[int, List[str]] = {}
    start_by_floor: Dict[int, List[str]] = {}
    for n in map_data["nodes"]:
        fid = int(n["floor"])
        if n["node_type"] in lobby:
            continue
        by_floor.setdefault(fid, []).append(n["id"])
        start_by_floor.setdefault(fid, []).append(n["id"])
    floors = sorted(by_floor)

    robots: List[Dict[str, Any]] = []
    r_count = int(spec["robots"])
    wheeled = round(r_count * WHEELED_FRACTION)
    for i in range(r_count):
        start = floor_balanced_start(rng, start_by_floor, i)
        robots.append({
            "id": f"r{i}",
            "robot_type": "wheeled" if i < wheeled else "legged",
            "task_type": "general",
            "start_node": start,
            "speed_xy": 1.5,
            "speed_z": 0.5,
            "available_at": 0.0,
        })

    tasks: List[Dict[str, Any]] = []
    t_count = int(spec["tasks"])
    for j in range(t_count):
        pickup_floor = rng.choice(floors)
        pickup_pool = task_pool_for(pickup_floor, by_floor, topo)
        pickup = rng.choice(pickup_pool)
        if len(floors) > 1 and rng.random() < CROSS_FLOOR_RATIO:
            other = [f for f in floors if f != pickup_floor]
            delivery_floor = rng.choice(other)
        else:
            delivery_floor = pickup_floor
        delivery_pool = task_pool_for(delivery_floor, by_floor, topo)
        delivery = rng.choice(delivery_pool)
        guard = 0
        while delivery == pickup and guard < 20:
            delivery = rng.choice(delivery_pool)
            guard += 1
        tasks.append({
            "id": f"t{j}",
            "pickup": pickup,
            "delivery": delivery,
            "pickup_floor": pickup_floor,
            "delivery_floor": delivery_floor,
            "task_type": "general",
            "release_time": 0.0,
        })

    return {
        "schema_version": "multifloor-scheduler-generic-v2-skeleton",
        "case": spec,
        "map_ref": f"maps/{spec['map']}",
        "robots": robots,
        "tasks": tasks,
        "config": dict(CONFIG),
    }


def build_manifest() -> Dict[str, Any]:
    cases: List[Dict[str, Any]] = []
    for scene, cfg in SCENES.items():
        for r in cfg["robots"]:
            for ratio in LOAD_RATIOS:
                m = max(1, round(r * ratio))
                ratio_code = str(ratio).replace(".", "p")
                for seed in SEEDS:
                    case_id = f"{scene}_R{r}_M{m}_rho{ratio_code}_S{seed}"
                    cases.append({
                        "case_id": case_id,
                        "scene": scene,
                        "map": cfg["map"],
                        "map_ref": f"maps/{cfg['map']}",
                        "robots": r,
                        "tasks": m,
                        "load_ratio": ratio,
                        "seed": seed,
                    })
    ids = [c["case_id"] for c in cases]
    if len(set(ids)) != len(ids):
        raise AssertionError("duplicate case ids")
    return {
        "dataset_version": "icde-real-v1",
        "scenes": {k: {"map": v["map"], "robots": v["robots"]} for k, v in SCENES.items()},
        "load_ratios": list(LOAD_RATIOS),
        "seeds": list(SEEDS),
        "wheeled_fraction": WHEELED_FRACTION,
        "cross_floor_ratio": CROSS_FLOOR_RATIO,
        "note": "Skeleton cases (map_ref), assembled at run time by run_all_real.py. "
                "Robot levels are capped by real floor area: hospital 512 max (≈73/floor), "
                "warehouse 1024 max (≈100-140/floor).",
        "cases": cases,
    }


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--only", type=str, default=None, help="case_id prefix filter, e.g. real_R64")
    ap.add_argument("--seed", type=int, default=None, help="only this seed")
    ap.add_argument("--robots", type=int, default=None, help="only this robot count")
    args = ap.parse_args()

    manifest = build_manifest()
    topo = load_topo("topo_points.json")
    maps_cache: Dict[str, Dict[str, Any]] = {}

    count = 0
    for item in manifest["cases"]:
        if args.only and not item["case_id"].startswith(args.only):
            continue
        if args.seed is not None and item["seed"] != args.seed:
            continue
        if args.robots is not None and item["robots"] != args.robots:
            continue
        if item["map"] not in maps_cache:
            maps_cache[item["map"]] = load_map(item["map"])
        spec = dict(item)
        skeleton = make_case_skeleton(spec, maps_cache[item["map"]], topo)
        write_json(CASES / f"{item['case_id']}.json", skeleton)
        count += 1

    write_json(ROOT / "manifests" / "case_manifest_real.json", manifest)
    print(f"manifest: {len(manifest['cases'])} cases; written skeletons: {count}")
    print("total cases per scene:", {s: len([c for c in manifest['cases'] if c['scene'] == s]) for s in SCENES})


if __name__ == "__main__":
    main()
