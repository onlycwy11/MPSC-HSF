#!/usr/bin/env python3
"""Generate a configurable synthetic building, robot set, and task batch."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from core import generate_virtual


def _csv_values(value: str) -> list[str]:
    values = [item.strip() for item in value.split(",") if item.strip()]
    if not values:
        raise argparse.ArgumentTypeError("provide at least one comma-separated value")
    return values


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Generate a synthetic multi-floor map with robot and task inputs"
    )
    parser.add_argument("--floors", type=int, default=3, help="number of floors")
    parser.add_argument("--width", type=int, default=6, help="grid nodes along X on each floor")
    parser.add_argument("--height", type=int, default=6, help="grid nodes along Y on each floor")
    parser.add_argument("--elevators", type=int, default=1, help="number of elevator shafts")
    parser.add_argument("--stairs", type=int, default=1, help="number of stair shafts")
    parser.add_argument("--robots", type=int, default=6, help="number of robots")
    parser.add_argument("--robot-types", type=_csv_values, default=["wheeled", "legged"],
                        help="comma-separated movement types, e.g. wheeled,legged")
    parser.add_argument("--tasks", type=int, default=6, help="number of tasks in this batch")
    parser.add_argument("--task-types", type=_csv_values, default=["general"],
                        help="comma-separated task types, e.g. medicine,sample")
    parser.add_argument("--seed", type=int, default=42, help="random seed")
    parser.add_argument("--grid-spacing", type=float, default=5.0, help="horizontal node spacing in meters")
    parser.add_argument("--floor-height", type=float, default=3.5, help="floor-to-floor height in meters")
    parser.add_argument("--speed-xy", type=float, default=1.5, help="default robot planar speed in m/s")
    parser.add_argument("--stair-speed", type=float, default=0.5, help="stair vertical speed in m/s")
    parser.add_argument("--output", required=True, help="path for the generated JSON input")
    args = parser.parse_args()

    config = {
        "floors": args.floors,
        "width": args.width,
        "height": args.height,
        "elevators": args.elevators,
        "stairs": args.stairs,
        "robots": args.robots,
        "robot_types": args.robot_types,
        "tasks": args.tasks,
        "task_types": args.task_types,
        "seed": args.seed,
        "grid_spacing": args.grid_spacing,
        "floor_height": args.floor_height,
        "speed_xy": args.speed_xy,
        "stair_speed": args.stair_speed,
    }
    try:
        payload = generate_virtual(config)
    except ValueError as error:
        parser.error(str(error))

    output_path = Path(args.output)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(f"Generated map input: {output_path.resolve()}")
    print(f"Nodes: {len(payload['map']['nodes'])}; edges: {len(payload['map']['edges'])}")
    print(f"Robots: {len(payload['robots'])}; tasks: {len(payload['tasks'])}")
    print("Node coordinates use meters: x, y, z; z is floor * floor_height.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
