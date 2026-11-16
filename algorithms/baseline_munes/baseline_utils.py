"""Small adapter shared by the standalone baseline copies.

The file is copied into every baseline directory, so each baseline remains
runnable without importing the main project.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any, Callable, Dict, List, Sequence

from core import (
    BuildingMap,
    Robot,
    SchedulerConfig,
    Task,
    generate_virtual,
    load_json,
    parse_payload,
    read_sqlite_inputs,
)


def arguments(description: str) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=description)
    parser.add_argument("--mode", choices=["virtual", "real", "generic"], default="virtual")
    parser.add_argument("--input")
    parser.add_argument("--map", dest="map_path")
    parser.add_argument("--db", dest="db_path")
    parser.add_argument("--output")
    return parser.parse_args()


def load_case(args: argparse.Namespace):
    if args.mode == "virtual":
        payload = generate_virtual(load_json(args.input) if args.input else {})
        return parse_payload(payload)
    if args.mode == "real":
        if not args.map_path or not args.db_path:
            raise SystemExit("real mode requires --map and --db")
        return load_real_inputs(args.map_path, args.db_path)
    if not args.input:
        raise SystemExit("generic mode requires --input")
    return parse_payload(load_json(args.input))


def load_real_inputs(map_path: str, db_path: str):
    building = BuildingMap.from_dict(load_json(map_path))
    robots, tasks = read_sqlite_inputs(db_path)
    return building, robots, tasks, SchedulerConfig()


def emit(args: argparse.Namespace, result: Dict[str, Any]) -> None:
    rendered = json.dumps(result, ensure_ascii=False, indent=2)
    if args.output:
        Path(args.output).write_text(rendered + "\n", encoding="utf-8")
    else:
        print(rendered)

