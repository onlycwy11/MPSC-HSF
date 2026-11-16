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
    Plan,
    Robot,
    SchedulerConfig,
    Task,
    build_initial_plans,
    detect_conflicts,
    generate_virtual,
    load_json,
    load_real_inputs,
    parse_payload,
)


def arguments(description: str) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=description)
    parser.add_argument("--mode", choices=["virtual", "real", "generic"], default="virtual")
    parser.add_argument("--input")
    parser.add_argument("--map", dest="map_path")
    parser.add_argument("--db", dest="db_path")
    parser.add_argument("--output")
    parser.add_argument("--max-iterations", type=int, default=None,
                       help="覆盖 CBS 约束树展开节点上限（默认取 case config，未设时为 1000）")
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


def plans_result(plans: Sequence[Plan], unmatched: Sequence[str], building: BuildingMap, initial_count: int, iterations: int = 0, reroutes: int = 0) -> Dict[str, Any]:
    final = detect_conflicts(plans, building)
    assignments: List[Dict[str, Any]] = []
    for plan in sorted(plans, key=lambda item: item.task.id):
        assignments.append(
            {
                "task_id": plan.task.id,
                "robot_id": plan.robot.id,
                "robot_type": plan.robot.robot_type,
                "start_time": round(plan.start_time, 6),
                "completion_time": round(plan.end_time, 6),
                "travel_time": round(plan.travel_time, 6),
                "wait_time": round(plan.wait_time, 6),
                "vertical_mode": plan.candidate.vertical_mode,
                "path": plan.candidate.nodes,
                "edges": [{"src": e.src, "dst": e.dst, "kind": e.kind, "resource": e.resource} for e in plan.candidate.edges],
            }
        )
    completion = [x["completion_time"] for x in assignments]
    travel = [x["travel_time"] for x in assignments]
    waits = [x["wait_time"] for x in assignments]
    metrics = {
        "assigned_count": len(assignments),
        "unassigned_count": len(unmatched),
        "makespan": max(completion, default=0.0),
        "sum_completion_time": round(sum(completion), 6),
        "average_travel_time": round(sum(travel) / len(travel), 6) if travel else 0.0,
        "total_wait_time": round(sum(waits), 6),
        "initial_conflict_count": initial_count,
        "final_conflict_count": len(final),
        "iterations": iterations,
        "reroutes": reroutes,
    }
    return {"assignments": assignments, "unassigned_tasks": list(unmatched), "metrics": metrics}


def emit(args: argparse.Namespace, result: Dict[str, Any]) -> None:
    rendered = json.dumps(result, ensure_ascii=False, indent=2)
    if args.output:
        Path(args.output).write_text(rendered + "\n", encoding="utf-8")
    else:
        print(rendered)

