"""Batch execution engine: run one scheduler over one dataset split.

The heavy lifting lives in ``experiments/run_benchmark.py``; this module holds
the reusable pieces:

* ``ALGORITHMS``     -- where each scheduler entry point lives
* ``extract_metrics`` -- pull the ``metrics`` block out of a scheduler output
* ``run_case``       -- assemble a case, invoke the scheduler, time it out
* ``run_split``      -- small/large-tier concurrency, resume-safe result files
"""

from __future__ import annotations

import csv
import json
import subprocess
import sys
import tempfile
import time
from concurrent.futures import ThreadPoolExecutor, as_completed
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional

from . import dataset as ds

REPO_ROOT = ds.REPO_ROOT

# algo name -> entry point (relative to the repository root)
ALGORITHMS: Dict[str, Path] = {
    "main": Path("algorithms/multifloor_scheduler/run.py"),
    "nearest": Path("algorithms/multifloor_scheduler_nearest/run.py"),
    "wait": Path("algorithms/baseline_wait_only/run.py"),
    "strip": Path("algorithms/baseline_strip/run.py"),
    "mapd": Path("algorithms/baseline_mapd/run.py"),
    "cbs": Path("algorithms/baseline_cbs/run.py"),
    "munes": Path("algorithms/baseline_munes/run.py"),
}

RESULT_FIELDS = [
    "case_id", "status", "wall_s", "assigned_count", "unassigned_count",
    "makespan", "sum_completion_time", "total_wait_time",
    "final_conflict_count", "reroutes", "error",
]


def extract_metrics(obj: Any) -> Dict[str, Any]:
    """Depth-first search for the first ``metrics`` dictionary in a payload."""
    if isinstance(obj, dict):
        if isinstance(obj.get("metrics"), dict):
            return obj["metrics"]
        for value in obj.values():
            found = extract_metrics(value)
            if found:
                return found
    elif isinstance(obj, list):
        for value in obj:
            found = extract_metrics(value)
            if found:
                return found
    return {}


def run_case(
    entry: Dict[str, Any],
    algo: str,
    dataset: str,
    out_path: Path,
    timeout: float,
    sched_path: Path,
) -> Dict[str, Any]:
    case_id = entry["case_id"]
    row: Dict[str, Any] = {field: "" for field in RESULT_FIELDS}
    row["case_id"] = case_id

    with tempfile.TemporaryDirectory(prefix="mpsc_case_") as tmpdir:
        case_file = Path(tmpdir) / f"{case_id}.json"
        ds.materialize(case_id, dataset, case_file)
        raw_out = Path(tmpdir) / "out.json"
        started = time.monotonic()
        try:
            proc = subprocess.run(
                [
                    sys.executable, str(sched_path),
                    "--mode", "generic",
                    "--input", str(case_file),
                    "--output", str(raw_out),
                ],
                capture_output=True,
                text=True,
                timeout=timeout,
            )
        except subprocess.TimeoutExpired:
            row["status"] = f"TIMEOUT>{int(timeout)}s"
            row["wall_s"] = round(time.monotonic() - started, 2)
            return row
        wall = round(time.monotonic() - started, 2)
        row["wall_s"] = wall
        if proc.returncode != 0 or not raw_out.exists():
            row["status"] = "ERROR"
            row["error"] = (proc.stderr or "")[-500:]
            return row
        metrics = extract_metrics(json.loads(raw_out.read_text(encoding="utf-8")))

    row["status"] = "OK"
    for key in (
        "assigned_count", "unassigned_count", "final_conflict_count", "reroutes",
    ):
        row[key] = metrics.get(key)
    for key in ("makespan", "sum_completion_time", "total_wait_time"):
        value = metrics.get(key)
        row[key] = round(value, 6) if isinstance(value, (int, float)) else value
    return row


def _append_summary(path: Path, row: Dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    new_file = not path.exists()
    with path.open("a", newline="", encoding="utf-8-sig") as fh:
        writer = csv.DictWriter(fh, fieldnames=RESULT_FIELDS, extrasaction="ignore")
        if new_file:
            writer.writeheader()
        writer.writerow(row)


def run_split(
    dataset: str,
    algo: str,
    entries: List[Dict[str, Any]],
    workers: int,
    big_workers: int,
    big_r: int,
    timeout: float,
    big_timeout: float,
    results_root: Path,
) -> None:
    """Execute ``entries`` with per-tier concurrency, skipping finished cases."""
    sched_path = REPO_ROOT / ALGORITHMS[algo]
    if not sched_path.exists():
        raise SystemExit(f"scheduler entry point missing: {sched_path}")

    out_dir = results_root / dataset / algo
    out_dir.mkdir(parents=True, exist_ok=True)
    summary = out_dir / "summary.csv"

    todo = [
        entry for entry in entries
        if not (out_dir / f"{entry['case_id']}.result.json").exists()
    ]
    if not todo:
        print(f"[{dataset}/{algo}] all {len(entries)} cases already finished")
        return

    small = [entry for entry in todo if entry.get("robots", 0) < big_r]
    large = [entry for entry in todo if entry.get("robots", 0) >= big_r]

    def worker(entry: Dict[str, Any]) -> Dict[str, Any]:
        limit = big_timeout if entry.get("robots", 0) >= big_r else timeout
        row = run_case(entry, algo, dataset, out_dir, limit, sched_path)
        payload = {
            "case_id": entry["case_id"],
            "dataset": dataset,
            "algo": algo,
            **{key: entry.get(key) for key in
               ("scene", "robots", "tasks", "load_ratio", "floors", "seed")},
            "status": row["status"],
            "wall_s": row["wall_s"],
            "metrics": {
                key: row[key] for key in
                ("assigned_count", "unassigned_count", "makespan",
                 "sum_completion_time", "total_wait_time",
                 "final_conflict_count", "reroutes")
                if row[key] != ""
            },
            "error": row["error"],
        }
        (out_dir / f"{entry['case_id']}.result.json").write_text(
            json.dumps(payload, ensure_ascii=False, indent=1), encoding="utf-8"
        )
        _append_summary(summary, row)
        return row

    for label, group, concurrency in (
        ("small", small, workers),
        ("large", large, big_workers),
    ):
        if not group:
            continue
        print(f"[{dataset}/{algo}] {label} tier: {len(group)} cases, {concurrency} workers")
        with ThreadPoolExecutor(max_workers=max(1, concurrency)) as pool:
            futures = [pool.submit(worker, entry) for entry in group]
            for index, future in enumerate(as_completed(futures), 1):
                row = future.result()
                print(f"  [{index}/{len(group)}] {row['status']} {row['case_id']} "
                      f"{row['wall_s']}s makespan={row['makespan']} "
                      f"wait={row['total_wait_time']}")
    print(f"[{dataset}/{algo}] done -> {summary}")
