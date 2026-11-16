#!/usr/bin/env python3
"""One-command runner for the ICDE real dataset (204 cases, 3 scenes).

Assembles each skeleton case (map_ref + robots + tasks) into a single-file
generic case, invokes a scheduler (main algorithm or a baseline), extracts the
metrics, stores results/{algo}/{case_id}.result.json, and finally writes a
summary.csv. Resume-safe: finished cases are skipped.

Usage (Windows PowerShell):
  python run_all_real.py --algo main                       # main algorithm, all scenes
  python run_all_real.py --algo all                        # main + all baselines
  python run_all_real.py --scenes real --robots 512        # one scene / one robot level
  python run_all_real.py --case real_R512_M512_rho1p0_S17  # one case
  python run_all_real.py --limit 3 --algo main             # quick smoke (first 3 unfinished)

Concurrency: R < big_r uses --workers threads; R >= big_r uses --big-workers
(default: 2 / 4 / threshold 256). Everything is resume-safe; kill and rerun.

Scheduler entrypoints (edit SCHEDULERS if your paths differ):
  main : D:\\Python\\multifloor_scheduler\\run.py
  nearest / wait / munes / cbs / strip / mapd : baseline_*\\run.py
"""

from __future__ import annotations

import argparse
import csv
import json
import os
import shutil
import subprocess
import sys
import time
from concurrent.futures import ThreadPoolExecutor, as_completed
from pathlib import Path

ROOT = Path(__file__).resolve().parent
MANIFEST = ROOT / "manifests" / "case_manifest_real.json"
RESULTS = ROOT / "results"
TMP = RESULTS / "_tmp"

# 算法入口 = 相对子路径。脚本自动探测常见位置；找不到时用 --sched-root 指定你的目录。
SCHEDULER_SUBS = {
    "main": Path("multifloor_scheduler") / "run.py",
    "nearest": Path("multifloor_scheduler_nearest") / "run.py",
    "wait": Path("baseline_wait_only") / "run.py",
    "munes": Path("baseline_munes") / "run.py",
    "cbs": Path("baseline_cbs") / "run.py",
    "strip": Path("baseline_strip") / "run.py",
    "mapd": Path("baseline_mapd") / "run.py",
}
ALL_ALGOS = list(SCHEDULER_SUBS)
PY = sys.executable


def resolve_schedulers(sched_root: str | None) -> dict:
    """Probe candidate roots (CLI --sched-root > env MF_SCHED_ROOT > defaults)."""
    roots: list[Path] = []
    if sched_root:
        roots.append(Path(sched_root).resolve())
    env_root = os.environ.get("MF_SCHED_ROOT")
    if env_root:
        roots.append(Path(env_root).resolve())
    roots += [Path.home() / "Python", Path("D:/Python"), Path("C:/Python"), ROOT]
    found: dict = {}
    for name, sub in SCHEDULER_SUBS.items():
        for root in roots:
            p = root / sub
            if p.exists():
                found[name] = p
                break
    return found


def find_metrics(obj, out=None):
    if out is None:
        out = []
    if isinstance(obj, dict):
        for k, v in obj.items():
            if k == "metrics":
                out.append(v)
            find_metrics(v, out)
    elif isinstance(obj, list):
        for v in obj:
            find_metrics(v, out)
    return out


def assemble_and_run(item: dict, algo: str, sched_path: Path, out_path: Path) -> dict:
    skeleton = json.loads((ROOT / "cases" / f"{item['case_id']}.json").read_text(encoding="utf-8"))
    map_data = json.loads((ROOT / skeleton["map_ref"]).read_text(encoding="utf-8"))
    single = {
        "schema_version": "multifloor-scheduler-generic-v2",
        "case": skeleton["case"],
        "map": map_data,
        "robots": skeleton["robots"],
        "tasks": skeleton["tasks"],
        "config": skeleton["config"],
    }
    TMP.mkdir(parents=True, exist_ok=True)
    single_path = TMP / f"{item['case_id']}_{algo}.json"
    single_path.write_text(json.dumps(single, ensure_ascii=False), encoding="utf-8")
    out_raw = single_path.with_suffix(".out.json")

    start = time.time()
    try:
        proc = subprocess.run(
            [PY, str(sched_path), "--mode", "generic", "--input", str(single_path), "--output", str(out_raw)],
            capture_output=True, text=True, timeout=86400,
        )
    finally:
        single_path.unlink(missing_ok=True)

    elapsed = time.time() - start
    metrics: dict = {}
    if out_raw.exists():
        try:
            payload = json.loads(out_raw.read_text(encoding="utf-8"))
            found = find_metrics(payload)
            metrics = found[0] if found else {}
        except Exception as e:  # noqa: BLE001
            metrics = {"error": f"parse output: {e}"}
        out_raw.unlink(missing_ok=True)
    if not metrics:
        metrics = {
            "error": "no metrics in output",
            "returncode": proc.returncode,
            "stderr_tail": (proc.stderr or "")[-800:],
        }
    result = {
        "case_id": item["case_id"],
        "scene": item["scene"],
        "robots": item["robots"],
        "tasks": item["tasks"],
        "load_ratio": item["load_ratio"],
        "seed": item["seed"],
        "algo": algo,
        "elapsed_s": round(elapsed, 2),
        "metrics": metrics,
    }
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(json.dumps(result, ensure_ascii=False, indent=1), encoding="utf-8")
    return result


def summarize(algo: str, results_dir: Path) -> None:
    rows = []
    for f in sorted(results_dir.glob("*.result.json")):
        try:
            rows.append(json.loads(f.read_text(encoding="utf-8")))
        except Exception:  # noqa: BLE001
            continue
    if not rows:
        return
    fieldnames = ["case_id", "scene", "robots", "tasks", "load_ratio", "seed", "elapsed_s"]
    metric_keys = ["makespan", "sum_completion_time", "total_wait_time", "initial_conflict_count", "final_conflict_count", "iterations", "assigned_count", "error"]
    for r in rows:
        m = r.get("metrics", {})
        for k in metric_keys:
            if k not in fieldnames and k not in r:
                fieldnames.append(k)
            if k not in r:
                r[k] = m.get(k)
    with (results_dir / "summary.csv").open("w", encoding="utf-8-sig", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=fieldnames, extrasaction="ignore")
        w.writeheader()
        for r in rows:
            w.writerow(r)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--algo", nargs="+", default=["main"], help="main | nearest | wait | munes | cbs | strip | mapd | all (space-separated)")
    ap.add_argument("--scenes", nargs="+", default=["all"], help="all | real | wh8 | wh10 (space-separated)")
    ap.add_argument("--robots", type=int, default=None, help="only this robot count")
    ap.add_argument("--case", "--cases", nargs="+", default=[], help="exact case_id(s), space-separated")
    ap.add_argument("--limit", type=int, default=0, help="run at most N unfinished cases (smoke)")
    ap.add_argument("--workers", type=int, default=2, help="concurrency for small cases")
    ap.add_argument("--big-workers", type=int, default=4, help="concurrency for large cases")
    ap.add_argument("--big-r", type=int, default=256, help="robots >= this value use --big-workers")
    ap.add_argument("--sched-root", default=None, help="your folder containing multifloor_scheduler/ and baseline_*/ (auto-probed otherwise)")
    args = ap.parse_args()

    algos = ALL_ALGOS if "all" in args.algo else args.algo
    for a in algos:
        if a not in SCHEDULER_SUBS:
            raise SystemExit(f"unknown algo: {a}")
    schedulers = resolve_schedulers(args.sched_root)
    missing = [a for a in algos if a not in schedulers]
    if missing:
        raise SystemExit(
            f"找不到以下算法入口: {missing}\n"
            f"已尝试位置: 当前目录 / 用户目录\\Python / D:\\Python / C:\\Python\n"
            f"请把主算法和 baseline 目录放在其中之一，或指定: python run_all_real.py --sched-root <你的代码目录>\n"
            f"（目录下需要有 multifloor_scheduler\\run.py、baseline_*\\run.py）"
        )

    manifest = json.loads(MANIFEST.read_text(encoding="utf-8"))
    items = manifest["cases"]
    if "all" not in args.scenes:
        items = [c for c in items if c["scene"] in args.scenes]
    if args.robots is not None:
        items = [c for c in items if c["robots"] == args.robots]
    if args.case:
        wanted = set(args.case)
        items = [c for c in items if c["case_id"] in wanted]

    for algo in algos:
        results_dir = RESULTS / algo
        todo = [c for c in items if not (results_dir / f"{c['case_id']}.result.json").exists()]
        if args.limit:
            todo = todo[: args.limit]
        print(f"[{algo}] scenes={args.scenes} cases={len(items)} todo={len(todo)} "
              f"(workers={args.workers} big={args.big_workers}@{args.big_r})")
        if not todo:
            continue

        def worker(item):
            return assemble_and_run(item, algo, schedulers[algo], results_dir / f"{item['case_id']}.result.json")

        big = [c for c in todo if c["robots"] >= args.big_r]
        small = [c for c in todo if c not in big]
        with ThreadPoolExecutor(max_workers=args.workers) as ex:
            futs = [ex.submit(worker, c) for c in small]
            for i, fut in enumerate(as_completed(futs), 1):
                r = fut.result()
                m = r["metrics"]
                print(f"  [{i}/{len(small)}] {r['case_id']} {r['elapsed_s']}s "
                      f"makespan={m.get('makespan')} wait={m.get('total_wait_time')} "
                      f"conf={m.get('final_conflict_count')} err={m.get('error', '')}")
        with ThreadPoolExecutor(max_workers=args.big_workers) as ex:
            futs = [ex.submit(worker, c) for c in big]
            for i, fut in enumerate(as_completed(futs), 1):
                r = fut.result()
                m = r["metrics"]
                print(f"  [BIG {i}/{len(big)}] {r['case_id']} {r['elapsed_s']}s "
                      f"makespan={m.get('makespan')} wait={m.get('total_wait_time')} "
                      f"conf={m.get('final_conflict_count')} err={m.get('error', '')}")
        summarize(algo, results_dir)
        print(f"[{algo}] done -> results/{algo}/summary.csv")

    shutil.rmtree(TMP, ignore_errors=True)


if __name__ == "__main__":
    main()
