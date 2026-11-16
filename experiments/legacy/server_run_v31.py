"""server_run_v31.py — 专门跑 v3.1 R128 补充集（84 case），与 v3 主实验完全隔离。

用法（在解压后的包根目录）:
    python server_run_v31.py                  # 默认3并发，输出到 results_v31/
    python server_run_v31.py --workers 4      # 4并发
    python server_run_v31.py --dry-run         # 只看case列表不跑

输出: results_v31/<case>.result.json + results_v31/summary.csv
不碰 v3 的 results/ 目录，不影响正在跑的主实验。
"""
import argparse
import csv
import json
import re
import subprocess
import sys
import threading
import time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

ROOT = Path(__file__).resolve().parent
ALGO_DIR = ROOT / "multifloor_scheduler"
V31_DIR = ROOT / "dataset_v3.1"
V3_DIR = ROOT / "dataset_v3"

ROBOT_RE = re.compile(r"R(\d+)_M\d+")

ROW_FIELDS = ["case", "status", "wall_s", "assigned_count", "unassigned_count",
              "makespan", "sum_completion_time", "total_wait_time",
              "final_conflict_count", "reroutes", "error"]

_WRITE_LOCK = threading.Lock()


def v31_all_cases() -> list:
    manifest = json.loads(
        (V31_DIR / "manifests" / "case_manifest_v3.1.json").read_text(encoding="utf-8")
    )
    return [c["case_id"] for c in manifest["cases"]]


def ensure_materialized(case: str) -> Path:
    # 优先用 v3/cases/ 下已物化好的文件（之前已复制过去）
    case_file = V3_DIR / "cases" / f"{case}.json"
    if case_file.exists():
        return case_file
    # 否则用 v3.1 生成器现场物化
    print(f"  物化 {case} ...", flush=True)
    gen = V31_DIR / "generate_v3_1.py"
    r = subprocess.run(
        [sys.executable, str(gen), "--materialize-case", case],
        capture_output=True, text=True, timeout=3600,
    )
    # v3.1 生成器物化到 v3.1/cases/，软链/复制到 v3/cases/
    v31_file = V31_DIR / "cases" / f"{case}.json"
    if v31_file.exists():
        import shutil
        shutil.copy(v31_file, case_file)
        return case_file
    raise RuntimeError(f"物化失败: {case} -> {r.stderr[-300:]}")


def empty_row(case: str, status: str, wall_s=None, error="") -> dict:
    row = {k: ("" if k in ("case", "status", "error") else None) for k in ROW_FIELDS}
    row["case"] = case
    row["status"] = status
    row["wall_s"] = wall_s
    row["error"] = error
    return row


def run_one(case: str, outdir: Path, timeout: float) -> dict:
    try:
        case_file = ensure_materialized(case)
    except RuntimeError as exc:
        return empty_row(case, "MATERIALIZE_ERROR", error=str(exc))
    out_file = outdir / f"{case}.result.json"
    t0 = time.monotonic()
    try:
        r = subprocess.run(
            [sys.executable, "run.py", "--mode", "generic",
             "--input", str(case_file), "--output", str(out_file)],
            cwd=ALGO_DIR, capture_output=True, text=True, timeout=timeout,
        )
        wall = round(time.monotonic() - t0, 2)
        if r.returncode != 0:
            return empty_row(case, "ERROR", wall, r.stderr[-500:])
        if not out_file.exists():
            return empty_row(case, "NO_OUTPUT", wall, "run.py 未生成输出文件")
        m = json.loads(out_file.read_text(encoding="utf-8"))["metrics"]
        row = empty_row(case, "OK", wall)
        row.update({
            "assigned_count": m.get("assigned_count"),
            "unassigned_count": m.get("unassigned_count"),
            "makespan": round(m.get("makespan", 0), 6),
            "sum_completion_time": round(m.get("sum_completion_time", 0), 6),
            "total_wait_time": round(m.get("total_wait_time", 0), 6),
            "final_conflict_count": m.get("final_conflict_count"),
            "reroutes": m.get("reroutes"),
        })
        return row
    except subprocess.TimeoutExpired:
        return empty_row(case, f"TIMEOUT>{int(timeout)}s", timeout)


def append_summary(outdir: Path, row: dict) -> None:
    outdir.mkdir(parents=True, exist_ok=True)
    path = outdir / "summary.csv"
    with path.open("a", newline="", encoding="utf-8") as f:
        if f.tell() == 0:
            f.write(",".join(ROW_FIELDS) + "\n")
        w = csv.DictWriter(f, fieldnames=ROW_FIELDS, extrasaction="ignore")
        w.writerow(row)


def _print_row(row: dict) -> None:
    line = f"[{row['status']}] {row['case']}"
    if row.get("wall_s") is not None:
        line += f"  {row['wall_s']}s"
    if row.get("sum_completion_time") is not None:
        line += (f"  makespan={row['makespan']}  sum={row['sum_completion_time']}"
                 f"  wait={row['total_wait_time']}  final_conf={row['final_conflict_count']}")
    print(line, flush=True)
    if row.get("error"):
        print("    ->", row["error"][:300], flush=True)


def main() -> None:
    p = argparse.ArgumentParser(description="跑 v3.1 R128 补充集（与 v3 主实验隔离）")
    p.add_argument("--workers", type=int, default=3, help="并发数，默认3")
    p.add_argument("--timeout", type=float, default=3600, help="单case上限秒，默认3600(1h)")
    p.add_argument("--outdir", default="results_v31", help="输出目录，默认 results_v31")
    p.add_argument("--dry-run", action="store_true", help="只打印case列表不执行")
    p.add_argument("--cases", nargs="+", default=None, help="只跑指定 case 列表")
    args = p.parse_args()

    cases = args.cases if args.cases else v31_all_cases()
    outdir = Path(args.outdir)
    if not outdir.is_absolute():
        outdir = ROOT / outdir
    outdir.mkdir(parents=True, exist_ok=True)

    print(f"v3.1 R128 补充集：共 {len(cases)} case，{args.workers} 并发，上限 {args.timeout}s", flush=True)
    print(f"输出目录：{outdir.resolve()}", flush=True)

    if args.dry_run:
        for c in cases:
            print("  ", c)
        return

    def task(case: str) -> dict:
        row = run_one(case, outdir, args.timeout)
        with _WRITE_LOCK:
            append_summary(outdir, row)
        return row

    with ThreadPoolExecutor(max_workers=max(1, args.workers)) as ex:
        for row in ex.map(task, cases):
            _print_row(row)

    print(f"\n完成。结果已写入 {outdir.resolve()}")


if __name__ == "__main__":
    main()
