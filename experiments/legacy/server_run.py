"""server_run.py — 服务器端批量跑主算法（multifloor_scheduler）

在旧版基础上新增：普通档/大档自动分组、分档并发、分档超时上限。
旧参数（--cases / --all-v3 / --shard / --dry-run / --outdir）全部兼容，
默认 workers=1 串行，行为与旧版完全一致。

分组规则（按 case_id 里的机器人数 R）：
  - 普通档 R < --big-r（默认4096）：--workers 并发，超时 --small-timeout（默认24h）
  - 大档   R >= --big-r：单独成组，--big-workers 并发（默认4），超时 --big-timeout（默认8h=28800s）
  先跑完普通档，再跑大档，避免大 case 抢占。

用法（在解压后的包根目录）:
    python server_run.py --all-v3                      # 默认串行，旧行为
    python server_run.py --all-v3 --workers 3          # 普通档3并发
    # 只跑 R4096 大档，4并发、8h上限：
    python server_run.py --all-v3 --big-r 4096 --big-workers 4 --big-timeout 28800
    # 手动指定大 case 列表：
    python server_run.py --cases R4096_M6144_rho1p5_F8_S17 ... --big-workers 4
    python server_run.py --all-v3 --shard 1/3 --workers 3
    python server_run.py --all-v3 --dry-run

输出: <outdir>/<case>.result.json，每个 case 跑完立即追加一行到 <outdir>/summary.csv。
仅依赖 Python 标准库（conda myenv 的 Python 3.10 即可）。
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
V2_DIR = ROOT / "dataset_v2"
V3_DIR = ROOT / "dataset_v3"

V3_CASE_RE = re.compile(r"^R\d+_M\d+_rho\d+p\d+_F\d+_S\d+$")
# 同时匹配 v3(R4096_M6144_...) 与 v2(A_R64_M64_...) 里的机器人数
ROBOT_RE = re.compile(r"R(\d+)_M\d+")

ROW_FIELDS = ["case", "status", "wall_s", "assigned_count", "unassigned_count",
              "makespan", "sum_completion_time", "total_wait_time",
              "final_conflict_count", "reroutes", "error"]

_WRITE_LOCK = threading.Lock()


def v3_all_cases() -> list:
    manifest = json.loads(
        (V3_DIR / "manifests" / "case_manifest_v3.json").read_text(encoding="utf-8")
    )
    return [c["case_id"] for c in manifest["cases"]]


def shard_cases(cases: list, shard: str) -> list:
    """Round-robin split so every shard contains the full size spectrum."""
    if not shard:
        return cases
    idx, total = (int(x) for x in shard.split("/"))
    return [c for i, c in enumerate(cases) if i % total == idx - 1]


def robots_of(case: str) -> int:
    m = ROBOT_RE.search(case)
    return int(m.group(1)) if m else 0


def locate_generator(case: str) -> tuple:
    if V3_CASE_RE.match(case):
        return V3_DIR, V3_DIR / "generate_icde_dataset_v3.py"
    return V2_DIR, V2_DIR / "generate_icde_dataset_v2.py"


def ensure_materialized(case: str) -> Path:
    ds, gen = locate_generator(case)
    case_file = ds / "cases" / f"{case}.json"
    if case_file.exists():
        return case_file
    print(f"  物化 {case} ...", flush=True)
    r = subprocess.run(
        [sys.executable, str(gen), "--materialize-case", case],
        capture_output=True, text=True, timeout=86400,
    )
    if r.returncode != 0 or not case_file.exists():
        raise RuntimeError(f"物化失败: {case} -> {r.stderr[-300:]}")
    return case_file


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="服务器端批量跑主算法（v2 + v3，分档并发/超时）")
    p.add_argument("--cases", nargs="+", help="case 名列表")
    p.add_argument("--all-v3", action="store_true", help="自动读取 v3 清单全部 case")
    p.add_argument("--shard", default=None, help="分片 i/n，如 1/3（可与 --all-v3 或 --cases 搭配）")
    p.add_argument("--dry-run", action="store_true", help="只打印将跑的 case 列表，不执行")
    p.add_argument("--outdir", default="results", help="结果输出目录，默认 results/")
    # 新增：并发与分档超时
    p.add_argument("--workers", type=int, default=1, help="普通档并发数，默认1（串行，旧行为）")
    p.add_argument("--big-r", type=int, default=4096, help="R>=此值视为大档，默认4096")
    p.add_argument("--big-workers", type=int, default=4, help="大档并发数，默认4")
    p.add_argument("--big-timeout", type=float, default=28800, help="大档单case上限秒，默认28800(8h)")
    p.add_argument("--small-timeout", type=float, default=86400, help="普通档单case上限秒，默认86400(24h)")
    return p.parse_args()


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


def _run_group(cases: list, outdir: Path, workers: int, timeout: float) -> None:
    if not cases:
        return
    def task(case: str) -> dict:
        row = run_one(case, outdir, timeout)
        with _WRITE_LOCK:
            append_summary(outdir, row)
        return row
    with ThreadPoolExecutor(max_workers=max(1, workers)) as ex:
        for row in ex.map(task, cases):
            _print_row(row)


def main() -> None:
    args = parse_args()
    if not args.cases and not args.all_v3:
        raise SystemExit("请用 --cases <case 列表> 或 --all-v3")
    cases = list(args.cases or [])
    if args.all_v3:
        cases = shard_cases(v3_all_cases(), args.shard)
    elif args.shard:
        cases = shard_cases(cases, args.shard)

    small = [c for c in cases if robots_of(c) < args.big_r]
    big = [c for c in cases if robots_of(c) >= args.big_r]

    if args.dry_run:
        print(f"dry-run: 共{len(cases)} case | 普通档{len(small)} (R<{args.big_r}, {args.workers}并发, {args.small_timeout}s上限) "
              f"| 大档{len(big)} (R>={args.big_r}, {args.big_workers}并发, {args.big_timeout}s上限)")
        for c in small:
            print("  [S]", c)
        for c in big:
            print("  [B]", c)
        return

    outdir = Path(args.outdir)
    if not outdir.is_absolute():
        outdir = ROOT / outdir
    outdir.mkdir(parents=True, exist_ok=True)

    print(f"普通档 {len(small)} case：{args.workers} 并发，上限 {args.small_timeout}s", flush=True)
    _run_group(small, outdir, args.workers, args.small_timeout)
    if big:
        print(f"\n大档 {len(big)} case：{args.big_workers} 并发，上限 {args.big_timeout}s", flush=True)
        _run_group(big, outdir, args.big_workers, args.big_timeout)

    print(f"\n完成。结果 JSON 与 summary.csv 已写入 {outdir.resolve()}")


if __name__ == "__main__":
    main()
