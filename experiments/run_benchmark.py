#!/usr/bin/env python3
"""Run one or more schedulers over the synthetic or real benchmark.

Examples
--------
Smoke test on a couple of small cases::

    python experiments/run_benchmark.py --dataset synthetic --algo main --limit 2

Main algorithm on the synthetic benchmark, small/large concurrency 12/4::

    python experiments/run_benchmark.py --dataset synthetic --algo main \
        --workers 12 --big-workers 4 --big-r 4096

All six baselines plus the two internal controls on the real scenes::

    python experiments/run_benchmark.py --dataset real --algo all --workers 2 --big-workers 4

One shard of the synthetic benchmark on a shared machine::

    python experiments/run_benchmark.py --dataset synthetic --algo main --shard 1/3

Results are written to ``results/<dataset>/<algo>/<case_id>.result.json`` with
one ``summary.csv`` per dataset/algo pair.  Finished cases are skipped, so the
command can be killed and restarted at any time.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from benchmark import dataset as ds  # noqa: E402
from benchmark import runner  # noqa: E402


def shard_split(entries, shard: str):
    index, total = (int(part) for part in shard.split("/"))
    return [entry for position, entry in enumerate(entries)
            if position % total == index - 1]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--dataset", choices=sorted(ds.DATASETS), default="synthetic")
    parser.add_argument("--algo", nargs="+", default=["main"],
                        help="main | nearest | wait | strip | mapd | cbs | munes | all")
    parser.add_argument("--cases", nargs="+", default=[], help="exact case_id(s)")
    parser.add_argument("--scenes", nargs="+", default=[], help="real dataset scenes: real wh8 wh10")
    parser.add_argument("--shard", default=None, help="round-robin shard i/n, e.g. 1/3")
    parser.add_argument("--limit", type=int, default=0, help="only the first N pending cases")
    parser.add_argument("--workers", type=int, default=1, help="concurrency for small cases")
    parser.add_argument("--big-workers", type=int, default=4, help="concurrency for large cases")
    parser.add_argument("--big-r", type=int, default=4096,
                        help="robots >= this value use the large tier (synthetic default 4096)")
    parser.add_argument("--timeout", type=float, default=86400.0, help="per-case timeout, small tier")
    parser.add_argument("--big-timeout", type=float, default=28800.0, help="per-case timeout, large tier")
    parser.add_argument("--out", default="results", help="results root directory")
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()

    algos = list(runner.ALGORITHMS) if "all" in args.algo else args.algo
    for algo in algos:
        if algo not in runner.ALGORITHMS:
            raise SystemExit(f"unknown algo '{algo}', expected one of {sorted(runner.ALGORITHMS)}")

    entries = ds.case_entries(args.dataset)
    if args.scenes:
        entries = [entry for entry in entries if entry.get("scene") in args.scenes]
    if args.cases:
        wanted = set(args.cases)
        entries = [entry for entry in entries if entry["case_id"] in wanted]
    if args.shard:
        entries = shard_split(entries, args.shard)
    if args.limit:
        entries = entries[: args.limit]

    if args.dry_run:
        print(f"{args.dataset}: {len(entries)} case(s) | algos={algos}")
        for entry in entries[:20]:
            print(" ", entry["case_id"])
        if len(entries) > 20:
            print(f"  ... and {len(entries) - 20} more")
        return 0

    results_root = Path(args.out)
    if not results_root.is_absolute():
        results_root = ds.REPO_ROOT / results_root

    for algo in algos:
        runner.run_split(
            dataset=args.dataset,
            algo=algo,
            entries=entries,
            workers=args.workers,
            big_workers=args.big_workers,
            big_r=args.big_r,
            timeout=args.timeout,
            big_timeout=args.big_timeout,
            results_root=results_root,
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
