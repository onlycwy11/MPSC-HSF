#!/usr/bin/env python3
"""Consistency checks for the released datasets.

Verifies that every case listed in a manifest has a case file, that every
``map_ref`` resolves, and prints the per-tier case counts.  Exit code is non-zero
when something is missing.

    python tools/verify_dataset.py                # both datasets
    python tools/verify_dataset.py --dataset real
"""

from __future__ import annotations

import argparse
import sys
from collections import Counter
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from benchmark import dataset as ds  # noqa: E402


def check(name: str) -> bool:
    root = ds.dataset_root(name)
    entries = ds.case_entries(name)
    ok = True

    missing_cases = []
    missing_maps = []
    for entry in entries:
        case_id = entry["case_id"]
        try:
            skeleton = ds.load_skeleton(case_id, name)
        except FileNotFoundError:
            missing_cases.append(case_id)
            continue
        if skeleton.get("map") is None:
            map_ref = skeleton.get("map_ref") or f"maps/{entry.get('map_key')}.json"
            if not ds.map_exists(map_ref, name):
                missing_maps.append(f"{case_id} -> {map_ref}")

    by_robots = Counter(entry.get("robots") for entry in entries)
    by_floor = Counter(entry.get("floors") for entry in entries)
    by_scene = Counter(entry.get("scene") for entry in entries)

    print(f"\n[{name}] {len(entries)} cases in manifest, {root}")
    print("  by robots:", dict(sorted(by_robots.items())))
    if any(entry.get("floors") for entry in entries):
        print("  by floors:", dict(sorted(by_floor.items())))
    if any(entry.get("scene") for entry in entries):
        print("  by scene: ", dict(sorted(by_scene.items())))
    if missing_cases:
        ok = False
        print(f"  MISSING CASE FILES ({len(missing_cases)}):", missing_cases[:10])
    if missing_maps:
        ok = False
        print(f"  UNRESOLVED map_ref ({len(missing_maps)}):", missing_maps[:10])
    if ok:
        print("  OK: all case files and maps present")
    return ok


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--dataset", choices=sorted(ds.DATASETS), default=None)
    args = parser.parse_args()

    names = [args.dataset] if args.dataset else sorted(ds.DATASETS)
    ok = all(check(name) for name in names)
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
