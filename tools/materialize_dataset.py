#!/usr/bin/env python3
"""Expand the released dataset into a plain working tree.

The repository stores maps once and cases as light skeletons (``.json.gz``).
Some tools expect self-contained single-file cases with an embedded map; this
utility writes them out:

    python tools/materialize_dataset.py --dataset synthetic --out work/synthetic
    python tools/materialize_dataset.py --dataset real --out work/real --cases real_R64_M64_rho1p0_S17

The output directory contains ``cases/*.json`` (embedded map), ``maps/*.json``
(decompressed) and the manifest, i.e. exactly the layout the runners in
``experiments/legacy/`` expect.
"""

from __future__ import annotations

import argparse
import gzip
import json
import shutil
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from benchmark import dataset as ds  # noqa: E402


def write_map_files(name: str, out_root: Path) -> int:
    source_dir = ds.dataset_root(name) / "maps"
    target_dir = out_root / "maps"
    target_dir.mkdir(parents=True, exist_ok=True)
    written = 0
    for source in sorted(source_dir.iterdir()):
        if source.suffix == ".gz":
            target = target_dir / source.stem
            with gzip.open(source, "rt", encoding="utf-8") as fh:
                target.write_text(fh.read(), encoding="utf-8")
        else:
            target = target_dir / source.name
            shutil.copy2(source, target)
        written += 1
    return written


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--dataset", choices=sorted(ds.DATASETS), required=True)
    parser.add_argument("--out", required=True, help="output directory")
    parser.add_argument("--cases", nargs="+", default=[], help="subset of case ids")
    args = parser.parse_args()

    out_root = Path(args.out).resolve()
    cases_dir = out_root / "cases"
    cases_dir.mkdir(parents=True, exist_ok=True)

    entries = ds.case_entries(args.dataset)
    if args.cases:
        wanted = set(args.cases)
        entries = [entry for entry in entries if entry["case_id"] in wanted]

    maps = write_map_files(args.dataset, out_root)
    count = 0
    for entry in entries:
        ds.materialize(entry["case_id"], args.dataset, cases_dir / f"{entry['case_id']}.json")
        count += 1

    manifest_source = ds.dataset_root(args.dataset)
    for candidate in (manifest_source / "manifest.json",
                      *(manifest_source / "manifests").glob("*.json")):
        if candidate.exists():
            shutil.copy2(candidate, out_root / candidate.name)

    print(f"materialized {count} case(s) and {maps} map(s) -> {out_root}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
