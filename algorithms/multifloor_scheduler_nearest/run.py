#!/usr/bin/env python3
"""CLI for the nearest-vertical-first (水平近路优先) scheduler variant."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

from core import generate_virtual, load_json, load_real_inputs, parse_payload, read_sqlite_reservations, schedule


def main() -> int:
    parser = argparse.ArgumentParser(description="Multi-floor robot batch task scheduler (nearest-vertical-first variant)")
    parser.add_argument("--mode", choices=["virtual", "real", "generic"], default="virtual")
    parser.add_argument("--input", help="JSON input/config path")
    parser.add_argument("--map", dest="map_path", help="Map JSON for real mode")
    parser.add_argument("--db", dest="db_path", help="SQLite database for real mode")
    parser.add_argument("--output", help="Output JSON path; defaults to stdout")
    args = parser.parse_args()

    if args.mode == "virtual":
        payload = load_json(args.input) if args.input else {}
        payload = generate_virtual(payload)
        building, robots, tasks, config = parse_payload(payload)
        reservations = payload.get("resource_schedule", [])
    elif args.mode == "real":
        if not args.map_path or not args.db_path:
            parser.error("real mode requires --map and --db")
        building, robots, tasks, config = load_real_inputs(args.map_path, args.db_path)
        reservations = read_sqlite_reservations(args.db_path)
    else:
        if not args.input:
            parser.error("generic mode requires --input")
        payload = load_json(args.input)
        building, robots, tasks, config = parse_payload(payload)
        reservations = payload.get("resource_schedule", [])

    result = schedule(building, robots, tasks, config, reservations)
    rendered = json.dumps(result, ensure_ascii=False, indent=2)
    if args.output:
        Path(args.output).write_text(rendered + "\n", encoding="utf-8")
    else:
        print(rendered)
    return 0


if __name__ == "__main__":
    sys.exit(main())
