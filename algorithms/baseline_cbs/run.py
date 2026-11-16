#!/usr/bin/env python3
"""Faithful Conflict-Based Search (CBS) baseline.

High-level Constraint Tree search with low-level constrained shortest paths
over the unified continuous-time multi-floor resource model.  See core.py
for the algorithm description and the README for the correspondence with
Sharon et al. (2015).
"""

from baseline_utils import arguments, emit, load_case
from core import cbs_schedule


def main() -> int:
    args = arguments("Conflict-Based Search baseline (faithful CBS)")
    building, robots, tasks, config = load_case(args)
    if args.max_iterations is not None:
        config.max_iterations = args.max_iterations
    result = cbs_schedule(building, robots, tasks, config)
    emit(args, result)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
