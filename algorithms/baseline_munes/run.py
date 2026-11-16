#!/usr/bin/env python3
"""MuNES baseline (faithful single-robot elevator-waiting-aware planning +
greedy assignment + first-come-first-served waiting).

Corresponds to Jung et al. (2024), arXiv:2402.04535.  See core.py and README.md
for the exact correspondence and the simplification rationale.
"""

from baseline_utils import arguments, emit, load_case
from core import munes_schedule


def main() -> int:
    args = arguments("MuNES baseline (faithful elevator-waiting-aware planning)")
    building, robots, tasks, config = load_case(args)
    result = munes_schedule(building, robots, tasks, config)
    emit(args, result)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
