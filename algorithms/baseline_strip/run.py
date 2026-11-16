#!/usr/bin/env python3
"""Strip-based route planning baseline (faithful single-floor SRP core).

Reproduces the Strip-based Route Planning framework of Shi et al. (ICDE 2023)
on regular grid layers: strip aggregation, 1-D time-space segments,
geometric collision judgement, backtracking waiting and slope-based indexing.
Cross-floor transfers are the user-approved simplification (direct transfer
edges with FIFO waiting).  See core.py and README.md.
"""

from baseline_utils import arguments, emit, load_case
from core import strip_schedule


def main() -> int:
    args = arguments("Strip-based Route Planning baseline (faithful SRP)")
    building, robots, tasks, config = load_case(args)
    result = strip_schedule(building, robots, tasks, config)
    emit(args, result)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
