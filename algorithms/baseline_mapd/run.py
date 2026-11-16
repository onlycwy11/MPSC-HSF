#!/usr/bin/env python3
"""Faithful Token Passing (TP) baseline.

Tasks are processed in release-time order by a global token.  For each task,
every idle type-matching robot plans pickup -> delivery against the
reservation table of already-assigned robots (token state); the robot with
the earliest completion time receives the task and its collision-avoiding
path is added to the reservation.  See core.py and the README for the
correspondence with Ma et al. (2017).
"""

from baseline_utils import arguments, emit, load_case
from core import tp_schedule


def main() -> int:
    args = arguments("Token Passing MAPD baseline (faithful TP)")
    building, robots, tasks, config = load_case(args)
    result = tp_schedule(building, robots, tasks, config)
    emit(args, result)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
