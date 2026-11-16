# MPSC-HSF

Code, datasets and evaluation scripts for our study on **multi-floor multi-robot
batch task scheduling** — scheduling a batch of delivery tasks onto a fleet of
robots that move through a multi-floor building using corridors, stairs and
elevators.

The repository contains

* the proposed scheduler and six adapted baselines,
* a synthetic multi-floor benchmark (756 instances) and a real-world benchmark
  built from a campus hospital plus two public warehouse layouts (204 instances),
* a small runner that reproduces the experiments (standard library only, no
  external solver required).

---

## Repository layout

```
.
├── algorithms/                 # one directory per scheduler, each with run.py
│   ├── multifloor_scheduler/           # proposed algorithm (+ docs, tests)
│   ├── multifloor_scheduler_nearest/   # internal control: nearest-robot assignment
│   ├── baseline_wait_only/             # baseline: wait-only assignment
│   ├── baseline_strip/                 # baseline: strip-based task partitioning
│   ├── baseline_mapd/                  # baseline: MAPD token passing
│   ├── baseline_cbs/                   # baseline: conflict-based search
│   └── baseline_munes/                 # baseline: MuNES
├── benchmark/
│   ├── dataset.py              # dataset access (manifests, maps, case assembly)
│   └── runner.py               # batch execution engine (resume-safe, tiered concurrency)
├── dataset/
│   ├── synthetic/              # 756 synthetic instances (see dataset/synthetic/README.md)
│   └── real/                   # 204 real-world instances (see dataset/real/README.md)
├── experiments/
│   ├── run_benchmark.py        # single entry point for all experiments
│   └── legacy/                 # original server-side batch scripts, kept for provenance
├── tools/
│   ├── verify_dataset.py       # integrity check of the released datasets
│   └── materialize_dataset.py  # expand skeletons into self-contained case files
└── docs/                       # design notes (Chinese), server guide
```

Every scheduler exposes the same CLI, which is what makes the comparison a
controlled one — the algorithms differ only in how they assign and route tasks:

```
python algorithms/<algo>/run.py --mode generic --input <case>.json --output <result>.json
```

---

## Datasets

Both datasets live under `dataset/` and follow one convention: the building
geometry is stored **once** per map in `maps/`, and each `cases/<case_id>.json.gz`
is a light skeleton holding the robot start positions, the task set and a
`map_ref` pointing at its map. `benchmark/dataset.py` assembles the two into the
single-file payload the schedulers consume, so the duplicated 20–70 MB grid is
never stored per case. Everything is gzipped (JSON grids compress ~35×).

| dataset | folder | instances | building sizes | fleet sizes | load ratios |
|---|---|---|---|---|---|
| synthetic | `dataset/synthetic` | 756 | 7 floor counts (2 – 100) | 9 fleet sizes (1 – 4096) | 0.5 / 1.0 / 1.5 / 2.0 |
| real | `dataset/real` | 204 | 7 / 8 / 10 floors | 16 – 1024 | 0.5 / 1.0 / 1.5 / 2.0 |

Detailed parameter tables, map statistics and the case naming scheme are in
[`dataset/synthetic/README.md`](dataset/synthetic/README.md) and
[`dataset/real/README.md`](dataset/real/README.md).

Check the download before running anything:

```bash
python tools/verify_dataset.py
```

---

## Quick start

Python 3.10+ with no third-party packages.

```bash
# 1. sanity check the datasets (counts, map references)
python tools/verify_dataset.py

# 2. what would run?
python experiments/run_benchmark.py --dataset synthetic --algo main --limit 3 --dry-run

# 3. smoke test: three instances of the synthetic benchmark
python experiments/run_benchmark.py --dataset synthetic --algo main --limit 3 --workers 2

# 4. full synthetic benchmark, small/large tiers at 12/4 workers
python experiments/run_benchmark.py --dataset synthetic --algo main --workers 12 --big-workers 4

# 5. all baselines on the real benchmark
python experiments/run_benchmark.py --dataset real --algo all --workers 2 --big-workers 4
```

Useful switches: `--shard i/n` (round-robin split across machines),
`--cases <id> ...`, `--scenes real wh8 wh10` (real dataset),
`--big-r` (fleet size at which the large tier starts, default 4096 for synthetic),
`--timeout` / `--big-timeout` (per-case wall-clock limits).

Results land in `results/<dataset>/<algo>/`:

* `<case_id>.result.json` — full scheduler output plus the instance metadata,
* `summary.csv` — one row per finished case (`makespan`, `sum_completion_time`,
  `total_wait_time`, `final_conflict_count`, `assigned_count`, …).

The runner is resume-safe: a case whose `.result.json` exists is skipped, so an
interrupted run can simply be restarted. Cases that exceed their timeout are
recorded as `TIMEOUT` and can be re-run with a larger `--timeout`.

If you need self-contained case files (for example for the original scripts in
`experiments/legacy/`), expand them once:

```bash
python tools/materialize_dataset.py --dataset synthetic --out work/synthetic
```

---

## Algorithms

| key | scheduler | role |
|---|---|---|
| `main` | `multifloor_scheduler` | proposed algorithm |
| `nearest` | `multifloor_scheduler_nearest` | internal control (nearest-robot assignment) |
| `wait` | `baseline_wait_only` | wait-only assignment |
| `strip` | `baseline_strip` | strip-based partitioning (Shi et al.) |
| `mapd` | `baseline_mapd` | MAPD token passing (Ma et al.) |
| `cbs` | `baseline_cbs` | conflict-based search (Sharon et al.) |
| `munes` | `baseline_munes` | MuNES (Jung et al.) |

Each baseline keeps its original assignment logic and is adapted only where the
multi-floor setting requires it (elevator/stair resource model, capacity and
timing). The adaptations are documented per algorithm in its `README.md` and in
[`docs/baseline_adaptation_note_zh.md`](docs/baseline_adaptation_note_zh.md)
(Chinese).

---

## Evaluation protocol

All schedulers are timed on the same physical cost model, so the numbers are
comparable:

* corridor travel: path length / `speed_xy` (1.5 m/s),
* stair travel: `floor_height × |Δfloor| / stair_speed` (3.5 m, 0.5 m/s),
* elevator ride: `board_time + floor_time × |Δfloor|` (4.5 s, 1.75 s),
* elevator/stair capacity is 1, i.e. resources are exclusive,
* 75 % of the fleet is wheeled and therefore restricted to elevators,
* half of the task stream is cross-floor.

Per-instance work is reported as `makespan`, `sum_completion_time` and
`total_wait_time`; every configuration is averaged over three seeds.

---

## Reproducibility notes

* Standard library only; results in the paper were produced with Python 3.10.12.
* The synthetic benchmark is generated by
  `dataset/synthetic/generate_synthetic_cases.py` (fixed seeds, deterministic),
  so the released cases can be regenerated instead of downloaded.
* `experiments/legacy/` holds the original server-side scripts
  (`server_run.py`, `server_run_v31.py`, `run_batch.py`, `run_all_real.py`)
  exactly as they were used for the reported runs. `experiments/run_benchmark.py`
  is the cleaned-up entry point over the same algorithms and datasets.
* The hospital layout is proprietary to the campus deployment; only aggregate
  statistics are released with the real dataset.

---

## Contact

Questions about the code or the datasets: open an issue on this repository.
