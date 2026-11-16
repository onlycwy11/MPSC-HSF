# Legacy server-side scripts

These are the batch scripts exactly as they were used for the reported runs on
the 152-core server, kept for provenance. They predate the cleaned-up entry
point `experiments/run_benchmark.py` and assume the original directory layout:

```
<run root>/
├── server_run.py, server_run_v31.py, run_batch.py
├── multifloor_scheduler/        (main algorithm)
├── algorithms/baseline_*/       (baselines used by run_batch.py)
├── dataset_v2/, dataset_v3/, dataset_v3.1/
└── dataset_real_v1/             (run_all_real.py lives here)
```

| script | purpose |
|---|---|
| `server_run.py` | main algorithm over the synthetic grid, small/large-tier concurrency, per-tier timeouts, `--shard i/n` |
| `server_run_v31.py` | the `R = 128` top-up batch after the timeout bump |
| `run_batch.py` | any algorithm in `algorithms/`, in-process, manifest- or case-file driven |
| `run_all_real.py` | the 204 real instances, assembles skeleton cases at run time |

Path notes for reuse in this repository:

* `server_run.py` reads `dataset_v3/` next to the script; point `V3_DIR` at
  `dataset/synthetic` (and use `tools/materialize_dataset.py` to produce plain
  case files, since this script expects `cases/<id>.json` with an embedded map).
* `run_all_real.py` resolves schedulers through `--sched-root`; pass the
  repository root (`--sched-root .`) so it finds `algorithms/`.
  Its `cases/` and `manifests/` are read relative to the script directory.

Differences from `experiments/run_benchmark.py` are limited to dataset plumbing
and result aggregation; the schedulers, the case payloads and the metrics are
identical.
