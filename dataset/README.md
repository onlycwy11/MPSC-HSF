# Datasets

Two benchmarks share one layout:

```
<dataset>/
├── manifest.json            # synthetic: merged case list
├── manifests/*.json         # real: case_manifest_real.json
├── maps/<map>.json[.gz]     # building geometry, stored once per map
└── cases/<case_id>.json[.gz]# per-case skeleton: robots, tasks, map_ref, config
```

A case file is intentionally small: it carries the fleet state, the task batch
and a `map_ref` that points at the building it runs in. `benchmark/dataset.py`
assembles map + case into the single-file payload the schedulers consume, and
`tools/materialize_dataset.py` writes those files out if you need them on disk.

Case identifiers encode the whole configuration:

```
R<robots>_M<tasks>_rho<load>_F<floors>_S<seed>        # synthetic
<scene>_R<robots>_M<tasks>_rho<load>_S<seed>          # real
```

`M` is the number of tasks in the batch: `M = max(1, round(R * rho))`, which
equals `ceil(R * rho)` for every configuration in this benchmark.

Verify the download with:

```bash
python tools/verify_dataset.py            # both datasets
python tools/verify_dataset.py --dataset synthetic
```
