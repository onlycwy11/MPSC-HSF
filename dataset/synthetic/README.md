# Synthetic benchmark

A configurable multi-floor building simulator sweeps fleet size, building height
and workload ratio.

| axis | values | count |
|---|---|---|
| robots `R` | 1, 16, 64, 128, 256, 512, 1024, 2048, 4096 | 9 |
| floors `F` | 2, 4, 8, 16, 32, 64, 100 | 7 |
| load ratio `rho` | 0.5, 1.0, 1.5, 2.0 | 4 |
| seed `S` | 17, 29, 43 | 3 |

Full grid: **9 × 7 × 4 × 3 = 756 instances** (84 per fleet size).
Tasks per batch: `M = max(1, round(R * rho))`.

> This is the merged release of the two batches used during development
> (`v3` and `v3.1`). The `R = 128` tier was originally shipped as the separate
> `v3.1` batch; its cases are byte-identical to the `R128` cases of the `v3`
> batch, so the two are merged here without duplicates. Case files keep their
> original names, and `manifest.json` is the union of both manifests.

## Fixed generation parameters

| parameter | value |
|---|---|
| wheeled robots | 0.75 (wheeled robots can use elevators only) |
| cross-floor tasks | 0.5 |
| corridor speed `speed_xy` | 1.5 m/s |
| stair speed `stair_speed` | 0.5 m/s |
| floor height `floor_height` | 3.5 m |
| elevator boarding `elevator_board_time` | 4.5 s |
| elevator per-floor `elevator_floor_time` | 1.75 s |
| elevator / stair capacity | 1 (exclusive resources) |
| elevators = stairs | `ceil(F / 8)` per balanced building |
| task floors | uniform over floors |
| robot floors | balanced over floors |
| map grid spacing | 2.5 m |
| map grid size | 40 × 40 for `F <= 8`, 32 × 32 for `F > 8` |

## Maps

`maps/` holds one map per building height plus a few facility-variant maps used
by earlier pilots:

| map | floors | note |
|---|---|---|
| `L-2F-balanced`, `L-4F-balanced`, `L-8F-balanced` | 2 / 4 / 8 | low-rise family |
| `M-2F-balanced` … `M-100F-balanced` | 2 / 4 / 8 / 16 / 32 / 64 / 100 | main family |
| `M-16F-elevator_scarce`, `M-16F-stair_scarce` | 16 | facility variants (not part of the 756) |

`L`/`M` differ in corridor width and shaft placement; the released 756 cases use
the maps referenced by their `map_key`.

## Files

```
synthetic/
├── manifest.json                       # 756 entries: case_id, robots, tasks, load_ratio, floors, seed, map_key
├── cases/<case_id>.json.gz             # skeleton: robots, tasks, config, map_ref
├── maps/<map>.json.gz
└── generate_synthetic_cases.py         # deterministic generator (reproduces every case)
```

To regenerate a case instead of reading it, run the generator with
`--materialize-case <case_id>`; it uses the same seeds and fixed parameters.

## Sanity check

```bash
python tools/verify_dataset.py --dataset synthetic
```
