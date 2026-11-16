# Real-world benchmark

Three scenes, all multi-floor graphs with floor edges, elevator shafts and
stair shafts:

| scene | building | floors | instances | fleet sizes |
|---|---|---|---|---|
| `real` | campus hospital outpatient building | 7 | 60 | 16, 64, 128, 256, 512 |
| `wh8` | warehouse layout (Moving AI benchmark) | 8 | 72 | 16, 64, 128, 256, 512, 1024 |
| `wh10` | warehouse layout (Moving AI benchmark) | 10 | 72 | 16, 64, 128, 256, 512, 1024 |

Each scene is instantiated with `rho ∈ {0.5, 1.0, 1.5, 2.0}` × three seeds
(`17, 29, 43`), giving **204 instances** (60 + 72 + 72).

The hospital was surveyed floor by floor by the deployment team. Its geometry is
proprietary to the campus and cannot be redistributed, so `maps/real_1-7F.json`
contains the surveyed graph used in our experiments while the published
statistics are aggregate only (see the paper's dataset-composition table).
The two warehouse maps are derived from the public Moving AI benchmark suite:

> N. R. Sturtevant, "Benchmarks for grid-based pathfinding," *IEEE Trans.
> Comput. Intell. AI Games*, vol. 4, no. 2, pp. 144–148, 2012.

Timing and capacity follow the same deployment values as the synthetic study
(corridor 1.5 m/s, stairs 0.5 m/s with 3.5 m floors, elevator 4.5 s boarding +
1.75 s per floor, capacity 1, 75 % wheeled robots, half of the tasks cross-floor).

## Files

```
real/
├── manifests/case_manifest_real.json    # 204 entries: case_id, scene, robots, tasks, load_ratio, seed, map_ref
├── cases/<case_id>.json.gz              # skeleton: robots, tasks, config, map_ref
├── maps/real_1-7F.json.gz               # hospital, 7 floors
├── maps/movingai_warehouse_8F.json.gz   # wh8
├── maps/movingai_warehouse_10F.json.gz  # wh10
├── maps/topo_points.json                # floor-transfer anchor points
├── build_real_map.py                    # rebuilds the hospital graph from the survey
├── build_warehouse_maps.py              # converts Moving AI maps into multi-floor graphs
└── generate_real_cases.py               # regenerates the 204 skeletons
```

## Sanity check

```bash
python tools/verify_dataset.py --dataset real
```
