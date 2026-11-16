#!/usr/bin/env python3
"""Build the ICDE real-world dataset map (Sandun hospital campus, 1F..7F).

Inputs (raw materials, kept outside this package):
  - 1F:  三墩门诊/一楼/sdmzl_zg_floor1D.pgm  1689x2841 @0.05, origin [-10.5, -42.7]
  - 2F:  三墩门诊/二楼/sdmz_floor2.pgm       1857x2290 @0.05, origin [-30.4, -16.1]
  - 3F:  sdmz_floor3.pgm         1429x1924 @0.05, origin [-1.61, -18.1]
         (三墩门诊/三楼/sdmz_floor3.yaml identical: verified)
  - 4F:  创投地图已处理/已转/四楼已编辑.pgm  658x1074 @0.05, origin ASSUMED [-20.3,-44.8]
         (from 0528-6.yaml, whose own pgm is missing; same-batch assumption)
  - 5F:  创投地图已处理/已转/五楼_1.pgm     658x1074 @0.05, origin ASSUMED [-20.3,-44.8]
  - 6F:  floot_1.pgm              480x905  @0.05, origin [-10.3, -19.2]
         floot4_inner.pgm         647x884  @0.05, origin [-10.4, -6.84]  (merged into 6F)
  - 7F:  floot_2.pgm              879x512  @0.05, origin [-11.6, -15.7]
  - stairs_1_topo.yaml            staircase 4F<->5F (3D polyline length = ~14.88 m)

Model (matches the elevator/stair semantics of the main algorithm):
  - 1 m grid cells; a cell is blocked when >= 50% of its pixels are occupied.
  - Elevator shaft: pierces 1F..7F at world XY (-0.03, -0.16) (3F topo node p7 "x01电梯";
    verified: 1F topo p1=(0,0) and 2F p1=(0,0) sit in the same shaft column).
    Vertical elevator edges link adjacent floors (algorithm auto-completes direct
    multi-floor arcs), length = floor height 3.5 m, resource elevator-0, capacity 1.
  - Staircase shaft: pierces 1F..7F at world XY (5.55, 13.01) (4F topo node p6),
    stair-0, length = 3D polyline length of stairs_1 topology (~14.88 m).
  - floor order: 1F -> floor 0 ... 7F -> floor 6 (z = 0 / 3.5 / ... / 21 m).

Outputs:
  - maps/real_1-7F.json      : {nodes, edges, resources, resource_metadata, meta}
  - maps/topo_points.json    : {floorN: {topo_name: node_id, ...}}  (navigable points)
"""

from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Any, Dict, List, Tuple

import numpy as np

ROOT = Path(__file__).resolve().parent
OUT_MAPS = ROOT / "maps"
RAW = Path(r"C:\Users\ONLY\Desktop\地图")          # desktop materials
SD = RAW / "三墩门诊"                                # Sandun outpatient 1F/2F/3F
RAR = RAW / "创投地图已处理" / "已转"               # rar-extracted 4F/5F (no topo)

RES = 0.05          # m per pixel
FREE_TH = 0.25      # below -> free
OCC_TH = 0.65       # above -> occupied
GRID = 1.0          # m per cell
OCC_RATIO_BLOCK = 0.5

FLOOR_HEIGHT = 3.5
STAIR_LEN = 14.88   # 3D polyline length of stairs_1 topology (4F p6 <-> 5F p1)

# world coordinates of the vertical resources (both shafts pierce 1F..7F)
ELEVATOR_XY = (-0.03, -0.16)         # 3F topo p7 "x01电梯"; 1F p1=(0,0)/2F p1=(0,0) in same column
STAIR_XY = (5.55, 13.01)             # 4F topo p6

# layer definitions: floor_idx -> (list of (pgm_path, origin_x, origin_y), grid origin, n_ix, n_iy)
LAYERS: Dict[int, Dict[str, Any]] = {
    0: {  # 1F (outpatient lobby floor, has topo)
        "name": "1F",
        "pgms": [(SD / "一楼" / "sdmzl_zg_floor1D.pgm", -10.5, -42.7)],
        "x0": -11.0, "y0": -43.0,
        "nx": 85, "ny": 143,
    },
    1: {  # 2F (outpatient floor, has topo)
        "name": "2F",
        "pgms": [(SD / "二楼" / "sdmz_floor2.pgm", -30.4, -16.1)],
        "x0": -31.0, "y0": -17.0,
        "nx": 94, "ny": 116,
    },
    2: {  # 3F
        "name": "3F",
        "pgms": [(RAW / "sdmz_floor3.pgm", -1.61, -18.1)],
        "x0": -2.0, "y0": -19.0,
        "nx": 73, "ny": 98,
    },
    3: {  # 4F (RAR version, no topo; origin assumed from 0528-6.yaml)
        "name": "4F",
        "pgms": [(RAR / "四楼已编辑.pgm", -20.3, -44.8)],
        "x0": -21.0, "y0": -45.0,
        "nx": 34, "ny": 54,
    },
    4: {  # 5F (RAR version, no topo; origin assumed from 0528-6.yaml)
        "name": "5F",
        "pgms": [(RAR / "五楼_1.pgm", -20.3, -44.8)],
        "x0": -21.0, "y0": -45.0,
        "nx": 34, "ny": 54,
    },
    5: {  # 6F (desktop 4F, has topo)
        "name": "6F",
        "pgms": [(RAW / "floot_1.pgm", -10.3, -19.2),
                 (RAW / "floot4_inner.pgm", -10.4, -6.84)],
        "x0": -11.0, "y0": -20.0,
        "nx": 34, "ny": 59,
    },
    6: {  # 7F (desktop 5F, has topo)
        "name": "7F",
        "pgms": [(RAW / "floot_2.pgm", -11.6, -15.7)],
        "x0": -12.0, "y0": -16.0,
        "nx": 46, "ny": 27,
    },
}


def read_pgm(path: Path) -> Tuple[np.ndarray, int, int, int]:
    """Return occupancy grid as float array (0 free .. 255 occupied), plus dims."""
    data = path.read_bytes()
    pos = 0
    n = len(data)

    def token() -> bytes:
        nonlocal pos
        while pos < n and data[pos] in b" \t\r\n":
            pos += 1
        if pos < n and data[pos:pos + 1] == b"#":
            while pos < n and data[pos] not in b"\r\n":
                pos += 1
            return token()
        start = pos
        while pos < n and data[pos] not in b" \t\r\n":
            pos += 1
        return data[start:pos]

    magic = token()
    if magic not in (b"P5", b"P2"):
        raise ValueError(f"unsupported PGM magic {magic} in {path}")
    w = int(token()); h = int(token()); maxv = int(token())
    while pos < n and data[pos] in b" \t\r\n":
        pos += 1
    if magic == b"P5":
        raw = np.frombuffer(data[pos:pos + w * h], dtype=np.uint8).reshape(h, w)
    else:
        vals = np.array([int(x) for x in data[pos:].split()], dtype=np.float32)
        raw = vals[: w * h].reshape(h, w)
    return raw.astype(np.float64), w, h, maxv


def pgm_to_world(pgm: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """Return (occupancy mask, free mask) for world pixels.

    ROS occupancy-grid semantics (negate=0): white(255)=free, black(0)=occupied.
    occ probability = (255 - pixel)/255;  occ if p>0.65 (pixel<~90), free if p<0.25 (pixel>~191).
    """
    occ = pgm < (OCC_TH * 255)          # dark pixels -> occupied
    free = pgm > (255 - FREE_TH * 255)  # bright pixels -> free (0.75*255=191)
    return occ, free


def world_grid(layer: Dict[str, Any]) -> Dict[str, Any]:
    """Rasterize the layer's PGM(s) onto the 1 m world grid.

    When a layer has several PGM files, the FIRST one is the primary map and wins
    in overlapping cells; later files only fill cells the primary map does not cover
    (e.g. floot4_inner fills the part of 6F missing from floot_1).
    """
    nx, ny = layer["nx"], layer["ny"]
    x0, y0 = layer["x0"], layer["y0"]
    ratio = np.full((ny, nx), np.nan, dtype=np.float64)
    covered = np.zeros((ny, nx), dtype=bool)

    for pgm_path, ox, oy in layer["pgms"]:
        pgm, w, h, _ = read_pgm(pgm_path)
        occ, _ = pgm_to_world(pgm)
        px = np.broadcast_to(np.arange(w)[None, :], (h, w)).ravel()
        py = np.broadcast_to(np.arange(h)[:, None], (h, w)).ravel()
        wx = ox + px * RES
        wy = oy + (h - 1 - py) * RES
        ix = np.floor((wx - x0) / GRID).astype(np.int64)
        iy = np.floor((wy - y0) / GRID).astype(np.int64)
        valid = (ix >= 0) & (ix < nx) & (iy >= 0) & (iy < ny)
        g_flat = iy[valid] * nx + ix[valid]
        occ_vec = occ.ravel()[valid]
        pix = np.ones(valid.sum(), dtype=np.float64)
        occ_cnt = np.zeros(nx * ny, dtype=np.float64)
        pix_cnt = np.zeros(nx * ny, dtype=np.float64)
        np.add.at(pix_cnt, g_flat, pix)
        np.add.at(occ_cnt, g_flat, occ_vec.astype(np.float64))
        with np.errstate(invalid="ignore"):
            r_this = np.where(pix_cnt > 0, occ_cnt / np.maximum(pix_cnt, 1e-9), np.nan)
        r_flat = ratio.ravel()
        cov_flat = covered.ravel()
        fill = ~cov_flat & ~np.isnan(r_this)
        r_flat[fill] = r_this[fill]
        cov_flat[fill] = True

    blocked = np.where(np.isnan(ratio), True, ratio >= OCC_RATIO_BLOCK)
    return {"blocked": blocked, "nx": nx, "ny": ny, "x0": x0, "y0": y0}


def xy_to_cell(layer_grid: Dict[str, Any], x: float, y: float) -> Tuple[int, int]:
    nx, ny = layer_grid["nx"], layer_grid["ny"]
    x0, y0 = layer_grid["x0"], layer_grid["y0"]
    ix = int(math.floor((x - x0) / GRID))
    iy = int(math.floor((y - y0) / GRID))
    ix = min(max(ix, 0), nx - 1)
    iy = min(max(iy, 0), ny - 1)
    return ix, iy


def nearest_free(layer_grid: Dict[str, Any], x: float, y: float, blocked: np.ndarray) -> Tuple[int, int]:
    nx, ny = layer_grid["nx"], layer_grid["ny"]
    ix, iy = xy_to_cell(layer_grid, x, y)
    if not blocked[iy, ix]:
        return ix, iy
    best = None
    best_d = float("inf")
    for r in range(1, max(nx, ny)):
        for dx in range(-r, r + 1):
            for dy in range(-r, r + 1):
                if max(abs(dx), abs(dy)) != r:
                    continue
                cx, cy = ix + dx, iy + dy
                if 0 <= cx < nx and 0 <= cy < ny and not blocked[cy, cx]:
                    d = dx * dx + dy * dy
                    if d < best_d:
                        best_d = d
                        best = (cx, cy)
        if best is not None:
            return best
    raise RuntimeError(f"no free cell near ({x},{y})")


def build() -> None:
    OUT_MAPS.mkdir(parents=True, exist_ok=True)

    grids: Dict[int, Dict[str, Any]] = {}
    for fidx, layer in LAYERS.items():
        grids[fidx] = world_grid(layer)

    # ---- locate resources (both shafts pierce every floor) ----
    res_cells: Dict[str, Dict[int, Tuple[int, int]]] = {}
    fallback_notes: Dict[str, List[str]] = {"elevator-0": [], "stair-0": []}
    for fidx, layer in LAYERS.items():
        g = grids[fidx]
        ix, iy = nearest_free(g, ELEVATOR_XY[0], ELEVATOR_XY[1], g["blocked"])
        res_cells.setdefault("elevator-0", {})[fidx] = (ix, iy)
        if (ix, iy) != xy_to_cell(g, ELEVATOR_XY[0], ELEVATOR_XY[1]):
            fallback_notes["elevator-0"].append(layer["name"])
        ix2, iy2 = nearest_free(g, STAIR_XY[0], STAIR_XY[1], g["blocked"])
        res_cells.setdefault("stair-0", {})[fidx] = (ix2, iy2)
        if (ix2, iy2) != xy_to_cell(g, STAIR_XY[0], STAIR_XY[1]):
            fallback_notes["stair-0"].append(layer["name"])

    print("resource cells:", {k: v for k, v in res_cells.items()})
    print("fallbacks:", fallback_notes)

    # ---- nodes & floor edges ----
    nodes: List[Dict[str, Any]] = []
    edges: List[Dict[str, Any]] = []
    resources: Dict[str, int] = {"elevator-0": 1, "stair-0": 1}
    resource_metadata: Dict[str, Dict[str, Any]] = {
        "elevator-0": {"kind": "elevator", "capacity": 1, "x": ELEVATOR_XY[0], "y": ELEVATOR_XY[1],
                       "shaft": "pierces 3F..7F", "note": "XY taken from 3F topo p7"},
        "stair-0": {"kind": "stair", "capacity": 1, "x": STAIR_XY[0], "y": STAIR_XY[1],
                    "shaft": "pierces 3F..7F",
                    "note": "XY taken from 4F topo p6; floors with off-map/blocked projection fall back to nearest free cell"},
    }

    free_cells: Dict[int, List[Tuple[int, int]]] = {}

    for fidx, layer in LAYERS.items():
        g = grids[fidx]
        blocked = g["blocked"]
        nx, ny = g["nx"], g["ny"]
        x0, y0 = g["x0"], g["y0"]
        lobby: Dict[Tuple[int, int], str] = {}
        for res_name, floors in res_cells.items():
            if fidx in floors:
                cx, cy = floors[fidx]
                lobby[(cx, cy)] = "elevator_lobby" if res_name.startswith("elevator") else "stair_lobby"

        id_map: Dict[Tuple[int, int], str] = {}
        for iy in range(ny):
            for ix in range(nx):
                if blocked[iy, ix]:
                    continue
                node_id = f"f{fidx}_x{ix}_y{iy}"
                id_map[(ix, iy)] = node_id
                free_cells.setdefault(fidx, []).append((ix, iy))
                nodes.append({
                    "id": node_id,
                    "floor": fidx,
                    "x": x0 + ix * GRID + GRID / 2,
                    "y": y0 + iy * GRID + GRID / 2,
                    "z": fidx * FLOOR_HEIGHT,
                    "node_type": lobby.get((ix, iy), "corridor"),
                })

        for (ix, iy), nid in id_map.items():
            for dx, dy in ((1, 0), (0, 1)):
                nb = (ix + dx, iy + dy)
                if nb in id_map:
                    nid2 = id_map[nb]
                    edges.append({"src": nid, "dst": nid2, "length": GRID, "kind": "floor"})
                    edges.append({"src": nid2, "dst": nid, "length": GRID, "kind": "floor"})

    # ---- topo points (real navigation points per floor; RAR 4F/5F have none) ----
    topo_points: Dict[int, List[Tuple[str, float, float, str]]] = {
        0: [  # 1F: node_type==3 navigable points (p1/p14/p18 are elevator/connect, excluded)
            ("p2", 3.69, -1.4, "room"), ("p3", 20.45, 2.75, "room"),
            ("p4", 29.04, -1.97, "room"), ("p5", 46.5, -0.12, "room"),
            ("p6", 67.11, 1.61, "room"), ("p7", 20.17, 52.9, "room"),
            ("p9", 17.79, 79.6, "room"), ("p10", 41.15, 52.6, "room"),
            ("p11", 43.18, 48.52, "room"), ("p12", 6.19, 1.35, "room"),
            ("p13", 41.08, 48.53, "room"), ("p15", 1.98, -0.41, "room"),
            ("p16", 20.84, 24.14, "room"), ("p17", 37.99, 53.4, "room")],
        1: [  # 2F: p1/p10 sit at the elevator column, excluded
            ("p2", 21.64, -0.66, "room"), ("p3", 21.76, 23.64, "room"),
            ("p4", 29.62, 26.46, "room"), ("p5", 13.73, 23.27, "room"),
            ("p6", 14.59, 52.85, "room"), ("p7", 13.92, 81.04, "room"),
            ("p8", 14.36, 95.01, "room"), ("p9", -2.67, 95.29, "room")],
        2: [("p2", 15.34, 0.82, "room"), ("p3", 16.53, 24.67, "room"),
            ("p4", 12.98, 24.93, "room"), ("p5", 11.94, 45.27, "room"),
            ("p6", 26.311, 26.141, "room"), ("p7", -0.03, -0.16, "elevator"),
            ("p8", 26.23, 31.39, "room")],
        # 3/4: RAR floors — no topo points (tasks fall back to corridor cells)
        5: [("p1", 0.16, -0.36, "room"), ("p2", 0.59, -0.37, "room"),
            ("p3", 8.85, 0.39, "room"), ("p4", 8.42, 12.26, "room"),
            ("p5", 5.61, 12.22, "room"), ("p6", 5.55, 13.01, "stair"),
            ("p7", -0.1, 11.78, "room")],
        6: [("p1", -0.12, -0.18, "elevator"), ("p2", 1.61, -0.26, "room"),
            ("p3", 1.34, 6.1, "room"), ("p4", -5.53, 5.93, "room"),
            ("p5", 12.91, 7.3, "room")],
    }

    topo_node: Dict[int, Dict[str, Tuple[int, int]]] = {}
    topo_seed_cells: List[Tuple[int, Tuple[int, int]]] = []
    for fidx, pts in topo_points.items():
        g = grids[fidx]
        d = {}
        for name, x, y, kind in pts:
            cx, cy = nearest_free(g, x, y, g["blocked"])
            d[name] = (cx, cy)
            topo_seed_cells.append((fidx, (cx, cy)))
        topo_node[fidx] = d

    seeds: List[Tuple[int, Tuple[int, int]]] = list(topo_seed_cells)
    for res_name, floors in res_cells.items():
        for fidx, c in floors.items():
            seeds.append((fidx, c))

    # ---- connectivity: keep every component containing a seed (resource or topo
    # point); drop seed-less noise islands. Floors are separate components because
    # vertical edges (elevator/stair) are not part of the BFS.
    from collections import deque

    visited: Dict[int, Dict[Tuple[int, int], bool]] = {
        fidx: {c: True for c in free_cells[fidx]} for fidx in free_cells
    }
    comp_of: Dict[Tuple[int, Tuple[int, int]], int] = {}
    seed_set = set(seeds)
    comp_has_seed: Dict[int, bool] = {}
    comp_id = 0
    for fidx in free_cells:
        for c in free_cells[fidx]:
            if (fidx, c) in comp_of:
                continue
            comp_id += 1
            has_seed = (fidx, c) in seed_set
            q = deque([(fidx, c)])
            comp_of[(fidx, c)] = comp_id
            while q:
                f2, c2 = q.popleft()
                ix2, iy2 = c2
                for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                    nb = (ix2 + dx, iy2 + dy)
                    if (f2, nb) in comp_of or nb not in visited[f2]:
                        continue
                    if (f2, nb) in seed_set:
                        has_seed = True
                    comp_of[(f2, nb)] = comp_id
                    q.append((f2, nb))
            comp_has_seed[comp_id] = has_seed

    keep: Dict[int, set] = {}
    for (fidx, c), cid in comp_of.items():
        if comp_has_seed.get(cid, False):
            keep.setdefault(fidx, set()).add(c)

    n_seed_comps = sum(1 for v in comp_has_seed.values() if v)
    print(f"components: {comp_id}, seed components: {n_seed_comps}, kept cells: {sum(len(v) for v in keep.values())}")

    # ---- rebuild nodes/edges on kept cells ----
    nodes2: List[Dict[str, Any]] = []
    edges2: List[Dict[str, Any]] = []
    id_map2: Dict[int, Dict[Tuple[int, int], str]] = {}
    for fidx, layer in LAYERS.items():
        g = grids[fidx]
        nx, ny = g["nx"], g["ny"]
        x0, y0 = g["x0"], g["y0"]
        lobby: Dict[Tuple[int, int], str] = {}
        for res_name, floors in res_cells.items():
            if fidx in floors:
                cx, cy = floors[fidx]
                if (cx, cy) in keep.get(fidx, set()):
                    lobby[(cx, cy)] = "elevator_lobby" if res_name.startswith("elevator") else "stair_lobby"
        id_map2[fidx] = {}
        for (ix, iy) in sorted(keep.get(fidx, set()), key=lambda c: (c[1], c[0])):
            node_id = f"f{fidx}_x{ix}_y{iy}"
            id_map2[fidx][(ix, iy)] = node_id
            nodes2.append({
                "id": node_id, "floor": fidx,
                "x": round(x0 + ix * GRID + GRID / 2, 2),
                "y": round(y0 + iy * GRID + GRID / 2, 2),
                "z": fidx * FLOOR_HEIGHT,
                "node_type": lobby.get((ix, iy), "corridor"),
            })
        for (ix, iy), nid in id_map2[fidx].items():
            for dx, dy in ((1, 0), (0, 1)):
                nb = (ix + dx, iy + dy)
                if nb in id_map2[fidx]:
                    nid2 = id_map2[fidx][nb]
                    edges2.append({"src": nid, "dst": nid2, "length": GRID, "kind": "floor"})
                    edges2.append({"src": nid2, "dst": nid, "length": GRID, "kind": "floor"})

    # ---- vertical edges (elevator/stair shafts pierce 3F..7F; adjacent-floor arcs) ----
    ev = res_cells["elevator-0"]
    sc = res_cells["stair-0"]
    fids = sorted(LAYERS)
    for a, b in zip(fids, fids[1:]):
        for res_name, cells in (("elevator-0", ev), ("stair-0", sc)):
            ca, cb = cells[a], cells[b]
            if ca in id_map2.get(a, {}) and cb in id_map2.get(b, {}):
                na, nb = id_map2[a][ca], id_map2[b][cb]
                length = FLOOR_HEIGHT if res_name.startswith("elevator") else STAIR_LEN
                edges2.append({"src": na, "dst": nb, "length": length, "kind": res_name[: res_name.index("-")], "resource": res_name})
                edges2.append({"src": nb, "dst": na, "length": length, "kind": res_name[: res_name.index("-")], "resource": res_name})

    meta = {
        "dataset": "icde-real-v1",
        "name": "real_1-7F",
        "building": "浙江医院三墩院区（省级三甲：门诊楼6层/住院楼14层）",
        "floor_mapping": {0: "1F", 1: "2F", 2: "3F", 3: "4F", 4: "5F", 5: "6F", 6: "7F"},
        "grid_m": GRID,
        "floor_height_m": FLOOR_HEIGHT,
        "elevator_xy": list(ELEVATOR_XY),
        "elevator_resource": "elevator-0",
        "stair_resource": "stair-0",
        "stair_xy": list(STAIR_XY),
        "stair_len_m": STAIR_LEN,
        "shaft_model": "both elevator and stair shafts pierce all floors 1F..7F; verified by 1F topo p1=(0,0) and 2F p1=(0,0) lying in the same vertical column as 3F p7 (-0.03,-0.16)",
        "fallback_notes": fallback_notes,
        "rar_origin_assumption": "RAR 4F/5F use origin [-20.3,-44.8,0] from 0528-6.yaml (companion 0528-6.pgm missing; same-batch alignment assumed); shaft projection on these floors is a modeling approximation",
        "sources": {
            "1F": "三墩门诊/一楼/sdmzl_zg_floor1D.pgm",
            "2F": "三墩门诊/二楼/sdmz_floor2.pgm",
            "3F": "sdmz_floor3.pgm (三墩门诊/三楼 identical, verified)",
            "4F": "创投地图已处理/已转/四楼已编辑.pgm (RAR, no topo)",
            "5F": "创投地图已处理/已转/五楼_1.pgm (RAR, no topo)",
            "6F": "floot_1.pgm + floot4_inner.pgm (merged)",
            "7F": "floot_2.pgm",
        },
        "note": "6F/7F reuse the desktop 4F/5F planes as higher floors per user decision; "
                "4F/5F are the RAR-extracted planes (distinct layouts, no navigation topo, "
                "tasks fall back to corridor cells). 7F topo p1 treated as elevator lobby (user-confirmed). "
                "1F task points exclude topo p1/p14/p18 (elevator/connect nodes); 2F excludes p1/p10 (elevator column).",
    }

    out = {
        "nodes": nodes2,
        "edges": edges2,
        "resources": resources,
        "resource_metadata": resource_metadata,
        "meta": meta,
    }
    map_path = OUT_MAPS / "real_1-7F.json"
    map_path.write_text(json.dumps(out, ensure_ascii=False, indent=1), encoding="utf-8")

    topo_points_out: Dict[str, Any] = {}
    for fidx, d in topo_node.items():
        fid = f"floor{fidx}"
        topo_points_out[fid] = {}
        for name, c in d.items():
            if c in id_map2.get(fidx, {}):
                topo_points_out[fid][name] = id_map2[fidx][c]
    (OUT_MAPS / "topo_points.json").write_text(
        json.dumps(topo_points_out, ensure_ascii=False, indent=1), encoding="utf-8")

    print(f"nodes={len(nodes2)} edges={len(edges2)} resources={resources}")
    print(f"written: {map_path}")
    print(f"written: {OUT_MAPS / 'topo_points.json'}")


if __name__ == "__main__":
    build()
