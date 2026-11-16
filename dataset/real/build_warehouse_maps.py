#!/usr/bin/env python3
"""Build public-benchmark multi-floor maps from MovingAI MAPF warehouse grids.

Source: Sturtevant, "Benchmarks for Grid-Based Pathfinding", IEEE TCIAIG 2012.
Downloads: https://movingai.com/benchmarks/mapf/{name}.map.zip
Maps used:
  - warehouse-10-20-10-2-1.map  161x63
  - warehouse-10-20-10-2-2.map  170x84
  - warehouse-20-40-10-2-1.map  321x123

Scenes:
  - movingai_warehouse_8F : 8 floors, layout alternates map1 / map2 (standard
    warehouse levels), floor 0..7, z = 0..24.5 m.
  - movingai_warehouse_10F: 10 floors, large map (20-40) repeated (standard
    floor), floor 0..9, z = 0..31.5 m.

Shaft model (same as the hospital map): elevator + stair shafts pierce all
floors at a fixed corridor position; adjacent-floor vertical edges, length 3.5 m
(elevator) / 14.88 m (stair, consistent modeling length). No navigation topo
points exist in the benchmark -> tasks use corridor cells (generator fallback).

Outputs:
  - maps/movingai_warehouse_8F.json
  - maps/movingai_warehouse_10F.json
"""

from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Any, Dict, List, Tuple

ROOT = Path(__file__).resolve().parent
OUT_MAPS = ROOT / "maps"
RAW = ROOT / "raw_public"

FLOOR_HEIGHT = 3.5
STAIR_LEN = 14.88   # modeling length consistent with the hospital dataset

# shaft corridor position (cells), near the left/right service corridors
ELEVATOR_CELL = (1, 1)
STAIR_DX_FROM_RIGHT = 2   # stair column at (w-2, 1)


def load_map(path: Path) -> Dict[str, Any]:
    """Parse a MovingAI .map file into {'grid': List[str], 'h': int, 'w': int}."""
    lines = path.read_text(encoding="utf-8").splitlines()
    h = w = None
    grid: List[str] = []
    started = False
    for ln in lines:
        ln = ln.strip()
        if not ln:
            continue
        if not started:
            if ln.startswith("height"):
                h = int(ln.split()[1])
            elif ln.startswith("width"):
                w = int(ln.split()[1])
            elif ln == "map":
                started = True
            continue
        grid.append(ln)
    if len(grid) != h or any(len(r) != w for r in grid):
        raise ValueError(f"bad map dims in {path}: expected {h}x{w}, got {len(grid)} rows")
    return {"grid": grid, "h": h, "w": w}


def build_scene(name: str, layer_names: List[str], n_floors: int) -> Dict[str, Any]:
    """layers: per-floor map file names (cycled if shorter than n_floors)."""
    maps = {n: load_map(RAW / (n + ".map")) for n in dict.fromkeys(layer_names)}
    # fixed shaft world positions: elevator near left corridor, stair near right
    # corridor; per-floor projection via nearest_free (floors may differ).
    ev_xy = (1.5, 1.5)
    st_xy = (0.0, 1.5)   # replaced per layer below using actual width

    nodes: List[Dict[str, Any]] = []
    edges: List[Dict[str, Any]] = []
    resources: Dict[str, int] = {"elevator-0": 1, "stair-0": 1}
    resource_metadata: Dict[str, Dict[str, Any]] = {
        "elevator-0": {"kind": "elevator", "capacity": 1, "x": ev_xy[0], "y": ev_xy[1],
                       "shaft": f"pierces all {n_floors} floors", "note": "corridor column, benchmark map"},
        "stair-0": {"kind": "stair", "capacity": 1, "x": 0.0, "y": 1.5,
                    "shaft": f"pierces all {n_floors} floors", "note": "right corridor column, benchmark map"},
    }

    res_cells: Dict[str, Dict[int, Tuple[int, int]]] = {}
    free_cells: Dict[int, List[Tuple[int, int]]] = {}
    fallbacks: Dict[str, List[int]] = {"elevator-0": [], "stair-0": []}

    def nearest_free(g: Dict[str, Any], x: float, y: float) -> Tuple[int, int]:
        blocked, nx, ny = g["blocked"], g["w"], g["h"]
        ix = min(max(int(math.floor(x)), 0), nx - 1)
        iy = min(max(int(math.floor(y)), 0), ny - 1)
        if not blocked[iy][ix]:
            return ix, iy
        best = None
        best_d = float("inf")
        for r in range(1, max(nx, ny)):
            for dx in range(-r, r + 1):
                for dy in range(-r, r + 1):
                    if max(abs(dx), abs(dy)) != r:
                        continue
                    cx, cy = ix + dx, iy + dy
                    if 0 <= cx < nx and 0 <= cy < ny and not blocked[cy][cx]:
                        d = dx * dx + dy * dy
                        if d < best_d:
                            best_d, best = d, (cx, cy)
            if best:
                return best
        raise RuntimeError("no free cell")

    grids: Dict[int, Dict[str, Any]] = {}
    for fidx in range(n_floors):
        mn = layer_names[fidx % len(layer_names)]
        g = maps[mn]
        w, h = g["w"], g["h"]
        blocked = [[c not in ".GS" for c in row] for row in g["grid"]]
        grids[fidx] = {"blocked": blocked, "w": w, "h": h, "map_name": mn}
        # shaft projection (each floor's own nearest free cell)
        ev = nearest_free(grids[fidx], ev_xy[0], ev_xy[1])
        res_cells.setdefault("elevator-0", {})[fidx] = ev
        if (ev[0], ev[1]) != (int(math.floor(ev_xy[0])), int(math.floor(ev_xy[1]))):
            fallbacks["elevator-0"].append(fidx)
        stx, sty = w - STAIR_DX_FROM_RIGHT - 0.5, 1.5
        st = nearest_free(grids[fidx], stx, sty)
        res_cells.setdefault("stair-0", {})[fidx] = st
        if (st[0], st[1]) != (int(math.floor(stx)), int(math.floor(sty))):
            fallbacks["stair-0"].append(fidx)

        lobby: Dict[Tuple[int, int], str] = {}
        for res_name, floors in res_cells.items():
            if fidx in floors:
                cx, cy = floors[fidx]
                lobby[(cx, cy)] = "elevator_lobby" if res_name.startswith("elevator") else "stair_lobby"

        id_map: Dict[Tuple[int, int], str] = {}
        for iy in range(h):
            for ix in range(w):
                if blocked[iy][ix]:
                    continue
                nid = f"f{fidx}_x{ix}_y{iy}"
                id_map[(ix, iy)] = nid
                free_cells.setdefault(fidx, []).append((ix, iy))
                nodes.append({"id": nid, "floor": fidx,
                              "x": ix + 0.5, "y": iy + 0.5,
                              "z": fidx * FLOOR_HEIGHT,
                              "node_type": lobby.get((ix, iy), "corridor")})
        for (ix, iy), nid in id_map.items():
            for dx, dy in ((1, 0), (0, 1)):
                nb = (ix + dx, iy + dy)
                if nb in id_map:
                    n2 = id_map[nb]
                    edges.append({"src": nid, "dst": n2, "length": 1.0, "kind": "floor"})
                    edges.append({"src": n2, "dst": nid, "length": 1.0, "kind": "floor"})

    # vertical edges (adjacent floors)
    ev = res_cells["elevator-0"]
    sc = res_cells["stair-0"]
    id_map_by_floor: Dict[int, Dict[Tuple[int, int], str]] = {}
    for n in nodes:
        id_map_by_floor.setdefault(n["floor"], {})[(int(n["x"] - 0.5), int(n["y"] - 0.5))] = n["id"]
    for a in range(n_floors - 1):
        b = a + 1
        for res_name, cells, length in (("elevator-0", ev, FLOOR_HEIGHT), ("stair-0", sc, STAIR_LEN)):
            ca, cb = cells[a], cells[b]
            na = id_map_by_floor[a].get(ca)
            nb = id_map_by_floor[b].get(cb)
            if na and nb:
                kind = "elevator" if res_name.startswith("elevator") else "stair"
                edges.append({"src": na, "dst": nb, "length": length, "kind": kind, "resource": res_name})
                edges.append({"src": nb, "dst": na, "length": length, "kind": kind, "resource": res_name})

    meta = {
        "dataset": "icde-real-v1-public",
        "name": name,
        "building": "MovingAI MAPF benchmark (Sturtevant, TCIAIG 2012) — warehouse",
        "source_url": "https://movingai.com/benchmarks/mapf/",
        "layers": [layer_names[i % len(layer_names)] for i in range(n_floors)],
        "n_floors": n_floors,
        "floor_height_m": FLOOR_HEIGHT,
        "elevator_xy": list(ev_xy),
        "stair_xy": "right corridor column per layer",
        "stair_len_m": STAIR_LEN,
        "shaft_model": f"elevator and stair shafts pierce all {n_floors} floors",
        "fallback_notes": fallbacks,
        "task_points": "no navigation topo in benchmark; tasks use corridor cells (generator fallback)",
        "note": "public benchmark scene for the ICDE experiments; standard-floor repetition follows common warehouse construction practice.",
    }

    out = {"nodes": nodes, "edges": edges, "resources": resources,
           "resource_metadata": resource_metadata, "meta": meta}
    path = OUT_MAPS / (name + ".json")
    path.write_text(json.dumps(out, ensure_ascii=False, indent=1), encoding="utf-8")
    print(f"{name}: nodes={len(nodes)} edges={len(edges)} -> {path}")


def main() -> None:
    OUT_MAPS.mkdir(parents=True, exist_ok=True)
    build_scene("movingai_warehouse_8F",
                ["warehouse-10-20-10-2-1", "warehouse-10-20-10-2-2"], 8)
    build_scene("movingai_warehouse_10F",
                ["warehouse-20-40-10-2-1", "warehouse-10-20-10-2-1"], 10)


if __name__ == "__main__":
    main()
