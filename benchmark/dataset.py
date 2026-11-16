"""Dataset access layer for the MPSC-HSF benchmarks.

Both datasets follow the same on-disk convention:

    dataset/<name>/
        manifest.json                 # synthetic: single merged manifest
        manifests/*.json              # real: case manifest(s)
        maps/<map>.json[.gz]          # shared building geometry, stored once
        cases/<case_id>.json[.gz]     # per-case robots/tasks + map_ref

A *case* is a light skeleton: it stores the robot start positions and the task
set, and points at its building map through ``map_ref``.  The schedulers consume
a single-file payload with an embedded ``map`` object, so this module performs
the assembly (``load_payload``).  Keeping the geometry out of every case file
avoids storing the same multi-floor grid a few hundred times: the released
synthetic benchmark is ~756 cases over 10 building sizes.

All files may be stored gzipped (``*.json.gz``); loaders accept both forms so a
decompressed working copy also works.
"""

from __future__ import annotations

import gzip
import json
from pathlib import Path
from typing import Any, Dict, Iterable, List

from functools import lru_cache

REPO_ROOT = Path(__file__).resolve().parents[1]
DATASET_ROOT = REPO_ROOT / "dataset"

DATASETS: Dict[str, Path] = {
    "synthetic": DATASET_ROOT / "synthetic",
    "real": DATASET_ROOT / "real",
}


def dataset_root(name: str) -> Path:
    if name not in DATASETS:
        raise SystemExit(
            f"unknown dataset '{name}', expected one of {sorted(DATASETS)}"
        )
    root = DATASETS[name]
    if not root.exists():
        raise SystemExit(f"dataset directory not found: {root}")
    return root


def load_json(path: Path) -> Any:
    """Load a JSON document, transparently handling ``.gz``."""
    return _load_json_cached(str(Path(path).resolve()))


@lru_cache(maxsize=2)
def _load_json_cached(path: str) -> Any:
    # Maps are large (tens of MB); caching keeps a batch runner from
    # re-inflating the same grid for every case of the same building.
    file = Path(path)
    if file.suffix == ".gz":
        with gzip.open(file, "rt", encoding="utf-8") as fh:
            return json.load(fh)
    return json.loads(file.read_text(encoding="utf-8"))


def dump_json(obj: Any, path: Path) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(obj, ensure_ascii=False), encoding="utf-8")


def _resolve(directory: Path, stem: str) -> Path:
    for suffix in (".json", ".json.gz"):
        candidate = directory / f"{stem}{suffix}"
        if candidate.exists():
            return candidate
    raise FileNotFoundError(f"{stem} not found under {directory}")


def load_manifest(name: str) -> Dict[str, Any]:
    root = dataset_root(name)
    merged = root / "manifest.json"
    if merged.exists():
        return load_json(merged)
    manifest_dir = root / "manifests"
    candidates = sorted(manifest_dir.glob("*.json"))
    if not candidates:
        raise FileNotFoundError(f"no manifest under {manifest_dir}")
    return load_json(candidates[0])


def case_entries(name: str) -> List[Dict[str, Any]]:
    manifest = load_manifest(name)
    entries = manifest.get("cases")
    if not isinstance(entries, list):
        raise SystemExit(f"manifest of dataset '{name}' has no 'cases' list")
    return entries


def case_ids(name: str) -> List[str]:
    return [entry["case_id"] for entry in case_entries(name)]


def load_skeleton(case_id: str, name: str) -> Dict[str, Any]:
    root = dataset_root(name)
    return load_json(_resolve(root / "cases", case_id))


def load_map(map_ref: str, name: str) -> Dict[str, Any]:
    """Load the map referenced by a case (``map_ref`` is relative to dataset root)."""
    root = dataset_root(name)
    relative = Path(map_ref)
    candidates = [root / relative, root / relative.with_suffix(relative.suffix + ".gz")]
    for candidate in candidates:
        if candidate.exists():
            return load_json(candidate)
    raise FileNotFoundError(f"map referenced by case not found: {root / relative}")


def map_exists(map_ref: str, name: str) -> bool:
    """Cheap existence check for a ``map_ref`` (no JSON parsing)."""
    root = dataset_root(name)
    relative = Path(map_ref)
    return any(candidate.exists() for candidate in (
        root / relative,
        root / relative.with_suffix(relative.suffix + ".gz"),
    ))


def load_payload(case_id: str, name: str) -> Dict[str, Any]:
    """Return the single-file payload consumed by the schedulers."""
    skeleton = load_skeleton(case_id, name)
    payload: Dict[str, Any] = {
        "schema_version": skeleton.get("schema_version", "multifloor-scheduler-generic-v2"),
        "case": skeleton.get("case", {}),
        "robots": skeleton["robots"],
        "tasks": skeleton["tasks"],
        "config": skeleton.get("config", {}),
    }
    inline_map = skeleton.get("map")
    if inline_map is not None:
        payload["map"] = inline_map
        return payload
    map_ref = skeleton.get("map_ref")
    if not map_ref and skeleton.get("case", {}).get("map_key"):
        map_ref = f"maps/{skeleton['case']['map_key']}.json"
    if not map_ref:
        raise SystemExit(f"case '{case_id}' neither embeds a map nor declares map_ref")
    payload["map"] = load_map(map_ref, name)
    return payload


def materialize(case_id: str, name: str, out_path: Path) -> Path:
    """Write the assembled single-file case (embedded map) to ``out_path``."""
    path = Path(out_path)
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = load_payload(case_id, name)
    path.write_text(json.dumps(payload, ensure_ascii=False), encoding="utf-8")
    return path


def summarize(name: str) -> Dict[str, Any]:
    """Aggregate metadata used by ``tools/verify_dataset.py``."""
    entries = case_entries(name)
    robots = sorted({entry.get("robots") for entry in entries if entry.get("robots")})
    floors = sorted({entry.get("floors") for entry in entries if entry.get("floors")})
    return {
        "dataset": name,
        "cases": len(entries),
        "robots": robots,
        "floors": floors,
    }
