#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""统一批量运行脚本：对指定算法跑一批 case，结果落盘并汇总 metrics.csv。

用法（在总包根目录执行，Python 3.10+，纯标准库，无需第三方依赖）：

  # v2 已物化 case（小规模 7 个），单段直接跑
  python run_batch.py --algo multifloor_scheduler --dataset dataset_v2 --out results

  # v3 自由组合 672 case，分 3 片并行（服务器 nohup 三片各一条）
  python run_batch.py --algo multifloor_scheduler --dataset dataset_v3 --out results --shard 1/3
  python run_batch.py --algo baseline_cbs --dataset dataset_v3 --out results --shard 1/3 --max-iterations 1000

参数：
  --algo         algorithms/ 下的目录名（见 README_运行指南.md 的算法清单）
  --dataset      dataset_v2 或 dataset_v3（也可传目录路径）
  --out          结果输出目录，默认 results
  --shard        n/k：把 case 列表平均分成 k 片，只跑第 n 片（n 从 1 开始）
  --max-iterations  覆盖冲突搜索上限（CBS 等用，默认 1000）
  --limit N      只跑前 N 个 case（调试用）
  --only ID      只跑指定 case_id（调试用）

每个 case 输出 results/<algo>/<case_id>.json；汇总指标追加到 results/<algo>_metrics.csv。
出错的 case 记 error，不会中断整批。
"""
from __future__ import annotations

import argparse
import csv
import importlib.util
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent


def load_algo(name: str):
    algo_dir = ROOT / "algorithms" / name
    if not (algo_dir / "core.py").exists():
        raise SystemExit(f"找不到算法目录: {algo_dir}")
    sys.path.insert(0, str(algo_dir))
    spec = importlib.util.spec_from_file_location(f"algo_{name}", algo_dir / "core.py")
    mod = importlib.util.module_from_spec(spec)
    sys.modules[f"algo_{name}"] = mod
    spec.loader.exec_module(mod)
    return mod


def v3_materialize(v3_dir: Path, item: dict) -> Path:
    """复用 v3 生成器把 manifest 中的一个 case 物化为 JSON，返回路径。

    v3 与 v3.1 共用同一套 make_case/make_map 逻辑；生成器固定取自 dataset_v3，
    地图与 case 写到目标目录（v3_dir）下。
    """
    gen_path = ROOT / "dataset_v3" / "generate_icde_dataset_v3.py"
    spec = importlib.util.spec_from_file_location("gen_v3", gen_path)
    gen = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(gen)
    return gen.materialize(v3_dir, item)


def case_iter(dataset_dir: Path, shard: str | None, only: str | None, limit: int | None):
    man3 = dataset_dir / "manifests" / "case_manifest_v3.json"
    man31 = dataset_dir / "manifests" / "case_manifest_v3.1.json"
    if man3.exists() or man31.exists():
        manifest_path = man3 if man3.exists() else man31
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        items = manifest["cases"]
        if only:
            items = [x for x in items if x["case_id"] == only]
        jobs = [("v3", x["case_id"], x) for x in items]
    else:
        cases_dir = dataset_dir / "cases"
        files = sorted(cases_dir.glob("*.json"))
        if only:
            files = [f for f in files if f.stem == only]
        jobs = [("v2", f.stem, f) for f in files]

    if shard:
        n, k = shard.split("/")
        n, k = int(n), int(k)
        jobs = [j for i, j in enumerate(jobs) if (i % k) + 1 == n]
    if only:
        pass
    if limit:
        jobs = jobs[:limit]
    return jobs


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--algo", required=True)
    ap.add_argument("--dataset", required=True)
    ap.add_argument("--out", default="results")
    ap.add_argument("--shard", default=None)
    ap.add_argument("--max-iterations", type=int, default=None)
    ap.add_argument("--limit", type=int, default=None)
    ap.add_argument("--only", default=None)
    args = ap.parse_args()

    mod = load_algo(args.algo)
    dataset_dir = (ROOT / args.dataset) if not Path(args.dataset).is_absolute() else Path(args.dataset)
    out_dir = ROOT / args.out / args.algo
    out_dir.mkdir(parents=True, exist_ok=True)
    csv_path = ROOT / args.out / f"{args.algo}_metrics.csv"
    new_csv = not csv_path.exists()

    jobs = case_iter(dataset_dir, args.shard, args.only, args.limit)
    rows = []
    for kind, case_id, payload in jobs:
        try:
            if kind == "v3":
                case_path = v3_materialize(dataset_dir, payload)
                data = json.loads(case_path.read_text(encoding="utf-8"))
            else:
                data = json.loads(Path(payload).read_text(encoding="utf-8"))
            building, robots, tasks, cfg = mod.parse_payload(data)
            if args.max_iterations is not None and hasattr(cfg, "max_iterations"):
                cfg.max_iterations = args.max_iterations
            result = mod.schedule(building, robots, tasks, cfg)
            met = result["metrics"]
            (out_dir / f"{case_id}.json").write_text(
                json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
            rows.append({
                "case_id": case_id,
                "assigned_count": met.get("assigned_count"),
                "unassigned_count": met.get("unassigned_count"),
                "makespan": met.get("makespan"),
                "sum_completion_time": met.get("sum_completion_time"),
                "total_wait_time": met.get("total_wait_time"),
                "final_conflict_count": met.get("final_conflict_count"),
                "iterations": met.get("iterations"),
                "status": "ok",
            })
        except Exception as e:  # 单 case 失败不中断整批
            rows.append({"case_id": case_id, "status": f"error:{type(e).__name__}:{e}"})

    with csv_path.open("a", newline="", encoding="utf-8-sig") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()) if rows else ["case_id", "status"])
        if new_csv:
            writer.writeheader()
        for r in rows:
            writer.writerow(r)

    ok = sum(1 for r in rows if r.get("status") == "ok")
    print(f"[{args.algo}] done {ok}/{len(rows)} -> {out_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
