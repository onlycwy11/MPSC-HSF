# baseline_wait_only — 纯等待基线（Pure-Wait Baseline）

导师要求的"纯等待"基线：检测到冲突后**不做任何换路/换资源处理**，只让机器人按排队逻辑依次等待使用资源。

## 算法

1. **匈牙利初分配**：成本 = 无限制最短路径时间（机器人-任务类型必须匹配，不匹配记罚值）。
2. **最短路径规划**：每台机器人按全图无限制最短路执行（允许电梯/楼梯，wheeled 不走楼梯）。
3. **纯 FIFO 排队**：所有冲突只通过等待解决——
   - 按 `(理想开始时刻, 机器人编号, 通行时长)` 顺序逐计划贴合预约表；
   - 后到者与先到者资源区间重叠时整体顺延（等待）；
   - 同起点同刻出发的机器人按编号错开 1s；
   - **不**换楼梯、**不**换电梯、**不**重规划任何路径。
4. 输出分配、路径、资源预约表与指标。

> 排队顺序（用户批准口径）：先到先服务（按到达资源入口时刻）→ 同时到达按机器人编号 → 同到再按通行时长短者优先。

## 快速开始

```bash
python run.py                                   # 虚拟示例
python run.py --mode generic --input <case.json> --output out.json
python run.py --mode real --map map.json --db tasks.db --output out.json
```

## 输出

与主算法 `multifloor_scheduler`、各 baseline 完全同构的 JSON（`assignments / unassigned_tasks / resource_schedule / metrics`），可直接作为对比基线。

## 依赖

仅 Python 标准库。

## 目录

| 文件 | 说明 |
|---|---|
| `core.py` | 纯等待基线实现（复用主算法的距离列缓存等性能优化） |
| `run.py` | 命令行入口 |

## 与其它算法的关系

| 算法 | 冲突消解方式 |
|---|---|
| 主算法 `multifloor_scheduler` | 循环检测冲突 → Δ 最大者保留当前路径、其余换次优（更快/换资源/走楼梯）→ 重检 |
| `multifloor_scheduler_nearest` | 检测一次 → 冲突段起点水平就近选电梯/楼梯换路一次 → FIFO 排队 |
| **`baseline_wait_only`（本目录）** | **纯 FIFO 排队等待，不做任何换路** |
