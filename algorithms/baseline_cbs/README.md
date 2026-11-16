# CBS baseline — Faithful Conflict-Based Search

## 对应论文

Guni Sharon, Roni Stern, Ariel Felner, Nathan R. Sturtevant,
*Conflict-based search for optimal multi-agent pathfinding*,
Artificial Intelligence 219, 2015. DOI 10.1016/j.artint.2014.11.006.

## 实现说明（2026-10-13 重写）

本实现忠实保留 CBS 的算法结构，只适配了运动模型：

- **高层**：约束树（Constraint Tree, CT）搜索。每个 CT 节点保存每个机器人的
  资源-时间约束集合、受约束路径和代价；按代价（完成时间总和）最小优先展开；
  每次取最早冲突，给冲突双方各生成一个加约束分支；弹出的第一个无冲突节点
  即为该分配下的最优解（best-first + 约束划分解空间 ⇒ CBS 最优性论证成立）。
- **低层**：受约束最短路径搜索。在统一连续时间多层资源模型下规划
  `start → pickup → delivery`，支持在节点等待；电梯为直达转移边（一次乘梯
  `4.5 + 1.75*|Δf|` 秒），楼梯为直达转移边（`3.5*|Δf| / v_stair` 秒），
  单行窄通道为容量 1 资源，双向通道为宽通道不参与冲突。
- **任务分配**：CBS 本身不分配任务。本实验先做类型感知的最小代价
  （Hungarian）分配，再由 CBS 消解该固定分配下的全部资源冲突。一个机器人
  一个 batch 至多一个任务。
- **约束类型**：`(resource, [t1, t2))`，表示该机器人不得在时间窗内使用该
  资源（电梯一次乘梯、楼梯一次转移、或单行通道的一段占用）。

## 与论文的差异（需在论文中说明）

论文假设二维离散栅格、等时长步；本实验为连续时间、多楼层、电梯/楼梯容量
资源模型。CBS 的高层约束搜索机制不依赖具体时间模型，因此算法结构完整保留，
仅低层运动模型更换为统一接口的连续时间模型。

## 运行

```powershell
python run.py --mode generic --input <case.json> --output <result.json>
```

## 性能说明

CBS 是最优算法，约束树规模随机器人数量和资源瓶颈（如单电梯）指数增长。
小规模（R≤16 级）可正常收敛；更大规模会触发 `max_iterations` 上限并回退到
确定性序列化保证无冲突，输出中 `iterations / cbs_nodes` 记录展开节点数，
达到上限的实例应单独统计，不与正常收敛实例混合计算。
