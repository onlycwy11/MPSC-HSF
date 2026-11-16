# Baseline适用性评估与实验建议

## 1. 两篇给定论文的定位

### MuNES

MuNES的研究对象是单机器人多层地图和轨迹规划。它将电梯、楼梯纳入统一体素地图，并在A*代价中考虑电梯等待时间和运行时间。它能够回答“一个机器人如何在多层建筑中选择电梯或楼梯”，但不解决本项目的核心组合问题：多个机器人与多个pickup-delivery任务的一对一分配、共享电梯时间窗冲突和批次总完成时间。因此它适合作为**多层路径代价基线**，不适合作为完整任务分配算法的唯一基线。

### Strip-based Route Planning

Strip-based论文研究规则单层仓库中的在线Collision-Aware Route Planning。其贡献是把行/列网格聚合成strip，在strip图上找路，并用strip内的时空线段交叉判断碰撞。论文默认货架仓库、单位速度、origin-destination请求流和makespan目标，没有楼层、电梯、楼梯、异构机器人能力和批次任务分配。因此它适合作为**规则布局碰撞规划/高效资源预留的对照**，但不能直接代表多层室内任务分配基线。

## 2. 两个是否足够

不够。两个基线分别覆盖单机器人多层导航和单层仓库碰撞规划，缺少：

1. 一类明确处理多机器人冲突约束的MAPF算法；
2. 一类明确处理pickup-delivery任务分配和在线调度的MAPD算法；
3. 一个不做冲突优化的匹配下界/消融，用于区分“分配收益”和“冲突处理收益”。

本交付新增两个真实可溯源的论文基线，形成四个项目：

- CBS：Sharon et al., *Conflict-based search for optimal multi-agent pathfinding*, Artificial Intelligence 219, 2015, DOI `10.1016/j.artint.2014.11.006`。高层按冲突分支加约束，低层重新求单机器人路径。
- MAPD Token Passing：Ma et al., *Lifelong Multi-Agent Path Finding for Online Pickup and Delivery Tasks*, AAMAS 2017。提出Token Passing和Token Passing with Task Swaps，直接面向在线pickup-delivery任务。

四者覆盖关系如下：

| 基线 | 多层路径 | 任务分配 | 多机器人冲突 | 主要用途 |
|---|---:|---:|---:|---|
| MuNES | 是 | 否 | 否 | 电梯/楼梯路径代价 |
| Strip-based | 否（本项目做垂直资源适配） | 否 | 是 | 规则布局碰撞处理 |
| CBS | 原论文否，本项目沿用统一多层图 | 否 | 是 | 冲突约束搜索 |
| MAPD-TP | 原论文单层，本项目沿用统一多层图 | 是 | 是 | 在线分配与token预留 |

另外建议在论文实验表中加入一个“Matching-only”消融：只执行初始Hungarian分配，不处理共享资源冲突。它不是额外论文基线，但能给出忽略冲突时的下界和冲突处理增量。

## 3. 统一接口与指标

四个项目均支持 `virtual`、`generic`、`real` 三种命令行模式，输入字段相同，输出字段相同。所有项目输出：

- `makespan`：所有已分配任务的最大完成时间；
- `sum_completion_time`：任务完成时间总和；
- `average_travel_time`、`total_wait_time`；
- `initial_conflict_count`、`final_conflict_count`；
- `iterations`、`reroutes`、未分配任务数；
- 每个任务的机器人、路径、垂直方式和时间。

Strip-based原论文还报告规划时间和内存；本项目先保证主算法和基线的功能指标可比。若要复现论文的效率结论，应在相同硬件上额外记录进程墙钟时间和峰值RSS，并提供规则仓库矩阵数据。

## 4. 复现边界

本交付的四个baseline项目是面向当前多层图/批次接口的可运行适配，不声称复现原论文的传感器建图、真实仓库数据、完整lifelong生命周期或严格离散CBS最优性。README中明确列出每个适配的假设；正式论文实验应把“原论文算法复现”和“统一接口适配”分开描述。

