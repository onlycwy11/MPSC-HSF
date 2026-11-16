# 多楼层多机器人调度场景下的基线适配说明
# Adaptation of Baseline Algorithms to the Multi-Floor Multi-Robot Scheduling Scenario

> 版本：2026-11-11　|　对应实验总包：`algorithms/`
> 本文逐一说明每个基线方法在保持原算法核心不变的前提下，如何被接入我们的统一多楼层、连续时间、批量调度仿真；并区分"论文/原方法忠实保留的部分"与"为适配场景而补齐的简化机制"。
> This document describes, for each baseline, how its original algorithmic core is preserved while the method is integrated into our unified continuous-time, batch multi-floor scheduling simulator. Parts faithfully retained from the source and parts added as minimal adaptation mechanisms are explicitly distinguished.

---

## 1. 统一场景定义 / Unified Scenario

### 1.1 中文
实验在一个离线批量调度场景中进行：

- **任务结构**：一批取送货任务（pickup–delivery），每台机器人在本批次中至多执行一个任务；任务按释放时间（release time）进入系统；不使用任务优先级。
- **机器人类型**：轮式机器人只能经电梯换层；腿式机器人可走楼梯，也可乘电梯。
- **共享资源**：电梯、楼梯、单向窄走廊均为容量 1 的时间窗资源；同一资源在同一时段只能被一个机器人占用。
- **时间模型**：连续时间；每条边有物理行程代价，电梯边由"赶来接人等待 + 乘梯"两段构成。
- **优化目标**：makespan（最后完成时刻）、总/平均完成时间（sum/mean completion time）、总等待时间（total wait）、最终冲突数（final conflict count）。

调度问题在方法上被分解为三层：**任务分配（assignment）→ 路径规划（path planning）→ 冲突消解（conflict resolution）**。每个基线都可以沿这三层刻画其保留与适配内容。

### 1.2 English
All experiments run in an offline batch scheduling setting:

- **Tasks**: a batch of pickup–delivery tasks; each robot executes at most one task per batch; tasks enter by release time; no task priority is used.
- **Robot types**: wheeled robots may change floors only by elevator; legged robots may use stairs or elevators.
- **Shared resources**: elevators, stairs, and one-way narrow corridors are capacity-1 time-window resources; at most one robot occupies a resource in any time interval.
- **Time model**: continuous time; every edge carries a physical travel cost; an elevator edge consists of an "come-to-pick-up" waiting segment plus a ride segment.
- **Objectives**: makespan, total/mean completion time, total waiting time, and final conflict count.

The problem is decomposed into three algorithmic layers: **assignment → path planning → conflict resolution**. Each baseline is characterized along these three layers.

---

## 2. 通用适配层 / Common Adaptation Layer

### 2.1 中文
所有基线共享同一套适配接口，以保证对比只反映算法本身的差异：

1. **统一输入输出**：同一 case JSON（或 real 模式的地图 + SQLite 状态库）解析为统一的 `BuildingMap / Node / Edge / Robot / Task` 结构；输出 JSON 与主算法同构（assignments、metrics、path）。
2. **统一物理代价**：走廊 `length / v_walk`；楼梯 `3.5·|Δ层| / v_stair`；电梯 `4.5 + 1.75·|Δ层|` 乘梯代价，另加电梯赶来等待 `1.75·|当前层−机器人层|`。
3. **统一预约表**：以连续时间窗记录每个资源的占用；冲突判定基于时间窗重叠（滑动窗口扫描）。
4. **独立可运行**：每个基线目录都自带一份 `baseline_utils.py` 拷贝，不 import 主项目代码，可在干净机器上单独复现。
5. **公平性原则**：原方法真正贡献的算法层忠实保留；原方法未覆盖的层（如缺任务分配、缺跨楼层冲突处理）用最朴素的机制补齐，避免引入可被归因到主算法的额外智能。

### 2.2 English
All baselines share one adaptation layer so that comparisons isolate algorithmic differences:

1. **Unified I/O**: each case JSON (or, in real mode, a map plus SQLite state DB) is parsed into a common `BuildingMap / Node / Edge / Robot / Task` representation; the output JSON matches the main algorithm's schema (assignments, metrics, path).
2. **Common physical costs**: corridor `length / v_walk`; stairs `3.5·|Δf| / v_stair`; elevator ride `4.5 + 1.75·|Δf|`, plus come-to-pick-up waiting `1.75·|elevator_floor − robot_floor|`.
3. **Common reservation table**: resource occupancy is recorded as continuous-time windows; conflicts are detected by window overlap (sliding-window sweep).
4. **Standalone executability**: each baseline directory carries its own copy of `baseline_utils.py` and does not import main-project code, so it reproduces on a clean machine.
5. **Fairness principle**: the source method's actual algorithmic contribution is retained verbatim; layers it does not cover are filled with the simplest possible mechanism, so that no hidden extra intelligence is attributed to the main method.

---

## 3. 基线总览 / Baseline Overview

| 基线 Baseline | 来源 Source | 任务分配 Assignment | 路径规划 Path planning | 冲突消解 Conflict resolution |
|---|---|---|---|---|
| **Wait-only** 纯等待 | 内部对照基线 Internal strawman | 匈牙利最小代价（类型匹配）Hungarian min-cost, type match | 无约束最短路 Unconstrained shortest path | 仅 FIFO 排队；不改路 Queueing only, no reroute |
| **Nearest-resource** 近路优先 | 内部对照变体 Internal one-pass variant | 匈牙利 Hungarian | 单趟冲突段重选水平最近资源 One-pass reroute to nearest horizontal resource | 剩余冲突 FIFO；每段至多一次重路 Remaining FIFO; one reroute max |
| **Strip / SRP** | Shi et al., ICDE 2023 | 随规划顺序分配 Sequential order | 带状聚合 + 带内一维时空段 Strip aggregation, 1-D time-space segments | 带内回溯等待；跨层 FIFO Intra-strip backtrack wait; cross-floor FIFO |
| **MAPD–Token Passing (TP)** | Ma et al., AAMAS 2017 | 令牌顺序 + 预约表选最早完成 Token order, earliest completion against reservation | 约束最短路 Constrained shortest path | 预约表避让 Reservation-based avoidance |
| **CBS** | Sharon et al., AIJ 2015 | 外部匈牙利预分配 Pre-assigned by Hungarian | 低层约束最短路 Low-level constrained SP | 高层约束树分裂冲突 High-level constraint-tree splitting |
| **MuNES** | Jung et al., 2024 (arXiv:2402.04535) | 顺序贪心：释放序、类型匹配、静态行程最短 Greedy by release, static nearest | 电梯等待感知寻路 Elevator-waiting-aware planning | 全局 FIFO 等待；不改路 Global FIFO wait, no reroute |

---

## 4. 各基线适配细节 / Per-Baseline Adaptation Details

### 4.1 Wait-only（纯等待基线）

**中文**
- **定位**：用于隔离"冲突消解层"的增益，本身不引入任何寻路或换路智能。
- **任务分配**：沿用与主算法相同的匈牙利最小代价匹配——代价矩阵基于无约束最短行程时间，并强制机器人类型与任务类型匹配。
- **路径规划**：在统一图上求无约束最短路，不考虑其他机器人的预约。
- **冲突消解**：所有资源冲突只通过 FIFO 排队解决——先到资源入口者优先，平局按机器人编号，再平局按更短竖向行程；**全程不换路、不换电梯、不走楼梯绕行**。
- **适配说明**：除问题定义要求的类型匹配与连续时间预约表外，未额外加入启发式。该基线与主算法的差异仅出现在冲突消解层，便于归因。

**English**
- **Role**: isolates the contribution of the conflict-resolution layer; it introduces no routing or resource-switching intelligence.
- **Assignment**: same Hungarian min-cost matching as the main method; the cost matrix uses unconstrained shortest travel time, with strict robot–task type matching.
- **Planning**: an unconstrained shortest path on the unified graph, ignoring other robots' reservations.
- **Conflict resolution**: all resource conflicts are resolved by FIFO queueing alone — first arrival at the resource entrance goes first, ties broken by robot id, then by shorter vertical ride time. **No route switching, no elevator change, no stair detour** at any point.
- **Adaptation**: apart from the type matching and continuous-time reservation required by the problem definition, no heuristic is added. Differences from the main method appear only in the conflict-resolution layer, enabling clean attribution.

### 4.2 Nearest-resource Reroute（近路优先基线）

**中文**
- **定位**：单趟（single-pass）换路变体，用于限定"一次性就近换资源"这一类简单重规划的能力上界。
- **任务分配**：匈牙利最小代价匹配，同 Wait-only。
- **路径规划**：每个分配对拆成两段——起点→取货、取货→送货；初始路径一次生成后只做一轮冲突检测。
- **冲突消解**：对与既有预约段重叠的后到机器人，定位包含冲突竖向资源的那一段，从该段起点枚举所有可达电梯/楼梯；**只比较水平两段步行代价**（起点→入口、出口→段终点），竖向乘梯代价不进入选择；轮式机器人只能选电梯。若水平最近资源与当前不同，则把该段改走该资源；相同则保留原段排队。每个冲突段至多重路一次，**不做第二轮冲突复检**；剩余竞争仍由资源级 FIFO 解决。
- **适配说明**：换路决策只看静态水平距离、不估算等待收益，是对主算法"等待代价 vs 绕行代价"权衡机制的简化对照。

**English**
- **Role**: a one-pass rerouting variant that bounds the capability of simple "switch to the nearest resource once" replanning.
- **Assignment**: Hungarian min-cost matching, as in Wait-only.
- **Planning**: each assigned pair is split into two segments (start→pickup, pickup→delivery); initial routes are built once and scanned for conflicts in a single pass.
- **Conflict resolution**: for a later robot whose interval overlaps an earlier reservation, the segment containing the conflicting vertical resource is located; every reachable elevator/stair from that segment's start is enumerated. **Only the two horizontal walking parts** (start→entry, exit→segment end) are compared; the vertical ride cost is excluded from the choice; wheeled robots may choose elevators only. If the horizontally nearest resource differs from the current one, that segment is rerouted through it; otherwise the original segment queues. Each conflicted segment is rerouted at most once, **with no second conflict re-detection**; remaining contention is handled by per-resource FIFO.
- **Adaptation**: the reroute decision uses static horizontal distance only, without estimating waiting-vs-detour trade-offs; it is a simplified counterpart to the main method's wait-vs-detour mechanism.

### 4.3 Strip / SRP（Shi et al., ICDE 2023）

**中文**
- **来源**：Shi et al., *Collision-Aware Route Planning in Warehouses Made Efficient: A Strip-based Framework*, ICDE 2023。
- **忠实保留（单层内核心）**：
  1. **带状聚合**：规则网格层按巷道行聚合为水平带状条（论文 Algorithm 1）。
  2. **带间规划**：在带状图上求无碰撞的带状序列（论文 Sec. IV）。
  3. **带内规划**：条内路线是一维时空段（斜率 +v / −v / 0）；冲突按几何线段相交判定（论文 Eq. 2/3），相撞前停在冲突点之前等待并重试（论文 Algorithm 2 回溯）。
  4. **斜率索引**：段按斜率分桶（+v、−v、0），同桶按时间排序，同斜率候选经旋转坐标过滤（论文 Sec. V-D / Algorithm 3）。
- **适配补齐（跨楼层部分）**：原 SRP 面向单层仓库；本基线把每层建成独立带状图，层与层之间用直接转移边连接，边代价为电梯/楼梯通行时间；**跨楼层冲突按 FIFO 先到先得等待**（电梯/楼梯容量 1）。这一跨层扩展是经确认的简化，不引入论文之外的智能策略。

**English**
- **Source**: Shi et al., *Collision-Aware Route Planning in Warehouses Made Efficient: A Strip-based Framework*, ICDE 2023.
- **Retained (within-floor core)**:
  1. **Strip aggregation**: each regular grid layer is aggregated into horizontal strips along aisle rows (paper Algorithm 1).
  2. **Inter-strip planning**: a collision-free strip sequence on the strip graph (paper Sec. IV).
  3. **Intra-strip planning**: a route inside a strip is a 1-D time-space segment (slope +v / −v / 0); collisions are geometric segment intersections (paper Eqs. 2–3); on collision the route stops just before the collision point, waits, and retries (paper Algorithm 2 backtracking).
  4. **Slope indexing**: segments are bucketed by slope (+v, −v, 0), each bucket time-ordered, and same-slope candidates filtered by rotated coordinates (paper Sec. V-D / Algorithm 3).
- **Adaptation (cross-floor part)**: SRP targets single-floor warehouses. Here each floor is an independent strip graph; floors are connected by direct transfer edges whose cost is the elevator/stair passage time; **cross-floor conflicts are resolved by FIFO first-come-first-served waiting** (elevator/stair capacity 1). This cross-floor extension is an approved simplification and adds no strategy beyond the paper.

### 4.4 MAPD – Token Passing, TP（Ma et al., AAMAS 2017）

**中文**
- **来源**：Ma, Li, Kumar, Koenig, *Lifelong Multi-Agent Path Finding for Online Pickup and Delivery Tasks*, AAMAS 2017。
- **原方法定位**：TP 是面向终身在线任务流的 MAPD 机制；实验为离线批量、每机器人至多一任务，因此仅适配批量设定，核心机制按原意保留：
  1. **令牌遍历任务**：全局令牌按释放时间顺序遍历任务列表。
  2. **预约表评估**：对当前任务，每个空闲且类型匹配的机器人，以其他机器人既有路径构成的预约表，计算自己最早能完成"取货→送货"的时刻。
  3. **分配与入表**：最早完成的机器人领取任务，其避让预约的路径追加到令牌状态（预约表）中。
  4. **任务交换省略**：原 TPTS（task swapping）在本批量设定下不适用——每机器人仅一任务、无交换对象，故不实现。
- **低层规划**：在统一资源模型上做约束最短路；电梯乘行、楼梯转移为同资源跨层直接边；容量受限资源只能使用预约表中空闲的时间窗。

**English**
- **Source**: Ma, Li, Kumar, Koenig, *Lifelong Multi-Agent Path Finding for Online Pickup and Delivery Tasks*, AAMAS 2017.
- **Adaptation**: TP is designed for a lifelong online task stream; our experiment is an offline batch with at most one task per robot. The setting is adapted to the batch protocol only, while the core TP mechanism is preserved in spirit:
  1. **Token traversal**: a global token walks the task list ordered by release time.
  2. **Reservation-based evaluation**: for the current task, every idle, type-matching robot computes its earliest pickup→delivery completion against a reservation table formed by the other robots' existing paths (token state).
  3. **Assignment and table update**: the robot with the earliest completion receives the task, and its collision-avoiding path is appended to the token state.
  4. **Task swapping omitted**: TPTS swapping does not apply to this batch protocol — one task per robot, no swap partner — and is not implemented.
- **Low-level planner**: a constrained shortest path over the unified resource model; elevator rides and stair transfers are direct cross-floor edges on the same resource; capacity-limited resources may only use time windows free in the reservation table.

### 4.5 CBS（Sharon et al., AIJ 2015）

**中文**
- **来源**：Sharon et al., *Conflict-based Search for Optimal Multi-Agent Pathfinding*, Artificial Intelligence 219, 2015。
- **忠实保留的算法结构**：
  1. **高层约束树（CT）**：节点携带各机器人的资源–时间约束；按完成时间之和最佳优先展开。
  2. **低层约束最短路**：在约束下为单机器人求可行路径。
  3. **冲突分裂**：检测到冲突即按冲突双方分裂约束，直到得到无冲突方案；约束划分解空间、展开按代价递增，CBS 的最优性论证在结构上沿用。
- **适配改动（仅运动模型）**：原 CBS 假设离散网格、单位时间步；本基线把低层搜索放在**连续时间图**上：换层以转移边表示——一次电梯乘行代价 `4.5 + 1.75·|Δf|`，一次楼梯转移 `3.5·|Δf| / v_stair`。
- **任务分配（外部补齐）**：CBS 本身不含分配；先以无约束路径时间做匈牙利最小代价匹配、把任务预分配给类型匹配机器人，再由 CBS 为这一固定分配消解全部资源冲突。这与实验协议一致（一批一任务；轮式仅乘电梯）。

**English**
- **Source**: Sharon et al., *Conflict-based Search for Optimal Multi-Agent Pathfinding*, Artificial Intelligence 219, 2015.
- **Retained algorithmic structure**:
  1. **High-level Constraint Tree (CT)**: nodes hold per-robot resource–time constraints; expansion is best-first by sum of completion times.
  2. **Low-level constrained shortest path**: a feasible single-robot path under the constraints.
  3. **Conflict splitting**: a detected conflict splits constraints between the two conflicting agents until a conflict-free plan is found. Because constraints partition the solution space and expansion proceeds by increasing cost, the CBS optimality argument carries over structurally.
- **Adaptation (motion model only)**: the original CBS assumes a discrete grid with unit time steps; here the low-level search runs on a **continuous-time graph**, where floor changes are transfer edges — one elevator ride `4.5 + 1.75·|Δf|`, one stair transfer `3.5·|Δf| / v_stair`.
- **Assignment (external addition)**: CBS has no assignment component. Tasks are pre-assigned to type-matching robots by a Hungarian min-cost matching on unconstrained path times; CBS then resolves all resource conflicts for that fixed assignment. This matches the experiment protocol (one task per batch; wheeled robots use elevators only).

### 4.6 MuNES（Jung et al., 2024）

**中文**
- **来源**：Jung, Kim, Cho, Kim, *MuNES: Multifloor Navigation Including Elevators and Stairs*, arXiv:2402.04535, 2024。
- **原方法定位**：MuNES 原文是**单机器人**多层轨迹规划，不含任务分配、不含多机冲突消解。
- **忠实保留（寻路层）**：
  1. **电梯等待感知代价**：每条电梯边 = 赶来等待 `1.75·|出发层−电梯当前层|` + 乘梯 `4.5 + 1.75·|Δ层|`，对应官方 `utils.calcCost`。
  2. **电梯全局状态**：电梯位置为全局状态，一次规划内固定，规划结束后更新到路径终点所在层（对应论文 `CURR_ELV_FLOOR` 语义）。
  3. **不可分割的电梯服务**：同一路径中连续电梯边构成一次完整服务（赶来 + 乘梯全程占用电梯）。
- **补齐的多机层（论文未覆盖）**：
  1. **分配 = 顺序贪心**：任务按释放时间排序，逐个交给类型匹配、静态行程（起点→取货→送货）最短的空闲机器人；无全局优化、无冲突预测。静态估计用无统一电梯等待的最短路。
  2. **冲突消解 = 全局 FIFO 等待**：后规划者若与既有预约窗重叠，整体顺延等待；**不改路径、不换电梯/楼梯**。走廊、楼梯与电梯均按预约窗 FIFO 占用。
- **适配说明**：这样接入后，MuNES 基线的智能部分只体现在寻路代价上；相对主算法的差距可归因到分配层与冲突消解层。

**English**
- **Source**: Jung, Kim, Cho, Kim, *MuNES: Multifloor Navigation Including Elevators and Stairs*, arXiv:2402.04535, 2024.
- **Source scope**: MuNES is a **single-robot** multi-floor navigation planner; it contains no task-assignment and no multi-agent conflict-resolution mechanism.
- **Retained (planning layer)**:
  1. **Elevator-waiting-aware cost**: each elevator edge = come-to-pick-up waiting `1.75·|departure floor − elevator floor|` + ride `4.5 + 1.75·|Δf|`, mirroring the official `utils.calcCost`.
  2. **Global elevator state**: elevator location is a global state, fixed within one plan and updated to the destination floor after planning (paper `CURR_ELV_FLOOR` semantics).
  3. **Indivisible elevator service**: consecutive elevator edges in one route form one continuous service (occupying the elevator through pickup-and-ride).
- **Added multi-agent layers (not covered by the paper)**:
  1. **Greedy assignment**: tasks in release order are given to the idle, type-matching robot with the shortest static travel (start→pickup→delivery); no global optimization, no conflict prediction. The static estimate uses an elevator-wait-free shortest path.
  2. **Global FIFO waiting**: a later planner whose window overlaps an existing reservation simply delays; **no route change, no elevator/stair switch**. Corridors, stairs, and elevators are all occupied under FIFO reservation windows.
- **Adaptation**: under this integration, MuNES contributes intelligence only in the planning-cost layer; its gap to the main method can be attributed to assignment and conflict resolution.

---

## 5. 对照：主算法 / For Contrast — Main Method

**中文**
- **任务分配**：匈牙利最小代价匹配 + 类型匹配；代价基于统一图上的可行行程时间。
- **冲突消解**：多趟竖向换路 pass——对被竖向资源卡住的机器人，枚举其他电梯、腿式楼梯与留队三种选项；仅当预估等待大于绕行额外代价时才换路；每机器人至多换路一次；冲突由等待、换资源、走楼梯三类手段联合消解。
- 与上述基线相比，主算法额外贡献了：全局分配的最优性、等待–绕行代价权衡、以及在电梯/楼梯间的主动资源切换。

**English**
- **Assignment**: Hungarian min-cost matching with type matching; costs use feasible travel times on the unified graph.
- **Conflict resolution**: a multi-pass vertical reroute pass enumerates, for a robot blocked on a vertical resource, the alternatives of another elevator, a leg-operated stair, or staying in queue; a switch is taken only when the estimated waiting exceeds the extra detour cost; each robot reroutes at most once. Conflicts are jointly resolved by waiting, resource switching, and stair detours.
- Relative to the baselines above, the main method additionally contributes global assignment optimality, an explicit wait-vs-detour trade-off, and proactive resource switching between elevators and stairs.

---

## 6. 复现与口径一致性 / Reproducibility and Consistency

**中文**
- 所有基线与主算法使用同一批 case、同一套物理参数、同一组指标定义；结果 JSON 同构，便于合并出图。
- Wait-only、Nearest、Strip、MAPD、CBS 的算法代码在本实验中未作策略性改动；MuNES 在适配框架内修正了 FIFO 登记时间戳的仿真缺陷（改用 shifted 时间戳重登记占用窗，并加入静态缓存），该修复只影响预约表登记的正确性，不改变论文寻路代价公式。
- 每个基线目录可独立运行：`python run.py --mode generic --input <case.json> --output <out.json>`。

**English**
- Every baseline and the main method runs on the same case set, the same physical parameters, and the same metric definitions; result JSONs are schema-aligned for pooled plotting.
- Wait-only, Nearest, Strip, MAPD, and CBS carry no strategic code changes in this experiment. Within the adaptation framework, MuNES had a FIFO registration timestamp defect fixed (occupancy windows are re-registered with shifted timestamps, plus a static cache); this repair affects reservation-table correctness only and leaves the paper's planning-cost formula unchanged.
- Each baseline is runnable standalone: `python run.py --mode generic --input <case.json> --output <out.json>`.
