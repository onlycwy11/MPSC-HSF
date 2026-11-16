# MAPD / Token Passing baseline — Faithful TP

## 对应论文

Hang Ma, Jiaoyang Li, T. K. Satish Kumar, Sven Koenig,
*Lifelong Multi-Agent Path Finding for Online Pickup and Delivery Tasks*,
AAMAS 2017.

## 实现说明（2026-10-13 重写）

本实现忠实保留 Token Passing 的核心机制，只按批次实验设定做了适配：

- **Token 机制**：任务按释放时间顺序被全局 token 依次处理；对当前任务，
  每个空闲且类型匹配的机器人基于**所有已分配机器人的当前路径预留表**
  （reservation table，即 token 状态）规划 `start → pickup → delivery`
  的避碰路径；完成时间最早的机器人获得该任务，其路径占用窗追加进预留表
  并广播给后续任务。
- **避碰规划**：低层为带预留约束的最短路径搜索（等价于 SIPP 的 reservation
  思想），支持等待；电梯一次乘梯为直达转移（`4.5 + 1.75*|Δf|` 秒），楼梯为
  直达转移（`3.5*|Δf| / v_stair` 秒），单行窄通道为容量 1 资源，双向通道为
  宽通道不参与冲突。
- **规划即避碰**：由于每个候选路径都在已有预留下规划，最终输出天然无冲突，
  无需事后冲突消解。

## 与论文的差异（需在论文中说明）

论文面向 lifelong 在线任务流（任务随执行持续到达、机器人连续承接多个任务）；
本实验为单 batch 离线设定且一个机器人至多一个任务，在线 token 循环退化为
批次内一次分配，TP 的"token 顺序 + 预留表避碰规划 + 最早完成者获得任务"
核心机制保留。TPTS（任务交换）在单任务 batch 中没有交换对象，未实现。

## 运行

```powershell
python run.py --mode generic --input <case.json> --output <result.json>
```

## 性能说明

TP 为贪心在线算法，规模扩展性好；每个任务只需为候选机器人各做一次带预留的
规划，预留表可增量维护。`iterations` 记录 token 处理的任务数。
