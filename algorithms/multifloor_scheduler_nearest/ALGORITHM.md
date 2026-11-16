# 水平近路优先冲突消解变体（Nearest-Vertical-First）

> 算法说明与伪代码。完整可运行实现见同目录 `core.py` / `run.py`。

## 0. 一句话

在匈牙利初分配、每台机器人按最短路径（两段：取货前、取货后）规划的基础上，**只做一次冲突检测**；对被判定"将在垂直资源（电梯/楼梯）前冲突"的机器人，按**水平两段平面最短路长度之和**（①段起点→本层入口平面最短路 + ③目标层出口→段终点平面最短路，**垂直段②不参与比较**）选择水平最近的垂直资源换路一次；换路完成后剩余冲突一律按 **FIFO 排队**（先到先服务），不再循环换路。

优化目标：最小化所有机器人-任务对执行任务的完成时间之和。

## 1. 问题定义（与主算法相同）

- 地图为多层图，节点含楼层/坐标；边分为三类：平面边（floor）、电梯边（elevator）、楼梯边（stair）。
- 宽双向平面边容量不限（不产生冲突）；窄通道/电梯/楼梯为容量 1 的共享资源。
- 机器人有类型；任务有类型；**类型必须匹配才能分配**。
- wheeled（轮式）机器人不能走楼梯；legged（腿式）可走电梯或楼梯。
- 电梯一次乘梯：`4.5 + 1.75 × |Δ楼层|` 秒（开关门+上下客 4.5s，每层 1.75s）；楼梯跨 n 层：`3.5 × n / 0.5` 秒；平面：`距离 / 1.5` 秒。均为默认值，可配置。
- Batch 型：一轮 batch 内任务已知，初分配到冲突消解全程在模拟中完成，输出分配后机器人再执行。

## 2. 总体流程

```
输入: 地图, 机器人集合 R, 任务集合 T, 配置, 已有预约表
1. 匈牙利初分配
     成本矩阵 C[i][j] = (机器人 i 到任务 j 的最短路径时间)   [类型不匹配 → ∞]
     求得 R→T 的一一分配（多退少补用罚值）
2. 路径构造
     每台被分配的机器人构建完整路径，划分为两段:
       段1: start -> pickup   (A→B)
       段2: pickup -> delivery (B→C)
3. 一次冲突检测
     按资源聚合所有占用区间(电梯/楼梯/窄通道)，排序后检测重叠
     仅标记: 同一垂直资源上 "后到且与先到者重叠" 的区间
4. 水平近路优先换路（每机器人每冲突段至多一次，不循环重检）
     对每个被标记的 (机器人, 冲突段):
       段起点 s = 段1 起点(A) 或 段2 起点(B); 段终点 e = 段1 终点(pickup) 或 段2 终点(delivery)
       可选垂直资源 = s 所在楼层有入口、且 e 所在楼层有出口的 电梯 ∪ 楼梯 (wheeled 只电梯)
       对每个候选资源只算水平两段平面最短路之和(垂直段②不参与):
         ① s→入口(平面最短路, 只走 floor 边)
         ③ 出口→e(平面最短路, 只走 floor 边)
         horiz = ① + ③
       选水平最近资源:
         · 水平最近资源 == 当前资源 → 保留，排队等待
         · 水平最近资源 != 当前资源 → 换路: s→入口(平面) → 乘资源(垂直) → 出口(平面)→e
5. FIFO 排队（不再换路）
     按 (理想开始时刻, 机器人编号, 乘梯/通行时长) 排序逐计划贴合预约表
     同起点同刻出发的机器人按编号错开 1s
6. 输出分配、路径、资源预约表与指标
```

## 3. 冲突判定细节（一次检测）

对每个资源 `r`，取其占用区间 `(robot, s, e)`，按 `(s, e, robot)` 排序：

```
active = 堆(按 e 升序)
for current in 排序后的区间:
    弹出 active 中 e ≤ current.s 的区间        # 已结束
    dir_active = active 中方向与 current 不同者   # 楼梯同向不冲突
    if dir_active 非空:                        # 电梯/楼梯容量为 1
        标记 current.robot 为 "在垂直资源前冲突"
    active.push(current)
```

- 只对**后到者**（current）标记：先到者已在用资源，后到者被判定冲突 → 由后到者换路（与"先到先执行"的直觉一致）。
- 平面窄通道冲突**不触发**水平近路（本变体只对垂直资源前冲突换路；平面冲突在排队阶段统一按 FIFO 解决）。

## 4. 水平近路换路（一次，不循环）

对每个被标记的 (robot, segment)，**只按水平两段平面最短路之和**选资源，垂直段不参与比较：

```
对每个被标记的 (robot, segment):
    s   = segment 起点      # 段1→A=robot.start_node；段2→B=task.pickup
    e   = segment 终点      # 段1→pickup；段2→delivery
    f_s = s 所在楼层; f_e = e 所在楼层
    cand = []
    for kind in {电梯, 楼梯}:                      # wheeled 只取电梯
        for res in 该 kind 的资源:
            if f_s 不是 res 的入口楼层 or f_e 不是 res 的出口楼层: continue
            entry = res 在 f_s 层入口中, s 平面最近者
            exit  = res 在 f_e 层出口中, e 平面最近者
            ① d_plane_in  = s → entry 的平面最短路 (仅 floor 边)
            ③ d_plane_out = exit → e 的平面最短路 (仅 floor 边)
            horiz = ① + ③                        # 垂直段②不计入
            cand.append((horiz, kind, res, 完整段 seg'))
    选 horiz 最小者
    if kind == 当前段的 kind 且 res == 当前资源: continue   # 水平最近的就是当前资源 → 保留等待
    以 seg' 替换该段, 更新机器人整条路径
```

关键点：

1. **只比水平两段，垂直段②不参与**：选择依据 = ①段起点→本层入口平面最短路 + ③目标层出口→段终点平面最短路；垂直通行时间（电梯 board/楼层、楼梯爬升）**不进比较**。这是"水平近路优先"的本义：只挑水平上少走路的资源，垂直怎么坐不影响选择。实际换路后走的仍是 ①平面+②垂直+③平面 的完整路线。
2. **段起点，不是机器人当前位置**：路径 A→B（取货段）冲突取 A；B→C（送货段）冲突取 B。同一机器人在两段都有冲突时，两段各自按各自起点分别处理一次。
3. **平面段用实际图最短路，不用直线距离**：①③用真实地图 floor 边 Dijkstra（`allowed_vertical="floor"`），与路径规划同源；平面速度统一，用距离之和或行走时间之和排序等价。
4. **每冲突（机器人×段）使用一次**：检测一次、换路一次，之后**不循环重新检测**——换路后的机器人各自走向水平最近的垂直资源，剩余冲突交给排队。
5. 若水平最近的资源就是当前正在使用的资源 → 不换，排队等先到者用完。

## 5. 排队顺序（明文规则）

换路完成后，所有冲突按资源排队解决：

1. **先到先服务（FIFO）**：按到达该资源入口的时刻排队，先到者先使用。
2. **同时到达 → 按机器人编号**（与既有楼梯"同层对向先入先走、同时进入按编号"规则一致）。
3. **同到再按通行时长**：仍无法区分时，乘梯/通行时间短者优先（降低资源占用时间）。

实现方式：按 `(理想开始时刻, 机器人编号, travel_time)` 排序逐计划贴合预约表（重叠时后到者顺延），与主算法的排队机制一致，但**不做任何换路**（`resolve_wait_only`）。

## 6. 与主算法（multifloor_scheduler）的差异

| 环节 | 主算法 | 水平近路变体 |
|---|---|---|
| 初分配 | 匈牙利（最短路径时间） | 同 |
| 冲突检测 | 循环：检测→解决一组→重检 | **一次**，不循环 |
| 换路依据 | 组内 Δ 最大者保留当前、其余换次优 | 各候选资源**水平两段平面最短路之和（①起点→入口 + ③出口→终点）最小者**，垂直②不参与 |
| 排队 | 贴合预约表（可能换楼梯） | 纯 FIFO 贴合，不再换路 |
| 复杂度 | 迭代收敛 | 单遍换路 + 一遍排队 |

## 7. 伪代码（紧凑版）

```
ScheduleNearest(building, R, T, cfg):
    plans ← HungarianAssignment(R, T, cfg)          # 类型匹配，成本=最短路径时间
    for p in plans:
        p.route ← ShortestPath(p.robot, p.task)     # 完整路径, 两段
    mark ← DetectOnce(plans)                         # 垂直资源上"后到重叠"的 (robot, seg)
    for (robot, seg) in mark:
        s ← seg.start; e ← seg.end
        best ← argmin_{(kind,res)} [ Planar(s→entry) + Planar(exit→e) ]
                                    # 只比水平两段之和；垂直 Ride 不参与选择
        if best ≠ seg.current_resource:
            seg' ← Planar(s→entry) ⊕ Ride(entry→exit, best) ⊕ Planar(exit→e)
            ReplaceRoute(robot, seg')
    WaitQueueFit(plans, cfg)                         # FIFO, 编号/时长 tie-break, 不换路
    return metrics(plans)                            # 完成时间和等指标
```

## 8. 运行

```bash
# 虚拟场景（默认 3 层 6×6、1 电梯 1 楼梯、6 机器人 6 任务）
python run.py

# 指定参数
python run.py --mode virtual --input config.json --output result.json

# 数据集 case / 任意通用输入（与主算法、各 baseline 同一接口）
python run.py --mode generic --input D:\Python\icde_experiments\dataset_v2\cases\A_R64_M64_F8_S17.json --output out.json

# 真实院区（SQLite）
python run.py --mode real --map map.json --db tasks.db --output out.json
```

输出 JSON 结构与主算法/各 baseline 完全一致：`assignments / unassigned_tasks / resource_schedule / metrics`。

## 9. 实现说明

- `core.py` 与主算法共享同一套物理模型与数据结构（BuildingMap/Robot/Task/Plan/Edge…），并复用其性能优化（整数图、反向距离列缓存、路径缓存）。
- 换路后的新段通过"平面→垂直→平面"三段拼接，垂直段限定走选定资源（`allowed_vertical=kind`）；平面段限定只走 floor（`allowed_vertical="floor"`）。
- 该变体是**独立的算法实现**（自洽文件夹、自洽入口），与主算法、纯等待基线相互独立，便于在论文中作为"算法 2（水平近路变体）"与"等待基线"对比。
