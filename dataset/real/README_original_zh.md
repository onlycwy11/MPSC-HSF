# ICDE 真实医院院区数据集 v1（real_1-7F + 公开基准场景）

> **给组员/协作者**：先看 `QUICKSTART.md`（三步跑起来 + 常用命令），本文件是详细设计口径。

真实多楼层机器人调度数据集：以 ROS 占用栅格地图（PGM + YAML）为基础构建的**七层医院院区**
（浙江医院三墩院区：门诊楼 6 层 / 住院楼 14 层，本数据集选取代表性 1~7F），
另附两个公开基准场景（MovingAI MAPF warehouse 叠 8 层 / 10 层）。
全部地图可直接被主算法与各 baseline（nearest / wait / munes / cbs / strip）以 generic 单文件 case 加载运行。

## 目录结构

```
dataset_real_v1/
├── build_real_map.py          # 栅格地图 → 1m 网格 map JSON（可复现重建）
├── build_warehouse_maps.py    # MovingAI warehouse 栅格 → 8F/10F 公开基准多楼层 map（可复现重建）
├── generate_real_cases.py     # 生成 204 个骨架 case + case_manifest_real.json
├── run_all_real.py            # 一键执行：组装 case → 跑算法/baseline → 断点续跑 → summary.csv
├── raw_public/                # MovingAI warehouse 源 .map（公开下载，可复现）
├── maps/
│   ├── real_1-7F.json         # 主产物：真实院区 7 层 {nodes, edges, resources, resource_metadata, meta}
│   ├── movingai_warehouse_8F.json   # 公开基准：warehouse 8 层
│   ├── movingai_warehouse_10F.json  # 公开基准：warehouse 10 层
│   └── topo_points.json       # 拓扑导航点名 → 网格 node_id 映射
├── manifests/
│   └── case_manifest_real.json      # 204 case 清单（场景/R/M/负载比/seed/map_ref）
├── cases/                     # 204 个骨架 case（map_ref 引用，运行时组装成单文件）
└── README.md
```

## 楼层编排（用户确认口径）

```
1F = 三墩门诊/一楼/sdmzl_zg_floor1D.pgm（有拓扑导航点，电梯口 p1 与竖井同列）
2F = 三墩门诊/二楼/sdmz_floor2.pgm（有拓扑导航点，电梯口 p1 与竖井同列）
3F = sdmz_floor3（桌面，有拓扑；三墩门诊/三楼版本核对一致）
4F = 四楼已编辑.pgm（RAR 版，无拓扑 → 任务点回退走廊随机）
5F = 五楼_1.pgm（RAR 版，无拓扑 → 任务点回退走廊随机）
6F = floot_1 + floot4_inner 并入（桌面，有拓扑导航点）
7F = floot_2（桌面，有拓扑导航点）
```

## 公开基准场景（MovingAI MAPF，可引用）

```
movingai_warehouse_8F  = warehouse-10-20-10-2-1(161x63) 与 warehouse-10-20-10-2-2(170x84) 交替叠 8 层
movingai_warehouse_10F = warehouse-20-40-10-2-1(321x123) 与 warehouse-10-20-10-2-1 交替叠 10 层
来源：Sturtevant, "Benchmarks for Grid-Based Pathfinding", IEEE TCIAIG 2012; movingai.com/benchmarks/mapf/
竖井模型与真实院区一致（电梯/楼梯贯穿全部楼层，容量 1）；无拓扑导航点 → 任务点用走廊节点
```

## 地图建模口径

| 项 | 取值 |
|---|---|
| 楼层 | 真实院区 1F~7F 共 7 层（floor 0~6，z = 0~21 m）；warehouse 8F / 10F |
| 网格 | 1 m，cell 内 ≥50% 像素占用 → 阻塞；PGM 灰度遵循 ROS 惯例（黑=占用） |
| 电梯 | **竖井贯穿全部楼层**，真实院区口部 XY = (-0.03, -0.16)（3F 拓扑 p7；1F/2F 的 p1=(0,0) 与竖井同列，已核验）；resource `elevator-0`，容量 1；边只写相邻层，算法自动补全跨层直达 |
| 楼梯 | **竖井贯穿全部楼层**，真实院区口部 XY = (5.55, 13.01)（4F 拓扑 p6）；resource `stair-0`；每层段 length = 14.88 m |
| 节点/边 | real_1-7F：nodes=36765 / edges=145184；8F：61900 / 205468；10F：141490 / 439396 |

资源口节点 `node_type = elevator_lobby / stair_lobby`，不可作为任务点；其余为 `corridor`。

## 竖井贯穿模型

- **电梯竖井**：一条垂直线贯穿全部楼层，真实院区口部 XY = 3F p7 (-0.03, -0.16)。
- **楼梯竖井**：一条垂直线贯穿全部楼层，真实院区口部 XY = 4F p6 (5.55, 13.01)。
- 7F 拓扑 p1 按用户确认视为 7F 电梯口（与电梯竖井 XY 差 <1m，同一网格 cell）。
- 7F 楼梯口：楼梯竖井 XY 超出 7F 图幅（y 最高 9.9m），按竖井投影取图内最近可通行 cell，meta 已注明。

## 任务点

- 1F：p2~p13, p15~p17（p1/p14/p18 为电梯/连廊节点，排除）
- 2F：p2~p9（p1/p10 位于电梯竖井列，排除）
- 3F：p2~p6, p8（p7 为电梯口，排除）
- 6F：p1~p5, p7（p6 为楼梯口，排除）
- 7F：p2~p5（p1 为电梯口，排除）
- 4F/5F（RAR 版）与 warehouse 场景：无拓扑导航点，任务取送点自动回退到该层走廊节点

## 已知口径与待确认

- RAR 版 4F/5F 无坐标标定（0528-6.yaml 配套 0528-6.pgm 缺失），origin 按同批假设
  [-20.3, -44.8, 0] 对齐；这两层竖井投影为建模近似，meta 已注明。
- RAR 版 4F/5F 与桌面版 4F/5F **不是重复平面**（已逐图对比：轮廓/布局均不同），为独立楼层。
- 电梯/楼梯物理口径沿用主算法锁定实现：电梯容量 1 串行 FIFO，乘梯 4.5+1.75×层差，
  空驶接驾与步行并行；楼梯仅 legged 可走。
- floot4_inner 与 floot_1 重叠区以 floot_1 为准，floot4_inner 只补充未覆盖区域。
- 待办：真实 case 规模档位（R/M/seed 组合）与任务生成分布策略。

## 重建 / 生成 / 运行

```bash
# 1. 重建 map（原始素材在 C:\Users\ONLY\Desktop\地图\）
python build_real_map.py
python build_warehouse_maps.py

# 2. 生成完整 case 集（204 个骨架 case + manifest）
python generate_real_cases.py

# 3. 主算法运行（generic 模式）
python D:\Python\multifloor_scheduler\run.py --mode generic \
  --input cases\real_R16_M8_rho0p5_S17.json --output out.json

# 4. baseline 同构加载（共用 BuildingMap 解析）
```

## 完整 case 集（204 个，三场景）

| 场景 | 地图 | R 档位（台/层） | case 数 |
|---|---|---|---|
| real  | 真实院区 1-7F（约 3.7 万节点） | 16 / 64 / 128 / 256 / 512（≈73 台/层） | 60 |
| wh8   | MovingAI warehouse 8F（6.2 万节点） | 16 / 64 / 128 / 256 / 512 / 1024 | 72 |
| wh10  | MovingAI warehouse 10F（14.1 万节点） | 16 / 64 / 128 / 256 / 512 / 1024（≈102 台/层） | 72 |

- 负载比 rho ∈ {0.5, 1.0, 1.5, 2.0} → 任务数 = round(R × rho)；种子 S ∈ {17, 29, 43}。
- **机器人上限按真实面积/楼层规模设计**（对比 v3 模拟集的 4096）：真实场景最高 512~1024 台，
  10 层楼配 4096 台（≈410 台/层）不符合医院/仓储物理密度，论文中也不利于审稿人信服。
- 任务口径与 v3 模拟集一致：75% 轮式 / 25% 腿式，任务楼层均匀，**50% 跨层任务**（取送在不同楼层，
  保证电梯/楼梯参与调度），取送点优先真实拓扑导航点（无拓扑的 RAR 层/公开场景回退走廊节点）。
- **骨架 case 体积控制**：真实地图 18~60MB，204 个 case 若内嵌地图将达 7GB+。
  采用 `map_ref` 引用（case 几 KB），`run_all_real.py` 运行时组装为单文件并即时清理临时文件。

## 一键执行（Windows PowerShell）

```powershell
# 主算法全量（204 case，断点续跑，中断后重跑即从断点继续）
python run_all_real.py --algo main

# 全部算法 + 全部 baseline（main / nearest / wait / munes / cbs / strip / mapd）
python run_all_real.py --algo all

# 单场景 / 单档位 / 单个 case 子集
python run_all_real.py --scenes real
python run_all_real.py --robots 512 --algo main nearest wait
python run_all_real.py --case real_R512_M512_rho1p0_S17

# 并发控制：小档 --workers，大档（R≥256）--big-workers
python run_all_real.py --algo main --workers 2 --big-workers 4
```

- 输出：`results/<algo>/<case_id>.result.json`（含 makespan / sum_completion / wait / 冲突数 / 耗时）
- 汇总：`results/<algo>/summary.csv`（每次运行自动重写，覆盖已完成 case）
- 断点续跑：已存在 result.json 的 case 自动跳过；进程被杀后重跑同一命令即可。
- 算法入口路径在脚本顶部 `SCHEDULERS` 字典中，如路径不同可自行修改。

## 示例验证结果

**一键脚本冒烟（真实医院 real_R16_M8_rho0p5_S17）**
```
main     makespan=229.75  wait=458.25  冲突=0
nearest  makespan=213.25  wait=350.00  冲突=0
wait     makespan=229.75  wait=458.25  冲突=0
```
**一键脚本冒烟（10F 公开基准 wh10_R16_M16_rho1p0_S17）**
```
main  makespan=722.67  wait=2891.42  冲突=0  （任务 16 > 机器人 16，排队等待属正常）
```
**早期三场景示例（r8_m10_s17，内嵌地图版）**
```
real_1-7F：makespan=94.67   0 冲突
wh8：      makespan=152.67  0 冲突
wh10：     makespan=394.0   0 冲突，电梯事件 1
```
R < M 时每个机器人执行一个任务，剩余任务未分配属正常行为。
