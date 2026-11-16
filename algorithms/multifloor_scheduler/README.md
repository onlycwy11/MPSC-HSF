# Multifloor Scheduler

这是一个独立的多层室内机器人批量任务分配项目。项目不依赖参考工程，也不修改参考工程文件；运行时只使用 Python 标准库。

## 功能

- 一个 batch 中每个机器人至多执行一个任务；不使用任务优先级。
- 机器人任务类型必须与任务需求类型完全一致；运动类型单独用于判断楼梯可达性。
- 默认平面速度为 1.5 m/s，楼梯垂直速度为 0.5 m/s，层高为 3.5 m；均可由配置或输入数据覆盖。
- 电梯一次行程耗时为 `4.5 + 1.75 * 跨越楼层数` 秒；同一电梯一次只服务一台机器人。相邻楼层电梯边会扩展为同一电梯的一次多层直达行程。
- 轮式机器人禁止使用楼梯，只能通过电梯跨楼层；其他机器人可按配置使用楼梯。
- 平面边按方向建模：相反方向的两条边表示双向宽通道，不产生迎面冲突；只配置单向边的窄通道不允许逆向通行。只有显式绑定容量资源的平面边才做时间窗互斥检测。
- 楼梯按物理楼梯及相邻楼层段预约，只检测同一楼层段的反向重叠；同向通行不互斥。
- 初始分配使用 Hungarian 最小代价匹配。
- 对电梯/楼梯等共享资源建立时间窗。处理一组冲突后，代码会重新对全部计划做全局冲突检测，直到无冲突、达到迭代上限或触发确定性串行兜底。
- 支持三种输入模式：虚拟地图、真实地图+SQLite、通用 JSON 输入接口。

## 运行

```powershell
python run.py --mode virtual --input examples\virtual_config.json --output result.json
python run.py --mode generic --input examples\generic_input.json --output result.json
python run.py --mode real --map examples\real_map.json --db examples\state.sqlite --output result.json
```

更简短的逐步说明见 [`docs/quick_start.md`](docs/quick_start.md)。

## 随机模拟输入生成器

`generate_simulation_map.py` 用命令行参数生成多层地图、机器人和任务输入；生成地图的每个节点含楼层及米制 `x/y/z` 坐标。示例：

```powershell
python generate_simulation_map.py --floors 5 --width 8 --height 8 --elevators 2 --stairs 3 --robots 20 --robot-types wheeled,legged --tasks 30 --task-types medicine,sample --seed 42 --output generated\hospital_5f.json
```

随后可用 `python run.py --mode generic --input generated\hospital_5f.json --output generated\hospital_5f_result.json` 运行。参数说明见快速使用文档。

真实模式的 SQLite 最小表结构：

```sql
CREATE TABLE robots(id TEXT, robot_type TEXT, task_type TEXT, start_node TEXT, speed_xy REAL, speed_z REAL, available_at REAL);
CREATE TABLE tasks(id TEXT, pickup TEXT, delivery TEXT, task_type TEXT, release_time REAL, status TEXT);
CREATE TABLE resource_schedule(resource TEXT, robot_id TEXT, task_id TEXT, start REAL, end REAL, direction INTEGER);
```
缺省任务类型统一按 `general` 处理，因此不会跳过类型相等性检查。`available_at` 表示机器人已承诺任务的预计空闲时刻。

通用 JSON 的顶层字段为 `map`、`robots`、`tasks`、`config`。机器人字段包含运动类型 `robot_type`、任务能力类型 `task_type`、`speed_xy`、`speed_z` 和可选 `available_at`；任务字段包含必需类型 `task_type`。地图包含 `nodes`、`edges`、`resources`；边字段包括 `src`、`dst`、`length`、`kind`（`floor/elevator/stair`）和可选 `resource`。完整示例见 `examples/generic_input.json`。

## 输出与对比指标

输出包括每项任务的机器人、路径、开始/完成时间、移动时间、等待时间和垂直通行方式，并提供 `resource_schedule` 预约表。JSON 模式把该字段随下一批输入传回；SQLite 模式使用同名表保存预约。`projected_node` 和 `available_at` 用于更新下一批的机器人状态。统一的 `metrics` 包括 `makespan`、完成时间总和、平均移动时间、总等待时间、初始/最终冲突数、冲突处理迭代数和重规划次数。基线项目使用相同字段，便于批量对比。
