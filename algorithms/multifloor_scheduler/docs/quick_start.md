# 快速使用

在 PowerShell 中进入项目目录：

```powershell
Set-Location D:\Python\multifloor_scheduler
```

运行虚拟配置：

```powershell
python run.py --mode virtual --input examples\virtual_config.json --output result.json
```

运行已有 JSON 输入：

```powershell
python run.py --mode generic --input examples\generic_input.json --output result.json
```

运行真实地图和 SQLite 状态：

```powershell
python run.py --mode real --map examples\real_map.json --db examples\state.sqlite --output result.json
```

## 生成模拟地图

`generate_simulation_map.py` 专门生成随机拓扑地图、机器人和一个 batch 的任务。下面生成 5 层、2 部电梯、3 部楼梯、20 台机器人和 30 个任务，并写入 `generated\hospital_5f.json`：

```powershell
python generate_simulation_map.py --floors 5 --width 8 --height 8 --elevators 2 --stairs 3 --robots 20 --robot-types wheeled,legged --tasks 30 --task-types medicine,sample --seed 42 --output generated\hospital_5f.json
```

然后运行该输入：

```powershell
python run.py --mode generic --input generated\hospital_5f.json --output generated\hospital_5f_result.json
```

可以通过 `--width`、`--height`、`--grid-spacing` 设置每层网格布局和节点间距；`--floor-height` 设置层高。节点输出 `floor` 和以米计的 `x`、`y`、`z`，其中 `z = floor * floor_height`。同一个随机种子会生成相同地图和输入。

电梯和楼梯位置从每层网格位置中随机抽取，且彼此不重叠；每部电梯/楼梯贯穿所有楼层。地图默认使用双向平面边。宽度乘高度必须至少容纳所有电梯和楼梯位置；任务数大于 0 时还需留一个普通通道节点作为任务点。

机器人类型列表循环分配为运动类型，任务类型列表循环分配给机器人和任务，类型必须匹配才能分配。例如 `--robot-types wheeled,legged --task-types medicine,sample`。若要生成指定且不均衡的类型数量，可在后续配置 JSON 中编辑 `robots` 与 `tasks` 数组。
