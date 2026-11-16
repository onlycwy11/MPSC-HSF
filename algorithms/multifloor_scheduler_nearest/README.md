# multifloor_scheduler_nearest — 水平近路优先变体

主算法（`D:\Python\multifloor_scheduler`）的一个冲突消解变体：

- 匈牙利初分配 + 最短路径规划（与主算法一致）；
- **只检测一次冲突**；对每个"将在垂直资源前冲突"的（机器人×路径段），按水平两段平面最短路之和（①段起点→入口 + ③出口→段终点，垂直段不参与比较）选择水平最近的电梯/楼梯并换路一次；
- 换路完成后剩余冲突**纯 FIFO 排队**（先到先服务；同时到达按机器人编号；同到再按通行时长短者优先），不再循环换路。

详细算法说明与伪代码见 **`ALGORITHM.md`**。

## 快速开始

```bash
# 虚拟示例
python run.py

# 指定输入/输出
python run.py --mode generic --input <case.json> --output out.json
```

## 输出

与主算法及各 baseline 完全同构的 JSON：

- `assignments`：每台机器人-任务对的路径、各资源占用、开始/完成时间；
- `resource_schedule`：全资源预约表（可跨 batch 传递）；
- `metrics`：makespan、完成时间总和、总等待、换路统计等。

## 依赖

仅 Python 标准库。Windows / Linux 均可运行。

## 目录

| 文件 | 说明 |
|---|---|
| `core.py` | 算法实现（含性能优化的 Dijkstra 缓存） |
| `run.py` | 命令行入口 |
| `ALGORITHM.md` | 算法说明与伪代码 |
