# 服务器运行说明（SERVER_GUIDE）— 主算法 · multifloor_scheduler · v2+v3 数据集

在服务器 `zju-G5500-V6` 上跑**主算法**（冲突消解 = 每次取第一组冲突，组内比较"当前路径 vs 次优路径"的完成时间差，Δ 最大者保留当前路径、其余换次优，循环重检直至收敛）。

- 服务器 SSH：`10.98.36.40`，端口 **3822**，用户 `Weiyi`
- 环境：Anaconda + conda 环境 `myenv`（Python 3.10），主算法**零第三方依赖**
- 包解压后根目录为 **`multifloor_server`**（顶层目录名用英文，避免服务器 unzip 中文乱码），内含：
  - `server_run.py`：批量脚本（v2/v3 自动识别 + 自动物化 + 一键全跑/三包分片）
  - `SERVER_GUIDE.md`：本说明
  - `multifloor_scheduler/`：主算法（core.py + run.py）
  - `dataset_v2/`：控制变量法 225 case（A~G 组）
  - `dataset_v3/`：**自由组合 672 case**（机器人 8 档 × 负载比 4 档 × 楼层 7 档 × 3 种子）

---

## ① 上传包到服务器（本地 Windows PowerShell）

> `scp` 的端口参数用**大写 `-P`**，`ssh` 用小写 `-p`。

```powershell
scp -P 3822 "D:\Doubao\DoubaoWork\chats\2026-10-14\主算法_服务器运行包.zip" Weiyi@10.98.36.40:/data/Weiyi/
```

首次连接问 `Are you sure...` 输 `yes`，再输密码。看到 `100%` 即成功。

## ② 登录服务器、解压、激活环境（服务器）

```bash
ssh -p 3822 Weiyi@10.98.36.40
cd /data/Weiyi
ls                                    # 应看到 主算法_服务器运行包.zip
unzip 主算法_服务器运行包.zip
cd multifloor_server                  # 注意：顶层目录是英文名 multifloor_server
conda activate myenv
python --version                      # 应显示 Python 3.10.x
```

解压后应看到：`server_run.py`、`SERVER_GUIDE.md`、`multifloor_scheduler/`、`dataset_v2/`、`dataset_v3/`

## ③ 验证环境（跑一个小 case）

```bash
python server_run.py --cases A_R1_M1_F8_S17
```

**预期输出**：`[OK] A_R1_M1_F8_S17  ...  makespan=301.0  ...  final_conf=0`
`makespan=301.0` 说明环境正常。也可直接验证 v3 数据：`python server_run.py --cases R16_M16_rho1p0_F8_S17`（首次会自动物化，约 10 秒后开跑）。

## ④ 跑 v3 自由组合全部 672 case（三包分工 / 一键）

不用手抄 case 名，脚本自动从 v3 清单读取。分片为**轮转分配**（每片都含从小到大的完整规模谱，负载均衡）。

**三包分工**（三条命令，可同时放后台并行，服务器核多更快；每条约 224 case）：

```bash
nohup python server_run.py --all-v3 --shard 1/3 > run_shard1.log 2>&1 &
nohup python server_run.py --all-v3 --shard 2/3 > run_shard2.log 2>&1 &
nohup python server_run.py --all-v3 --shard 3/3 > run_shard3.log 2>&1 &
```

**一键全跑**（一条命令串行跑全部 672 case，最省心）：

```bash
nohup python server_run.py --all-v3 > run_v3.log 2>&1 &
```

**先看将跑哪些**（不执行，确认无误再放后台）：

```bash
python server_run.py --all-v3 --dry-run
python server_run.py --all-v3 --shard 2/3 --dry-run
```

**只跑指定 case（v2 / v3 可混）**：

```bash
python server_run.py --cases A_R64_M64_F8_S17 R256_M256_rho1p0_F16_S17
```

进度：`tail -f run_v3.log`（Ctrl+C 退出查看，不影响任务）。结果写入 `results/`（每 case 一个 JSON + `summary.csv`，多次运行自动追加不同 case 的行）。

## ⑤ 确认结果

```bash
cat results/summary.csv
```

每行一个 case：`makespan`、`sum_completion_time`（优化目标）、`total_wait_time`、`final_conflict_count`（应为 0）。

## ⑥ 取回结果（本地 PowerShell）

```powershell
scp -P 3822 -r Weiyi@10.98.36.40:/data/Weiyi/multifloor_server/results D:\Doubao\DoubaoWork\chats\2026-10-14\
```

---

## 规模与耗时参考（单 case，主算法，单线程）

| 规模 | 参考耗时 |
|---|---|
| R≤64 | ~16 s |
| R128 | ~32 s |
| R256 | ~63 s |
| R512 | ~数分钟 |
| R1024 | ~10 分钟量级 |
| R2048 / R5000 / 100 层 | 半小时~数小时，建议后台跑 |

## 常见问题

- **解压后没有顶层文件夹 / cd 失败**：确认用的是最新包（顶层目录英文名 `multifloor_server`）；旧包顶层是平铺的。
- **中文文件名解压乱码（如 цЬН...md）**：服务器 unzip 不支持 UTF-8 文件名所致；本包已把顶层目录和说明文件名改为英文（`multifloor_server`、`SERVER_GUIDE.md`），不再受影响。算法代码文件均为英文名，不影响运行。
- **`ModuleNotFoundError: No module named 'core'`** → 必须在 `multifloor_server` 目录内运行（`server_run.py` 已自动切目录，无需手动 cd）。
- **找不到 case** → case 名写错；v3 命名形如 `R256_M128_rho0p5_F16_S17`（含 `rho`），v2 命名形如 `A_R64_M64_F8_S17`。
- **物化慢/磁盘不足** → 大 case（R5000 / 100 层）物化要写几百 MB JSON，属正常；跑完可从 `dataset_*/cases/` 删除该 case，需要时会自动重新物化（结果不变）。
- **内存不足（R5000+100 层）** → 该组合 case 极大，若 MemoryError 说明当前机器不适合，换小 case 或更高内存机器。
- **不要修改 `multifloor_scheduler/core.py` 与两个生成器**，改任何一行都会破坏对比有效性。

## 包内容结构

```
主算法_服务器运行包.zip
└── multifloor_server/
    ├── server_run.py                  # 批量脚本（v2/v3 自动识别 + 自动物化 + 一键/分片）
    ├── SERVER_GUIDE.md                # 本说明
    ├── multifloor_scheduler/          # 主算法（core.py + run.py）
    ├── dataset_v2/                    # 控制变量法数据集（225 case 清单 + 生成器 + 地图）
    └── dataset_v3/                    # 自由组合数据集（672 case 清单 + 生成器 + 地图）
```
