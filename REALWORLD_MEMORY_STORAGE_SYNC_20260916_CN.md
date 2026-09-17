# 真机 Memory Storage 接入与静止测量（2026-09-16）

## 当前状态

新增显式配置入口，连接研究工作区已有的 `native_interval7` 写入、
`detector_support` 历史档案，以及 `reader_precision` / `lossless_bf16` KV。
**生产默认仍是 `legacy / dense / native`，未启动机器人运动。**

四组独立服务配对测量已完成：每组从同一真实 Survey 重放 120 张 RGB，
安装原目标，再从 Jetson 发送同一组 196 个冻结 RGB 请求。
累计 784 次请求全部完成，其中 544 次实际运行 GEM + NavDP 规划，
240 次只更新几何。没有启动 ROS 执行器或机器人运动。

新版稀疏档案与完整档案的 136 次规划输出逐项相同；在此基础上启用无损 KV，
136 次输出仍相同。规划往返中位数分别为旧版 697 ms、新版完整档案 388 ms、
新版稀疏档案 336 ms、稀疏档案加无损 KV 334 ms。
这些是准备好 JPEG 后的 HTTP 往返，不是实时相机到运动的延迟。

本次独立证据目录：
`/home/asus/Research/Nav-graph-blind/.diagnostics/realworld_memory_sync_20260916_ryhKtJ/`。

## 为什么原部署不等于论文的存储优化配置

旧部署已使用同一 GEM 对象、同帧深度复用、`online_history` 历史读取
和目标位姿缓存。但是，它仍保存完整历史深度/置信度，未选择新版写入和 KV 存储。
可选存储代码存在，并不表示启动入口已启用它。

此前约 1.01 Hz、RGB 源时间戳至规划响应中位数 0.821 s，属于旧部署的真实
静止测量。它既不是新存储配置的测量，也不是最大可达模型频率。

需要区分三件事：

- 同帧深度复用：每张新 RGB 仍产生新深度，控制读出不重复计算该帧。
- 稀疏历史档案：只保留固定关键点插值所需的历史深度/置信度样本。
- Survey 状态快照：持久化整个 episode 以减少下次初始化重放；不是 KV 无损编码。

## 配置接口

在独立 system 配置的 `stack.cec` 中加入：

```json
{
  "historical_depth_source": "online_history",
  "eager_depth_cache": false,
  "memory_storage": {
    "mechanism": "native_interval7",
    "geometry": "detector_support",
    "kv": "reader_precision"
  },
  "survey_initialization": "rgb_replay"
}
```

这是对原有 `stack.cec` 的增量字段，不替换其他配置。
`kv` 还可显式选择 `lossless_bf16`。不配置这些字段时，旧模式、W32、
原 flow gate 和 Survey 快照行为保持不变。

新版写入固定采用已验证的 W64 和 interval7：每张观测都预测几何并入档案，
每七张观测提交一次工作 KV。它不同于仅改变存储格式，不能据此声称与旧写入
具有完全相同的几何和导航轨迹。BF16 与无损编码的比较须保持同一写入协议。

## Survey 初始化适配

原 `export_episode_state` / `restore_episode_state` 只支持旧 flat 状态格式。
本次不伪造兼容快照，也不改变已有快照：新版明确选择 RGB 重放，仍经过原
dataset manifest、帧序、目标安装和查询起点 FIFO 初始化流程。

加载收据报告 `survey_initialization=rgb_replay`、重放帧数和恢复帧数。
旧 checkpoint 模式仍可使用；它不能搭配 `native_interval7`。

## 改动范围

- `deployment/runtime_config.py`：解析可选存储配置，生成不可变运行配置和启动参数。
- `deployment/gpu/scripts/run_memnav_server.sh`：仅在显式选择新版时启用 W64 和存储选项。
- `deployment/gpu/realworld_cec_hub.py`、`scripts/run_cec_hub.sh`：显式选择 Survey RGB 重放。
- `deployment/gpu/resident_policy.py`：驻留模型身份包含存储配置，避免跨配置复用旧模型。
- `deployment/gpu/resident_memnav_server.py`：报告实际存储模式；状态查询不遍历稀疏档案
  并重建所有历史深度图。
- `deployment/gpu/measure_memory_replay_latency.py`：Jetson 端冻结 JPEG 请求回放器，
  不导入 ROS 或 Unitree，不发布运动、到达或停止指令。

未修改研究工作区的模型、控制器、几何验证参数和尺度公式。未修改原始 Survey、
原实验登记、论文结果。未将生产默认切到新模式，也未覆盖 Jetson 现有工作区。

## 配对测量设计

固定先前静止实测中的 120 帧 Survey、目标 JPEG、相机内参、0.42 m 高度，
以及随后 196 个观测（136 次规划、60 次仅几何更新）。固定随机种子。

| 配置 | 写入 | 历史档案 | KV |
|---|---|---|---|
| 当前部署 | legacy / W32 | 完整图 | native |
| 新写入参考 | interval7 / W64 | 完整图 | BF16 |
| 新稀疏档案 | interval7 / W64 | detector support | BF16 |
| 可选无损编码 | interval7 / W64 | detector support | lossless BF16 |

四组均从 RGB 重建同一历史，不使用 Survey 快照。保留第一条查询延迟；稳态
统计排除原记录中标为 warmup 的请求，规划与仅几何接口分别统计。

测量口径是 **Jetson 准备好的 JPEG 请求 → RTX 处理 → 完整响应回到 Jetson**。
不包含本轮新相机采集、JPEG 编码、到达匹配、ROS 调度和电机执行。
它用于隔离存储/写入影响，不能替代实时 RGB 源时间戳到控制响应的测量。

本轮经用户允许，仅暂停本机论文队列调度父进程的下一任务领取，正在执行的
task 8 自然完成并通过独立校验后才使用 GPU。四组测量结束后已恢复调度进程；
测量服务、端口和临时反向隧道均已退出。任务结果、原始 Survey 和正式登记未改。

## 完整测量结果

每组稳态规划统计 N=121，排除原记录中的 15 次 warmup；几何接口 N=60。
首次查询单列，不与稳态统计混合。所有数值来自这一组固定 Survey/目标，
不是跨场景平均，也不代表运动中的导航成功率。

| 配置 | 规划中位数 / P95 | 首次查询 | 仅几何中位数 / P95 | 历史数组载荷 | 几何模块峰值 allocated |
|---|---:|---:|---:|---:|---:|
| legacy / 完整图 / native | 696.8 / 800.1 ms | 1106.0 ms | 253.6 / 264.1 ms | 630.52 MiB | 22.91 GiB |
| interval7 / 完整图 / BF16 | 387.7 / 490.8 ms | 796.3 ms | 249.3 / 300.5 ms | 646.90 MiB | 10.77 GiB |
| interval7 / 稀疏档案 / BF16 | 336.0 / 394.1 ms | 648.5 ms | 195.4 / 215.8 ms | 16.24 MiB | 10.77 GiB |
| interval7 / 稀疏档案 / 无损 BF16 | 334.4 / 393.7 ms | 752.7 ms | 193.6 / 208.8 ms | 16.24 MiB | 10.06 GiB |

历史数组载荷只计 depth/confidence 和稀疏格式的相应索引元数据，不是总 RAM、
RGB 档案或神经状态。四组终态都是 316 次观测；legacy 的在线深度缓存从初始化后
开始，共 308 帧，新版包含 316 帧。存储倍率应比较同为新版的完整／稀疏组：
**646.90 / 16.24 = 39.82 倍**。实际压缩归档文件分别为 506,991,579 和
11,459,244 bytes，与数组载荷是不同口径。

GPU 列是几何服务整个生命周期（含模型初始化、Survey 和查询）的 PyTorch
`max_memory_allocated`，不含独立 NavDP 进程；不能当作稳态 KV 大小。
本轮无损 KV 比相同 BF16 稀疏组减少约 0.70 GiB、6.5%，延迟基本持平。
`nvidia-smi` 1 Hz 采样的整卡峰值依次为 26,723、14,305、14,305、12,399 MiB，
包含 NavDP、CUDA 保留内存及显示占用，不与上面的 allocated 数值混用。
316 帧流最终只提交 52 个 KV 视图；这里没有测量满 W64 或更长序列的收益。

120 帧 Survey 的 RGB 重放时间依次为 26.87、25.42、19.40、19.98 s；
四组都未加载或保存旧状态快照。

### 提速来自哪里

- 新旧组的稳态检索阶段中位数从 179.85 ms 降为 0.48 ms。
  旧 `policy_agent.plan(retrieval_only=True)` 仍进入快照、完整规划缓存整理及
  恢复路径；新版直接调用独立历史检索。描述子增量缓存也是新版的一部分，
  不能把全部提速单独归因于描述子缓存或 KV 压缩。
- 新版完整／稀疏档案的写入时间中位数为 228.19 / 172.76 ms（查询期 196 帧）。
  这一时间包括该写入路径的处理与归档，不能解释为神经网络 FLOPs 减少。
- 当前深度仍由每张新 RGB 计算；544 次规划全部复用该帧预测，RGB SHA 和帧号
  全部匹配，传感器 metric depth 消费次数为 0。深度载荷物化仍约 33 ms。
- 位姿缓存命中后的定位更新约 0.3 ms；无损 KV 不是降低首次匹配延迟的机制。

### 输出一致性与迁移边界

以下两组比较，136/136 次规划均逐项一致：

1. 新版完整档案 BF16 → 新版稀疏档案 BF16；
2. 新版稀疏档案 BF16 → 新版稀疏档案无损 BF16。

核对项包括深度 PNG SHA、尺度收据、选中支持帧、几何验证结果、unit bearing、
点提示、选中轨迹、所有候选轨迹和 critic 评分；数值字段最大绝对差为 0。
因此，本轮实际服务调用支持这两项存储优化的输出等价性。

**旧版 → 新版写入不等价。** 旧版选支持帧 10，新版三组选帧 17；初始 bearing
相差 13.92°，全 136 次规划的方向差异中位数 75.99°、P95 94.54°。
这是两个预测结果之间的差异，不是针对 GT 的误差。没有位置真值或运动试验，
不能据此判断哪版更准，也不能以延迟改善为由直接继承旧版导航 SR。
本次没有修改控制器、尺度规则或几何验证参数，但改变写入协议本身会改变
历史几何和后续定位。因此生产默认暂不切换。

原始文件位于证据目录下 `paired_{legacy,dense_bf16,support_bf16,support_lossless}/`；
统一复算结果为 `paired_analysis.json`，工具为 `analyze_paired.py`。
输入身份保存在 `input/manifest.json`；32 个固定研究源码／部署文件的哈希未变化。

## 已完成检查与尚未完成的部分

- Python 语法、实际 GPU 环境下 Hub CLI 导入、Shell 语法通过。
- 原运行配置仍解析为旧默认；新配置正确传到 W64 / detector_support / reader_precision。
- 原源码深度复用校验通过，没有重写原固定源码收据。
- 前置集成检查完成 120 帧 Survey 和 3 次规划；随后四组正式延迟测量全部完成。
- 完整四组延迟／显存／输出配对已完成；无损组使用现有 Triton 3.4.0，未安装新依赖。
- 论文队列父进程已恢复；task 8 的执行、独立验证和导出均完成，已正常领取并运行 task 13。
- Jetson 生产入口尚未整体同步本次新增配置字段；要在那里启用新配置，应先同步匹配的
  runtime config / GPU 启动与服务文件，而不是只同步模型文件。
- 新版 Survey 状态快照尚未实现；当前使用明确的 RGB 重放初始化。
- 此次没有新的实时相机源时间戳测量。旧实测 1.01 Hz 包含在线采集、调度和其他
  开销，新 334–336 ms 不能直接写成机器人已经达到 3 Hz。
- 下一步若选择新部署：同步两机配置解析和启动代码，以新运行身份启用选定模式，
  再做相同静止流程的实时 RGB→响应测量。改变写入策略的导航效果需另外验证，
  不在本轮“只测延迟”的范围内。
