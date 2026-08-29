# MemNav / NavDP Unitree Go2 真机实验完整手册

Snapshot: **2026-08-29**
适用系统：**Unitree Go2 + Jetson Orin NX + RealSense D435i + RTX 4090**
当前协议：**Full-Mono protocol-v3 + direct-bearing-v2**
评测计划：**4 scenes × 5 paired native/CEC blocks = 20 pairs / 40 runs**

本文是本仓库真机实验、现场交接和结果整理的统一入口。它把系统架构、双机路径、
Survey/Formal 两阶段协议、目标图来源、在线决策、Go2 控制、安全门、RViz、双视角采集、
SR/SPL、故障处理和当前缺口集中在一个文档中。其他文档继续保留为实现细节、历史发布
记录和独立审计依据；现场执行应优先遵循本文和 `CURRENT_STATUS.md` 的最新边界。

---

## 1. 先读结论

### 1.1 当前系统是什么

当前系统是一个双机、单目导航、局部深度安全、长期视觉记忆增强的 ImageGoal/Revisit
导航系统：

- RTX 4090 负责一个 causal RGB stream、LingBot 单目深度、CEC 长期记忆检索、几何证明
  和 frozen NavDP 轨迹推理；
- Jetson Orin NX 负责 D435i ROS 传输、aligned metric depth 局部碰撞门、轨迹跟踪、
  `/navdp/cmd_vel`、Go2 watchdog 和全部运动权限；
- 4090 不安装 Unitree SDK、不发布 ROS 速度，也没有到电机的直接路径；
- D435i metric depth 不进入 MemNav、CEC 或 NavDP，只留在 Jetson 安全层和可选离线证据；
- 不使用 TinyNav VIO、TinyNav mapping 或 TinyNav planner；TinyNav 环境只复用已验证的
  CycloneDDS 和 Unitree SDK Python 依赖；
- 不构建 metric global map，也没有传统全局路径规划器；CEC 提供长期视觉内容寻址和
  scale-free 方向，NavDP 反复生成机器人局部 24 点轨迹。

### 1.2 当前已经建立什么

- 真实 D435i RGB 能通过 Jetson、SSH tunnel、CEC Hub 到达 4090 frozen NavDP；
- LingBot 能从同一个 causal RGB state 提供 NavDP 单目深度和 CEC 几何证明；
- protocol-v3 强制 `memory_recording -> prepare_revisit -> revisit_query`；
- Survey 能保存 exact-byte RGB、与 memory 互斥的候选目标图和 SHA-256 manifest；
- Formal 能重启双机、验证并重放 sealed dataset、安装目标并初始化 NavDP 短期上下文；
- CEC 能产生有用的长期 revisit bearing；
- current-to-goal direct proof 能在近目标区域提供 scale-free bearing；
- Jetson tracker 已以 `0.30 m/s` 正向驱动 Go2，修复了早期低速门控导致的左右 hunting；
- 独立 RGB-only commissioning arrival gate 已完成一次近 D 点有电锁存、disable、estop
  和零速度闭环，证明 detector-to-adapter 停车传输可工作；
- 每轮可以绑定 ROS bag、CEC/status JSONL、RViz dashboard、第三人称视频和 Git revision。

### 1.3 当前仍没有建立什么

- 没有经过跨场景独立物理标定、可用于正式实验的 autonomous ImageGoal arrival/STOP；
- 单目 PnP translation 没有 metric control authority，也没有 STOP authority；
- 自动目标候选选择尚未被证明与 externally frozen benchmark goal 完全等价；
- candidate-id 到独立物理 pose 的自动收据仍不完整；
- 正式 `P_i` 物理路径测量系统仍未冻结；
- 还没有可发表的 Full-Mono real-world SR/SPL；
- 当前 4×5 paired 页面是预注册空模板，不是已有结果。

因此当前可以进行：

1. disabled 静态验收；
2. 系绳、现场监管的 engineering run；
3. RGB arrival/operator termination 的正式证据采集演练；
4. arrival calibration 数据采集；

但在本文第 20 节的 P0 门完成前，不得把实验描述为 autonomous ImageGoal success，
也不得填写正式 SR/SPL。

---

## 2. 一次实验到底指什么

一个正式 campaign 由场景资产和独立 rollout 两层组成：

```text
Campaign: cec-four-scene-five-paired-block-v2
├── Scene 01
│   ├── 一个 sealed survey dataset
│   ├── 一个冻结的 exact goal JPEG / SHA-256
│   ├── 一个冻结的正式起点、yaw、L、预算和终止合同
│   └── Pair 01 ... Pair 05
│       ├── mono-native rollout
│       └── mono-CEC rollout
├── Scene 02
│   └── 同样 5 个 paired blocks
├── Scene 03
│   └── 同样 5 个 paired blocks
└── Scene 04
    └── 同样 5 个 paired blocks
```

Survey 不是正式 rollout，不计入 SR/SPL。Survey 是每个场景一次性的 causal memory 和
目标候选数据采集。每个 pair 的 native 与 CEC 必须各自独立重启并物理复位：

- 使用同一 sealed dataset；
- 使用同一 exact goal JPEG；
- 从同一预声明起点和 yaw 开始；
- 使用同一控制参数、时间/路径预算和终止规则；
- 每次重启双机模型状态；
- 每次使用唯一 run ID；
- 不允许根据前几次结果改场景或阈值。

每个 pair 共享 scene、Survey、goal、start/yaw、checkpoint 和预算；正式 query 不写回
Survey memory。20 个 pair 的 arm order 在任何 outcome 前冻结为 native-first 10 个、
CEC-first 10 个。原先“先跑完整 CEC campaign、之后再补 baseline”的 v1 设计不再控制
正式实验，因为它不能消除光照、电量和地面状态的时间混杂。

建议 run ID 固定为：

```text
scene01_formal_01 ... scene01_formal_05
scene02_formal_01 ... scene02_formal_05
scene03_formal_01 ... scene03_formal_05
scene04_formal_01 ... scene04_formal_05
```

---

## 3. 硬件、主机和路径

### 3.1 Robot / Jetson

| 项目 | 当前配置 |
| --- | --- |
| Robot | Unitree Go2 |
| Robot compute | Jetson Orin NX 16 GB |
| Camera | Intel RealSense D435i |
| Jetson workspace | `/home/nvidia/twork/NavDP` |
| ROS | ROS 2 Humble |
| Go2 network interface | 默认 `eth0`，必须有 `192.168.123.x` 地址 |
| Unitree SDK source | `/home/nvidia/unitree_ws/src/unitree_sdk2_python` |
| CycloneDDS | `/home/nvidia/twork/cyclonedds/install` |
| Unitree Python | `/home/nvidia/twork/tinynav/.venv/bin/python` |

`/home/nvidia/twork/tinynav/.venv` 只是已验证的 Unitree/CycloneDDS Python 环境，不表示
实验使用 TinyNav 的 VIO、地图或规划。

### 3.2 RTX 4090 workstation

| 项目 | 当前配置 |
| --- | --- |
| SSH alias | 默认 `work-pc` |
| Standalone repository | `/home/asus/Research/Memnav_Realworld` |
| External research workspace | `/home/asus/Research/Nav-graph-blind` |
| MemNav/LingBot service | `127.0.0.1:18888` |
| Frozen NavDP service | `127.0.0.1:8888` |
| Unified CEC Hub | `127.0.0.1:18889` |
| tmux session | 默认 `cec-realworld` |

如现场路径变化，必须通过 `CEC_HUB_SSH_HOST` 和 `CEC_GPU_REPO` 显式覆盖；不要修改脚本
默认值来掩盖机器差异。

### 3.3 Camera height

2026-08-21 记录的 D435i optical-center height 为 `0.42 m`，但相机支架、机身姿态或安装
位置发生变化后必须重新测量。4090 启动要求显式提供：

```bash
export CEC_CAMERA_HEIGHT_M=0.42
```

没有默认值。健康检查只接受有限且位于 `[0.1, 2.0] m` 的显式测量值。

---

## 4. 完整双机架构

```text
                  RTX 4090 workstation
┌──────────────────────────────────────────────────────────────┐
│ Unified CEC Hub :18889                                       │
│   ├── protocol-v3 phase authority                            │
│   ├── exact-once RGB transaction                             │
│   ├── goal selection / installed-goal SHA authority          │
│   ├── route selection and persistent receipts                │
│   │                                                          │
│   ├── MemNav / LingBot :18888                                │
│   │   ├── causal RGB history                                 │
│   │   ├── first-40 camera-height scale bootstrap             │
│   │   ├── dense monocular depth readout                      │
│   │   ├── DINO history retrieval                             │
│   │   ├── SuperPoint/LightGlue geometry                      │
│   │   └── PnP certificate / scale-free bearing               │
│   │                                                          │
│   └── Frozen NavDP :8888                                     │
│       ├── 8-frame short observation FIFO                     │
│       ├── ImageGoal or ImageGoal+PointGoal conditioning      │
│       ├── 16 candidate trajectories                          │
│       └── selected 24-point robot-local trajectory           │
└────────────────────────────┬─────────────────────────────────┘
                             │ loopback-only service
                             │ SSH local forward
                             ▼
                 Jetson Orin NX / Unitree Go2
┌──────────────────────────────────────────────────────────────┐
│ D435i                                                        │
│   ├── RGB 848x480@30 ───────────────┐                        │
│   └── aligned depth 848x480@30 ─────┤                        │
│                                     ▼                        │
│ navdp_go2_adapter                                            │
│   ├── approximate RGB-D sync                                 │
│   ├── RGB -> HTTP client -> CEC Hub                          │
│   ├── aligned depth -> local collision guard only            │
│   ├── 1.5 Hz planning worker                                 │
│   ├── trajectory -> lookahead tracker                        │
│   ├── enable / estop / freshness / proof validation          │
│   └── 20 Hz /navdp/cmd_vel                                   │
│                             │                                │
│                             ▼                                │
│ go2_cmd_bridge                                               │
│   ├── speed limits and hardware command floors               │
│   ├── 0.35 s watchdog                                        │
│   ├── hand-controller priority                               │
│   └── Unitree SportClient.Move() -> motors                   │
│                                                              │
│ Observation-only sidecars                                    │
│   ├── RViz dashboard                                         │
│   ├── RGB arrival / audit                                    │
│   ├── ROS bag + JSONL collector                              │
│   └── external third-person camera                           │
└──────────────────────────────────────────────────────────────┘
```

### 4.1 权限边界

| 能力 | 4090 | Jetson adapter | Go2 bridge | Evaluator/collector |
| --- | ---: | ---: | ---: | ---: |
| 读取当前 RGB | 是 | 是 | 否 | 可选 |
| 使用 D435i metric depth 规划 | 否 | 否 | 否 | 否 |
| 使用 D435i depth 做碰撞门 | 否 | 是 | 否 | 否 |
| 生成 NavDP trajectory | 是 | 否 | 否 | 否 |
| 转换 trajectory 为 Twist | 否 | 是 | 否 | 否 |
| 发布 `/navdp/cmd_vel` | 否 | 是 | 否 | 否 |
| 调用 Unitree `Move()` | 否 | 否 | 是 | 否 |
| 发布 RGB arrival estop | 否 | 否 | 否 | 可选，但不是 policy STOP |
| 最终现场接管 | 否 | 否 | 释放权限 | Unitree 遥控器操作者 |

---

## 5. 传感器和状态合同

### 5.1 RealSense 配置

相机启动脚本只启用：

- color `848x480x30`；
- depth `848x480x30`；
- `enable_sync=true`；
- `align_depth.enable=true`；
- `publish_tf=true`；

并明确禁用：

- infra1 / infra2；
- gyro / accel；
- pointcloud；
- VIO。

最低固件门默认是 `5.17`。设备枚举、固件检查或 CameraInfo 超时会使启动事务回滚。

### 5.2 RGB-D 同步

Adapter 只接受 RGB 和 aligned depth 时间戳差不超过 `0.10 s` 的完整帧对：

- sync queue size：`15`；
- RGB-D freshness timeout：`0.60 s`；
- depth scale：`0.001 m`；
- CameraInfo 必须是有限有效内参；
- RGB/depth shape 必须一致。

超过 skew 的帧对被丢弃；如果 `0.60 s` 内没有新合法帧对，控制层输出零速度。

### 5.3 Navigation policy sensor contract

4090 健康收据必须证明：

```text
navigation_sensor_contract=causal_monocular_rgb_v1
navdp_depth_source=monocular_sidecar
metric_depth_sensor_consumed_by_policy=false
protocol_version=3
terminal_handoff_schema=cec_direct_bearing_handoff_v2_20260824
```

Jetson HTTP client仍携带旧 depth multipart 字段以保持 wire compatibility，但 CEC Hub 丢弃
该字段。NavDP只能通过 MemNav `/monocular_depth_query` 取当前 RGB 对应的 LingBot depth。

### 5.4 SportModeState

`rt/sportmodestate` 不进入 NavDP，也不改变 CEC bearing。当前只用于：

- 只读记录 Go2 body pose/velocity；
- 辅助最小距离、路径长度和 yaw 分析；
- 现场故障回放。

它不是认证 ground truth：腿式接触可能打滑，内部估计会漂移，重启可能改变局部坐标系。

---

## 6. 协议状态机

```text
POST /navigator_reset
        │
        ▼
phase = memory_recording
        │
        ├── /memory_step            合法：只 append causal RGB
        ├── /goal_candidate         合法：候选不写入 memory
        └── /imagegoal_step         非法：HTTP 400，无上游状态变化
        │
        ▼
POST /prepare_revisit 或 /prepare_revisit_goal
        │  必须在机器人静止、正式 query 起点执行
        │  评分/安装目标、初始化 NavDP FIFO、验证 queue receipt
        ▼
phase = revisit_query
        │
        ├── /imagegoal_step         合法
        ├── /memory_step            非法
        └── repeated prepare        幂等读取已提交结果
```

该状态机避免从 frame 0 就发 ImageGoal。MemNav 在某一目标的第一次 query 时冻结
`goal_start_frame` 和 candidate ceiling；若从 frame 0 发目标，随后整段 survey history 都会被
排除在合法 Revisit candidate 之外。Protocol-v3 把这个错误变成结构上不可能发生。

Hub是 phase authority；Jetson只镜像 phase 用于本地 gating 和 RViz状态。

---

## 7. 每个场景的预注册

Formal 01 之前应填写并冻结：

| 字段 | 要求 |
| --- | --- |
| `scene_id` | `scene01` ... `scene04` |
| 场景描述 | 地点、光照、地面、障碍、目标对象 |
| `dataset_id` | 唯一、不可覆盖 |
| dataset manifest SHA | seal 后填写 |
| exact goal JPEG | 五次必须完全相同 |
| goal SHA-256 | Formal 启动硬检查的目标值 |
| 正式起点 S | 地面标记或独立测量 |
| 起始 yaw | 允许误差必须预先声明 |
| goal pose/区域 | 用于独立 success adjudication |
| shortest feasible path `L` | Formal 01 前测量 |
| time budget | 超时为 failure |
| path budget | 超限为 failure |
| collision/abort rule | 不允许事后定义 |
| arrival contract | 阈值、hold、termination authority |
| trial order | 提前冻结 |
| control profile | 必须是 `formal` |
| max speed | `0.30 m/s` |
| capture profile | `audit`，除非预注册 full replay |

目标、起点、预算、顺序或阈值在 Formal 01 后发生变化时，应创建新的 scene/version，而不是
继续填原有五次表格。

---

## 8. Pass 1：Survey 与 sealed dataset

### 8.1 启动 Survey

在 Jetson 执行：

```bash
cd /home/nvidia/twork/NavDP
export CEC_HUB_SSH_HOST=work-pc

bash deployment/go2/offboard/revisit_experiment.sh \
  survey-start scene01_dataset --with-rviz
```

该命令会：

1. 启动或复用 4090 policy stack；
2. 建立 SSH tunnel；
3. 启动 D435i；
4. 等待真实 CameraInfo；
5. 启动 two-phase ImageGoal adapter；
6. 在第一次 `navigator_reset` 内原子创建 dataset；
7. 保持 `enabled=false + estop=true`；
8. 不启动 Go2 bridge；
9. 关闭自动候选门；
10. 将 survey metadata 写入 dataset。

Survey 机器人只允许由原装 Unitree 遥控器移动。

### 8.2 Survey 路线设计

推荐路线是较长的“去程—回程”：

```text
survey start
   │
   │ outbound：连续建立 history，不允许自动候选
   ▼
physical turnaround
   │
   │ 显式声明 return boundary
   ▼
return leg：再次经过早期区域，产生非重复共视
   │
   ▼
survey end
```

建议：

- 3–8 分钟；
- 300–900 个 memory frames；
- 经过两个以上转角或通道；
- 回程有自然横向偏移；
- 与去程保持约 10–30° 的朝向差；
- 允许自然遮挡，但避免目标被长期完全遮住；
- 不要原地拍摄大量近重复帧；
- 不要只走单向直线。

`160` 帧只是 seal 拒绝明显过短数据集的底线，不是推荐规模。

### 8.3 查看 Survey 状态

```bash
bash deployment/go2/offboard/revisit_experiment.sh survey-status
```

至少检查：

- `recording=true`；
- dataset ID 正确；
- phase 为 `memory_recording`；
- `frames_recorded` 连续增长；
- adapter 仍 disabled；
- estop 仍为 true；
- `/navdp/cmd_vel` 为零；
- 无 `memory_degraded` 或 `native_state_uncertain`。

### 8.4 声明回程并打开候选门

在物理折返点、准备回程时执行：

```bash
bash deployment/go2/offboard/revisit_experiment.sh \
  survey-return scene01_dataset
```

该动作只记录 causal return boundary，不改变电机权限。之后 adapter 默认：

- 每 24 个 recorded memory frames 尝试一个候选；
- 最多接受 6 个候选；
- 候选自身绝不 append 到 memory；
- 接受候选后跳过后续 4 帧；
- 候选只允许读取 capture boundary 至少 16 帧以前的 history。

### 8.5 候选支持门

候选图依次进行：

1. stride-8 history DINO support scan；
2. 最相关 anchor 的特征匹配；
3. fundamental/epipolar RANSAC；
4. 几何 inlier 计数；
5. near-duplicate 检查；
6. causal eligible-anchor ceiling 检查。

默认接受条件：

- geometry inliers `>=16`；
- best DINO cosine `<=0.90`，避免近复制；
- `eligible_anchor_ceiling` 必须位于冻结 capture window 内。

拒绝类型：

| 状态 | 含义 |
| --- | --- |
| `reject_unsupported` | 几何支持不足 |
| `reject_near_duplicate` | 过于接近历史 JPEG |
| `provisional_weak_covis` | 有可用共视且非完全重复，可注册 |

### 8.6 Seal

走完去程和回程后：

```bash
bash deployment/go2/offboard/revisit_experiment.sh \
  survey-seal scene01_dataset
```

Seal 前脚本再次执行 motion lock。合法 dataset 形状：

```text
episodic_datasets/scene01_dataset/
├── memory/
│   └── 000000_<sha>.jpg ...
├── goals/
│   ├── candidate_000_<sha>.jpg
│   ├── candidate_000_<sha>_depth.png
│   └── ...
├── manifest.json
├── MANIFEST.sha256
└── SEALED
```

Seal 校验：

- memory frame index 连续；
- exact bytes、size、SHA 与 manifest 一致；
- 至少一个合法 candidate；
- candidate 记录 `appended_to_memory=false`；
- goal SHA 与任一 memory SHA 交集为零；
- goal aligned depth 明确标记 `evaluation_depth_policy_authority=false`；
- evaluation depth 采用 `1e-3 m` PNG scale；
- dataset ID 不可覆盖或续写。

---

## 9. Pass 2：Formal rollout 准备

### 9.1 每次物理复位

每个 paired block 的每个 arm 前：

1. 确认上一轮 adapter disabled、estop asserted；
2. 用遥控器把 Go2 移到预声明正式起点 S；
3. 恢复预声明 yaw 和机身站姿；
4. 保持机器人静止；
5. 确认 D435i支架和 optical-center height 没变；
6. 清除测试区域人员和动态障碍；
7. 放置第三人称相机；
8. 确认遥控器操作者到位；
9. 确认 run ID 尚未存在；
10. 不允许直接复用上一轮仍存活的 RTX/Jetson状态。

Formal 起点不必等于 Survey 终点。Survey 是长期记忆数据采集；Formal 是独立 episode。

### 9.2 Formal-start

```bash
bash deployment/go2/offboard/revisit_experiment.sh \
  formal-start scene01_dataset --with-rviz
```

它会：

1. 锁止运动；
2. 停止上一轮 Jetson 和 RTX process tree；
3. 全新启动 RTX MemNav、NavDP、Hub；
4. 建立 SSH tunnel；
5. 启动 D435i并等待 CameraInfo；
6. 启动 adapter、Go2 watchdog bridge 和 RViz；
7. 保持 `disabled + estop`；
8. 逐文件验证 sealed dataset；
9. 将 survey RGB 重放给 LingBot/CEC 长期 memory；
10. 不把 survey 尾部伪装成 NavDP 当前短期上下文；
11. 用 formal 起点的当前 RGB 初始化 NavDP FIFO；
12. 评分并安装目标候选；
13. 校验 active goal SHA；
14. 将 selected goal JPEG 和 optional offline depth 保存到本轮 run root；
15. 输出 formal-ready health 和 prepare receipt。

长 survey 的逐帧重放可能需要数分钟。不要因为终端暂时没有新输出而中断；先查看 RTX
和 Jetson日志，确认不是失败。

### 9.3 Formal-ready 必须检查

```bash
bash deployment/go2/offboard/revisit_experiment.sh formal-status

ros2 topic echo --once /navdp/status
ros2 topic echo --once /navdp/cec_receipt
```

必须满足：

```text
phase = revisit_query
enabled = false
estop = true
server_initialized = true
last_error = ""
active_goal_sha256 != null
loaded_dataset_id = expected DATASET_ID
loaded_dataset_manifest_sha256 != null
navdp_warmup_mode = independent_formal_query_start
navdp_queue_lengths = [1]
navdp_memory_size = 8
terminal_handoff_schema = cec_direct_bearing_handoff_v2_20260824
metric_depth_sensor_consumed_by_policy = false
```

`inference_busy=true` 时不得 enable。首个未缓存 CEC anchor 可能接近 20 秒。

### 9.4 目标图冻结注意事项

系统支持两条目标来源，必须在scene预注册时选择其一，五次run不能混用。

#### 路径A：externally frozen exact goal（正式benchmark推荐）

在导航停止、机器人位于目标位置且静止时，单独采集目标参考：

```bash
bash deployment/go2/scripts/run_realsense.sh
bash deployment/go2/scripts/capture_image_goal.sh
```

冻结：

- exact goal RGB；
- goal RGB SHA-256；
- 可选的 aligned goal depth及SHA（仅作离线证据）；
- 目标采集元数据；
- 目标采集时间和scene ID。

Formal-start前显式指定固定目标：

```bash
export NAVDP_REVISIT_IMAGE_GOAL_PATH=/absolute/path/to/frozen_goal.png

bash deployment/go2/offboard/revisit_experiment.sh \
  formal-start scene01_dataset --with-rviz
```

Adapter会调用Hub的`prepare_revisit_goal`，目标来源收据应为
`operator_frozen_external`。Hub冻结这些exact bytes；后续Jetson上传的goal只作为兼容字段，
不能替换committed target。这条路径最符合当前正式benchmark claim boundary。

目标aligned depth只供独立evaluator。若external goal route没有通过Hub返回depth，使用预先冻结
并由scene manifest绑定SHA的本地goal depth；它仍然没有policy authority。

#### 路径B：sealed Survey自动候选（lifelong/engineering路径）

未设置`NAVDP_REVISIT_IMAGE_GOAL_PATH`时，当前`formal-start`会对sealed candidates重新评分，
按以下确定性顺序选择：

1. geometry inliers 最大；
2. inlier ratio 最大；
3. DINO support 最大；
4. candidate ID 最早。

该路径适合测试“在线候选采集、评分并安装为revisit goal”的完整lifelong能力。但注册的正式
campaign仍要求每个场景使用一个exact goal JPEG。若要将自动候选纳入正式campaign，必须在
第一个 paired block 前先固定选中candidate及SHA，之后40次rollout都比较
`active_goal_sha256`；只要
SHA不同，就不得开始该rollout。

当前 launcher 尚未接受强制 `EXPECTED_GOAL_SHA256` 参数。这是进入正式 4×5 paired
campaign 前应完成的
软件门。门完成前，自动选择路径只能视为 engineering/lifelong demo 或人工逐次核对路径。

---

## 10. 单次 Formal 的证据会话

### 10.1 Preflight

Formal stack 和 RViz 已启动、运动仍锁止时：

```bash
bash deployment/go2/offboard/experiment_capture.sh preflight
```

Preflight 检查：

- `ros2 bag record`；
- `tmux`；
- X11 display；
- RViz进程；
- GStreamer H.264 components；
- 必需 ROS topics。

它不发布速度、不 clear estop、不 enable adapter。

### 10.2 启动 run

```bash
bash deployment/go2/offboard/experiment_capture.sh start \
  scene01_formal_01 \
  --dataset scene01_dataset \
  --trial-kind revisit \
  --profile audit
```

推荐 `audit` profile，记录：

- `/navdp/status`；
- `/navdp/cec_receipt`；
- `/navdp/trajectory`；
- `/navdp/cmd_vel`；
- `/navdp/estop`；
- `/navdp/enabled`；
- `/navdp/image_goal`；
- `/navdp/rgb_arrival_status`；
- `/navdp/rgb_arrival_debug`；
- `/navdp/debug/markers`；
- `/navdp/experiment_event`；
- `/rt/sportmodestate`；
- `/camera/camera/color/camera_info`。

`audit` 不复制 raw RGB-D，因为 RTX sealed episodic dataset 是 causal RGB authority。只有需要
离线传感器重放且已检查磁盘带宽/容量时，才使用 `--profile full`；full profile 每分钟可能
产生数 GB 数据。

### 10.3 第三人称同步

Collector 输出 `START` 后：

1. 开始独立第三人称相机；
2. 做一次画面清晰可见的同步拍手；
3. 确认 RViz dashboard 正在录制；
4. 再确认 RGB arrival 模块；
5. 最后才执行运动授权。

第三人称视频用于证明真实运动、足部接触、碰撞、操作员干预和物理终点；RViz用于解释
策略内部状态。两者不能互相替代。

---

## 11. 每一帧的在线决策链

### 11.1 Jetson snapshot

Adapter按 `1.5 Hz` 规划触发，从最新合法 RGB-D 对复制：

- current RGB；
- aligned depth；
- CameraInfo intrinsic；
- installed ImageGoal；
- active goal SHA；
- reset/phase状态。

Adapter即使 disabled 也可以规划，方便在运动前验证轨迹；只有 control loop 被 enable/estop
和安全门约束。

### 11.2 Exactly-once causal append

当前 RGB 以 JPEG quality 95 编码，通过 Jetson loopback Hub URL 和 SSH tunnel 到 4090。
Hub调用 `/retrieval_probe_step`，完成：

1. 当前 RGB append 恰好一次；
2. 更新共享 LingBot causal state；
3. materialize 当前 mono depth；
4. 返回 RGB SHA、frame index、transaction token；
5. 返回历史 retrieval candidates。

Hub将 token 和 frame index传给 frozen NavDP。NavDP再调用 `/monocular_depth_query`，只有
以下全部一致才接受轨迹：

- NavDP receipt 的 `depth_source=monocular_sidecar`；
- `metric_depth_sensor_consumed=false`；
- receipt RGB SHA 等于当前 JPEG SHA；
- transaction token 一致；
- mono-depth frame index 一致。

如果 append 失败，不能伪装为 native fallback，因为同一 LingBot state 同时拥有当前 depth
和历史 proof；必须 latch `reset_required`。

### 11.3 长程 CEC

```text
current RGB + goal JPEG
          │
          ▼
history retrieval candidates
          │
          ▼
SuperPoint / LightGlue matching
          │
          ▼
LingBot depth + PnP + certificate
          │
          ├── accepted -> scale-free revisit bearing
          └── rejected -> abstain
```

CEC不输出可信全局位置，也不输出可用于停止的可靠 metric distance。Accepted bearing 只保留
单位方向，再投影到固定 `2.5 m` local PointGoal：

```text
unit_bearing = raw_vector / ||raw_vector||
controller_pointgoal = 2.5 * unit_bearing
```

`2.5 m` 是冻结的 controller residual，不代表机器人真的距离目标 2.5 m。

### 11.4 Direct current-to-goal proof

每帧还独立调用 current RGB 到 goal JPEG 的 `/local_pose_query`：

- certificate accepted 且 bearing 位于 `±60°`：生成同样的 `2.5 m` fixed-radius PointGoal；
- certificate accepted 但目标位于 `±60°` 外：不进入 NavDP point token，Jetson只执行
  bounded zero-translation atomic turn；
- direct proof 丢失：返回前一条 long-range CEC 或 native route；
- PnP predicted metric distance 只记录诊断，不参与局部平移，也不授权 STOP。

### 11.5 路由优先级

| 优先级 | 条件 | Controller |
| ---: | --- | --- |
| 1 | direct proof，bearing 在 `±60°` | ImageGoal + direct fixed-bearing mixed NavDP |
| 2 | direct proof，bearing 在支持外 | native NavDP更新 FIFO；Jetson atomic turn 覆盖 |
| 3 | direct proof 无效，CEC certificate accepted | ImageGoal + long-range fixed-bearing mixed NavDP |
| 4 | direct/CEC 都 abstain | exact mono-native ImageGoal NavDP |
| 5 | certificate endpoint错误但 RGB append成功 | native NavDP + error receipt |
| 6 | causal append / mono-depth receipt不确定 | no new trajectory，reset required |

### 11.6 Frozen NavDP

NavDP使用：

- 最近最多 8 个 RGB observations；
- 每个当前 RGB 对应的 LingBot monocular depth；
- ImageGoal JPEG；
- 可选的 fixed-radius PointGoal token。

输出：

- 16 条 candidate trajectories；
- 每条 24 点；
- 一个 selected trajectory；
- candidate values；
- mono-depth transaction receipt。

轨迹位于当前 robot-local plane：

```text
x forward
y left
```

系统不把这些路径累计成 global map。下一次 query 根据新画面重新生成局部轨迹。

---

## 12. Jetson轨迹控制

### 12.1 Lookahead tracker

Selected trajectory进入本地 tracker：

1. 必要时在 path 开头补 `(0,0)`；
2. 去除非有限点和重复点；
3. 累加路径长度；
4. 取 `0.60 m` lookahead target；
5. 计算 body-frame heading；
6. 转成 forward velocity 和 yaw rate。

### 12.2 Formal profile

| 参数 | 值 |
| --- | ---: |
| lookahead | `0.60 m` |
| max linear | `0.30 m/s` |
| max angular | `0.55 rad/s` |
| heading deadband | `8°` |
| rotate-in-place threshold | `0.70 rad` |
| rotate gain | `1.50` |
| slow path length | `1.00 m` |
| allow reverse | `false` |
| max linear accel | `0.45 m/s²` |
| max angular accel | `1.00 rad/s²` |

控制规律：

- heading `>=0.70 rad`：线速度为零，原地转向；
- heading `<8°`：`wz=0`，避免小误差触发Go2角速度门控；
- 其他情况：线速度约为 `0.30 * path_scale * cos(heading)`；
- path短于 `1.0 m` 时按路径长度减速；
- 默认不倒车，避免前视相机看不到后方风险。

### 12.3 Acceleration limit

Control loop以 `20 Hz` 对目标命令做 slew limit。它先应用 adapter 的 acceleration limit，再
交给 Go2 bridge。这样避免每次新轨迹使速度从零瞬间跳到上限。

---

## 13. Jetson aligned-depth safety

Aligned depth只进入本地安全门。中央 ROI：

```text
horizontal: 35% -- 65%
vertical:   30% -- 70%
```

计算有效像素的第 10 百分位 clearance：

| 状态 | 行为 |
| --- | --- |
| 有效深度比例 `<3%` | fail-closed stop |
| clearance `<=0.45 m` | obstacle stop |
| clearance `0.45–0.80 m` | 线速度线性缩放 |
| clearance `>=0.80 m` | 不因depth减速 |
| depth > `5.0 m` | 不参与前方clearance |

该 ROI 同时保护前进、倒车和旋转命令，但它不是认证安全系统。透明物体、镜面、楼梯、
悬空/低矮障碍、相机视野外物体和后方风险必须由现场操作者控制。

---

## 14. Go2 bridge和速度门控

Adapter发布 `/navdp/cmd_vel`，bridge通过 Unitree SDK调用 `SportClient.Move()`。

| 参数 | 默认值 |
| --- | ---: |
| cmd topic | `/navdp/cmd_vel` |
| loop rate | `20 Hz` |
| max `vx` | `0.30 m/s` |
| max `vy` | `0.0 m/s` |
| max `wz` | `0.60 rad/s` |
| min translation command | `0.10 m/s` |
| min rotation command | `0.20 rad/s` |
| watchdog | `0.35 s` |
| remote priority | `true` |
| remote deadband | `0.12` |
| remote hold | `0.8 s` |

关键顺序：先在 adapter 应用 8° heading deadband，再由 bridge应用 `0.10/0.20` command
floors。若反过来，小的正负 heading error会反复被放大到最小角速度，产生左右 hunting。

Bridge 在命令超时、零命令或遥控器接管时执行 `Move(0,0,0)`，并按配置调用
`StopMove()` 释放运动。

---

## 15. 安全与运动授权

### 15.1 启动状态

```text
enable_on_start=false
estop_on_start=true
plan_while_disabled=true
```

`--with-go2` 只启动 watchdog bridge，不是 arm。Formal-start 故意没有无人值守 `arm`
子命令。

### 15.2 Enable前现场门

必须确认：

- scene/run ID 正确；
- formal-ready receipts 全部通过；
- expected dataset SHA 和 goal SHA 一致；
- current RGB、depth、goal、paths 和状态在 RViz可见；
- inference不忙；
- `/navdp/cmd_vel` 在 disabled 时为零；
- arrival/termination procedure 已运行；
- ROS bag、JSONL和dashboard已开始录制；
- 第三人称相机已开始并完成同步拍手；
- 测试区清空；
- 遥控器操作者就位；
- 首次/新场景运动使用系绳；
- 电量、网线、相机和支架可靠。

### 15.3 两步授权

```bash
source /opt/ros/humble/setup.bash

ros2 topic pub --once /navdp/estop \
  std_msgs/msg/Bool "{data: false}"

ros2 service call /navdp_go2_adapter/set_enabled \
  std_srvs/srv/SetBool "{data: true}"
```

先 clear estop，再 enable。Enable后仍可能因为其他安全门而保持零速度，这是正确行为。

### 15.4 Fail-closed矩阵

| 失败 | Jetson/Hub行为 |
| --- | --- |
| adapter disabled | 发布零速度 |
| estop asserted | 发布零速度 |
| RGB-D age `>0.60 s` | `rgbd_stale`，零速度 |
| RGB/depth skew过大 | 不接受该帧对，最终freshness stop |
| trajectory age `>2.50 s` | `trajectory_stale`，零速度 |
| inference exception | `inference_error`，零速度 |
| local depth不可用 | `depth_unavailable_stop` |
| clearance `<=0.45 m` | `obstacle_stop` |
| SSH tunnel断开 | 请求失败，旧轨迹过期，零速度 |
| causal append失败 | memory degraded，必须reset |
| mono receipt不一致 | native state uncertain，必须reset |
| Hub并发请求 | HTTP 409 |
| Go2 cmd age `>0.35 s` | bridge零速并StopMove |
| 手柄活动 | bridge释放自主authority |
| terminal schema不匹配 | startup/reset拒绝 |

### 15.5 紧急停止

现场优先使用 Unitree 遥控器。软件侧：

```bash
ros2 topic pub --once /navdp/estop \
  std_msgs/msg/Bool "{data: true}"

ros2 service call /navdp_go2_adapter/set_enabled \
  std_srvs/srv/SetBool "{data: false}"
```

必要时再停止整个栈：

```bash
bash deployment/go2/offboard/revisit_experiment.sh stop
```

不要先杀 ROS/SSH 再尝试发 estop。优先让仍存活的控制链明确归零和disable。

---

## 16. RViz和实时调试

### 16.1 启动与连接

Formal-start使用 `--with-rviz` 后：

```bash
tmux attach -t navdp-go2-offboard
```

也可以在其他终端观察：

```bash
ros2 topic echo /navdp/status
ros2 topic echo /navdp/cec_receipt
ros2 topic hz /camera/camera/color/image_raw
ros2 topic hz /camera/camera/aligned_depth_to_color/image_raw
```

### 16.2 Dashboard内容

| RViz display | Topic/含义 |
| --- | --- |
| RGB Camera | `/camera/camera/color/image_raw` |
| Aligned Depth | `/camera/camera/aligned_depth_to_color/image_raw`，仅本地安全 |
| Image Goal | `/navdp/image_goal`，当前安装目标 |
| RGB Arrival Match | `/navdp/rgb_arrival_debug` |
| Candidate paths | `/navdp/debug/markers` |
| Selected trajectory | `/navdp/trajectory` |
| Goal/status markers | phase、CEC、clearance、vx/wz、error |
| Local grid | `navdp_local` robot-local frame |

### 16.3 `/navdp/status`重点字段

- `enabled` / `estop`；
- `phase`；
- `server_initialized`；
- `inference_busy`；
- `rgbd_age_s`；
- `rgb_depth_skew_s`；
- `plan_age_s`；
- `last_inference_s`；
- `candidate_count`；
- `clearance_m`；
- `stop_reason`；
- `last_error`；
- `frames_recorded`；
- `active_goal_sha256`；
- `cec_takeover` / `cec_reason` / `cec_selected_anchor`；
- `terminal_handoff_disposition`；
- `terminal_predicted_bearing_deg`；
- `terminal_stop_authorized`；
- `monocular_depth_receipt`；
- `cmd_vx` / `cmd_wz`。

### 16.4 正常停顿和异常停顿

当前系统可能出现“移动一段、停住、再移动”：

- planning trigger是 `1.5 Hz`，不是30 Hz；
- CEC/LightGlue第一次处理未缓存anchor可能接近20秒；
- 旧trajectory在 `2.50 s` 后过期；
- 过期期间Jetson必须输出零速度；
- 新receipt到达后才恢复控制。

如果 RViz路径合理但长期不动，依次检查：

1. `enabled`；
2. `estop`；
3. `inference_busy`；
4. `last_error`；
5. `plan_age_s`；
6. `stop_reason`；
7. `clearance_m`；
8. bridge是否仍在tmux；
9. 遥控器是否触发priority；
10. `/navdp/cmd_vel` 是否非零且超过hardware floor。

---

## 17. Arrival、STOP和独立 RGB gate

### 17.1 为什么不能用NavDP零轨迹判断到达

零速度/短轨迹可能来自：

- NavDP确实认为无需继续移动；
- 网络/推理超时；
- RGB-D stale；
- depth obstacle stop；
- adapter disabled或estop；
- trajectory decoder退化；
- 当前视图歧义；
- 目标在转向支持外。

因此“机器人停了”不等于“到达成功”。配置中的 `goal_arrival_m=0.60` 只用于 PointGoal，
不为 ImageGoal提供到达判定。

### 17.2 为什么不能用PnP metric distance停止

已有真实轨迹中，direct PnP预测距离从 `0.769 m` 降到 `0.125 m`，而当时独立测量收据的
真实最小距离仍是 `0.993 m`，至少低估 `7.9x`。因此bearing-v2只保留方向：

```text
terminal_predicted_distance_control_authority=false
terminal_metric_scale_control_authority=false
terminal_stop_authorized=false
```

### 17.3 唯一内置到达模块：RGB homography

`rgb_goal_arrival.py` 只读取当前 RGB 和冻结参考图，不读取目标/当前深度、Go2 位姿或
NavDP 轨迹。它计算 SIFT ratio-test、homography RANSAC、匹配覆盖、中心偏移、尺度、
旋转和重投影误差。当前默认值为：

| 门 | 默认值 |
| --- | ---: |
| good matches | `>=45` |
| inliers | `>=30` |
| inlier ratio | `>=0.45` |
| coverage | `>=0.07` |
| center offset | `<=0.22` normalized |
| image scale | `[0.60, 1.45]` |
| rotation | `<=16°` |
| reprojection error | `<=4 px` |
| consecutive matches | `1` |

命中后它锁存 `/navdp/arrival`、发布 `/navdp/estop` 并请求禁用 adapter。该模块已在一次
near-D 有电测试中验证停车传输和一个正样本窗口；这不证明完整路线、跨场景鲁棒性或
false-positive rate。完整收据见 `CURRENT_STATUS.md`。它只能描述为实验性 independent
termination，不是 autonomous policy STOP。

### 17.4 当前Formal成功判定

在arrival calibration完成前：

- RGB arrival gate作为独立termination；
- 现场操作者保持遥控器；
- 成功/失败由预注册的独立规则和事后证据共同adjudicate；
- 不能把CEC、PnP、SportModeState或人工主观“差不多到了”单独作为成功依据；
- engineering run可以记录观察结果，但不进入正式SR/SPL。

---

## 18. 一次run的停止、封存和验证

### 18.1 停止顺序

1. RGB arrival gate完成终止或现场发布estop；
2. adapter disable；
3. 确认 `/navdp/cmd_vel` 为零；
4. 停止 evidence processes；
5. 停止第三人称相机；
6. 不移动/覆盖结果文件，先完成导入和hash；
7. 需要时再停止policy stack。

### 18.2 停止capture

```bash
bash deployment/go2/offboard/experiment_capture.sh stop \
  scene01_formal_01
```

### 18.3 导入第三人称原片

```bash
bash deployment/go2/offboard/experiment_capture.sh attach-third-view \
  scene01_formal_01 /path/from/camera/third_view.mp4
```

导入是byte-preserving copy。第三人称原始master应另行备份；Git里只放审核后的浏览器
derivative，不放raw evidence。

### 18.4 Finalize

```bash
bash deployment/go2/offboard/experiment_capture.sh finalize \
  scene01_formal_01 success \
  --notes "RGB arrival/operator termination; adjudication attached"
```

若是abort/failure，将 outcome改为对应结果并写清原因。不要为了生成 `FINALIZED` 把失败
写成success。

正式bundle必须包含：

- closed rosbag和`metadata.yaml`；
- 非空dashboard MP4；
- byte-preserved third-view视频；
- 非空status JSONL；
- 非空CEC receipt JSONL；
- start/stop experiment events；
- Git revision；
- configuration identity；
- artifact size和SHA-256。

### 18.5 Verify

```bash
bash deployment/go2/offboard/experiment_capture.sh verify \
  scene01_formal_01
```

默认目录：

```text
runtime/go2/experiment_capture/scene01_formal_01/
├── manifest.json
├── MANIFEST.sha256
├── FINALIZED
├── rosbag/
├── logs/
│   ├── status.jsonl
│   ├── cec_receipt.jsonl
│   ├── rgb_arrival_status.jsonl
│   └── experiment_event.jsonl
├── media/
│   ├── dashboard.mp4
│   └── third_view.mp4
└── receipts/
```

`--allow-incomplete` 只适用于aborted engineering run；manifest必须记录
`formal_complete=false`，不能进入正式统计。

---

## 19. SR、SPL和结果登记

### 19.1 定义

对run `i`：

```text
SR = sum(S_i) / N
SPL_i = S_i * L_i / max(L_i, P_i)
SPL = sum(SPL_i) / N
```

- `S_i`：独立adjudicated binary success；
- `L_i`：Formal 01前预声明的场景最短可行路径；
- `P_i`：独立测量的Go2真实路径；
- failure贡献零SPL；
- `N=5`用于单场景SR/SPL，整体campaign `N=20`。

### 19.2 不允许填写结果的情况

任一项缺失时，结果表保持 `—`/`null`：

- run capture manifest未finalize；
- independent success record缺失；
- `L_i`未在Formal前冻结；
- `P_i`没有合法独立测量；
- goal SHA与scene registry不一致；
- start/yaw超出预声明容差；
- 证据不完整；
- evaluator阈值在run后修改；
- safety/operator intervention没有记为failure/abort；
- 运行使用了acceptance profile而不是formal profile。

### 19.3 SportModeState边界

在没有额外校准前，SportModeState只能作为auxiliary path estimate。正式 `P_i` 不能仅凭
其积分自动生成，除非预先验证打滑、坐标重置、漂移和丢包误差满足评测要求。

当前推荐正式源改为完全隔离的Odin1 reference lane：`P_i`积分Odin局部odom增量，`L_i`
来自同一Survey冻结occupancy上的A*，`S_i`结合Odin metric region、D435i视觉确认和静止
hold。它不进入NavDP控制，且仍需完成当前Go2安装/重定位/尺度现场验证后才能用于正式统计。

### 19.4 发布

每个场景页面最终应包含：

- 5个run ID；
- 每个run success、`L_i`、`P_i`、`SPL_i`；
- capture manifest SHA；
- third-view MP4/GIF；
- RViz dashboard MP4/GIF；
- 失败原因；
- 场景aggregate SR和mean SPL。

浏览器预览不是raw evidence的替代品。只有审核后的derivatives进入 `media/demo/`，原始bag、
JSONL和master保留在runtime/外部归档。

---

## 20. 正式4×5 paired campaign前必须完成的P0

### P0-A：Arrival/STOP标定

机器人保持disabled，在至少3–4个地点采集带独立物理标签的目标邻域：

```text
distance: 0 / 0.25 / 0.5 / 1.0 m
yaw:      0 / ±10 / ±20°
```

要求：

1. 物理距离/yaw在查看visual score前记录；
2. 用一个地点选proof-conditioned convergence rule；
3. 用不同地点验证；
4. 统计false positive/false negative；
5. 冻结阈值、连续帧数、hold和failure semantics；
6. 明确是independent RGB arrival termination还是policy STOP；
7. 未通过跨场景验证前，只允许显式 opt-in 的 engineering STOP，不得用于正式结果。

### P0-B：Exact goal SHA启动门

为`formal-start`增加并验证：

```text
EXPECTED_GOAL_SHA256=<scene-registry value>
```

自动选择结果不等于expected SHA时，保持disabled+estop并退出。

### P0-C：Candidate物理pose收据

在自动candidate capture时同步冻结只读SportModeState/独立pose样本，使：

```text
candidate_id -> timestamp -> image SHA -> independent physical pose receipt
```

可审计。CEC/单目PnP不能当ground truth。

### P0-D：`L_i` / `P_i`测量系统

仓库已实现候选系统：`deployment/odin1_gt/`。它解决了代码、收据、哈希、A*和证据包
接线，但尚未关闭硬件验收门。

必须冻结：

- scene shortest-path测量方法；
- 起点/目标成功区域；
- path采样源；
- time synchronization；
- dropout/interpolation；
- slip/drift误差；
- reset坐标系处理；
- 失败episode的path终止点。

### P0-E：Formal scene registry

在 `REALWORLD_EVALUATION.md` 和
`manifests/realworld_paired_evaluation_plan_v2.json` 填写但不伪造：

- 4个scene描述；
- 正好2个Novel与2个Revisit scene roles；
- dataset IDs和manifest hashes；
- goal hashes；
- start/yaw；
- `L_i`；
- time/path budgets；
- trial order；
- arrival module version；
- method configuration hash。

同时必须保持20个pair的arm order为native-first 10个、CEC-first 10个；每个pair两臂
共享scene、Survey、goal、start/yaw和预算，并在两臂间做物理复位与进程重启。

---

## 21. 时延和性能预期

### 21.1 已有测量

2026-08-18 dry-run记录：

- Jetson到4090直连RTT约 `2.3 ms`；
- 普通端到端inference p50/p95/max约 `0.638/0.681/0.760 s`；
- RGB-D age p95/max约 `0.066/0.138 s`。

这些数据来自早期dry-run，不是完整CEC正式campaign的p99保证。

### 21.2 当前可能的长停顿

- 首个未缓存CEC anchor可能约20秒；
- long survey冷启动逐帧重放可能需要数分钟；
- 4090被其他GPU任务占用时延迟会增加；
- Jetson trajectory timeout为2.50秒；
- 超过timeout时机器人停止，收到新trajectory后才恢复。

因此间歇式运动可能是安全设计，而不一定是D435i或Go2故障。判断时必须结合
`inference_busy`、`last_inference_s`、`plan_age_s`、`last_error`和`stop_reason`。

---

## 22. 常见故障排查

### 22.1 `waiting_for_rgbd_or_camera_info`

检查：

```bash
ros2 topic echo --once /camera/camera/color/camera_info
ros2 topic hz /camera/camera/color/image_raw
ros2 topic hz /camera/camera/aligned_depth_to_color/image_raw
tail -n 100 runtime/go2/logs/realsense.log
```

可能原因：D435i未枚举、固件门失败、USB重连、ROS discovery慢、RGB/depth shape不一致。

### 22.2 `rgbd_stale`

检查相机是否仍30Hz、同步skew、USB链路和CPU负载。不要调大timeout掩盖断流。

### 22.3 `trajectory_stale`

检查：

- SSH tunnel；
- 4090 Hub health；
- `inference_busy`；
- RTX logs；
- CEC首次anchor耗时；
- GPU是否被其他任务占用。

### 22.4 `inference_error` / `reset_required`

如果是causal append、mono receipt或NavDP state ambiguous，不允许只重试当前step。保持
运动锁止，执行完整reset或重新`formal-start`。

### 22.5 RViz路径正常但Go2不动

依次检查：

```bash
ros2 topic echo --once /navdp/status
ros2 topic echo /navdp/cmd_vel
tmux list-windows -t navdp-go2-offboard
```

重点：

- `enabled=true`？
- `estop=false`？
- `stop_reason=ready/clear`？
- `cmd_vx`是否达到 `0.10 m/s` floor？
- `cmd_wz`是否达到 `0.20 rad/s` floor？
- bridge窗口是否存活？
- 遥控器是否正在触发remote priority？
- clearance是否触发slow/stop？

### 22.6 原地左右摆动

正式profile必须是：

```text
max_linear=0.30
max_angular=0.55
heading_deadband=8 deg
bridge min_cmd_v=0.10
bridge min_cmd_w=0.20
```

不得使用旧低速override。Formal launcher会拒绝非`0.30/0.55`的正式Go2配置。

### 22.7 Goal不一致

比较：

- scene registry goal SHA；
- `/navdp/status.active_goal_sha256`；
- Hub `/healthz.active_goal`；
- Jetson selected goal file SHA；
- capture manifest goal/config identity。

任何不一致都不得enable。

### 22.8 Tunnel断开

不要依赖自动恢复继续同一formal run。断开后本轮通常应记failure/abort；恢复网络后重新
完整启动新的run ID，避免状态连续性不确定。

---

## 23. 现场角色分工

建议最少三人，资源不足时也必须明确职责：

| 角色 | 职责 |
| --- | --- |
| Safety operator | 手持Unitree遥控器，观察机器人和场地，拥有立即接管权 |
| System operator | Jetson/4090启动、status、enable/estop、日志和run ID |
| Evidence/arrival operator | 第三人称相机、同步拍手、arrival 状态、结果记录和文件封存 |

同一人兼任时，不能在机器人运动中低头操作复杂终端而失去现场观察。

---

## 24. 单次run现场检查表

### 24.1 Run前

```text
[ ] scene ID / run ID正确且唯一
[ ] dataset SEALED且manifest verify通过
[ ] expected dataset SHA已记录
[ ] expected goal SHA已记录
[ ] Go2位于预声明start/yaw
[ ] D435i安装和height未变化
[ ] formal-start全新重启双机
[ ] phase=revisit_query
[ ] active goal SHA匹配
[ ] adapter disabled
[ ] estop asserted
[ ] current RGB / depth / goal / paths在RViz可见
[ ] last_error为空
[ ] inference不忙且receipt新鲜
[ ] capture preflight通过
[ ] ROS bag / JSONL / dashboard已START
[ ] 第三人称相机已START并同步拍手
[ ] arrival/termination procedure已运行
[ ] 场地清空、遥控器操作者就位
```

### 24.2 Enable

```text
[ ] 先clear estop
[ ] 再set_enabled=true
[ ] 首个命令方向与RViz一致
[ ] vx/wz不超过formal limit
[ ] 第三人称画面持续覆盖机器人
```

### 24.3 Run中

```text
[ ] 观察真实机器人，不只看RViz
[ ] 记录任何接管、碰撞、打滑、遮挡和动态干扰
[ ] 观察status/receipt是否fresh
[ ] 观察clearance和stop_reason
[ ] 不在run中改参数、目标或预算
```

### 24.4 Run后

```text
[ ] estop asserted
[ ] adapter disabled
[ ] /navdp/cmd_vel为零
[ ] capture正常stop
[ ] third-view已导入
[ ] outcome和notes真实填写
[ ] manifest finalize成功
[ ] manifest verify成功
[ ] success adjudication文件存在
[ ] L/P来源可审计
[ ] 结果登记只填有证据字段
```

---

## 25. 完整命令速查

### 25.1 Survey

```bash
cd /home/nvidia/twork/NavDP
export CEC_HUB_SSH_HOST=work-pc

bash deployment/go2/offboard/revisit_experiment.sh \
  survey-start scene01_dataset --with-rviz

bash deployment/go2/offboard/revisit_experiment.sh survey-status

bash deployment/go2/offboard/revisit_experiment.sh \
  survey-return scene01_dataset

bash deployment/go2/offboard/revisit_experiment.sh \
  survey-seal scene01_dataset
```

### 25.2 Formal-ready

```bash
bash deployment/go2/offboard/revisit_experiment.sh \
  formal-start scene01_dataset --with-rviz

bash deployment/go2/offboard/revisit_experiment.sh formal-status

ros2 topic echo --once /navdp/status
ros2 topic echo --once /navdp/cec_receipt
```

### 25.3 Capture

```bash
bash deployment/go2/offboard/experiment_capture.sh preflight

bash deployment/go2/offboard/experiment_capture.sh start \
  scene01_formal_01 --dataset scene01_dataset \
  --trial-kind revisit --profile audit
```

### 25.4 Motion authority

```bash
source /opt/ros/humble/setup.bash

ros2 topic pub --once /navdp/estop \
  std_msgs/msg/Bool "{data: false}"

ros2 service call /navdp_go2_adapter/set_enabled \
  std_srvs/srv/SetBool "{data: true}"
```

### 25.5 Immediate stop

```bash
ros2 topic pub --once /navdp/estop \
  std_msgs/msg/Bool "{data: true}"

ros2 service call /navdp_go2_adapter/set_enabled \
  std_srvs/srv/SetBool "{data: false}"
```

### 25.6 Seal evidence

```bash
bash deployment/go2/offboard/experiment_capture.sh stop \
  scene01_formal_01

bash deployment/go2/offboard/experiment_capture.sh attach-third-view \
  scene01_formal_01 /path/to/third_view.mp4

bash deployment/go2/offboard/experiment_capture.sh finalize \
  scene01_formal_01 OUTCOME --notes "NOTES"

bash deployment/go2/offboard/experiment_capture.sh verify \
  scene01_formal_01
```

### 25.7 Stop all

```bash
bash deployment/go2/offboard/revisit_experiment.sh stop
```

---

## 附录 A：一次性安装与双机预检

### A.1 获取并验证仓库

在两台机器的各自workspace中使用同一发布commit。Jetson示例：

```bash
git clone git@github.com:AlanZhu2006/MemNav-RealWorld.git
cd MemNav-RealWorld

python3 tools/verify_public_baseline.py --workspace .
git rev-parse HEAD
git status --short
```

正式run前工作树应保持已知状态。Runtime evidence和local `.env`被Git忽略是正常的；未知
source修改必须在run manifest notes中解释或先恢复到发布commit。

### A.2 4090 `.env`

在4090：

```bash
cd /home/asus/Research/Memnav_Realworld
cp deployment/gpu/env.example deployment/gpu/.env
nano deployment/gpu/.env
```

必须指向本地合法artifact：

| 变量 | 内容 |
| --- | --- |
| `MEMNAV_PY` | MemNav/Full-Mono Python interpreter |
| `MEMNAV_SOURCE_ROOT` | external Nav-graph-blind source |
| `MEMNAV_CKPT` | MemNav checkpoint |
| `INTERNNAV_ROOT` | InternNav source |
| `LINGBOT_REPO` | LingBot repository |
| `LINGBOT_WEIGHTS` | LingBot weights |
| `LIGHTGLUE_REPO` | LightGlue source |
| `DEPENDENCY_ROOT` | frozen Python dependency tree |
| `NAVDP_CKPT` | frozen NavDP ImageGoal checkpoint |

`.env`只保存路径，不保存SSH key、token或其他credentials。External research source和model
weights不进入本仓库。

### A.3 4090 preflight和测试

```bash
cd /home/asus/Research/Memnav_Realworld
export CEC_CAMERA_HEIGHT_M=0.42

bash deployment/gpu/scripts/preflight.sh
python3 -m pytest -q deployment/gpu/tests
```

需要单独验证服务时：

```bash
bash deployment/gpu/scripts/run_policy_stack.sh
curl -fsS http://127.0.0.1:18889/healthz | python3 -m json.tool
tmux attach -t cec-realworld
```

完成后：

```bash
bash deployment/gpu/scripts/stop_policy_stack.sh
```

三个端口必须只监听loopback。不要将`8888/18888/18889`暴露到LAN或公网。

### A.4 Jetson一次性安装

```bash
cd /home/nvidia/twork/NavDP

bash deployment/go2/scripts/download_weights.sh all
bash deployment/go2/scripts/setup_jetson.sh
bash deployment/go2/scripts/preflight.sh --backend base
```

随后运行不接机器人、不发运动命令的测试：

```bash
.venv-navdp/bin/python -m unittest discover -v deployment/go2/tests
```

检查：

- ROS 2 Humble可source；
- `.venv-navdp`可执行；
- `realsense2_camera`和`rs-enumerate-devices`可用；
- D435i firmware不低于门限；
- CycloneDDS安装存在；
- Unitree SDK Python存在；
- `eth0`或指定接口拥有`192.168.123.x`地址；
- tmux、curl、GStreamer、RViz和rosbag可用；
- 磁盘空间满足capture profile。

### A.5 SSH和tunnel

```bash
ssh work-pc 'hostname; test -d /home/asus/Research/Memnav_Realworld'

export CEC_HUB_SSH_HOST=work-pc
bash deployment/go2/offboard/run_policy_tunnel.sh
```

另一个Jetson终端：

```bash
curl -fsS http://127.0.0.1:18889/healthz | python3 -m json.tool
bash deployment/go2/offboard/preflight_offboard.sh
```

Preflight必须验证protocol version、monocular sensor contract、metric-depth exclusion和terminal
schema。仅端口可连接或`algo`字段正确不足以通过。

### A.6 Camera-only十分钟验收

首次部署、换相机、换支架或更新依赖后，不启动Go2 bridge：

```bash
export NAVDP_GOAL=/absolute/path/to/image_goal.png
export CEC_CAMERA_HEIGHT_M=0.42
bash deployment/go2/nav_stack.sh start \
  --profile fullmono-lingbot-cec \
  --goal "$NAVDP_GOAL" \
  --arrival operator \
  --camera-height "$CEC_CAMERA_HEIGHT_M" \
  --with-rviz
tmux attach -t navdp-go2-offboard
```

至少运行十分钟并验证：

- adapter始终disabled；
- estop始终asserted；
- `/navdp/cmd_vel`始终为零；
- RGB/depth持续fresh；
- frames 0–39为`bootstrap_zero_depth`；
- frame 40只冻结一次scale receipt；
- 每条trajectory证明`metric_depth_sensor_consumed=false`；
- mono-depth receipt image SHA等于当前policy RGB；
- left/right certified bearing符号符合Go2 body frame；
- 停止tunnel时plan过期并归零；
- 停止MemNav时causal failure要求reset，而不是切到metric depth。

验收完成：

```bash
bash deployment/go2/nav_stack.sh stop
```

### A.7 首次有电运动验收

只在上述静态门全部通过后，以`0.5–1.0 m`短路线、系绳、宽阔平地和遥控器操作者进行。
启动`--with-go2 --with-rviz`后adapter仍disabled，必须按第15节现场检查和两步授权。

Commissioning smoke如果必须使用不同控制参数，应显式设置
`NAVDP_CONTROL_PROFILE=acceptance`，并标记为engineering run；它不能进入正式SR/SPL。

---

## 26. 文档和代码定位

本文是统一入口；以下文件提供更细的实现或历史证据：

| 文件 | 用途 |
| --- | --- |
| `CURRENT_STATUS.md` | 最新已验证/未验证边界，实验前必须查看 |
| `ARCHITECTURE.md` | protocol-v3、mono transaction和failure semantics |
| `RUNBOOK.md` | 通用启动、fault injection和旧在线两阶段兼容流程 |
| `TWO_PASS_REVISIT_RUNBOOK.md` | sealed Survey/Formal生命周期 |
| `EXPERIMENT_DATA_COLLECTION.md` | ROS bag、dashboard、third-view和manifest |
| `REALWORLD_EVALUATION.md` | 4×5 paired空白结果登记页 |
| `deployment/go2/README_CN.md` | Jetson/Go2组件级部署细节和旧ImageGoal evaluator流程 |
| `deployment/go2/STACK_MODULES_CN.md` | 当前两个导航profile、三个arrival模块和统一启动入口 |
| `deployment/gpu/realworld_cec_hub.py` | Hub、phase、goal、CEC/NavDP路由 |
| `deployment/go2/navdp_ros_node.py` | ROS adapter、服务、状态和安全门 |
| `deployment/go2/trajectory_control.py` | lookahead、depth safety和slew limit |
| `deployment/go2/go2_cmd_bridge.py` | Unitree Move、watchdog和遥控器优先级 |
| `deployment/go2/offboard/revisit_experiment.sh` | 两遍实验单入口 |
| `deployment/go2/offboard/experiment_capture.sh` | evidence-only采集入口 |
| `manifests/realworld_paired_evaluation_plan_v2.json` | 20-pair / 40-rollout machine-readable空模板 |

如果本文与旧日期文档在claim boundary上冲突，以 `CURRENT_STATUS.md` 和最新代码为准；
不得用较早文档中更宽松的arrival或自动STOP描述覆盖最新的fail-closed结论。

---

## 27. Odin1独立参考真值栈

Odin1现在可以作为与导航方法完全隔离的reference/evaluation sidecar：

```text
Survey: Odin mode 1 -> .bin + occupancy + D435i goal SHA/Odin map-pose anchor
Formal: Odin mode 2 -> stable map->odom -> map distance + local odom P_i
Score:  frozen occupancy A* L_i + D435i visual/metric/stationary S_i -> SPL_i
```

权限边界：Odin不输入CEC、NavDP、D435i安全层或Go2控制；monitor不发速度、不清急停、
不自动停止机器人。若未来A*给NavDP发子目标，必须另立mapped-navigation方法，不能计入
当前Full-Mono campaign。

一次场景流程：

```bash
# 1. 冻结0.14固件、标定与安装收据，测量高度带，再人工走完整往返Survey
bash deployment/odin1_gt/scripts/odin_gt.sh start-map scene01_survey_v1 \
  --sensor-serial <reported-serial> \
  --firmware-version <exact-0.14.x-version> \
  --calibration-file <absolute-calib.yaml> \
  --mount-receipt <absolute-validated-mount.json> \
  --obstacle-min-z <measured> --obstacle-max-z <measured>

# 2. 在B处静止，先保存D435i目标图，再绑定Odin map pose
bash deployment/odin1_gt/scripts/odin_gt.sh capture-goal scene01_survey_v1 \
  /absolute/image_goal.png /absolute/image_goal_depth.png

# 3. 回到A后保存厂商地图、occupancy和sealed goal receipt
bash deployment/odin1_gt/scripts/odin_gt.sh finish-map scene01_survey_v1
```

每次正式run必须先于NavDP启动Odin并通过重定位门：

```bash
bash deployment/odin1_gt/scripts/odin_gt.sh start-formal scene01_formal_01 \
  runtime/odin1_gt/maps/scene01_survey_v1/goal_anchor.json
bash deployment/odin1_gt/scripts/odin_gt.sh wait-ready scene01_formal_01 120

bash deployment/go2/offboard/experiment_capture.sh start scene01_formal_01 \
  --dataset scene01_survey_v1 --trial-kind revisit --profile audit \
  --gt-source odin1
```

结束时先急停/disable，再停止Odin和capture，计算并附加SPL收据：

```bash
bash deployment/odin1_gt/scripts/odin_gt.sh stop-formal scene01_formal_01
bash deployment/go2/offboard/experiment_capture.sh stop scene01_formal_01
bash deployment/odin1_gt/scripts/odin_gt.sh score scene01_formal_01 \
  --robot-radius <measured-radius> --inflation-margin 0.05
bash deployment/go2/offboard/experiment_capture.sh attach-odin-gt \
  scene01_formal_01 \
  runtime/odin1_gt/formal/scene01_formal_01/monitor/result.json \
  runtime/odin1_gt/formal/scene01_formal_01/spl_receipt.json
```

`map -> odom`缺失代表厂商可能仍在fallback SLAM，不能因为`/odin1/odometry`有数据就
开始正式run。ready后TF大跳、odom跳变、D435i视觉过期、地图SHA变化或A*无合法路径均
使run无效/失败，禁止人工补填指标。

完整安装、建图、目标绑定、formal、RViz、失败语义和P0标定见
`deployment/odin1_gt/README_CN.md`；机器冻结清单见
`manifests/odin1_gt_reference_v1.json`。截至2026-08-28，代码与离线测试已完成，但Odin
当前未连接，serial/外参/高度带/重定位率/路径精度/到达阈值均未现场冻结，因此它还不是
可直接发布结果的计量级GT。
