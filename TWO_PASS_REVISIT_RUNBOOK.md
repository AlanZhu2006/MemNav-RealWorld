# 真机两阶段 Revisit 框架

Current procedure; first introduced on 2026-08-25 and maintained without a
date-stamped filename.

## 1. 目标

真机实验不再把“边走边临时挑一张图”和“正式导航”混在同一轮。系统现在明确分成：

1. **Survey / 数据集采集**：用手柄走一条较长的去程—回程路线，冻结因果 RGB
   memory 和与 memory 互斥的 Revisit goal candidates；
2. **Formal / 正式实验**：重启两台机器，从磁盘验证并重放冻结 memory，在新的物理
   起点安装一个候选目标，然后运行 CEC + frozen NavDP。

这解决了此前真机实验最重要的可复现性问题：正式结果不再依赖某个仍存活的 RTX
进程或临时内存，数据、目标和控制阶段都有独立收据。

## 2. 数据契约

一次 sealed survey 包含：

```text
episodic_datasets/<dataset_id>/
├── memory/
│   └── 000000_<sha>.jpg ...       # RTX 实际收到的 JPEG 字节
├── goals/
│   ├── candidate_000_<sha>.jpg    # 不写入 memory
│   └── ..._depth.png              # 只给独立 evaluator
├── manifest.json                  # canonical JSON
├── MANIFEST.sha256
└── SEALED
```

冻结门包括：

- 默认至少 160 个 memory frames；
- 至少一个通过在线支持门的 goal candidate；
- memory frame index 连续，文件大小和 SHA-256 与收据一致；
- goal candidate 明确记录 `appended_to_memory=false`；
- goal JPEG 与任一 memory JPEG 的 SHA-256 交集必须为零；
- aligned depth 明确记录 `evaluation_depth_policy_authority=false`；
- aligned-depth PNG 使用既有 evaluator 的毫米格式，并冻结米制比例
  `evaluation_depth_scale_m=1e-3`；它与 NavDP HTTP wire 的 `1e-4` 编码明确隔离；
- 已有 dataset id 不能覆盖或续写。

候选图不能是 memory 中的完全重复帧。否则 Revisit 会退化为 JPEG 自匹配，不能作为
定位或长程记忆证据。自动候选沿用冻结协议：只读取拍摄边界至少 16 帧以前的 history，
候选自身不 append，接受后再跳过 4 个相邻帧。自动候选门在去程保持关闭；系统没有
可信全局位姿，不能假装能自行猜中物理折返点。

## 3. 为什么要走“去程—回程”

推荐路线不是原地转圈，也不是只走单向直线：

- 去程先建立连续的环境 history；
- 回程再次经过同一区域，使当前视图与较早 history 有共视；
- 自然的横向偏移、10–30° 朝向差和动态遮挡让候选不是像素复制；
- 候选只在回程被单独截取，不写入长期 memory。

建议实际采集 3–8 分钟，经过两个以上转角或通道，最终获得 300–900 个 memory
frames 和 2–6 个候选。160 只是拒绝明显过短数据的底线，不是推荐长度。

## 4. Jetson 单入口

脚本：

```text
deployment/go2/offboard/revisit_experiment.sh
```

### 第一次：采集并冻结数据集

```bash
cd /home/nvidia/twork/MemNav-RealWorld
bash deployment/go2/offboard/revisit_experiment.sh \
  survey-start office_loop_01 --with-rviz
```

该命令从 Jetson 拉起 RTX policy stack、SSH tunnel、D435i 和 adapter。Dataset id 在 RTX
hub 第一次 `navigator_reset` 内原子打开，因此不存在 adapter 提前写入、dataset 后启动而
漏掉开头帧的竞态。Go2 bridge 不启动，adapter 始终 `disabled + estop`；机器人只由原装
手柄移动。

查看进度：

```bash
bash deployment/go2/offboard/revisit_experiment.sh survey-status
```

到达物理折返点、准备开始回程时，显式打开候选门：

```bash
bash deployment/go2/offboard/revisit_experiment.sh \
  survey-return office_loop_01
```

这个动作不改变任何电机权限，只冻结“从哪一帧起属于回程”的因果边界。之后系统才会
按固定间隔自动尝试支持验证并截取 memory-excluded candidates。

走完去程和回程后：

```bash
bash deployment/go2/offboard/revisit_experiment.sh \
  survey-seal office_loop_01
```

Seal 前脚本会再次发布 `set_enabled=false` 和 `estop=true`。若帧数、候选、哈希或互斥门
不满足，seal 失败且不会产生 `SEALED`。

### 第二次：一键准备正式实验

先把机器人手动放到预声明的正式起点，保持静止，然后执行：

```bash
bash deployment/go2/offboard/revisit_experiment.sh \
  formal-start office_loop_01 \
  --scene-id scene01 --run-id scene01_pair01_cec \
  --arm mono_cec --goal /abs/path/scene01_goal.jpg \
  --expected-goal-sha256 "$GOAL_SHA256" \
  --expected-dataset-sha256 "$DATASET_SHA256" --with-rviz
```

该命令会：

1. 安全锁止并停止上一轮 Jetson/RTX 进程；
2. 全新启动 RTX、D435i、adapter 和 Go2 watchdog bridge；
3. 逐文件校验 sealed dataset；
4. 把 survey RGB 重放给 LingBot/CEC **长程 memory**；
5. 不把 survey 末帧注入 NavDP 的短期 FIFO；
6. 用正式实验现场的当前 RGB 初始化 NavDP short context；
7. 安装 scene registry 中预先冻结的外部目标 JPEG，并逐字节核对 goal/dataset SHA-256；
8. 写入 role-hidden `formal_ready.json`，不向运行时传 Novel/Revisit 标签；
9. 保持 `disabled + estop`，等待独立 evaluator 和现场人员。

因此“一键启动”指一键达到**可审计、运动锁止的 formal-ready 状态**，不是无人值守给
电机授权。当前自动到达/STOP 尚未通过物理标定，脚本故意没有 `arm` 子命令。

正式配对实验必须显式选择 `--arm mono_native` 或 `--arm mono_cec`。前者仍重放同一
sealed Survey、使用同一 goal、因果 RGB 和 LingBot 单目深度，但 RTX hub 会跳过
certificate 与 direct-local bearing，并在每个 plan receipt 中写入
`cec_authority_mode=native`、`cec_takeover=false` 和
`cec_controller=navdp_image_authority_disabled`。不能用“CEC 恰好 reject”冒充 native arm。

在启动独立 evaluator 和现场运动授权之前，先建立本轮双视角证据会话：

```bash
bash deployment/go2/offboard/experiment_capture.sh preflight
bash deployment/go2/offboard/experiment_capture.sh start \
  office_loop_01_formal_01 \
  --dataset office_loop_01 --trial-kind revisit --profile audit
```

它自动记录 ROS bag、CEC/status JSONL 和 RViz dashboard，不改变 `disabled + estop`。
第三人称相机在命令返回后启动并做一次同步拍手。停止、导入外部视频和 SHA-256 封存步骤
见 `EXPERIMENT_DATA_COLLECTION.md`。

状态和停止：

```bash
bash deployment/go2/offboard/revisit_experiment.sh formal-status
bash deployment/go2/offboard/revisit_experiment.sh stop
```

## 5. 当前已补齐

- survey 的 exact-byte RGB 持久化；
- memory / goal candidate 强互斥；
- optional aligned depth 的 evaluator-only 权限；
- canonical manifest、SHA 和不可覆盖 seal；
- 进程重启后的全量验证与重放；
- 长程 memory 与 NavDP 短期 FIFO 的时间尺度隔离；
- 当前 query-start RGB 的正式短期初始化；
- 目标 JPEG/SHA 在线安装和 Jetson evaluator artifact；
- 每轮 ROS bag、CEC/status 收据、RViz 与第三人称视频的统一 run manifest；
- Jetson 单入口 survey / seal / formal / status / stop；
- 全流程默认无运动权限。

## 6. 仍缺失的 P0

### 6.1 独立 arrival / STOP 标定

当前最大缺口仍是到达判定，不是 CEC 检索或底盘控制。已有真机轨迹证明：

- LightGlue/PnP 可以给出有用方向；
- 单目 PnP translation scale 曾至少低估 7.9 倍；
- NavDP 零轨迹不等于到达；
- 机器人可能经过高共视窗口但没有自动停止。
- RGB-only commissioning gate 已在一次近 D 点有电测试中完成自动锁存和停车，但尚未
  经过跨场景负样本、完整路线和重复 trial 验证。

在获得带物理距离标签的 proof-conditioned convergence 规则前，formal trial 必须由独立
evaluator/操作员终止，不能报告 autonomous ImageGoal success。

### 6.2 候选时刻的独立物理 pose

新数据集已经自动保存目标 RGB 和 aligned depth，但还没有在每个自动候选时刻同步冻结
30 个 `rt/sportmodestate` 样本。因此视觉 evaluator 可以准备，辅助 ground-truth 距离、
SPL 和最终 yaw 的全自动绑定仍缺一层 candidate-id → pose receipt。正式论文 trial 前应
补齐该只读记录，不得把 CEC 的单目位姿当 ground truth。

### 6.3 正式实验矩阵

框架完成后仍需预注册并严格配对：

- frozen mono NavDP native；
- raw memory bearing；
- CEC certified bearing；
- 相同 dataset、目标、正式起点、速度/路径预算和停止 evaluator；
- 多个物理位置与多个重复 trial，而不是只展示一次成功视频。

### 6.4 重放耗时

冷启动会逐帧重放 survey 以重建 frozen LingBot state。它保证语义正确但可能需要数分钟。
在测量真实重放时延前不做缓存优化；后续可增加 hash-bound feature/KV snapshot，但快照
必须被原始 JPEG manifest 完整验证，不能改变模型状态。

## 7. 推荐下一步

1. 在机器人禁用状态下，用 3–4 个地点采集 `0/0.25/0.5/1.0m × 0/±10/±20°` 的物理
   标签，冻结 arrival 规则；
2. 同时补 candidate-id → SportModeState pose receipt；
3. 先完成一次 `survey-start -> survey-seal -> formal-start` 无运动验收；
4. 再进行系绳、低风险、操作员终止的两臂 paired trial：显式
   `mono_native` 与 `mono_cec`；
5. 当前 RGB commissioning gate 只用于显式 opt-in engineering run；跨场景 confirmation
   通过后，才允许把冻结 evaluator 合同用于正式 automatic STOP。

正式收据完成后，不回填 preregistration JSON；使用
`tools/verify_realworld_paired_campaign.py --require-complete` 从封存证据独立复算结果。
