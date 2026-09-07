# STOP 延迟与 Revisit 起步方向审计（2026-09-07）

对象：`episode_20260907T055127_584882Z`；不修改该轮结果、冻结目标、Survey 或录制文件。

## STOP 的实际时间线

| 证据 | 本地时间 |
|---|---|
| Operator 状态进入 stopping | 13:55:24.963563 |
| 仍记录到 enabled=true、vx=0.288 m/s | 13:55:28.748668 |
| Adapter 日志首次收到 enable=false | 13:55:29.058094 |
| 状态记录到 disabled / estop / zero | 13:55:29.245255 |
| 录制关闭记录 | 13:55:34.100376 |
| 保存、收尾完成 | 13:55:53.684139 |

软件锁定记录比 operator 接受 STOP 晚约 4.28 秒。不能用“只是在保存录像”解释中间仍有非零命令。
现有记录没有浏览器实际点击时间或独立物理刹停真值；也不足以把延迟唯一归因于 DDS 或某个 callback。
旧实现的停止主题和 service 都要经过 adapter；Go2 bridge 自身只接收速度，没有独立的用户 STOP 锁存。
取消导航进程时还记录了 `rcl node's context is invalid`：默认 SIGINT 先关闭 ROS，后续清理尝试发 STOP 失败。

## 停车改动

- 新增 `/navdp/operator/stop_motion` 单向通道，用户 STOP 立即发出，并以 20 Hz 重发，最长 30 秒；使用独立、无历史保留的 BEST_EFFORT 通道，原可靠停止主题和 service 继续保留。
- Go2 bridge 直接订阅并锁存 STOP，丢弃旧指令、发送 `Move(0,0,0)` 和 `StopMove()`；锁存后不再转发后续速度，只有新 bridge 进程重新建立运行上下文。
- Adapter 同时用独立 callback group 接收该通道，禁止运动并丢弃旧执行状态。
- Operator 将相机回调与操作回调分开执行；各操作 service 仍相互串行，避免 START 清掉并发 STOP。
- `/navdp/go2/motion_stop` 返回软件锁存和零速发送状态。界面先显示 STOP 已发送，收到执行桥回执后显示零速已发送、后台保存录制；不把软件回执写成物理静止证明。
- 导航监督器自行处理 SIGINT/SIGTERM，在发送停止请求之后再销毁 ROS。
- 新的停止请求及回执进入后续 MCAP/JSONL 记录，保留实际延迟依据。

## 静止接口验证

使用新诊断工具 `deployment/go2/observe_stop_latency.py`，实际 Go2 SDK、生产 operator/bridge 类，在独立 ROS domain 73 内执行；桥接器 enabled=false，vx/vy/wz 上限全为 0。没有创建导航或 Survey，没有发送非零速度。

- STOP service 请求至真实 SDK 零速发送回执：**0.097591 秒**。
- `operator_stop_latched=true`、`zero_command_sent=true`。
- 结果：`runtime/go2/diagnostics/stop_delivery_20260907_01/result.json`。
- 这是一次静止、无完整导航负载的接口测量，不含浏览器网络，不证明运动中机械刹停时间或最坏延迟。
- 首次诊断退出时 SDK 读线程有销毁回调报错；已为诊断工具补充关闭 SDK subscribers。没有将它算作导航成功证据。

## 为什么起步向前

本轮加载了 **60 帧 sealed Survey**，NavDP FIFO 从当前 query-start 图初始化；运行 authority 为 CEC。
已记录 **49 条规划，CEC 接管 0 次**；其中运动时段（13:55:01–13:55:29）28 条规划也全部未接管，候选均为 17、拒绝原因均为 `precheck_fundamental_inliers`。

首次几何候选：

| Survey frame index | LightGlue matches | fundamental inliers |
|---|---:|---:|
| 17 | 14 | 10 |
| 45 | 16 | 9 |
| 8 | 12 | 8 |
| 33 | 9 | 8 |
| 29 | 10 | 7 |
| 24 | 8 | 7 |
| 40 | 6 | 0 |
| 59 | 6 | 0 |

Frame 17 为 DINO 第 5 名、cosine≈0.8703，经几何排序成为候选，但认证预检查未通过，没有有效 PnP 位姿/后方 bearing。`selected_anchor=17` 不能表述成“正确认出了目标”。本次看到的 frame 17 主要为支架、地面，冻结目标主要为桌下纸箱，视觉内容明显不同；未逐帧审查全部 Survey，因此不宣称整段历史一定没有目标覆盖。

没有有效记忆方向，也没有通过直接目标认证，于是按现有方法回退 `navdp_image_router`。初次监督检查记录：24 点、3.16 m 路径，预测 vx=0.25 m/s、wz=-0.49 rad/s；实际开始为向前加右转，而非经过认证的原地掉头。它的 critic 高于拒绝阈值，所以低 critic 停车也未触发。

本次不放宽几何认证、不把失败候选强制当作掉头依据，也不改变 CEC/Baseline 的回退定义。若要“没有定位就不允许起步”，那是另一个明确的实验策略改动，必须与现有方法区分。
