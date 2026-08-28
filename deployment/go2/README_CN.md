# NavDP 在 Unitree Go2（Jetson + D435i）上的部署

> **2026-08-21 Full-Mono 主协议：** 本文件后续章节仍保留原生
> NavDP/X-NavDP、RGB-D 调试和到达评测说明；正式 CEC offboard 路径则以
> 仓库根目录的 `ARCHITECTURE.md` 与 `RUNBOOK.md` 为准。正式路径中，Jetson
> 只向策略提供当前 RGB 和 ImageGoal；旧 HTTP depth 字段仅为 wire
> compatibility，hub 会丢弃。D435i aligned depth 只在 Jetson 本地用于近障停车
> 与可选到达审计，不输入 CEC、bearing 或 NavDP。上位机从同一 causal RGB
> LingBot state 提供 mono-depth sidecar 和 CEC proof，NavDP 仍是唯一轨迹生成器。
> 未实测 D435i 光心离地高度、未通过静态十分钟验收和故障注入前，禁止启动
> Go2 bridge 或声称已完成真机闭环。

完成一次性上位机 `.env` 配置后，推荐从 Jetson 统一启动两台机器：

```bash
cd /home/nvidia/twork/NavDP
bash deployment/go2/offboard/fullmono.sh start --with-rviz
bash deployment/go2/offboard/fullmono.sh status
bash deployment/go2/offboard/fullmono.sh stop
```

默认启动 4090 策略服务、SSH 隧道、D435i 和禁用态 adapter，不启动 Go2 bridge。
即使显式增加 `--with-go2`，也只启动带 watchdog 的底盘桥，仍不会自动解锁运动。

这套部署针对当前机器：Jetson Orin NX 16GB、JetPack/L4T 36.4、ROS 2 Humble、Intel RealSense D435i、Unitree Go2。默认策略是 **X-NavDP quadruped**，默认不使用 TinyNav VIO，也不启动 TinyNav 的任何感知、建图或规划节点。

## 1. 先说明：NavDP 是否需要 VIO

NavDP 是无地图局部导航策略，不是 VIO。它从 RGB 历史、当前对齐深度和可选目标条件中预测 24 个局部轨迹点；它本身不输出机器人全局位置，也不维护可供 ROS 使用的 `map -> odom -> base_link` 位姿。

X-NavDP 服务端允许额外传入 `robot_pos/robot_quat`，但代码中它们只用于上一条轨迹的 RTC 引导和卡住检测，主策略推理不依赖它们。本部署故意不传这两个字段。

无 VIO 时可用四种模式：

| 模式 | 后端 | 目标输入 | 是否需要 VIO | 说明 |
|---|---|---|---|---|
| `startgoal`（默认） | X-NavDP / NavDP | 启动时给一次机体系 `(x,y)` | 否 | 保持初始目标向量；对应仓库中 `eval_startgoal_wheeled.py` 的“No odometry”思路 |
| `pointgoal` | X-NavDP / NavDP | 持续给“当前机体系” `(x,y)` | 否，但上游必须更新目标 | 目标超时立即停车；适合外部定位器或上层系统提供当前相对目标 |
| `imagegoal` | 原版 NavDP | D435i 在目标位置采集的一张 RGB 图 | 否 | 视觉条件导航；没有真值位置时不能自动确认到达，首轮必须人工停车 |
| `nogoal` | 原版 NavDP | 无 | 否 | 纯视觉探索，只做局部避障和前进选择 |

重要边界：`startgoal` 没有度量定位，因此不能精确计算剩余距离，也不能像有里程计的 PointGoal 那样可靠判断到达；`imagegoal` 能根据目标画面规划，但原项目仿真仍用真值距离判成功，并没有可直接移植到真机的可靠视觉到达检测。首次真机验证应设置短目标、人工观察并随时停车。若要求“给一个全局坐标后稳定到达”，仍需要某种定位来源，但不必是 TinyNav VIO。

## 2. 部署链路

```text
D435i color ───────────────┐
                           ├─> ROS adapter ─HTTP─> NavDP/X-NavDP
D435i aligned depth ───────┘                         │
                                                    └─ 24 点局部轨迹
                                                           │
                            depth hard-stop + watchdog <────┘
                                                           │
                                             /navdp/cmd_vel (vx, wz)
                                                           │
                                      Go2 bridge + 0.35s timeout
                                                           │
                                            SportClient.Move(vx,0,wz)
```

轨迹使用 ROS 机体平面约定：`x` 向前、`y` 向左。控制器采用几何前视，将局部轨迹变成 `vx/wz`；默认禁止倒车，`vy=0`。控制话题使用独立的 `/navdp/cmd_vel`，不会和现有 `/cmd_vel` 链路互相抢占。

## 3. 文件说明

- `navdp_ros_node.py`：RGB-D/目标到策略服务的 ROS 2 适配器，以及全部运动看门狗。
- `trajectory_control.py`：可单元测试的轨迹跟踪、速度斜坡和深度安全逻辑。
- `navdp_client.py`：严格匹配原项目 JPEG + 16-bit depth HTTP 格式。
- `capture_image_goal.py`、`image_goal_io.py`：从 D435i 采集并无损保存目标图。
- `imagegoal_experiment.py`：读取 Go2 本体位姿作为旧版辅助评测源，记录首次到达/revisit 的距离、SPL 和最终朝向误差；它不会发送给NavDP，但腿式打滑/漂移使其不再作为推荐正式GT。正式评测优先使用隔离的`deployment/odin1_gt/`参考栈。
- `debug_visualization.py`：候选轨迹排序与 Q 值颜色映射，不参与控制。
- `go2_cmd_bridge.py`：从本机已成功 TinyNav 部署中移植的 `SportClient.Move()` 桥；保留超时和手柄优先权。
- `navdp_base_server.py`：原版 NavDP 的无可视化轻量服务端，支持 `pointgoal/nogoal/imagegoal`。
- `config/navdp_go2.yaml`：保守真机参数。
- `config/navdp_debug.rviz`：RGB-D、最优路径、候选路径、目标和控制状态的预配置 RViz 视图。
- `scripts/`：环境安装、权重、相机、服务端、适配器、Go2 桥和整栈启动脚本。

Go2 桥默认借用 `/home/nvidia/twork/tinynav/.venv` 中已经验证过的 `cyclonedds` 与 `unitree_sdk2py` Python 依赖；这只是在复用 SDK 运行环境，不会运行 TinyNav VIO。可通过 `GO2_PYTHON` 换成其他等价环境。

## 4. 相机安装与输入

X-NavDP 的 Go2 仿真配置使用前视 D455，位置约为 Go2 `base` 前方 `0.32m`、上方 `0.20m`，图像比例为 16:9。D435i 应尽量：

1. 固定在机身正前方，光轴与机身 `x` 轴对齐，避免明显侧偏或滚转；
2. 高度和俯仰尽量接近训练配置；
3. 使用 848×480×30 的 16:9 color/depth；
4. 必须使用 `/camera/camera/aligned_depth_to_color/image_raw`，不能把未对齐 depth 与 color 混用；
5. 保持镜片清洁，并在实测深度 ROI 后再调整急停距离。

适配器从 `CameraInfo.K` 读取真实内参，不写死 D455 内参。RealSense 原始 `16UC1` 毫米深度会转换为米，再按 NavDP 协议编码成 `meter × 10000` 的 uint16 PNG。

## 5. 一次性安装

当前工作区已经下载的权重位置：

```text
baselines/x-navdp/checkpoints/x-navdp_posttrain.ckpt
baselines/navdp/checkpoints/navdp_pretrain.ckpt
```

重新下载或换机部署：

```bash
cd /home/nvidia/twork/NavDP
bash deployment/go2/scripts/download_weights.sh all
bash deployment/go2/scripts/setup_jetson.sh
```

`setup_jetson.sh` 创建带 ROS system packages 的 `.venv-navdp`，安装 NVIDIA JetPack 6.x aarch64 PyTorch wheel和本地 cuSPARSELt，再安装最小推理依赖。它不安装 Isaac Sim、Isaac Lab、Open3D、acados 或训练依赖。

验证：

```bash
bash deployment/go2/scripts/preflight.sh --backend x
```

## 6. 首次联调：不连接 Go2

先只启动相机、策略服务和适配器：

```bash
cd /home/nvidia/twork/NavDP
bash deployment/go2/scripts/run_stack.sh --backend x --mode startgoal
tmux attach -t navdp-go2
```

第一次收到 CameraInfo 后才会加载 833MB 权重；Jetson 上第一次 reset/inference 需要明显更久。另开终端观察：

```bash
source /opt/ros/humble/setup.bash
ros2 topic echo /navdp/status
ros2 topic hz /camera/camera/color/image_raw
ros2 topic hz /camera/camera/aligned_depth_to_color/image_raw
curl http://127.0.0.1:8888/healthz
```

发布一个初始机体系目标，例如前方 3m、左侧 0.5m：

```bash
ros2 topic pub --once /navdp/relative_goal geometry_msgs/msg/PointStamped \
  "{header: {frame_id: base_link}, point: {x: 3.0, y: 0.5, z: 0.0}}"
```

此时运动仍锁定，但会持续发布 `/navdp/trajectory`。可以临时解锁适配器来检查 `/navdp/cmd_vel`，因为 Go2 桥尚未运行，所以机器人不会动：

```bash
ros2 service call /navdp_go2_adapter/set_enabled std_srvs/srv/SetBool "{data: true}"
ros2 topic echo /navdp/cmd_vel
ros2 service call /navdp_go2_adapter/set_enabled std_srvs/srv/SetBool "{data: false}"
```

必须确认：直行轨迹产生正 `linear.x`；左转产生正 `angular.z`；RGB/深度遮挡或策略停止时，命令立即归零。

## 7. 实时 Debug 界面

底盘可以保持下电；不要加 `--with-go2`。一条命令启动相机、X-NavDP、禁用态适配器和 RViz：

```bash
cd /home/nvidia/twork/NavDP
NAVDP_TMUX_SESSION=navdp-debug \
  bash deployment/go2/scripts/run_stack.sh \
  --backend x --mode startgoal --with-rviz
```

`run_debug_ui.sh` 会自动寻找 Jetson 本地桌面的 `DISPLAY=:0/:1`。如果从没有图形转发的纯 SSH 终端运行，可在 Jetson 桌面终端执行，或配置 SSH X forwarding。界面启动后发布目标：

```bash
source /opt/ros/humble/setup.bash
ros2 topic pub --once /navdp/relative_goal geometry_msgs/msg/PointStamped \
  "{header: {frame_id: base_link}, point: {x: 1.5, y: 0.0, z: 0.0}}"
```

RViz 使用仅供显示的恒等 `navdp_local -> base_link` 静态 TF；它不包含任何定位或里程计。标记含义：

- 粗绿色线：策略最终选中的 `/navdp/trajectory`；
- 细线：Q 值最高的 6 条候选轨迹，红色分数高、蓝色分数低，末端显示 Q 值；
- 红球：相对目标；青球：控制器 `lookahead` 前视点；灰色矩形：Go2 近似 footprint；
- 黄色状态字：运动锁定；绿色：已使能；红色：急停或推理错误；同时显示 `vx/wz`、中央深度和推理耗时；
- `RGB Camera` 与 `Aligned Depth`：实时 D435i 原图和对齐深度。

只单独打开界面：

```bash
bash deployment/go2/scripts/run_debug_ui.sh
```

常用的辅助调试窗口：

```bash
ros2 run rqt_image_view rqt_image_view /camera/camera/color/image_raw
rqt_plot /navdp/cmd_vel/linear/x /navdp/cmd_vel/angular/z
ros2 topic echo /navdp/status --qos-durability transient_local
```

这里看到的是**每次推理时以当前机器人为原点的局部轨迹**，不是世界地图轨迹。由于本部署没有 VIO/里程计，RViz 不会出现 `map -> odom` 运动轨迹，`startgoal` 也不能在界面中显示真实剩余距离。

原项目的典型开发方法也是“策略 HTTP server + teleop/eval client”：服务端输出 `trajectory/all_trajectory/all_values`，teleop 或仿真评测端把候选路径画到俯视图。X-NavDP 自带可视化还会写入 `vis_output/.../*.mp4`；真机部署关闭了那条 Matplotlib/MP4 路径以节省 Jetson 推理资源，改由上述 ROS Marker/RViz 做实时可视化。

### 7.1 正式实验双视角采集

正式实验不再只依赖手工录屏。栈和 RViz 已启动、但还没有解除急停时，先执行：

```bash
bash deployment/go2/offboard/experiment_capture.sh preflight
bash deployment/go2/offboard/experiment_capture.sh start RUN_ID \
  --dataset DATASET_ID --trial-kind revisit --profile audit
```

脚本自动记录 ROS bag、`/navdp/status`、完整 CEC 收据、evaluator 状态和 RViz
desktop H.264 视频，但不会发布速度、使能 adapter 或解除急停。命令返回后启动独立手机/相机
的第三人称录像，并做一次可见的同步拍手，再按正式 runbook 单独启动 evaluator 和运动授权。

实验结束必须先急停，再封存采集：

```bash
bash deployment/go2/offboard/experiment_capture.sh stop RUN_ID
bash deployment/go2/offboard/experiment_capture.sh attach-third-view \
  RUN_ID /path/to/third_view.mp4
bash deployment/go2/offboard/experiment_capture.sh finalize \
  RUN_ID success --notes "operator-confirmed outcome"
bash deployment/go2/offboard/experiment_capture.sh verify RUN_ID
```

每个文件的大小和 SHA-256 会写入不可变 manifest。缺 ROS bag、RViz 视频、第三人称视频、
status 或 CEC 收据时，正式 finalize 默认失败。完整证据边界见仓库根目录
`EXPERIMENT_DATA_COLLECTION.md`。

## 8. 接入 Go2

当前成功网络约定是 Jetson `eth0=192.168.123.100/24`，Go2 `192.168.123.161`：

```bash
sudo ip link set eth0 up
sudo ip addr replace 192.168.123.100/24 dev eth0
ping -c 2 192.168.123.161
bash deployment/go2/scripts/preflight.sh --backend x --with-go2
```

停止无机器人栈，然后显式启用 Go2 桥：

```bash
bash deployment/go2/scripts/stop_stack.sh
bash deployment/go2/scripts/run_stack.sh --backend x --mode startgoal --with-go2
```

建议先将 Go2 放在宽阔平整区域，速度使用已完成真机验证的默认 `0.30m/s`，一人始终拿着手柄。目标发布后，最后一步才解锁：

```bash
ros2 topic pub --once /navdp/relative_goal geometry_msgs/msg/PointStamped \
  "{header: {frame_id: base_link}, point: {x: 2.0, y: 0.0, z: 0.0}}"
ros2 service call /navdp_go2_adapter/set_enabled std_srvs/srv/SetBool "{data: true}"
```

Go2 桥有第二层 0.35s 命令超时；手柄任一摇杆超过 deadband 后，自动释放 NavDP 的 SportClient 控制并调用 `StopMove()`。

Go2 本体有速度门控。NavDP 桥必须保持与本机已验证 TinyNav 桥相同的最小非零命令：平移 `GO2_MIN_CMD_V=0.10m/s`、旋转 `GO2_MIN_CMD_W=0.20rad/s`。低于门槛时可能只出现腿部姿态响应而没有有效位移，不能据此判断导航已驱动底盘。与 TinyNav 一样，轨迹控制器必须先应用 `8°` 航向误差死区，再由 Go2 桥施加最小角速度；不能把正负交替的微小角速度逐帧直接抬升到 `±0.20rad/s`，否则会产生左右 hunting。正常实验的线速度上限默认使用 TinyNav 导航模式及本轮 NavDP 真机共同验证有效的 `GO2_MAX_VX=0.30m/s`；首次短距测试若需更保守，可临时覆盖上限，但不要把 `GO2_MIN_CMD_V` 降到 `0.10` 以下，应通过限制解锁时长控制测试距离。

## 9. PointGoal、ImageGoal 与 NoGoal

`pointgoal` 不接收里程计，但要求上游持续发送“当前”机体系目标，建议至少 2Hz：

```bash
bash deployment/go2/scripts/run_stack.sh --backend x --mode pointgoal
ros2 topic pub -r 5 /navdp/relative_goal geometry_msgs/msg/PointStamped \
  "{header: {frame_id: base_link}, point: {x: 2.0, y: 0.0, z: 0.0}}"
```

上面的固定示例只表示持续朝当前机体前方 2m 导航，不代表固定世界目标。固定世界目标必须由上游根据某种定位结果更新相对向量。

### ImageGoal 真机流程

ImageGoal 使用原版 NavDP `NavDP_Agent.step_imagegoal()`，不使用 LoGoPlanner、X-NavDP 或 TinyNav 的模型、VIO、规划节点。Go2 DDS 传输默认只复用本机已验证环境中安装的 Unitree SDK Python 包，不会启动 TinyNav。首轮选择静态、纹理明显、无遮挡的直线路线，目标距起点约 `2–3m`。目标图和起始姿态应保持相同机身朝向、相机高度与俯仰，避免目标画面中出现人员、移动门或强烈光照变化。

先关闭导航栈，只运行相机，并用原装遥控器将 Go2 移到目标位置：

```bash
bash deployment/go2/scripts/stop_stack.sh
bash deployment/go2/scripts/run_realsense.sh
```

Go2 在目标位置完全静止后，另开终端采集。组合脚本从 10 对同步 D435i RGB-D 中保存最清晰的一对，同时从 `rt/sportmodestate` 的 30 个样本记录辅助位姿：

```bash
bash deployment/go2/scripts/capture_imagegoal_reference.sh
```

默认目标文件是 `deployment/go2/goals/image_goal.png`，对齐深度是 `deployment/go2/goals/image_goal_depth.png`，评测参考是 `deployment/go2/goals/image_goal_pose.json`。原版 NavDP 的 HTTP 请求仍只收到目标 RGB 和当前 RGB-D；保存的目标深度只供独立视觉评测器确认是否回到相同视角，Go2 位姿只记录辅助距离、路径和 SPL。采集完成后不要重启 Go2，否则本体位置坐标系可能重置；停止相机，再用遥控器把 Go2 移回起点。最好保持原朝向，或在起点恢复到采集目标图时的相同 yaw。

第一阶段不接 Go2 桥，只检查目标图、候选轨迹和选中轨迹：

```bash
bash deployment/go2/scripts/run_stack.sh --backend base --mode imagegoal --with-rviz
ros2 topic echo /navdp/status
```

RViz 的 `Image Goal` 显示保存的目标图，`RGB Camera` 显示当前画面。只有当连续多次规划方向都合理时，才停止无桥栈并启动真机栈：

```bash
bash deployment/go2/scripts/stop_stack.sh
bash deployment/go2/scripts/run_stack.sh --backend base --mode imagegoal --with-go2 --with-rviz
```

评测终端先启动首次到达评测器：

```bash
bash deployment/go2/scripts/run_imagegoal_evaluator.sh run \
  --episode first --arrival-mode object --auto-estop
```

确认评测器开始输出 `visual.reason`、当前匹配指标和辅助距离后，在控制终端解锁适配器：

```bash
ros2 service call /navdp_go2_adapter/set_enabled std_srvs/srv/SetBool "{data: true}"
```

默认 `--arrival-mode object` 判定“找到并靠近目标物”。它仍保留 SIFT ratio test、RANSAC 单应性、内点数/比例、匹配覆盖、中心、尺度、旋转和重投影误差门槛，同时检查匹配点的深度变化是否一致。默认允许当前视角比目标图近 `1.25m`、最多远 `0.25m`，但深度差的 MAD 必须不大于 `0.20m`；因此它允许靠得比目标照片近，又不会把远处偶然相似纹理当作到达。连续 3 帧识别成功后，还必须同时满足：

1. Go2 三轴 L1 速度不大于 `0.10m/s`；
2. `/navdp/status` 表明原版 NavDP 已使能、无急停、无传感器/推理/障碍停车；
3. NavDP 发布速度接近零，且局部轨迹最大半径不大于 `0.20m`，持续至少 `1.50s`。

这些条件把“策略主动认为无需继续运动”和“急停、失联、障碍导致的被动零速度”区分开。它是目标实例的几何外观确认，不是开放词汇语义识别。

`--arrival-mode visual` 保留原来的严格同视角标准：除上述基础几何匹配外，还要求更大的匹配覆盖、更窄的中心/尺度范围以及匹配点绝对深度误差不大于 `0.40m`。`--arrival-mode visual_pose` 再叠加原仿真 `0.85m` 位姿门槛，`--arrival-mode pose` 仅用于对照。结果 schema v3 同时记录 `goal_object_success`、`exact_view_success`、`pose_success` 和 `policy_stop`，不能把四者混为一个指标。RViz 的 `ImageGoal Visual Match` 显示目标/当前匹配画面；成功时 `--auto-estop` 向 `/navdp/estop` 发布急停。`SportModeState` 只做静止门控及辅助距离、路径和旧版SPL记录，不输入 NavDP。正式`L_i/P_i/S_i`可由隔离的Odin1参考栈产生，但Odin同样不输入策略。策略本身仍不会收到位置、目标深度或 `goal_reached`；操作员必须继续拿着遥控器。

### Revisit 协议

首次到达后保持急停，关闭适配器运动，用遥控器将 Go2 移到第二个起点；目标图和目标位姿文件都不能更换：

```bash
ros2 service call /navdp_go2_adapter/set_enabled std_srvs/srv/SetBool "{data: false}"
```

在第二个起点调用 `reset_policy`，清空原版 NavDP 的 8 帧历史，使 revisit 成为使用同一目标图的独立 episode：

```bash
ros2 service call /navdp_go2_adapter/reset_policy std_srvs/srv/Trigger "{}"
```

在评测终端启动 revisit：

```bash
bash deployment/go2/scripts/run_imagegoal_evaluator.sh run \
  --episode revisit --arrival-mode object --auto-estop
```

确认评测器开始输出视觉匹配指标、策略重新产生合理轨迹后，在控制终端释放急停并解锁：

```bash
ros2 topic pub --once /navdp/estop std_msgs/msg/Bool "{data: false}"
ros2 service call /navdp_go2_adapter/set_enabled std_srvs/srv/SetBool "{data: true}"
```

主实验要求 `first` 和 `revisit` 两个 episode 都达到 `goal_object_success=true`；同时独立报告 `exact_view_success`，不要求机器狗精确复现拍照距离。比较两次匹配内点数、画面覆盖率、深度差及 MAD、最小辅助距离、路径长度、SPL、耗时和最终 yaw 误差。后续可增加“不 reset 历史”的连续回访作为补充实验，但不能替代上述独立 revisit。

纯视觉无目标探索使用原版模型：

```bash
bash deployment/go2/scripts/run_stack.sh --backend base --mode nogoal
```

`nogoal` 没有任务级终点，只适合受控空间实验，不能替代全局导航系统。

## 10. 安全机制与停止方式

从策略到电机共有以下 fail-closed 条件：

1. 默认 `enable_on_start=false`；必须调用服务才允许运动；
2. RGB 或 depth 超过 `0.60s` 未更新，立即归零；
3. RGB/depth 只以时间戳差不超过 `0.10s` 的完整帧对更新；超过 `0.60s` 没有新帧对时立即归零；
4. 轨迹超过 `2.50s`，立即归零；
5. `pointgoal` 超过 `0.75s` 未更新，立即归零；
6. 中央深度 ROI 无有效值时 fail-closed；低于 `0.45m` 全停，`0.45–0.80m` 线性减速；
7. Go2 桥超过 `0.35s` 未收到命令，`Move(0,0,0)` 并 `StopMove()`；
8. 手柄活动优先于自主命令；
9. 默认禁止倒车，避免前视相机看不到后方风险。
10. ImageGoal 到达评测与策略隔离：目标物模式使用 RGB-D 几何匹配、NavDP 零轨迹和 Go2 静止状态；Go2 位姿只做辅助指标。可选 `--auto-estop` 只做 episode termination，操作员仍需随时接管。

软件急停：

```bash
ros2 topic pub --once /navdp/estop std_msgs/msg/Bool "{data: true}"
ros2 service call /navdp_go2_adapter/set_enabled std_srvs/srv/SetBool "{data: false}"
```

恢复前需先释放急停，再重新显式 enable：

```bash
ros2 topic pub --once /navdp/estop std_msgs/msg/Bool "{data: false}"
```

停止所有进程：

```bash
bash deployment/go2/scripts/stop_stack.sh
```

## 11. 状态与调参

`/navdp/status` 是 JSON 字符串，关键字段：

- `stop_reason=ready/clear/obstacle_slow`：链路可运行；
- `disabled/estop`：人为锁定；
- `waiting_for_rgbd_or_camera_info`：相机输入不完整；
- `rgbd_stale`：超过 `0.60s` 没有收到满足时间戳阈值的完整 RGB-D 帧对；
- `waiting_for_goal/goal_stale`：目标未提供或 PointGoal 上游停止；
- `image_goal_loaded`：ImageGoal 目标图已成功载入；
- `waiting_for_plan/trajectory_stale/inference_error`：策略端不可用；
- `depth_unavailable_stop/obstacle_stop`：深度安全停车。
- `candidate_count/last_inference_s`：当前发布的候选数量与最近一次策略耗时；
- `clearance_m/cmd_vx/cmd_wz`：中央深度 ROI 距离与适配器最终输出速度。

真机调参顺序：

1. 先验证相机视野、色彩和对齐深度；
2. 机器人不接桥时验证轨迹方向；
3. 只调 `depth_roi_*` 与安全距离，避免地面造成长期误停；
4. 首次场地验收可临时以 `max_linear_mps=0.10/0.15/0.20` 分级测试；完成验收后使用已验证默认 `0.30`。Go2 桥始终保持 `min_cmd_v=0.10`、`min_cmd_w=0.20`，短距测试通过限制解锁时长而不是降低最小速度；
5. 若 Jetson 推理时间大于 2s，将 `trajectory_timeout_s` 调到约 `2 × 实测推理时间`，但不能取消 Go2 桥 0.35s 命令超时；
6. 不建议在没有后视传感器时开启 `allow_reverse`。

## 12. 当前设计取舍

- PointGoal/StartGoal 推荐 X-NavDP，因为官方后训练权重包含 quadruped/Go2 embodiment；ImageGoal 和 NoGoal 使用包含对应网络头的原版 NavDP 权重。
- 不使用 TinyNav VIO；因此也不依赖 `/slam/odometry`。
- 不用原仓库的仿真 MPC/acados。真机适配器直接跟踪局部轨迹，依赖更少，也便于设置硬看门狗。
- 不在策略服务中写 MP4，减少 Jetson 的 CPU、磁盘和内存压力。
- 当前深度急停只是前视 ROI，不是认证安全系统；楼梯、透明物体、低矮障碍和后方仍需人工与额外传感器保护。

## 13. 当前机器验证记录

2026-08-11 已在本机完成：

- 两份官方权重的 SHA256 校验；
- NVIDIA PyTorch 2.5 wheel、CUDA 12.6 和 Orin GPU 可用性验证；
- X-NavDP HTTP 合成输入推理，输出 `(1,24,3)`，冷态约 `1.75s`；
- D435i 848×480 aligned RGB-D 实时推理，热态约 `0.94–1.07s`；
- 原版 NavDP `nogoal` 推理，约 `1.46s`；
- 原版 NavDP 权重包含完整 ImageGoal 编码器参数；合成 RGB-D 与目标图经真实 `/imagegoal_step` 推理得到 `(1,24,3)` 选中轨迹、`(1,16,24,3)` 候选轨迹，耗时约 `1.48s`；
- ImageGoal ROS 无相机烟测成功载入目标图并发布 `/navdp/image_goal`，状态为 `image_goal_loaded=true`、`disabled`、零速度；测试栈随后已清理；
- ImageGoal 评测器已从 `rt/sportmodestate` 连续读取 30 个稳定样本，并完成只读 ROS episode 超时烟测；随后增加与策略隔离的同步 RGB-D 视角验证器，离线正例、错场景、深度不一致和连续命中逻辑均通过测试；目标图/深度 SHA256、视觉匹配指标、辅助最小距离、路径长度、SPL 和最终 yaw 误差均可落盘；
- 两次独立 D435i 目标视角采样得到 667 个 RANSAC 几何内点、`0.994` 内点率和约 `0.001m` 深度中位误差；在 Unitree 运行环境中单次验证平均约 `0.066s`，本地 ROS 端到端烟测已验证连续 3 次命中后正常结束 episode；
- 实际 CameraInfo 内参、深度单位/有效率、24 点 ROS Path 和禁用时零速检查；
- RViz2 中实时 RGB、aligned depth、候选轨迹、选中轨迹、目标、前视净空和控制状态显示；
- 适配器临时解锁但不启动 Go2 桥时的非零控制输出，以及重新锁定后的立即归零；
- tmux 整栈启动、停止和无遗留进程检查。

随后已验证 `eth0` 的 Go2 子网、到 `192.168.123.161` 的零丢包连通，以及 Go2 桥的 SDK/ROS 初始化、零 Twist 和干净退出。重新上电后的短时运动测试中，桥实际发送 `Move(vx=0.100, vy=0.000, wz=0.000)`；Go2 `rt/sportmodestate` 独立监测到前向速度峰值 `0.127m/s`、最大平面位移约 `0.036m`，确认已产生有效位移而不只是腿部姿态响应。遥控器摇杆峰值为零，测试后速度回到近零、四足受力正常，适配器恢复 `disabled + estop`，整栈关闭后 ROS 图与相关进程为空。

该短时测试约 `0.4s` 后检测到一次 `rgb_depth_unsynchronized` 并按 fail-closed 逻辑归零，独立硬看门狗随后也切断了 Go2 桥。后续无桥对照确认 D435i 同步帧本身正常，误停来自适配器分别维护两条话题的“最新帧”；当前实现已改为用 `ApproximateTimeSynchronizer` 原子更新完整 RGB-D 帧对。

同步修复后完成了两级回归，并继续完成速度分级测试：

- X-NavDP、D435i 与 RViz 全负载、无 Go2 桥运行 `45s`，91 个启用状态全部为 `clear`，没有 `rgbd_stale`；帧对最大年龄 `0.220s`、最大时间戳偏差 `0.0667s`，热推理 `1.042–1.122s`；
- 同一负载下以 `0.10m/s` 再次进行约 `1.5s` 真机短移，桥连续记录 4 次 `Move(vx=0.100)`，4 个运动状态全部为 `clear`；帧对最大年龄 `0.138s`、最大偏差 `0.0334s`，Go2 前向速度峰值 `0.131m/s`、有效位移约 `0.073m`，遥控器未介入且硬看门狗未触发。
- 分级提高速度后确认 `0.15m/s` 会受机体门控影响而长时间间歇，`0.20m/s` 已可运动但仍有停顿；以 `0.30m/s` 连续运行 `5s` 时桥达到 `Move(vx=0.300)`，净位移约 `1.052m`、平均位移速度约 `0.210m/s`，约 `97%` 的采样处于运动状态，期间策略状态保持 `clear`。因此适配器、轨迹控制器和 Go2 桥的线速度上限统一默认为 `0.30m/s`。

测试后 Go2 速度回到近零、四足受力正常，适配器恢复 `disabled + estop`；tmux、ROS 图和相关进程均已清空。当前已通过直行连续运动与同步修复验证，长距离、明显转弯和动态避障仍需分阶段验收。
