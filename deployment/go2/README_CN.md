# Unitree Go2 ImageGoal 部署与运行

当前支持两套清晰隔离的真机栈：原生 NavDP RGB-D baseline，以及 Jetson + RTX
4090 的 Full-Mono LingBot/CEC 栈。两者共用 Jetson 端 adapter、近障深度安全、
watchdog、Go2 bridge 和独立到达模块。

## 1. 唯一配置入口

```bash
cd /home/nvidia/twork/MemNav-RealWorld

bash deployment/go2/nav_stack.sh start \
  --config deployment/config/experiments/native_imagegoal.json \
  --dry-run
```

配置分两层：

- `deployment/config/system.json`：两机路径、模型、端口、相机高度 `0.42 m`、速度和安全参数；
- `deployment/config/experiments/*.json`：profile、ImageGoal、arrival 和是否启动相机/Go2/Foxglove。

入口先生成 `runtime/config/<config_id>.json`。这个文件包含最终绝对路径、Git
revision、ImageGoal 尺寸和 SHA-256。所有 tmux 进程只收到这一份文件；Full-Mono
把完全相同的文件复制到 4090 并再次校验。不要创建 GPU `.env`，也不要 export
`NAVDP_*` 或 `CEC_*` 参数。

## 2. ImageGoal 如何准备和选择

相机已经运行时，显式指定输出；仅保存 RGB 可省略 `--depth-output`：

```bash
bash deployment/go2/scripts/capture_image_goal.sh \
  --output deployment/go2/goals/image_goal.png
```

若需要离线深度证据：

```bash
bash deployment/go2/scripts/capture_image_goal.sh \
  --output deployment/go2/goals/image_goal.png \
  --depth-output deployment/go2/goals/image_goal_depth.png
```

然后在实验 JSON 中设置：

```json
"navigation": {
  "image_goal": "deployment/go2/goals/image_goal.png",
  "revisit_image_goal": null
}
```

`arrival.image_goal` 是终止检测参考，可以与导航目标相同，也可以独立。目标深度不
输入 RGB arrival；原生 baseline 的当前 aligned depth 是 NavDP 和本地安全输入，
Full-Mono 的 D435 depth 只留在 Jetson 本地安全层。

## 3. 启动原生 baseline

按需修改 `native_imagegoal.json` 的 `launch.go2_bridge`、`launch.foxglove` 和 arrival，
然后：

```bash
bash deployment/go2/nav_stack.sh start \
  --config deployment/config/experiments/native_imagegoal.json
```

`start` 也是唯一的整栈刷新入口：若同 profile 的 tmux 会话已经存在，它会先调用
fail-closed 停止路径、关闭整个旧会话，再让所有窗口使用当前 revision 解析出的同一
配置启动。提交代码后不要人工混用新旧配置重启单个窗口；相机恢复按钮除外，因为它
始终沿用当前会话的运行合同，并且不会恢复运动权限。

原生 profile 不访问 4090，不启动 CEC/MemNav/LingBot。它调用原版
`NavDP_Agent.step_imagegoal()`，当前 RGB-D 进入策略，配置中的目标 RGB 作为
ImageGoal。

`launch.foxglove=true`只在Jetson启动无界面、观察为主的Bridge。操作电脑打开Foxglove，
连接`ws://JETSON_IP:8765`并导入
`deployment/go2/config/navdp_debug.foxglove-layout.json`。不需要VNC。Bridge不允许浏览器
发布topic、修改参数、reset、解除estop或enable；只开放两个fail-closed调用：
`/navdp_go2_adapter/operator_stop`和`/navdp_camera_recovery/restart`。

为避免原始RGB-D把无线链路占满，启动器同时运行观察专用的`fox-preview`窗口：RGB被缩放为
640×360、15 Hz、JPEG质量75，深度被缩放、按200--4000 mm做Turbo着色后以640×360、
10 Hz、JPEG质量70发布；ImageGoal以2 Hz压缩，arrival debug最多以5 Hz压缩并保留原始
宽幅比例（当前通常为960×272），不会再把左右两幅图强拉成640×360。侧车还把
`/navdp/status`渲染成720×272、2 Hz的只读操作状态卡。

版本化布局左侧优先显示选中轨迹和宽幅arrival对比，右侧显示当前RGB、ImageGoal、深度及
状态卡。`/navdp/debug/markers`仍保留候选路径和Q值供诊断，但默认关闭，避免与
`/navdp/trajectory`中的绿色选中轨迹重复叠加。完整`/navdp/status`和
`/navdp/rgb_arrival_status`仍可从Topics侧栏按需查看，不再占用默认dashboard。
ImageGoal、最后一次arrival对比和状态卡使用transient-local显示QoS，因此Bridge或浏览器
重连后仍能立即取得最近快照；arrival panel表示“最后一次评估”，不是锁定期间的新判断。
状态卡右侧的红色`STOP NAVIGATION`按钮只执行`enabled=false + estop=true + zero command`；
它不能启动机器人，重复点击也安全。调用成功后应在状态卡看到`E-STOP / LOCKED`和零命令。
橙色`RECOVER CAMERA`会先执行相同的运动锁止，再只重启`rgbd`窗口，并等待RGB与aligned
depth各至少10幅新帧后才返回成功；无论成功或失败都不自动解除estop或恢复导航。
这些topic有损且只用于显示；NavDP、arrival和`--profile full`采集仍读取原始
848×480×30 Hz RGB-D。修改布局文件不会覆盖Foxglove已经导入的本地副本，升级后需要重新
导入一次布局，或手动更新对应panel的topic和可见性。

Bridge白名单故意不暴露四个原始图像topic，防止旧布局或临时panel绕过限流；相机标定、
状态和其他低带宽调试topic仍可查看。需要原始传感器回放时使用本机ROS或`--profile full`
MCAP，不通过远程dashboard传输。

默认Bridge监听所有网卡且不启用TLS。它会暴露相机与状态数据，并允许已连接客户端触发
STOP和fail-closed相机恢复，因此只应在可信实验局域网或Tailscale内使用；跨公网时必须
另加防火墙或加密代理。两个调用都只能移除或保持运动权限，不能授予运动权限。

数据链路是`ROS publisher -> Foxglove Bridge广告白名单topic -> 可见panel按需订阅`。
Bridge不会把所有高带宽图像无条件推给浏览器；如果Topics侧栏能看到topic但panel没有画面，
应先重新导入版本化布局并确认该panel已选择对应topic，而不是重启导航。

首次部署由`setup_jetson.sh`安装`ros-humble-foxglove-bridge`和
`ros-humble-rosbag2-storage-mcap`。Foxglove Desktop/Web运行在操作电脑，不需要安装在
Jetson。

## 4. 启动 Full-Mono

确保 Jetson 可通过 SSH alias `work-pc` 无密码登录 4090，并修改
`fullmono_imagegoal.json` 后执行：

```bash
bash deployment/go2/nav_stack.sh start \
  --config deployment/config/experiments/fullmono_imagegoal.json
```

入口自动同步 resolved 配置、执行 4090 preflight、启动/复用同 config ID 的 GPU
session、建立 loopback SSH tunnel，再启动 Jetson 相机和 adapter。4090 的模型路径
只在 `system.json` 修改。

## 5. Survey → Formal Revisit

```bash
bash deployment/go2/offboard/revisit_experiment.sh survey-start DATASET_ID
# 遥控器走出并返回；转身处执行：
bash deployment/go2/offboard/revisit_experiment.sh survey-return DATASET_ID
bash deployment/go2/offboard/revisit_experiment.sh survey-seal DATASET_ID
bash deployment/go2/offboard/revisit_experiment.sh formal-start DATASET_ID \
  --scene-id SCENE_ID --run-id RUN_ID --arm mono_cec \
  --goal /absolute/path/to/frozen_goal.jpg \
  --expected-goal-sha256 "$GOAL_SHA256" \
  --expected-dataset-sha256 "$DATASET_SHA256"
```

脚本从 Full-Mono 基础配置派生带哈希的 survey/formal 配置，不使用临时环境变量。
Formal 会自动设置选中历史目标的输出路径并启动 Go2 bridge，但仍保持运动锁定。

## 6. 状态、安全和停止

```bash
bash deployment/go2/nav_stack.sh status
bash deployment/go2/nav_stack.sh stop
```

软件急停：

```bash
ros2 topic pub --once /navdp/estop std_msgs/msg/Bool "{data: true}"
ros2 service call /navdp_go2_adapter/set_enabled \
  std_srvs/srv/SetBool "{data: false}"
```

所有启动默认运动锁定。RGB/depth 过期、同步失败、轨迹过期、近障、安全深度无效、
Go2 命令 watchdog 或遥控器接管都会阻止/终止自主速度。软件安全不能替代现场人员、
宽阔场地和手持 Unitree 遥控器。

模块分层和内部调用关系见 [STACK_MODULES_CN.md](STACK_MODULES_CN.md)；正式实验顺序
以仓库根目录 `REALWORLD_EXPERIMENT_HANDBOOK_CN.md` 和 `CURRENT_STATUS.md` 为准。
