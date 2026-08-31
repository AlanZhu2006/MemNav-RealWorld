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

## 3. 一条命令运行原生 baseline

按需修改 `native_imagegoal.json` 的 `launch.go2_bridge`、`launch.foxglove` 和 arrival，
现场人员和遥控器就位后直接执行：

```bash
bash deployment/go2/nav_stack.sh run
```

`run` 默认使用 `deployment/config/experiments/native_imagegoal.json`，本身就是本次
运动的显式授权。它先确保机器人锁止；若当前 tmux 合同一致且所有窗口健康，就复用
相机、模型和 Foxglove，不做整栈重启。仅在会话不存在、config ID 过期、窗口缺失或
进程死亡时才走冷启动。随后它在一个 ROS 进程内完成策略 reset、等待一条 reset 后的
新轨迹、检查 RGB-D 新鲜度、ImageGoal、到达锁存、轨迹几何和至少 `0.80 m` 前方净空，
再依次 clear estop 和 enable。到达、异常、Ctrl-C 或默认 `60 s` 超时都会恢复
`disabled + estop + zero command`。

CMD 会以 `[+ 0.0s]` 形式输出 `CONNECT / RESET / PLAN / GOAL / ARM / RUNNING /
ARRIVED` 各阶段累计时间。正常热运行不再枚举全部 topic 或读取 tmux 日志；只有预检
失败时才输出具体阻塞原因。需要另一份原生配置或不同运行上限时使用：

```bash
bash deployment/go2/nav_stack.sh run \
  --config deployment/config/experiments/native_imagegoal.json \
  --timeout-s 90
```

若只想启动观察服务而绝不授权运动，仍使用锁止入口：

```bash
bash deployment/go2/nav_stack.sh start \
  --config deployment/config/experiments/native_imagegoal.json
```

`start` 是唯一的整栈刷新入口：若同 profile 的 tmux 会话已经存在，它会先调用
fail-closed 停止路径、关闭整个旧会话，再让所有窗口使用当前 revision 解析出的同一
配置启动。提交代码后不要人工混用新旧配置重启单个窗口；相机恢复按钮除外，因为它
始终沿用当前会话的运行合同，并且不会恢复运动权限。

原生 profile 不访问 4090，不启动 CEC/MemNav/LingBot。它调用原版
`NavDP_Agent.step_imagegoal()`，当前 RGB-D 进入策略，配置中的目标 RGB 作为
ImageGoal。

`launch.foxglove=true`只在Jetson启动无界面、观察为主的Bridge。操作电脑打开Foxglove，
连接`ws://JETSON_IP:8765`并选择组织 Layout `MemNav Go2 Navigation`；组织同步尚未配置时，
才回退到导入`deployment/go2/config/navdp_debug.foxglove-layout.json`。不需要VNC。Bridge不允许浏览器
发布topic、修改参数、reset、解除estop或enable；除 STOP 和相机恢复外，只开放两个
config-bound Survey 生命周期调用：`survey_start`只开始/恢复RGB记录，`survey_seal`先锁停
再校验并冻结当前resolved config指定的数据集。它们均不取得运动权限。

为避免原始RGB-D把无线链路占满，启动器同时运行观察专用的`fox-preview`窗口：RGB被缩放为
640×360、15 Hz、JPEG质量75，深度被缩放、按200--4000 mm做Turbo着色后以640×360、
10 Hz、JPEG质量70发布；ImageGoal以2 Hz压缩，arrival debug最多以5 Hz压缩并保留原始
宽幅比例（当前通常为960×272），不会再把左右两幅图强拉成640×360。侧车还把
`/navdp/status`渲染成720×272、2 Hz的只读操作状态卡。

版本化布局左侧优先显示选中轨迹和宽幅arrival对比，右侧显示当前RGB、ImageGoal、深度及
状态卡。`/navdp/trajectory`明确按4 cm细折线显示，并用绿到青的渐变区分轨迹起终方向；
`/navdp/debug/markers`仍保留候选路径和Q值供诊断，但默认关闭，避免与选中轨迹重复叠加。
完整`/navdp/status`和
`/navdp/rgb_arrival_status`仍可从Topics侧栏按需查看，不再占用默认dashboard。
ImageGoal、最后一次arrival对比和状态卡使用transient-local显示QoS，因此Bridge或浏览器
重连后仍能立即取得最近快照；arrival panel表示“最后一次评估”，不是锁定期间的新判断。
状态卡下方的绿色`START SURVEY`与蓝色`SEAL SURVEY`只在`survey-prepare`生成的Survey
栈中可用。前者建立第一帧记录边界，后者等待当前帧提交完成后冻结dataset。状态卡会用
大字显示`ACTIVE / PAUSED / SEALED`、已保存帧数和最近一次按钮结果；每个按钮面板也保留
完整response。seal失败时保持记录暂停，已有帧不会丢失，可再次Start继续采集。红色
`STOP NAVIGATION`按钮只执行
`enabled=false + estop=true + zero command`；
它不能启动机器人，重复点击也安全。调用成功后应在状态卡看到`E-STOP / LOCKED`和零命令。
橙色`RECOVER CAMERA`会先执行相同的运动锁止，再只重启`rgbd`窗口，并等待RGB与aligned
depth各至少10幅新帧后才返回成功；无论成功或失败都不自动解除estop或恢复导航。
这些topic有损且只用于显示；NavDP、arrival和`--profile full`采集仍读取原始
848×480×30 Hz RGB-D。修改布局文件不会覆盖Foxglove已经导入的本地副本，升级后需要重新
导入一次布局，或手动更新对应panel的topic和可见性。

仓库同时提供组织级 Layout 自动同步：`.github/workflows/sync-foxglove-layout.yml`
只在 `navdp_debug.foxglove-layout.json` 或同步器本身变化并 push 到 `main` 时运行，使用
唯一名称 `MemNav Go2 Navigation` 创建组织 Layout；Foxglove 首次生成的合法 ID
`lay_0eaA5tDP1ifAOz3F` 已固定在 workflow 中，之后使用该 ID 原位更新，不会反复生成
副本。首次启用前，由 Foxglove 组织管理员创建
具备 Layout 读取、创建和更新能力的新 API Key，并通过交互式命令写入 GitHub Secret；
不要把 Key 放入 JSON、workflow、shell 历史或文档：

```bash
gh secret set FOXGLOVE_API_KEY --repo AlanZhu2006/MemNav-RealWorld
```

随后可在 GitHub Actions 手动运行一次 `Sync Foxglove organization layout`，或修改布局
并 push。Foxglove 客户端只需首次从 Organization layouts 选择该 Layout；以后 CI
保存的云端版本会跨设备同步，不再导入本地 JSON。如果客户端对该 Layout 有未保存的
本地草稿，Foxglove会保护草稿而不会强制覆盖，此时需要在 Layout 菜单选择 Revert 或
刷新。同步器将 Foxglove 的 `data` 当作不稳定的不透明对象整包 round-trip，不自行转换
panel schema。

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
bash deployment/go2/offboard/revisit_experiment.sh survey-prepare DATASET_ID
# 在 Foxglove 点击 START SURVEY；旧的 survey-start 仍可一条命令直接开始
# 遥控器走出并返回；转身处执行：
bash deployment/go2/offboard/revisit_experiment.sh survey-return DATASET_ID
# 在 Foxglove 点击 SEAL SURVEY；或使用下面的CLI等价入口
bash deployment/go2/offboard/revisit_experiment.sh survey-seal DATASET_ID
bash deployment/go2/offboard/revisit_experiment.sh formal-start DATASET_ID \
  --scene-id SCENE_ID --run-id RUN_ID --arm mono_cec \
  --goal /absolute/path/to/frozen_goal.jpg \
  --expected-goal-sha256 "$GOAL_SHA256" \
  --expected-dataset-sha256 "$DATASET_SHA256"
```

脚本从 Full-Mono 基础配置派生带哈希的 survey/formal 配置，不使用临时环境变量。
Formal 会自动设置选中历史目标的输出路径并启动 Go2 bridge，但仍保持运动锁定。

### 5.1 M 点单程工程调试

若目的是先在 M 点冻结外部目标、再用原装 Unitree 手柄从 M 开到新的物理起点，关闭
双机栈后重新加载同一段历史做 CEC Revisit，使用独立的 engineering 入口：

```bash
bash deployment/go2/offboard/revisit_debug.sh record-start m_route_01 \
  --goal /absolute/path/to/revisit_m.png --point-label M

# 只用 Unitree 手柄开动；策略运动保持 disabled + estop，Go2 bridge 不启动。
bash deployment/go2/offboard/revisit_debug.sh status

# 到达新的 Revisit 起点后；未满足持久化 seal 门时会拒绝停止并保留记录现场。
bash deployment/go2/offboard/revisit_debug.sh record-stop

# 以后在同一物理起点重新加载历史并安装 exact M 目标；仍然不会启动运动。
bash deployment/go2/offboard/revisit_debug.sh revisit-prepare
```

`record-start`启动 RTX 上的 MemNav、LingBot、CEC 和 frozen NavDP，但 Jetson 不启动
Go2命令 bridge。它把单程采集明确标为
`manual_one_way_external_goal_debug`。该模式不制造与任务无关的Survey候选，候选数量
必须为零，并在数据契约中强制后续使用命令行指定且由SHA-256冻结的外部M图；普通正式
去程—回程Survey仍必须包含至少一个受支持、memory-excluded候选。

更严格地说，单程模式封存时的 Survey 候选数必须正好为零。若异常中断后留下一个与
causal memory 完全重复的候选，只能在双机停止、原始 staging 已逐文件哈希备份的前提
下使用 `deployment/gpu/recover_one_way_debug_dataset.py` 做离线恢复；恢复工具保留原始
staging 和无效候选用于审计，只把验证后的 memory 与“必须使用外部 M”契约写入最终
manifest。它不是普通正式往返 Survey 的候选门绕过工具。

Survey 锁停期间，`m-match` 是纯观察节点：持续比较 M 与实时 RGB，在 Foxglove 宽幅
match panel 顶部显示绿色 `MATCH` 或红色 `NO MATCH`，并发布 good matches、inliers、
scale 和拒绝原因。它没有 `/navdp/arrival`、enable、estop 或速度 publisher。完成
`revisit-prepare` 后，标准 RGB arrival 模块恢复；最终有电授权仍必须由现场人员单独执行。

冻结目标保留两个不可混淆的身份：`frozen_goal_source_sha256` 校验操作员指定的原始
PNG/JPEG 文件，`committed_goal_sha256` 校验 `NavDPClient` 固定 quality-95 编码后真正
安装到 CEC/策略线上的 JPEG 字节。`revisit-prepare` 同时验证两者，不能用视觉近似或
任意重编码结果代替其中任何一个。

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
