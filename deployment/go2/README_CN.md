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
- `deployment/config/experiments/*.json`：profile、ImageGoal、arrival 和是否启动相机/Go2/RViz。

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

按需修改 `native_imagegoal.json` 的 `launch.go2_bridge`、`launch.rviz` 和 arrival，
然后：

```bash
bash deployment/go2/nav_stack.sh start \
  --config deployment/config/experiments/native_imagegoal.json
```

原生 profile 不访问 4090，不启动 CEC/MemNav/LingBot。它调用原版
`NavDP_Agent.step_imagegoal()`，当前 RGB-D 进入策略，配置中的目标 RGB 作为
ImageGoal。

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
