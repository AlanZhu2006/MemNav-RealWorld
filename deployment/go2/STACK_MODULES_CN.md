# Go2 真机栈模块与统一启动入口

本页回答三个问题：应该执行哪个命令、当前运行的是哪套导航，以及谁负责判断到达。
先查看可组合的 profile：

```bash
bash deployment/go2/nav_stack.sh list
bash deployment/go2/nav_stack.sh describe native-navdp-rgbd
bash deployment/go2/nav_stack.sh status
```

任何真机启动前都可以只解析合同而不启动进程：

```bash
bash deployment/go2/nav_stack.sh start \
  --profile native-navdp-rgbd \
  --goal /absolute/path/to/goal.jpg \
  --arrival operator \
  --dry-run
```

`nav_stack.sh` 只组合模块并启动为禁用/急停状态，绝不自动授予电机权限。

## 0. 启动脚本分层：应该用哪一个

| 你的目的 | 应使用的入口 | 说明 |
| --- | --- | --- |
| 在原生 NavDP 与 Full-Mono 之间切换，显式组合 arrival | `nav_stack.sh` | profile 门面；不实现模型或底盘逻辑 |
| 直接启动/查看/停止日常 Full-Mono 双机栈 | `offboard/fullmono.sh` | 你此前使用的入口，仍然正确且受支持 |
| 执行 sealed Survey → Formal Revisit | `offboard/revisit_experiment.sh` | 实验生命周期入口，内部使用 `fullmono.sh` |
| 单独调试原生 NavDP、X-NavDP、PointGoal 或 NoGoal | `scripts/run_stack.sh` | Jetson 本地 baseline/组件诊断入口 |
| 只调试 Full-Mono 的 Jetson 本地半边 | `offboard/run_offboard_stack.sh` | 内部/诊断入口；假设 RTX Hub 已就绪，不负责完整双机生命周期 |

真实调用关系是：

```text
nav_stack.sh
  ├─ native-navdp-rgbd ─────> scripts/run_stack.sh
  └─ fullmono-lingbot-cec ──> offboard/fullmono.sh
                                  ├─ SSH -> GPU run_policy_stack.sh
                                  └─ Jetson run_offboard_stack.sh
                                        ├─ tunnel
                                        ├─ D435i
                                        ├─ adapter / arrival
                                        └─ Go2 bridge / RViz（可选）

offboard/revisit_experiment.sh
  └─ survey/formal 状态机 ─────> offboard/fullmono.sh
```

因此 `fullmono.sh` 和 `run_stack.sh` 不是同一功能的两份实现：前者管理 Full-Mono 双机
生命周期，后者管理本地 native/X-NavDP baseline。脚本数量主要来自“单进程、Jetson
组合、双机组合、实验生命周期”四个层级，而不是同时运行四套导航。

## 1. 固定的四层边界

```text
D435i RGB / depth
        │
        ├── Navigation profile ──> 24点局部轨迹
        │       ├── native NavDP RGB-D
        │       └── Full-Mono LingBot + CEC + frozen NavDP
        │
        ├── Arrival module ──────> /navdp/arrival (可选)
        │       ├── operator
        │       ├── external-topic (AprilTag/SLAM/evaluator)
        │       └── rgb-homography (临时实验模块)
        │
        └── Jetson execution ───> depth safety + controller + watchdog + Go2
```

模块权限互不混淆：

- Navigation 只生成轨迹；原生 NavDP 没有可靠的 `goal_reached` 输出。
- CEC 只存在于 Full-Mono profile，负责历史检索和有证据的方向，不直接发速度。
- Arrival 只负责 episode termination，不改变 NavDP 输入或轨迹。
- Jetson execution 是两种 profile 共用的唯一底盘控制与急停层。

## 2. 已支持的导航 profile

| Profile | NavDP | 策略深度 | 记忆/CEC | 两阶段 | 用途 |
| --- | --- | --- | --- | --- | --- |
| `native-navdp-rgbd` | 原版 `navdp_pretrain.ckpt` | D435i aligned metric depth | 无 | 否 | 原生 ImageGoal baseline，隔离检查 NavDP 本身 |
| `fullmono-lingbot-cec` | frozen NavDP | causal LingBot monocular depth | CEC episodic memory | 是 | 当前 Full-Mono Novel/Revisit 系统 |

仓库当前不存在名为 `Libero` 的后端。现有单目后端名为 **LingBot**；如果“Libero”指
另一个未来模型，应新增第三个 profile，而不是改写上述两个 profile。

profile 的机器可读定义集中在 `deployment/go2/stack_profiles.py`。新增后端时，adapter、
Go2 bridge、arrival module 和相机脚本不应随之复制或改名。
命令行只接受表中的完整 profile/arrival 名称，不再接受 `native`、`cec`、`rgb`、
`manual` 等未文档化短别名。

## 3. 已支持的到达模块

| Arrival | 后台进程 | 到达权限 | 说明 |
| --- | --- | --- | --- |
| `operator` | 无 | 现场操作员 | 默认；适合先验证导航质量 |
| `external-topic` | 由实验者提供 | `/navdp/arrival` | AprilTag、SLAM、Odin或独立 evaluator 发布 Bool |
| `rgb-homography` | `run_rgb_goal_arrival.sh` | 临时 RGB 几何门 | 严格阈值首次真机验收出现 false negative；放宽后已完成一次近 D 点自动停车调试，尚未成为正式 STOP 合同 |

导航 ImageGoal 与终止参考图是两个显式参数：

```text
--goal          输入 NavDP 的任务目标图
--arrival-goal  仅输入 arrival module 的终止参考图
--arrival-phases 决定 arrival matcher 在 Novel 或 Revisit 阶段工作
```

二者可以相同，但启动器不再把这种相同关系隐藏在脚本内部。

## 4. 原生 NavDP + D435i baseline

这条链路已经实现，底层由 `scripts/run_stack.sh --backend base --mode
imagegoal`、`navdp_base_server.py` 和共用 ROS adapter 组成。统一启动方式为：

```bash
cd /home/nvidia/twork/MemNav-RealWorld

bash deployment/go2/nav_stack.sh start \
  --profile native-navdp-rgbd \
  --goal /absolute/path/to/goal_d.jpg \
  --arrival operator \
  --with-go2 \
  --with-rviz
```

该 profile 的策略输入为当前 D435i RGB、D435i aligned depth 和 ImageGoal；不访问 RTX
CEC hub，不启动 MemNav/LingBot。`--arrival operator` 表示先人工停止，从而单独判断原生
NavDP 路线是否正确。使用深度作为策略输入不等于把深度保存到实验数据。

若以后使用独立标签或定位终止：

```bash
bash deployment/go2/nav_stack.sh start \
  --profile native-navdp-rgbd \
  --goal /absolute/path/to/goal_d.jpg \
  --arrival external-topic \
  --with-go2
```

外部模块连续确认后发布：

```bash
ros2 topic pub --once /navdp/arrival std_msgs/msg/Bool "{data: true}"
```

adapter 会锁存到达、禁用运动、断言急停。它不会把外部位姿送给 NavDP。

## 5. Full-Mono LingBot + CEC

```bash
cd /home/nvidia/twork/MemNav-RealWorld

bash deployment/go2/nav_stack.sh start \
  --profile fullmono-lingbot-cec \
  --camera-height 0.42 \
  --goal /absolute/path/to/goal_d.jpg \
  --revisit-goal /absolute/path/to/goal_m.jpg \
  --arrival operator \
  --novel-navigation \
  --with-go2 \
  --with-rviz
```

`--novel-navigation` 是显式开关：缺省时两阶段 Novel memory recording 不授权导航。
临时 RGB 到达门必须显式选择，并可单独指定参考图：

```bash
bash deployment/go2/nav_stack.sh start \
  --profile fullmono-lingbot-cec \
  --camera-height 0.42 \
  --goal /absolute/path/to/goal_d.jpg \
  --arrival rgb-homography \
  --arrival-goal /absolute/path/to/arrival_d_reference.jpg \
  --arrival-phases memory_recording \
  --novel-navigation \
  --with-go2
```

## 6. 统一状态和停止

```bash
bash deployment/go2/nav_stack.sh status
bash deployment/go2/nav_stack.sh stop
```

状态会显示：profile、arrival module、导航目标路径、终止参考路径和每个 tmux window。
停止命令同时覆盖本机原生 session 与双机 Full-Mono session。
`nav_stack.sh` 会把原生 profile 委托给 `scripts/run_stack.sh`，把 Full-Mono profile
委托给 `offboard/fullmono.sh`。`fullmono.sh` 同时是受支持的双机生命周期直接入口，也是
`revisit_experiment.sh` 的底座；`run_offboard_stack.sh` 才是 Full-Mono 的 Jetson 本地
进程组合层。`scripts/run_stack.sh` 另外承担 X-NavDP/NoGoal 组件诊断。

启动成功仍然只代表栈就绪。运动必须另行释放 estop 并调用
`/navdp_go2_adapter/set_enabled`，现场人员必须握住遥控器。
