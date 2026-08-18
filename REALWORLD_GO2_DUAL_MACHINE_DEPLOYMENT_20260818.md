# CEC + NavDP Unitree Go2 双机部署（2026-08-18）

## 1. 当前结论

真机部署采用以下职责边界：

```text
Unitree Go2 / Jetson Orin NX                         RTX 4090 workstation
────────────────────────────                        ───────────────────────
D435i 同步 RGB-D
        ↓
已有 NavDP ROS adapter ── loopback SSH tunnel ──→  CEC unified hub
        ↑                                             ├─ MemNav causal buffer
        │                                             ├─ DINO top-K
24 点局部轨迹 ←───────────────────────────────────────┤  LightGlue + LingBot/PnP
        ↓                                             └─ frozen NavDP
本地轨迹跟踪 + depth fail-closed
        ↓
/navdp/cmd_vel
        ↓
0.35 s watchdog + 手柄优先权
        ↓
SportClient.Move()
```

TopoFocus 仅作为双机职责和 fail-closed 设计的参考。没有复用或修改
TopoFocus 的模型、planner、协议或代码。下位机沿用已经部署并真机验证过的
NavDP/Go2 执行链，不强制引入 TinyNav/VIO/地图规划。

## 2. 为什么这样分

- Jetson 保留传感器、同步、轨迹跟踪、深度急停、速度限幅、手柄接管和底盘
  watchdog，因此 4090 断线不会让旧命令持续生效。
- 4090 只输出 NavDP 的 24 点局部轨迹，不直接输出底盘速度，也没有 Unitree
  SDK 权限。
- CEC 的多服务状态被封装在一个上位机入口内；Jetson 不分别调用 MemNav 和
  NavDP，避免一次规划中出现多端状态分叉。
- 所有策略端口只监听 `127.0.0.1`，Jetson 通过 SSH local-forward 访问，不在
  局域网暴露模型服务。

## 3. 冻结的在线路由

上位机 `realworld_cec_hub.py` 对下位机继续暴露原 NavDP 契约：

- `POST /navigator_reset`
- `POST /imagegoal_step`
- `GET /healthz`

每个 ImageGoal step 的内部顺序固定为：

```text
current RGB + goal RGB
    → MemNav /retrieval_probe_step（当前帧只 append 一次）
    → /certified_relocalize
        ├─ certificate accept + units 合法
        │    → verified_bearing_v1
        │    → 归一化后投影到冻结 2.5 m
        │    → NavDP /navdp_step_ip_mixgoal
        └─ reject / certificate error
             → NavDP /imagegoal_step
```

故障语义：

- retrieval probe 失败：当前步仍走 native；该 session 的 memory 永久降级，
  直到显式 reset，避免在缺帧历史上继续作证；
- certificate 失败：当前帧已正确写入 memory，精确回退 native，下步可继续；
- native/mixed 请求出现不确定失败：hub 锁存 `reset_required`，不再自动推进
  NavDP 状态；
- hub 并发请求返回 `409 hub_busy`，不会并行写两个 stateful policy。

## 4. 下位机现状（只读审计）

主机：`tegra-ubuntu`，用户 `nvidia`，Jetson Orin NX 16GB。

现有部署：

```text
/home/nvidia/twork/NavDP/deployment/go2
```

关键文件：

- `navdp_ros_node.py`：RGB-D 同步、异步推理、轨迹执行、运动锁；
- `navdp_client.py`：NavDP JPEG + uint16 depth HTTP wire contract；
- `trajectory_control.py`：局部轨迹前视、速度斜坡、深度安全；
- `go2_cmd_bridge.py`：Unitree SDK、0.35 s watchdog、手柄优先；
- `scripts/run_adapter.sh`：已经支持用 `NAVDP_SERVER_URL` 指向远端；
- `goals/image_goal.png`：现有真机目标图流程。

新增覆盖层在：

```text
/home/nvidia/twork/NavDP/deployment/go2/offboard
```

它只新增脚本，不覆盖上述现有文件。

## 5. 4090 端代码和启动

核心代码：

- `MemNavData/realworld_cec_hub.py`
- `MemNavData/run_realworld_memnav_server.sh`
- `MemNavData/run_realworld_navdp_server.sh`
- `MemNavData/run_realworld_cec_hub.sh`
- `MemNavData/run_realworld_policy_stack.sh`
- `MemNavData/stop_realworld_policy_stack.sh`

启动：

```bash
cd /home/asus/Research/Nav-graph-blind
MemNavData/run_realworld_policy_stack.sh
curl -fsS http://127.0.0.1:18889/healthz
```

查看：

```bash
tmux attach -t cec-realworld
tail -f .diagnostics/realworld_cec_stack/logs/{memnav,navdp,hub}.log
nvidia-smi
```

停止：

```bash
MemNavData/stop_realworld_policy_stack.sh
```

端口只允许 loopback：

- native NavDP：`127.0.0.1:8888`
- MemNav/CEC：`127.0.0.1:18888`
- unified hub：`127.0.0.1:18889`

## 6. Jetson 端启动顺序

### 6.1 只测隧道，不启动相机或底盘

```bash
cd /home/nvidia/twork/NavDP
tmux new -s cec-tunnel \
  'exec deployment/go2/offboard/run_policy_tunnel.sh'
```

另一个终端：

```bash
curl -fsS http://127.0.0.1:18889/healthz
bash deployment/go2/offboard/preflight_offboard.sh
```

### 6.2 无运动 dry-run

```bash
cd /home/nvidia/twork/NavDP
bash deployment/go2/offboard/run_offboard_stack.sh --with-rviz
```

此命令不传 `--with-go2`，因此不会启动 Unitree bridge；adapter 默认
`enable_on_start=false`，只发布轨迹用于检查。

必须检查：

1. `/navdp/status` 中 server 是 `127.0.0.1:18889`；
2. RGB 与 aligned depth 连续、时间差不超过 0.10 s；
3. `/navdp/trajectory` 是有限的 `(x forward, y left)` 局部路径；
4. 人工遮挡 depth ROI 时状态进入 `obstacle_stop`；
5. 杀掉 tunnel 后，2.5 s 内进入 `trajectory_stale/inference_error`；
6. 没有 `SportClient.Move()` 进程。

### 6.3 首次连接底盘

只有完成上面的静态检查后才允许：

```bash
bash deployment/go2/offboard/stop_offboard_stack.sh
bash deployment/go2/offboard/run_offboard_stack.sh --with-go2 --with-rviz
```

即使启动 bridge，adapter 仍是锁定态。最后的 `set_enabled=true` 属于物理动作，
必须由现场操作者在 Go2 处于宽阔平地、手持遥控器时单独执行；本文件不把它
包装进自动启动脚本。

## 7. 2026-08-18 已完成验证

- Jetson 到 4090 直连 RTT：约 2.3 ms；
- Jetson 已有免密 SSH alias `work-pc → 10.208.2.249`；
- 统一 hub/adapter 单元测试：21 passed；
- MemNav、NavDP、hub 三服务在 4090 同时加载；
- reset 返回 `algo=cec_hybrid_navdp`、certificate enabled；
- 三端口均已验证只监听 loopback；
- reset 后显存约：MemNav 5.2 GiB、NavDP 1.5 GiB；
- 一次真实 multipart 推理成功：首帧 `no_causal_candidate`，正确走
  `navdp_image_router`，返回有限 24 点轨迹；
- 覆盖层已 checksum 一致地同步到 Jetson；offboard preflight 5/5 通过；
- Jetson loopback tunnel 已实际连通 4090 hub；
- 真实 D435i + 禁用态 adapter 无运动 dry-run 成功，使用 CameraInfo 完成 reset，
  连续返回有限 24 点轨迹；
- 20 秒采集 38 个状态：全部 `enabled=false`，0 error、0 非零命令；端到端
  inference p50/p95/max 为 0.638/0.681/0.760 s，RGB-D age p95/max 为
  0.066/0.138 s；该数字受到同机无关 Pi3X 进程 100% GPU 占用影响；
- fault injection：杀 tunnel 后观察到连接错误、plan age 5.086 s、
  `vx=0,wz=0`；恢复 tunnel 后错误清空并重新规划；
- 整个 dry-run 没有启动 `go2_cmd_bridge`，`/navdp/cmd_vel` 订阅数始终为 0；
  验证后已停止 Jetson dry-run。

## 8. 尚未完成与不得夸大的部分

- 尚未在 4090 GPU 独占条件下测长时间 p99；当前有无关 Pi3X 自标定任务占用
  约 19 GiB 并使 GPU 利用率达到 100%；
- 尚未做相机光轴到 Go2 `base_link` 的 bearing 符号校准；
- 尚未启动底盘，不存在真机 SR 结果；
- Flask 仅在 loopback 单客户端模式使用，不能作为公网服务。

## 9. 下一次上电后的最小验收

1. 找回 Jetson 当前 IP，更新本机 `wsj` alias；
2. 核对 `/home/nvidia/twork/NavDP/deployment/go2/offboard` 是否完整；
3. tunnel-only health；
4. `--with-rviz` 无底盘 dry-run 10 分钟，记录 p50/p95/p99 推理延迟；
5. 手持目标图做左/右 bearing 静态符号检查；
6. 人工 kill tunnel，验证 2.5 s/0.35 s 两级 watchdog；
7. 最后才进行 0.5–1.0 m 系绳低速直行测试。
