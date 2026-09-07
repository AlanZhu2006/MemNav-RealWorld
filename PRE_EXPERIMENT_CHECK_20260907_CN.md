# 实验前兼容性检查与准备提速（2026-09-07）

本次为代码、配置、依赖和只读设备检查；未启动导航栈，未解除急停，未启动运动。

## 发现与处理

| 项目 | 结果 |
|---|---|
| 双机版本不一致 | Jetson 原为 `aeca0fa`，GPU 原为 `95afe64` 加本地修改，会被 resolved config 的 revision 校验拒绝。已备份并将 GPU 快进到 `aeca0fa`，保留两机本地工作。 |
| 新旧 Hub 接口 | 新客户端要求 query-observation、v3 terminal handoff。两机源码已对齐；保留原有 schema、terminal mode 检查，补上配置与 Hub 的历史深度来源一致性检查。 |
| 重复锁定等待 | 原流程反复启动 `ros2 service call`、`ros2 topic pub/echo`，部分调用还忽略失败。改为一个 DDS 节点完成单向 operator_stop，并等待响应之后的新鲜 disabled/estop/zero 状态；合并重复的阶段入口锁定。 |
| 残留服务导致重复加载模型 | 新 Survey 清理上轮残留 Full-Mono 服务时改用 park，保留兼容 GPU 权重，清空 Episode 状态。独立正式运行仍回放 sealed Survey 并初始化 query-start FIFO。 |
| 启动状态确认 | Adapter 启动等待现在明确检查 enabled=false、estop=true，收到任意 status 不再直接当作就绪。 |
| 准备耗时不透明 | 每个新 formal run 写入 `preparation_timing.json`，分别记录服务启动、Survey 回放、目标准备时间；总时间字段明确从配置生成后计起。 |
| Agent 重复确认/额外诊断 | AGENTS 明确直接 Revisit 请求即进入既有监督流程，不再次索要授权口令，不给每次运行加代码审计或独立诊断栈。保留实际运行必需的自动门槛。 |
| 实验登记说明过时 | AGENTS 不再将 pair_001 写成“当前唯一有效 pair”，以现有 index.json 为准。未修改任何 pair 登记、数据或历史结果。 |

## 实际验证

- Jetson 与 GPU 均为 `aeca0facb6c56b10474211d6a54e22a665a7317a`；190 个 deployment/NavDP/AGENTS 文件 SHA 比对一致，包含本地补丁与新工具。
- Python 语法 20 个文件、Shell 语法 12 个文件、Shell 内嵌 Python 27 段、JSON 3 个文件检查通过；Git 无冲突、diff whitespace 检查通过。
- Jetson 实际 ROS/venv 环境导入 adapter、navigation runner、operator service 与新锁定工具通过。
- GPU 实际 Python 环境的 Hub 导入及 CLI 参数检查通过。外部 MemNav 接口、当前帧深度复用补丁、深度运行时一致性、模型文件和依赖路径通过现有 preflight。
- GPU preflight 有两项端口占用：18888、8888 为既有 parked 模型服务占用；18889 空闲。未为了让端口检查通过而停止原进程。正常 launcher 会检查受管 parked 状态和模型签名后处理重绑定。
- 使用现存 Episode 的真实 goal，只生成审计用新 resolved config 并完成两端 verify：`b29e99d08c3b312b8c759d11f2f0b6aaeb5afac67e7d4bb5b3ad64cb993aee83`。未将其安装为 formal run，未修改历史 immutable config。
- D435i `344422071135`，固件 5.17.0.10，USB 5000M；相机观察服务运行。
- 现有 capture readiness 返回成功，约 2.837 秒：RGB、aligned depth、battery 发布者存在，Foxglove WebSocket 协议通过。发布者存在不代表 Go2 有实时反馈。
- Go2 `192.168.123.161` 当时 ping 无响应。因此没有执行导航前最终 live arming 检查。
- GPU 原 resident PIDs 950984 / 950997 保持不变，状态 parked，memory_frames=0、queue_lengths=[0]、depth_transactions=0。
- 遵照仓库规则，未创建或运行单元测试；新锁定工具尚未对运行中的导航 adapter 做实测，提速幅度尚未做整轮对比。

## 下一次实验的实际边界

- 机器人当前不通，连接恢复后由正式流程自动检查；不能跳过该故障直接启用运动。
- 现有 resident 进程的模型签名与新源码不同。第一次正常启动会冷加载受管旧模型；后续代码/模型契约不变时复用。当前未重启服务。
- 旧 Episode 已 stopped，debug 状态中的 formal_ready 是历史记录，不等于当前已准备好导航。后续按用户选择的新 Episode/新 run ID 运行，不自动重启旧轮次。
- 默认仍为 bearing_only；低 critic 搜索仍关闭。纯 yaw 时 LingBot 平移漂移、运动中滚动换轨与近目标闭环没有在本次得到物理验证。
- 新增历史深度复用减少历史 anchor 几何重放，不取消独立实验的 sealed Survey RGB 回放；不能承诺每轮零等待。

## 备份

- Jetson 拉取前完整工作：`/home/unitree/.local/share/memnav/git-backups/main-update-20260907T093947`，以及保留的 Git stash。
- GPU 拉取前完整工作：`/home/asus/.local/share/memnav/git-backups/pre-experiment-sync-20260907T123445`，以及保留的 Git stash。
- GPU 同步本地修正前文件：`/home/asus/.local/share/memnav/git-backups/compatibility-fixes-20260907T123837`。

未删除 runtime 数据，未修改控制速度、到达阈值、功耗或原有实验标签。后续提交同步不代表已重启服务或完成运动验证。

## 13:48 START SURVEY 失败补记

- Episode `episode_20260907T054815_261035Z` 在 GPU 冷启动阶段失败，尚未开始录制。
- 日志顺序为：旧 parked 模型签名变化 → 关闭 tmux → 立即执行 GPU preflight → `port already in use: 18888`。其余依赖检查通过；事后检查该端口及旧模型进程均已退出，与关闭会话后进程尚未释放监听端口的竞态一致。
- 修正冷启动替换和显式停止路径：关闭自己的会话后，每 0.1 秒观察一次监听端口，端口释放即继续，最多等 30 秒；不结束不明端口占用进程，不固定睡满 30 秒。
- 保留冻结目标和失败日志；本次诊断不自动重跑 Survey 或启动运动。
