# Go2 真机栈：模块与入口

原生 NavDP 的日常监督运行只使用一个入口：

```bash
bash deployment/go2/nav_stack.sh run
```

它默认读取 `native_imagegoal.json`；`run` 是显式运动授权，而 `start` 只启动锁止栈。
运行行为不再由 `.env` 或 `NAVDP_*`/`CEC_*` 环境变量拼装。机器、
模型、端口和安全值在 `deployment/config/system.json`；本次实验选择在
`deployment/config/experiments/*.json`。

## 调用层级

```text
experiment.json + system.json
             │
             └─ runtime_config.py
                    │  严格校验、ImageGoal SHA、config_id
                    ▼
              resolved JSON（唯一运行合同）
                    │
          nav_stack.sh
            ├─ native start ─> scripts/run_stack.sh（锁止栈）
            ├─ native run ───> 健康合同则复用，否则先 start
            │                     └─ navigation_run_agent.py
            │                         lock → reset → plan → arm → monitor
            └─ Full-Mono -> offboard/fullmono.sh
                                ├─ 同一 JSON -> RTX policy stack
                                └─ 同一 JSON -> Jetson offboard stack
```

`run_stack.sh`、`run_offboard_stack.sh` 和 GPU `run_*.sh` 是内部组合/叶子层；
它们只接受 `--config RESOLVED.json`，不再是人工调参入口。

## 两套导航与独立到达模块

| Profile | 策略输入 | 记忆 | 运行位置 |
| --- | --- | --- | --- |
| `native-navdp-rgbd` | 当前 RGB + D435 aligned depth + ImageGoal | 无 | Jetson |
| `fullmono-lingbot-cec` | causal RGB + LingBot mono depth + ImageGoal | CEC | Jetson + RTX 4090 |

到达模块与导航解耦：`operator`、`external-topic` 或纯 RGB
`rgb-homography`。原生 NavDP 只产生局部轨迹，没有可靠的原生 STOP 动作；到达模块
只终止 episode，不参与路径生成。

## 配置、状态和停止

```bash
bash deployment/go2/nav_stack.sh list
bash deployment/go2/nav_stack.sh resolve \
  --config deployment/config/experiments/native_imagegoal.json
bash deployment/go2/nav_stack.sh start \
  --config deployment/config/experiments/native_imagegoal.json --dry-run
bash deployment/go2/nav_stack.sh run
bash deployment/go2/nav_stack.sh status
bash deployment/go2/nav_stack.sh stop
```

启动会输出最终 ImageGoal 绝对路径、图像 SHA-256、源码 revision 和 config ID。
Full-Mono 还会把同一字节序列复制到 4090；两端 revision 或 config ID 不一致就拒绝
启动。

`start` 成功仍是 `disabled + estop`。`run` 只供现场监督运行：该命令自动验证 reset 后
的新轨迹并执行两步授权，同时输出各阶段耗时；任何失败、超时或中断都调用单向
`operator_stop`。Foxglove仍不暴露 reset、clear-estop 或 enable。
