# Go2 真机栈：模块与入口

日常只使用一个入口和一个配置参数：

```bash
bash deployment/go2/nav_stack.sh start \
  --config deployment/config/experiments/native_imagegoal.json
```

启动行为不再由命令行选项、`.env` 或 `NAVDP_*`/`CEC_*` 环境变量拼装。机器、
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
            ├─ native ───> scripts/run_stack.sh（Jetson）
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
bash deployment/go2/nav_stack.sh status
bash deployment/go2/nav_stack.sh stop
```

启动会输出最终 ImageGoal 绝对路径、图像 SHA-256、源码 revision 和 config ID。
Full-Mono 还会把同一字节序列复制到 4090；两端 revision 或 config ID 不一致就拒绝
启动。

启动成功仍是 `disabled + estop`。只有现场操作员检查画面、到达模块和遥控接管后，
才能显式解除急停并调用 `/navdp_go2_adapter/set_enabled`。
