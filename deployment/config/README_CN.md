# 运行配置

`system.json` 是两台机器、模型、端口、相机、速度和安全参数的唯一受 Git
管理的站点配置。`experiments/*.json` 只描述一次实验选择：栈 profile、
ImageGoal、到达模块和要启动的可选进程。

不要再创建 `deployment/gpu/.env`，也不要通过 `CEC_*`、`NAVDP_*` 环境变量
覆盖实验。正式入口只接收一个实验配置：

```bash
deployment/go2/nav_stack.sh start \
  --config deployment/config/experiments/native_imagegoal.json
```

启动前，入口会把两层配置解析成 `runtime/config/<config_id>.json`。解析结果
包含绝对 ImageGoal 路径、图像尺寸、文件 SHA-256、源码 Git revision 和完整
两机参数。Full-Mono 会把这一份完全相同的文件复制到 4090，再由两端分别
校验 `config_id`；因此不会出现 SSH 或 tmux 少传某个变量的情况。

代码提交后不要单独用新版本重启某个 tmux 窗口。再次执行同一个 `start` 命令时，
入口会先锁停并停止已有整栈，再用当前 revision 对应的新 resolved 配置统一启动；
正常的相机恢复仍在同一份运行合同内只重启 `rgbd`。`status` 中
`contract=current|stale` 用于提示运行会话是否仍对应当前源码。

要换 ImageGoal，只修改（或复制）实验 JSON 里的
`experiment.navigation.image_goal`。到达检测可独立使用另一张图，路径在
`experiment.arrival.image_goal`。

`experiment.authority_mode` 固定普通 Full-Mono 的 `cec|native` 权限边界。正式
paired run 仍必须通过 `formal-start --arm mono_native|mono_cec` 显式选择；脚本会把
该选择写入派生配置并和 frozen goal/dataset SHA 一起校验。
