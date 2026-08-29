# Odin1 独立参考真值栈

## 1. 结论与命名边界

可以为当前 MemNav/NavDP-Go2 系统单独搭一套 Odin1 数据栈，而且这是比继续使用
`SportModeState` 作为距离/SPL来源更干净的方案。它包含：

1. 一次完整往返 Survey 和 Odin mode-1 SLAM；
2. 厂商 `.bin` 地图和独立二维 occupancy 的哈希冻结；
3. D435i ImageGoal RGB 与 Odin 地图目标位姿的同步锚定；
4. 每次正式 run 的 Odin mode-2 重定位；
5. `map -> odom` 重定位门、连续 Odin 里程积分和 D435i 视觉到达融合；
6. 冻结地图上的 A* `L_i`、实际路径 `P_i`、`S_i` 和 SPL；
7. ROS bag、JSONL、结果和A*收据进入现有双视角证据包。

但论文和README中应称它为 **independent Odin1 reference/evaluation
stack** 或“独立参考真值栈”，不能不加限定地声称 motion-capture 级绝对 ground
truth。Odin1本身仍是SLAM：会受纹理、动态物体、回环、打滑观测退化和重定位失败影响。
如果未来接入Vicon、OptiTrack、全站仪或外部AprilTag测量系统，后者才可升级为计量级GT。

机器可读边界见
[`manifests/odin1_gt_reference_v1.json`](../../manifests/odin1_gt_reference_v1.json)。
当前状态是“代码完成、尚未在本轮Go2/Odin硬件上验证”，不能据此填写正式SR/SPL。

## 2. 与导航系统的隔离架构

```text
                          决策/执行链（保持不变）

D435i RGB ──> Jetson/SSH ──> RTX 4090 CEC + NavDP ──> local trajectory
D435i depth ───────────────────────────────> Jetson collision safety
                                                    |
                                                    v
                                           Go2 cmd_vel bridge

                         独立参考真值链（只读）

Odin1 mode 1 ──> .bin map + cloud_slam ──> frozen occupancy ──> A* L_i
      |                    |
      |                    └──> D435i goal SHA + Odin map pose anchor
      |
Odin1 mode 2 ──> stable map->odom ──> map pose / local odom increments ──> P_i
                                      |
D435i RGB arrival gate ──────────────┴──> S_i / arrival receipt ──> SPL

第三人称视频 + RViz/dashboard + NavDP receipts + Odin receipts ──> hash manifest
```

不可违反的隔离条件：

- Odin RGB、点云、位姿、A*路径不得输入CEC、NavDP或Go2控制器；
- D435i仍是唯一策略RGB和本地碰撞深度来源；
- Odin A*只计算评测最短路，不给NavDP发全局子目标；
- Odin GT monitor不发布`cmd_vel`、不清除`estop`、不调用运动服务；
- 若未来让A*路径参与控制，必须作为新的“mapped navigation”方法/消融实验，不能与当前
  Full-Mono结果混写。

## 3. 数据产品与因果关系

每个场景的Survey输出：

| 文件 | 含义 |
| --- | --- |
| `odin_map.bin` | Odin厂商mode-1专有地图，用于mode-2重定位 |
| `map.receipt.json` | `.bin`、驱动配置及SHA收据 |
| `occupancy.pgm/.yaml` | 从`cloud_slam`和Odin odom射线累计得到的冻结二维评测地图 |
| `occupancy.receipt.json` | 高度带、分辨率、点数、地图尺寸和SHA |
| `goal_anchor.draft.json` | 目标点静止时的Odin map pose和D435i目标RGB/depth SHA |
| `goal_anchor.json` | 将目标草稿、`.bin`和occupancy三者封在同一mapping session的收据 |
| `scene_contract.json` | 本场景Odin serial、固件、calibration、driver profile和刚性安装SHA |
| `rosbag/odin_mapping/` | Survey原始Odin话题和TF |

每个正式run输出：

| 文件 | 含义 |
| --- | --- |
| `monitor/status.jsonl` | 5 Hz重定位、位姿、视觉、路径、到达状态 |
| `monitor/result.json` | `S_i`前置结果和Odin积分`P_i` |
| `spl_receipt.json` | 冻结A*设置、`L_i/P_i/S_i/SPL_i`和所有输入SHA |
| `astar_overlay.png` | 人工审查用A*路线叠图，不参与数值计算 |
| `rosbag/odin_formal/` | Odin odom/cloud/path/TF、NavDP状态、命令和评测话题 |

`.bin`是专有重定位地图，不能直接对它运行A*。因此本栈同时冻结一张二维occupancy；
两张地图必须来自同一个Survey并由`goal_anchor.json`绑定，禁止事后替换其中之一。

## 4. 坐标、路径和到达定义

### 4.1 Mapping阶段

厂商mode 1规定地图原点是驱动启动时的Odin odom原点。本栈在目标处读取
`/odin1/odometry`，以`odom == mapping map`记录`goal_pose_map`。目标相机仍是D435i；
目标收据同时保存D435i RGB和可选aligned-depth SHA。

### 4.2 Formal阶段

厂商mode 2在重定位成功后发布`map -> odom`。在TF出现以前，厂商可能处于fallback
SLAM，即使`/odin1/odometry`仍在发布也不能视为已定位。本栈要求：

- `.bin` SHA与目标收据完全一致；
- Odin odom新鲜；
- `map -> odom`至少5个样本、连续稳定2 s；
- 稳定窗平移变化不超过0.15 m、旋转变化不超过5°；
- ready以后出现超阈值TF跳变会永久使该run无效。

只有`/navdp/gt/status`显示`reference_ready=true`以后，操作员才能启动NavDP正式run。

Survey occupancy不使用“回调时最新odom”近似点云原点，而按Odin header stamp选择最近
odom；默认只接受时间差不超过0.10 s的配对，并把拒绝数量和实测最大skew写入
`occupancy.receipt.json`。

### 4.3 `P_i`

`P_i`从ready时刻开始，对连续`/odin1/odometry`的局部xy增量求和：

```text
P_i = sum || odom_xy(k) - odom_xy(k-1) ||_2
```

不能对`map`位姿直接求相邻差，因为回环或重定位修正会把TF跳变错误算成机器人行走距离。
本栈拒绝单步超过0.50 m或推算速度超过2.0 m/s的Odin样本。

### 4.4 `L_i`

`score_odin_gt.py`在冻结occupancy上运行8连通A*：

- unknown一律视为障碍；
- 禁止对角穿墙角；
- 障碍按“实测Go2平面半径 + 冻结inflation margin”膨胀；
- 起点和目标最多只能在0.20 m内吸附到可行栅格，吸附距离计入`L_i`；
- 分辨率、半径、inflation、吸附和地图SHA全部写入SPL收据。

### 4.5 `S_i`与自动到达

当前默认到达组合为：

```text
relocalization_ready
AND Odin map distance <= 0.85 m
AND fresh D435i RGB arrival latch
AND Odin planar speed <= 0.10 m/s
AND all conditions held for 1.0 s
```

这比仅靠电机里程计或仅靠SIFT/RANSAC更稳健：Odin给出独立metric region，D435i 的
纯 RGB arrival gate确认目标画面，静止门避免高速掠过。GT monitor只记录成功，不自动
急停；Go2最终停止仍由 RGB arrival gate、现场操作员和安全链负责，防止GT参考栈意外
获得运动权限。

上述`0.85 m/0.10 m/s/1.0 s`仍是实现默认值，不是已发表标定结果。正式campaign前必须
用`0/0.25/0.5/1.0 m × 0/±10/±20°`物理偏置实验冻结。

### 4.6 SPL

```text
SPL_i = S_i * L_i / max(L_i, P_i)
```

A*只提供`L_i`，不能单独“算SPL”。`P_i`来自正式run的Odin odom积分，`S_i`来自上述
独立融合到达门。失败run的SPL为0，但仍需保存`L_i`和`P_i`以供审计。

## 5. 一次性驱动安装

当前默认是已刷0.14固件后的原生Mode1路径：

- profile：`native_0_14`；
- 官方driver tag：`v0.14.0`；
- pinned commit：`6f993ccc4ccad9395bfc68bc3235f993d83c4fe6`；
- 不应用0.13.1的Mode1冷启动补丁；
- 仅应用runtime-config最小修复（SHA-256
  `953bd96ad3cea5c336f11882f92a428ff090ba13abd28c742314f072cd637f86`），因为官方
  ROS2 launch虽然传了`config_file`，C++仍固定读取源码配置。该修复保证每个session的
  哈希配置确实选择Mode1或Mode2。

TopoFocus曾验证的0.13.1冷启动方案仍以`legacy_0_13_1`兼容profile保留：commit
`13aa528b1da581e2168ac858f8b144f0b4438a7a`，patch SHA-256
`2a73aa48d163e2a362670b7b9b778edf8328aba7323e1cc04dd6b8fb28ba5806`。它不是默认值，
也不得应用到0.14固件。现场操作员已报告0.14下Mode1数据在此前测试中正常；正式栈仍需
为本次传感器、驱动和场景生成硬件收据，不能用口头验证替代。

安装依赖、udev、clone、patch和build：

```bash
cd /home/nvidia/twork/MemNav-RealWorld
bash deployment/odin1_gt/scripts/odin_gt.sh setup --install-deps
```

如果系统依赖和udev已经配置：

```bash
bash deployment/odin1_gt/scripts/odin_gt.sh setup
```

两条命令均默认`native_0_14`。只有传感器确实回刷0.13.1时才允许显式执行：

```bash
bash deployment/odin1_gt/scripts/odin_gt.sh setup \
  --driver-profile legacy_0_13_1
```

安装成功会写`/home/nvidia/twork/odin_ws/.memnav_odin_driver_profile.json`；后续preflight
验证commit、patch和所有被修改驱动文件的SHA，拒绝“源码已变但仍沿用旧build”的状态。

默认workspace：

```text
/home/nvidia/twork/odin_ws
  src/odin_ros_driver
  build/
  install/
```

可用`ODIN_WS`覆盖。脚本不会启动Go2、NavDP或任何速度节点。

当前Jetson同时安装JetPack OpenCV 4.8和ROS `cv_bridge`所依赖的OpenCV 4.5，官方驱动
链接时会出现双版本/TBB警告。编译已经成功，且操作员此前报告0.14 Mode1正常出数；但
`preflight`会持续提示这一ABI风险，必须以接入Odin后的持续RGB/cloud/odom话题测试作为
现场放行条件，不能仅凭“build成功”放行正式Survey。

驱动和重定位行为以厂商官方文档为准：
[Odin ROS Driver](https://github.com/manifoldsdk/odin_ros_driver)、
[Relocalization Guide](https://github.com/manifoldsdk/odin_ros_driver/blob/main/RELOCALIZATION_GUIDE.md)。

## 6. 接线后硬件预检

插入Odin1后先保持Go2下电或`estop=true`：

```bash
bash deployment/odin1_gt/scripts/odin_gt.sh preflight
lsusb -d 2207:0019
```

启动某个Odin session以后可检查实时话题：

```bash
bash deployment/odin1_gt/scripts/preflight.sh --hardware
ros2 topic hz /odin1/image
ros2 topic hz /odin1/cloud_slam
ros2 topic hz /odin1/odometry
```

必须记录：Odin serial、SoC/SLAM firmware、设备专属calibration SHA、USB3速率、刚性安装
ID和Odin到Go2机体的外参。TopoFocus中`O1-P070100205`的校准只能在确认是同一台传感器
且字节SHA一致时复用；绝不能把另一台Odin或另一副安装架的外参直接当作当前Go2标定。

复制`config/go2_odin_mount_receipt.template.json`为运行时文件，填入传感器serial、安装ID、
测量方法、独立验证证据及有限的4×4 `T_go2base_odin`。只有独立复核后才能把
`validated`置为`true`；不要修改Git跟踪的模板。

## 7. 场景Survey：完整往返建图

### 7.1 先量两个强制参数

`cloud_slam`在`odom`坐标系。必须根据当前Odin安装高度/方向量出障碍点高度带：

```text
obstacle_min_z_m = <实测>
obstacle_max_z_m = <实测>
```

脚本不提供默认值，避免把地板或天花板误投成障碍。还要实测Go2平面包络半径，后续A*
通过`--robot-radius`传入。

### 7.2 启动建图数据栈

示例中的数值必须替换为实测值：

```bash
cd /home/nvidia/twork/MemNav-RealWorld
bash deployment/odin1_gt/scripts/odin_gt.sh start-map scene01_survey_v1 \
  --sensor-serial <reported-serial> \
  --firmware-version <exact-0.14.x-version> \
  --calibration-file <absolute-calib.yaml> \
  --mount-receipt <absolute-validated-mount.json> \
  --obstacle-min-z <measured-min-z> \
  --obstacle-max-z <measured-max-z>
```

这会先生成`scene_contract.json`，把serial、精确固件版本、calibration SHA、安装外参SHA和
driver profile SHA绑定到本场景，再启动三个只读窗口：Odin Mode1 driver、occupancy
builder、mapping rosbag。缺少任何字段或安装收据未经验证都会拒绝启动。

### 7.3 人工走完整往返

用Unitree手柄或已验证的人工遥控，以低速沿正式可行区域完成：

```text
A起点 -> 沿主路线到B目标 -> 继续覆盖目标周边 -> 原路/替代路返回A
```

往返比单程更适合触发回环并暴露漂移。至少覆盖：

- 正式起点附近不同朝向；
- 目标前方、左右偏置和后退视角；
- 所有可能通行分叉；
- A-B往返闭环；
- 动态人群尽量清空。

### 7.4 在B处锚定目标

Go2在B处静止。先用现有D435i目标采集脚本保存目标RGB/depth，再立刻绑定Odin位姿：

```bash
bash deployment/odin1_gt/scripts/odin_gt.sh capture-goal scene01_survey_v1 \
  /absolute/path/to/image_goal.png \
  /absolute/path/to/image_goal_depth.png
```

它要求连续20个静止Odin样本，并生成`goal_anchor.draft.json`。之后继续完成回程，不要
重启Odin driver；否则mapping原点会改变。

### 7.5 保存并封图

完成回程并静止：

```bash
bash deployment/odin1_gt/scripts/odin_gt.sh finish-map scene01_survey_v1
```

命令触发厂商`save_map=1`，等待`.bin`大小稳定，优雅停止bag/builder/driver，并把目标
草稿封成`goal_anchor.json`。输出默认位于：

```text
runtime/odin1_gt/maps/scene01_survey_v1/
```

封图后禁止修改。任何补图都必须使用新session ID，重新产生整套SHA。

## 8. 正式run完整命令顺序

假设现有RTX/Jetson NavDP栈已按主手册配置，但运动仍保持`enabled=false, estop=true`。

### 8.1 先启动Odin参考栈

```bash
bash deployment/odin1_gt/scripts/odin_gt.sh start-formal \
  scene01_formal_01 \
  runtime/odin1_gt/maps/scene01_survey_v1/goal_anchor.json

bash deployment/odin1_gt/scripts/odin_gt.sh wait-ready scene01_formal_01 120
```

`wait-ready`成功前禁止启动正式运动。厂商建议重定位尽量从原建图轨迹约1 m、±10°内
开始，但实际容差高度依赖场景；失败时Odin可能继续发布fallback SLAM odom，本栈不会
放行。

### 8.2 启动现有证据包

```bash
bash deployment/go2/offboard/experiment_capture.sh start scene01_formal_01 \
  --dataset scene01_survey_v1 \
  --trial-kind revisit \
  --profile audit \
  --gt-source odin1
```

此时ROS bag同时记录NavDP、CEC、D435 RGB arrival、Odin odom/cloud/TF和
`/navdp/gt/status`，桌面继续录RViz dashboard；第三人称相机同步开录。

### 8.3 再启动NavDP正式episode

按照主手册执行`formal-start`、确认goal SHA、清急停、enable。Odin命令不替代任何现有
安全步骤。运行中观察：

```bash
bash deployment/odin1_gt/scripts/odin_gt.sh status formal scene01_formal_01
ros2 topic echo /navdp/gt/status
```

关键字段：

- `reference_ready=true`；
- `relocalization_evidence.invalid_reason=""`；
- `odometry.invalid_reason=""`；
- `distance_to_goal_m`连续合理；
- `rgb_arrival.latched`只在纯 RGB 到达模块已锁存时为true；
- `arrival.success=true`后仍由现有安全链/操作员确认停止。

### 8.4 结束、评分和封包

先让Go2停止并置急停，再停止Odin参考栈：

```bash
bash deployment/odin1_gt/scripts/odin_gt.sh stop-formal scene01_formal_01
bash deployment/go2/offboard/experiment_capture.sh stop scene01_formal_01
```

用实测Go2包络半径计算SPL：

```bash
bash deployment/odin1_gt/scripts/odin_gt.sh score scene01_formal_01 \
  --robot-radius <measured-planar-radius> \
  --inflation-margin 0.05
```

把第三人称视频、GT结果和SPL收据附到同一证据包：

```bash
bash deployment/go2/offboard/experiment_capture.sh attach-third-view \
  scene01_formal_01 /path/to/third_view.mp4

bash deployment/go2/offboard/experiment_capture.sh attach-odin-gt \
  scene01_formal_01 \
  runtime/odin1_gt/formal/scene01_formal_01/monitor/result.json \
  runtime/odin1_gt/formal/scene01_formal_01/spl_receipt.json

bash deployment/go2/offboard/experiment_capture.sh finalize \
  scene01_formal_01 success --notes "Odin1 independent reference lane"

bash deployment/go2/offboard/experiment_capture.sh verify scene01_formal_01
```

当`--gt-source odin1`时，缺少`odin_gt_status.jsonl`、GT result或SPL receipt都会拒绝
正式finalize；不能用手填SPL绕过。

## 9. RViz与现场观察

现有NavDP RViz继续负责D435i画面、局部轨迹、CEC和控制状态。Odin调试建议另开窗口：

```bash
source /opt/ros/humble/setup.bash
source /home/nvidia/twork/odin_ws/install/setup.bash
rviz2 -d deployment/odin1_gt/config/odin_gt.rviz
```

Formal时Fixed Frame使用`map`，检查：

- `/odin1/cloud_slam`能通过`map -> odom`稳定显示；
- `/odin1/odometry`和`/odin1/path`没有突跳；
- TF树存在`map -> odom -> odin1_base_link`；
- NavDP局部轨迹仍在机器人局部坐标，不与Odin A*路线混作控制输出。

桌面录制会捕获整个VNC display；若并排展示两个RViz窗口，应在正式campaign前冻结布局，
20次run不要临时改窗口尺寸或topic配置。

## 10. 正式campaign前的P0验证

### 10.1 传感器与安装

- 同一Odin serial重复插拔后calibration SHA不变；
- USB3稳定，RGB/cloud/odom约10 Hz且无长期丢帧；
- 双OpenCV依赖下驱动持续运行无崩溃、图像解码错误或点云异常；
- 刚性安装无可见松动，外参和安装照片有SHA收据；
- D435i与Odin时间均在同一Jetson记录，保存各自header stamp和host receipt time。

### 10.2 地图

- 每场景至少两次独立往返建图比较闭环和占据边界；
- 用卷尺/地标核对2 m、5 m和整段A-B尺度；
- 固定障碍高度带不会把地面误占据；
- 固定Go2 footprint/inflation后A*路线符合真实可通行区域；
- 地图、occupancy、目标收据SHA冻结后再排20次正式实验。

### 10.3 重定位

- 完整stop/restart后在A点至少10次mode-2重定位；
- 在目标B及A附近`0/0.25/0.5/1.0 m × 0/±10/±20°`测试；
- 统计首次TF时间、成功率、TF跳变和fallback时长；
- 无`map -> odom`时确认`reference_ready`始终false；
- ready后人为遮挡/快速转动，确认超阈值TF跳变使run无效。

### 10.4 路径和到达

- 用卷尺标记直线1/2/5 m，核对Odin积分误差；
- 原地转向时`P_i`不应明显增长；
- 腿部踏步/打滑时记录Odin与第三人称差异；
- 到达偏置标定后冻结metric radius、速度和hold；
- D435误识别、Odin近目标但视觉错误、视觉正确但Odin远离三类负例均不得成功。

## 11. 必须判无效或失败的情况

| 情况 | 处理 |
| --- | --- |
| `.bin` SHA与goal receipt不一致 | 禁止启动formal GT |
| 没有`map -> odom` | 视为fallback SLAM，禁止开始episode |
| ready后TF大跳 | GT result无效，run记system failure |
| Odin odom超时/单步跳变 | 停止正式实验，不能计算可信`P_i` |
| D435视觉话题过期 | 不允许arrival success |
| occupancy或PGM SHA变化 | A*评分拒绝 |
| A*无路径或吸附超0.20 m | 场景地图/起终点定义无效，不能手填`L_i` |
| Odin进入NavDP/CEC输入 | 方法污染，该run不能计入当前Full-Mono结果 |
| GT monitor自动发速度/清急停 | 架构违规；当前实现明确没有这些publisher |

## 12. 当前尚未完成的现场事实

截至2026-08-28本轮代码完成时：

- Jetson上ROS 2 Humble、D435i和现有NavDP栈可用；
- 官方`v0.14.0`原生Mode1 driver已在`/home/nvidia/twork/odin_ws`编译并通过无硬件
  preflight；
- 操作员报告当前0.14固件下Mode1此前已经正常出数；本机也保存TopoFocus的0.13.1历史
  补丁和校准经验，但旧补丁不再作为默认方案；
- 当前`lsusb`没有检测到`2207:0019`，ROS图也没有Odin节点；
- 因此尚未验证当前Go2上的serial、安装外参、高度带、重定位率、到达阈值和路径误差；
- `manifests/odin1_gt_reference_v1.json`中的现场冻结字段保持`null`是正确状态。

下一次现场工作的正确顺序是：插入Odin、只读硬件预检、测安装参数、做一个debug
往返Survey、反复重定位和物理偏置标定，最后才冻结四场景并开始`4 × 5`正式run。
