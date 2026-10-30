# 基于位姿图优化 SLAM 与 Transformer 轨迹预测的 Gazebo 自动泊车辅助系统

在 Gazebo 城市一角搭建露天停车场，用一台阿克曼转向乘用车跑通完整的自动泊车辅助（APA）闭环：
巡场建图与定位 → 行人/车辆轨迹预测与风险评估 → 空闲车位选择 → Hybrid A* 泊车轨迹规划 → 跟踪控制执行。

系统由 8 个 ROS 2 包组成，模块之间只通过标准 ROS 2 话题通信。仓库内所有仿真世界、模型、配置、
标定数据与调试脚本齐备，克隆后按本文档步骤即可复现。

---

## 项目介绍

### 要解决的问题

真实停车场里，自动泊车要同时回答三件事：我在哪、别人要往哪去、我该怎么揉进车位。这三件事分别对应
定位与建图、动态目标预测、路径规划与控制。本项目把三者放进同一个仿真闭环，让规划层能真正消费
定位结果和风险等级，而不是各做各的。

### 核心实现思路

整体分四层，数据自下而上流动，控制指令自上而下回流。

| 层 | 负责内容 | 关键实现 |
|---|---|---|
| 仿真层 | 场景、车辆、传感器、动态元素 | Gazebo Harmonic + ros_gz 桥接，阿克曼 `AckermannSteering` 插件 |
| 建图定位层 | 停车场建图与实时位姿 | `slam_toolbox`（Karto 前端 + SPA 位姿图后端，Ceres 求解），`tf_bridge` 复合 `map→odom` 与 `odom→base_footprint`，20 Hz 输出 `/vehicle_pose` |
| 预测与风险层 | 多目标跟踪、轨迹预测、风险分级 | 激光聚类跟踪 → 轻量 Transformer（2 层 Encoder，`d_model=64`，4 头注意力）预测未来 5 步 → 预测轨迹与泊车走廊做时空重叠检测，输出 `safe/warning/danger` |
| 规划控制层 | 选位、规划、执行 | 激光自由空间证据判定车位占用 → Hybrid A*（7 圆车体模型 + 测地距离场启发 + Reeds-Shepp 解析收尾）→ 曲率前馈加横向/航向反馈的跟踪律 |

### 几个关键的工程决定

**空闲车位不靠地面漆线判断。** 2D 激光看不到贴地 1 cm 的漆线，因此车位几何取自先验地图，
占用与否用射线证据判断：从自车向车位入口内侧打一条射线，全程不撞障碍即判空闲，撞在车位内部
判为占用，被车位外的邻车挡住则判为不可见（不做结论）。

**规划转弯半径按实测等效转角算，不按理论限位。** Gazebo 的 `AckermannSteering` 满舵时实际前轮
转角约 0.49 rad，比理论限位 0.61 rad 小约 20%。若按理论值规划，圆弧画得太小，车走不出来，
会系统性切内弯压上邻车。规划器统一按实测等效转角 0.44 rad（R = 5.74 m）计算。

**平行车位需要多点揉库。** 平行位深度只有 2.4 m，最小转弯半径 5.74 m，一把入库在几何上不成立，
规划器会给出一到两次换挡的揉库解；控制侧对连续换挡做了换挡点门限与参考点复位，避免换挡瞬间
航向误差被放大。

**动态目标让行与脱困分开处理。** 场景里有 2 台循环行驶的干扰车和 5 组行人。被挡时先把
「等别人让路」和「自己几何卡死」区分开：前者冻结等待（不消耗重规划预算），后者才触发重规划、
提前交接或倒车脱困。

### 最终达成效果

系统支持全部 42 个车位作为目标（场内垂直 W/E 各 10、场内平行 P 4、南侧道路 B 6、西侧街 SW 6、
东侧街 SE 6）。下表是三类场景的无界面仿真实测数据。

| 场景 | 位置误差 | 航向误差 | 整段最小车体间隙 | 备注 |
|---|---|---|---|---|
| 场内垂直车位 | 0.23–0.30 m | 2.9–10.0° | ≥ 0.20 m | 连续 4 次无碰撞 |
| 路侧平行车位（SW 排） | 0.15–0.30 m | 0.9–8.5° | ≥ 0.20 m | 一把或一次换挡 |
| 南侧道路平行车位（B 排） | 0.10–0.22 m | 0.1–6.0° | 0.25–2.22 m | 两侧均有停放车 |

其它指标：

| 指标 | 要求 | 实测 |
|---|---|---|
| Transformer 轨迹预测 ADE（5 步） | < 0.4 m | 0.319 m |
| 全局路线可行性（曲率 + 不穿设施） | 全部可行 | 104 条（8 起点 × 13 目标）全部通过 |
| 局部规划可解率 | 各类型车位可解 | 15/15 |
| 建图与定位输出 | `/map` + `/vehicle_pose` | 0.05 m 栅格地图，20 Hz 位姿 |

验收判据：位置误差 ≤ 0.35 m、航向误差 ≤ 9°，且整段入位过程车体到障碍的最小间隙 ≥ 0.20 m。
不满足时状态标记为 `INCOMPLETE`，不会谎报成功。

---

## 仿真环境说明

**场景。** 城市路口一角：主干道、东西两条侧街、南侧道路，中间围出一块露天停车场。停车场内
有垂直车位两排（W/E）、平行车位一排（P）、单向环形过道与立柱；路边另有南侧道路与东西街沿街
车位。全场共 42 个可用车位，几何先验写在 `src/parking_planner/config/lot_layout.yaml`。

**车辆。** 4.35 m × 1.85 m 乘用车，轴距 2.7 m，前轮转向后轮驱动的阿克曼模型，最大前轮转角
限位 0.61 rad。车上带 2D 激光雷达（`/scan`）、IMU（`/imu`）、轮式里程计（`/odom`），
模型里也挂了单目相机，但当前链路未使用相机数据。

**动态元素。** 2 台沿全地图闭环航点行驶的干扰车（橙、蓝），5 组沿人行道/斑马线/停车场环线
行走的行人，3 台停在 B 排的静止车辆。干扰车与行人都由 `scene_agents` 包驱动，行为是脚本化的
航点跟踪加让行策略，不具备学习能力。

**坐标系与地图。** `map` 帧来自 SLAM，`odom` 帧来自轮式里程计，自车出生点在主干道上
（`spawn_x/spawn_y/spawn_yaw` 可改）。仓库内附带一张预建栅格地图 `parking_map.pgm`，
用于不跑建图也能直接定位的场景。

---

## 环境配置

### 1. 系统要求

Ubuntu 24.04 LTS（WSL2 亦可），ROS 2 Jazzy，Gazebo Harmonic，Python 3.12。
整套流程在 WSL2 + 无界面 Gazebo 下验证通过，内存建议 ≥ 8 GB。

### 2. 获取代码

```bash
git clone https://github.com/IcekyPrime/apa-parking-gazebo.git
cd apa-parking-gazebo
```

仓库根目录就是本文档所在目录，下文的 `src/`、`training/`、`scripts/`、`docs/` 均指这里的子目录。

### 3. 安装 ROS 2 Jazzy 与 Gazebo

```bash
# ROS 2 apt 源
sudo apt update && sudo apt install -y curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install -y ros-jazzy-desktop ros-dev-tools

# 仿真、SLAM、可视化
sudo apt install -y ros-jazzy-ros-gz ros-jazzy-slam-toolbox ros-jazzy-rviz2

# 数值与配置依赖（多数随 desktop 版已装）
sudo apt install -y python3-numpy python3-scipy python3-yaml
```

### 4. 安装预测模型的运行时

轨迹预测节点需要 PyTorch。CPU 版即可满足推理与训练需求：

```bash
pip install --break-system-packages torch \
  --index-url https://download.pytorch.org/whl/cpu
```

### 5. 搭建工作区并编译

工作区约定为 `~/apa_ws`，`training/` 必须与 `src/` 同级（训练脚本按相对位置定位模型定义）：

```bash
mkdir -p ~/apa_ws
cp -r src ~/apa_ws/          # -> ~/apa_ws/src
cp -r training ~/apa_ws/     # -> ~/apa_ws/training
bash scripts/build.sh        # colcon build --symlink-install，共 8 个包
```

编译完成后 `~/apa_ws/install` 下应出现：`parking_lot_city`、`parking_slam`、
`prediction_interfaces`、`detection_tracking`、`trajectory_prediction`、`risk_assessment`、
`scene_agents`、`parking_planner`。

### 6. 训练轨迹预测模型（首次运行需要）

仓库不附带模型权重，未训练时预测节点会告警并退化为不输出预测，泊车流程仍可运行。

```bash
cd ~/apa_ws/training
python3 generate_dataset.py     # 生成 4900 条合成轨迹样本
python3 train.py                # CPU 约 3 分钟，验证集 ADE ≈ 0.32 m
```

权重输出到 `~/apa_ws/training/checkpoints/trajectory_transformer.pt`，与启动文件里的默认
`model_path` 一致。

---

## 项目配置

### 启动前置配置

WSL2 下 Fast-DDS 默认走共享内存，残留会表现为「进程活着但一个话题都不发、卡在 `rcl_node_init`」。
项目已内置两项处理，启动脚本会自动带上：

- 使用 `src/parking_planner/config/fastdds_udp_only.xml`，强制 UDP 回环传输；
- 每次启动前执行 `scripts/clean_env.sh` 清理残留的 Gazebo 与 ROS 进程。

手工启动时需要自行设置：

```bash
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export FASTRTPS_DEFAULT_PROFILES_FILE=\
~/apa_ws/install/parking_planner/share/parking_planner/config/fastdds_udp_only.xml
```

### 资源加载规则

- 世界文件与模型通过包共享目录定位（`get_package_share_directory`），不依赖任何本机绝对路径，
  仓库放在哪个目录都能跑。
- 世界里的停放车、干扰车、行人都是 `<include>` 引入 `model://` 的实体，Gazebo 从
  `parking_lot_city` 的 `share` 目录解析模型；桥接配置同理由共享目录读取。
- 车位几何、车道图、各车位的预备位姿分别来自 `lot_layout.yaml`、`route_graph.yaml`、
  `staging_poses.yaml`，改场景时必须同步改这三份配置（`scripts/check_layout.sh` 会做一致性校验）。
- 注意：B 排（南侧道路）6 个车位的中心是**手工标定**的，按两侧停放车的真实中点和端部余量取值，
  与 `build_world.py` 里按等距网格画的标线并不重合。因此 `check_layout.sh` 会对 B 排报
  「几何不一致」，这是已知项，不影响运行；不要用 `python3 -m parking_planner.make_layout`
  重新生成布局，那会用标线网格覆盖这组手工值，B 排会重新出现擦邻车。

### 输出位置

| 文件 | 内容 |
|---|---|
| `~/parking_eval.csv` | 每次泊入一行：终态、车位、行驶距离、路径长度、终点误差、规划耗时 |
| `~/parking_trace.csv` | 逐拍轨迹与内部量，用于复现执行细节 |
| `~/apa_ws/logs/ate.csv` | 定位误差评估（`slam_eval` 节点输出） |

---

## 运行与效果验证

### 一键运行

```bash
bash scripts/run_demo.sh
```

默认行为：从主干道出生点出发，自动选空闲车位 → 规划 → 泊入，无界面运行。常用变体：

```bash
GOAL=W5 bash scripts/run_demo.sh           # 指定车位（W0-W9/E0-E9/P0-P3/B0-B5/SW0-SW5/SE0-SE5）
bash scripts/run_demo.sh                   # 不指定则自动选位
SX=-70 bash scripts/run_demo.sh            # 改主路出生位置
HEADLESS=false RVIZ=true bash scripts/run_demo.sh   # 需要界面时
PARKER=true bash scripts/run_demo.sh       # 让干扰车恒速抢停 E8，测试动态避障与改选
PREDICT=false bash scripts/run_demo.sh     # 关闭预测与风险评估
```

等价的手工启动：

```bash
ros2 launch parking_planner parking_demo.launch.py headless:=true use_rviz:=false
```

### 结果判读

```bash
cat ~/parking_eval.csv                     # 每次泊入的结果行
ros2 topic echo /parking_status            # 状态机：SELECT_SPOT → ROUTE → ALIGN → FOLLOW → DONE
ros2 topic echo /target_spot               # 当前选中的车位
```

正常一轮的日志会依次出现「选定车位」「生成全局路线」「泊车规划成功」「对齐完成」「泊车完成」。
其中「泊车完成」一行同时给出终点误差与整段最小车体间隙，`INCOMPLETE` 表示未达验收线。

### 逐车位回归

```bash
LIST="W1 W4 W5" bash scripts/batch_real.sh          # 指定若干车位逐个跑
LIST="B1 B3 B5" TMO=900 bash scripts/batch_real.sh  # 平行位建议放宽超时
cat /tmp/real_batch.csv                             # 汇总结果
```

每个车位会先硬清场再启仿真，出现评测行即收工，结果追加到 CSV。

### 离线回归（不启 Gazebo）

这些脚本要 import `parking_planner`，先 source 一次工作区：

```bash
source ~/apa_ws/install/setup.bash

python3 scripts/verify_park_sim.py B3      # 交接位姿 → 规划 → 标定运动学模型闭环
bash scripts/check_layout.sh               # 生成物一致性 + 几何/连通性检查
python3 scripts/check_goal_gap.py all      # 逐车位的目标位姿可停性
```

`verify_park_sim.py` 会输出每个车位的路径长度、换挡次数、终点误差与执行轨迹的真实车体间隙，
适合在改代码后快速确认没有回归。

---

## 可调参数说明

### 启动参数（`parking_demo.launch.py`）

| 参数 | 默认 | 作用 | 常用范围 |
|---|---|---|---|
| `goal_stall` | 空 | 指定目标车位 ID，空则自动选位 | W/E/P/B/SW/SE 系列编号 |
| `stall_region` | 空 | 限定选位区域，如 `lot`、`road_south` | 见 `lot_layout.yaml` 的 `region` |
| `prefer_stalls` | 空 | 优先车位列表（逗号分隔） | 任意车位 ID |
| `spawn_x` / `spawn_y` / `spawn_yaw` | −45.0 / 15.65 / 0.0 | 自车出生位姿 | 主干道上任意合法位置 |
| `headless` | false | 无界面运行 | `true` 更适合批量回归 |
| `use_rviz` | true | 是否启动 RViz2 | 录屏时开 |
| `drive_patrol` | false | 是否先跑巡场建图 | 需要新地图时置 `true` |
| `use_prediction` | true | 是否启用预测与风险评估 | 排查时置 `false` |
| `use_traffic` | true | 干扰车开关 | 批量回归可置 `false` |
| `use_peds` | true | 行人开关 | 批量回归可置 `false` |
| `use_parker` | false | 抢位干扰车开关 | 测动态改选时置 `true` |
| `max_speed` | 0.45 | 入位段限速（m/s） | 0.25–0.6 |
| `model_path` | `~/apa_ws/training/checkpoints/trajectory_transformer.pt` | 预测模型权重 | 任意绝对/相对路径 |
| `eval_csv` | `~/parking_eval.csv` | 评测结果落盘位置 | 任意可写路径 |

### 规划与控制节点参数（`planner_node`）

| 参数 | 默认 | 作用 | 常用范围 |
|---|---|---|---|
| `route_speed` | 1.2 | 巡场路线巡航速度（m/s） | 0.8–1.5 |
| `max_speed` | 0.45 | 入位段速度上限（m/s） | 0.3–0.6 |
| `control_rate` | 20.0 | 控制频率（Hz） | 10–30 |
| `xy_res` | 0.25 | Hybrid A* 栅格分辨率（m） | 0.15–0.3，越小越慢 |
| `th_res_deg` | 3.0 | 航向离散分辨率（°） | 2–5 |
| `local_half_size` | 15.0 | 局部规划窗口半宽（m） | 10–20 |
| `plan_clearance` | 0.15 | 栅格搜索时的车体膨胀余量（m） | 0.1–0.3 |
| `plan_min_clear` | 0.30 | 选解的绝对余量门限（m），低于则重搜 | 0.2–0.5 |
| `plan_max_steer` | 0.42 | 规划用等效前轮转角（rad） | 0.40–0.44，超过实测值会切内弯 |
| `staging_along` | 9.0 | 在线重算预备位姿的纵向偏置（m） | 7–11 |
| `anchor_yaw_tol` | 4.0 | 预备位姿锚定的航向门限（°） | 3–6 |
| `park_side_gap` | 0.12 | 入位段贴障停车门限（m） | 0.10–0.20 |
| `ped_yield` | 2.5 | 巡航段行人让行半径（m） | 2–4 |
| `ped_yield_park` | 1.5 | 入位段行人让行半径（m） | 1.0–2.0 |
| `guard_ahead` | 4.5 | 激光护栏前视距离（m） | 3.5–5.5 |
| `replan_tolerance` | 0.8 | 触发重规划的横向偏差门限（m） | 0.5–1.0 |

### 场景参数（改场景时同步修改）

| 文件 | 作用 |
|---|---|
| `src/parking_lot_city/tools/build_world.py` | 生成世界：道路、停车场、车位框、植被、停放车与行人模型位置 |
| `src/parking_lot_city/worlds/city_parking_lot.sdf` | 生成后的世界文件，Gazebo 实际加载的那一份 |
| `src/parking_planner/config/lot_layout.yaml` | 42 个车位的中心、尺寸、朝向、所在区域 |
| `src/parking_planner/config/route_graph.yaml` | 车道图节点与边，决定全局路线 |
| `src/parking_planner/config/staging_poses.yaml` | 各车位预备位姿（车位局部坐标） |
| `src/scene_agents/scene_agents/traffic_driver.py` | 干扰车 A/B 的闭环航点 |
| `src/scene_agents/scene_agents/pedestrian_agent.py` | 行人航点与让行策略 |

---

## 目录结构

```
apa-parking-gazebo/
├── README.md                    本文档
├── Instruction.md               课程任务书（原始要求，留档）
├── src/                         ROS 2 工作区源码，8 个包
│   ├── parking_lot_city/        场景：世界文件、车辆与停放车模型、桥接配置、建图 launch
│   ├── parking_slam/            slam_toolbox 接入、TF 桥、位姿真值评估
│   ├── prediction_interfaces/   自定义消息（跟踪目标 / 预测轨迹）
│   ├── detection_tracking/      激光聚类 + 多目标跟踪
│   ├── trajectory_prediction/   Transformer 轨迹预测节点
│   ├── risk_assessment/         泊车走廊时空重叠风险评估
│   ├── scene_agents/            干扰车与行人行为
│   └── parking_planner/         选位、Hybrid A*、Reeds-Shepp、跟踪控制、仲裁与评测
├── training/                    预测模型训练脚本（与 src 同级）
│   ├── generate_dataset.py      合成轨迹数据集
│   └── train.py                 训练与 ADE 评估
├── scripts/                     构建、清理、运行、离线校验与批量回归脚本
└── docs/                        设计文档与记录
    ├── 01_系统设计与接口.md      分层架构、话题契约、坐标系约定
    ├── 02_规划控制层设计.md      选位、Hybrid A*、Reeds-Shepp、跟踪律细节
    ├── 03_标定与验收记录.md      控制标定实验、实测数据、指标对照
    ├── 04_对上游交付的修改清单.md 收尾阶段对上游代码的修复与补齐
    ├── 05_运行与排错手册.md      环境搭建、常见故障、WSL2 特有问题
    ├── 06_缺陷与修复清单.md      逐条缺陷的现象、根因、修改与验证
    └── data/                    评测与轨迹样例数据
```

---

## 已知限制

1. **平行车位依赖揉库。** 平行位深度 2.4 m，最小转弯半径 5.74 m，一把入库在几何上不成立。
   规划侧已能给出一到两次换挡的解，控制侧对连续换挡做了专门处理，但末端收敛比垂直位慢，
   单轮耗时明显更长。
2. **转向存在约 20% 不足。** Gazebo 的 `AckermannSteering` 满舵实际约 0.49 rad，规划统一按
   等效 0.44 rad 计算，控制侧另有增益补偿。场内过道转角半径只能做到 6 m 左右，正好压在能力
   边界上，过弯后允许 1–2 m 的横向偏差，交给局部规划收尾。
3. **干扰物是脚本驱动而非学习得到。** 干扰车与行人按固定航点和让行规则行动，会为自车让路，
   但不理解自车意图，也不会协同。多车相遇时依赖「等 15 s → 退 12 m → 停 15 s」的退让策略
   打破僵持，行为偏机械。
4. **预测模型权重不入库。** 仓库不附带 `trajectory_transformer.pt`，需按上文训练一遍；未加载
   权重时预测节点不输出结果，风险恒为 `safe`。
5. **WSL2 下的 DDS 稳定性。** 必须使用 UDP-only 的 Fast-DDS 配置并定期清理残留进程，脚本已内置。
   内存建议 ≥ 8 GB，规划窗口开太大会挤压 Gazebo 的余量。
6. **B 排车位的几何是手工标定的。** 六个平行位的中心按两侧停放车的真实中点取值，与世界里按等距
   网格画的标线不重合，`check_layout.sh` 会稳定报这一项不一致。这是为了让平行位有可执行的端部
   余量而做的取舍，代价是布局自检不能做到全绿。
7. **W0 车位的预备位姿落在障碍里。** 布局自检的第 [3] 项会点出 W0，该位一旦被选中，入位规划
   必然失败。当前 W0 本身停着车、不会被选作目标，所以没有暴露；如果将来要启用这个位，
   需要先调整该排的预备位姿纵向偏移 `staging_along`，或把车位中心挪开。

---

## 复现自查

按顺序执行，全部通过即说明环境与代码一致：

```bash
bash scripts/clean_env.sh                 # 清残留
bash scripts/build.sh                     # 8 个包全部编译通过
source ~/apa_ws/install/setup.bash
bash scripts/check_layout.sh              # 布局自检：B 排会报已知的标线不一致，其余项应为 PASS
python3 scripts/verify_park_sim.py B3     # 离线闭环：终点 ≤0.35 m、间隙 >0、剐蹭 0 个
bash scripts/run_demo.sh                  # 全链路：日志出现「泊车完成」且状态 DONE
```

判读要点：`check_layout.sh` 的第 [2] 项会稳定列出 B0–B5 六个车位的几何不一致（原因见上文
「资源加载规则」的最后一条），其余检查项与 `verify_park_sim.py` 的结果应全部通过。
