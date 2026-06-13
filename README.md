# FastNav

> ROS1 + PX4 SITL + Gazebo Sim + Mid360/LiDAR 的无人机局部导航避障实验平台。

FastNav 的目标是构建一套 **sim / real 解耦**、模块清晰、便于逐层调试的无人机导航避障链路。当前工程以 Gazebo Sim + PX4 SITL 为主要验证环境，算法后端只依赖 FastNav 标准接口，不直接依赖 GZ、PX4、MAVROS 或具体雷达驱动 topic。

---

## ✨ 当前能力

- 🚁 PX4 SITL + Gazebo Sim 仿真无人机，当前默认机型为 `x250_mid360`
- 🌲 Forest 系列障碍环境：`forest_easy`、`forest_medium`、`forest_hard`
- 📡 GZ Mid360 点云桥接到 ROS1
- 🧭 GZ odom / MAVROS 状态桥接与 TF 发布
- 🧱 点云滤波、odom 坐标转换、局部 VoxelMap 建图与膨胀
- 🧠 Planner FSM + LocalPlannerManager 架构
- 🔎 A* 前端搜索，支持 frontend inflated map、base map fallback、goal projection、timeout best-effort、escape search
- 📐 Safe corridor 生成与 GCOPTER / MINCO 后端优化
- 🎮 MINCO trajectory server 采样轨迹并生成 FastNav 控制指令
- 🛫 FastNav control FSM + MAVROS Offboard 控制 PX4
- 🧪 RViz debug、实时状态曲线、bad case 录制与离线重放

---

## 🧩 总体架构

FastNav 采用分层模块组织：

```text
Gazebo Sim / PX4 SITL
        |
        v
fastnav_gz_bridge / MAVROS
        |
        v
FastNav 标准接口
  /fastnav/state/odom
  /fastnav/lidar/points
  TF: odom -> base_link -> mid360_link
        |
        v
fastnav_perception
  cloud_transform_node
  cloud_filter_node
        |
        v
/fastnav/perception/cloud_filtered
        |
        +--------------------------+
        |                          |
        v                          v
fastnav_mapping              fastnav_planner
  local_voxel_map_node         PlannerFSM
  VoxelMap                     LocalPlannerManager
                                AStarPlanner
                                PathOptimizer
                                GCOPTER / MINCO
        |                          |
        v                          v
  map debug topics          /fastnav/planner/minco_trajectory
                                      |
                                      v
                                traj_utils/minco_traj_server
                                      |
                                      v
                                /fastnav/control_cmd
                                      |
                                      v
                                fastnav_control
                                      |
                                      v
                                PX4 Offboard
```

设计原则：

- 算法模块只依赖 FastNav 标准接口。
- sim / real 差异只保留在 bridge / adapter 层。
- planner 不通过 ROS service 高频查询 mapping，而是在同一进程中直接持有 `VoxelMap` 指针。
- Planner 只规划轨迹，不构造 PX4 控制量；控制量由 traj server 和 control 模块处理。

---

## 📦 主要目录

```text
FastNav/
├── config/
│   ├── common/              # sim / real 共用配置：frame、topic、safety、uav、sensor
│   ├── sim/                 # 仿真配置：planner、mapping、controller、perception、px4、gz
│   └── real/                # 真机配置预留
├── ros1_ws/
│   └── src/
│       ├── fastnav_bringup  # 总 launch 与配置加载
│       ├── fastnav_gz_bridge
│       ├── fastnav_tf
│       ├── fastnav_perception
│       ├── fastnav_mapping
│       ├── fastnav_planner
│       ├── fastnav_control
│       ├── fastnav_msgs
│       └── traj_utils       # MINCO trajectory、safe corridor、traj server、可视化工具
├── scripts/                 # 启动、环境、清理脚本
└── tools/
    ├── analysis/            # 实时曲线、bad case 重放
    ├── bag/                 # rosbag 录制脚本
    ├── px4/                 # PX4 参数写入 / airframe 安装脚本
    └── rviz/                # RViz 配置
```

---

## 🔌 标准接口

FastNav 后端模块约定只消费以下标准输入：

```text
/fastnav/state/odom
  type: nav_msgs/Odometry
  frame_id: odom
  child_frame_id: base_link

/fastnav/lidar/points
  type: sensor_msgs/PointCloud2
  frame_id: mid360_link

TF:
  odom
    └── base_link
          └── mid360_link
```

核心中间 topic：

```text
/fastnav/perception/cloud_filtered
/fastnav/map/occupied_cloud
/fastnav/map/inflated_cloud
/fastnav/planner/path
/fastnav/planner/minco_trajectory
/fastnav/control_cmd
```

常用目标点 topic：

```text
/move_base_simple/goal
/fastnav/planner/goal
```

二者消息类型均为：

```text
geometry_msgs/PoseStamped
```

---

## 🚀 快速启动仿真

### 1. 安装 / 更新 PX4 自定义 airframe

第一次使用 FastNav 自定义机型时，需要把 airframe 安装到 PX4：

```bash
cd /home/shukun/Project/FastNav
bash tools/px4/install_fastnav_airframes.sh
```

### 2. 编译 ROS1 工作空间

```bash
cd /home/shukun/Project/FastNav/ros1_ws
catkin_make
source devel/setup.bash
```

### 3. 启动 PX4 + Gazebo Sim

推荐使用脚本：

```bash
cd /home/shukun/Project/FastNav
FASTNAV_GZ_WORLD=forest_easy ./scripts/start_px4_gz.sh
```

也可以直接在 PX4 目录运行：

```bash
cd /home/shukun/ThirdPackages/PX4-Autopilot
PX4_GZ_WORLD=forest_easy make px4_sitl gz_x250_mid360
```

可选世界：

```text
forest_easy
forest_medium
forest_hard
```

### 4. 启动完整 ROS 链路

```bash
cd /home/shukun/Project/FastNav
source ros1_ws/devel/setup.bash
roslaunch fastnav_bringup fastnav_full_sim.launch
```

如果希望单独看 PlannerFSM / timing 日志，推荐拆成两个终端：

```bash
roslaunch fastnav_bringup fastnav_sim_base.launch
roslaunch fastnav_bringup fastnav_planner_fsm.launch
```

### 5. 打开 RViz

```bash
rviz -d /home/shukun/Project/FastNav/tools/rviz/fastnav.rviz
```

RViz 中建议：

```text
Fixed Frame = odom
```

---

## 🎯 发送目标点

示例：

```bash
rostopic pub -1 /fastnav/planner/goal geometry_msgs/PoseStamped "header:
  frame_id: 'odom'
pose:
  position:
    x: -3.0
    y: -4.0
    z: 1.2
  orientation:
    w: 1.0"
```

注意：

- 目标点坐标是 `odom` 坐标，不是地图图片坐标。
- 若目标点超出 planner 局部地图，A* 会尝试 goal projection / best-effort。
- 若目标点位于 inflated obstacle 内，前端可能会报 `Goal is inside inflated obstacle`。

---

## ⚙️ 配置文件

当前运行参数统一放在顶层 `config/`，各 ROS 包内部不再维护独立 config。

最常改的四个仿真配置：

```text
config/sim/perception.yaml
config/sim/mapping.yaml
config/sim/planner.yaml
config/sim/controller.yaml
```

常见参数位置：

| 参数类别 | 文件 |
| --- | --- |
| 地图分辨率、膨胀半径 | `config/sim/mapping.yaml`、`config/sim/planner.yaml` |
| A* 搜索、planning horizon、FSM | `config/sim/planner.yaml` |
| MINCO / GCOPTER 参数 | `config/sim/planner.yaml` |
| 控制频率、起飞高度、MAVROS topic | `config/sim/controller.yaml` |
| PX4 速度 / 加速度 / jerk 限制 | `config/sim/px4/multicopter_limits_x250.params` |
| 机体质量、轴距、推力模型 | `config/sim/gz/models/x250_mid360/model.sdf` |
| Mid360 安装位姿 | `config/common/frames.yaml`、`config/common/sensor/mid360.yaml` |

planner debug 开关：

```yaml
local_planner:
  debug:
    enable: true
    record_failure_cases: true
```

当 `debug.enable=false` 时，planner 不创建 searched nodes、debug map、corridor、FSM state、local target、timing 等调试 publisher。

---

## 🧠 Planner 结构

`fastnav_planner` 当前采用接近 EGO-Planner 的分层思想：

```text
PlannerFSM
  └── LocalPlannerManager
        ├── VoxelMap
        ├── AStarPlanner
        └── PathOptimizer
              ├── shortcut
              ├── safe corridor
              └── GCOPTER / MINCO
```

主要文件：

```text
ros1_ws/src/fastnav_planner/include/fastnav_planner/fsm/planner_fsm.h
ros1_ws/src/fastnav_planner/src/fsm/planner_fsm.cpp

ros1_ws/src/fastnav_planner/include/fastnav_planner/manager/local_planner_manager.h
ros1_ws/src/fastnav_planner/src/manager/local_planner_manager.cpp

ros1_ws/src/fastnav_planner/include/fastnav_planner/frontend/astar_planner.h
ros1_ws/src/fastnav_planner/src/frontend/astar_planner.cpp

ros1_ws/src/fastnav_planner/include/fastnav_planner/optimizer/path_optimizer.h
ros1_ws/src/fastnav_planner/src/optimizer/path_optimizer.cpp
```

FSM 状态：

```text
INIT
WAIT_TARGET
GEN_NEW_TRAJ
REPLAN_TRAJ
EXEC_TRAJ
EMERGENCY_STOP
```

---

## 🧪 调试工具

### 实时状态曲线

```bash
cd /home/shukun/Project/FastNav
source ros1_ws/devel/setup.bash
python3 tools/analysis/live_plot_state.py
```

显示内容包括：

- 实际速度与期望速度
- Planner FSM 状态
- Controller FSM 状态
- local target 更新标记
- planner timing 曲线

### Bad case 离线重放

当 `record_failure_cases=true` 时，planner 后端失败会保存到：

```text
tools/debug_cases/fail_xxx/
```

重放最新 case：

```bash
cd /home/shukun/Project/FastNav
source ros1_ws/devel/setup.bash
python3 tools/analysis/replay_planner_failure_case.py --case latest
```

脚本运行后可在终端按键：

```text
s: 下一个 case
w: 上一个 case
r: 重新加载当前 case
q: 退出
```

RViz 中添加：

```text
/fastnav/debug_case/cloud_filtered
/fastnav/debug_case/occupied_cloud
/fastnav/debug_case/inflated_cloud
/fastnav/debug_case/searched_nodes
/fastnav/debug_case/frontend_path
/fastnav/debug_case/reference_path
/fastnav/debug_case/shortcut_path
/fastnav/debug_case/sampled_path
/fastnav/debug_case/safe_corridor
```

重点对比：

```text
inflated_cloud + frontend_path + reference_path + shortcut_path + sampled_path + safe_corridor
```

### rosbag 录制

```bash
tools/bag/record_sim.sh
```

---

## 🧰 PX4 参数

FastNav 提供多组 PX4 multicopter 限制参数：

```text
config/sim/px4/multicopter_limits_low.params
config/sim/px4/multicopter_limits_medium.params
config/sim/px4/multicopter_limits_high.params
config/sim/px4/multicopter_limits_x250.params
```

写入 PX4：

```bash
cd /home/shukun/Project/FastNav
tools/px4/write_px4_params.sh --sim
```

只预览：

```bash
tools/px4/write_px4_params.sh --sim --dry-run
```

---

## 🧹 清理进程

如果 PX4 / Gazebo / ROS 节点残留：

```bash
cd /home/shukun/Project/FastNav
scripts/kill_px4_gz_ros.sh
```

---

## 🧯 常见问题

### `/fastnav/lidar/points` 没有数据

检查：

```bash
gz topic -l | grep point
rostopic info /fastnav/lidar/points
rosnode list | grep bridge
```

确认 GZ 模型名和 bridge 配置一致。当前默认模型为：

```text
x250_mid360_0
```

### `TF unavailable odom <- mid360_link`

检查：

```bash
rosrun tf view_frames
rosrun tf tf_echo odom base_link
rosrun tf tf_echo base_link mid360_link
```

期望 TF：

```text
odom -> base_link -> mid360_link
```

### `Goal is outside local map`

说明目标点超出当前 planner 局部 VoxelMap。当前 A* 会尝试投影目标点到局部地图内，但如果投影点也不可行，仍可能失败。

相关参数：

```yaml
local_planner:
  map:
    local_x_size
    local_y_size
    local_z_min
    local_z_max
  astar:
    enable_goal_projection
    projection_margin_voxels
```

### `fine check failed: violation=collision`

说明 MINCO 轨迹采样后穿过 inflated map。建议用 bad case 重放工具检查：

```text
sampled_path 是否穿入 inflated_cloud
shortcut_path 是否过度拉直
safe_corridor 是否覆盖 sampled_path
```

### `fine check failed: violation=velocity / acceleration`

说明几何上可能可行，但轨迹动力学超过限制。检查：

```text
config/sim/planner.yaml
  optimizer.minco.max_vel
  optimizer.feasibility.max_vel
  optimizer.feasibility.max_acc
  optimizer.feasibility.max_jerk

config/sim/px4/multicopter_limits_x250.params
```

原则：

```text
planner limit <= PX4 limit <= vehicle physical limit
```

---

## 📌 当前阶段边界

当前 FastNav 重点是局部导航避障闭环验证，暂不包含：

- 全局拓扑地图
- ESDF
- OctoMap
- 完整 raycasting free-space update
- 多机协同
- 真机 MAVROS / Livox adapter 完整接入
- 高级轨迹跟踪控制器

这些内容预留给后续阶段。

