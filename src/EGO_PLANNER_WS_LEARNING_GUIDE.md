# ego-planner_ws/src 总体学习说明

## 1. 一句话认识这个工作空间

`ego-planner_ws` 是一套面向四旋翼无人机的 ROS 1 局部规划与闭环仿真工作空间。

它将以下模块组合在一起：

- 随机三维地图生成。
- 基于全局地图和无人机位姿的局部传感器模拟。
- 概率占据栅格地图。
- A* 路径搜索。
- B 样条轨迹生成与碰撞优化。
- 在线重规划有限状态机。
- B 样条轨迹采样与位置命令发布。
- SO(3) 控制器。
- 四旋翼动力学仿真。
- RViz 可视化、目标点输入和通用数学工具。

默认仿真链路可以概括为：

```text
随机地图生成器
      │ 全局点云
      ▼
局部感知模拟器 ◄──── 无人机 odom
      │ 局部点云
      ▼
plan_env 占据栅格
      │
      ▼
EGO-Planner
  ├── 全局/局部目标管理
  ├── A* 搜索
  ├── B 样条初始化
  ├── 碰撞回弹优化
  └── 在线重规划 FSM
      │ B 样条
      ▼
traj_server
      │ PositionCommand
      ▼
SO(3) 控制器
      │ SO3Command
      ▼
四旋翼动力学模拟器
      │
      └── odom 回到规划、感知和控制模块
```

---

## 2. 工作空间规模

`src` 下共有：

```text
18 个 ROS 功能包
约 560 个文件
约 10 万行文件内容
```

它们分为两大目录：

```text
src/
├── planner/         # EGO-Planner 核心规划算法
└── uav_simulator/   # 地图、传感器、控制、动力学与工具
```

18 个包：

```text
planner/
├── bspline_opt
├── path_searching
├── plan_env
├── plan_manage        # 实际 ROS 包名为 ego_planner
└── traj_utils

uav_simulator/
├── local_sensing
├── map_generator
├── mockamap
├── so3_control
├── so3_quadrotor_simulator
└── Utils/
    ├── cmake_utils
    ├── multi_map_server
    ├── odom_visualization
    ├── pose_utils
    ├── quadrotor_msgs
    ├── rviz_plugins
    ├── uav_utils
    └── waypoint_generator
```

---

## 3. 先认识两种“包”

本工作空间中的功能包并不都对应一个可运行节点。

### 3.1 算法库包

这些包主要编译为共享库，被其他包链接：

| 包 | 构建产物 |
|---|---|
| `plan_env` | `libplan_env.so` |
| `path_searching` | `libpath_searching.so` |
| `bspline_opt` | `libbspline_opt.so` |
| `traj_utils` | `libtraj_utils.so` |
| `pose_utils` | `libpose_utils.so` |
| `so3_control` | `libSO3Control.so`、nodelet |

### 3.2 节点与系统包

这些包提供可运行节点：

| ROS 包 | 主要节点 |
|---|---|
| `ego_planner` | `ego_planner_node`、`traj_server` |
| `map_generator` | `random_forest` |
| `mockamap` | `mockamap_node` |
| `local_sensing_node` | `pcl_render_node` |
| `waypoint_generator` | `waypoint_generator` |
| `so3_quadrotor_simulator` | `quadrotor_simulator_so3` |
| `odom_visualization` | `odom_visualization` |
| `multi_map_server` | `multi_map_visualization` |

---

## 4. 默认仿真从哪里启动

最重要的入口位于：

```text
planner/plan_manage/launch/simple_run.launch
```

注意：

```text
目录名：plan_manage
ROS 包名：ego_planner
```

典型启动：

```bash
source /home/yxc/Desktop/ego-planner_ws/devel/setup.bash
roslaunch ego_planner simple_run.launch
```

`simple_run.launch` 启动或包含：

```text
ego_planner_node
traj_server
waypoint_generator
simulator.xml
rviz.launch
```

`simulator.xml` 又启动：

```text
map_generator/random_forest
mockamap/mockamap_node
so3_quadrotor_simulator/quadrotor_simulator_so3
so3_control/SO3ControlNodelet
odom_visualization/odom_visualization
local_sensing_node/pcl_render_node
```

其中 `run_in_sim.launch` 与 `simple_run.launch` 的主要差异之一是：

- `run_in_sim.launch` 默认 `flight_type = 1`。
- `simple_run.launch` 默认 `flight_type = 2`，并启动 RViz。

---

## 5. 默认核心话题链

默认仿真中的关键话题：

```text
/map_generator/global_cloud
    全局障碍物点云

/pcl_render_node/cloud
    局部感知点云

/visual_slam/odom
    仿真无人机里程计

/waypoint_generator/waypoints
    目标航点 Path

/planning/bspline
    EGO-Planner 输出的 B 样条

/planning/pos_cmd
    traj_server 输出的位置、速度、加速度命令

so3_cmd
    SO(3) 控制命令
```

默认数据流：

```text
random_forest
  └── /map_generator/global_cloud
          │
          ▼
local_sensing/pcl_render_node
  └── /pcl_render_node/cloud
          │
          ▼
ego_planner_node 内部 GridMap
          │
          ▼
/planning/bspline
          │
          ▼
traj_server
  └── /planning/pos_cmd
          │
          ▼
SO3ControlNodelet
  └── so3_cmd
          │
          ▼
quadrotor_simulator_so3
  └── /visual_slam/odom
```

---

## 6. EGO-Planner 核心思想

EGO-Planner 的核心不是每次都先生成一条完整无碰撞搜索路径，再严格沿着路径优化。

它更强调：

```text
直接在 B 样条控制点空间进行优化，
利用障碍物方向信息把碰撞轨迹“弹”出障碍物。
```

典型规划过程：

```text
当前状态 + 局部目标
        │
        ▼
生成初始轨迹
  ├── 首次：多项式轨迹
  └── 重规划：复用当前轨迹后半段
        │
        ▼
参数化为均匀 B 样条控制点
        │
        ▼
检测碰撞控制点
        │
        ├── 必要时用 A* 提供绕障方向
        ▼
优化代价
  ├── 平滑性
  ├── 碰撞距离
  ├── 动力学可行性
  └── 轨迹贴合
        │
        ▼
检查速度和加速度限制
        │
        ▼
发布 B 样条
```

---

## 7. planner/plan_manage：系统总控

### 7.1 包定位

目录：

```text
planner/plan_manage
```

实际包名：

```text
ego_planner
```

这是整个规划系统的集成与调度中心。

它负责：

- 初始化地图、优化器、A* 和可视化模块。
- 接收里程计和目标。
- 维护重规划 FSM。
- 选择局部目标。
- 调用轨迹生成与优化。
- 发布 B 样条。
- 将 B 样条采样为控制命令。

### 7.2 构建目标

```text
ego_planner_node
traj_server
ego_planner/Bspline.msg
ego_planner/DataDisp.msg
```

### 7.3 ego_planner_node

节点入口很薄：

```text
构造 EGOReplanFSM
→ init()
→ ros::spin()
```

真正逻辑集中在：

- `ego_replan_fsm.cpp`
- `planner_manager.cpp`

### 7.4 重规划 FSM

状态：

```text
INIT
WAIT_TARGET
GEN_NEW_TRAJ
REPLAN_TRAJ
EXEC_TRAJ
EMERGENCY_STOP
```

主要转换：

```text
INIT
  └── odom 与触发条件就绪
      → WAIT_TARGET

WAIT_TARGET
  └── 收到目标
      → GEN_NEW_TRAJ

GEN_NEW_TRAJ
  ├── 成功 → EXEC_TRAJ
  └── 失败 → 重试 GEN_NEW_TRAJ

EXEC_TRAJ
  ├── 轨迹结束 → WAIT_TARGET
  ├── 接近终点 → 继续执行
  └── 达到重规划条件 → REPLAN_TRAJ

REPLAN_TRAJ
  ├── 成功 → EXEC_TRAJ
  └── 失败 → 继续重规划

EMERGENCY_STOP
  └── 停稳后 → GEN_NEW_TRAJ
```

FSM 执行定时器：

```text
100 Hz
```

碰撞安全检查：

```text
20 Hz
```

### 7.5 EGOPlannerManager

`EGOPlannerManager` 将核心算法库组合起来：

```text
GridMap
BsplineOptimizer
AStar
PlanningVisualization
```

最重要接口：

```cpp
reboundReplan(...)
```

它负责：

1. 根据当前状态和局部目标生成初始轨迹。
2. 采样初始轨迹。
3. 参数化为 B 样条。
4. 调用回弹优化。
5. 检查动力学可行性。
6. 必要时延长轨迹时间并细化。
7. 更新当前局部轨迹。

### 7.6 traj_server

`traj_server` 订阅：

```text
planning/bspline
```

以 100 Hz 发布：

```text
quadrotor_msgs/PositionCommand
```

它从 B 样条求出：

- 位置。
- 一阶导数速度。
- 二阶导数加速度。
- 根据未来轨迹方向计算的 yaw 与 yaw rate。

轨迹结束后，它继续发布终点悬停命令。

### 7.7 自定义消息

`Bspline.msg` 包含：

- 阶数。
- 轨迹 ID。
- 开始时间。
- 节点向量。
- 位置控制点。
- 预留的 yaw 控制点和间隔。

当前 `traj_server` 主要使用位置 B 样条；yaw B 样条代码处于注释状态。

---

## 8. planner/plan_env：局部概率占据地图

### 8.1 包定位

`plan_env` 为规划器提供三维局部栅格地图。

它处理：

- 深度图与相机位姿。
- 深度图与 odom。
- 点云与 odom。
- 射线清空。
- 概率占据更新。
- 障碍物膨胀。
- 地图可视化。

### 8.2 核心类

```text
GridMap
RayCaster
```

`GridMap` 提供：

- 世界坐标与栅格索引转换。
- 地图边界检查。
- 占据、空闲和未知状态查询。
- 膨胀占据查询。

`RayCaster` 使用三维栅格射线遍历，确定传感器到测量点之间哪些栅格应被更新为空闲。

### 8.3 输入模式

地图支持两类感知输入：

```text
深度图 + Pose/Odometry
```

或：

```text
PointCloud2 + Odometry
```

默认仿真通过 launch 重映射：

```text
/grid_map/cloud → /pcl_render_node/cloud
/grid_map/odom  → /visual_slam/odom
```

### 8.4 输出

```text
/grid_map/occupancy
/grid_map/occupancy_inflate
/grid_map/unknown
```

这些主要用于 RViz 调试。

规划器本身直接持有 `GridMap` 对象并查询其内存数据。

### 8.5 关键参数

`advanced_param.xml` 集中配置：

- 地图分辨率和尺寸。
- 局部更新范围。
- 障碍物膨胀尺寸。
- 深度滤波。
- 相机内参。
- 概率更新参数。
- 最大/最小射线长度。
- 虚拟天花板高度。

---

## 9. planner/path_searching：三维 A*

### 9.1 包定位

`path_searching` 提供三维栅格 A*。

核心类：

```text
AStar
GridNode
```

### 9.2 在 EGO-Planner 中的作用

它不是持续作为最终轨迹输出。

在回弹优化中，当初始 B 样条与障碍物相交时，A* 可用于寻找绕开障碍物的引导路径，并帮助计算控制点应该被推向哪个方向。

### 9.3 地图接口

它直接查询：

```cpp
grid_map_->getInflateOccupancy(pos)
```

因此规划搜索基于膨胀后的障碍物，而不是原始占据点。

### 9.4 构建形式

该包只生成：

```text
libpath_searching.so
```

没有独立 ROS 节点。

---

## 10. planner/bspline_opt：B 样条与轨迹优化

### 10.1 包定位

这是核心算法代码量最大的 planner 包。

主要包括：

```text
UniformBspline
BsplineOptimizer
GradientDescentOptimizer
LBFGS
```

### 10.2 UniformBspline

提供：

- 均匀 B 样条表示。
- De Boor 求值。
- 速度和加速度导数轨迹。
- 轨迹时长。
- 动力学可行性检查。
- 时间拉伸。
- 从采样点和边界导数参数化控制点。

位置轨迹求导后可得到：

```text
position B-spline
→ velocity B-spline
→ acceleration B-spline
```

### 10.3 BsplineOptimizer

核心代价包括：

```text
平滑代价
碰撞距离代价
动力学可行性代价
轨迹贴合代价
```

主要优化模式：

```text
Rebound
Refine
```

Rebound 阶段主要将碰撞控制点推离障碍物。

Refine 阶段用于进一步满足动力学限制并改善轨迹。

### 10.4 A* 的嵌入

优化器内部持有：

```text
a_star_
```

当轨迹穿过障碍物时，可借助 A* 路径寻找合适的绕障方向，而不是仅依赖局部距离梯度。

### 10.5 构建形式

该包生成：

```text
libbspline_opt.so
```

没有独立节点。

---

## 11. planner/traj_utils：轨迹与可视化工具

### 11.1 包定位

该包提供规划过程中的通用轨迹和 RViz 可视化工具。

主要类：

```text
PolynomialTraj
PlanningVisualization
```

### 11.2 PolynomialTraj

用于生成：

- 单段多项式轨迹。
- Min-snap 多段轨迹。
- 位置、速度和加速度求值。

在 EGO-Planner 中，多项式轨迹常用于生成首次 B 样条优化的初始路径。

### 11.3 PlanningVisualization

发布 RViz Marker：

```text
goal_point
global_list
init_list
optimal_list
a_star_list
```

用于观察：

- 目标点。
- 全局路径。
- 初始路径。
- 优化后控制点。
- A* 绕障路径。

### 11.4 构建形式

该包生成：

```text
libtraj_utils.so
```

---

## 12. uav_simulator/map_generator：随机障碍物点云

### 12.1 包定位

`map_generator` 生成适合规划测试的随机三维障碍物点云。

节点：

```text
random_forest
```

### 12.2 地图内容

可生成：

- 随机柱状障碍物。
- 圆环或复杂形状障碍物。

主要参数包括：

- 地图尺寸。
- 分辨率。
- 障碍物数量。
- 障碍物宽度、高度。
- 圆环数量和尺寸。
- 最小障碍物间距。

### 12.3 输出

```text
/map_generator/global_cloud
/map_generator/local_cloud
/pcl_render_node/local_map
```

全局点云供局部感知模拟器使用。

### 12.4 值得注意

源码读取感知频率时再次使用了：

```text
sensing/radius
```

而不是独立的 `sensing/rate` 参数，属于值得核查的参数键问题。

---

## 13. uav_simulator/mockamap：程序化测试地图

### 13.1 包定位

`mockamap` 是另一套地图生成器，专注于可配置的程序化地图类型。

节点：

```text
mockamap_node
```

### 13.2 支持地图

通过 `type` 选择：

```text
1：Perlin 噪声三维地图
2：柱状障碍物地图
3：二维迷宫
4：三维迷宫
```

### 13.3 输出

```text
mock_map
```

在默认 `simulator.xml` 中被重映射为：

```text
/map_generator/global_cloud
```

### 13.4 与 map_generator 的关系

两个包都能提供全局障碍物点云。

它们更像可替换的地图来源：

```text
map_generator：随机森林和圆环，适合 EGO-Planner 原始演示
mockamap：多种程序化地图，适合系统化测试
```

默认 launch 中两者可能同时启动并发布到相关地图链路，实际测试时应明确希望使用哪个来源。

---

## 14. uav_simulator/local_sensing：局部感知模拟

### 14.1 包定位

`local_sensing_node` 根据：

```text
全局点云 + 无人机 odom
```

模拟无人机当前能感知到的局部障碍物。

### 14.2 CPU 与 CUDA 两条路径

CMake 中：

```cmake
set(ENABLE_CUDA false)
```

当前默认构建 CPU 版本：

```text
src/pointcloud_render_node.cpp
```

如果启用 CUDA，则构建：

```text
src/pcl_render_node.cpp
src/depth_render.cu
```

### 14.3 CPU 版本

CPU 版本主要按距离截取局部点云并发布：

```text
/pcl_render_node/cloud
```

它不是真正的相机遮挡与深度图渲染。

### 14.4 CUDA 版本

CUDA 版本可模拟：

- 深度图。
- 彩色深度图。
- 相机位姿。
- 渲染局部点云。

并支持动态重配置外参。

### 14.5 默认输入

```text
global_map
local_map
odometry
```

通过 `simulator.xml` 重映射到全局地图和 `/visual_slam/odom`。

---

## 15. uav_simulator/so3_control：SO(3) 控制器

### 15.1 包定位

该包将规划器输出的：

```text
PositionCommand
```

转换为：

```text
SO3Command
```

用于驱动四旋翼动力学模型。

### 15.2 运行形式

主要以 nodelet 运行：

```text
so3_control/SO3ControlNodelet
```

同时提供：

```text
control_example
```

用于简单命令测试。

### 15.3 输入与输出

输入：

```text
odom
position_cmd
imu
motors
corrections
```

输出：

```text
so3_cmd
```

### 15.4 控制结构

SO(3) 控制器使用：

- 位置误差。
- 速度误差。
- 期望加速度。
- 质量与重力。
- 姿态和角速度增益。

生成期望总力、姿态和控制增益。

配置位于：

```text
config/gains_*.yaml
config/corrections_*.yaml
```

---

## 16. uav_simulator/so3_quadrotor_simulator：动力学仿真

### 16.1 包定位

该包是一个不依赖 Gazebo 的四旋翼刚体动力学模拟器。

节点：

```text
quadrotor_simulator_so3
```

### 16.2 输入

```text
SO3Command
外部力扰动
外部力矩扰动
```

### 16.3 输出

```text
nav_msgs/Odometry
sensor_msgs/Imu
```

默认仿真中 odom 被重映射到：

```text
/visual_slam/odom
```

形成完整闭环。

### 16.4 动力学

核心库：

```text
libquadrotor_dynamics.so
```

负责积分：

- 位置和速度。
- 姿态和角速度。
- 推力与力矩。
- 外部扰动。

默认仿真频率可达：

```text
1000 Hz
```

odom 发布频率通常为：

```text
100 Hz 或 200 Hz
```

### 16.5 包体积较大的原因

包内包含一整套 ODE 头文件副本，因此文件数量和行数远高于其他包。

---

## 17. uav_simulator/Utils/waypoint_generator：目标输入适配器

### 17.1 包定位

该节点将不同来源的目标转换成：

```text
nav_msgs/Path
```

供 EGO-Planner FSM 使用。

### 17.2 输入

```text
odom
goal
traj_start_trigger
```

默认目标来自 RViz：

```text
/move_base_simple/goal
```

### 17.3 输出

```text
waypoints
waypoints_vis
```

EGO-Planner 默认订阅：

```text
/waypoint_generator/waypoints
```

### 17.4 waypoint_type

支持手动目标和预设路径等模式。

默认 launch 使用：

```text
manual-lonely-waypoint
```

---

## 18. uav_simulator/Utils/quadrotor_msgs：消息协议中心

### 18.1 包定位

该包定义规划、控制和模拟器之间使用的自定义消息。

最关键的消息：

```text
PositionCommand
SO3Command
AuxCommand
Corrections
```

### 18.2 当前主链中的消息

```text
traj_server
  └── PositionCommand
          │
          ▼
SO3ControlNodelet
  └── SO3Command
          │
          ▼
quadrotor_simulator_so3
```

### 18.3 PositionCommand

包含：

- 期望位置。
- 期望速度。
- 期望加速度。
- yaw 与 yaw rate。
- 位置和速度增益。
- 轨迹 ID 和状态。

### 18.4 SO3Command

包含：

- 期望合力。
- 期望姿态。
- 姿态和角速度增益。
- 电机使能与修正信息。

### 18.5 旧串口协议

包内还保留：

```text
encode_msgs
decode_msgs
Serial
StatusData
OutputData
PPROutputData
```

它们属于较旧的飞控串口通信体系，不是默认 EGO-Planner 仿真主链。

---

## 19. uav_simulator/Utils/odom_visualization：里程计可视化

该节点订阅：

```text
odom
cmd
```

并发布：

- PoseStamped。
- Path。
- 速度 Marker。
- 协方差 Marker。
- 轨迹 Marker。
- 机器人 mesh。
- 高度 Range。

默认仿真中用于在 RViz 显示四旋翼位置、轨迹和姿态。

可配置：

- 颜色。
- 模型缩放。
- mesh 资源。
- frame。
- 是否显示协方差。
- TF 姿态变换选项。

---

## 20. uav_simulator/Utils/pose_utils：位姿数学库

`pose_utils` 提供基于 Armadillo 的位姿、旋转和几何计算工具。

它编译为：

```text
libpose_utils.so
```

主要服务于较旧的地图、可视化和无人机工具模块。

它没有独立节点。

---

## 21. uav_simulator/Utils/uav_utils：轻量无人机工具

`uav_utils` 主要提供：

- ROS 消息与 Eigen 类型转换。
- 四元数、旋转矩阵和欧拉角工具。
- 角度归一化与数值限幅。
- odom、TF 和欧拉角调试脚本。
- 数学工具测试。

它主要采用 header-only 形式。

脚本包括：

```text
send_odom.py
odom_to_euler.py
tf_assist.py
topic_statistics.py
```

---

## 22. uav_simulator/Utils/rviz_plugins：自定义 RViz 交互

该包提供自定义 RViz 插件，包括：

- 目标点发布工具。
- 位姿工具。
- 类游戏式多机输入工具。
- 概率地图显示。
- 多地图显示。
- 航拍地图显示。

它编译为：

```text
librviz_plugins.so
```

在 EGO-Planner 默认单机演示中，最常用目标输入仍可直接使用 RViz 自带的 `2D Nav Goal`；该包更多服务于扩展交互和旧地图显示链。

---

## 23. uav_simulator/Utils/multi_map_server：多地图消息与可视化

该包定义：

```text
MultiOccupancyGrid
MultiSparseMap3D
SparseMap3D
VerticalOccupancyGridList
```

并构建：

```text
multi_map_visualization
```

用于将多层二维地图和稀疏三维地图转换为可视化输出。

真正的 `multi_map_server.cc` 位于：

```text
src/unused/
```

且没有在当前 CMake 中构建。

因此这个包在当前工作空间更偏向旧系统兼容与 RViz 插件依赖，不属于 EGO-Planner 默认主链。

---

## 24. uav_simulator/Utils/cmake_utils：旧式 CMake 工具

该包提供：

- 架构检测。
- 彩色 CMake 输出。
- CMake 模块辅助。
- Eigen、GSL 等查找模块。

它不包含运行节点。

主要被较旧的无人机控制和工具包依赖。

---

## 25. 默认仿真参数分层

参数主要分布在三层。

### 25.1 simple_run.launch

定义场景级参数：

- 地图尺寸。
- odom topic。
- 最大速度与加速度。
- 规划视野。
- 飞行类型。
- 预设航点。

### 25.2 advanced_param.xml

定义规划算法级参数：

- FSM 重规划阈值。
- GridMap 参数。
- 规划管理器限制。
- 优化代价权重。
- B 样条速度和加速度限制。

### 25.3 simulator.xml

定义仿真系统级参数：

- 初始位置。
- 地图障碍物数量。
- 局部感知半径和频率。
- 动力学模拟器频率。
- SO(3) 控制器增益。
- odom 可视化。

---

## 26. 规划参数如何理解

### 26.1 max_vel / max_acc

同时影响：

- 初始轨迹时间估计。
- B 样条动力学可行性。
- 轨迹优化代价。
- 时间拉伸。

### 26.2 planning_horizon

规划器不会每次直接优化到很远的最终目标，而是沿全局轨迹选择一个局部目标。

`planning_horizon` 控制局部规划距离。

### 26.3 control_points_distance

控制 B 样条控制点密度。

更小：

- 轨迹表达更灵活。
- 优化变量更多。
- 计算开销更大。

### 26.4 lambda_smooth

控制平滑性代价权重。

### 26.5 lambda_collision

控制避障代价权重。

### 26.6 lambda_feasibility

控制速度和加速度约束惩罚。

### 26.7 dist0

障碍物开始产生优化排斥代价的安全距离尺度。

---

## 27. 目标模式

FSM 定义：

```text
MANUAL_TARGET = 1
PRESET_TARGET = 2
REFENCE_PATH = 3
```

默认 launch 常用：

- `flight_type = 1`：手动目标。
- `flight_type = 2`：预设航点。

手动模式中，RViz 目标经 `waypoint_generator` 转为 Path。

预设模式中，FSM 读取 launch 中配置的 waypoint 数组。

---

## 28. 轨迹从规划到执行

### 28.1 ego_planner 发布 Bspline

规划成功后，FSM 发布：

```text
/planning/bspline
```

消息携带控制点、节点向量和开始时间。

### 28.2 traj_server 采样

`traj_server` 以 100 Hz 求值：

```text
位置 B 样条
速度 B 样条
加速度 B 样条
```

并向前观察轨迹方向生成 yaw。

### 28.3 SO(3) 控制

SO3ControlNodelet 接收 `PositionCommand` 与当前 odom，计算期望合力与姿态。

### 28.4 动力学积分

模拟器接收 `SO3Command`，积分动力学并发布新 odom。

这个 odom 同时反馈给：

- EGO-Planner FSM。
- GridMap。
- local_sensing。
- SO(3) 控制器。
- odom_visualization。

---

## 29. 可视化链路

默认 RViz 可观察：

- 全局障碍物点云。
- 局部感知点云。
- 占据地图和膨胀地图。
- 目标点。
- 初始轨迹。
- 优化控制点。
- A* 路径。
- 无人机模型。
- odom 轨迹。

关键可视化发布者：

```text
PlanningVisualization
GridMap
odom_visualization
map_generator
local_sensing
```

---

## 30. 推荐阅读顺序

### 第一阶段：先跑通并看懂数据流

```text
planner/plan_manage/launch/simple_run.launch
planner/plan_manage/launch/simulator.xml
planner/plan_manage/launch/advanced_param.xml
```

目标：

- 知道启动哪些节点。
- 知道 topic 如何重映射。
- 知道参数位于哪里。

### 第二阶段：理解系统总控

```text
ego_planner_node.cpp
ego_replan_fsm.h/.cpp
planner_manager.h/.cpp
traj_server.cpp
```

目标：

- 理解 FSM。
- 理解何时规划、重规划和急停。
- 理解 B 样条如何变成 PositionCommand。

### 第三阶段：理解环境与搜索

```text
plan_env/grid_map.*
plan_env/raycast.*
path_searching/dyn_a_star.*
```

目标：

- 理解局部占据地图。
- 理解点云如何形成障碍物。
- 理解 A* 在优化器中的辅助角色。

### 第四阶段：理解轨迹优化

```text
bspline_opt/uniform_bspline.*
bspline_opt/bspline_optimizer.*
traj_utils/polynomial_traj.*
```

目标：

- 理解 B 样条。
- 理解初始轨迹。
- 理解回弹优化和动力学可行性。

### 第五阶段：理解闭环仿真

```text
local_sensing
so3_control
so3_quadrotor_simulator
map_generator
mockamap
```

---

## 31. 常用运行与检查命令

### 启动默认演示

```bash
source /home/yxc/Desktop/ego-planner_ws/devel/setup.bash
roslaunch ego_planner simple_run.launch
```

### 查看节点

```bash
rosnode list
```

### 查看主链话题

```bash
rostopic hz /visual_slam/odom
rostopic hz /pcl_render_node/cloud
rostopic hz /planning/bspline
rostopic hz /planning/pos_cmd
```

### 检查规划命令

```bash
rostopic echo -n 1 /planning/pos_cmd
```

### 检查地图

```bash
rostopic echo -n 1 /map_generator/global_cloud/header
rostopic echo -n 1 /grid_map/occupancy_inflate/header
```

### 检查目标

```bash
rostopic echo /waypoint_generator/waypoints
```

---

## 32. 当前源码中值得注意的问题

### 32.1 plan_manage 包名与目录名不同

使用命令时应写：

```text
ego_planner
```

而不是：

```text
plan_manage
```

### 32.2 local_sensing 默认关闭 CUDA

默认 CPU 版本主要输出局部点云，并不提供完整深度相机渲染。

launch 中虽然保留深度图和相机位姿接口，但默认点云模式依赖 cloud + odom。

### 32.3 map_generator 感知频率参数键疑似错误

源码两次读取：

```text
sensing/radius
```

第二次看起来应为 `sensing/rate`。

### 32.4 planner_manager 含有本地调试输出

源码中存在：

```text
yxc see reboundReplan step1
hahaha1
hahaha2
```

这些不影响核心算法，但说明当前工作区包含个人调试修改。

### 32.5 多个包包含旧 ROS 构建模板和备份文件

例如：

- `.cpp~`
- `.msg~`
- `src/unused`
- 旧 Python 生成文件。
- 大量注释掉的 rosbuild 模板。

阅读时应优先看 CMake 当前真正编译的文件。

### 32.6 包依赖声明普遍较旧

部分包使用旧式 `run_depend`，或没有完整声明源码直接依赖。

它们在当前完整工作空间中可构建，但独立复用时可能需要补依赖。

### 32.7 仿真主链不是 PX4/MAVROS

这个工作空间默认使用：

```text
SO3ControlNodelet + quadrotor_simulator_so3
```

而不是 PX4、MAVROS 或 `px4ctrl`。

若接入真实 PX4 系统，通常保留：

```text
ego_planner_node + traj_server
```

再将 `PositionCommand` 接入外部飞控控制器。

---

## 33. 如何与 px4_ws 对接

当前 `ego-planner_ws` 的 `traj_server` 发布：

```text
quadrotor_msgs/PositionCommand
```

你的 `px4_ws` 中 `px4ctrl` 也订阅同名语义消息。

概念对接方式：

```text
ego_planner
  └── /planning/bspline
          │
          ▼
traj_server
  └── /position_cmd
          │
          ▼
px4ctrl
  └── MAVROS AttitudeTarget
          │
          ▼
PX4
```

需要重点确认：

- 两个工作空间使用的 `quadrotor_msgs/PositionCommand` MD5 是否一致。
- odom topic 与坐标系是否一致。
- `world`、ENU/NED 和机体系定义。
- px4ctrl 命令模式和超时配置。
- traj_server 输出频率。
- 目标与触发流程。

当前两个工作空间的 `PositionCommand` 定义并不完全相同：`px4_ws` 版本包含 `jerk` 字段，而该 EGO-Planner 工作空间版本没有。

因此不能仅凭消息同名直接连接，必须统一消息定义并重新编译。

---

## 34. 各包速查表

| 包 | 类型 | 一句话职责 | 默认主链 |
|---|---|---|---|
| `ego_planner` | 节点/总控 | FSM、规划管理、B 样条发布与轨迹采样 | 是 |
| `plan_env` | 库 | 局部概率占据地图与射线更新 | 是 |
| `path_searching` | 库 | 三维 A* 搜索 | 是 |
| `bspline_opt` | 库 | B 样条表示、碰撞优化与可行性检查 | 是 |
| `traj_utils` | 库 | 多项式初始轨迹与规划可视化 | 是 |
| `map_generator` | 节点 | 随机障碍物点云生成 | 是 |
| `mockamap` | 节点 | Perlin、柱体和迷宫地图生成 | 可替换/辅助 |
| `local_sensing_node` | 节点 | 从全局地图模拟局部感知 | 是 |
| `so3_control` | nodelet/库 | PositionCommand 到 SO3Command | 是 |
| `so3_quadrotor_simulator` | 节点/库 | 四旋翼刚体动力学仿真 | 是 |
| `waypoint_generator` | 节点 | 将目标输入转换为 waypoint Path | 是 |
| `quadrotor_msgs` | 消息/库 | 规划、控制与模拟器通信协议 | 是 |
| `odom_visualization` | 节点 | odom、轨迹和无人机模型可视化 | 是 |
| `uav_utils` | 工具 | Eigen/ROS 转换与调试脚本 | 辅助 |
| `pose_utils` | 库 | 旧式位姿数学工具 | 辅助 |
| `rviz_plugins` | RViz 插件 | 目标、地图和多机交互 | 辅助 |
| `multi_map_server` | 消息/节点 | 多层地图消息和可视化 | 遗留/辅助 |
| `cmake_utils` | 构建工具 | 旧式 CMake 模块 | 构建依赖 |

---

## 35. 最小心智模型

第一次学习时，可以只记住六层：

```text
第一层：地图来源
  map_generator / mockamap

第二层：局部感知
  local_sensing

第三层：规划环境
  plan_env

第四层：规划算法
  path_searching + bspline_opt + traj_utils

第五层：规划调度与轨迹输出
  ego_planner_node + traj_server

第六层：控制与动力学
  so3_control + so3_quadrotor_simulator
```

出现问题时按层排查：

```text
没有障碍物
→ 检查地图来源与局部感知

规划器认为地图为空
→ 检查 plan_env 输入和 odom

没有轨迹
→ 检查目标、FSM 和优化日志

有 B 样条但无人机不动
→ 检查 traj_server、PositionCommand 和 SO3 控制

无人机运动但规划发散
→ 检查 odom、坐标系、控制跟踪和地图时间
```

---

## 36. 总结

这个工作空间是一套较完整的 EGO-Planner 单机闭环仿真环境。

真正的核心规划代码只有五个包：

```text
plan_env
path_searching
bspline_opt
traj_utils
ego_planner
```

其余包主要负责：

```text
地图生成
局部感知
目标输入
控制执行
动力学模拟
可视化与工具
```

默认演示的主闭环是：

```text
随机地图
→ 局部点云
→ 局部占据地图
→ EGO 在线重规划
→ B 样条
→ PositionCommand
→ SO(3) 控制
→ 四旋翼动力学
→ odom 反馈
```

学习时最值得优先掌握的三个文件是：

```text
ego_replan_fsm.cpp
planner_manager.cpp
bspline_optimizer.cpp
```

它们分别回答：

```text
什么时候规划？
如何组织一次规划？
轨迹究竟怎样避开障碍物？
```

掌握这三层后，再向外阅读地图、仿真和工具包，整个工作空间的结构会非常清晰。
