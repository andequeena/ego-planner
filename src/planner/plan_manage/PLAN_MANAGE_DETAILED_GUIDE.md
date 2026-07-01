# plan_manage详细学习指南

## 1. 一句话认识这个功能包

`plan_manage` 是 EGO-Planner 的**规划总调度层**。

它本身不负责完成所有底层算法，而是把已经存在的地图、搜索、B 样条优化、轨迹消息和控制接口组织成一条完整运行链：

```text
目标点 / 预设航点
        ↓
全局参考轨迹
        ↓
滚动选取局部目标
        ↓
生成并优化局部 B 样条
        ↓
碰撞检查与动态重规划
        ↓
发布 Bspline 消息
        ↓
traj_server 按时间采样
        ↓
PositionCommand
        ↓
飞控 / 仿真控制器
```

如果把 EGO-Planner 看成一支协作团队：

- `plan_env` 负责回答“哪里可通行、哪里有障碍”；
- `path_searching` 负责在必要时寻找绕障方向；
- `bspline_opt` 负责把控制点优化成平滑、安全、动力学可行的轨迹；
- `plan_manage` 负责决定“什么时候规划、从哪里规划、规划到哪里、失败后怎么办、何时重规划”；
- `traj_server` 负责把规划结果变成控制器能够持续接收的时序指令。

---

## 2. 目录名与 ROS 包名

这个目录叫：

```text
plan_manage
```

但它的 `CMakeLists.txt` 和 `package.xml` 定义的 ROS 包名是：

```text
ego_planner
```

因此使用 ROS 命令时要写：

```bash
rosrun ego_planner ego_planner_node
rosrun ego_planner traj_server
roslaunch ego_planner run_in_sim.launch
```

而不是：

```bash
rosrun plan_manage ...
```

源码中的自定义消息也属于 `ego_planner` 命名空间：

```text
ego_planner/Bspline
ego_planner/DataDisp
```

---

## 3. 目录结构

```text
plan_manage/
├── CMakeLists.txt
├── package.xml
├── include/plan_manage/
│   ├── ego_replan_fsm.h
│   ├── plan_container.hpp
│   └── planner_manager.h
├── src/
│   ├── ego_planner_node.cpp
│   ├── ego_replan_fsm.cpp
│   ├── planner_manager.cpp
│   └── traj_server.cpp
├── msg/
│   ├── Bspline.msg
│   └── DataDisp.msg
└── launch/
    ├── advanced_param.xml
    ├── run_in_sim.launch
    ├── simple_run.launch
    ├── simulator.xml
    ├── rviz.launch
    └── default.rviz
```

### 文件职责速查

| 文件                     | 主要职责                                                     |
| ------------------------ | ------------------------------------------------------------ |
| `ego_planner_node.cpp` | 规划节点入口，创建并启动重规划状态机                         |
| `ego_replan_fsm.cpp`   | 状态机、目标接收、局部目标选取、规划触发、碰撞检查、轨迹发布 |
| `planner_manager.cpp`  | 全局参考轨迹生成、局部轨迹初始化、B 样条优化、可行性调整     |
| `traj_server.cpp`      | 接收 B 样条并按时间采样，发布 `PositionCommand`            |
| `plan_container.hpp`   | 保存全局轨迹、局部轨迹及其导数                               |
| `ego_replan_fsm.h`     | 状态机类声明及 ROS 接口                                      |
| `planner_manager.h`    | 规划管理器类声明                                             |
| `Bspline.msg`          | 规划节点与轨迹服务器之间的轨迹消息                           |
| `DataDisp.msg`         | 调试数据消息，当前实际使用很少                               |
| `advanced_param.xml`   | 规划、地图、优化器的主要参数                                 |
| `simulator.xml`        | 仿真器、控制器、地图和局部感知节点组合                       |

---

## 4. 构建结果与依赖关系

## 4.1 构建出的节点

该包构建两个可执行文件。

### `ego_planner_node`

由以下源码组成：

```text
src/ego_planner_node.cpp
src/ego_replan_fsm.cpp
src/planner_manager.cpp
```

它负责：

- 接收里程计和目标；
- 维护重规划状态机；
- 生成全局参考轨迹；
- 调用地图、A* 和 B 样条优化器；
- 发布局部 B 样条；
- 在线进行碰撞检查与紧急停止。

### `traj_server`

由以下源码组成：

```text
src/traj_server.cpp
```

它负责：

- 接收 `ego_planner/Bspline`；
- 重建位置、速度、加速度 B 样条；
- 以 100 Hz 对轨迹进行时间采样；
- 计算期望偏航角；
- 发布 `quadrotor_msgs/PositionCommand`。

## 4.2 主要依赖

```text
roscpp
std_msgs
geometry_msgs
quadrotor_msgs
plan_env
path_searching
bspline_opt
traj_utils
message_generation
cv_bridge
Eigen3
PCL
```

其中最关键的规划依赖关系是：

```text
plan_manage
├── plan_env        地图、膨胀占据查询、局部地图更新
├── path_searching  A* 搜索和控制点方向引导
├── bspline_opt     B 样条控制点优化
└── traj_utils      多项式轨迹、UniformBspline、可视化工具
```

## 4.3 构建配置中值得注意的地方

当前 `CMakeLists.txt` 同时加入了：

```cmake
ADD_COMPILE_OPTIONS(-std=c++11)
ADD_COMPILE_OPTIONS(-std=c++14)
```

最终通常以后出现的 `-std=c++14` 为准，但这种重复配置会让工程意图不够清晰。

`catkin_package()` 声明了：

```cmake
LIBRARIES ego_planner
```

但当前包并没有通过 `add_library()` 构建名为 `ego_planner` 的库。对于本工作空间内直接构建两个节点通常不影响运行，但其他包若试图链接这个声明的库，就会产生困惑。

`package.xml` 中的依赖声明也不完整。源码和 CMake 使用了 `geometry_msgs`、`quadrotor_msgs`、`message_generation`、`cv_bridge` 等依赖，但它们没有被完整写入 `package.xml`。在已有工作空间中可能因为其他包顺带安装而正常构建，单独发布或在干净环境构建时则容易失败。

---

## 5. 总体架构

## 5.1 两节点分工

该包没有让规划节点直接发布每一时刻的位置指令，而是采用了两级结构：

```text
ego_planner_node
    低频、事件驱动地生成完整局部轨迹
              ↓ /planning/bspline
traj_server
    高频、按当前时间采样轨迹
              ↓ /planning/pos_cmd
控制器
```

这种设计的优点是：

- 规划与控制采样解耦；
- 规划器无需以控制频率运行；
- 即使短时间没有新轨迹，`traj_server` 仍可沿旧轨迹连续发布命令；
- 控制器获得的是平滑的连续位置、速度和加速度参考。

## 5.2 核心对象关系

```text
ego_planner_node
└── EGOReplanFSM
    ├── EGOPlannerManager
    │   ├── GridMap
    │   ├── BsplineOptimizer
    │   ├── AStar
    │   ├── LocalTrajData
    │   └── GlobalTrajData
    └── PlanningVisualization
```

### `EGOReplanFSM`

这是系统的决策与调度中心。

它保存当前状态、里程计、目标点、局部目标和规划器实例，并决定下一步动作。

### `EGOPlannerManager`

这是算法组合层。

状态机告诉它“从当前状态规划到这个局部目标”，它负责完成轨迹初始化、优化、可行性检查和轨迹数据更新。

### `GridMap`

提供障碍物信息和膨胀占据查询。

### `BsplineOptimizer`

优化 B 样条控制点，使轨迹同时兼顾平滑性、避障、终点约束和动力学约束。

### `AStar`

不是直接生成最终飞行轨迹，而是为陷入障碍附近的 B 样条控制点提供绕障方向信息。

---

## 6. 节点入口：ego_planner_node.cpp

入口代码很短，其核心流程可以概括为：

```cpp
ros::init(argc, argv, "ego_planner_node");
ros::NodeHandle nh("~");

EGOReplanFSM rebo_replan;
rebo_replan.init(nh);

ros::Duration(1.0).sleep();
ros::spin();
```

### 重要理解

1. 使用的是私有节点句柄 `~`，所以参数通常位于节点私有命名空间下。
2. `init()` 内创建订阅器、发布器、定时器、地图和优化器。
3. `ros::spin()` 是单线程回调循环。

第三点非常重要：规划计算、状态机、碰撞检查、里程计回调和地图回调默认都在同一线程串行执行。一次耗时较长的轨迹优化会推迟其他回调处理，包括安全检查和新里程计接收。

---

## 7. 重规划状态机 EGOReplanFSM

## 7.1 六个状态

```text
INIT
WAIT_TARGET
GEN_NEW_TRAJ
REPLAN_TRAJ
EXEC_TRAJ
EMERGENCY_STOP
```

可以用下面的简化状态图理解：

```text
                  收到目标
INIT ─────────→ WAIT_TARGET ─────────→ GEN_NEW_TRAJ
  │                                      │
  │ 预设目标模式                          │ 规划成功
  └──────────────────────────────────────┤
                                         ↓
                                    EXEC_TRAJ
                                    │   │   │
                         正常重规划 │   │   │ 到达终点
                                    ↓   │   ↓
                              REPLAN_TRAJ│ WAIT_TARGET
                                    │   │
                                    └───┘
                           预测即将碰撞且重规划失败
                                         ↓
                                  EMERGENCY_STOP
                                         ↓
                                  GEN_NEW_TRAJ
```

## 7.2 INIT：等待系统就绪

`INIT` 主要等待：

- 已收到里程计；
- 已有触发条件。

手动目标模式下，用户发送目标后会设置触发标志。

预设航点模式在初始化过程中会直接准备全局目标，并切换到 `GEN_NEW_TRAJ`，因此它走的状态路径与手动模式略有不同。

## 7.3 WAIT_TARGET：等待任务目标

系统已经具备规划条件，但尚无目标时停留在这里。

一旦 `have_target_` 为真，就转入 `GEN_NEW_TRAJ`。

## 7.4 GEN_NEW_TRAJ：从当前里程计状态生成新轨迹

该状态通常用于：

- 第一次收到目标；
- 紧急停止后重新起步；
- 没有可复用的旧轨迹。

规划起点来自当前里程计：

```text
位置：odom_pos_
速度：odom_vel_
加速度：通常设为零
```

它会强制使用多项式轨迹作为 B 样条初始化。如果连续失败，会进一步启用带随机中间点的初始化，以尝试跳出局部最优或困难环境。

## 7.5 REPLAN_TRAJ：基于当前轨迹重规划

执行中的无人机不能简单把当前里程计状态当成规划起点，因为新规划需要时间，且控制器在此期间仍沿旧轨迹飞行。

因此重规划时会从当前局部轨迹上选取稍后的状态作为新轨迹起点，使新旧轨迹衔接更自然。

重规划采用逐级回退策略：

```text
1. 优先复用当前局部轨迹进行初始化
2. 失败后改用普通多项式初始化
3. 再失败后使用带随机扰动的多项式初始化
```

这体现了一个实用思想：正常情况下追求轨迹连续性，困难情况下优先保证能找到可行解。

## 7.6 EXEC_TRAJ：执行轨迹并决定是否重规划

轨迹执行过程中，状态机会检查：

- 是否已经到达轨迹终点；
- 是否距离终点很近；
- 是否仍离本段轨迹起点很近；
- 是否需要继续滚动规划。

主要判断逻辑是：

```text
轨迹已结束
    → 清除目标，回到 WAIT_TARGET

距离最终目标小于 thresh_no_replan
    → 暂不重规划，避免终点附近反复抖动

距离当前轨迹起点小于 thresh_replan
    → 暂不重规划，避免刚发布轨迹就立刻替换

其他情况
    → 转入 REPLAN_TRAJ
```

因此 EGO-Planner 并不是只在碰撞时才重规划，而是持续执行“滚动局部规划”。

## 7.7 EMERGENCY_STOP：紧急停止

当安全检查发现即将碰撞，且立即重规划失败、剩余安全时间又过短时，状态机会发布紧急停止轨迹。

紧急停止轨迹由多个相同控制点构成：

```text
P0 = P1 = P2 = P3 = P4 = P5 = 当前停止位置
```

这相当于要求无人机尽快停在某个固定位置。

当里程计速度下降到约 `0.1 m/s` 以下后，状态机重新进入 `GEN_NEW_TRAJ`，尝试恢复规划。

---

## 8. 目标输入模式

状态机定义了三种类型：

```text
MANUAL_TARGET = 1
PRESET_TARGET = 2
REFENCE_PATH = 3
```

其中 `REFENCE_PATH` 名称本身存在拼写问题，而且当前初始化逻辑没有完整支持这种模式。

## 8.1 手动目标模式

手动目标通常来自：

```text
/waypoint_generator/waypoints
```

消息类型为：

```text
nav_msgs/Path
```

回调会读取第一项姿态的位置作为目标。

当前代码会强制把手动目标高度设为：

```text
z = 1.0 m
```

也就是说，即使用户在 RViz 中输入了不同的高度，规划目标仍会被改成 1 米。调试三维飞行时要特别注意。

## 8.2 预设航点模式

预设航点由启动文件中的参数给出：

```xml
<param name="fsm/waypoint_num" ... />
<param name="fsm/waypoint0_x" ... />
<param name="fsm/waypoint0_y" ... />
<param name="fsm/waypoint0_z" ... />
```

状态机读取这些点，并让规划管理器生成一条穿过航点的全局多项式参考轨迹。

航点被保存于固定大小数组：

```cpp
double waypoints_[50][3];
```

源码未检查 `waypoint_num` 是否超过 50，也没有充分检查是否为 0。因此修改预设航点参数时必须保证：

```text
1 <= waypoint_num <= 50
```

---

## 9. 全局轨迹与局部轨迹

这是理解该包最关键的一组概念。

## 9.1 全局轨迹不是最终避障轨迹

收到最终目标或一组预设航点后，规划器首先生成全局多项式轨迹。

它主要用于：

- 表示任务的总体前进方向；
- 在全局轨迹上跟踪规划进度；
- 为每次局部规划选择局部目标；
- 判断何时接近最终目标。

这条全局轨迹**并不负责绕开障碍物**。

因此，不应因为 RViz 中全局参考线穿过障碍物就直接判断规划器失效。真正用于飞行和避障的是不断更新的局部 B 样条。

## 9.2 局部轨迹是真正执行的轨迹

局部轨迹由 `EGOPlannerManager::reboundReplan()` 生成。

它具有：

- 有限的局部规划范围；
- 障碍物约束；
- 平滑性约束；
- 速度和加速度可行性约束；
- 可随无人机运动不断更新。

发布给 `traj_server` 的正是这条局部 B 样条。

## 9.3 局部目标如何选取

状态机从上次全局轨迹进度开始向前采样，直到满足：

```text
局部目标与当前规划起点的距离 >= planning_horizon
```

若全局轨迹剩余距离不足，则直接使用最终目标。

这个策略把一个很长的全局任务分解成许多局部问题：

```text
当前位置 → 前方局部目标
当前位置 → 更前方局部目标
当前位置 → 更前方局部目标
...
当前位置 → 最终目标
```

## 9.4 局部目标速度

当局部目标仍远离最终终点时，目标速度取自全局参考轨迹，使无人机保持向前运动。

当已经接近最终目标，且剩余距离不足以继续高速飞行时，局部目标速度会设为零。

判断中使用了类似制动距离的概念：

```text
stopping_distance ≈ max_vel² / (2 × max_acc)
```

这让末端轨迹更倾向于平稳停车。

---

## 10. EGOPlannerManager：规划算法组合层

## 10.1 初始化

规划管理器初始化时会：

1. 读取最大速度、最大加速度、控制点间距等参数；
2. 创建并初始化 `GridMap`；
3. 创建 `BsplineOptimizer`；
4. 将地图对象传给优化器；
5. 创建 A* 搜索器；
6. 为 A* 分配三维搜索节点池；
7. 初始化全局和局部轨迹容器。

## 10.2 局部重规划主流程

`reboundReplan()` 是整个局部规划过程的核心。

可概括为：

```text
输入：
  起点位置、速度、加速度
  局部目标位置、速度
  是否强制多项式初始化
  是否使用随机初始化

步骤：
  1. 检查起点与目标是否过近
  2. 估计 B 样条时间间隔 ts
  3. 生成初始路径
  4. 将初始路径参数化为 B 样条控制点
  5. 利用 A* 初始化障碍物附近控制点的绕障方向
  6. 执行 rebound 优化
  7. 检查速度和加速度可行性
  8. 必要时重新分配时间并细化轨迹
  9. 更新 LocalTrajData
```

## 10.3 为什么需要初始化轨迹

B 样条优化属于非线性优化问题。优化器需要一个初始解，初始解的形状会显著影响最终结果。

当前代码支持两类主要初始化：

### 多项式初始化

使用起点状态和局部目标状态构造一条平滑多项式曲线，再从曲线上采样并转换为 B 样条控制点。

适合：

- 第一次规划；
- 当前轨迹不可复用；
- 需要重新从当前状态出发。

### 当前轨迹初始化

使用旧局部轨迹的剩余部分，再连接到新局部目标。

适合：

- 正常滚动重规划；
- 希望新旧轨迹尽量连续；
- 减少轨迹突然改变。

### 随机初始化

连续规划失败时，在初始多项式中加入随机偏移，以改变优化初值。

这不是随机飞行，而是给优化器提供不同的搜索起点，尝试摆脱失败的局部最优。

## 10.4 控制点间距与时间间隔

初始时间间隔大致根据以下关系设置：

```text
ts ≈ control_points_distance / max_velocity × 1.2
```

控制点过密：

- 优化变量增多；
- 计算量增大；
- 轨迹表达更灵活。

控制点过疏：

- 计算更快；
- 但难以表达复杂绕障形状；
- 碰撞检查和动力学约束表达更粗糙。

## 10.5 A* 在这里扮演什么角色

当前 EGO-Planner 并不是先运行 A* 生成完整路径，再沿路径拟合轨迹。

它会分析初始 B 样条控制点与障碍物的关系，并在需要时调用 A* 找到绕障方向，为优化器构造引导信息。

因此 A* 更像是“局部避障提示器”，而不是最终轨迹生成器。

## 10.6 rebound 优化

优化器通过调整 B 样条控制点，综合考虑：

- 平滑性；
- 与障碍物的距离；
- 起点和终点约束；
- 控制点分布；
- 动力学可行性。

“rebound” 可以理解为：当轨迹控制点进入障碍物或距离障碍过近时，利用障碍物边界和搜索方向将控制点向安全区域反弹，再继续优化。

## 10.7 动力学可行性检查

优化完成并不代表轨迹一定能执行。

规划器还会检查：

- 最大速度是否超限；
- 最大加速度是否超限。

若超限，常见处理不是立即改变几何路径，而是先延长轨迹时间。相同空间曲线在更长时间内执行，速度和加速度会下降。

随后规划器会重新参数化并细化轨迹。

---

## 11. 轨迹数据容器 plan_container.hpp

## 11.1 LocalTrajData

`LocalTrajData` 保存当前执行中的局部轨迹：

```text
traj_id_       轨迹编号
duration_      轨迹总时长
start_time_    轨迹起始 ROS 时间
start_pos_     轨迹起点
position_traj_ 位置 B 样条
velocity_traj_ 速度 B 样条
acceleration_traj_ 加速度 B 样条
```

速度和加速度轨迹由位置 B 样条求导得到。

`traj_id_` 会随新轨迹更新，用于下游区分轨迹版本。

## 11.2 GlobalTrajData

`GlobalTrajData` 保存全局多项式参考轨迹及其持续时间、起始时间和局部片段相关信息。

当前实际运行中，最重要的是：

- 全局多项式轨迹；
- 全局轨迹总时间；
- 当前局部规划进度；
- 上一次局部目标对应的全局时间。

该类还包含把局部 B 样条拼接回全局轨迹的接口，但当前工作空间中没有找到 `setLocalTraj()` 的实际调用。阅读时应把这部分视为遗留或未启用设计，不要误以为它参与了当前主流程。

---

## 12. 全局参考轨迹生成

规划管理器支持：

- 从当前位置到单一目标生成全局轨迹；
- 经过多个预设航点生成全局轨迹。

## 12.1 中间点插入

若相邻目标点距离过长，代码会插入中间点。

这样做可以：

- 避免单段多项式跨度过大；
- 让全局轨迹参数化更稳定；
- 为局部目标选取提供更合理的参考。

## 12.2 时间分配

每段时间主要按：

```text
segment_time ≈ segment_distance / max_velocity
```

分配。

首段和末段通常会分配更长时间，以便从当前速度平稳过渡并在终点减速。

## 12.3 最小 snap 轨迹

对于多个航点，代码调用最小 snap 多项式轨迹生成方法。

Snap 是位置关于时间的四阶导数：

```text
位置 → 速度 → 加速度 → jerk → snap
```

最小化 snap 常用于四旋翼轨迹生成，因为它能够产生较平滑、适合飞行器执行的参考轨迹。

---

## 13. B 样条消息 Bspline.msg

消息定义为：

```text
int32 order
int64 traj_id
time start_time
float64[] knots
geometry_msgs/Point[] pos_pts
float64[] yaw_pts
float64 yaw_dt
```

### 字段解释

| 字段           | 含义                               |
| -------------- | ---------------------------------- |
| `order`      | B 样条阶数，当前通常发布为 3       |
| `traj_id`    | 轨迹编号                           |
| `start_time` | 轨迹开始执行时间                   |
| `knots`      | 完整节点向量                       |
| `pos_pts`    | 位置 B 样条控制点                  |
| `yaw_pts`    | 偏航角控制点，当前未实际使用       |
| `yaw_dt`     | 偏航角样条时间间隔，当前未实际使用 |

规划节点发布时主要填充：

```text
order
traj_id
start_time
knots
pos_pts
```

偏航角相关字段当前没有参与主流程。`traj_server` 中虽然保留了构造偏航 B 样条的注释代码，但实际偏航角是根据未来位置方向在线计算的。

---

## 14. traj_server：从轨迹到控制指令

## 14.1 订阅新轨迹

收到 `planning/bspline` 后，`traj_server` 会：

1. 将 `pos_pts` 转换为 Eigen 控制点矩阵；
2. 构造位置 B 样条；
3. 使用消息中的完整节点向量替换默认节点；
4. 对位置轨迹求导得到速度轨迹；
5. 再求导得到加速度轨迹；
6. 记录轨迹起始时间、持续时间和轨迹编号。

## 14.2 100 Hz 轨迹采样

节点创建周期为 `0.01 s` 的定时器，即理论上以 100 Hz 发布控制参考。

当前轨迹时间为：

```text
t_cur = ros::Time::now() - start_time
```

当：

```text
0 <= t_cur <= duration
```

节点分别计算：

```text
position(t_cur)
velocity(t_cur)
acceleration(t_cur)
```

并写入 `quadrotor_msgs/PositionCommand`。

轨迹结束后，它会持续发布终点位置和零速度、零加速度，使无人机保持在终点。

## 14.3 偏航角计算

当前偏航角不是由规划器直接给出，而是让机头朝向轨迹前方某一时刻的位置。

基本思想：

```text
当前期望位置 p(t)
前视期望位置 p(t + time_forward)
方向向量 d = p(t + time_forward) - p(t)
yaw = atan2(d_y, d_x)
```

代码还会：

- 限制最大偏航变化率；
- 处理 `-π` 与 `π` 附近的角度跳变；
- 对偏航角和偏航角速度做简单滤波。

## 14.4 PositionCommand

发布内容包括：

```text
位置
速度
加速度
偏航角
偏航角速度
轨迹编号
轨迹状态
```

启动文件通常把 `traj_server` 内部发布的绝对话题：

```text
/position_cmd
```

重映射到：

```text
/planning/pos_cmd
```

随后由 `so3_control` 等控制节点订阅。

## 14.5 增益字段

`traj_server` 会将 `PositionCommand` 中的位置和速度增益数组填为零。

这通常表示不在轨迹服务器中覆盖控制器自身配置的增益，而是让控制器使用其内部参数。

---

## 15. 碰撞检查与安全逻辑

状态机以 `0.05 s` 周期创建安全检查定时器，即理论频率 20 Hz。

## 15.1 检查哪一段轨迹

安全检查不会总是遍历整条局部轨迹。

当无人机尚未执行到轨迹后段时，通常只检查到约前 `2/3` 的范围。这是因为后续滚动重规划会继续更新远端轨迹，优先保障即将执行的部分更有意义。

## 15.2 检查方式

代码沿局部位置 B 样条按固定时间步长采样，并调用地图的膨胀占据查询：

```text
getInflateOccupancy(position)
```

使用膨胀地图意味着检查时已经考虑无人机尺寸和安全距离，而不是只把无人机当成数学质点。

## 15.3 发现碰撞后的处理

```text
发现未来轨迹碰撞
        ↓
立即尝试从当前轨迹重规划
        ↓
成功：发布新轨迹并继续执行
失败：
    若距离碰撞时间小于 emergency_time
        → EMERGENCY_STOP
    否则
        → REPLAN_TRAJ，继续尝试
```

这是一种分级处理：

- 有时间时继续寻找新轨迹；
- 没有时间时先停车。

---

## 16. ROS 话题接口

以下名称结合源码与启动文件重映射理解。

## 16.1 ego_planner_node 订阅

| 话题                              | 类型                   | 用途                    |
| --------------------------------- | ---------------------- | ----------------------- |
| `/odom_world`                   | `nav_msgs/Odometry`  | 当前无人机位置与速度    |
| `/waypoint_generator/waypoints` | `nav_msgs/Path`      | 手动目标输入            |
| 地图相关话题                      | 点云、深度图、里程计等 | 由内部 `GridMap` 订阅 |

在仿真启动文件中，`/odom_world` 通常被重映射为：

```text
/visual_slam/odom
```

## 16.2 ego_planner_node 发布

| 话题                       | 类型                     | 用途                   |
| -------------------------- | ------------------------ | ---------------------- |
| `/planning/bspline`      | `ego_planner/Bspline`  | 发布局部 B 样条        |
| `/planning/data_display` | `ego_planner/DataDisp` | 调试数据               |
| 可视化话题                 | Marker 等                | 显示轨迹、目标和控制点 |

## 16.3 traj_server 订阅与发布

| 方向 | 话题                 | 类型                               | 用途           |
| ---- | -------------------- | ---------------------------------- | -------------- |
| 订阅 | `planning/bspline` | `ego_planner/Bspline`            | 接收新局部轨迹 |
| 发布 | `/position_cmd`    | `quadrotor_msgs/PositionCommand` | 发布控制参考   |

注意：

- `planning/bspline` 是相对话题名；
- `/position_cmd` 是绝对话题名；
- 启动文件通常重映射 `/position_cmd` 为 `/planning/pos_cmd`。

命名空间和重映射错误是“规划器有轨迹但控制器没收到命令”的常见原因。

---

## 17. 启动文件解读

## 17.1 advanced_param.xml

这是规划器的主要参数集合，启动：

```text
ego_planner_node
traj_server
```

并配置：

- FSM 参数；
- 地图尺寸与分辨率；
- 相机或点云输入；
- 膨胀距离；
- 规划最大速度、加速度；
- B 样条控制点间距；
- 优化器权重；
- 可行性检查参数；
- 里程计和控制指令重映射。

## 17.2 simulator.xml

它组合了仿真所需的多个包，大体包括：

```text
随机地图 / mockamap
四旋翼动力学仿真器
SO3 控制器
里程计可视化
局部感知 / 点云渲染
```

## 17.3 run_in_sim.launch

主要特点：

- 使用手动目标模式；
- 包含高级规划参数；
- 包含仿真器；
- 启动 waypoint generator；
- 默认不额外包含独立 RViz 启动文件。

## 17.4 simple_run.launch

主要特点：

- 使用预设航点模式；
- 包含高级规划参数；
- 包含仿真器；
- 启动 waypoint generator；
- 包含 RViz。

两者都用于仿真，只是目标输入方式和默认可视化组合不同。

---

## 18. 关键参数说明

以下参数名称以当前源码为准。

## 18.1 FSM 参数

| 参数                     | 含义                           | 调大后的典型影响       |
| ------------------------ | ------------------------------ | ---------------------- |
| `fsm/flight_type`      | 目标模式                       | 决定手动目标或预设航点 |
| `fsm/thresh_replan`    | 离局部轨迹起点多远后允许重规划 | 重规划启动更晚         |
| `fsm/thresh_no_replan` | 距最终目标多近时停止滚动重规划 | 更早进入终点稳定阶段   |
| `fsm/planning_horizon` | 局部规划空间范围               | 看得更远，但优化更重   |
| `fsm/emergency_time`   | 预计碰撞剩余时间阈值           | 更早触发紧急停止       |
| `fsm/waypoint_num`     | 预设航点数                     | 必须与航点参数一致     |

当前源码读取的紧急时间参数名称末尾带下划线：

```text
fsm/emergency_time_
```

修改启动参数时必须与源码读取名称一致。

## 18.2 Manager 参数

| 参数                                | 含义                                  |
| ----------------------------------- | ------------------------------------- |
| `manager/max_vel`                 | 最大速度                              |
| `manager/max_acc`                 | 最大加速度                            |
| `manager/max_jerk`                | 最大 jerk，当前主流程中基本未实际使用 |
| `manager/control_points_distance` | B 样条控制点期望间距                  |
| `manager/feasibility_tolerance`   | 动力学可行性容差                      |
| `manager/planning_horizon`        | 管理器侧规划范围参数                  |

需要注意，`planning_horizon` 在 FSM 和 Manager 中分别读取。调参时应检查两个值是否保持一致，而不是只修改其中一个。

## 18.3 地图参数

常见关键参数：

```text
map/resolution
map/map_size_x
map/map_size_y
map/map_size_z
map/local_update_range_x
map/local_update_range_y
map/local_update_range_z
map/obstacles_inflation
map/virtual_ceil_height
```

它们决定：

- 地图精细程度；
- 局部感知更新范围；
- 安全膨胀距离；
- 可飞高度范围；
- 地图内存与计算开销。

## 18.4 优化器参数

优化器参数主要控制各目标项权重，例如：

```text
optimization/lambda_smooth
optimization/lambda_collision
optimization/lambda_feasibility
optimization/lambda_fitness
optimization/dist0
```

一般规律：

- `lambda_collision` 太小：可能更贴近障碍或碰撞；
- `lambda_collision` 太大：轨迹可能过度避障、难以收敛；
- `lambda_smooth` 太大：轨迹更平滑，但可能不愿绕过狭窄区域；
- `lambda_feasibility` 太小：速度和加速度更容易超限；
- `dist0` 增大：倾向于与障碍保持更大距离。

调参必须结合地图分辨率、膨胀距离和飞行器动力学，不能只看单个权重。

---

## 19. 默认仿真数据链

根据当前启动文件，可以把完整闭环理解为：

```text
waypoint_generator / 预设航点
        ↓
ego_planner_node
        ↓ /planning/bspline
traj_server
        ↓ /planning/pos_cmd
so3_control
        ↓ /so3_cmd
so3_quadrotor_simulator
        ↓ /visual_slam/odom
ego_planner_node + 地图 + 可视化
```

局部感知节点根据地图和无人机位姿生成传感器观测，再交给 `GridMap` 更新占据地图。

这是一个闭环：

```text
规划 → 控制 → 动力学运动 → 里程计与感知 → 地图更新 → 再规划
```

---

## 20. 推荐源码阅读顺序

### 第一遍：建立运行链路

```text
1. src/ego_planner_node.cpp
2. include/plan_manage/ego_replan_fsm.h
3. src/ego_replan_fsm.cpp
4. launch/run_in_sim.launch
5. launch/advanced_param.xml
```

目标：理解节点怎么启动、状态机如何切换、目标和里程计从哪里来。

### 第二遍：理解规划算法组合

```text
1. include/plan_manage/planner_manager.h
2. src/planner_manager.cpp
3. include/plan_manage/plan_container.hpp
4. bspline_opt 功能包
5. path_searching 功能包
6. plan_env 功能包
```

目标：理解初始轨迹、A* 引导、B 样条优化、可行性检查和轨迹保存。

### 第三遍：理解控制接口

```text
1. msg/Bspline.msg
2. src/traj_server.cpp
3. quadrotor_msgs/PositionCommand.msg
4. uav_simulator/so3_control
```

目标：理解局部轨迹如何变成控制器每一时刻的参考指令。

---

## 21. 调试方法

## 21.1 确认节点

```bash
rosnode list
```

重点确认规划器、轨迹服务器、地图、控制器和仿真器均已启动。

## 21.2 确认话题连接

```bash
rostopic info /planning/bspline
rostopic info /planning/pos_cmd
rostopic info /visual_slam/odom
```

预期：

- `/planning/bspline` 有规划节点发布、`traj_server` 订阅；
- `/planning/pos_cmd` 有 `traj_server` 发布、控制器订阅；
- 里程计有仿真器发布、规划器订阅。

## 21.3 检查发布频率

```bash
rostopic hz /planning/pos_cmd
rostopic hz /visual_slam/odom
```

`/planning/pos_cmd` 理论上接近 100 Hz。

`/planning/bspline` 不应固定为高频，它只在生成新局部轨迹时发布。

## 21.4 检查参数是否进入正确命名空间

```bash
rosparam get /ego_planner_node
```

由于规划节点使用私有 NodeHandle，参数命名空间错误会导致读取默认值 `-1`。许多核心参数默认值无效，出现这种情况后规划行为可能非常异常。

## 21.5 查看状态机输出

源码会打印状态切换、规划成功或失败、碰撞和紧急停止等信息。

若不断在以下状态间切换：

```text
GEN_NEW_TRAJ ↔ REPLAN_TRAJ
```

通常应检查：

- 地图是否把起点或目标标为占据；
- 最大速度和最大加速度是否合理；
- 控制点间距是否过大；
- 障碍膨胀是否过大；
- 局部目标是否位于地图外；
- 优化器参数是否难以收敛。

---

## 22. 常见现象与定位思路

## 22.1 点击目标后没有反应

检查：

```text
flight_type 是否为手动目标模式
/waypoint_generator/waypoints 是否有消息
nav_msgs/Path 是否至少包含一个 pose
里程计是否已经收到
目标是否被强制改为 z=1.0 后落入障碍
```

## 22.2 有 B 样条但无人机不动

检查：

```text
traj_server 是否启动
/planning/pos_cmd 是否发布
控制器是否订阅正确话题
PositionCommand 消息定义是否来自同一工作空间版本
仿真器是否收到 so3_cmd
```

## 22.3 规划器频繁重规划

检查：

```text
thresh_replan 是否太小
planning_horizon 是否太短
地图是否因传感噪声不断变化
局部轨迹是否接近障碍
控制器跟踪误差是否过大
```

滚动重规划本身是正常行为，问题在于是否高频失败或导致轨迹明显抖动。

## 22.4 一直提示碰撞

检查：

```text
地图原点、坐标系和里程计坐标系是否一致
obstacles_inflation 是否过大
传感器点云中是否包含无人机自身
深度图和点云是否被同时输入并产生冲突
目标或轨迹是否超出地图边界
```

## 22.5 终点附近来回摆动

检查：

```text
thresh_no_replan
最大速度与最大加速度
终点目标速度是否为零
控制器增益和跟踪误差
局部目标是否已经正确切换为最终目标
```

## 22.6 偏航角异常跳变

检查：

```text
time_forward 参数
轨迹前方点与当前点是否几乎重合
ROS 时间是否暂停或跳变
轨迹是否在起始时间之前被采样
```

---

## 23. 当前源码中的潜在风险与遗留逻辑

本节不是说这些问题一定会在默认演示中触发，而是指出二次开发时值得优先处理的工程风险。

## 23.1 回调单线程执行

节点使用 `ros::spin()`。

轨迹优化耗时时，里程计、地图和安全检查回调会被延迟。复杂环境或计算资源有限时，这会削弱实时性。

可考虑的改进方向：

- 使用异步 spinner；
- 将耗时规划放入独立线程；
- 明确保护共享轨迹和地图数据；
- 对规划耗时和回调延迟做统计。

## 23.2 参数默认值缺乏校验

许多参数读取失败时默认为 `-1`，但初始化后没有统一合法性检查。

例如：

```text
max_vel <= 0
max_acc <= 0
control_points_distance <= 0
planning_horizon <= 0
```

都可能导致除零、循环异常或规划失败。

## 23.3 手动目标回调未检查空 Path

回调直接访问：

```text
msg->poses[0]
```

若收到空 `nav_msgs/Path`，会发生越界访问。

## 23.4 预设航点边界不足

固定数组最大保存 50 个航点，但没有检查 `waypoint_num` 上限。

航点数为 0 时，后续代码也可能访问空容器首尾元素。

## 23.5 部分布尔成员可能未显式初始化

如触发、新目标和紧急状态相关标志并非都在 `init()` 中清晰初始化。默认流程可能恰好在使用前赋值，但二次开发或改变状态路径后容易出现未定义行为。

## 23.6 `planning_horizen_time_` 当前未实际参与主流程

源码读取了：

```text
fsm/planning_horizen_time
```

但当前主要逻辑没有使用它。修改该参数通常不会改变规划效果。

其中 `horizen` 也是 `horizon` 的拼写错误，后续整理接口时应谨慎兼容。

## 23.7 `max_jerk_` 当前基本未使用

管理器读取最大 jerk 参数，但局部轨迹主流程主要检查速度和加速度。不要仅修改 `max_jerk` 就期待轨迹 jerk 被严格限制。

## 23.8 DataDisp 内容未填充

`DataDisp.msg` 定义了多个数值字段，但当前状态机主要只更新消息时间戳。它更像未完成的调试接口。

## 23.9 偏航 B 样条字段未使用

`Bspline.msg` 中的 `yaw_pts` 和 `yaw_dt` 未进入当前实际控制链。

如要实现独立偏航规划，需要同时修改：

- 规划器消息填充；
- `traj_server` 偏航轨迹重建；
- 偏航角和偏航角速度采样逻辑；
- 控制器接口测试。

## 23.10 traj_server 缺少消息合法性检查

收到 B 样条消息后，代码没有系统检查：

- 控制点是否为空；
- 节点向量长度是否正确；
- 阶数是否合法；
- 起始时间是否合理；
- 新轨迹编号是否比当前轨迹更新。

异常或乱序消息可能覆盖正在执行的正确轨迹。

## 23.11 轨迹开始前仍可能发布零值命令

当当前时间早于轨迹 `start_time` 时，代码打印无效时间提示，但后续仍可能发布尚未正确赋值的命令。

更稳妥的策略是：

- 保持上一条有效命令；
- 或等待到起始时间再发布新轨迹命令；
- 或显式发布安全悬停状态。

## 23.12 偏航角时间差可能过小

偏航角速度计算依赖时间差。如果 ROS 时间暂停、回跳或两次定时器时间相同，理论上存在除零或数值异常风险。

## 23.13 全局局部拼接逻辑未启用

`GlobalTrajData` 中存在局部轨迹拼接接口，但当前主流程没有调用。相关代码可能是旧版本设计残留，修改前应先确认目标架构。

## 23.14 `getTrajByRadius()` 存在潜在除零风险

若某段长度小于采样距离：

```text
floor(segment_length / distance_point) = 0
```

后续用该段数进行除法时可能出错。该函数当前似乎不是主路径，但若重新启用应先修复。

## 23.15 全局轨迹不避障是设计行为

全局参考轨迹可能穿过障碍物。这不是代码缺陷，而是当前架构选择。

但它也意味着：

- 局部规划范围必须足够；
- 地图必须及时更新；
- 狭长死胡同中可能缺少真正全局层面的绕行能力。

## 23.16 消息同名版本兼容风险

该工作空间中的 `quadrotor_msgs` 位于：

```text
uav_simulator/Utils/quadrotor_msgs
```

其他工作空间也可能存在同名包和同名消息，但字段定义不同。ROS 1 会按消息 MD5 判断兼容性，同名并不代表可以直接互通。

跨工作空间连接规划器、控制器或 PX4 桥接节点时，应确认实际加载的是同一份消息定义。

## 23.17 启动文件可能同时配置深度图和点云

启动文件注释提示深度图与点云输入通常二选一，但当前配置可能同时为两者设置话题。结合 `plan_env` 的地图更新逻辑，这可能造成数据源竞争或地图状态不稳定。

---

## 24. 二次开发建议

## 24.1 想接入真实无人机

优先替换和确认：

```text
里程计话题
点云 / 深度图话题
坐标系定义
时间同步
PositionCommand 到真实控制器的桥接
紧急停止行为
消息版本
```

不要直接把仿真中的 `/visual_slam/odom` 和 `/planning/pos_cmd` 名称照搬到真实系统，而应按实际定位和控制链重映射。

## 24.2 想接入 PX4

需要明确 `traj_server` 输出的是高层轨迹参考，不一定能直接作为 PX4 原生消息使用。

通常需要中间桥接层完成：

- ENU/NED 坐标转换；
- ROS/PX4 时间处理；
- 位置、速度、加速度设定值映射；
- yaw 与 yaw rate 映射；
- Offboard 模式与心跳维护；
- 失联和轨迹超时保护。

## 24.3 想增加真正全局避障

当前全局轨迹只是几何参考。

可以在更高层加入：

- 全局栅格 A*；
- 拓扑路径规划；
- ESDF 上的全局引导路径；
- 多走廊候选路径。

然后把全局避障路径作为局部目标选取和局部 B 样条优化的参考。

## 24.4 想提高实时性

建议先测量：

```text
每次 reboundReplan 耗时
A* 耗时
B 样条优化耗时
地图更新耗时
状态机与安全回调延迟
```

再考虑：

- 减少不必要的控制点；
- 调整局部规划范围；
- 限制优化迭代；
- 独立规划线程；
- 使用多线程 spinner；
- 对新旧地图和轨迹数据加锁或使用快照。

## 24.5 想提高安全性

建议增加：

- 输入消息完整性检查；
- 参数合法性检查；
- 轨迹编号单调性检查；
- 轨迹超时监控；
- 地图过期监控；
- 里程计过期监控；
- 控制器确认与急停接口；
- 规划耗时超过安全余量时的降级策略。

---

## 25. 建议的学习实验

### 实验 1：观察滚动重规划

1. 启动仿真；
2. 发布一个较远目标；
3. 在 RViz 中观察局部 B 样条不断更新；
4. 使用 `rostopic hz /planning/bspline` 观察其非固定发布频率；
5. 对照 `EXEC_TRAJ` 中的重规划阈值。

目标：理解局部轨迹不是一次生成后执行到底。

### 实验 2：修改 planning_horizon

分别尝试较小和较大的局部规划范围。

观察：

- 每次局部轨迹长度；
- 计算耗时；
- 绕障提前量；
- 重规划频率；
- 狭窄环境成功率。

### 实验 3：修改障碍膨胀距离

观察轨迹与障碍物的距离以及狭窄通道可通过性。

目标：理解“安全距离”和“可通行空间”之间的权衡。

### 实验 4：触发紧急停止

在保证仿真安全的前提下，让新障碍物出现在未来轨迹附近。

观察：

```text
碰撞检查
立即重规划
失败后的 EMERGENCY_STOP
速度降低后的重新规划
```

### 实验 5：追踪完整消息链

依次查看：

```bash
rostopic echo /planning/bspline
rostopic echo /planning/pos_cmd
rostopic echo /so3_cmd
rostopic echo /visual_slam/odom
```

目标：建立从规划结果到动力学反馈的闭环认识。

---

## 26. 对核心流程的伪代码总结

```cpp
while (ros_is_running) {
  receive_odometry_and_map();

  switch (state) {
    case INIT:
      wait_until_system_ready();
      break;

    case WAIT_TARGET:
      wait_until_target_received();
      break;

    case GEN_NEW_TRAJ:
      select_local_target_on_global_reference();
      plan_from_odometry_state();
      if (success) {
        publish_bspline();
        state = EXEC_TRAJ;
      }
      break;

    case REPLAN_TRAJ:
      select_local_target_on_global_reference();
      plan_from_future_state_on_current_trajectory();
      if (success) {
        publish_bspline();
        state = EXEC_TRAJ;
      }
      break;

    case EXEC_TRAJ:
      if (trajectory_finished)
        state = WAIT_TARGET;
      else if (rolling_replan_condition_met)
        state = REPLAN_TRAJ;
      break;

    case EMERGENCY_STOP:
      publish_stop_trajectory_once();
      if (vehicle_has_stopped)
        state = GEN_NEW_TRAJ;
      break;
  }

  if (future_trajectory_collides) {
    if (!immediate_replan_success) {
      if (collision_is_imminent)
        state = EMERGENCY_STOP;
      else
        state = REPLAN_TRAJ;
    }
  }
}
```

`traj_server` 的伪代码则是：

```cpp
on_new_bspline(msg) {
  reconstruct_position_bspline(msg);
  velocity_bspline = derivative(position_bspline);
  acceleration_bspline = derivative(velocity_bspline);
}

every_0_01_second {
  t = now - trajectory_start_time;
  sample_position_velocity_acceleration(t);
  calculate_yaw_from_forward_direction(t);
  publish_position_command();
}
```

---

## 27. 最终总结

`plan_manage` 的核心价值不是某一个单独算法，而是把整个局部规划系统组织成可持续运行的闭环。

需要牢牢记住以下几点：

1. 目录名是 `plan_manage`，ROS 包名是 `ego_planner`。
2. `ego_planner_node` 负责规划决策，`traj_server` 负责高频轨迹采样。
3. 全局多项式轨迹主要提供方向和进度参考，并不直接负责避障。
4. 真正执行的是不断滚动更新的局部 B 样条。
5. 状态机决定何时首次规划、何时重规划、何时等待、何时急停。
6. `EGOPlannerManager` 把多项式初始化、旧轨迹复用、A* 引导、B 样条优化和动力学检查串联起来。
7. 碰撞检查会先尝试立即重规划，时间不足时才进入紧急停止。
8. 仿真链路最终形成“规划—控制—动力学—感知—地图—再规划”的闭环。
9. 当前源码中存在未使用参数、遗留接口、输入检查不足和单线程实时性等工程风险。
10. 二次开发时，坐标系、时间、话题重映射、消息版本和安全降级与规划算法本身同样重要。

理解这个包之后，再阅读 `plan_env`、`path_searching` 和 `bspline_opt`，会更容易看清每个底层模块为什么存在、由谁调用、输出最终流向哪里。
