# `so3_control` 功能包详细学习说明

> 适用源码：`/home/yxc/Desktop/ego-planner_ws/src/uav_simulator/so3_control`
>
> ROS 功能包名称：`so3_control`
>
> 所属系统：EGO-Planner 四旋翼仿真控制链路

---

## 1. 一句话认识这个包

`so3_control` 将规划器给出的：

```text
期望位置
期望速度
期望加速度
期望偏航角
```

结合当前无人机状态，转换为：

```text
世界坐标系期望合力
期望机体姿态
SO(3) 姿态控制增益
辅助校正参数
```

并通过：

```text
quadrotor_msgs/SO3Command
```

发送给四旋翼动力学仿真器或真实飞控接口。

---

## 2. 它在 EGO-Planner 中的位置

完整链路：

```text
EGO-Planner
生成 B 样条轨迹
        |
        v
traj_server
100 Hz 采样轨迹
        |
        | PositionCommand
        v
so3_control
位置/速度/加速度外环
        |
        | SO3Command
        v
so3_quadrotor_simulator
SO(3) 姿态内环 + 电机混控 + 动力学
        |
        | Odometry + IMU
        v
so3_control 与规划器反馈
```

因此该包位于：

```text
轨迹规划
    和
四旋翼底层姿态/电机控制
```

之间。

---

## 3. “SO(3) 控制”到底指什么

SO(3) 是三维旋转矩阵集合：

```text
SO(3) = {R ∈ R³ˣ³ |
         RᵀR = I,
         det(R) = 1}
```

它描述刚体在三维空间中的姿态。

本工作区的控制分成两层：

```text
so3_control 包
  外层：位置误差 -> 期望合力和期望姿态

so3_quadrotor_simulator
  内层：当前姿态与期望姿态的 SO(3) 误差
        -> 力矩
        -> 四个电机转速
```

严格来说，真正计算：

- SO(3) 姿态误差；
- 角速度误差；
- 控制力矩；

的代码位于 `so3_quadrotor_simulator`。

`so3_control` 主要负责几何控制的平移外环和期望姿态构造。

---

## 4. 目录结构

```text
so3_control/
├── CMakeLists.txt
├── package.xml
├── nodelet_plugin.xml
├── mainpage.dox
├── include/so3_control/
│   └── SO3Control.h
├── src/
│   ├── SO3Control.cpp
│   ├── so3_control_nodelet.cpp
│   └── control_example.cpp
└── config/
    ├── gains.yaml
    ├── gains_hummingbird.yaml
    ├── gains_pelican.yaml
    ├── corrections_hummingbird.yaml
    └── corrections_pelican.yaml
```

核心文件：

| 文件 | 职责 |
|---|---|
| `SO3Control.h/.cpp` | 平移外环控制律与期望姿态计算 |
| `so3_control_nodelet.cpp` | ROS Nodelet、参数、订阅、发布和状态管理 |
| `control_example.cpp` | 位置、速度、加速度控制示例 |
| `nodelet_plugin.xml` | Nodelet 插件注册 |
| `gains_*.yaml` | 外环与内环增益 |
| `corrections_*.yaml` | 推力和姿态修正 |

---

## 5. 构建产物

### 5.1 `SO3Control` 库

```cmake
add_library(
  SO3Control
  src/SO3Control.cpp
)
```

包含纯控制计算类。

### 5.2 Nodelet 库

```cmake
add_library(
  so3_control_nodelet
  src/so3_control_nodelet.cpp
)
```

链接：

```text
catkin 库
SO3Control
```

### 5.3 示例节点

```cmake
add_executable(
  control_example
  src/control_example.cpp
)
```

它只发布 `PositionCommand`，不直接调用 `SO3Control` 类。

---

## 6. 为什么默认以 Nodelet 运行

默认 launch：

```xml
<node pkg="nodelet"
      type="nodelet"
      args="standalone so3_control/SO3ControlNodelet"
      name="so3_control"/>
```

`standalone` 表示：

- 使用 Nodelet 插件接口；
- 但由独立 nodelet manager 进程承载；
- 不与其他 Nodelet 共享进程。

Nodelet 类通过：

```cpp
PLUGINLIB_EXPORT_CLASS(
    SO3ControlNodelet,
    nodelet::Nodelet);
```

导出。

插件描述：

```xml
<class
  name="so3_control/SO3ControlNodelet"
  type="SO3ControlNodelet"
  base_class_type="nodelet::Nodelet">
```

---

## 7. 关键依赖

| 依赖 | 用途 |
|---|---|
| Eigen3 | 三维向量、矩阵、四元数 |
| `roscpp` | ROS 接口 |
| `nodelet` | Nodelet 生命周期和插件 |
| `nav_msgs` | 当前里程计 |
| `sensor_msgs` | IMU 加速度 |
| `std_msgs` | 电机启停布尔命令 |
| `quadrotor_msgs` | PositionCommand、SO3Command、Corrections |
| `tf` | 从四元数提取当前 yaw |
| `cmake_utils` | 构建依赖，但当前源码未直接使用 |

`std_msgs` 和 `sensor_msgs` 在源码中直接使用，但 CMake 和 `package.xml` 没有明确列入依赖。

当前可能依靠其他包的传递依赖成功构建。

---

## 8. 编译配置

```cmake
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")
```

含义：

- C++11；
- Release 高优化；
- 常见警告；
- 保留调试符号。

CMake 文件前半部分还保留了大量 rosbuild 时代的注释代码。

这些不参与当前 catkin 构建。

---

## 9. ROS 接口总览

### 9.1 订阅

| 私有话题 | 类型 | 用途 |
|---|---|---|
| `~odom` | `nav_msgs/Odometry` | 当前位置、速度和 yaw |
| `~position_cmd` | `quadrotor_msgs/PositionCommand` | 期望轨迹状态 |
| `~motors` | `std_msgs/Bool` | 电机启停标志 |
| `~corrections` | `quadrotor_msgs/Corrections` | 推力与角度修正 |
| `~imu` | `sensor_msgs/Imu` | 当前线加速度 |

### 9.2 发布

| 私有话题 | 类型 | 用途 |
|---|---|---|
| `~so3_cmd` | `quadrotor_msgs/SO3Command` | 合力、姿态及内环参数 |

所有订阅都使用：

```cpp
ros::TransportHints().tcpNoDelay()
```

以减少 TCP 小消息延迟。

---

## 10. 默认话题重映射

`simulator.xml`：

```text
~odom
  -> /visual_slam/odom

~position_cmd
  -> /planning/pos_cmd

~so3_cmd
  -> /so3_cmd

~motors
  -> /motors

~corrections
  -> /corrections
```

仿真器：

```text
~cmd -> /so3_cmd
~odom -> /visual_slam/odom
```

于是形成闭环：

```text
/planning/pos_cmd
        |
        v
/so3_control
        |
        | /so3_cmd
        v
/quadrotor_simulator_so3
        |
        | /visual_slam/odom
        +--------------------+
                             |
                             v
                       /so3_control
```

---

## 11. `PositionCommand` 消息

字段：

```text
position
velocity
acceleration
yaw
yaw_dot
kx[3]
kv[3]
trajectory_id
trajectory_flag
```

控制器实际读取：

- `position`；
- `velocity`；
- `acceleration`；
- `yaw`；
- `yaw_dot`；
- `kx`；
- `kv`。

它不读取：

- `trajectory_id`；
- `trajectory_flag`；
- 消息时间戳；
- frame_id。

---

## 12. `SO3Command` 消息

字段：

```text
force
orientation
kR[3]
kOm[3]
aux
```

其中 `aux`：

```text
current_yaw
kf_correction
angle_corrections[2]
enable_motors
use_external_yaw
```

含义：

| 字段 | 用途 |
|---|---|
| `force` | 世界系期望合力 |
| `orientation` | 期望机体姿态 |
| `kR` | 姿态误差增益 |
| `kOm` | 角速度误差增益 |
| `current_yaw` | 外部 yaw 估计 |
| `kf_correction` | 螺旋桨推力系数修正 |
| `angle_corrections` | roll/pitch trim |
| `enable_motors` | 电机启停标志 |
| `use_external_yaw` | 内环是否替换当前 yaw |

---

## 13. `SO3Control` 类总览

公开接口：

```cpp
void setMass(double mass);
void setGravity(double g);
void setPosition(Vector3d position);
void setVelocity(Vector3d velocity);
void setAcc(Vector3d acc);

void calculateControl(
    des_pos,
    des_vel,
    des_acc,
    des_yaw,
    des_yaw_dot,
    kx,
    kv);

getComputedForce();
getComputedOrientation();
```

内部状态：

| 成员 | 含义 |
|---|---|
| `mass_` | 无人机质量 |
| `g_` | 重力加速度 |
| `pos_` | 当前世界系位置 |
| `vel_` | 当前世界系速度 |
| `acc_` | 当前 IMU 线加速度 |
| `force_` | 计算出的世界系期望合力 |
| `orientation_` | 计算出的期望姿态 |

---

## 14. 默认值

构造函数：

```cpp
mass_ = 0.5;
g_ = 9.81;
acc_.setZero();
```

但：

```text
pos_
vel_
force_
orientation_
```

没有显式初始化。

正常链路中，首次 odometry 到达后才会设置位置和速度。

---

## 15. NaN 表示“禁用该控制层”

控制器分别检查：

```cpp
flag_use_pos = des_pos 三维都不是 NaN;
flag_use_vel = des_vel 三维都不是 NaN;
flag_use_acc = des_acc 三维都不是 NaN;
```

因此可选择：

```text
位置控制：
  position 有效

速度控制：
  position = NaN
  velocity 有效

加速度控制：
  position = NaN
  velocity = NaN
  acceleration 有效
```

这是 `control_example.cpp` 展示的核心用法。

注意：

```text
只要一个分量是 NaN，
整个三维控制项都会被禁用。
```

不能只禁用单独一个坐标轴。

---

## 16. 标准外环控制思想

理想几何控制外环常写为：

```text
F =
  m g e3
  + Kx (xd - x)
  + Kv (vd - v)
  + m ad
```

其中：

- `m g e3`：重力补偿；
- `Kx(xd-x)`：位置反馈；
- `Kv(vd-v)`：速度反馈；
- `m ad`：期望加速度前馈。

本实现以此为基础，又增加一个与当前误差和加速度误差相关的自适应项。

---

## 17. 总误差 `totalError`

源码：

```cpp
totalError = 0;

if use_pos:
  totalError += des_pos - pos

if use_vel:
  totalError += des_vel - vel

if use_acc:
  totalError += des_acc - acc
```

它直接把：

- 位置误差，单位 m；
- 速度误差，单位 m/s；
- 加速度误差，单位 m/s²；

相加。

这在物理量纲上并不严格一致。

该值只用于后续计算经验增益 `ka`。

---

## 18. 自适应加速度增益 `ka`

每个轴：

```text
若 |totalError| > 3：
  ka = 0
否则：
  ka = 0.2 × |totalError|
```

因此：

```text
0 <= ka <= 0.6
```

但在刚超过 3 时会突然从接近 0.6 跳到 0。

其作用项：

```text
m × Ka × (ad - a)
```

可以理解为基于 IMU 加速度误差的附加反馈。

---

## 19. 当前实际合力公式

源码：

```text
F = m g e3
```

如果位置有效：

```text
F += Kx(xd - x)
```

如果速度有效：

```text
F += Kv(vd - v)
```

如果加速度有效：

```text
F += m Ka(ad - a) + m ad
```

完整表达：

```text
F =
  m g e3
  + Kx ep
  + Kv ev
  + m Ka ea
  + m ad
```

其中各项是否存在由 NaN 开关决定。

---

## 20. 增益矩阵

源码：

```cpp
kx.asDiagonal()
kv.asDiagonal()
ka.asDiagonal()
```

所以三个方向完全解耦：

```text
Kx = diag(kx_x, kx_y, kx_z)
Kv = diag(kv_x, kv_y, kv_z)
Ka = diag(ka_x, ka_y, ka_z)
```

可以为 z 轴设置更大增益，以抵抗重力和高度误差。

---

## 21. 合力的坐标系

`pos_`、`vel_` 和规划器输出都按世界坐标使用。

重力补偿：

```cpp
m * g * (0, 0, 1)
```

也在世界系。

因此 `force_` 是：

```text
世界坐标系中的期望总推力向量。
```

仿真器后续将其投影到当前机体 z 轴，得到总推力标量。

---

## 22. 所谓“45 度限制”

源码注释：

```cpp
// Limit control angle to 45 degree
```

但实际：

```cpp
double theta = M_PI / 2;
double c = cos(theta);
```

即：

```text
theta = 90°
c ≈ 0
```

判断：

```text
e3 · normalized(F) < 0
```

只在期望合力指向水平面下方时触发。

所以当前不是 45° 倾角限制，而更接近：

```text
不允许期望推力朝下。
```

---

## 23. 限制公式

先去掉重力：

```text
f = F - m g e3
```

若倾角超过限制，求比例 `s`：

```text
F_limited = s f + m g e3
```

并通过二次方程使最终合力方向落到圆锥边界。

系数：

```text
A = c²||f||² - fz²
B = 2(c²-1)fzmg
C = (c²-1)m²g²
```

当前 `c≈0` 时，该公式可能退化。

正常轨迹下期望合力通常向上，因此分支很少触发。

---

## 24. 从合力构造期望姿态

四旋翼机体 z 轴应沿期望推力方向：

```text
b3c = F / ||F||
```

若合力太小：

```text
b3c = (0,0,1)
```

根据期望 yaw 构造水平参考方向：

```text
b1d = (cos ψd, sin ψd, 0)
```

然后：

```text
b2c = normalize(b3c × b1d)
b1c = normalize(b2c × b3c)
```

最终：

```text
Rc = [b1c b2c b3c]
```

再转成四元数。

---

## 25. 这个姿态构造的几何意义

`b3c` 决定：

```text
推力方向
```

`b1d` 表达：

```text
期望机头水平朝向
```

通过两次叉乘构造正交基，得到同时尽量满足：

- 机体 z 轴产生所需合力；
- 机头朝向期望 yaw；

的姿态。

当无人机倾斜时，实际机体 x 轴在水平面的投影才近似对应期望 yaw。

---

## 26. 退化情况

若：

```text
b3c 与 b1d 平行或反平行
```

则：

```text
b3c × b1d = 0
```

对零向量执行：

```cpp
normalized()
```

会产生 NaN。

这可能发生在期望合力完全水平且方向正好等于期望机头方向时。

正常飞行中重力补偿使 `b3c` 通常带正 z 分量，降低了概率，但代码没有显式保护。

---

## 27. `des_yaw_dot` 当前没有使用

接口接收：

```cpp
des_yaw_dot
```

Nodelet 也从 `PositionCommand` 保存它。

但 `calculateControl()` 中没有任何地方使用。

因此：

```text
期望 yaw 影响期望姿态，
期望 yaw 角速度不会影响控制输出。
```

下游姿态内环的期望角速度被隐式视为 0。

---

## 28. Nodelet 初始化

`onInit()`：

```text
读取机体名称
设置 frame_id
读取质量
读取 yaw 模式
读取 kR、kOm、kx、kv
读取校正参数
读取初始悬停位置
创建发布器和订阅器
```

所有参数属于 Nodelet 私有命名空间。

---

## 29. 控制参数总览

| 参数 | 代码默认值 |
|---|---:|
| `quadrotor_name` | `quadrotor` |
| `mass` | `0.5` |
| `use_external_yaw` | `true` |
| `gains/rot/x,y,z` | `1.5, 1.5, 1.0` |
| `gains/ang/x,y,z` | `0.13, 0.13, 0.1` |
| `gains/kx/x,y,z` | `5.7, 5.7, 6.2` |
| `gains/kv/x,y,z` | `3.4, 3.4, 4.0` |
| `corrections/z,r,p` | `0,0,0` |
| `so3_control/init_state_x,y` | `0,0` |
| `so3_control/init_state_z` | `-10000` |

默认仿真会将质量改为：

```text
0.98 kg
```

---

## 30. 外环增益和内环增益

两组增益职责不同：

```text
kx, kv
  由 so3_control 使用
  位置和速度外环

kR, kOm
  原样写入 SO3Command
  由仿真器/飞控使用
  姿态和角速度内环
```

所以调参时应区分：

- 位置跟踪太软：调 `kx/kv`；
- 姿态跟踪太软：调 `kR/kOm`。

---

## 31. YAML 增益文件

`gains_hummingbird.yaml`：

```yaml
gains:
  pos: {x: 2.0, y: 2.0, z: 3.5}
  vel: {x: 1.8, y: 1.8, z: 2.0}
  rot: {x: 1.0, y: 1.0, z: 0.3}
  ang: {x: 0.07, y: 0.07, z: 0.02}
```

`gains_pelican.yaml`：

```yaml
gains:
  pos: {x: 5.0, y: 5.0, z: 15.0}
  vel: {x: 5.0, y: 5.0, z: 5.0}
  rot: {x: 3.5, y: 3.5, z: 1.0}
  ang: {x: 0.4, y: 0.4, z: 0.1}
```

---

## 32. YAML 与源码的关键命名错位

YAML 使用：

```text
gains/pos/x
gains/vel/x
```

源码读取：

```text
gains/kx/x
gains/kv/x
```

所以加载 YAML 后：

```text
rot 和 ang 会生效
pos 和 vel 不会被源码读取
```

位置和速度增益实际回退到代码默认值：

```text
kx = (5.7, 5.7, 6.2)
kv = (3.4, 3.4, 4.0)
```

除非 `PositionCommand` 中提供非零 `kx/kv`。

---

## 33. `PositionCommand` 如何覆盖增益

回调：

```cpp
if (cmd->kx 任一分量 > 1e-5)
  覆盖整个 kx

if (cmd->kv 任一分量 > 1e-5)
  覆盖整个 kv
```

`traj_server` 初始化：

```text
kx = (0,0,0)
kv = (0,0,0)
```

所以默认 EGO-Planner 轨迹命令不会覆盖 Nodelet 增益。

如果消息中只有一个轴为正，三个轴都会整体复制，包括其他可能为 0 或负的分量。

---

## 34. `PositionCommand` 回调

收到命令后：

1. 保存期望位置；
2. 保存期望速度；
3. 保存期望加速度；
4. 可选更新 `kx/kv`；
5. 保存 yaw 和 yaw_dot；
6. 标记命令已初始化、已更新；
7. 立即发布一条 `SO3Command`。

所以控制输出会被新轨迹命令直接触发。

---

## 35. Odometry 回调

回调读取：

```text
position
linear velocity
orientation -> yaw
```

并更新控制器当前位置和速度。

随后：

```text
如果已经收到过 PositionCommand：
  若本轮没有刚更新命令，则重新发布控制

否则如果配置了有效初始 z：
  控制到初始位置
```

这使控制输出通常跟随 odometry 频率更新，即使轨迹命令暂时没有新消息。

---

## 36. `position_cmd_updated_` 机制

设计假设：

```text
一帧 odometry 后通常很快会来一帧 position_cmd
```

逻辑：

```text
PositionCommand 回调：
  updated = true
  立即发布

Odometry 回调：
  若 updated = false：
    自己发布
  updated = false
```

目标是避免同一周期重复发布。

但两个话题没有时间同步，实际回调顺序取决于 ROS 调度。

---

## 37. 轨迹命令丢失时的行为

源码注释：

```cpp
TODO: Fallback to hover if position_cmd
hasn't been received for some time
```

当前没有超时检测。

一旦收到过命令：

```text
即使 traj_server 停止，
控制器也会一直使用最后一条期望状态。
```

这可能表现为：

- 继续追踪旧位置；
- 继续使用旧速度/加速度前馈；
- 无法自动安全悬停。

---

## 38. 启动前的初始位置控制

如果尚未收到轨迹命令，且：

```cpp
init_z_ > -9999
```

则：

```text
des_pos = (init_x, init_y, init_z)
des_vel = 0
des_acc = 0
```

并在每帧 odometry 时发布控制。

默认 launch 将初始状态传入，因此无人机在规划轨迹到来前会保持初始位置。

若未配置：

```text
init_z = -10000
```

表示关闭该行为。

---

## 39. IMU 加速度回调

```cpp
controller_.setAcc(
    imu.linear_acceleration);
```

该值用于：

```text
m Ka(ad - a)
```

附加反馈。

但默认 `simulator.xml` 没有为 Nodelet 的：

```text
~imu
```

设置重映射。

仿真器发布的是私有：

```text
/quadrotor_simulator_so3/imu
```

Nodelet 默认订阅：

```text
/so3_control/imu
```

因此默认完整仿真中，这两个话题并不相连。

`acc_` 会保持构造时的 0。

---

## 40. 电机启停

`~motors` 接收 `std_msgs/Bool`：

```text
true  -> enable_motors = true
false -> enable_motors = false
```

初始值：

```text
true
```

该标志被写入 `SO3Command.aux`。

但当前 `so3_quadrotor_simulator` 的命令回调没有保存：

```text
enable_motors
```

仿真器控制计算也没有检查它。

所以该开关对当前动力学仿真没有实际作用。

它主要为真实飞控通信链路保留。

---

## 41. Corrections

三项：

```text
corrections/z -> kf_correction
corrections/r -> roll correction
corrections/p -> pitch correction
```

Nodelet 可以：

- 从 YAML 初始化；
- 通过 `Corrections` 话题在线更新。

当前仿真器实际使用：

```text
kf_correction
```

修正螺旋桨推力系数。

它虽然接收两个 `angle_corrections`，但当前 `getControl()` 中没有使用。

---

## 42. `use_external_yaw`

Nodelet 将当前 odometry yaw 写入：

```text
aux.current_yaw
```

若：

```text
use_external_yaw = true
```

仿真器构造当前旋转矩阵时：

```text
保留仿真姿态的 pitch、roll
用命令携带的 current_yaw 替换 yaw
```

这适合真实系统中 yaw 来自外部估计器的场景。

纯仿真一般不需要。

---

## 43. 默认 launch 中带空格的参数名

launch 写成：

```xml
<param name="use_external_yaw "
       value="false"/>
```

参数名末尾有一个空格。

源码读取：

```text
use_external_yaw
```

没有空格。

所以该 launch 参数不会覆盖源码默认值。

默认实际值仍可能是：

```text
true
```

同样：

```text
use_angle_corrections 
```

也带尾部空格，而且源码根本没有读取这个参数。

---

## 44. `publishSO3Command()`

每次发布前：

```cpp
controller_.calculateControl(...)
```

然后填充：

```text
header.stamp = ros::Time::now()
header.frame_id = /quadrotor_name
force
orientation
kR
kOm
current_yaw
corrections
enable_motors
use_external_yaw
```

发布队列长度：

```text
10
```

---

## 45. Nodelet 输出频率

没有内部定时器。

发布由：

- `PositionCommand` 回调；
- odometry 回调；

触发。

默认：

```text
traj_server      100 Hz
odometry         200 Hz
```

由于去重逻辑，实际 `SO3Command` 频率取决于回调交错，通常接近 odometry 或命令频率中的较高有效节奏。

可用：

```bash
rostopic hz /so3_cmd
```

实测。

---

## 46. `traj_server` 如何提供命令

`traj_server`：

1. 接收 B 样条；
2. 构造位置、速度、加速度曲线；
3. 每 0.01 s 采样一次；
4. 计算面向前方轨迹点的 yaw；
5. 发布 `PositionCommand`。

默认话题：

```text
/position_cmd
```

在 launch 中重映射为：

```text
/planning/pos_cmd
```

---

## 47. 默认 `traj_server` 增益字段

全局数组：

```cpp
double pos_gain[3] = {0,0,0};
double vel_gain[3] = {0,0,0};
```

初始化到消息：

```text
cmd.kx = 0
cmd.kv = 0
```

所以 Nodelet 的：

```cpp
if (任一分量 > 1e-5)
```

不会触发。

外环增益由 Nodelet 自己决定。

---

## 48. 仿真器中的姿态内环

仿真器收到 `SO3Command` 后，计算：

```text
当前姿态 R
期望姿态 Rd
```

姿态误差：

```text
eR =
  0.5 (RdᵀR - RᵀRd)∨
```

源码展开成三个标量分量。

角速度误差当前简化为：

```text
eΩ = Ω
```

即期望角速度视为 0。

---

## 49. 姿态控制力矩

内环力矩：

```text
M =
  -KR eR
  -KΩ eΩ
  + Ω × JΩ
```

其中：

- 第一项纠正姿态；
- 第二项阻尼角速度；
- 第三项补偿刚体陀螺耦合。

这就是 `kR`、`kOm` 真正使用的位置。

---

## 50. 总推力计算

期望合力在世界系：

```text
F_world
```

当前机体只能沿自己的 z 轴产生推力。

所以仿真器计算：

```text
f = F_world · (R e3)
```

即：

```text
将世界系期望合力投影到当前机体 z 轴。
```

只有姿态误差函数：

```text
Psi < 1
```

时才使用该推力。

---

## 51. `Psi < 1` 的意义

```text
Psi = 0.5 tr(I - RdᵀR)
```

它衡量当前姿态和期望姿态的差异。

大致：

```text
Psi = 0   姿态一致
Psi = 1   接近 90° 误差边界
Psi = 2   约 180° 相反
```

源码注释：

```text
位置控制稳定性只在 Psi < 1 时保证
```

姿态差过大时将总推力设为 0，优先恢复姿态。

---

## 52. 电机混控

根据总推力和三个力矩求四个电机转速平方：

```text
w1² = f/(4kf) - M2/(2dkf) + M3/(4km)
w2² = f/(4kf) + M2/(2dkf) + M3/(4km)
w3² = f/(4kf) + M1/(2dkf) - M3/(4km)
w4² = f/(4kf) - M1/(2dkf) - M3/(4km)
```

若某个：

```text
wi² < 0
```

则截为 0。

最终：

```text
rpm_i = sqrt(wi²)
```

---

## 53. `control_example`

示例按每段约 5 秒循环三种模式。

### 位置控制

```text
position = (2,0,1)
```

速度和加速度字段保持消息默认 0，因此三层实际上都有效：

```text
位置反馈 + 速度反馈到 0 + 加速度前馈 0
```

### 速度控制

```text
position = NaN
velocity = (-1,0,0)
```

位置项关闭。

### 加速度控制

```text
position = NaN
velocity = NaN
acceleration = (1,0,0)
```

只保留重力补偿和加速度项。

发布频率约：

```text
100 Hz
```

---

## 54. 配置文件对比

### Hummingbird

特点：

- 位置增益较小；
- 旋转增益较小；
- 适合较轻小型机。

### Pelican

特点：

- z 位置增益明显更大；
- 姿态和角速度增益更高；
- 适合更大、更重平台。

但由于 `pos/vel` 与 `kx/kv` 命名错位，当前只有：

```text
rot/ang
```

差异会真正进入 Nodelet。

---

## 55. 当前源码中的重要风险与注意事项

以下结论针对当前工作空间源码。

### 55.1 `pos_` 和 `vel_` 未初始化

如果 `PositionCommand` 在第一帧 odometry 前到达，控制器会使用未初始化位置和速度。

可能输出随机值或 NaN。

应：

- 构造时置零；
- 或在有 odometry 前拒绝发布。

### 55.2 YAML 外环增益键与源码不一致

YAML：

```text
gains/pos
gains/vel
```

源码：

```text
gains/kx
gains/kv
```

默认外环增益不会按 YAML 生效。

### 55.3 launch 参数名尾部有空格

```text
use_external_yaw 
use_angle_corrections 
```

不是源码读取的参数名。

### 55.4 `use_angle_corrections` 没有实现

Nodelet 从未读取该参数。

仿真器也没有使用两个角度校正值。

### 55.5 默认 IMU 话题没有连接

`acc_` 通常保持 0。

加速度误差反馈变成：

```text
des_acc - 0
```

而不是真正测量反馈。

### 55.6 IMU 加速度坐标系可能不匹配

控制器的 `des_acc` 是世界系。

标准 IMU `linear_acceleration` 往往在机体系，并可能包含或不包含重力，取决于消息来源。

源码直接相减，没有坐标变换或重力语义处理。

即使正确连接 IMU，也可能物理意义错误。

### 55.7 `totalError` 混合不同量纲

位置、速度、加速度误差直接相加。

`ka` 是经验项，不是严格量纲一致控制律。

### 55.8 `ka` 在误差 3 处不连续

误差从 2.999 增到 3.001 时：

```text
ka ≈ 0.6 -> 0
```

会造成控制项突变。

### 55.9 “45 度限制”实际是 90 度

注释和代码不一致。

若想限制 45° 应使用：

```cpp
theta = M_PI / 4;
```

### 55.10 倾角限制公式可能数值退化

当前 `c≈0`，且：

```text
A
判别式
分母 2A
```

都可能接近特殊值。

没有检查：

- `A≈0`；
- 判别式为负；
- `s` 非有限。

### 55.11 `force_.norm()` 可能为 0

倾角判断在姿态构造之前直接使用：

```cpp
force_ / force_.norm()
```

若合力恰好为 0，会除零。

后面的 `b3c` 分支虽检查 1e-6，但已经太晚。

### 55.12 姿态基构造可能退化

`b3c × b1d` 接近 0 时会产生 NaN。

### 55.13 `des_yaw_dot` 未使用

轨迹服务器辛苦计算的 yaw_dot 没有进入内环。

### 55.14 期望角速度被固定为 0

仿真器：

```text
eOm = current omega
```

没有使用期望角速度和姿态前馈。

快速 yaw 或激进轨迹的姿态跟踪会受限。

### 55.15 没有命令超时保护

会无限使用最后一条命令。

### 55.16 不检查 `trajectory_flag`

即使轨迹状态表示异常、完成或终止，Nodelet 仍会使用消息字段。

### 55.17 不检查消息时间戳

延迟或乱序命令仍可能覆盖较新命令。

### 55.18 不检查 frame_id

默认假设命令和 odometry 都在同一世界坐标系。

### 55.19 `position_cmd_updated_` 不是严格同步

两个异步回调的顺序可能导致：

- 重复发布；
- 跳过一次发布；
- 输出节奏抖动。

### 55.20 `enable_motors` 对当前仿真无效

仿真器没有消费该字段。

### 55.21 Nodelet 默认电机开启

```cpp
enable_motors_(true)
```

启动即允许电机，不需要显式 arm。

### 55.22 动态分配消息不是内存泄漏

源码注释：

```cpp
//! @note memory leak?
```

使用的是 ROS `shared_ptr`：

```cpp
SO3Command::Ptr
```

发布完成并释放引用后会自动回收。

这通常不是内存泄漏，只是每次发布有堆分配开销。

### 55.23 `frame_id_` 以 `/` 开头

```text
/quadrotor
```

现代 TF/ROS 命名通常不建议 frame_id 带前导斜杠。

### 55.24 质量未校验

`mass <= 0` 会破坏重力补偿和控制公式。

### 55.25 重力不可通过 ROS 参数配置

类有：

```cpp
setGravity()
```

但 Nodelet 没有读取重力参数。

### 55.26 增益未校验

负增益或非有限增益会导致正反馈或 NaN。

### 55.27 消息覆盖增益判据不合理

只要任一分量大于 `1e-5` 就覆盖整个向量。

无法：

- 合法设置全零增益；
- 使用负增益做实验；
- 逐轴保留默认值。

### 55.28 加速度模式会重复放大 `des_acc`

若 IMU 未连接，`acc_=0`：

```text
m Ka(des_acc - 0) + m des_acc
```

期望加速度同时出现在反馈项和前馈项。

### 55.29 Corrections 的角度修正未使用

配置和消息接口存在，但仿真内环忽略它。

### 55.30 `use_external_yaw` 默认值与构造值不同

构造函数先设：

```text
false
```

`onInit()` 参数默认值却是：

```text
true
```

没有参数时最终为 true。

### 55.31 `control_example` 未包含 `<limits>`

它使用：

```cpp
std::numeric_limits<float>
```

但没有直接 include `<limits>`。

当前可能通过传递头文件编译，不够稳健。

### 55.32 `control_example` 注释编号重复

三段都标为：

```text
example 1
```

属于文档小问题。

### 55.33 CMake `catkin_package()` 没导出库和头文件

虽然生成 `SO3Control` 库，但没有声明：

```text
INCLUDE_DIRS
LIBRARIES
CATKIN_DEPENDS
```

其他包难以规范链接它。

### 55.34 没有安装规则

库、插件 XML、头文件和配置都没有 install。

在 install space 部署时可能缺失。

### 55.35 CMake 和 package 依赖不完整

直接使用的：

```text
sensor_msgs
std_msgs
pluginlib
Eigen3
```

未完整声明。

### 55.36 rosbuild 遗留内容较多

增加维护噪声。

### 55.37 默认增益与平台质量组合未经显式说明

Hummingbird YAML 配合：

```text
mass = 0.98
```

但 YAML 外环增益又不生效。

真实控制参数实际上是多个来源混合后的结果。

### 55.38 SO3Command 初始全局状态风险在下游

仿真器中的静态 `command` 初始为全零：

- 期望四元数全零；
- 增益全零；
- 力为零。

在第一条命令到来前，动力学可能自由落体或出现退化姿态计算。

### 55.39 仿真器 NaN 回退使用未初始化上一控制

首次控制若产生 NaN：

```cpp
auto last = control;
```

此时 `control` 可能尚未初始化。

这是下游风险，但会放大控制器非法输出的影响。

---

## 56. 推荐调试方法

### 56.1 检查 Nodelet

```bash
rosnode info /so3_control
```

确认订阅和发布话题。

### 56.2 检查命令链路频率

```bash
rostopic hz /planning/pos_cmd
rostopic hz /so3_cmd
rostopic hz /visual_slam/odom
```

默认大致：

```text
PositionCommand  100 Hz
Odometry         200 Hz
```

### 56.3 查看实际参数

```bash
rosparam get /so3_control
```

重点观察是否同时存在：

```text
use_external_yaw
"use_external_yaw "
```

以及：

```text
gains/pos
gains/vel
gains/kx
gains/kv
```

### 56.4 验证 YAML 命名错位

```bash
rosparam get /so3_control/gains
```

可看到 `pos/vel` 已加载，但源码仍使用内置 `kx/kv`。

### 56.5 查看合力和姿态

```bash
rostopic echo /so3_cmd/force
rostopic echo /so3_cmd/orientation
```

悬停时预期：

```text
force_z ≈ mass × 9.81
```

质量 0.98 kg 时：

```text
约 9.61 N
```

### 56.6 检查 IMU 连接

```bash
rostopic info /so3_control/imu
rostopic info /quadrotor_simulator_so3/imu
```

默认可能发现前者没有 publisher。

### 56.7 检查 NaN

```bash
rostopic echo /so3_cmd
```

重点观察：

- force；
- quaternion；
- kR/kOm。

姿态退化时四元数可能出现 NaN。

### 56.8 运行控制示例

使用 `simulator_example.launch`，再启用：

```xml
<node pkg="so3_control"
      name="control_example"
      type="control_example"/>
```

观察位置、速度和加速度三种模式。

---

## 57. 常见故障

### 57.1 无人机启动后下落

检查：

- `so3_cmd` 是否发布；
- odometry 是否到达控制器；
- 初始位置参数是否有效；
- 仿真器是否收到 `/so3_cmd`；
- 合力 z 是否约等于 `mg`；
- 第一条命令是否含 NaN。

### 57.2 修改 YAML 位置增益没有效果

原因是 YAML 使用：

```text
gains/pos
gains/vel
```

而源码读取：

```text
gains/kx
gains/kv
```

### 57.3 设置 `use_external_yaw=false` 没效果

检查参数名末尾空格。

### 57.4 发布 motors=false 仍继续飞

当前仿真器没有使用 `enable_motors`。

### 57.5 加速度反馈不工作

默认 Nodelet IMU 与仿真器 IMU 没有重映射连接。

### 57.6 姿态四元数出现 NaN

可能原因：

- 合力为 0；
- `b3c` 与 `b1d` 平行；
- 输入状态未初始化；
- 增益或命令含 NaN；
- 倾角限制公式退化。

### 57.7 轨迹服务器停止后无人机仍运动

控制器没有命令超时，会继续使用最后一条速度或加速度命令。

### 57.8 控制振荡

分别排查：

- `kx/kv` 外环过大；
- `kR/kOm` 内环过大或阻尼不足；
- 质量设置错误；
- IMU 加速度反馈语义错误；
- 轨迹加速度不连续；
- odometry 延迟。

---

## 58. 推荐改进方向

### 58.1 低风险修复

优先建议：

1. 初始化所有 Eigen 成员；
2. 有 odometry 后才允许发布；
3. 修复 YAML `pos/vel` 与源码命名；
4. 删除 launch 参数尾部空格；
5. 补齐 IMU 重映射；
6. 设置命令超时；
7. 校验质量和增益；
8. 删除或实现无效参数；
9. 补齐依赖和安装规则。

### 58.2 明确加速度反馈语义

应明确 IMU 消息：

- 位于哪个坐标系；
- 是否包含重力；
- 是否需要旋转到世界系；
- 是否经过滤波。

再决定是否保留：

```text
Ka(ad-a)
```

### 58.3 使用连续的自适应增益

例如饱和函数：

```text
ka = min(kmax, slope × |error|)
```

而不是超过阈值突然归零。

### 58.4 正确实现倾角限制

直接约束水平力：

```text
||Fxy|| <= Fz tan(theta_max)
```

通常更容易保证数值稳定。

### 58.5 增加姿态构造退化保护

当：

```text
|b3 × b1d| < epsilon
```

时选择备用水平轴。

### 58.6 使用 yaw_dot 和期望角速度

从：

- 推力方向变化；
- yaw、yaw_dot；

计算期望角速度 `Ωd`，传给内环。

这能改善激进轨迹跟踪。

### 58.7 统一发布时钟

可以使用固定控制定时器：

```text
200 Hz
```

保存最新状态和命令，每次定时器统一计算。

比依赖两个异步回调更稳定。

### 58.8 实现安全状态机

建议状态：

```text
WAIT_ODOM
WAIT_COMMAND
ACTIVE
HOVER_TIMEOUT
MOTORS_DISABLED
```

### 58.9 真正实现电机禁用

仿真器应检查：

```text
enable_motors
```

禁用时将所有 rpm 置 0。

### 58.10 分离外环和内环接口

更清晰地定义：

```text
outer-loop controller
attitude controller
mixer
```

并为每层增加单元测试。

---

## 59. 推荐源码阅读顺序

### 第一阶段：理解消息链路

阅读：

```text
PositionCommand.msg
SO3Command.msg
AuxCommand.msg
```

先明确输入和输出。

### 第二阶段：理解控制类

阅读：

```text
SO3Control.h
SO3Control.cpp
```

重点：

- NaN 开关；
- 合力公式；
- `ka`；
- 期望姿态构造。

### 第三阶段：理解 ROS 包装

阅读：

```text
so3_control_nodelet.cpp
```

顺序：

1. `onInit()`；
2. `position_cmd_callback()`；
3. `odom_callback()`；
4. `publishSO3Command()`；
5. IMU、motors、corrections。

### 第四阶段：理解上游

阅读：

```text
planner/plan_manage/src/traj_server.cpp
```

了解 B 样条如何变成 PositionCommand。

### 第五阶段：理解下游

阅读：

```text
so3_quadrotor_simulator/
src/quadrotor_simulator_so3.cpp
```

重点：

- 姿态误差；
- 力矩；
- 推力投影；
- 电机混控。

### 第六阶段：运行示例

阅读和运行：

```text
control_example.cpp
simulator_example.launch
```

---

## 60. 建议学习实验

### 实验一：悬停力验证

固定目标为当前位置。

检查：

```text
F ≈ (0,0,mg)
orientation ≈ yaw 对应的水平姿态
```

### 实验二：位置阶跃

目标 x 增加 1 m。

观察：

- 合力 x；
- 期望 pitch；
- 位置响应；
- 超调。

### 实验三：只使用速度控制

将位置设为 NaN。

验证位置项确实关闭。

### 实验四：只使用加速度控制

位置和速度设为 NaN。

对比 IMU 未连接和正确连接时的输出。

### 实验五：验证 YAML 错位

大幅修改 `gains/pos`，观察响应不变。

再将 YAML 改为 `gains/kx`，验证生效。

### 实验六：倾角限制

给出很大的水平位置误差。

测量：

```text
acos(Fz / ||F||)
```

验证当前并非限制在 45°。

### 实验七：命令超时

发送持续速度命令后停止发布。

观察无人机继续使用旧命令。

### 实验八：yaw 跟踪

保持位置不变，缓慢改变 yaw。

检查 yaw_dot 对输出没有影响。

### 实验九：姿态退化

构造接近水平、与 `b1d` 平行的期望合力，验证是否产生 NaN。

### 实验十：内外环增益分离

分别只修改：

- `kx/kv`；
- `kR/kOm`。

比较位置响应和姿态响应。

### 实验十一：电机禁用

发布：

```text
/motors = false
```

验证当前仿真仍继续运行，再实现下游禁用。

### 实验十二：控制频率

记录：

```text
/planning/pos_cmd
/visual_slam/odom
/so3_cmd
```

分析异步回调如何决定输出频率。

---

## 61. 常见问题

### Q1：为什么输出不是四个电机转速？

该包只完成外层平移控制和期望姿态构造。

电机混控位于 `so3_quadrotor_simulator` 或真实飞控。

### Q2：为什么叫 SO3Control，但类里没计算姿态误差？

当前系统把 SO(3) 姿态误差内环放在下游仿真器。

本类生成期望 SO(3) 姿态。

### Q3：`force` 是推力标量吗？

不是。

它是世界坐标系三维期望合力向量。

### Q4：为什么需要期望姿态？

四旋翼只能沿机体 z 轴产生主要推力。

要产生某个世界系合力，必须将机体 z 轴转向该方向。

### Q5：yaw 与推力方向冲突怎么办？

推力方向优先决定机体 z 轴。

yaw 用于在剩余自由度中选择机体 x/y 轴朝向。

### Q6：NaN 为什么能切换控制模式？

控制器显式检测 NaN，并跳过对应反馈项。

### Q7：默认 EGO-Planner 使用消息里的增益吗？

不使用。

`traj_server` 将 `kx/kv` 设为全零，Nodelet 保留自己的增益。

### Q8：Hummingbird YAML 的位置增益生效吗？

当前不生效，因为键名与源码不一致。

### Q9：IMU 是必须的吗？

标准位置/速度/加速度前馈可以在没有 IMU 附加反馈时运行。

当前 `ka` 项则依赖正确的加速度测量语义。

### Q10：`enable_motors` 能停止仿真无人机吗？

当前不能，仿真器没有使用该字段。

### Q11：为什么需要 `current_yaw`？

允许内环使用外部 yaw 估计替换动力学状态中的 yaw。

### Q12：控制器有命令超时吗？

没有。

需要上层持续发布或补充安全状态机。

### Q13：控制角真的限制在 45° 吗？

当前不是。代码使用 90°，与注释不一致。

### Q14：`yaw_dot` 有作用吗？

当前没有作用。

### Q15：为什么控制器还在初始位置悬停？

在第一条轨迹命令到达前，Nodelet 使用 launch 提供的初始位置作为目标。

---

## 62. 总结

`so3_control` 是 EGO-Planner 仿真控制链路中的轨迹跟踪外环。

它完成：

```text
PositionCommand
  -> 位置/速度/加速度误差反馈
  -> 世界系期望合力
  -> 期望机体姿态
  -> SO3Command
```

下游再完成：

```text
SO3Command
  -> SO(3) 姿态误差
  -> 控制力矩
  -> 四电机转速
  -> 四旋翼动力学
```

理解本包时最重要的八条主线是：

1. **它默认以 standalone Nodelet 运行。**
2. **输入是轨迹状态和 odometry，输出是合力与期望姿态。**
3. **位置、速度和加速度项可通过 NaN 整体禁用。**
4. **世界系合力方向决定机体 z 轴。**
5. **期望 yaw 决定剩余水平朝向。**
6. **`kx/kv` 属于外环，`kR/kOm` 属于下游姿态内环。**
7. **默认 YAML 的 `pos/vel` 键与源码 `kx/kv` 不匹配。**
8. **当前系统还存在 IMU 未连接、yaw_dot 未使用和命令无超时等工程问题。**

默认真实链路：

```text
traj_server
  -> /planning/pos_cmd
  -> so3_control
  -> /so3_cmd
  -> so3_quadrotor_simulator
  -> /visual_slam/odom
  -> so3_control
```

当前实现足以支持 EGO-Planner 仿真飞行，但在：

- 初始化安全；
- 参数命名；
- 倾角限制；
- IMU 坐标语义；
- yaw 角速度前馈；
- 命令超时；
- 电机启停；
- 构建与安装规范；

方面仍有清晰的改进空间。

结合：

```text
traj_server
quadrotor_msgs
so3_quadrotor_simulator
```

一起阅读，就能完整理解 EGO-Planner 从平滑轨迹到四个电机动力学输入的控制闭环。
