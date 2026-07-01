# `so3_quadrotor_simulator` 功能包详细学习说明

> 适用源码：`/home/yxc/Desktop/ego-planner_ws/src/uav_simulator/so3_quadrotor_simulator`
>
> ROS 功能包名称：`so3_quadrotor_simulator`
>
> 所属系统：EGO-Planner 四旋翼动力学仿真链路

---

## 1. 一句话认识这个包

`so3_quadrotor_simulator` 是一个包含：

```text
SO(3) 姿态内环
四电机混控
电机一阶动态
六自由度刚体动力学
外力与外力矩扰动
Odometry 与 IMU 输出
```

的轻量级四旋翼仿真器。

它接收 `so3_control` 生成的：

```text
世界系期望合力
期望姿态
姿态与角速度增益
```

计算四个电机目标转速，并积分得到下一时刻的无人机状态。

---

## 2. 它在 EGO-Planner 中的位置

完整闭环：

```text
EGO-Planner
生成 B 样条轨迹
        |
        v
traj_server
生成 PositionCommand
        |
        v
so3_control
生成世界系合力和期望姿态
        |
        | SO3Command
        v
so3_quadrotor_simulator
姿态控制、混控和动力学积分
        |
        | Odometry + IMU
        v
so3_control、local_sensing、GridMap、规划器
```

这个包同时承担：

```text
控制系统的姿态内环
        和
被控对象的动力学模型
```

这两部分在真实系统中通常分别位于：

- 飞控；
- 真实飞行器。

---

## 3. 与 `so3_control` 的分工

`so3_control`：

```text
位置误差
速度误差
期望加速度
        |
        v
世界系期望合力 Fd
期望姿态 Rd
```

`so3_quadrotor_simulator`：

```text
当前姿态 R
当前角速度 Ω
期望姿态 Rd
        |
        v
SO(3) 姿态误差 eR
角速度误差 eΩ
        |
        v
总推力 f 与控制力矩 M
        |
        v
四个电机转速
        |
        v
刚体动力学
```

所以当前工程中完整的几何控制器跨越两个功能包。

---

## 4. 目录结构

```text
so3_quadrotor_simulator/
├── CMakeLists.txt
├── package.xml
├── include/
│   ├── quadrotor_simulator/
│   │   └── Quadrotor.h
│   └── ode/
│       └── boost/numeric/odeint/...
├── src/
│   ├── quadrotor_simulator_so3.cpp
│   ├── dynamics/
│   │   └── Quadrotor.cpp
│   └── test_dynamics/
│       └── test_dynamics.cpp
├── launch/
│   └── simulator_example.launch
└── config/
    └── rviz.rviz
```

核心文件：

| 文件 | 职责 |
|---|---|
| `Quadrotor.h` | 状态、物理参数和动力学接口 |
| `Quadrotor.cpp` | 电机、推力、力矩和刚体微分方程 |
| `quadrotor_simulator_so3.cpp` | ROS 节点、SO(3) 内环、混控与消息转换 |
| `test_dynamics.cpp` | 高度控制和动力学性能测试 |
| `include/ode/` | 随包附带的 Boost.Odeint 源码 |
| `simulator_example.launch` | 控制器与仿真器独立演示 |

---

## 5. 构建产物

### 5.1 动力学库

```cmake
add_library(
  quadrotor_dynamics
  src/dynamics/Quadrotor.cpp
)
```

生成：

```text
libquadrotor_dynamics.so
```

### 5.2 ROS 仿真节点

```cmake
add_executable(
  quadrotor_simulator_so3
  src/quadrotor_simulator_so3.cpp
)
```

链接：

```text
catkin 库
quadrotor_dynamics
```

### 5.3 测试程序

`test_dynamics.cpp` 存在，但 CMake 没有：

```cmake
add_executable(...)
```

所以当前不会自动构建。

---

## 6. 内置 Odeint

目录：

```text
include/ode/
```

包含一份完整的旧版 Boost.Odeint：

- 头文件；
- 文档；
- 示例；
- 测试；
- 性能程序；
- 图片和 BoostBook 文件。

实际动力学只包含：

```cpp
#include "ode/boost/numeric/odeint.hpp"
```

其余文档、示例和图片不参与编译。

将整个上游库复制进功能包可以避免系统 Boost 版本差异，但也显著增大源码体积。

---

## 7. 关键依赖

| 依赖 | 用途 |
|---|---|
| Eigen3 | 向量、旋转矩阵、惯性矩阵和四元数 |
| Boost.Odeint | 常微分方程数值积分 |
| `roscpp` | ROS 节点和消息 |
| `quadrotor_msgs` | 接收 `SO3Command` |
| `uav_utils` | 旋转矩阵转 yaw-pitch-roll |
| `nav_msgs` | 发布 Odometry |
| `sensor_msgs` | 发布 IMU |
| `geometry_msgs` | 接收外力和外力矩 |
| `cmake_utils` | 构建依赖，源码未直接使用 |

CMake 还查找：

```cmake
find_package(Armadillo REQUIRED)
```

但当前源码没有使用或链接 Armadillo。

---

## 8. 构建配置

```cmake
add_compile_options(-std=c++11)
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")
```

C++11 被重复指定。

`catkin_package()` 声明：

```cmake
INCLUDE_DIRS include
CATKIN_DEPENDS roscpp quadrotor_msgs uav_utils
DEPENDS Eigen3 system_lib
```

其中：

```text
system_lib
```

不是实际查找的依赖，看起来是 catkin 模板遗留。

---

## 9. ROS 接口总览

### 9.1 订阅

| 私有话题 | 类型 | 用途 |
|---|---|---|
| `~cmd` | `quadrotor_msgs/SO3Command` | 合力、期望姿态和控制增益 |
| `~force_disturbance` | `geometry_msgs/Vector3` | 世界系外力 |
| `~moment_disturbance` | `geometry_msgs/Vector3` | 机体系外力矩 |

### 9.2 发布

| 私有话题 | 类型 | 用途 |
|---|---|---|
| `~odom` | `nav_msgs/Odometry` | 仿真位置、姿态、速度和角速度 |
| `~imu` | `sensor_msgs/Imu` | 仿真姿态、角速度和线加速度 |

所有订阅都使用：

```cpp
tcpNoDelay()
```

降低命令和扰动延迟。

---

## 10. 默认话题连接

`plan_manage/launch/simulator.xml`：

```text
~cmd
  -> /so3_cmd

~odom
  -> /visual_slam/odom

~force_disturbance
  -> /force_disturbance

~moment_disturbance
  -> /moment_disturbance
```

默认没有重映射：

```text
~imu
```

所以 IMU 完整名称是：

```text
/quadrotor_simulator_so3/imu
```

而 `so3_control` 默认订阅：

```text
/so3_control/imu
```

两者并未连接。

---

## 11. 默认闭环

```text
/planning/pos_cmd
        |
        v
so3_control
        |
        | /so3_cmd
        v
quadrotor_simulator_so3
        |
        | /visual_slam/odom
        +------------------------+
        |                        |
        v                        v
  so3_control              local_sensing
                                 |
                                 v
                              GridMap
```

仿真器输出的 odometry 同时用于：

- 控制反馈；
- 地图局部感知；
- 无人机可视化；
- 规划器状态估计。

---

## 12. 节点公共参数

| 参数 | 默认值 | 用途 |
|---|---:|---|
| `simulator/init_state_x` | `0.0` | 初始 x |
| `simulator/init_state_y` | `0.0` | 初始 y |
| `simulator/init_state_z` | `1.0` | 初始 z |
| `rate/simulation` | `1000 Hz` | 动力学积分循环频率 |
| `rate/odom` | `100 Hz` | Odometry 和 IMU 发布频率 |
| `quadrotor_name` | `quadrotor` | odometry child frame |

默认 EGO-Planner launch 只显式设置：

```text
rate/odom = 200 Hz
```

所以动力学仍按默认：

```text
1000 Hz
```

积分。

---

## 13. `Quadrotor::State`

```cpp
struct State
{
  Vector3d x;
  Vector3d v;
  Matrix3d R;
  Vector3d omega;
  Array4d motor_rpm;
};
```

字段：

| 字段 | 坐标系/含义 |
|---|---|
| `x` | 世界系位置 |
| `v` | 世界系线速度 |
| `R` | 机体系到世界系旋转矩阵 |
| `omega` | 机体系角速度 |
| `motor_rpm` | 四个电机实际转速 |

总自由状态为：

```text
3 + 3 + 9 + 3 + 4 = 22
```

旋转矩阵虽然只有 3 个独立自由度，但积分时保存全部 9 个元素。

---

## 14. 22 维内部状态布局

```text
0..2    position x
3..5    velocity v
6..8    R 第一列
9..11   R 第二列
12..14  R 第三列
15..17  angular velocity omega
18..21  four motor rpm
```

`updateInternalState()` 在公开状态和 Odeint 数组之间复制。

这种平铺布局允许 Boost.Odeint 直接处理：

```cpp
boost::array<double, 22>
```

---

## 15. 默认物理参数

构造函数硬编码：

| 参数 | 默认值 |
|---|---:|
| 重力 `g` | `9.81 m/s²` |
| 质量 `m` | `0.98 kg` |
| `Ixx` | `2.64e-3 kg·m²` |
| `Iyy` | `2.64e-3 kg·m²` |
| `Izz` | `4.96e-3 kg·m²` |
| 螺旋桨半径 | `0.062 m` |
| 推力系数 `kf` | `8.98132e-9` |
| 臂长 `d` | `0.26 m` |
| 电机时间常数 | `1/30 s` |
| 最低转速 | `1200 rpm` |
| 最高转速 | `35000 rpm` |

这些参数更接近 AscTec Hummingbird 级别模型。

---

## 16. 反扭矩系数 `km`

源码：

```cpp
km = 0.07 * (3 * prop_radius) * kf;
```

代入：

```text
prop_radius = 0.062 m
kf = 8.98132e-9
```

得到：

```text
km ≈ 1.17e-10
```

注释说明来源于：

```text
km = (Cq/Ct) × propeller diameter × kf
```

但代码使用：

```text
3 × prop_radius
```

而常规直径是：

```text
2 × prop_radius
```

这是一个经验模型。

---

## 17. 初始状态

动力学类构造时：

```text
x = 0
v = 0
R = I
omega = 0
motor_rpm = 0
input = 0
external_force = 0
```

ROS 节点随后仅调用：

```cpp
quad.setStatePos(init_position);
```

所以初始：

- 姿态水平；
- 速度为 0；
- 电机实际转速为 0；
- 位置来自 launch。

---

## 18. 电机编号和旋转方向

头文件注释：

```text
       1 前

   3       4

       2 后
```

旋转方向：

```text
1、2 顺时针
3、4 逆时针
```

俯视无人机。

这不是常见的 X 型四旋翼编号图，而更接近十字 `+` 型布局。

---

## 19. SO(3) 命令缓存

ROS 节点将消息复制到静态结构：

```cpp
struct Command
{
  force[3];
  qx,qy,qz,qw;
  kR[3];
  kOm[3];
  corrections[3];
  current_yaw;
  use_external_yaw;
};
```

注意没有保存：

```text
enable_motors
```

也不保存命令：

- 时间戳；
- frame_id；
- 序号。

---

## 20. `kf_correction`

内环先计算：

```text
kf_corrected = kf_model - kf_correction
```

并保持：

```text
km/kf
```

比例不变：

```text
km_corrected = km_model / kf_model × kf_corrected
```

该校正用于补偿真实电机推力模型误差。

如果校正过大使：

```text
kf_corrected <= 0
```

后续混控会除零或符号异常。

---

## 21. 外部 yaw 模式

正常情况下使用动力学状态：

```text
R
```

先转为：

```text
yaw, pitch, roll
```

若：

```text
use_external_yaw = true
```

则只替换 yaw：

```text
yaw = command.current_yaw
```

保留动力学 pitch 和 roll，再重建旋转矩阵。

用途是模拟：

```text
飞控使用外部定位系统提供的 yaw。
```

---

## 22. 期望姿态矩阵

命令提供四元数：

```text
qx, qy, qz, qw
```

源码手动展开为旋转矩阵 `Rd`。

它没有先归一化四元数。

因此输入四元数必须满足：

```text
qx² + qy² + qz² + qw² = 1
```

否则 `Rd` 不再是正交旋转矩阵。

---

## 23. SO(3) 姿态误差函数

源码：

```text
Psi =
  0.5 × (3 - trace(RdᵀR))
```

等价于经典几何控制误差：

```text
Psi(R,Rd)
  = 0.5 trace(I - RdᵀR)
```

若姿态夹角为 `theta`：

```text
Psi = 1 - cos(theta)
```

所以：

| 姿态误差角 | `Psi` |
|---:|---:|
| `0°` | `0` |
| `90°` | `1` |
| `180°` | `2` |

---

## 24. 为什么 `Psi < 1` 才给推力

源码：

```cpp
if (Psi < 1)
  force = Fd · (R e3);
```

`Psi < 1` 对应姿态误差小于约 90°。

几何控制理论中，平移跟踪稳定性通常要求姿态初始误差在一定吸引域内。

若姿态偏差太大，代码将总推力设为 0：

```text
先依靠姿态力矩恢复朝向，
避免错误方向的大推力。
```

---

## 25. 总推力投影

`SO3Command.force` 是世界系期望合力：

```text
Fd
```

当前机体只能沿自身 z 轴：

```text
b3 = R e3
```

产生推力。

因此实际总推力标量：

```text
f = Fd · b3
```

源码：

```cpp
force =
  cmd.force[0] * R13
  + cmd.force[1] * R23
  + cmd.force[2] * R33;
```

---

## 26. SO(3) 姿态误差向量

经典定义：

```text
eR =
  0.5 (RdᵀR - RᵀRd)∨
```

其中 `∨` 将反对称矩阵转为三维向量。

源码将三个分量完全展开为：

```text
eR1
eR2
eR3
```

避免矩阵乘法和 vee 函数调用。

---

## 27. 角速度误差

当前：

```text
eOmega = current omega
```

即：

```text
期望角速度 Omega_d = 0
```

没有使用：

- `so3_control` 接收的 yaw_dot；
- 期望姿态变化率；
- 前馈角加速度。

所以内环只追踪静态期望姿态。

对于每一时刻不断更新的 `Rd`，它通过高频位置命令间接跟随。

---

## 28. 刚体陀螺补偿

旋转动力学：

```text
J Omega_dot
  + Omega × J Omega
  = M
```

控制器中的补偿项：

```text
Omega × J Omega
```

源码展开为：

```text
in1, in2, in3
```

最终控制力矩：

```text
M =
  -KR eR
  -KOmega eOmega
  + Omega × J Omega
```

---

## 29. 鲁棒姿态控制代码

源码保留了一段被注释的鲁棒项：

```text
eA = eOmega + c2 J^-1 eR
muR = ...
```

当前没有参与力矩计算。

最终代码注释仍保留：

```cpp
// - I[i][i] * muRi
```

说明原实现曾考虑模型不确定性补偿。

---

## 30. 四电机混控

由总推力 `f` 和力矩：

```text
M1 roll
M2 pitch
M3 yaw
```

求电机转速平方：

```text
w1² = f/(4kf) - M2/(2dkf) + M3/(4km)
w2² = f/(4kf) + M2/(2dkf) + M3/(4km)
w3² = f/(4kf) + M1/(2dkf) - M3/(4km)
w4² = f/(4kf) - M1/(2dkf) - M3/(4km)
```

负值被截为：

```text
0
```

然后：

```text
rpm_i = sqrt(wi²)
```

---

## 31. 为什么混控先得到转速平方

螺旋桨模型：

```text
thrust_i = kf × rpm_i²
moment_i = km × rpm_i²
```

总推力和力矩对：

```text
rpm²
```

是线性的。

所以先解线性混控矩阵得到 `rpm²`，再开方得到转速。

---

## 32. 命令转速限幅

`Quadrotor::setInput()`：

```text
NaN：
  替换为 (max_rpm + min_rpm)/2

高于 max：
  截为 max

低于 min：
  截为 min
```

当前：

```text
min_rpm = 1200
max_rpm = 35000
```

即使混控要求：

```text
0 rpm
```

最终目标输入也会被抬到：

```text
1200 rpm
```

---

## 33. 电机一阶动态

电机实际转速不会瞬间等于命令：

```text
rpm_dot =
  (rpm_command - rpm_actual)
  / motor_time_constant
```

时间常数：

```text
tau = 1/30 s ≈ 0.0333 s
```

一步响应约：

```text
1 tau  达到 63.2%
3 tau  达到 95%
```

所以电机具有约 0.1 秒的明显建立过程。

---

## 34. 四个电机产生的总推力

```text
T =
  kf (w1² + w2² + w3² + w4²)
```

该推力沿机体 z 轴正方向。

世界系推力：

```text
T_world = T × R e3
```

进入平移动力学。

---

## 35. 电机产生的 roll 力矩

源码：

```text
Mx =
  kf (w3² - w4²) d
```

即左右两个电机推力差产生绕机体 x 轴力矩。

---

## 36. 电机产生的 pitch 力矩

```text
My =
  kf (w2² - w1²) d
```

后、前电机推力差产生绕机体 y 轴力矩。

---

## 37. 电机产生的 yaw 力矩

```text
Mz =
  km (w1² + w2² - w3² - w4²)
```

顺时针组与逆时针组反扭矩差产生偏航力矩。

---

## 38. 平移动力学

```text
x_dot = v
```

```text
v_dot =
  -g e3
  + T R e3 / m
  + F_external / m
  - F_drag / m
```

其中：

- 重力向世界 z 负方向；
- 推力沿当前机体 z 轴；
- 外力按世界系直接加入；
- 阻力与速度方向相反。

---

## 39. 简化空气阻力

阻力大小：

```text
D =
  C × S × ||v||²
```

源码：

```text
C = 0.1
S = pi × arm_length²
```

方向：

```text
-v / ||v||
```

所以：

```text
F_drag =
  0.1 pi d² ||v||² v_hat
```

这是一个各向同性二次阻力模型。

它没有考虑：

- 机体姿态；
- 不同轴迎风面积；
- 螺旋桨下洗；
- 空气密度；
- 风场。

---

## 40. 旋转动力学

```text
R_dot = R Omega_hat
```

其中 `Omega_hat` 是角速度的反对称矩阵。

```text
Omega_dot =
  J^-1
  (M_motor
   - Omega × J Omega
   + M_external)
```

外部力矩按机体系直接加入。

---

## 41. 角速度反对称矩阵

源码构造：

```text
             [  0   -wz   wy ]
Omega_hat =  [ wz    0   -wx ]
             [-wy   wx    0  ]
```

满足：

```text
Omega_hat a = Omega × a
```

姿态运动学：

```text
R_dot = R Omega_hat
```

说明 `omega` 表达在机体系。

---

## 42. 外部扰动

### 外力

话题：

```text
~force_disturbance
```

直接赋给：

```cpp
external_force_
```

并在世界系平移动力学中使用。

### 外力矩

话题：

```text
~moment_disturbance
```

直接赋给：

```cpp
external_moment_
```

并在机体系旋转动力学中使用。

扰动会一直保持最后一条值，直到收到新消息。

---

## 43. ODE 数值积分

每次：

```cpp
quad.step(dt);
```

调用：

```cpp
odeint::integrate(
    boost::ref(*this),
    internal_state_,
    0.0,
    dt,
    dt);
```

动力学对象的：

```cpp
operator()(state, derivative, time)
```

提供微分方程。

外层循环默认：

```text
dt = 1 / 1000 = 0.001 s
```

---

## 44. 为什么积分旋转矩阵会漂移

数值积分：

```text
R_dot = R Omega_hat
```

不能自动保证积分后的：

```text
RᵀR = I
det(R) = 1
```

浮点误差会让矩阵逐渐偏离 SO(3)。

所以源码在：

- 每次导数计算；
- 每次 step 完成；

都尝试重新正交化 `R`。

---

## 45. 当前正交化方法

源码计算：

```cpp
LLT(RᵀR) -> P
R_corrected = R P^-1
```

注释称其为：

```text
polar decomposition
```

严格极分解应使用：

```text
R_corrected =
  R (RᵀR)^(-1/2)
```

LLT 的下三角因子通常不等于对称平方根。

所以当前方法并非严格的极分解投影，尽管在误差很小时可能近似保持正交。

---

## 46. 地面模型

积分完成后：

```cpp
if (z < 0 && vz < 0)
{
  z = 0;
  vz = 0;
}
```

这是非常简化的地面：

- 只限制 z；
- 只清零向下速度；
- 不处理碰撞冲量；
- 不处理姿态碰撞；
- 不处理水平摩擦；
- 不阻止机体穿过其他障碍；
- 不处理弹跳。

无人机仍可在 z=0 处保持倾斜或高速水平滑动。

---

## 47. NaN 防护

动力学导数中：

```text
若 dxdt[i] 是 NaN：
  将该导数置 0
```

积分后：

```text
若任一状态是 NaN：
  恢复 step 前的完整状态
```

ROS 主循环中：

```text
若新计算的 rpm 是 NaN：
  使用上一轮 rpm
```

这是三层容错，但部分回退本身存在未初始化风险。

---

## 48. 主循环完整流程

```text
ros::spinOnce()
接收最新命令和扰动
        |
        v
getControl()
计算推力、力矩和四个目标 rpm
        |
        v
NaN rpm 回退
        |
        v
setInput()
做 rpm 限幅
        |
        v
设置外力和外力矩
        |
        v
quad.step(dt)
积分电机与刚体动力学
        |
        v
到 odom 发布时刻？
        |
        +---- 是 ----+
        |            |
        v            v
生成 Odometry      生成 IMU
        |            |
        +------发布--+
```

---

## 49. 仿真频率与发布频率

默认：

```text
simulation_rate = 1000 Hz
odom_rate       = 100 Hz
```

EGO-Planner launch：

```text
simulation_rate = 1000 Hz
odom_rate       = 200 Hz
```

所以每发布一帧 odometry，大约进行：

```text
5 次动力学积分
```

控制命令可在每个 1 ms 主循环开头更新。

---

## 50. Odometry 输出

```text
pose.position     = state.x
pose.orientation  = Quaternion(state.R)
twist.linear      = state.v
twist.angular     = state.omega
```

header：

```text
frame_id = /simulator
child_frame_id = /quadrotor_name
stamp = 当前 ROS 时间
```

当前约定：

- `state.v` 是世界系；
- `state.omega` 是机体系。

它们被直接放进同一个 Odometry twist。

---

## 51. IMU 输出

```text
orientation       = 当前姿态
angular_velocity  = 机体系 omega
linear_acceleration = quad.getAcc()
```

`getAcc()` 返回动力学中的：

```text
世界系 v_dot
```

它包含重力影响，是惯性坐标加速度。

这与标准 IMU 常见的：

```text
机体系 specific force
```

语义不同。

---

## 52. IMU 与真实传感器的差异

当前 IMU：

- 没有噪声；
- 没有偏置；
- 没有随机游走；
- 没有延迟；
- 没有量程；
- 没有坐标旋转；
- 没有正确 specific force 模型；
- 没有 covariance；
- 没有单独设置时间戳。

它更适合内部调试，不是严格传感器仿真。

---

## 53. `test_dynamics.cpp`

测试流程：

1. 计算悬停转速；
2. 将电机实际转速初始化为悬停值；
3. 用简单 PD 控制高度到 `0.5 m`；
4. 前 3 秒施加向下外力；
5. 后 3 秒移除外力；
6. 输出位置、姿态、角速度和电机转速；
7. 统计计算耗时。

它可以验证：

- 悬停公式；
- 外力扰动响应；
- 电机动态；
- 数值积分性能。

当前没有构建目标，需要手动加入 CMake 才能运行。

---

## 54. 悬停转速

悬停条件：

```text
4 kf rpm_hover² = m g
```

所以：

```text
rpm_hover =
  sqrt(mg / (4kf))
```

代入默认参数：

```text
m = 0.98
g = 9.81
kf = 8.98132e-9
```

得到约：

```text
16350 rpm
```

位于 `[1200,35000]` 范围内。

---

## 55. `simulator_example.launch`

该 launch 同时启动：

1. `quadrotor_simulator_so3`；
2. `so3_control` Nodelet；
3. `odom_visualization`；
4. RViz。

并正确连接：

```text
/sim/odom
/sim/imu
/so3_cmd
/position_cmd
```

与默认 EGO-Planner 的 `simulator.xml` 不同，它显式将 Nodelet IMU 重映射到：

```text
/sim/imu
```

所以独立示例中加速度回调确实会工作。

---

## 56. 当前源码中的重要风险与注意事项

以下结论针对当前工作空间源码。

### 56.1 `external_moment_` 没有初始化

构造函数只执行：

```cpp
external_force_.setZero();
```

没有：

```cpp
external_moment_.setZero();
```

第一次 `setExternalMoment()` 前，旋转动力学可能使用未初始化力矩。

ROS 主循环每轮都会用静态零初始化的 `disturbance.m` 调用设置，因此节点路径通常很快修正。

但单独使用 `Quadrotor` 类时存在风险。

### 56.2 `acc_` 构造时未初始化

第一次动力学导数计算前调用 `getAcc()` 会返回未定义值。

ROS 节点先 `step()` 再发布 IMU，正常路径通常安全。

### 56.3 首轮 `control` 和 `last` 未初始化

```cpp
Control control;
auto last = control;
```

首次 `getControl()` 若产生 NaN，回退到的 `last.rpm` 也是未初始化值。

### 56.4 初始命令是全零四元数

静态 `command` 被零初始化：

```text
force = 0
quaternion = (0,0,0,0)
gains = 0
```

第一条 `SO3Command` 到达前：

- `Rd` 是零矩阵，不是旋转矩阵；
- `Psi = 1.5`；
- 推力被置 0；
- 混控结果 0；
- `setInput()` 又把目标 rpm 抬到 1200。

无人机会以最低电机转速下落。

### 56.5 没有命令接收标志

节点不会等待第一条合法命令。

更安全的做法是：

- 未收到命令时悬停；
- 或保持电机关闭；
- 或不推进动力学控制输入。

### 56.6 不检查 SO3Command 时间戳

陈旧、乱序或长时间未更新的命令会无限使用。

### 56.7 不检查 `enable_motors`

`SO3Command.aux.enable_motors` 被完全忽略。

### 56.8 角度校正没有使用

`corrections[1]` 和 `corrections[2]` 被保存，但不参与控制。

### 56.9 `kf_correction` 可能使系数非正

没有检查：

```text
0 < kf_corrected
```

会导致混控除零或方向反转。

### 56.10 输入四元数没有归一化

非法四元数会产生非正交 `Rd` 和错误姿态误差。

### 56.11 `Psi < 1` 是硬切换

姿态误差跨过 90° 时，总推力从投影值瞬间跳到 0。

可能造成不连续控制。

### 56.12 期望角速度固定为 0

快速变化的期望姿态缺少角速度前馈。

### 56.13 混控没有处理执行器整体饱和

负 `rpm²` 单独截 0，之后每个电机再独立限幅。

这会改变期望总推力和力矩比例。

更先进的控制分配应在饱和时保持优先级。

### 56.14 最低转速导致无法真正关机

所有低于 1200 的输入都会抬升到 1200。

配合未实现 `enable_motors`，电机无法置零。

### 56.15 NaN 输入被替换为中间转速

```text
(35000 + 1200)/2 = 18100 rpm
```

这是接近悬停甚至更高的转速。

控制计算出错时突然使用中间转速可能不安全。

### 56.16 最大最小转速关系未校验

可以设置：

```text
min_rpm > max_rpm
```

导致限幅逻辑不一致。

### 56.17 `setMass()` 和 `setGravity()` 不校验

质量为 0 会在动力学中除零。

### 56.18 惯性矩阵只检查精确对称

```cpp
if (inertia != inertia.transpose())
```

浮点矩阵几乎对称时也可能被拒绝。

同时没有检查正定性和可逆性。

### 56.19 `J_.inverse()` 每次导数计算

惯性矩阵通常固定，却在每个 ODE 导数评估中求逆。

应预计算：

```text
J_inverse
```

### 56.20 正交化方法并非严格极分解

LLT 下三角因子不是一般意义上的对称平方根。

长期积分可能仍有旋转矩阵误差。

### 56.21 LLT 失败没有检查

若 `RᵀR` 非正定或含 NaN：

```text
llt.matrixL()
```

结果可能无效。

### 56.22 NaN 导数直接置 0 会掩盖错误

系统可能继续运行，但状态已经不可信。

缺少：

- 错误日志；
- 故障状态；
- 安全停止。

### 56.23 状态 NaN 回滚后没有报告完整原因

只打印状态快照，不打印：

- 输入；
- 力矩；
- 参数；
- 导数；
- 命令。

### 56.24 地面模型不处理电机和姿态碰撞

无人机触地后仍可能保持高速电机和翻滚姿态。

### 56.25 地面约束只在 `z<0 && vz<0`

若：

```text
z < 0
vz >= 0
```

位置不会立即投影回地面。

### 56.26 没有障碍物碰撞

地图中的墙、柱体和 Perlin 障碍不会影响动力学。

无人机可穿过障碍物。

规划器只通过地图避免碰撞，仿真器不做物理碰撞验证。

### 56.27 阻力模型参数硬编码

不能通过 ROS 参数调整。

### 56.28 阻力迎风面积不随姿态变化

所有方向使用同一个：

```text
pi d²
```

### 56.29 外力坐标系没有消息标识

`Vector3` 没有 header。

用户必须知道外力按世界系解释、外矩按机体系解释。

### 56.30 扰动没有超时

最后一条扰动永久生效。

### 56.31 物理参数不能从 ROS 配置

类有 setter，但节点只读取初始位置和频率。

质量、惯量、电机和螺旋桨参数全部硬编码。

### 56.32 控制器质量和动力学质量可能不一致

默认两者恰好都是：

```text
0.98 kg
```

但仿真器质量无法通过 launch 同步修改。

### 56.33 `quadrotor_name` 只影响 child frame

不会创建多实例命名空间，也不会改变话题。

### 56.34 frame_id 使用前导斜杠

```text
/simulator
/quadrotor
```

现代 ROS TF 通常不建议前导斜杠。

### 56.35 Odometry 的线速度坐标语义非标准

`state.v` 是世界系速度，却直接写入 `twist.linear`。

ROS Odometry 的 twist 常按 child frame 解释。

当前 `so3_control` 也按世界系使用，因此内部链路一致，但与通用工具可能不兼容。

### 56.36 IMU 时间戳没有设置

发布 odometry 时只更新：

```cpp
odom_msg.header.stamp
```

没有：

```cpp
imu.header.stamp
```

所以 IMU 时间戳通常为 0。

### 56.37 IMU 线加速度语义不标准

发布世界系 `v_dot`，包含重力。

真实加速度计通常输出机体系 specific force。

### 56.38 IMU 没有噪声和 covariance

不适合测试真实状态估计算法。

### 56.39 Odometry 没有 covariance

默认全零可能被某些系统解释为完全确定或未知，取决于使用者。

### 56.40 Odom 发布频率未校验

```cpp
1 / odom_rate
```

若 `odom_rate <= 0` 会除零或产生无效 Duration。

### 56.41 ROS 时间跳变处理不足

使用：

```cpp
next_odom_pub_time += duration
```

仿真时间跳变或暂停后可能出现连续补发或节奏异常。

### 56.42 数值积分使用固定墙上循环而非仿真时钟差

每次始终积分：

```text
dt = 1/simulation_rate
```

若计算负载导致实际循环低于 1000 Hz，模拟时间仍慢于或独立于墙上时间。

### 56.43 `ros::Rate` 不是硬实时

控制和动力学延迟会随系统负载抖动。

### 56.44 没有发布电机转速

内部状态包含 motor rpm，但 ROS 接口无法直接观察。

### 56.45 测试程序没有纳入构建

动力学缺少自动回归测试。

### 56.46 `test_dynamics` 输出的是 step 前状态

循环先读取 `state`，再 `step()`，随后打印旧的局部 `state`。

输出相对积分结果滞后一拍。

### 56.47 测试中的睡眠被注释

性能测试不会按实时速度运行。

### 56.48 `alpha0` 未使用

原本为桨叶攻角模型预留。

相关空气动力学代码仍是 TODO。

### 56.49 螺旋桨半径只用于计算默认 `km`

后续动力学没有显式使用 prop radius。

修改半径 setter 后不会自动重新计算 `km`。

### 56.50 CMake 强制要求未使用的 Armadillo

系统缺少 Armadillo 时会构建失败，尽管源码不需要。

### 56.51 `system_lib` 是无效模板依赖

`catkin_package(DEPENDS Eigen3 system_lib)` 应整理。

### 56.52 package 依赖不完整

源码直接使用：

- `nav_msgs`；
- `sensor_msgs`；
- `geometry_msgs`；
- Eigen3；
- Boost。

但 `package.xml` 未完整声明。

### 56.53 没有安装规则

库、头文件、可执行文件和 launch 没有 install。

### 56.54 内置 Odeint 包含大量无关文件

增加仓库体积和索引负担。

可以只保留必要头文件或使用系统 Boost。

### 56.55 没有线程安全保证

全局 `command` 和 `disturbance` 依赖单线程 `spinOnce()`。

改用多线程 spinner 时需要同步。

---

## 57. 推荐调试方法

### 57.1 检查节点接口

```bash
rosnode info /quadrotor_simulator_so3
```

确认：

- `/so3_cmd` 输入；
- `/visual_slam/odom` 输出；
- 扰动话题；
- IMU 实际名称。

### 57.2 检查频率

```bash
rostopic hz /so3_cmd
rostopic hz /visual_slam/odom
rostopic hz /quadrotor_simulator_so3/imu
```

### 57.3 检查悬停

```bash
rostopic echo /so3_cmd/force
rostopic echo /visual_slam/odom/pose/pose/position
```

水平悬停时：

```text
force_z ≈ 9.61 N
```

### 57.4 注入外力

```bash
rostopic pub /force_disturbance \
  geometry_msgs/Vector3 \
  "{x: 1.0, y: 0.0, z: 0.0}"
```

观察水平位移。

停止扰动需要显式发布零向量。

### 57.5 注入外力矩

```bash
rostopic pub /moment_disturbance \
  geometry_msgs/Vector3 \
  "{x: 0.0, y: 0.0, z: 0.01}"
```

观察 yaw 变化和控制恢复。

### 57.6 检查 IMU 时间戳

```bash
rostopic echo -n 1 \
  /quadrotor_simulator_so3/imu/header
```

当前 stamp 可能为 0。

### 57.7 检查坐标语义

让无人机 yaw 旋转 90° 后沿世界 x 方向飞。

比较：

```text
odom.twist.linear
```

验证它仍表达世界系速度。

### 57.8 观察地面行为

关闭控制或给向下外力，让无人机落地。

观察：

- z 是否截为 0；
- 姿态是否继续变化；
- 水平速度是否保留；
- 电机是否仍旋转。

---

## 58. 常见故障

### 58.1 无人机启动后立即下落

检查：

- `/so3_cmd` 是否已经到达；
- `so3_control` 是否有 odometry；
- 初始命令四元数是否有效；
- 合力是否约为 `mg`；
- 姿态误差是否导致 `Psi >= 1`。

### 58.2 无人机永远无法完全关机

原因：

- 忽略 `enable_motors`；
- `min_rpm = 1200`；
- 目标 0 rpm 会被限幅。

### 58.3 电机转速或状态出现 NaN

检查：

- 期望四元数；
- `kf_correction`；
- 惯性矩阵；
- 控制增益；
- 上一轮 control 是否初始化；
- 正交化是否失败。

### 58.4 发布扰动后一直偏移

扰动是保持型输入。

需要发布零向量取消。

### 58.5 IMU 订阅不到

默认 EGO-Planner launch 没有重映射。

实际话题：

```text
/quadrotor_simulator_so3/imu
```

### 58.6 IMU 加速度与预期相反

当前发布的是世界系惯性加速度并包含重力，不是标准加速度计读数。

### 58.7 无人机穿过障碍物

该仿真器没有地图碰撞模型。

### 58.8 仿真速度变慢

默认 1000 Hz 积分不是硬实时。

高 CPU 负载时墙上运行速度可能下降。

---

## 59. 推荐改进方向

### 59.1 低风险修复

优先建议：

1. 初始化 `external_moment_` 和 `acc_`；
2. 初始化 `Control`；
3. 增加已接收合法命令标志；
4. 检查命令超时；
5. 使用 `enable_motors`；
6. 校验四元数、`kf` 和参数；
7. 设置 IMU 时间戳；
8. 校验 odom_rate；
9. 删除 Armadillo 无用依赖；
10. 补齐 package 依赖。

### 59.2 参数化动力学模型

通过 ROS/YAML 配置：

```text
mass
gravity
inertia
arm_length
kf
km
motor_time_constant
min/max rpm
drag coefficient
```

并与 `so3_control` 的质量保持一致。

### 59.3 使用更安全的控制分配

加入执行器饱和约束：

```text
优先保持总推力
其次 roll/pitch
最后 yaw
```

或求解带上下界的二次规划。

### 59.4 正确处理电机关闭

```text
enable_motors = false
  -> command rpm = 0
  -> 允许实际 rpm 按时间常数衰减到 0
```

最低怠速只在已 armed 状态使用。

### 59.5 改进旋转积分

可选择：

- 四元数积分并归一化；
- 李群积分器；
- 指数映射更新；
- SVD 极分解投影。

### 59.6 预计算惯性矩阵逆

物理参数不变时：

```text
J_inv
```

只需计算一次。

### 59.7 标准化 IMU

输出：

```text
specific_force_body =
  Rᵀ(v_dot + g e3)
```

并添加：

- 时间戳；
- covariance；
- 噪声；
- bias；
- 随机游走。

### 59.8 标准化 Odometry

明确：

- pose 在 header frame；
- twist 在 child frame或世界系；

必要时把世界系速度旋转到机体系。

### 59.9 增加碰撞模型

至少支持：

- 地面接触；
- 地图占据碰撞；
- 碰撞停止或反弹；
- 碰撞事件话题。

### 59.10 增加自动测试

测试：

- 悬停；
- 自由落体；
- 单轴力矩；
- 电机一阶响应；
- 外力响应；
- 能量和旋转矩阵正交性；
- 饱和；
- NaN 输入；
- 电机禁用。

---

## 60. 推荐源码阅读顺序

### 第一阶段：理解状态和参数

阅读：

```text
include/quadrotor_simulator/Quadrotor.h
```

重点：

- `State`；
- 22 维内部状态；
- 电机编号；
- 物理 setter；
- `step()` 和 `operator()`。

### 第二阶段：理解动力学

阅读：

```text
src/dynamics/Quadrotor.cpp
```

建议顺序：

1. 构造函数；
2. `setInput()`；
3. `operator()`；
4. `step()`；
5. 参数 getter/setter。

### 第三阶段：理解内环

阅读：

```text
src/quadrotor_simulator_so3.cpp
```

先看：

```text
Command
getControl()
cmd_callback()
```

### 第四阶段：理解 ROS 循环

继续阅读：

```text
main()
stateToOdomMsg()
quadToImuMsg()
```

### 第五阶段：理解上游外环

阅读：

```text
uav_simulator/so3_control/
```

理解 `SO3Command.force` 和 `orientation` 的来源。

### 第六阶段：运行独立示例

阅读：

```text
launch/simulator_example.launch
src/test_dynamics/test_dynamics.cpp
```

---

## 61. 建议学习实验

### 实验一：悬停转速

根据公式计算约 16350 rpm。

将四电机设为该值，验证高度近似不变。

### 实验二：电机阶跃

从 0 跳到悬停转速。

记录实际 rpm，拟合时间常数是否约 `1/30 s`。

### 实验三：自由落体

未发送命令，观察：

- 初始 1200 rpm 怠速；
- z 下降；
- 地面截断。

### 实验四：单轴 roll

令：

```text
w3 > w4
```

观察正 roll 力矩方向。

### 实验五：yaw 混控

增加 1、2 号电机并减小 3、4 号，观察 yaw 方向。

### 实验六：外力扰动

施加固定水平力，验证：

```text
初始加速度 ≈ F/m
```

并观察阻力导致终端速度。

### 实验七：外力矩扰动

施加 z 力矩，观察内环恢复。

### 实验八：姿态误差边界

构造 89°、91° 的期望姿态差。

观察总推力在 `Psi=1` 附近的硬切换。

### 实验九：执行器饱和

发送极大力矩命令，检查 rpm 截断后实际控制效果。

### 实验十：旋转矩阵正交性

长时间高速旋转，记录：

```text
||RᵀR-I||
det(R)
```

验证当前 LLT 修正效果。

### 实验十一：IMU 语义

水平静止悬停时查看：

```text
linear_acceleration
```

当前世界系惯性加速度应接近 0，而真实加速度计 z 轴通常读约 `+g` 或 `-g`，取决于约定。

### 实验十二：地面模型

让无人机带水平速度和倾斜姿态落地，观察非真实行为。

---

## 62. 常见问题

### Q1：这个包只是动力学模型吗？

不是。

它还包含 SO(3) 姿态内环和电机混控。

### Q2：输入为什么不是四个电机转速？

ROS 节点输入是更高层的 `SO3Command`。

节点内部先计算力矩和四电机目标转速。

`Quadrotor` 动力学类本身的输入才是四个 rpm。

### Q3：`SO3Command.force` 在哪个坐标系？

世界坐标系。

仿真器将其投影到当前机体 z 轴。

### Q4：`omega` 在哪个坐标系？

机体系。

因为姿态运动学使用：

```text
R_dot = R Omega_hat
```

### Q5：外力在哪个坐标系？

世界系。

外力矩按机体系使用。

### Q6：电机命令能瞬间达到吗？

不能。

使用一阶电机模型，时间常数约 0.033 秒。

### Q7：为什么电机不能低于 1200 rpm？

模型设置了怠速下限。

但当前没有独立的 disarmed 状态，这是工程缺陷。

### Q8：为什么姿态误差大于 90° 时没有推力？

代码只在 `Psi<1` 的几何控制吸引域内启用平移推力。

### Q9：仿真器会撞墙吗？

不会。

只实现了极简地面约束，没有地图碰撞。

### Q10：IMU 可以直接用于真实 VIO 测试吗？

不建议。

其加速度坐标系、重力语义、噪声和时间戳都不符合严格传感器模型。

### Q11：动力学参数可以通过 launch 修改吗？

当前不可以。

虽然类有 setter，ROS 节点没有读取对应参数。

### Q12：为什么源码自带整套 Odeint？

可能是为了固定依赖版本并支持旧 ROS 环境。

### Q13：测试程序会自动编译吗？

不会。

CMake 没有为它创建目标。

### Q14：默认积分频率是多少？

1000 Hz。

### Q15：默认 odometry 频率是多少？

类默认 100 Hz，EGO-Planner launch 设置为 200 Hz。

---

## 63. 总结

`so3_quadrotor_simulator` 是 EGO-Planner 仿真闭环中的控制内环和被控对象。

它完成：

```text
SO3Command
  -> 姿态误差与角速度误差
  -> 总推力和控制力矩
  -> 四电机目标 rpm
  -> 电机一阶动态
  -> 六自由度刚体动力学
  -> Odometry 与 IMU
```

核心动力学：

```text
x_dot = v

v_dot =
  -g e3
  + T R e3 / m
  + Fext / m
  - drag / m

R_dot = R Omega_hat

Omega_dot =
  J^-1(M - Omega×J Omega + Mext)

rpm_dot =
  (rpm_cmd-rpm)/tau
```

理解本包时最重要的九条主线是：

1. **22 维状态同时积分刚体和四个电机。**
2. **SO(3) 姿态内环使用 `eR`、`eOmega` 和陀螺补偿。**
3. **世界系期望合力被投影到当前机体 z 轴。**
4. **总推力和三轴力矩通过解析混控转换为四个 rpm。**
5. **电机具有一阶响应和 1200～35000 rpm 限幅。**
6. **动力学包含重力、二次阻力、外力和外力矩。**
7. **Boost.Odeint 默认以 1000 Hz 积分 22 维状态。**
8. **Odometry 是主要闭环反馈，IMU 只是简化调试输出。**
9. **仿真器没有障碍碰撞、真实传感器噪声和完整安全状态机。**

默认闭环：

```text
so3_control
  -> /so3_cmd
  -> so3_quadrotor_simulator
  -> /visual_slam/odom
  -> so3_control + local_sensing + planner
```

当前实现足以支撑 EGO-Planner 的算法演示，但在：

- 初始化安全；
- 电机启停；
- 参数化；
- 执行器饱和；
- 旋转积分；
- IMU 语义；
- 碰撞模型；
- 自动测试；

方面仍有明确的改进空间。

结合：

```text
so3_control
quadrotor_msgs
traj_server
local_sensing
```

一起阅读，就能完整理解 EGO-Planner 从轨迹命令到仿真无人机运动状态的整个闭环。
