# traj_utils 功能包详细学习指南

> 源码目录：`/home/yxc/Desktop/ego-planner_ws/src/planner/traj_utils`
>
> 本文基于当前工作空间中的实际源码整理，重点解释多项式轨迹的数学表示、轨迹生成方法、RViz 可视化接口、它在 EGO-Planner 主流程中的实际用途，以及当前实现中值得留意的边界条件和潜在问题。

---

## 1. 一句话认识 traj_utils

`traj_utils` 是 EGO-Planner 中的**轨迹基础工具库**。

它不负责地图构建、路径搜索或 B 样条优化，也不会独立启动 ROS 节点。它主要提供两类能力：

```text
traj_utils
├── PolynomialTraj
│   ├── 构造单段五次多项式轨迹
│   ├── 构造经过多个航点的分段多项式轨迹
│   ├── 查询位置、速度和加速度
│   └── 提供若干轨迹统计接口
└── PlanningVisualization
    ├── 显示目标点
    ├── 显示全局参考轨迹
    ├── 显示局部轨迹初始化点
    ├── 显示优化后控制点
    └── 显示 A* 路径与箭头
```

它在整个规划系统中的位置可以概括为：

```text
目标点 / 航点
      ↓
traj_utils::PolynomialTraj
生成平滑全局参考或局部初始化曲线
      ↓
plan_manage
将参考曲线采样并交给 B 样条优化器
      ↓
traj_utils::PlanningVisualization
把规划过程发布为 RViz Marker
```

---

## 2. 包的性质

## 2.1 它是库包，不是节点包

`traj_utils` 的 `CMakeLists.txt` 只构建一个共享库：

```cmake
add_library(traj_utils
  src/planning_visualization.cpp
  src/polynomial_traj.cpp
)
```

构建结果位于：

```text
devel/lib/libtraj_utils.so
```

该包没有：

- `add_executable()`；
- 独立 ROS 节点；
- launch 文件；
- 自定义消息；
- 参数文件；
- 测试文件。

因此不能直接运行：

```bash
rosrun traj_utils ...
```

其他包需要在 CMake 中依赖并链接 `traj_utils`，然后在源码中调用它提供的类。

## 2.2 目录结构

```text
traj_utils/
├── CMakeLists.txt
├── package.xml
├── include/traj_utils/
│   ├── planning_visualization.h
│   └── polynomial_traj.h
└── src/
    ├── planning_visualization.cpp
    └── polynomial_traj.cpp
```

当前包总共约 982 行源码和构建配置，内容集中，适合完整通读。

---

## 3. 文件职责

| 文件 | 作用 |
|---|---|
| `polynomial_traj.h` | 定义 `PolynomialTraj`，并以内联方式实现大部分查询和统计函数 |
| `polynomial_traj.cpp` | 实现单段五次多项式生成和多段闭式优化轨迹生成 |
| `planning_visualization.h` | 定义 `PlanningVisualization` 可视化接口 |
| `planning_visualization.cpp` | 构造并发布 ROS `Marker` / `MarkerArray` |
| `CMakeLists.txt` | 构建 `libtraj_utils.so` |
| `package.xml` | 声明 ROS 包元数据和依赖 |

---

## 4. 构建与依赖

## 4.1 CMake 中声明的依赖

```text
bspline_opt
path_searching
roscpp
std_msgs
cv_bridge
Eigen3
PCL
```

## 4.2 实际源码需要的核心依赖

从当前源码看，真正直接使用的主要内容是：

```text
Eigen
roscpp
geometry_msgs
visualization_msgs
```

`planning_visualization.h` 还包含：

```cpp
#include <bspline_opt/uniform_bspline.h>
```

但当前类接口中没有实际使用 `UniformBspline`。

`path_searching`、`cv_bridge` 和 PCL 也没有在当前 `traj_utils` 源码中直接使用。

这意味着当前构建依赖偏重，会导致：

- 编译和链接链条变长；
- 下游包间接携带大量无关依赖；
- `libtraj_utils.so` 构建时链接不必要的 OpenCV、PCL 和规划库；
- 单独复用 `PolynomialTraj` 的成本变高。

## 4.3 package.xml 不完整

`PlanningVisualization` 直接使用：

```text
geometry_msgs
visualization_msgs
```

但当前 `package.xml` 没有声明它们。

在完整工作空间中，这些依赖可能由其他包间接提供，所以仍能构建；在干净环境、CI 或单独发布该包时则可能失败。

## 4.4 编译设置

当前包使用：

```cmake
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")
```

含义：

- 使用 C++11；
- Release 模式启用 `-O3`；
- 启用常见警告；
- 同时保留调试符号 `-g`。

这是一种“高优化但仍可调试”的配置。

---

## 5. 在 EGO-Planner 中的实际调用关系

当前工作空间中，`traj_utils` 的主要直接使用者是：

```text
planner/plan_manage
```

具体关系如下：

```text
EGOReplanFSM
├── 创建 PlanningVisualization
├── 显示目标点
├── 显示全局参考轨迹
└── 显示优化轨迹控制点

EGOPlannerManager
├── 调用 PolynomialTraj::one_segment_traj_gen()
├── 调用 PolynomialTraj::minSnapTraj()
├── evaluate() 采样位置
├── evaluateVel() 查询边界速度
├── evaluateAcc() 查询边界加速度
├── 显示初始轨迹
└── 显示 A* 搜索路径
```

主流程中并未使用 `PolynomialTraj` 的大多数统计接口，例如：

```text
getLength()
getMeanVel()
getAccCost()
getJerk()
getMeanAndMaxVel()
getMeanAndMaxAcc()
```

这些接口更像历史遗留的评估工具，且其中部分实现存在明显问题，不能未经验证地用于实验结论。

---

# 第一部分：PolynomialTraj

## 6. PolynomialTraj 的核心职责

`PolynomialTraj` 表示一条由若干段多项式拼接而成的三维轨迹。

每一段在 x、y、z 三个轴上分别使用一条一维多项式：

```text
x(t) = a₅t⁵ + a₄t⁴ + a₃t³ + a₂t² + a₁t + a₀
y(t) = b₅t⁵ + b₄t⁴ + b₃t³ + b₂t² + b₁t + b₀
z(t) = c₅t⁵ + c₄t⁴ + c₃t³ + c₂t² + c₁t + c₀
```

当前两个静态生成器均产生五次多项式，每个轴每一段有 6 个系数。

## 6.1 为什么是三条一维多项式

三维轨迹可以写成：

```text
p(t) = [x(t), y(t), z(t)]ᵀ
```

三个轴共享同一段时间，但分别求解系数。这样可以把三维轨迹问题拆成三个结构相同的一维问题。

## 6.2 为什么使用五次多项式

一条五次多项式有 6 个未知系数。

如果已知：

```text
起点位置 p(0)
起点速度 v(0)
起点加速度 a(0)
终点位置 p(T)
终点速度 v(T)
终点加速度 a(T)
```

正好得到 6 个边界约束，可以唯一求解 6 个系数。

---

## 7. 内部数据结构

`PolynomialTraj` 的主要成员为：

```cpp
vector<double> times;
vector<vector<double>> cxs;
vector<vector<double>> cys;
vector<vector<double>> czs;

double time_sum;
int num_seg;
```

### `times`

保存每一段的持续时间：

```text
times[0] = 第 0 段持续时间
times[1] = 第 1 段持续时间
...
```

### `cxs`、`cys`、`czs`

分别保存每一段 x、y、z 轴的多项式系数。

例如：

```text
cxs[2]
```

表示第 2 段 x 轴多项式的全部系数。

### 系数顺序

类中最终保存的顺序是**从最高次项到常数项**：

```text
[a₅, a₄, a₃, a₂, a₁, a₀]
```

这点非常重要，因为 `evaluate()`、`evaluateVel()` 和 `evaluateAcc()` 都依赖这一顺序。

### `time_sum`

保存所有分段持续时间之和。

### `num_seg`

保存分段数量，但当前源码中基本没有被后续逻辑使用。

### 缓存成员

```cpp
vector<Eigen::Vector3d> traj_vec3d;
double length;
```

用于 `getTraj()` 和 `getLength()` 的离散轨迹与长度缓存。

---

## 8. 对象生命周期与正确调用顺序

一个典型使用流程为：

```cpp
PolynomialTraj traj;

traj.addSegment(cx, cy, cz, duration);
traj.addSegment(...);

traj.init();

Eigen::Vector3d p = traj.evaluate(t);
Eigen::Vector3d v = traj.evaluateVel(t);
Eigen::Vector3d a = traj.evaluateAcc(t);
```

静态生成器的典型用法为：

```cpp
PolynomialTraj traj =
    PolynomialTraj::one_segment_traj_gen(...);

traj.init();
```

当前实现中：

- 构造函数不初始化 `time_sum`、`num_seg` 和 `length`；
- `addSegment()` 不更新总时间；
- `one_segment_traj_gen()` 不调用 `init()`；
- `minSnapTraj()` 不调用 `init()`。

因此调用者必须在查询总时长或使用依赖总时长的函数前显式调用：

```cpp
traj.init();
```

在 EGO-Planner 主流程中，`GlobalTrajData::setGlobalTraj()` 会完成这一步：

```cpp
global_traj_ = traj;
global_traj_.init();
```

---

## 9. 基础管理接口

## 9.1 reset()

```cpp
void reset()
```

清空：

```text
times
cxs
cys
czs
```

并把：

```text
time_sum = 0
num_seg = 0
```

当前实现不会清空 `traj_vec3d`，也不会重置 `length`。

如果在 `reset()` 后直接调用依赖旧缓存的函数，可能得到与当前轨迹不一致的结果。

## 9.2 addSegment()

```cpp
void addSegment(vector<double> cx,
                vector<double> cy,
                vector<double> cz,
                double t)
```

向轨迹尾部添加一段。

调用者需要保证：

- x、y、z 系数长度一致；
- 系数按最高次到最低次排列；
- 持续时间大于 0；
- 添加完成后调用 `init()`。

当前实现不进行这些合法性检查。

## 9.3 init()

```cpp
void init()
```

完成：

```text
num_seg = times.size()
time_sum = 所有分段时间之和
```

它不是构造轨迹，而是对已有分段信息做汇总。

## 9.4 getTimes()

返回所有分段时间的副本。

## 9.5 getCoef(axis)

根据轴编号返回系数：

```text
axis = 0 → x
axis = 1 → y
axis = 2 → z
```

非法轴编号会打印错误并返回空二维数组。

由于返回的是副本，外部修改不会影响轨迹内部系数。

---

## 10. 分段时间定位

位置、速度和加速度查询都使用类似逻辑：

```cpp
int idx = 0;
while (times[idx] + 1e-4 < t) {
  t -= times[idx];
  ++idx;
}
```

输入 `t` 是整条轨迹上的全局时间。

循环会找到 `t` 所在的分段，并把 `t` 转换为该段内部的局部时间。

例如：

```text
第 0 段：2 秒
第 1 段：3 秒
查询全局 t = 3.2 秒
```

定位后：

```text
idx = 1
局部 t = 1.2 秒
```

### `1e-4` 的作用

`times[idx] + 1e-4 < t` 给分段边界留了一个很小的容差。

在边界附近，查询可能仍落在前一段，并对前一段做极小范围的外推。

### 边界风险

当前定位逻辑没有检查：

- `times` 是否为空；
- `t` 是否为负；
- `t` 是否大于总时长；
- `idx` 是否超过分段数量。

如果 `t > time_sum + 1e-4`，循环可能越界访问 `times[idx]`。

---

## 11. 位置求值 evaluate()

接口：

```cpp
Eigen::Vector3d evaluate(double t)
```

对于系数：

```text
cx = [a₅, a₄, a₃, a₂, a₁, a₀]
```

代码构造时间幂向量：

```text
tv = [t⁵, t⁴, t³, t², t, 1]
```

然后计算：

```text
x(t) = tv · cx
y(t) = tv · cy
z(t) = tv · cz
```

返回：

```text
[x(t), y(t), z(t)]ᵀ
```

---

## 12. 速度求值 evaluateVel()

接口：

```cpp
Eigen::Vector3d evaluateVel(double t)
```

若：

```text
x(t) = a₅t⁵ + a₄t⁴ + a₃t³ + a₂t² + a₁t + a₀
```

则：

```text
vx(t) = 5a₅t⁴ + 4a₄t³ + 3a₃t² + 2a₂t + a₁
```

代码先根据位置系数构造速度系数，再与：

```text
[1, t, t², t³, t⁴]
```

进行点积。

注意这里速度系数的内部排列与位置查询时不同，但最终数学含义一致。

---

## 13. 加速度求值 evaluateAcc()

接口：

```cpp
Eigen::Vector3d evaluateAcc(double t)
```

位置多项式二阶求导得到：

```text
ax(t) = 20a₅t³ + 12a₄t² + 6a₃t + 2a₂
```

三个轴分别计算后组合为三维加速度。

主规划流程会使用它获取：

- 全局轨迹起点加速度；
- 全局轨迹局部目标处加速度；
- 新旧轨迹衔接边界条件。

---

## 14. 单段五次轨迹生成

接口：

```cpp
PolynomialTraj::one_segment_traj_gen(
    start_pt,
    start_vel,
    start_acc,
    end_pt,
    end_vel,
    end_acc,
    duration);
```

## 14.1 输入

```text
起点位置、速度、加速度
终点位置、速度、加速度
总持续时间 T
```

## 14.2 求解问题

对每个轴求解：

```text
p(t) = a₅t⁵ + a₄t⁴ + a₃t³ + a₂t² + a₁t + a₀
```

满足：

```text
p(0)  = p_start
p'(0) = v_start
p''(0)= a_start

p(T)  = p_end
p'(T) = v_end
p''(T)= a_end
```

## 14.3 约束矩阵

代码构造 6×6 矩阵：

```text
[ 0      0      0      0    0  1 ]   起点位置
[ 0      0      0      0    1  0 ]   起点速度
[ 0      0      0      2    0  0 ]   起点加速度
[ T⁵     T⁴     T³     T²   T  1 ]   终点位置
[ 5T⁴    4T³    3T²    2T   1  0 ]   终点速度
[ 20T³   12T²   6T     2    0  0 ]   终点加速度
```

未知量为：

```text
[a₅, a₄, a₃, a₂, a₁, a₀]ᵀ
```

然后使用 Eigen 的：

```cpp
C.colPivHouseholderQr().solve(B)
```

求解系数。

相比直接求逆，QR 求解通常具有更好的数值稳定性。

## 14.4 在主流程中的用途

当全局路径只有起点和终点两个点时，规划管理器使用该函数生成全局参考轨迹。

它也用于：

- 第一次局部规划的多项式初始化；
- 旧局部轨迹末端连接到新局部目标；
- 随机扰动初始化中的分段连接。

---

## 15. 多段闭式优化轨迹 minSnapTraj()

接口：

```cpp
PolynomialTraj::minSnapTraj(
    Pos,
    start_vel,
    end_vel,
    start_acc,
    end_acc,
    Time);
```

## 15.1 输入矩阵

`Pos` 为 3×N 航点矩阵：

```text
Pos.col(0) = 起点
Pos.col(1) = 中间航点 1
...
Pos.col(N-1) = 终点
```

`Time` 长度应为：

```text
N - 1
```

每个元素表示对应分段的持续时间。

## 15.2 输出轨迹

输出由：

```text
N - 1
```

段五次多项式组成。

轨迹经过全部航点，并满足：

- 起点速度和加速度约束；
- 终点速度和加速度约束；
- 内部航点处位置连续；
- 内部航点处速度连续；
- 内部航点处加速度连续。

即至少具有 `C²` 连续性。

---

## 16. 名称与实际目标函数

函数名为：

```text
minSnapTraj
```

源码注释也写着：

```text
minimum snap matrix
```

但当前实现实际构造的是三阶导数代价矩阵。

它对每段计算的形式为：

```text
∫ ||p'''(t)||² dt
```

其中：

```text
p'''(t) = jerk
```

因此从当前代码的数学实现看，它更准确地说是：

```text
minimum jerk trajectory
```

而不是：

```text
minimum snap trajectory
```

真正的 minimum snap 通常最小化四阶导数平方积分，并常使用至少七次多项式，以提供足够自由度。

阅读论文、比较算法或撰写实验报告时，应以源码实际目标函数为准。

---

## 17. minSnapTraj() 的矩阵推导思路

虽然源码矩阵较多，但它的整体思路可以分为五步。

## 17.1 第一步：定义每段端点导数

每段保存 6 个端点量：

```text
[起点位置,
 终点位置,
 起点速度,
 终点速度,
 起点加速度,
 终点加速度]
```

对 x、y、z 分别构造：

```text
Dx
Dy
Dz
```

## 17.2 第二步：构造系数到端点导数的映射 A

对于每一段，`Ab` 将多项式系数映射为该段两端的位置、速度和加速度。

所有分段的 `Ab` 放在大矩阵 `A` 的对角块中：

```text
A = block_diag(A₀, A₁, ..., Aₘ₋₁)
```

## 17.3 第三步：选择固定变量和自由变量

固定变量包括：

- 所有航点位置；
- 起点速度；
- 起点加速度；
- 终点速度；
- 终点加速度。

自由变量包括：

- 每个内部航点的速度；
- 每个内部航点的加速度。

选择矩阵 `Ct` 将各段重复的端点量映射到一组统一变量，并确保相邻段在内部航点共享同一速度和加速度。

## 17.4 第四步：构造 jerk 代价矩阵 Q

对每一段：

```text
J = ∫₀ᵀ ||p'''(t)||² dt
```

可写成二次型：

```text
J = cᵀ Q c
```

其中 `c` 是多项式系数。

三维轨迹分别对 x、y、z 求解，但使用同样的矩阵结构。

## 17.5 第五步：闭式求解自由导数

将总代价写成固定变量与自由变量的二次型：

```text
R = C A⁻ᵀ Q A⁻¹ Cᵀ
```

并分块：

```text
R = [Rff Rfp
     Rpf Rpp]
```

最优自由变量满足：

```text
Dp = -Rpp⁻¹ Rpf Df
```

源码使用等价形式求出内部航点速度和加速度，再恢复每段多项式系数。

---

## 18. minSnapTraj() 在主流程中的用途

当全局参考路径包含三个或更多点时，`EGOPlannerManager` 调用该函数。

典型过程：

```text
起点与目标距离较远
      ↓
规划管理器在线段之间插入中间点
      ↓
根据距离和最大速度分配每段时间
      ↓
调用 minSnapTraj()
      ↓
得到平滑分段全局参考轨迹
      ↓
状态机沿该轨迹滚动选取局部目标
```

这条多项式轨迹主要是方向和进度参考，不负责障碍物避让。

局部避障最终由 B 样条优化器完成。

---

## 19. 轨迹离散与统计接口

## 19.1 getTimeSum()

返回 `time_sum`。

必须在 `init()` 后使用，否则可能读取未初始化值。

## 19.2 getTraj()

以固定：

```text
0.01 s
```

时间步长对整条轨迹采样，并缓存到 `traj_vec3d`。

当前循环条件：

```text
eval_t < time_sum
```

因此不会采样精确终点。

## 19.3 getLength()

对 `traj_vec3d` 中相邻采样点的距离求和，近似轨迹长度。

它依赖调用者事先执行：

```cpp
getTraj();
```

如果缓存为空，当前实现会直接访问：

```text
traj_vec3d[0]
```

从而发生越界。

## 19.4 getMeanVel()

设计意图是返回：

```text
平均速度 = 轨迹长度 / 总时长
```

但当前函数只计算局部变量，没有 `return`：

```cpp
double getMeanVel() {
  double mean_vel = length / time_sum;
}
```

调用它会产生未定义行为，不能使用其返回值。

## 19.5 getAccCost()

当前实现取每段多项式的二次项系数，计算该段起始时刻的加速度，并乘以持续时间。

这并不是一般意义上的：

```text
∫ ||a(t)||² dt
```

对于五次多项式，加速度会随时间变化，因此该函数不能准确表示整段加速度代价。

## 19.6 getJerk()

该函数构造 jerk 二次型并计算：

```text
∫ ||p'''(t)||² dt
```

它与多段轨迹生成中的目标函数形式一致。

## 19.7 getMeanAndMaxVel()

设计意图是在每段内以 `0.01 s` 采样速度，并统计均值和最大值。

但当前循环内构造时间幂时使用了：

```cpp
pow(ts, i)
```

而不是：

```cpp
pow(eval_t, i)
```

因此它会重复计算该段末端速度，无法得到真实平均速度或最大速度。

## 19.8 getMeanAndMaxAcc()

与速度统计存在相同问题：循环中重复计算分段末端加速度，而不是按 `eval_t` 采样。

这两个统计函数当前未被 EGO-Planner 主流程使用。

---

# 第二部分：PlanningVisualization

## 20. PlanningVisualization 的职责

`PlanningVisualization` 把规划器内部的 Eigen 点、路径和控制点转换为 ROS 可视化消息。

它并不负责打开 RViz，而是持续发布：

```text
visualization_msgs/Marker
visualization_msgs/MarkerArray
```

RViz 订阅这些话题后显示规划过程。

---

## 21. 构造函数与发布话题

构造函数接收一个 `ros::NodeHandle`，并创建以下发布器：

| 相对话题名 | 消息类型 | 主要内容 |
|---|---|---|
| `goal_point` | `visualization_msgs/Marker` | 目标点 |
| `global_list` | `visualization_msgs/Marker` | 全局参考轨迹 |
| `init_list` | `visualization_msgs/Marker` | 局部规划初始路径 |
| `optimal_list` | `visualization_msgs/Marker` | 优化后控制点 |
| `a_star_list` | `visualization_msgs/Marker` | A* 路径 |

在 `ego_planner_node` 中传入的是私有 NodeHandle，因此实际话题通常是：

```text
/ego_planner_node/goal_point
/ego_planner_node/global_list
/ego_planner_node/init_list
/ego_planner_node/optimal_list
/ego_planner_node/a_star_list
```

当前 RViz 配置也订阅这些绝对话题。

头文件还声明了：

```text
guide_vector_pub
intermediate_state_pub
```

但构造函数没有 advertise 它们，当前主流程也未使用。

---

## 22. displayMarkerList()

这是最常用的基础显示函数。

输入：

```text
发布器
三维点列表
显示尺度
RGBA 颜色
Marker ID
```

它同时构造两个 Marker：

```text
SPHERE_LIST  用球显示所有点
LINE_STRIP   用折线连接所有点
```

ID 规则：

```text
球列表 ID = id
折线 ID = id + 1000
```

两者都使用：

```text
frame_id = "world"
```

Marker 不设置 lifetime，因此默认会一直保留，直到同一话题中相同 namespace 和 ID 的新 Marker 覆盖它。

---

## 23. generatePathDisplayArray()

该函数与 `displayMarkerList()` 类似，但不会立即发布，而是把两个 Marker 加入一个 `MarkerArray`。

区别包括：

```text
frame_id = "map"
ID 使用 id 和 id + 1
折线宽度为 scale / 3
```

当前工作空间主流程没有直接调用该函数。

它与其他显示函数使用不同坐标系，二次开发时要确保 `map` 与 `world` 之间存在正确 TF。

---

## 24. generateArrowDisplayArray()

输入点列表被按两个点一组解释：

```text
list[0] → list[1]  第一根箭头
list[2] → list[3]  第二根箭头
...
```

每根箭头是一个 `visualization_msgs/Marker::ARROW`。

ID 规则：

```text
arrow.id = arrow_index + id × 1000
```

若输入点数量为奇数，最后一个点会被忽略。

当前使用：

```text
frame_id = "map"
```

---

## 25. 具体显示接口

## 25.1 displayGoalPoint()

显示单个球形目标点。

```text
类型：SPHERE
坐标系：world
颜色：由调用者指定
尺度：由调用者指定
```

状态机用它显示手动目标或预设航点。

## 25.2 displayGlobalPathList()

显示全局参考轨迹。

默认颜色：

```text
RGBA = (0, 0.5, 0.5, 1)
```

即青绿色。

它在发布前检查是否存在订阅者；若当前没有 RViz 订阅，则直接返回。

## 25.3 displayInitPathList()

显示局部轨迹优化前的初始化点。

默认颜色：

```text
RGBA = (0, 0, 1, 1)
```

即蓝色。

这有助于比较：

```text
初始路径
      ↓ 优化
最终控制点 / 最终轨迹
```

## 25.4 displayOptimalList()

接收一个控制点矩阵：

```text
每一列为一个三维控制点
```

转换为点列表后显示。

默认颜色：

```text
RGBA = (1, 0, 0, 1)
```

即红色。

当前状态机在发布成功轨迹时调用它显示位置 B 样条控制点。

## 25.5 displayAStarList()

显示多条 A* 路径。

每条路径使用：

```text
粉色
随机尺度
不同 Marker ID
```

当前源码还会在每次调用时打印：

```text
yxc yxc yxc yxc yxc yxc
```

这是明显的个人调试输出，会污染运行日志。

## 25.6 displayArrowList()

该函数先发布一个空 `MarkerArray`，然后生成并发布箭头数组。

但空数组并不会自动删除 RViz 中旧 Marker。若希望可靠清理旧箭头，应使用：

```text
Marker::DELETE
Marker::DELETEALL
```

或显式覆盖所有旧 ID。

---

## 26. 默认 RViz 视觉含义

结合当前代码，可以按以下方式识别规划显示：

| 内容 | 默认颜色 | 形态 |
|---|---|---|
| 目标点 / 航点 | 调用者指定，常为青绿色 | 单个球 |
| 全局参考轨迹 | 青绿色 | 点列 + 折线 |
| 局部初始化轨迹 | 蓝色 | 点列 + 折线 |
| 优化控制点 | 红色 | 点列 + 折线 |
| A* 引导路径 | 粉色 | 点列 + 折线 |

这些可视化表达的是不同规划阶段，不应把全局参考线、A* 路径或控制点折线直接当作最终飞行轨迹。

---

## 27. 可视化数据链

```text
EGOReplanFSM / EGOPlannerManager
          ↓ Eigen::Vector3d / Eigen::MatrixXd
PlanningVisualization
          ↓ visualization_msgs::Marker
/ego_planner_node/*
          ↓
RViz
```

规划器不会等待 RViz，也不会依赖 RViz 才能运行。

对于多数路径显示函数，如果没有订阅者，代码会跳过消息构造和发布，从而减少不必要的运行开销。

---

## 28. 调试可视化话题

查看当前发布者和订阅者：

```bash
rostopic info /ego_planner_node/global_list
rostopic info /ego_planner_node/init_list
rostopic info /ego_planner_node/optimal_list
rostopic info /ego_planner_node/a_star_list
```

查看消息：

```bash
rostopic echo /ego_planner_node/goal_point
rostopic echo /ego_planner_node/global_list
```

查看发布频率：

```bash
rostopic hz /ego_planner_node/optimal_list
```

可视化话题通常只在规划事件发生时发布，不是固定高频话题。

---

## 29. 坐标系注意事项

当前代码混合使用：

```text
world
map
```

具体而言：

- `displayMarkerList()` 使用 `world`；
- `displayGoalPoint()` 使用 `world`；
- `generatePathDisplayArray()` 使用 `map`；
- `generateArrowDisplayArray()` 使用 `map`。

默认 RViz Fixed Frame 为：

```text
world
```

如果启用使用 `map` 的接口，但系统中不存在 `map → world` TF，RViz 会提示变换失败并无法显示 Marker。

更稳妥的设计是：

- 统一规划坐标系；
- 或将 frame ID 作为构造参数；
- 或从 ROS 参数读取固定坐标系。

---

## 30. 当前源码中的重要风险

## 30.1 PolynomialTraj 构造后成员未初始化

默认构造函数为空。

在调用 `init()` 前读取：

```text
time_sum
num_seg
length
```

可能得到未定义值。

## 30.2 静态生成器不自动 init()

两个轨迹生成函数只添加分段，不计算总时长。

调用者若忘记 `init()`，`getTimeSum()` 和 `getTraj()` 等接口会失效。

## 30.3 evaluate 系列没有时间边界检查

空轨迹或超出总时长的查询可能导致数组越界。

负时间会直接对第一段进行外推，也没有告警。

## 30.4 addSegment 缺少输入合法性检查

没有检查：

- 三轴系数长度一致；
- 系数数量足以计算速度和加速度；
- 时间为正；
- 系数中是否存在 NaN 或 Inf。

## 30.5 getLength() 依赖隐式调用顺序

必须先调用 `getTraj()`，否则可能越界。

这种隐式状态依赖容易被误用。

## 30.6 getMeanVel() 没有返回值

函数声明返回 `double`，但实现没有 `return`，属于明确缺陷。

## 30.7 速度与加速度统计采样错误

`getMeanAndMaxVel()` 和 `getMeanAndMaxAcc()` 使用分段总时间 `ts` 构造时间幂，而不是循环变量 `eval_t`。

统计结果本质上是重复的段末值。

## 30.8 getAccCost() 不是完整加速度积分

它只根据每段起始加速度估算，无法表示一般五次轨迹的加速度代价。

## 30.9 minSnapTraj 名称不符合实际数学目标

当前目标是最小化 jerk，而不是 snap。

这会影响算法理解、论文对照和实验表述。

## 30.10 minSnapTraj 单段边界逻辑不完整

代码使用：

```cpp
if (k == 0) {
  设置起点速度和加速度
} else if (k == seg_num - 1) {
  设置终点速度和加速度
}
```

当 `seg_num == 1` 时，只会执行第一个分支，终点速度与加速度不会被正确写入。

当前主流程对单段轨迹使用 `one_segment_traj_gen()`，因此通常避开了这个问题；单独调用 `minSnapTraj()` 时必须注意。

## 30.11 显式矩阵求逆影响稳定性

`minSnapTraj()` 多次使用：

```cpp
A.inverse()
Rpp.inverse()
```

显式求逆通常比线性方程求解更慢、数值稳定性更差。

更稳妥的方式是使用 Eigen 分解器求解，例如：

```text
LDLT
LLT
QR
```

## 30.12 minSnapTraj 缺少输入维度检查

没有检查：

```text
Pos 是否为 3×(M+1)
Time 是否有 M 个元素
所有时间是否大于 0
航点和时间是否包含 NaN
```

输入错误可能触发 Eigen 断言或产生不可用轨迹。

## 30.13 可视化坐标系不统一

`world` 与 `map` 混用，启用未使用接口时可能出现 TF 问题。

## 30.14 A* 可视化会产生日志噪声

每次显示 A* 路径都会打印个人调试字符串。

规划频繁时会污染终端输出，掩盖真正的重要警告。

## 30.15 A* 路径尺度随机

每次调用使用随机 Marker 尺度，会让同类路径在视觉上不一致，也降低截图和实验复现性。

## 30.16 旧 Marker 可能残留

如果新一次发布的 A* 路径数量少于上一次，旧的较大 ID Marker 不会被删除。

空 `MarkerArray` 也不能自动清空旧 Marker。

## 30.17 无订阅者时首次显示会丢失

全局路径、初始路径、优化点和 A* 路径在没有订阅者时直接返回。

发布器不是 latched publisher，因此如果 RViz 在规划完成后才连接，而规划器没有再次发布，该次结果不会显示。

## 30.18 颜色透明度处理不一致

路径显示中，如果传入 alpha 接近 0，会强制改为 1。

目标点显示则直接使用调用者传入的 alpha。相同颜色参数可能在不同接口中产生不同效果。

## 30.19 依赖声明过重且不完整

一方面依赖了未使用的 PCL、OpenCV、搜索和 B 样条库；另一方面又没有在 `package.xml` 中完整声明实际使用的可视化消息依赖。

---

## 31. 推荐源码阅读顺序

### 第一遍：快速建立整体认识

```text
1. CMakeLists.txt
2. include/traj_utils/polynomial_traj.h
3. include/traj_utils/planning_visualization.h
```

目标：知道包构建什么、提供哪些类和接口。

### 第二遍：理解轨迹数学

```text
1. one_segment_traj_gen()
2. evaluate()
3. evaluateVel()
4. evaluateAcc()
5. minSnapTraj()
```

目标：先理解单段五次多项式，再理解多段闭式优化。

### 第三遍：结合主流程理解用途

```text
1. plan_manage/src/planner_manager.cpp
2. plan_manage/include/plan_manage/plan_container.hpp
3. plan_manage/src/ego_replan_fsm.cpp
4. planning_visualization.cpp
```

目标：理解多项式轨迹为什么只是全局参考和 B 样条初始化，而不是最终局部避障轨迹。

---

## 32. 一个最小使用示例

下面展示单段轨迹的典型使用方式：

```cpp
#include <traj_utils/polynomial_traj.h>

Eigen::Vector3d p0(0.0, 0.0, 1.0);
Eigen::Vector3d v0 = Eigen::Vector3d::Zero();
Eigen::Vector3d a0 = Eigen::Vector3d::Zero();

Eigen::Vector3d p1(5.0, 2.0, 1.5);
Eigen::Vector3d v1 = Eigen::Vector3d::Zero();
Eigen::Vector3d a1 = Eigen::Vector3d::Zero();

double duration = 4.0;

PolynomialTraj traj =
    PolynomialTraj::one_segment_traj_gen(
        p0, v0, a0,
        p1, v1, a1,
        duration);

traj.init();

for (double t = 0.0; t <= duration; t += 0.1) {
  Eigen::Vector3d p = traj.evaluate(t);
  Eigen::Vector3d v = traj.evaluateVel(t);
  Eigen::Vector3d a = traj.evaluateAcc(t);
}
```

实际工程中建议先将查询时间限制到合法范围：

```cpp
t = std::max(0.0, std::min(t, traj.getTimeSum()));
```

---

## 33. 多航点轨迹使用示意

```cpp
Eigen::MatrixXd pos(3, 4);
pos.col(0) = Eigen::Vector3d(0, 0, 1);
pos.col(1) = Eigen::Vector3d(2, 1, 1);
pos.col(2) = Eigen::Vector3d(4, -1, 1.5);
pos.col(3) = Eigen::Vector3d(6, 0, 1);

Eigen::VectorXd time(3);
time << 2.0, 2.5, 2.0;

PolynomialTraj traj = PolynomialTraj::minSnapTraj(
    pos,
    Eigen::Vector3d::Zero(),
    Eigen::Vector3d::Zero(),
    Eigen::Vector3d::Zero(),
    Eigen::Vector3d::Zero(),
    time);

traj.init();
```

注意：

- 当前函数实际最小化 jerk；
- 至少应提供两段，避免单段边界分支问题；
- 所有分段时间必须为正；
- 该轨迹不检查障碍物。

---

## 34. 调试与验证建议

## 34.1 验证边界条件

对单段轨迹检查：

```text
evaluate(0)              ≈ start_pt
evaluateVel(0)           ≈ start_vel
evaluateAcc(0)           ≈ start_acc
evaluate(T)              ≈ end_pt
evaluateVel(T)           ≈ end_vel
evaluateAcc(T)           ≈ end_acc
```

## 34.2 验证多段连续性

对每个内部边界 `t_k` 检查左右极限：

```text
p(t_k - ε) ≈ p(t_k + ε)
v(t_k - ε) ≈ v(t_k + ε)
a(t_k - ε) ≈ a(t_k + ε)
```

当前设计应保证位置、速度和加速度连续。

## 34.3 验证时间分配影响

使用相同航点，改变分段时间：

- 时间变长，速度和加速度通常下降；
- 时间过短，系数幅值增大，轨迹更激进；
- 极短时间可能导致矩阵病态和数值问题。

## 34.4 验证可视化坐标系

```bash
rosrun tf tf_echo world map
```

若启用 `map` 坐标系 Marker，应确认它可以转换到 RViz Fixed Frame。

## 34.5 检查动态库

```bash
ldd ~/Desktop/ego-planner_ws/devel/lib/libtraj_utils.so
```

可观察当前库实际链接的 ROS 和系统依赖。

---

## 35. 建议的单元测试

当前包没有测试。若准备长期使用，建议至少加入以下测试：

1. 单段轨迹六个边界条件测试；
2. 多段轨迹航点通过测试；
3. 多段轨迹位置、速度、加速度连续性测试；
4. `evaluate()` 在起点、终点和分段边界测试；
5. 空轨迹与越界时间行为测试；
6. 非法持续时间测试；
7. jerk 代价与数值积分对比测试；
8. 轨迹总时长测试；
9. 可视化 Marker 类型、ID、颜色和 frame ID 测试；
10. 旧 Marker 清理行为测试。

---

## 36. 二次开发建议

## 36.1 提升 PolynomialTraj 接口安全性

建议：

- 构造时初始化全部成员；
- `addSegment()` 校验输入；
- 自动维护总时长；
- 静态生成器返回前自动 `init()`；
- 对查询时间进行 clamp 或显式返回失败；
- 为空轨迹和非法时间抛出异常或返回状态；
- 增加 `const` 成员函数；
- 避免返回不必要的数据副本。

## 36.2 修正数学命名

可选择：

- 将 `minSnapTraj()` 重命名为 `minJerkTraj()`；
- 或真正改为最小 snap 的七次多项式实现；
- 并为轨迹阶次、连续阶次和目标函数写清楚文档。

## 36.3 改善数值稳定性

建议用矩阵分解求解替代显式求逆，并检查矩阵条件数。

对于非常短或差异极大的分段时间，可使用归一化时间降低病态程度。

## 36.4 修复统计接口

建议：

- `getMeanVel()` 正确返回值；
- 使用 `eval_t` 进行速度和加速度采样；
- 用解析积分或可靠数值积分计算加速度代价；
- 让 `getLength()` 自己完成采样或直接接受分辨率；
- 明确统计结果的采样步长。

## 36.5 简化依赖

可以让多项式数学工具尽量只依赖 Eigen。

可视化模块则依赖 ROS 和 `visualization_msgs`。

更彻底的做法是拆成：

```text
trajectory_math
trajectory_visualization
```

从而让纯算法代码可在非 ROS 环境中测试和复用。

## 36.6 改善可视化

建议：

- 统一或参数化 frame ID；
- 使用 latched publisher 或按需周期重发关键结果；
- 删除个人调试打印；
- 移除随机尺度；
- 使用明确的 Marker namespace；
- 发布新列表前删除多余旧 Marker；
- 让颜色和尺度成为配置参数。

---

## 37. 与 B 样条的区别

EGO-Planner 同时使用多项式轨迹和 B 样条，它们承担不同角色。

| 特性 | PolynomialTraj | UniformBspline |
|---|---|---|
| 主要用途 | 全局参考、初始轨迹、边界连接 | 最终局部优化和执行轨迹 |
| 参数 | 每段多项式系数 | 控制点和节点向量 |
| 局部修改性 | 修改一段可能影响边界衔接 | 控制点具有局部支撑特性 |
| 障碍优化 | 当前类不支持 | `bspline_opt` 负责 |
| 边界约束 | 易直接指定位置、速度、加速度 | 需要通过控制点参数化满足 |
| 主流程地位 | 引导和初始化 | 局部安全轨迹主体 |

简单理解：

```text
PolynomialTraj 负责先画出一条平滑的参考草图
UniformBspline 负责把局部轨迹优化成可执行结果
```

---

## 38. 核心流程伪代码

### 多项式轨迹

```cpp
if (only_start_and_end) {
  traj = solve_quintic_from_boundary_states();
} else {
  fixed = waypoint_positions
        + start_velocity_and_acceleration
        + end_velocity_and_acceleration;

  free = internal_waypoint_velocities_and_accelerations;

  minimize_integral_squared_jerk(fixed, free);
  recover_each_segment_coefficients();
}

traj.init();

position = traj.evaluate(t);
velocity = traj.evaluateVel(t);
acceleration = traj.evaluateAcc(t);
```

### 规划可视化

```cpp
receive_eigen_points_from_planner();

marker_points = convert_to_geometry_msgs_points();

publish_sphere_list();
publish_line_strip();

rviz_displays_markers_in_world_frame();
```

---

## 39. 最终总结

理解 `traj_utils` 时，需要抓住以下关键点：

1. 它是共享库，不是可独立运行的 ROS 节点。
2. 它由 `PolynomialTraj` 和 `PlanningVisualization` 两个核心模块组成。
3. `PolynomialTraj` 使用分段多项式分别描述 x、y、z 三个轴。
4. 当前静态生成器产生的是五次多项式轨迹。
5. 单段生成器用起终点位置、速度和加速度唯一求解五次多项式。
6. 多段 `minSnapTraj()` 保证内部航点的位置、速度和加速度连续。
7. 当前 `minSnapTraj()` 实际最小化的是 jerk 平方积分，而不是 snap。
8. 多项式轨迹在 EGO-Planner 中主要用于全局参考和局部 B 样条初始化，不直接负责避障。
9. `PlanningVisualization` 把规划中间状态发布为 RViz Marker。
10. 默认可视化中，青绿色表示全局参考，蓝色表示初始路径，红色表示优化控制点，粉色表示 A* 路径。
11. 当前统计接口、时间边界、成员初始化和多段求解均存在值得修复的工程风险。
12. 当前包依赖偏重且声明不完整，适合在后续重构中拆分数学与 ROS 可视化职责。

`traj_utils` 的代码量不大，却连接了轨迹数学、规划初始化和调试可视化。读懂它以后，`plan_manage` 中全局参考轨迹的生成方式，以及局部 B 样条优化之前的数据来源，会变得清晰许多。
