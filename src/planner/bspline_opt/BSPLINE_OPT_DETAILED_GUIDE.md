# `bspline_opt` 功能包详细学习说明

> 适用源码：`/home/yxc/Desktop/ego-planner_ws/src/planner/bspline_opt`
>
> 功能包名称：`bspline_opt`
>
> 所属系统：EGO-Planner

---

## 1. 一句话认识这个包

`bspline_opt` 是 EGO-Planner 的局部轨迹表示与优化核心：

- 使用均匀 B 样条表示无人机轨迹；
- 将初始轨迹转换为一组可优化的控制点；
- 根据占据栅格和 A* 绕障路径构造“反弹”方向；
- 使用 L-BFGS 同时降低平滑、碰撞和动力学不可行代价；
- 必要时重新分配轨迹时间，并执行第二阶段精修；
- 提供轨迹求值、求导、可行性检查和性能统计工具。

它不是一个独立 ROS 节点，不直接订阅或发布话题，而是被 `plan_manage` 中的
`EGOPlannerManager` 作为 C++ 库调用。

可以把它理解为：

```text
初始离散路径 / 初始控制点
          |
          v
发现碰撞段 + A* 绕障
          |
          v
构造控制点反弹约束
          |
          v
L-BFGS 优化控制点
          |
          v
检查速度与加速度
          |
          v
必要时延长时间并精修
          |
          v
可执行的三次均匀 B 样条轨迹
```

---

## 2. 它在 EGO-Planner 中的位置

EGO-Planner 的局部重规划主链路可以概括为：

```text
里程计 + 局部目标 + 局部占据地图
                |
                v
        EGOPlannerManager
                |
                | 生成初始点集
                v
UniformBspline::parameterizeToBspline()
                |
                | 得到初始控制点
                v
BsplineOptimizer::initControlPoints()
                |
                | 识别碰撞段并运行 A*
                v
BsplineOptimizer::BsplineOptimizeTrajRebound()
                |
                | 平滑 + 避障 + 动力学约束
                v
UniformBspline::checkFeasibility()
                |
      不满足速度/加速度限制？
           /            \
         否               是
         |                |
         |                v
         |       重新分配轨迹时间
         |                |
         |                v
         |  BsplineOptimizeTrajRefine()
         |                |
         +----------------+
                |
                v
       保存并发布局部轨迹
```

上游主要来自：

- `plan_manage`：组织完整规划流程；
- `plan_env`：提供 `GridMap` 占据栅格查询；
- `path_searching`：提供 A* 搜索；
- `traj_utils`：负责轨迹消息和可视化。

下游主要是：

- `plan_manage/src/traj_server.cpp`：接收并执行 B 样条轨迹；
- 控制器：消费轨迹服务器产生的位置、速度和加速度指令。

---

## 3. 目录与文件职责

```text
bspline_opt/
├── CMakeLists.txt
├── package.xml
├── include/bspline_opt/
│   ├── bspline_optimizer.h
│   ├── gradient_descent_optimizer.h
│   ├── lbfgs.hpp
│   └── uniform_bspline.h
└── src/
    ├── bspline_optimizer.cpp
    ├── gradient_descent_optimizer.cpp
    └── uniform_bspline.cpp
```

### 3.1 `uniform_bspline.h/.cpp`

定义 `ego_planner::UniformBspline`，负责：

- 保存控制点、阶数、节点向量和时间间隔；
- 使用 de Boor 算法求轨迹位置；
- 构造速度、加速度、jerk 对应的导数 B 样条；
- 将离散路径点参数化为三次 B 样条控制点；
- 检查速度和加速度是否满足限制；
- 通过调整节点向量延长轨迹时间；
- 统计轨迹时长、长度、jerk、平均/最大速度和加速度。

它解决的是“如何表示、求值和检查一条 B 样条轨迹”。

### 3.2 `bspline_optimizer.h/.cpp`

定义 `ControlPoints` 和 `BsplineOptimizer`，负责：

- 保存控制点及每个控制点的避障几何约束；
- 查询地图并识别碰撞轨迹段；
- 对碰撞段运行 A*；
- 从 A* 路径构造反弹方向；
- 计算平滑、避障、动力学可行性和拟合代价；
- 调用 L-BFGS 优化控制点；
- 在优化中新发现碰撞时重新构造约束并重启；
- 在时间重分配后执行轨迹精修。

它解决的是“如何把一条初始轨迹优化成安全、平滑且可执行的轨迹”。

### 3.3 `lbfgs.hpp`

头文件形式的 L-BFGS 求解器实现，包含：

- L-BFGS 参数；
- 目标函数和迭代回调接口；
- More-Thuente 线搜索；
- 有限历史拟牛顿方向计算；
- 返回码和错误文本。

当前真正执行反弹优化和精修优化的求解器就是它。

### 3.4 `gradient_descent_optimizer.h/.cpp`

实现一个带 Barzilai-Borwein 步长和 Armijo 回溯的梯度下降器。

但当前代码没有实际调用它，头文件中也直接写有：

```cpp
// 好像暂时没有使用
```

因此阅读优先级低于 L-BFGS。

### 3.5 `CMakeLists.txt`

该包只生成一个库：

```cmake
add_library(bspline_opt
  src/uniform_bspline.cpp
  src/bspline_optimizer.cpp
  src/gradient_descent_optimizer.cpp
)
```

没有 `add_executable()`，因此没有独立节点。

---

## 4. 构建产物与依赖

### 4.1 构建产物

```text
libbspline_opt.so
```

其他包通过：

```cpp
#include <bspline_opt/bspline_optimizer.h>
```

以及链接 `bspline_opt` 库来使用它。

### 4.2 关键依赖

| 依赖               | 用途                                                              |
| ------------------ | ----------------------------------------------------------------- |
| `Eigen3`         | 矩阵、向量、QR 求解和几何运算                                     |
| `roscpp`         | 参数读取、时间统计和日志                                          |
| `plan_env`       | `GridMap` 与占据查询                                            |
| `path_searching` | `AStar` 路径搜索                                                |
| `PCL`            | CMake 中声明，但本包核心源码未直接使用                            |
| `cv_bridge`      | CMake 中声明，但 `package.xml` 未完整声明，核心源码也未直接使用 |

### 4.3 编译设置注意事项

`CMakeLists.txt` 同时写了：

```cmake
ADD_COMPILE_OPTIONS(-std=c++11)
ADD_COMPILE_OPTIONS(-std=c++14)
```

后出现的 `-std=c++14` 通常会覆盖前者，但这种写法不够清晰，建议未来只保留一个标准。

---

## 5. 数据约定：控制点到底如何存储

理解矩阵方向非常重要，因为两个类的注释看起来并不完全一致。

本项目实际使用的三维控制点矩阵是：

```text
3 x N
```

即：

- 每一列是一个三维控制点；
- 第 0 行是全部 x；
- 第 1 行是全部 y；
- 第 2 行是全部 z。

示意：

```text
           P0    P1    P2         PN-1
ctrl_pts = [x0    x1    x2   ...   xN-1
            y0    y1    y2   ...   yN-1
            z0    z1    z2   ...   zN-1]
```

常见访问方式：

```cpp
ctrl_pts.col(i)
```

`UniformBspline` 可以表示任意维度，因此它以“行数为维度、列数为控制点数”的方式工作。

---

## 6. B 样条基础

### 6.1 为什么使用 B 样条

相比直接优化大量时间采样点，B 样条有几个关键优势：

1. **局部支撑性**：移动一个控制点只影响局部轨迹。
2. **高阶连续性**：三次 B 样条天然具有良好的平滑性。
3. **导数仍是 B 样条**：速度、加速度可以高效求得。
4. **凸包性质**：可利用控制点差分对速度和加速度进行保守约束。
5. **变量数量较少**：适合在线局部重规划。

### 6.2 阶数与次数

源码参数名为 `order`，实际传入值通常是：

```text
order = 3
```

在这份实现里，它对应三次 B 样条的次数 `p = 3`。

若有 `N` 个控制点：

```text
n = N - 1
m = n + p + 1
节点数量 = m + 1
```

### 6.3 均匀节点向量

`setUniformBspline()` 以固定间隔 `interval_` 构造节点向量：

```cpp
u_(i) = double(-p_ + i) * interval_;
```

有效轨迹时间范围由：

```cpp
u_(p_) 到 u_(m_ - p_)
```

确定。对于默认构造，它通常对应：

```text
t ∈ [0, duration]
```

### 6.4 de Boor 求值

`evaluateDeBoor(u)` 的流程：

1. 将输入参数限制在有效节点范围；
2. 找到参数所在节点区间；
3. 取局部相关的 `p + 1` 个控制点；
4. 递归线性插值；
5. 返回曲线点。

使用相对轨迹时间时调用：

```cpp
traj.evaluateDeBoorT(t);
```

它内部执行：

```cpp
evaluateDeBoor(t + u_(p_));
```

---

## 7. `UniformBspline` 详细说明

### 7.1 构造轨迹

```cpp
UniformBspline traj(control_points, 3, ts);
```

参数含义：

| 参数               | 含义                                              |
| ------------------ | ------------------------------------------------- |
| `control_points` | `维度 x 控制点数`矩阵，位置轨迹通常是 `3 x N` |
| `3`              | 三次 B 样条                                       |
| `ts`             | 均匀节点时间间隔                                  |

### 7.2 求导

B 样条的导数仍然是 B 样条。导数控制点为：

```text
Q_i = p(P_{i+1} - P_i) / (u_{i+p+1} - u_{i+1})
```

源码调用：

```cpp
UniformBspline vel = pos.getDerivative();
UniformBspline acc = vel.getDerivative();
UniformBspline jerk = acc.getDerivative();
```

规划管理器正是这样保存位置、速度和加速度轨迹。

### 7.3 离散点参数化为控制点

入口：

```cpp
UniformBspline::parameterizeToBspline(
    ts,
    point_set,
    start_end_derivative,
    ctrl_pts);
```

输入包括：

- 一组希望轨迹拟合/经过的离散位置点；
- 起点速度；
- 终点速度；
- 起点加速度；
- 终点加速度；
- 时间间隔 `ts`。

`start_end_derivative` 的顺序是：

```text
[start_vel, end_vel, start_acc, end_acc]
```

若位置点数为 `K`：

```text
约束数量 = K + 4
控制点数量 = K + 2
```

位置关系使用三次均匀 B 样条模板：

```text
P(t_i) = (Q_i + 4Q_{i+1} + Q_{i+2}) / 6
```

边界速度模板：

```text
[-1, 0, 1] / (2 ts)
```

边界加速度模板：

```text
[1, -2, 1] / ts²
```

最终分别对 x、y、z 使用：

```cpp
A.colPivHouseholderQr().solve(b)
```

求得控制点。

这里的方程通常是超定约束，因此 QR 求解得到最小二乘意义下的结果。

### 7.4 动力学可行性检查

调用：

```cpp
traj.setPhysicalLimits(max_vel, max_acc, tolerance);
bool feasible = traj.checkFeasibility(ratio, false);
```

检查依据是导数 B 样条控制点。

速度控制点：

```text
V_i = p(P_{i+1} - P_i) / Δu
```

加速度控制点由二阶差分得到。

若这些导数控制点各轴分量均在限制内，根据 B 样条凸包性质，整段导数曲线也在对应范围内。

注意：这里检查的是每个轴的绝对值，而不是速度向量的欧氏范数。

`ratio` 用于估计需要延长多少时间：

```text
ratio = max(max_vel_actual / vel_limit,
            sqrt(max_acc_actual / acc_limit))
```

原因是：

- 速度与时间尺度成反比；
- 加速度与时间尺度平方成反比。

### 7.5 延长轨迹时间

```cpp
traj.lengthenTime(ratio);
```

该函数主要拉伸内部节点区间，降低速度和加速度。

规划管理器随后会重新采样被拉伸的轨迹，再调用
`parameterizeToBspline()` 生成新的均匀 B 样条控制点。

### 7.6 性能统计接口

| 接口                   | 作用                        |
| ---------------------- | --------------------------- |
| `getTimeSum()`       | 获取轨迹总时长              |
| `getLength(res)`     | 通过时间采样估计轨迹长度    |
| `getJerk()`          | 计算积分形式的 jerk 代价    |
| `getMeanAndMaxVel()` | 采样统计平均/最大速度模长   |
| `getMeanAndMaxAcc()` | 采样统计平均/最大加速度模长 |

---

## 8. `ControlPoints`：反弹优化的数据核心

`ControlPoints` 保存：

```cpp
double clearance;
int size;
Eigen::MatrixXd points;
vector<vector<Eigen::Vector3d>> base_point;
vector<vector<Eigen::Vector3d>> direction;
vector<bool> flag_temp;
```

### 8.1 字段含义

| 字段              | 含义                                            |
| ----------------- | ----------------------------------------------- |
| `points`        | 当前控制点，形状为 `3 x N`                    |
| `clearance`     | 期望安全距离，来自 `optimization/dist0`       |
| `base_point[i]` | 第 `i` 个控制点对应的障碍边界基准点，可有多个 |
| `direction[i]`  | 从障碍侧指向安全侧的单位方向，可有多个          |
| `flag_temp[i]`  | 构造约束时使用的临时标记                        |

### 8.2 一个控制点的避障约束

对控制点 `Q_i`，设：

- 基准点为 `B_ij`；
- 单位反弹方向为 `D_ij`。

则沿反弹方向的有符号距离为：

```text
d_ij = (Q_i - B_ij) · D_ij
```

安全要求近似为：

```text
d_ij >= clearance
```

它不是精确欧氏距离场，而是根据 A* 绕障路径构造的局部方向性约束。

---

## 9. 为什么叫“反弹优化”

传统基于 ESDF 的轨迹优化通常直接使用：

```text
障碍物距离 + 距离场梯度
```

EGO-Planner 的关键思路是避免维护昂贵的全局 ESDF。

本实现采用：

1. 在占据栅格中找出轨迹穿过障碍物的区段；
2. 对每个碰撞区段，在自由空间中运行 A*；
3. 将 A* 绕障路径与原控制点轨迹做几何关联；
4. 为相关控制点构造“从障碍物向外推”的方向；
5. 优化器沿这些方向推动控制点；
6. 优化过程中发现新碰撞时，重新构造方向并继续。

所以“反弹”指的是控制点被人工构造的碰撞梯度从障碍区域推回自由空间。

---

## 10. 初始控制点与碰撞约束构造

入口：

```cpp
initControlPoints(ctrl_pts, true);
```

该函数是理解 EGO-Planner 无 ESDF 优化思想的重点。

### 10.1 第一步：初始化控制点容器

首次调用时：

```cpp
cps_.clearance = dist0_;
cps_.resize(init_points.cols());
cps_.points = init_points;
```

### 10.2 第二步：检测碰撞区段

函数在相邻控制点连线上密集采样，并调用：

```cpp
grid_map_->getInflateOccupancy(point)
```

判断采样点是否落入膨胀障碍物。

采样步长根据：

- 地图分辨率；
- 控制点平均间距；

动态计算，目的是避免跨过细小障碍物。

最终得到若干：

```text
[in_id, out_id]
```

表示轨迹进入和离开障碍区域附近的控制点索引。

只检测靠近无人机的前约三分之二局部轨迹，因为远端之后还会继续重规划。

### 10.3 第三步：为每个碰撞段运行 A*

对每个碰撞段：

```cpp
a_star_->AstarSearch(0.1, in, out);
```

得到一条局部自由空间绕障路径。

A* 在这里不是最终轨迹生成器，而是用于告诉连续优化器：

```text
应该从障碍物的哪一侧绕过去
```

### 10.4 第四步：寻找几何交点

对碰撞段内的控制点 `Q_j`：

```cpp
ctrl_pts_law = Q_{j+1} - Q_{j-1}
```

该向量近似表示控制点轨迹的局部切向。

随后在 A* 路径上寻找与经过 `Q_j`、法向为 `ctrl_pts_law` 的平面相交的位置。

得到交点后，从控制点朝交点方向采样，找到障碍物边界附近的基准点：

```text
base_point[j]
```

并保存单位方向：

```text
direction[j] = normalize(intersection_point - Q_j)
```

### 10.5 第五步：向邻近控制点传播约束

不是每个控制点都一定能直接找到稳定交点。

因此源码从已成功生成方向的控制点向碰撞段两侧传播：

```text
相邻控制点复用最近的 base_point 和 direction
```

这样整段控制点都能受到连续的避障推动。

---

## 11. 优化变量与固定边界

反弹和精修阶段都只优化内部控制点。

```cpp
start_id = order_;
end_id = cps_.size - order_;
variable_num_ = 3 * (end_id - start_id);
```

对于三次 B 样条：

- 前 3 个控制点固定；
- 后 3 个控制点固定；
- 只优化中间控制点的 x、y、z。

这是为了保持由初始参数化建立的起止边界条件。

Eigen 默认采用列主序，因此：

```cpp
memcpy(q, cps_.points.data() + 3 * start_id, ...)
```

能将连续的内部控制点坐标直接复制为 L-BFGS 的一维变量数组。

---

## 12. 两阶段优化总体结构

### 12.1 阶段一：反弹优化

入口：

```cpp
BsplineOptimizeTrajRebound(ctrl_pts, ts);
```

总代价：

```text
J_rebound =
    λ_smooth     J_smooth
  + λ_collision  J_collision
  + λ_feasibility J_feasibility
```

目标：

- 从障碍物中反弹出来；
- 保持轨迹平滑；
- 尽量满足速度和加速度限制。

### 12.2 阶段二：时间重分配后的精修

只有阶段一结果不满足严格动力学限制时才执行。

入口：

```cpp
BsplineOptimizeTrajRefine(ctrl_pts, ts, optimal_points);
```

总代价：

```text
J_refine =
    λ_smooth      J_smooth
  + λ_fitness     J_fitness
  + λ_feasibility J_feasibility
```

这里不再直接使用碰撞反弹代价，而是让新轨迹贴合时间拉伸前后的参考轨迹，避免重新参数化造成形状明显漂移。

---

## 13. 平滑代价 `J_smooth`

默认使用控制点三阶差分，也就是离散 jerk：

```text
j_i = Q_{i+3} - 3Q_{i+2} + 3Q_{i+1} - Q_i
```

总代价：

```text
J_smooth = Σ ||j_i||²
```

对应梯度会分配到四个相邻控制点：

```text
∂J/∂Q_i     += -2j_i
∂J/∂Q_{i+1} +=  6j_i
∂J/∂Q_{i+2} += -6j_i
∂J/∂Q_{i+3} +=  2j_i
```

源码也支持使用二阶差分加速度作为平滑项，但当前调用使用默认参数：

```cpp
falg_use_jerk = true
```

增大 `lambda_smooth` 的效果：

- 轨迹更平滑；
- 控制点不愿剧烈弯折；
- 狭窄环境中可能降低绕障能力；
- 可能与碰撞代价竞争。

---

## 14. 碰撞反弹代价 `J_collision`

对每个控制点及其每条反弹约束：

```text
d = (Q - B) · D
e = clearance - d
```

其中：

- `B` 是障碍边界附近的基准点；
- `D` 是反弹单位方向；
- `clearance` 是期望安全距离；
- `e > 0` 表示距离不足。

当 `e < 0` 时不产生代价。

当距离不足但偏差较小时：

```text
J = e³
```

当偏差较大时，改用与三次函数连续衔接的二次形式：

```text
J = a e² + b e + c
```

这样做的目的：

- 靠近安全边界时保持高阶平滑；
- 偏差很大时避免三次代价和梯度爆炸；
- 给 L-BFGS 更稳定的优化曲面。

梯度方向本质上沿 `-D`，优化器执行下降时会把控制点沿 `+D` 推离障碍物。

### 动态碰撞约束更新

迭代超过 3 次且轨迹足够平滑时，代码调用：

```cpp
check_collision_and_rebound();
```

若发现旧约束未覆盖的新碰撞：

1. 找到新的碰撞段；
2. 再运行 A*；
3. 添加新的 `base_point/direction`；
4. 设置 `STOP_FOR_REBOUND`；
5. 通过 `earlyExit()` 中止当前 L-BFGS；
6. 使用新约束重新启动优化。

这正是“优化、检查、再反弹”的闭环。

---

## 15. 动力学可行性代价 `J_feasibility`

源码默认使用分段二次惩罚。

### 15.1 速度近似

```text
v_i = (Q_{i+1} - Q_i) / ts
```

若某一轴：

```text
v_i > max_vel
```

则加入：

```text
(v_i - max_vel)² / ts²
```

负方向超限同理。

### 15.2 加速度近似

```text
a_i = (Q_{i+2} - 2Q_{i+1} + Q_i) / ts²
```

若某轴超过 `max_acc`，则惩罚超限部分平方。

### 15.3 为什么是软约束

L-BFGS 解决的是无约束优化，因此速度和加速度限制通过代价项近似表达。

这意味着反弹优化完成后仍可能略微超限，所以规划管理器还会调用：

```cpp
UniformBspline::checkFeasibility()
```

做严格检查，并在必要时延长时间。

### 15.4 一个容易混淆的点

`calcFeasibilityCost()` 中的速度/加速度差分形式用于优化代价；
`UniformBspline::checkFeasibility()` 使用严格的导数控制点公式做最终检查。

两者目的不同：

- 前者提供易优化的软惩罚；
- 后者进行最终可行性判定。

---

## 16. 轨迹拟合代价 `J_fitness`

精修阶段需要让重新参数化后的控制点轨迹贴近参考轨迹。

三次均匀 B 样条在节点附近的位置近似：

```text
X_i = (Q_{i-1} + 4Q_i + Q_{i+1}) / 6
```

与参考点的误差：

```text
x = X_i - ref_i
```

参考轨迹切向：

```text
v = normalize(ref_{i+1} - ref_{i-1})
```

代价将误差分为切向和法向：

```text
J_fitness =
    |x · v|² / a²
  + |x × v|² / b²
```

源码取：

```text
a² = 25
b² = 1
```

因此：

- 切向偏差惩罚较弱；
- 法向偏差惩罚较强。

这允许轨迹沿自身方向小幅滑动，但不希望明显偏离原轨迹形状。

---

## 17. L-BFGS 如何接入

### 17.1 反弹优化参数

```cpp
lbfgs_params.mem_size = 16;
lbfgs_params.max_iterations = 200;
lbfgs_params.g_epsilon = 0.01;
```

### 17.2 精修优化参数

```cpp
lbfgs_params.mem_size = 16;
lbfgs_params.max_iterations = 200;
lbfgs_params.g_epsilon = 0.001;
```

精修阶段的梯度收敛阈值更严格。

### 17.3 目标函数桥接

L-BFGS 使用 C 风格回调：

```cpp
static double costFunctionRebound(
    void *func_data,
    const double *x,
    double *grad,
    const int n);
```

`func_data` 被转换回：

```cpp
BsplineOptimizer *opt
```

随后调用：

```cpp
opt->combineCostRebound(...)
```

### 17.4 反弹阶段提前退出

`earlyExit()` 检查：

```cpp
force_stop_type_
```

可能状态：

| 状态                 | 含义                           |
| -------------------- | ------------------------------ |
| `DONT_STOP`        | 正常继续                       |
| `STOP_FOR_REBOUND` | 发现新碰撞，需要更新约束后重启 |
| `STOP_FOR_ERROR`   | 发生无法继续的地图/终点问题    |

### 17.5 优化后的安全复查

即使求解器正常结束，反弹阶段还会对轨迹前约三分之二进行采样碰撞检测。

如果仍碰撞：

- 重新调用 `initControlPoints()`；
- 将碰撞权重 `new_lambda2_` 乘 2；
- 最多执行有限次数的重启。

这比仅信任连续代价函数更稳妥。

---

## 18. 完整运行调用链

真实调用发生在：

```text
planner/plan_manage/src/planner_manager.cpp
```

### 18.1 初始化阶段

```cpp
bspline_optimizer_rebound_.reset(new BsplineOptimizer);
bspline_optimizer_rebound_->setParam(nh);
bspline_optimizer_rebound_->setEnvironment(grid_map_);
bspline_optimizer_rebound_->a_star_.reset(new AStar);
bspline_optimizer_rebound_->a_star_->initGridMap(
    grid_map_, Eigen::Vector3i(100, 100, 100));
```

必须注入：

- ROS 参数；
- `GridMap`；
- 初始化完成的 `AStar`。

否则 `initControlPoints()` 无法工作。

### 18.2 生成初始轨迹

`EGOPlannerManager::reboundReplan()` 首先从：

- 起点状态；
- 局部目标；
- 多项式初始轨迹，或上一条局部轨迹；

采样出 `point_set`。

随后：

```cpp
UniformBspline::parameterizeToBspline(
    ts, point_set, start_end_derivatives, ctrl_pts);
```

### 18.3 构造反弹约束

```cpp
a_star_pathes =
    bspline_optimizer_rebound_->initControlPoints(ctrl_pts, true);
```

这里得到的 A* 路径也会送去可视化。

### 18.4 反弹优化

```cpp
bool success =
    bspline_optimizer_rebound_->BsplineOptimizeTrajRebound(ctrl_pts, ts);
```

若失败，本次重规划直接返回失败。

### 18.5 可行性检查和精修

```cpp
UniformBspline pos(ctrl_pts, 3, ts);
pos.setPhysicalLimits(max_vel, max_acc, tolerance);

if (!pos.checkFeasibility(ratio, false))
{
    refineTrajAlgo(...);
}
```

`refineTrajAlgo()` 会：

1. 根据 `ratio` 拉伸时间；
2. 对拉伸后的轨迹重新采样；
3. 重新参数化为均匀 B 样条；
4. 保存参考点 `ref_pts_`；
5. 调用 `BsplineOptimizeTrajRefine()`。

---

## 19. ROS 参数详解

参数定义来自：

```text
planner/plan_manage/launch/advanced_param.xml
```

默认值如下：

| 参数                                |                             默认值 | 作用                       |
| ----------------------------------- | ---------------------------------: | -------------------------- |
| `optimization/lambda_smooth`      |                            `1.0` | 平滑代价权重               |
| `optimization/lambda_collision`   |                            `0.5` | 反弹避障代价权重           |
| `optimization/lambda_feasibility` |                            `0.1` | 速度/加速度软约束权重      |
| `optimization/lambda_fitness`     |                            `1.0` | 精修阶段参考轨迹拟合权重   |
| `optimization/dist0`              |                            `0.5` | 控制点期望障碍净空         |
| `optimization/max_vel`            |                        launch 参数 | 优化器使用的最大轴向速度   |
| `optimization/max_acc`            |                        launch 参数 | 优化器使用的最大轴向加速度 |
| `optimization/order`              | 未在该 launch 显式设置，默认 `3` | B 样条次数                 |

相关管理器参数：

| 参数                                |      默认值 | 关系                      |
| ----------------------------------- | ----------: | ------------------------- |
| `manager/control_points_distance` |     `0.4` | 影响初始采样密度和 `ts` |
| `manager/max_vel`                 | launch 参数 | 最终轨迹速度限制          |
| `manager/max_acc`                 | launch 参数 | 最终轨迹加速度限制        |
| `manager/feasibility_tolerance`   |    `0.05` | 严格可行性检查容差        |

### 19.1 参数调优方向

#### 轨迹容易撞障碍

优先检查：

- 地图是否及时更新；
- 地图膨胀半径是否合理；
- 初始路径是否过差；
- `dist0` 是否太小；
- `lambda_collision` 是否太小。

可以尝试：

```text
增大 lambda_collision
适度增大 dist0
减小 control_points_distance
```

#### 轨迹绕障太保守

可以尝试：

```text
减小 dist0
减小 lambda_collision
检查地图膨胀是否过大
```

但必须给真实无人机保留定位误差、控制误差和机体尺寸裕量。

#### 轨迹弯折明显

可以尝试：

```text
增大 lambda_smooth
减小控制点间距
检查 A* 路径是否出现不必要折线
```

#### 轨迹经常动力学不可行

可以尝试：

```text
增大 lambda_feasibility
降低期望飞行速度
增大控制点时间间隔
允许时间重分配
```

#### 精修后轨迹碰撞

规划器源码已有明确提示：

```text
Increase parameter "lambda_fitness".
```

因为较大的 `lambda_fitness` 会让精修轨迹更贴近时间重分配后的参考轨迹。

### 19.2 不要孤立调整参数

几个参数之间存在明显耦合：

```text
地图分辨率
地图膨胀距离
dist0
control_points_distance
max_vel / max_acc
lambda_collision / lambda_smooth / lambda_feasibility
```

例如只增大 `lambda_collision`，可能导致：

- 控制点快速远离障碍；
- 轨迹曲率变大；
- 动力学不可行；
- 平滑项与碰撞项强烈竞争。

更合理的方式是结合 RViz 中的：

- 初始路径；
- A* 绕障路径；
- 优化后控制点；
- 膨胀障碍地图；

一起观察。

---

## 20. 如何单独使用这个库

最小调用逻辑如下：

```cpp
ros::NodeHandle nh("~");

ego_planner::GridMap::Ptr grid_map(new ego_planner::GridMap);
grid_map->initMap(nh);

ego_planner::BsplineOptimizer::Ptr optimizer(
    new ego_planner::BsplineOptimizer);

optimizer->setParam(nh);
optimizer->setEnvironment(grid_map);

optimizer->a_star_.reset(new ego_planner::AStar);
optimizer->a_star_->initGridMap(
    grid_map, Eigen::Vector3i(100, 100, 100));

Eigen::MatrixXd control_points;
ego_planner::UniformBspline::parameterizeToBspline(
    ts, point_set, boundary_derivatives, control_points);

optimizer->initControlPoints(control_points, true);

bool success =
    optimizer->BsplineOptimizeTrajRebound(control_points, ts);
```

使用前必须保证：

- `point_set.size() > 3`；
- 边界导数数量为 4；
- 地图已初始化且有有效占据信息；
- A* 已创建并绑定同一地图；
- 控制点数量足以固定两端并优化中间点；
- ROS 参数已经设置，避免 `-1.0` 默认值进入优化。

---

## 21. 调试建议

### 21.1 首先看规划器日志

常见输出：

```text
iter(+1)=..., time(ms)=..., total_t(ms)=..., cost=...
```

表示一次 L-BFGS 优化成功完成。

```text
rebound
```

表示优化过程中发现新碰撞，更新了反弹约束并重启。

```text
keep optimizing
```

表示求解器结束后安全复查仍发现碰撞，碰撞权重增大后继续。

```text
First 3 control points in obstacles!
```

表示轨迹起始边界附近已经处于障碍内。由于前 3 个控制点固定，优化器无法修复。

### 21.2 在 RViz 中重点观察

建议同时显示：

- 膨胀占据地图；
- 初始采样路径；
- A* 局部绕障路径；
- 优化后的 B 样条；
- 无人机当前位置和局部目标。

若 A* 绕障正确但优化轨迹仍碰撞，重点检查权重和控制点密度。

若 A* 本身找不到路，重点检查地图、搜索范围和起终点占据状态。

### 21.3 检查参数是否真的加载

`setParam()` 的默认值多数是 `-1.0`。

可以使用：

```bash
rosparam get /drone_0_ego_planner_node/optimization
```

具体节点命名空间需按实际 launch 调整。

若 `max_vel`、`max_acc` 或权重仍是 `-1`，优化结果将没有意义。

### 21.4 验证梯度

修改代价函数时，建议用有限差分验证解析梯度：

```text
g_numeric[i] =
    (f(x + εe_i) - f(x - εe_i)) / (2ε)
```

再与代码返回的 `grad[i]` 比较。

L-BFGS 对错误梯度非常敏感，常见表现包括：

- 线搜索失败；
- 代价不降；
- 控制点突然跳变；
- 大量重启；
- 返回异常错误码。

---

## 22. 源码中的重要注意事项

以下结论基于当前工作空间源码，而不是泛化的 EGO-Planner 说明。

### 22.1 头文件中存在未实现的旧接口

`bspline_optimizer.h` 声明了：

```cpp
BsplineOptimizeTraj(...)
setCostFunction(...)
setTerminateCond(...)
setGuidePath(...)
setWaypoints(...)
optimize()
costFunction(...)
combineCost(...)
```

但当前 `bspline_optimizer.cpp` 没有对应实现，工作空间中也没有实际调用。

这些很可能是旧版优化器遗留接口。不要在新代码中直接调用，否则链接阶段会失败。

当前应使用：

```cpp
initControlPoints()
BsplineOptimizeTrajRebound()
BsplineOptimizeTrajRefine()
```

### 22.2 `gradient_descent_optimizer` 当前未使用

它会被编译进库，但实际规划流程只调用 L-BFGS。

### 22.3 `order_` 可配置，但部分代码硬编码为三次

例如：

```cpp
UniformBspline(cps_.points, 3, bspline_interval_)
```

以及多处三次 B 样条专用公式。

因此虽然存在 `optimization/order` 参数，当前实现实际上应保持：

```text
order = 3
```

不建议直接改成其他值。

### 22.4 使用了变长栈数组

源码中有：

```cpp
double q[variable_num_];
```

变长数组不是标准 C++，属于 GCC 扩展。若迁移到更严格的编译器，建议改为：

```cpp
std::vector<double> q(variable_num_);
```

### 22.5 精修阶段的重试循环实际上不会再次执行

源码条件：

```cpp
do
{
    ...
    iter_count++;
} while (!flag_safe && iter_count <= 0);
```

首次循环结束后 `iter_count == 1`，因此条件必然为假。

虽然代码在不安全时执行：

```cpp
lambda4_ *= 2;
```

但不会进入下一轮优化，所以该权重增长当前没有实际重试效果。

如果设计意图是允许若干次精修重试，应重新检查循环上限。

### 22.6 轨迹安全检查只覆盖前约三分之二

这是局部滚动重规划的设计选择，能降低计算量，但意味着：

- 系统依赖后续持续重规划；
- 若重规划停止或地图更新异常，远端轨迹风险会增大。

### 22.7 `initControlPoints()` 中存在调试输出

当前源码包含：

```cpp
cout << "yxc see using Asearch!" << endl;
```

这不会影响算法，但会污染运行日志，稳定后可考虑移除或改为 ROS debug 日志。

### 22.8 文档注释与实际矩阵方向不完全一致

部分注释提到“每行一个控制点”，但实际三维规划路径普遍使用：

```text
3 x N，每列一个控制点
```

阅读和扩展时应以调用代码及 `col(i)` 操作为准。

### 22.9 默认参数值缺少有效性校验

许多参数读取失败时默认为 `-1.0`，但 `setParam()` 没有检查。

更稳健的实现应在初始化后验证：

```text
lambda_* >= 0
dist0 > 0
max_vel > 0
max_acc > 0
order == 3
```

### 22.10 `package.xml` 与 CMake 依赖不完全对齐

`CMakeLists.txt` 声明了 `cv_bridge`，但 `package.xml` 未声明；
同时 PCL 和 `cv_bridge` 在核心源码中没有直接使用。

未来可以清理无用依赖，或补齐包清单声明。

---

## 23. 推荐源码阅读顺序

### 第一阶段：理解轨迹表示

1. `include/bspline_opt/uniform_bspline.h`
2. `src/uniform_bspline.cpp`
3. 重点看：
   - `setUniformBspline()`
   - `evaluateDeBoor()`
   - `getDerivative()`
   - `parameterizeToBspline()`
   - `checkFeasibility()`

目标：理解控制点、时间间隔、轨迹求值和导数。

### 第二阶段：理解代价函数

1. `include/bspline_opt/bspline_optimizer.h`
2. `src/bspline_optimizer.cpp`
3. 重点看：
   - `calcSmoothnessCost()`
   - `calcDistanceCostRebound()`
   - `calcFeasibilityCost()`
   - `calcFitnessCost()`
   - `combineCostRebound()`
   - `combineCostRefine()`

目标：理解每一项在优化中解决什么问题。

### 第三阶段：理解反弹机制

重点看：

1. `initControlPoints()`
2. `check_collision_and_rebound()`
3. `rebound_optimize()`
4. `earlyExit()`

目标：理解为什么不依赖 ESDF 也能生成有效碰撞梯度。

### 第四阶段：理解系统调用

阅读：

```text
planner/plan_manage/src/planner_manager.cpp
```

重点看：

- `initPlanModules()`
- `reboundReplan()`
- `refineTrajAlgo()`
- `reparamBspline()`
- `updateTrajInfo()`

目标：将本包放回完整规划系统理解。

### 第五阶段：最后再读求解器

阅读：

```text
include/bspline_opt/lbfgs.hpp
```

一般不需要逐行阅读，先掌握：

- 回调接口；
- 线搜索；
- 收敛条件；
- 返回码；
- 提前终止回调。

---

## 24. 建议的学习实验

### 实验一：只观察 B 样条

构造一组控制点，分别绘制：

- 位置轨迹；
- 速度轨迹；
- 加速度轨迹；
- 控制点多边形。

移动一个控制点，观察局部支撑性。

### 实验二：改变 `ts`

保持控制点不变，只修改时间间隔：

```text
ts 变大：轨迹形状不变，速度和加速度下降
ts 变小：轨迹形状不变，速度和加速度上升
```

这能直观理解时间重分配。

### 实验三：分别关闭代价项

在仿真环境中分别令某个权重接近零：

- 关闭平滑项；
- 关闭碰撞项；
- 关闭可行性项。

观察每项的真实作用。不要在真机实验中这样做。

### 实验四：观察反弹方向

扩展可视化，将每个控制点的：

```text
base_point -> base_point + direction
```

画成箭头。

这会让 `initControlPoints()` 的几何逻辑一目了然。

### 实验五：有限差分检查梯度

为四种代价分别编写小测试，对比解析梯度和数值梯度。

这是修改优化器前最值得做的工程保障。

---

## 25. 常见问题

### Q1：为什么 A* 已经能绕障，还需要 B 样条优化？

A* 路径是离散折线路径：

- 通常不平滑；
- 不满足无人机动力学；
- 不能直接作为高频控制指令；
- 栅格分辨率会限制路径质量。

A* 在这里主要提供绕障拓扑，B 样条优化负责生成连续可执行轨迹。

### Q2：为什么不直接让 B 样条穿过 A* 路径点？

严格穿过每个 A* 点会：

- 继承栅格折线；
- 增加不必要约束；
- 降低优化自由度；
- 产生更大 jerk。

反弹方向只告诉优化器“往哪边绕”，允许它自行找到平滑轨迹。

### Q3：为什么需要固定前后几个控制点？

前后控制点决定轨迹起终端的位置、速度和加速度边界。

如果全部参与优化，轨迹可能：

- 从当前位置跳变；
- 破坏与当前飞行状态的连续性；
- 偏离局部目标。

### Q4：为什么反弹优化后还要检查碰撞？

因为碰撞代价：

- 是基于有限控制点约束构造的；
- 不是完整连续距离场；
- 与平滑、动力学代价存在竞争；
- 优化器可能停在局部最优。

所以最终必须对连续轨迹采样复查。

### Q5：为什么动力学不可行时优先延长时间？

若几何轨迹已经安全且平滑，仅继续移动控制点可能破坏避障形状。

延长时间能在几何形状基本不变的情况下降低速度和加速度，是更自然的修复方式。

### Q6：`dist0` 是否等于无人机与障碍物的真实最小距离？

不完全等于。

它作用在人工构造的方向性控制点距离上，并且地图本身通常已经膨胀。
真实安全裕量由：

```text
地图膨胀 + dist0 + 控制跟踪误差 + 定位误差
```

共同决定。

---

## 26. 总结

`bspline_opt` 包含两类能力：

```text
UniformBspline
  负责轨迹表示、求值、求导、参数化和可行性检查

BsplineOptimizer
  负责从占据地图与 A* 路径构造反弹约束，
  再用 L-BFGS 优化控制点
```

其最核心的思想不是简单地“用 B 样条做平滑”，而是：

```text
用 A* 提供绕障拓扑和人工梯度方向，
用连续优化获得平滑、无碰撞、近似动力学可行的轨迹，
再通过时间重分配和精修保证可执行性。
```

阅读本包时最值得牢牢记住的四条主线：

1. **控制点矩阵是 `3 x N`，每列一个三维控制点。**
2. **A* 不是最终轨迹，而是反弹方向的几何引导。**
3. **反弹阶段优化安全形状，精修阶段修复时间重分配后的形状。**
4. **软动力学代价之后仍需严格可行性检查。**

掌握这些主线后，`bspline_optimizer.cpp` 中一千余行代码就不再是一团复杂循环，而是围绕“构造约束、计算代价、优化、复查、重启”展开的一套完整在线轨迹优化流程。
