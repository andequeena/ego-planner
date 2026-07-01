# `path_searching` 功能包详细学习说明

> 适用源码：`/home/yxc/Desktop/ego-planner_ws/src/planner/path_searching`
>
> 功能包名称：`path_searching`
>
> 所属系统：EGO-Planner

---

## 1. 一句话认识这个包

`path_searching` 提供一个面向局部三维占据地图的 A* 路径搜索库。

在当前 EGO-Planner 中，它不是用来直接生成无人机最终飞行轨迹，而是服务于
`bspline_opt` 的“反弹优化”：

```text
B 样条初始轨迹穿过障碍物
            |
            v
找出进入和离开障碍物的位置
            |
            v
path_searching 使用三维 A* 找到局部绕障路径
            |
            v
bspline_opt 根据 A* 路径构造反弹方向
            |
            v
连续优化器生成平滑、可执行的 B 样条轨迹
```

这个功能包只生成 C++ 库，不启动独立 ROS 节点，也不直接发布话题。

---

## 2. 它在 EGO-Planner 中的位置

完整局部规划链路可以简化为：

```text
传感器点云 / 深度图
        |
        v
plan_env::GridMap
局部占据地图与膨胀障碍物
        |
        v
bspline_opt 检测初始轨迹碰撞区段
        |
        v
path_searching::AStar
寻找碰撞区段两端之间的自由空间路径
        |
        v
bspline_opt 从 A* 路径提取绕障方向
        |
        v
L-BFGS 优化 B 样条控制点
        |
        v
traj_server / 控制器
```

相关功能包：

| 功能包 | 与 `path_searching` 的关系 |
|---|---|
| `plan_env` | 提供 `GridMap` 和膨胀占据查询 |
| `bspline_opt` | 直接创建并调用 `AStar` |
| `plan_manage` | 初始化地图、优化器和 A* 节点池 |
| `traj_utils` | 将 A* 绕障路径显示到 RViz |

---

## 3. 目录结构

```text
path_searching/
├── CMakeLists.txt
├── package.xml
├── include/path_searching/
│   └── dyn_a_star.h
└── src/
    └── dyn_a_star.cpp
```

源码规模很小，核心只有两个文件：

| 文件 | 职责 |
|---|---|
| `dyn_a_star.h` | 定义网格节点、优先队列比较器、A* 类及坐标转换接口 |
| `dyn_a_star.cpp` | 实现节点池初始化、启发函数、起终点调整、A* 搜索和路径回溯 |

---

## 4. 构建产物与依赖

### 4.1 构建产物

`CMakeLists.txt` 只生成一个库：

```cmake
add_library(path_searching
  src/dyn_a_star.cpp
)
```

构建结果：

```text
devel/lib/libpath_searching.so
```

没有：

```cmake
add_executable(...)
```

因此没有独立 ROS 节点。

### 4.2 关键依赖

| 依赖 | 实际用途 |
|---|---|
| `Eigen3` | 三维位置、索引和向量计算 |
| `roscpp` | 时间限制、错误和警告日志 |
| `plan_env` | `GridMap` 与 `getInflateOccupancy()` |
| C++ STL | `priority_queue`、`vector` 和 `shared_ptr` |

### 4.3 当前构建文件中的冗余依赖

`CMakeLists.txt` 还声明了：

- `rospy`
- `std_msgs`
- `visualization_msgs`
- `cv_bridge`
- `PCL`

但当前核心源码没有直接使用它们。

同时 `package.xml` 与 `CMakeLists.txt` 的依赖声明不完全一致。未来整理功能包时，可以删除无用依赖，或补齐必要的包清单声明。

### 4.4 编译标准

当前同时添加：

```cmake
ADD_COMPILE_OPTIONS(-std=c++11)
ADD_COMPILE_OPTIONS(-std=c++14)
```

通常以后出现的 `-std=c++14` 为准，但建议只保留一个标准，避免构建行为依赖编译器参数顺序。

---

## 5. A* 基础原理

### 5.1 A* 要解决的问题

给定：

- 一个起点；
- 一个终点；
- 一个由自由栅格和障碍栅格构成的空间；

A* 希望找到一条低代价的自由空间路径。

对每个候选节点 `n`，A* 计算：

```text
f(n) = g(n) + h(n)
```

其中：

- `g(n)`：从起点走到节点 `n` 的已知代价；
- `h(n)`：从节点 `n` 到终点的估计代价；
- `f(n)`：经过节点 `n` 到终点的总估计代价。

算法总是优先扩展 `fScore` 最小的节点。

### 5.2 OPEN 集和 CLOSED 集

```text
OPENSET
  已被发现但还没有完成扩展的候选节点

CLOSEDSET
  已完成扩展的节点

UNDEFINED
  当前搜索轮次尚未访问的节点
```

本实现用：

```cpp
std::priority_queue<GridNodePtr, ..., NodeComparator>
```

保存 OPEN 集。

---

## 6. `GridNode` 数据结构

定义：

```cpp
struct GridNode
{
    enum_state state;
    int rounds;
    Eigen::Vector3i index;
    double gScore;
    double fScore;
    GridNodePtr cameFrom;
};
```

字段解释：

| 字段 | 含义 |
|---|---|
| `index` | 节点在 A* 局部搜索池中的三维整数索引 |
| `gScore` | 从起点到当前节点的累计代价 |
| `fScore` | `gScore + heuristic` |
| `cameFrom` | 最优父节点，用于最终回溯路径 |
| `state` | OPEN、CLOSED 或未定义 |
| `rounds` | 该节点最后参与的是第几次 A* 搜索 |

### 6.1 为什么需要 `rounds`

该实现会频繁调用 A*，但不想每次搜索前清空最多一百万个节点。

因此每次调用：

```cpp
++rounds_;
```

节点是否属于当前搜索通过：

```cpp
neighborPtr->rounds == rounds_
```

判断。

这样旧搜索留下的：

- `state`
- `gScore`
- `fScore`
- `cameFrom`

不需要逐个重置。只要 `rounds` 不等于当前轮次，该节点就会被视为未探索节点并重新赋值。

这是一种以“搜索代号”复用节点池的优化技巧。

---

## 7. `NodeComparator` 与最小堆

`std::priority_queue` 默认是最大堆，而 A* 需要优先弹出最小 `fScore`。

比较器：

```cpp
bool operator()(GridNodePtr node1, GridNodePtr node2)
{
    return node1->fScore > node2->fScore;
}
```

含义是：

```text
fScore 更大的节点优先级更低
```

因此 `openSet_.top()` 返回当前 `fScore` 最小的节点。

---

## 8. `AStar` 类总览

核心公开接口：

```cpp
void initGridMap(GridMap::Ptr occ_map,
                 const Eigen::Vector3i pool_size);

bool AstarSearch(const double step_size,
                 Eigen::Vector3d start_pt,
                 Eigen::Vector3d end_pt);

std::vector<Eigen::Vector3d> getPath();
```

典型使用方式：

```cpp
AStar::Ptr a_star(new AStar);
a_star->initGridMap(grid_map, Eigen::Vector3i(100, 100, 100));

if (a_star->AstarSearch(0.1, start, goal))
{
    std::vector<Eigen::Vector3d> path = a_star->getPath();
}
```

内部关键成员：

| 成员 | 含义 |
|---|---|
| `grid_map_` | 实际占据地图 |
| `GridNodeMap_` | 可复用的三维 A* 节点池 |
| `POOL_SIZE_` | 节点池三个方向的尺寸 |
| `CENTER_IDX_` | 节点池中心索引 |
| `center_` | 本次搜索的世界坐标中心 |
| `step_size_` | 本次搜索栅格的空间间隔 |
| `inv_step_size_` | `1 / step_size_` |
| `openSet_` | A* OPEN 集 |
| `gridPath_` | 最近一次成功搜索的节点路径 |
| `rounds_` | 搜索轮次编号 |

---

## 9. 动态局部搜索网格

这个 A* 没有直接使用 `GridMap` 的体素索引。

它维护一个固定尺寸的局部节点池，但每次搜索会重新定义：

- 搜索池在世界中的中心；
- 搜索池每个格子的物理尺寸。

### 9.1 搜索中心

每次搜索：

```cpp
center_ = (start_pt + end_pt) / 2;
```

因此局部搜索池以起终点中点为中心。

### 9.2 搜索分辨率

调用者传入：

```cpp
step_size
```

当前 `bspline_opt` 固定使用：

```cpp
AstarSearch(0.1, in, out);
```

所以 A* 搜索分辨率是 `0.1 m`。

这与 `GridMap` 自身的地图分辨率不一定相同。

### 9.3 节点池大小

规划管理器初始化：

```cpp
a_star_->initGridMap(
    grid_map_, Eigen::Vector3i(100, 100, 100));
```

所以节点池总节点数：

```text
100 × 100 × 100 = 1,000,000
```

当 `step_size = 0.1 m` 时，理论覆盖范围约为：

```text
10 m × 10 m × 10 m
```

实际搜索还会避开最外层一圈索引，因此可用范围略小。

### 9.4 `step_size` 与节点池的耦合

```text
物理搜索范围 ≈ POOL_SIZE × step_size
```

减小 `step_size`：

- 路径更精细；
- 同一节点池覆盖的物理范围变小；
- 搜索节点数和耗时可能增加；
- 更容易出现 `Ran out of pool`。

增大 `step_size`：

- 搜索范围更大；
- 路径更粗糙；
- 可能跨过窄障碍或窄通道；
- 绕障方向精度下降。

---

## 10. 坐标与索引转换

### 10.1 世界坐标转局部索引

源码：

```cpp
idx = ((pt - center_) * inv_step_size_
       + Eigen::Vector3d(0.5, 0.5, 0.5))
          .cast<int>()
      + CENTER_IDX_;
```

概念上是：

```text
局部索引 =
    round((世界坐标 - 搜索中心) / step_size)
  + 中心索引
```

### 10.2 局部索引转世界坐标

```cpp
return ((index - CENTER_IDX_).cast<double>() * step_size_)
       + center_;
```

### 10.3 搜索池越界

若转换后的索引不在：

```text
[0, POOL_SIZE)
```

函数会输出：

```text
Ran out of pool
```

并返回失败。

### 10.4 与 `GridMap` 的关系

A* 的局部节点索引仅用于搜索。

判断障碍物时，会先将节点索引转换回世界坐标，再查询：

```cpp
grid_map_->getInflateOccupancy(pos)
```

因此：

```text
A* 局部网格 != GridMap 原生体素网格
```

---

## 11. 占据查询

接口：

```cpp
inline bool checkOccupancy(const Eigen::Vector3d &pos)
{
    return (bool)grid_map_->getInflateOccupancy(pos);
}
```

`GridMap::getInflateOccupancy()` 返回：

```text
0   自由
1   膨胀占据
-1  超出地图范围
```

转换为 `bool` 后：

```text
0  -> false
1  -> true
-1 -> true
```

所以地图外区域会被当作障碍物，这是合理的安全默认行为。

### 为什么查询膨胀地图

路径规划时不能只把无人机当成一个无尺寸质点。

使用膨胀地图可以将：

- 无人机机体尺寸；
- 安全裕量；
- 定位和控制误差；

部分吸收到障碍物膨胀半径中。

---

## 12. 起终点调整

入口：

```cpp
ConvertToIndexAndAdjustStartEndPoints(...)
```

首先将起终点转换为局部索引。

如果离散后的起点位于障碍物中：

```cpp
start_pt =
    (start_pt - end_pt).normalized() * step_size_
  + start_pt;
```

即沿远离终点的方向逐步移动，直到找到自由栅格。

终点位于障碍物中时，则沿远离起点的方向移动。

这一行为适合当前调用场景：

- A* 起终点来自轨迹碰撞段两侧；
- 因为离散化或地图膨胀，它们可能仍落在占据栅格；
- 向碰撞段外侧移动通常能够回到自由区域。

注意：

- 调整后的路径端点不一定等于原始输入坐标；
- 若一直找不到自由栅格，最终会越出节点池并失败；
- 起终点完全重合且位于障碍物中时，方向归一化存在风险。

---

## 13. 三维 26 邻域扩展

当前节点的邻居偏移遍历：

```cpp
for (dx = -1; dx <= 1; ++dx)
  for (dy = -1; dy <= 1; ++dy)
    for (dz = -1; dz <= 1; ++dz)
```

跳过：

```text
dx = dy = dz = 0
```

因此最多有：

```text
3³ - 1 = 26
```

个邻居。

### 13.1 三种移动

| 移动类型 | 偏移示例 | 网格代价 |
|---|---|---:|
| 轴向移动 | `(1, 0, 0)` | `1` |
| 平面对角移动 | `(1, 1, 0)` | `sqrt(2)` |
| 空间对角移动 | `(1, 1, 1)` | `sqrt(3)` |

源码统一计算：

```cpp
static_cost = sqrt(dx * dx + dy * dy + dz * dz);
```

### 13.2 为什么代价没有乘 `step_size`

因为一次搜索中所有边都共享相同 `step_size`。

将所有路径代价同时乘以固定正数不会改变路径排序，因此可以直接使用网格单位代价。

---

## 14. 启发函数

实现了三种启发函数：

```cpp
getDiagHeu()
getManhHeu()
getEuclHeu()
```

当前实际使用：

```cpp
getHeu() -> tie_breaker_ * getDiagHeu()
```

### 14.1 曼哈顿距离

```text
h = |dx| + |dy| + |dz|
```

适合只允许轴向移动的 6 邻域。

对于当前 26 邻域，它通常会高估最短移动代价，不适合作为保持最优性的启发函数。

### 14.2 欧氏距离

```text
h = sqrt(dx² + dy² + dz²)
```

是保守且直观的估计，但对 26 邻域的离散移动结构利用不足。

### 14.3 三维对角线距离

当前使用的启发函数会尽量组合：

- `sqrt(3)` 空间对角移动；
- `sqrt(2)` 平面对角移动；
- `1` 轴向移动。

例如三个方向差值排序后为：

```text
d_min <= d_mid <= d_max
```

无障碍最短网格代价可以理解为：

```text
h =
    sqrt(3) * d_min
  + sqrt(2) * (d_mid - d_min)
  + 1 * (d_max - d_mid)
```

它与 26 邻域的真实无障碍移动代价匹配较好，因此比欧氏距离更有搜索方向性。

### 14.4 Tie-breaker

源码：

```cpp
tie_breaker_ = 1.0 + 1.0 / 10000;
```

实际启发：

```text
h_used = 1.0001 × h_diag
```

这会轻微偏向目标方向，减少大量 `fScore` 完全相同节点造成的扩展。

代价是启发函数被轻微放大，严格来说不再保证完全可采纳，因此可能牺牲极小的最优性来换取速度。

在本系统中，A* 只用于提供绕障方向，而不是最终轨迹，所以这是合理的工程折中。

---

## 15. A* 搜索完整流程

入口：

```cpp
bool AStar::AstarSearch(
    double step_size,
    Vector3d start_pt,
    Vector3d end_pt);
```

### 步骤 1：开启新搜索轮次

```cpp
++rounds_;
```

使旧节点状态自动失效。

### 步骤 2：定义本次局部搜索空间

```cpp
step_size_ = step_size;
inv_step_size_ = 1 / step_size;
center_ = (start_pt + end_pt) / 2;
```

### 步骤 3：转换并修正起终点

```cpp
ConvertToIndexAndAdjustStartEndPoints(...)
```

若无法放入搜索池或无法移出障碍物，返回失败。

### 步骤 4：清空 OPEN 集

`priority_queue` 没有 `clear()`，所以使用：

```cpp
priority_queue<...> empty;
openSet_.swap(empty);
```

### 步骤 5：初始化起点

```cpp
start.gScore = 0;
start.fScore = heuristic(start, end);
start.state = OPENSET;
start.cameFrom = nullptr;
openSet_.push(start);
```

### 步骤 6：循环取出最优候选节点

```cpp
current = openSet_.top();
openSet_.pop();
```

若当前节点等于终点：

```cpp
gridPath_ = retrievePath(current);
return true;
```

### 步骤 7：将当前节点放入 CLOSED 集

```cpp
current->state = CLOSEDSET;
```

### 步骤 8：扩展 26 个邻居

对每个邻居：

1. 跳过搜索池边界；
2. 跳过当前轮次已关闭节点；
3. 查询膨胀占据地图；
4. 计算新的 `tentative_gScore`；
5. 新节点加入 OPEN 集；
6. 已发现节点若有更短路径，则更新父节点和代价。

### 步骤 9：检查时间限制

每轮扩展后检查：

```cpp
if (elapsed > 0.2 seconds)
    return false;
```

### 步骤 10：OPEN 集耗尽

若没有候选节点且未到终点，返回失败。

---

## 16. 路径回溯与输出

找到终点后：

```cpp
retrievePath(current)
```

沿：

```cpp
cameFrom
```

从终点回溯至起点。

所以内部 `gridPath_` 最初顺序是：

```text
终点 -> ... -> 起点
```

`getPath()` 将索引转换为世界坐标并执行：

```cpp
reverse(path.begin(), path.end());
```

最终返回：

```text
起点附近 -> ... -> 终点附近
```

路径由局部 A* 网格节点组成，不保证包含原始输入的精确起终点。

---

## 17. 节点池与内存复用

### 17.1 初始化

```cpp
GridNodeMap_ = new GridNodePtr **[POOL_SIZE_(0)];
```

随后为每个三维索引创建一个 `GridNode`。

默认池大小：

```text
100 × 100 × 100 = 1,000,000 个节点
```

这会占用数十 MB 内存，并在初始化时进行大量小对象分配。

### 17.2 为什么一次分配

在线重规划中，A* 可能被频繁调用。

如果每次搜索都重新创建节点：

- 会产生大量动态分配；
- 增加延迟抖动；
- 降低实时性。

节点池加 `rounds_` 的设计避免了每次搜索重复分配和清空全部节点。

### 17.3 代价

这种设计带来的代价：

- 初始化较慢；
- 内存占用固定且较大；
- 搜索范围受池尺寸限制；
- 手动内存管理复杂；
- 当前实现不是线程安全的。

---

## 18. 与 `bspline_opt` 的真实调用链

### 18.1 初始化位置

在：

```text
planner/plan_manage/src/planner_manager.cpp
```

中：

```cpp
bspline_optimizer_rebound_->a_star_.reset(new AStar);
bspline_optimizer_rebound_->a_star_->initGridMap(
    grid_map_, Eigen::Vector3i(100, 100, 100));
```

### 18.2 初始碰撞段搜索

`BsplineOptimizer::initControlPoints()`：

1. 检测初始 B 样条控制点轨迹穿过障碍物的区段；
2. 对每个区段取自由空间入口和出口；
3. 调用：

```cpp
a_star_->AstarSearch(0.1, in, out);
```

4. 成功后获取：

```cpp
a_star_->getPath();
```

5. 根据该路径为控制点构造反弹方向。

### 18.3 优化过程中发现新碰撞

`BsplineOptimizer::check_collision_and_rebound()` 也会再次调用 A*。

因此一次完整 B 样条优化过程中，A* 可能执行多次。

### 18.4 A* 路径可视化

规划管理器会调用：

```cpp
visualization_->displayAStarList(a_star_pathes, vis_id);
```

默认 RViz 话题：

```text
/ego_planner_node/a_star_list
```

这些线段用于理解反弹优化选择了障碍物哪一侧。

---

## 19. 为什么 A* 路径不是最终轨迹

A* 输出的是三维栅格折线：

- 方向变化离散；
- 包含大量尖角；
- 不保证速度连续；
- 不保证加速度连续；
- 不满足无人机动力学；
- 路径质量受搜索分辨率影响。

EGO-Planner 只从 A* 路径中提取：

```text
绕障拓扑与大致方向
```

再由 B 样条连续优化生成最终轨迹。

这是一种很重要的分工：

```text
A* 擅长解决“从障碍物哪边绕”
B 样条优化擅长解决“怎样平滑且可执行地绕”
```

---

## 20. 搜索参数与调优

当前没有通过 ROS 参数服务器读取 A* 参数。

关键参数均由代码直接决定：

| 参数 | 当前值/来源 | 作用 |
|---|---|---|
| 节点池大小 | `100 × 100 × 100` | 限制最大搜索范围与内存 |
| `step_size` | `0.1 m` | A* 搜索分辨率 |
| 时间限制 | `0.2 s` | 单次搜索最大耗时 |
| 邻域 | 26 邻域 | 允许三维对角运动 |
| 启发函数 | 对角线距离 × `1.0001` | 加速目标导向搜索 |
| 占据判断 | 膨胀占据地图 | 保留安全裕量 |

### 20.1 经常出现 `Ran out of pool`

原因可能包括：

- 起终点距离过远；
- `step_size` 太小；
- 绕障需要离开起终点中点附近较远；
- 节点池尺寸不足。

可以考虑：

```text
增大 POOL_SIZE
适度增大 step_size
缩短一次 A* 搜索的起终点跨度
```

增大池尺寸会显著增加内存和初始化时间。

### 20.2 经常超过 0.2 秒

可以检查：

- 搜索池是否过大；
- `step_size` 是否过小；
- 障碍环境是否复杂；
- 起终点是否被障碍包围；
- 地图更新是否异常；
- 是否存在大量需要探索的开放空间。

可尝试：

```text
增大 step_size
优化节点存储和 OPEN 集更新
减小搜索任务跨度
根据硬件调整时间限制
```

### 20.3 路径过于粗糙

A* 路径本来就不是最终轨迹。

如果粗糙程度已经影响反弹方向，可以：

- 减小 `step_size`；
- 对 A* 路径做简化或平滑；
- 改善控制点与 A* 路径的几何关联；
- 使用 Theta* 等允许任意视线连接的搜索方法。

---

## 21. 调试方法

### 21.1 观察 A* 可视化

在 RViz 中显示：

```text
/ego_planner_node/a_star_list
```

重点观察：

- 路径是否确实位于自由空间；
- 路径从障碍物哪一侧绕过；
- 路径是否触及局部搜索边界；
- 起终点是否因占据被明显向外调整；
- 多次重规划时绕障侧是否频繁跳变。

### 21.2 观察错误日志

#### `Ran out of pool`

说明某个世界坐标无法映射进局部节点池。

#### `Unable to handle the initial or end point`

说明起终点越界，或起终点占据调整过程中越界。

#### `0.2 seconds time limit exceeded`

说明搜索没有在规定时间内完成。

#### `a star error`

这是 `bspline_opt` 对 A* 失败的上层日志，需要结合前面的具体 A* 日志判断原因。

### 21.3 添加建议统计

当前成功搜索时的耗时输出被注释。

调试时可以统计：

```text
搜索耗时
扩展节点数
最终路径节点数
路径网格代价
OPEN 集峰值
起终点调整距离
搜索失败原因
```

相比只打印成功或失败，这些指标更容易定位性能瓶颈。

### 21.4 构造最小测试地图

建议增加单元测试：

1. 完全自由空间；
2. 一堵平面墙；
3. 墙上有一个窄门；
4. 起点位于膨胀障碍中；
5. 终点在搜索池外；
6. 没有可行路径；
7. 三维上下绕障；
8. 连续调用多轮搜索验证 `rounds_` 复用。

当前功能包没有测试目标。

---

## 22. 当前源码中的重要风险与注意事项

以下结论针对当前工作空间源码。

### 22.1 `inf` 实际等于 0

头文件定义：

```cpp
constexpr double inf = 1 >> 20;
```

`1 >> 20` 是整数右移，结果为：

```text
0
```

因此：

```cpp
double gScore{inf}, fScore{inf};
```

初始值实际为 0，而不是无穷大。

当前搜索对“本轮首次发现的节点”会直接覆盖 `gScore/fScore`，所以主要流程通常仍能运行；但这个定义明显错误，并会使代码维护和扩展存在风险。

正确写法可以是：

```cpp
constexpr double inf = std::numeric_limits<double>::infinity();
```

或：

```cpp
constexpr double inf = 1e18;
```

### 22.2 OPEN 集中的节点降价后没有重新入队

当发现更短路径时，源码执行：

```cpp
neighborPtr->gScore = tentative_gScore;
neighborPtr->fScore = ...;
```

但没有重新：

```cpp
openSet_.push(neighborPtr);
```

`std::priority_queue` 不知道其内部指针指向对象的 `fScore` 已经变化，因此不会自动重新建堆。

这可能导致：

- OPEN 集顺序不再满足最小堆性质；
- 搜索扩展顺序异常；
- 性能下降；
- 在某些情况下失去最优性或影响搜索结果。

常见修复方式是允许重复入队，并在弹出时忽略过期条目；或者使用支持 decrease-key 的数据结构。

### 22.3 析构函数没有释放三层指针数组

当前析构只执行：

```cpp
delete GridNodeMap_[i][j][k];
```

但没有释放：

```cpp
delete[] GridNodeMap_[i][j];
delete[] GridNodeMap_[i];
delete[] GridNodeMap_;
```

所以会泄漏用于组织三维指针数组的内存。

虽然节点对象本身被删除，且规划器通常只创建一个长期存在的 A* 实例，但内存管理仍不完整。

更现代的实现可以使用一维连续 `std::vector<GridNode>`，同时减少大量小对象分配。

### 22.4 析构函数默认假设已经初始化

若创建 `AStar` 后未调用 `initGridMap()` 就析构，`POOL_SIZE_` 和 `GridNodeMap_` 的状态可能无效。

当前真实调用链总会初始化，但类本身不够健壮。

### 22.5 坐标四舍五入对负方向不完全对称

源码通过：

```cpp
(value + 0.5).cast<int>()
```

近似四舍五入。

但 C++ 浮点转整数是向 0 截断。对负数方向，这种方式并不总是等价于标准 `round()`。

这可能让搜索中心负方向附近的坐标映射产生一个栅格偏差。

更明确的实现应逐维使用：

```cpp
std::round(...)
```

或使用一致的 `floor()` 规则。

### 22.6 对角移动可能“穿角”

扩展对角邻居时，代码只检查目标邻居是否占据。

例如从：

```text
(0, 0, 0) -> (1, 1, 0)
```

不会额外检查：

```text
(1, 0, 0)
(0, 1, 0)
```

因此理论上可能从两个相邻障碍栅格的角之间穿过。

膨胀地图会降低风险，但若要求严格无穿角，需要检查对角移动经过的中间相邻栅格，或对边进行连续碰撞检测。

### 22.7 成功路径只到达调整后的网格中心

起终点可能被移动到自由栅格，且坐标会离散到 A* 网格中心。

返回路径不保证精确包含原始起终点。

当前用途只是构造 B 样条反弹方向，因此可以接受；若将来直接执行 A* 路径，需要额外连接和碰撞检查。

### 22.8 失败搜索不会清空旧 `gridPath_`

`gridPath_` 只在搜索成功时更新。

若一次搜索失败后仍调用 `getPath()`，会返回上一次成功搜索留下的路径。

当前上层仅在 `AstarSearch()` 返回 `true` 时调用 `getPath()`，所以真实流程是安全的，但 API 容易被误用。

### 22.9 没有校验 `step_size`

源码直接执行：

```cpp
inv_step_size_ = 1 / step_size;
```

如果传入 0 或负数，会产生除零或无效搜索空间。

公开接口应检查：

```text
step_size > 0
```

### 22.10 使用 ROS 时间实施算法超时

搜索时间限制使用：

```cpp
ros::Time::now()
```

当启用 `/use_sim_time` 时，ROS 时间可能暂停、跳变或没有及时更新。

对纯计算耗时限制，更稳健的选择通常是单调墙上时间，例如：

```cpp
ros::WallTime
```

或 `std::chrono::steady_clock`。

### 22.11 `rounds_` 最终可能溢出

`rounds_` 是 `int`，长期极高频运行后理论上可能溢出并与旧节点轮次碰撞。

实际达到该条件需要非常久，但可以改用更大的无符号计数器，并在回绕时显式重置节点池。

### 22.12 当前实现不是线程安全的

同一个 `AStar` 实例共享：

- `openSet_`
- `gridPath_`
- `center_`
- `step_size_`
- `rounds_`
- 节点池状态

不能被多个线程同时调用。

### 22.13 A* 类位于全局命名空间

`AStar`、`GridNode` 和全局常量 `inf` 都没有放入项目命名空间。

这会增加与其他库类型或变量命名冲突的风险。

### 22.14 声明了未实现且未使用的接口

头文件声明：

```cpp
coord2gridIndexFast(...)
```

但当前源码没有实现，也没有调用。

若未来误用，会在链接阶段失败。

### 22.15 搜索超时是硬编码

```cpp
if (elapsed > 0.2)
```

不同硬件、地图密度和飞行速度下合理时间限制不同。

更灵活的实现应将其作为 ROS 参数或构造参数。

---

## 23. 推荐改进方向

### 23.1 低风险修复

优先建议：

1. 修复 `inf` 定义；
2. 校验 `step_size > 0`；
3. 搜索开始时清空 `gridPath_`；
4. 使用单调时钟检查超时；
5. 完整释放节点池；
6. 删除未实现的旧接口或补齐实现；
7. 将硬编码参数改为可配置参数。

### 23.2 OPEN 集正确性改进

可以采用“重复入队 + 延迟删除”模式：

```text
节点获得更小 gScore
        |
        v
更新节点代价并再次 push
        |
        v
弹出时若条目已过期则跳过
```

另一种方式是为队列项保存独立的：

```text
index + 入队时 fScore
```

而不是仅保存可变节点指针。

### 23.3 连续内存节点池

使用：

```cpp
std::vector<GridNode> nodes;
```

并通过：

```text
address = x * size_y * size_z + y * size_z + z
```

访问。

优点：

- 一次内存分配；
- 更好的缓存局部性；
- 析构自动完成；
- 避免三层指针数组泄漏；
- 初始化速度更稳定。

### 23.4 严格碰撞边检查

对每条移动边进行：

- 中间栅格检查；
- 射线检查；
- 或连续采样检查；

可以消除对角穿角问题。

### 23.5 搜索算法扩展

可根据需求考虑：

| 算法 | 适用方向 |
|---|---|
| Weighted A* | 进一步用最优性换速度 |
| Theta* | 生成更少折点、任意角度路径 |
| Jump Point Search | 在规则栅格中减少扩展 |
| D* Lite / LPA* | 地图变化时增量重规划 |
| Kinodynamic A* | 直接考虑速度和动力学状态 |

但在当前 EGO-Planner 中，A* 只承担局部绕障拓扑引导，复杂算法未必带来等比例收益。

---

## 24. 推荐源码阅读顺序

### 第一阶段：掌握接口和数据结构

阅读：

```text
include/path_searching/dyn_a_star.h
```

重点理解：

- `GridNode`
- `NodeComparator`
- `AStar` 成员变量
- `Coord2Index()`
- `Index2Coord()`

### 第二阶段：理解搜索流程

阅读：

```text
src/dyn_a_star.cpp
```

建议顺序：

1. `initGridMap()`
2. `AstarSearch()`
3. `retrievePath()`
4. `getPath()`
5. 三种启发函数
6. `ConvertToIndexAndAdjustStartEndPoints()`

### 第三阶段：理解地图接口

阅读：

```text
planner/plan_env/include/plan_env/grid_map.h
```

重点看：

- `getInflateOccupancy()`
- `isInMap()`
- `posToIndex()`
- `indexToPos()`

### 第四阶段：理解真实用途

阅读：

```text
planner/bspline_opt/src/bspline_optimizer.cpp
```

重点看：

- `initControlPoints()`
- `check_collision_and_rebound()`

理解 A* 路径如何被转换为控制点的反弹方向。

### 第五阶段：理解系统初始化与可视化

阅读：

```text
planner/plan_manage/src/planner_manager.cpp
planner/traj_utils/src/planning_visualization.cpp
```

---

## 25. 建议学习实验

### 实验一：自由空间启发函数对比

在同一起终点下分别使用：

- 对角线启发；
- 欧氏启发；
- 零启发，即 Dijkstra；

记录：

```text
扩展节点数
搜索耗时
路径代价
```

### 实验二：改变 `step_size`

尝试：

```text
0.05 m
0.10 m
0.20 m
```

观察：

- 路径精细程度；
- 搜索耗时；
- 搜索范围；
- 是否出现节点池越界。

### 实验三：验证节点池复用

连续搜索数千次，确认：

- 每轮不会受旧 `state` 干扰；
- 搜索结果稳定；
- 内存不会持续增长；
- `rounds_` 正确区分搜索。

### 实验四：复现优先队列更新问题

构造一个节点先以较高代价进入 OPEN 集，之后发现更短路径。

比较：

- 当前直接修改 `fScore`；
- 修改后重新入队；

观察扩展顺序和最终路径代价。

### 实验五：测试对角穿角

布置两个相邻障碍栅格，只留下对角接触的缝隙，观察 A* 是否从角点穿过。

### 实验六：测试搜索超时

构造无解的复杂障碍环境，观察：

- 0.2 秒限制是否稳定触发；
- 使用 ROS 仿真时间时是否仍正确；
- OPEN 集峰值和扩展数量。

---

## 26. 常见问题

### Q1：为什么叫 `dyn_a_star`？

这里的“dyn”更接近动态局部搜索网格和可复用节点池，而不是显式考虑无人机动力学的 kinodynamic A*。

节点状态只有三维位置索引，没有速度、加速度或控制输入。

### Q2：它是否搜索动态障碍物？

没有。

搜索只查询当前时刻的膨胀占据地图，不预测障碍物未来运动。

### Q3：它是否保证最短路径？

理论标准 A* 在启发函数可采纳且 OPEN 集更新正确时可以保证最优。

当前实现：

- 使用 `1.0001` 放大的启发函数；
- OPEN 节点降价后没有重新建堆；

因此不应宣称严格保证最短路径。

对于只提供绕障引导的用途，路径可行性和搜索速度比严格最优性更重要。

### Q4：为什么不用 `GridMap` 原生分辨率？

独立 `step_size` 允许 A* 搜索精度与占据地图分辨率解耦。

但如果两者差异过大：

- A* 太粗可能漏掉细节；
- A* 太细会重复查询相同地图体素并增加计算量。

### Q5：为什么使用膨胀占据地图？

为了让点状 A* 搜索近似考虑无人机尺寸和安全裕量。

### Q6：为什么只搜索碰撞段两端之间的路径？

EGO-Planner 已经有一条连续初始轨迹。

只需为穿障碍的局部区段找到绕障拓扑，不需要用 A* 重建整条轨迹。这样更快，也更适合高频局部重规划。

### Q7：A* 失败是否一定意味着没有路？

不一定。

也可能是：

- 节点池范围不足；
- 0.2 秒超时；
- 搜索分辨率不合适；
- 起终点调整失败；
- 地图外区域被视为占据；
- 当前实现的数据结构问题影响搜索。

---

## 27. 总结

`path_searching` 是一个小而关键的功能包。

它完成的不是最终轨迹规划，而是：

```text
在局部膨胀占据地图中，
快速为碰撞轨迹段找到一条三维自由空间绕障路径，
从而为 B 样条反弹优化提供方向。
```

理解这个包时，最重要的五条主线是：

1. **它是库，不是 ROS 节点。**
2. **它使用以起终点中点为中心的动态局部搜索网格。**
3. **固定节点池配合 `rounds_`，避免每次搜索清空百万节点。**
4. **26 邻域和三维对角线启发用于快速寻找局部绕障路径。**
5. **A* 路径只提供绕障拓扑，最终轨迹由 `bspline_opt` 连续优化生成。**

当前实现已经满足 EGO-Planner 的主要工程需求，但在：

- OPEN 集降价更新；
- 内存释放；
- `inf` 定义；
- 坐标取整；
- 对角穿角；
- 参数硬编码；
- API 健壮性；

方面仍有明确的改进空间。

阅读完本包后，再回看 `bspline_opt::initControlPoints()`，就能完整理解 EGO-Planner 如何将离散 A* 路径转化为连续优化中的“反弹方向”。
