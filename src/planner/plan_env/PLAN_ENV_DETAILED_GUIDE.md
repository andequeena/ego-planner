# `plan_env` 功能包详细学习说明

> 适用源码：`/home/yxc/Desktop/ego-planner_ws/src/planner/plan_env`
>
> 功能包名称：`plan_env`
>
> 所属系统：EGO-Planner

---

## 1. 一句话认识这个包

`plan_env` 是 EGO-Planner 的三维局部环境地图层。

它接收深度图、相机位姿、里程计或点云，将传感器观测转换为三维体素地图，并向规划模块提供快速的障碍物查询接口。

它主要完成：

- 深度图反投影；
- 相机坐标到世界坐标转换；
- 三维体素射线遍历；
- 基于 log-odds 的概率占据融合；
- 局部地图维护和过期区域清理；
- 障碍物膨胀；
- 虚拟天花板；
- 原始占据地图和膨胀地图可视化；
- 为 A*、B 样条优化和安全检查提供查询接口。

可以把它理解为规划系统的“空间记忆”：

```text
深度图 / 点云 + 位姿
          |
          v
      plan_env
          |
          +--> 原始概率占据地图
          |
          +--> 膨胀占据地图
                    |
                    +--> path_searching 三维 A*
                    +--> bspline_opt 碰撞检测
                    +--> FSM 轨迹安全检查
```

---

## 2. 它在 EGO-Planner 中的位置

完整局部规划数据流：

```text
仿真传感器 / 深度相机 / 激光雷达
                |
                | depth + pose/odom
                | 或 point cloud + odom
                v
          plan_env::GridMap
                |
                +--> occupancy_buffer_
                |    概率占据状态
                |
                +--> occupancy_buffer_inflate_
                     规划直接使用的膨胀障碍物
                           |
              +------------+-------------+
              |                          |
              v                          v
    path_searching::AStar       bspline_opt::BsplineOptimizer
      搜索自由空间路径             检测碰撞并反弹
              |                          |
              +------------+-------------+
                           |
                           v
                     局部安全轨迹
```

`GridMap` 不是独立节点。它由 `ego_planner_node` 内部的 `EGOPlannerManager` 创建：

```cpp
grid_map_.reset(new GridMap);
grid_map_->initMap(nh);
```

因此地图订阅、定时器和发布器都运行在 `ego_planner_node` 进程中。

---

## 3. 目录结构

```text
plan_env/
├── CMakeLists.txt
├── package.xml
├── include/plan_env/
│   ├── grid_map.h
│   └── raycast.h
└── src/
    ├── grid_map.cpp
    └── raycast.cpp
```

### 3.1 `grid_map.h/.cpp`

定义：

- `MappingParameters`
- `MappingData`
- `GridMap`

负责地图参数、数据缓冲区、ROS 通信、传感器回调、概率融合、膨胀、查询和可视化。

### 3.2 `raycast.h/.cpp`

实现三维体素射线遍历：

- 两个批量 `Raycast()` 接口；
- 一个逐步输出体素的 `RayCaster` 类。

核心算法来自 Amanatides & Woo 的快速体素遍历方法。

当前 `GridMap::raycastProcess()` 实际使用 `RayCaster`。

---

## 4. 构建产物与依赖

### 4.1 构建产物

```cmake
add_library(plan_env
  src/grid_map.cpp
  src/raycast.cpp
)
```

构建得到：

```text
devel/lib/libplan_env.so
```

没有独立可执行节点。

### 4.2 关键依赖

| 依赖 | 用途 |
|---|---|
| `roscpp` | 订阅、发布、参数、定时器 |
| `message_filters` | 同步深度图与位姿/里程计 |
| `cv_bridge` / OpenCV | 将 ROS 深度图转换为 `cv::Mat` |
| Eigen | 位姿、坐标变换、索引和向量计算 |
| PCL | 点云输入与地图点云发布 |
| `sensor_msgs` | 深度图和点云消息 |
| `nav_msgs` | 里程计消息 |
| `geometry_msgs` | 相机位姿消息 |

### 4.3 包清单注意事项

`CMakeLists.txt` 使用了：

- `cv_bridge`
- `message_filters`
- `visualization_msgs`
- OpenCV
- PCL

但 `package.xml` 只显式列出了少量 ROS 依赖，声明并不完整。

在干净环境、ROS 打包或 CI 中，这可能导致依赖解析问题。

---

## 5. 两种地图更新模式

当前 `GridMap` 同时具备两条不同的数据链路。

## 5.1 深度图概率融合模式

输入：

```text
/grid_map/depth
/grid_map/pose 或 /grid_map/odom
```

流程：

```text
深度图 + 同步位姿
        |
        v
projectDepthImage()
像素反投影到世界坐标点
        |
        v
raycastProcess()
端点 hit，中间射线体素 miss
        |
        v
log-odds 概率更新
        |
        v
clearAndInflateLocalMap()
清理局部外数据并膨胀障碍
```

特点：

- 能利用射线穿过区域推断自由空间；
- 能通过多帧证据积累减少噪声；
- 原始概率地图和膨胀地图都会维护；
- 计算量较大；
- 依赖相机内参、深度尺度和位姿同步。

## 5.2 点云直接膨胀模式

输入：

```text
/grid_map/cloud
/grid_map/odom
```

流程：

```text
世界坐标系点云 + 里程计
        |
        v
清空无人机附近旧膨胀地图
        |
        v
筛选局部更新范围内点
        |
        v
直接膨胀并写入 occupancy_buffer_inflate_
```

特点：

- 简单直接；
- 不执行射线投影；
- 不维护点云射线经过的自由空间；
- 不更新 `occupancy_buffer_` 概率地图；
- 主要供已经位于世界坐标系的局部点云使用。

### 两种模式的重要区别

| 特性 | 深度图模式 | 点云模式 |
|---|---|---|
| 原始概率占据地图 | 更新 | 不更新 |
| 膨胀占据地图 | 从概率地图生成 | 从点云直接生成 |
| 射线自由空间 | 有 | 无 |
| 多帧概率融合 | 有 | 无 |
| 虚拟天花板 | 有 | 当前点云回调不添加 |
| 主要更新入口 | 20 Hz 定时器 | 点云回调直接更新 |

---

## 6. ROS 话题与定时器

### 6.1 输入话题

| 内部话题名 | 类型 | 用途 |
|---|---|---|
| `/grid_map/depth` | `sensor_msgs/Image` | 深度图 |
| `/grid_map/pose` | `geometry_msgs/PoseStamped` | 相机位姿 |
| `/grid_map/odom` | `nav_msgs/Odometry` | 机体里程计或相机位姿来源 |
| `/grid_map/cloud` | `sensor_msgs/PointCloud2` | 世界坐标局部点云 |

这些话题通常在 `advanced_param.xml` 中重映射到实际传感器话题。

### 6.2 输出话题

| 话题 | 类型 | 内容 |
|---|---|---|
| `/grid_map/occupancy` | `sensor_msgs/PointCloud2` | 原始概率占据体素 |
| `/grid_map/occupancy_inflate` | `sensor_msgs/PointCloud2` | 膨胀后的规划障碍体素 |
| `/grid_map/unknown` | `sensor_msgs/PointCloud2` | 未知体素，但默认定时器未发布 |

### 6.3 定时器

```cpp
occ_timer_ = createTimer(0.05, updateOccupancyCallback);
vis_timer_ = createTimer(0.05, visCallback);
```

频率约：

```text
20 Hz
```

其中：

- `updateOccupancyCallback()` 处理深度图融合；
- `visCallback()` 发布原始和膨胀地图。

---

## 7. 核心数据结构

## 7.1 `MappingParameters`

保存所有固定参数。

主要分类：

```text
地图属性
相机内参
深度过滤
概率融合
射线范围
局部地图维护
可视化与飞行边界
```

关键字段：

| 字段 | 含义 |
|---|---|
| `map_origin_` | 地图世界坐标原点 |
| `map_size_` | 地图物理尺寸 |
| `map_voxel_num_` | 三轴体素数量 |
| `resolution_` | 体素边长 |
| `local_update_range_` | 相机附近保留和更新范围 |
| `obstacles_inflation_` | 障碍膨胀距离 |
| `fx_, fy_, cx_, cy_` | 深度相机内参 |
| `p_hit_, p_miss_` | hit/miss 观测概率 |
| `p_min_, p_max_` | 占据概率上下限 |
| `p_occ_` | 判定占据的概率阈值 |
| `min_ray_length_, max_ray_length_` | 射线有效距离 |
| `virtual_ceil_height_` | 虚拟天花板高度 |
| `ground_height_` | 地图底部高度 |

## 7.2 `MappingData`

保存运行中不断变化的数据。

关键字段：

| 字段 | 含义 |
|---|---|
| `occupancy_buffer_` | 每个体素的 log-odds 概率状态 |
| `occupancy_buffer_inflate_` | 膨胀障碍二值状态 |
| `camera_pos_, camera_q_` | 当前相机世界位姿 |
| `depth_image_` | 当前深度图 |
| `proj_points_` | 深度图投影后的世界坐标点 |
| `count_hit_` | 当前融合轮次中体素 hit 次数 |
| `count_hit_and_miss_` | 当前融合轮次中体素总观测次数 |
| `cache_voxel_` | 等待概率更新的体素队列 |
| `flag_traverse_` | 一帧内射线经过体素去重 |
| `flag_rayend_` | 一帧内射线端点去重 |
| `local_bound_min/max_` | 最近更新区域索引边界 |
| `occ_need_update_` | 是否有待处理深度数据 |
| `local_updated_` | 是否需要重新膨胀局部地图 |

---

## 8. 三维地图存储

### 8.1 体素数量

每轴体素数：

```text
voxel_num_i = ceil(map_size_i / resolution)
```

总缓冲区大小：

```text
N = Nx × Ny × Nz
```

默认仿真参数：

```text
map_size = 40 m × 40 m × 3 m
resolution = 0.1 m
```

对应：

```text
400 × 400 × 30 = 4,800,000 个体素
```

### 8.2 一维连续地址

三维索引：

```text
(x, y, z)
```

转换为一维地址：

```text
address = x × Ny × Nz + y × Nz + z
```

这种布局中 z 方向连续。

### 8.3 世界坐标转索引

```text
id_i = floor((pos_i - origin_i) / resolution)
```

### 8.4 索引转体素中心

```text
pos_i = (id_i + 0.5) × resolution + origin_i
```

### 8.5 地图原点

源码设置：

```cpp
map_origin_ =
    (-map_size_x / 2,
     -map_size_y / 2,
      ground_height);
```

因此：

- x、y 地图以世界原点为中心；
- z 从 `ground_height` 向上延伸；
- 它不是随无人机移动的环形地图；
- 只是在固定全局缓冲区中维护无人机附近的局部有效区域。

---

## 9. 原始概率地图与膨胀地图

## 9.1 `occupancy_buffer_`

类型：

```cpp
std::vector<double>
```

每个体素保存 log-odds 值。

可表示：

- 未知；
- 已知自由；
- 已知占据；
- 多帧观测积累的置信度。

## 9.2 `occupancy_buffer_inflate_`

类型：

```cpp
std::vector<char>
```

值：

```text
0 = 可通行
1 = 膨胀障碍物
```

规划器主要查询这个缓冲区：

```cpp
getInflateOccupancy(pos)
```

## 9.3 为什么分成两个缓冲区

原始概率地图回答：

```text
这个体素被占据的证据有多强？
```

膨胀地图回答：

```text
无人机规划时能不能经过这里？
```

障碍膨胀将机体尺寸和安全裕量转换为地图空间中的禁行区域。

---

## 10. 未知、自由与占据状态

初始化时：

```cpp
occupancy_buffer_ =
    clamp_min_log - unknown_flag
```

其中：

```text
unknown_flag = 0.01
```

所以未知状态被编码为略低于最小 log-odds 的特殊值。

判断逻辑：

```text
unknown:
  occupancy < clamp_min_log - 1e-3

known free:
  occupancy >= clamp_min_log
  且 inflated occupancy == 0

known occupied:
  inflated occupancy == 1
```

`getOccupancy()` 使用原始概率阈值：

```text
occupancy_buffer > min_occupancy_log
```

而 `getInflateOccupancy()` 直接读取膨胀二值地图。

---

## 11. Log-odds 概率融合

### 11.1 为什么使用 log-odds

概率：

```text
p ∈ (0, 1)
```

转换为 log-odds：

```text
l = log(p / (1 - p))
```

多次独立观测的更新可以近似写成加法：

```text
l_new = l_old + l_measurement
```

比反复执行贝叶斯乘除更高效。

### 11.2 参数转换

初始化时：

```cpp
prob_hit_log_ = logit(p_hit)
prob_miss_log_ = logit(p_miss)
clamp_min_log_ = logit(p_min)
clamp_max_log_ = logit(p_max)
min_occupancy_log_ = logit(p_occ)
```

默认 launch 参数大致对应：

| 参数 | 概率 | log-odds 约值 |
|---|---:|---:|
| `p_hit` | `0.65` | `+0.619` |
| `p_miss` | `0.35` | `-0.619` |
| `p_min` | `0.12` | `-1.992` |
| `p_max` | `0.90` | `+2.197` |
| `p_occ` | `0.80` | `+1.386` |

这意味着一个新体素通常需要多次 hit 才会跨过 `p_occ = 0.8` 的占据阈值。

### 11.3 单帧观测汇总

同一融合轮次中，一个体素可能被多条射线重复观察。

`setCacheOccupancy()` 记录：

```text
count_hit
count_hit_and_miss
```

最终只对该体素执行一次概率更新。

判定：

```text
若 hit_count >= miss_count:
    使用 prob_hit_log
否则:
    使用 prob_miss_log
```

平票时 hit 获胜，这是一种偏保守的障碍物策略。

### 11.4 概率截断

更新后限制在：

```text
[clamp_min_log, clamp_max_log]
```

作用：

- 防止置信度无限增大；
- 让曾经稳定占据的体素仍能被后续 miss 清除；
- 让曾经稳定自由的体素仍能被新障碍覆盖。

---

## 12. 深度图输入与同步

### 12.1 两种位姿输入

参数：

```text
grid_map/pose_type
```

取值：

| 值 | 模式 |
|---:|---|
| `1` | `PoseStamped`，深度图与 `/grid_map/pose` 同步 |
| `2` | `Odometry`，深度图与 `/grid_map/odom` 同步 |

当前 launch 默认：

```text
pose_type = 2
```

### 12.2 近似时间同步

使用：

```cpp
message_filters::sync_policies::ApproximateTime
```

深度图和位姿不必时间戳完全相同，但时间差过大会造成：

- 障碍物位置拖影；
- 飞行中地图错位；
- 膨胀地图出现虚假障碍；
- 轨迹错误碰撞或漏检。

### 12.3 深度图编码

若输入为：

```text
TYPE_32FC1
```

会按：

```text
k_depth_scaling_factor
```

转换为 `CV_16UC1`。

默认：

```text
k_depth_scaling_factor = 1000
```

对应毫米深度整数。

---

## 13. 相机位姿转换

## 13.1 `PoseStamped` 模式

`depthPoseCallback()` 直接把输入位姿作为相机世界位姿：

```cpp
camera_pos = pose.position
camera_q = pose.orientation
```

所以输入话题必须确实表示相机光学坐标系在世界中的位姿。

## 13.2 `Odometry` 模式

`depthOdomCallback()` 先读取机体世界位姿，再乘固定外参：

```text
camera_to_world =
    body_to_world × camera_to_body
```

当前 `cam2body_` 在源码中硬编码：

```text
[ 0  0  1   0
 -1  0  0   0
  0 -1  0  -0.02
  0  0  0   1 ]
```

它包含相机坐标轴到机体坐标轴的旋转，以及 z 方向约 `-0.02 m` 平移。

如果实际相机安装方向或位置不同，必须修改外参，否则深度地图会整体错位。

---

## 14. 深度图反投影

入口：

```cpp
projectDepthImage()
```

像素：

```text
(u, v)
```

深度：

```text
d
```

在相机坐标系中的三维点：

```text
x = (u - cx) d / fx
y = (v - cy) d / fy
z = d
```

转换到世界坐标：

```text
p_world = R_camera_world p_camera + t_camera_world
```

### 14.1 深度过滤

启用 `use_depth_filter` 后：

- 忽略图像边缘 `depth_filter_margin`；
- 每隔 `skip_pixel` 个像素采样；
- 小于 `depth_filter_mindist` 的深度跳过；
- 无效或大于 `depth_filter_maxdist` 的深度被当作超出最大射线范围；
- 可选的前后帧一致性检查当前被 `if (false)` 禁用。

### 14.2 第一帧行为

开启深度过滤时，第一帧只用于建立：

- `last_camera_pos_`
- `last_camera_q_`
- `last_depth_image_`

不会产生投影点。

从第二帧开始正常更新地图。

---

## 15. 三维射线遍历

### 15.1 目的

深度点只告诉系统：

```text
射线末端可能有障碍物
```

而相机到深度点之间的空间通常应该是自由的。

所以需要遍历：

```text
深度端点 -> 相机中心
```

沿途所有体素标记 miss。

### 15.2 算法

`RayCaster` 使用 Amanatides & Woo 快速体素遍历算法。

核心维护：

```text
tMaxX / tMaxY / tMaxZ
```

分别表示射线下一次穿过 x、y、z 体素边界时的参数。

每一步选择最小的 `tMax`，进入对应方向的下一个体素。

复杂度与射线穿过的体素数量近似线性相关。

### 15.3 为什么输入除以分辨率

调用：

```cpp
raycaster.setInput(
    pt_world / resolution,
    camera_pos / resolution);
```

这样世界米制坐标被转换为体素坐标，整数边界就对应体素边界。

输出体素再转换回世界中心：

```cpp
(ray_pt + 0.5) * resolution
```

### 15.4 端点和沿途状态

端点：

```text
在有效最大距离内 -> hit
超出地图或最大距离 -> miss
```

沿途：

```text
miss
```

端点体素也可能被射线遍历记录一次 miss，但同帧 hit/miss 平票时 hit 获胜。

---

## 16. 射线去重与缓存更新

一帧深度图可能产生数万条射线，很多射线会穿过相同体素。

为避免重复工作，代码使用：

```text
flag_rayend_
flag_traverse_
raycast_num_
```

### 16.1 端点去重

若多个深度像素落入相同端点体素，该端点只执行一次完整射线处理。

### 16.2 穿越体素去重

若当前射线进入一个本帧其他射线已经遍历过的体素，则停止继续向相机回溯。

这利用了相邻相机射线接近相机时高度重合的特点，显著减少计算量。

### 16.3 缓存队列

一个体素在本帧第一次收到 hit/miss 时加入：

```cpp
cache_voxel_
```

所有射线处理完后，队列中的每个体素只执行一次 log-odds 更新。

---

## 17. 最大射线范围与地图裁剪

### 17.1 超出地图

投影点位于地图外时：

```cpp
closetPointInMap()
```

将射线端点裁剪到地图边界内。

该端点记为 miss，因为系统没有观测到边界处存在障碍物。

### 17.2 超过最大射线距离

若深度点距离相机超过：

```text
max_ray_length
```

将端点裁剪到最大距离，并记为 miss。

### 17.3 最小射线距离

参数：

```text
min_ray_length
```

虽然被读取，但 `raycastProcess()` 中对应检查当前被注释：

```cpp
// if (length < mp_.min_ray_length_) break;
```

所以当前实际运行中该参数基本不起作用。

---

## 18. 局部地图维护

虽然底层缓冲区覆盖整个固定地图，但只维护无人机附近的局部有效区域。

### 18.1 更新区域

射线处理过程中统计本次观测涉及的包围盒：

```text
local_bound_min_
local_bound_max_
```

### 18.2 局部更新范围

以相机为中心：

```text
camera_pos ± local_update_range
```

范围外被本轮触及的体素会重置为最低自由状态。

### 18.3 清除旧区域

`clearAndInflateLocalMap()` 会将最近局部更新区域加 margin 后，外围一圈旧概率体素重置为未知。

目的：

- 防止局部地图无限保留过期障碍；
- 限制膨胀和发布计算量；
- 适应滚动局部规划。

注意：底层缓冲区本身没有随无人机移动，只是不断清理并复用固定全局地图中的局部区域。

---

## 19. 障碍物膨胀

### 19.1 膨胀步数

```text
inf_step = ceil(obstacles_inflation / resolution)
```

默认：

```text
obstacles_inflation = 0.099 m
resolution = 0.1 m
```

得到：

```text
inf_step = 1
```

### 19.2 深度概率地图模式

每个占据体素向：

```text
[-inf_step, +inf_step]
```

三个方向的立方体邻域膨胀。

当 `inf_step = 1` 时，一个障碍体素最多影响：

```text
3 × 3 × 3 = 27 个体素
```

这是立方体膨胀，不是严格欧氏球形膨胀。

### 19.3 点云模式

点云回调中：

- x、y 膨胀使用 `inf_step`；
- z 方向固定使用 `inf_step_z = 1`。

因此点云模式的 z 膨胀与 `obstacles_inflation` 不完全一致。

### 19.4 规划使用

以下模块都主要查询膨胀地图：

```text
path_searching::AStar
bspline_opt::BsplineOptimizer
EGOReplanFSM::checkCollisionCallback
```

---

## 20. 虚拟天花板

若：

```text
virtual_ceil_height > -0.5
```

深度地图膨胀阶段会将对应高度整层体素设为占据：

```text
z = virtual_ceil_height
```

作用：

- 限制规划飞行高度；
- 防止轨迹从障碍物上方无限抬升；
- 符合室内或指定空域限制。

当前 launch 默认：

```text
virtual_ceil_height = 2.5 m
```

可视化截断高度：

```text
visualization_truncate_height = 2.4 m
```

因此虚拟天花板通常不会在默认占据点云可视化中显示出来。

---

## 21. 点云直接更新模式详解

入口：

```cpp
cloudCallback()
```

### 21.1 前置条件

必须已经收到里程计：

```cpp
md_.has_odom_ == true
```

点云必须：

- 非空；
- 位置有效；
- 位于预期世界坐标系。

源码没有对点云执行坐标变换。

### 21.2 清除旧膨胀地图

每次点云到达时，先清空：

```text
camera_pos ± local_update_range
```

范围内的 `occupancy_buffer_inflate_`。

### 21.3 筛选与膨胀

只处理与相机位置各轴差值均小于局部更新范围的点。

每个点直接在膨胀缓冲区写入占据。

### 21.4 适用场景

适合：

- 仿真局部感知点云；
- 已经在世界坐标系的激光雷达局部点云；
- 不需要概率融合的快速地图输入。

不适合直接输入：

- 传感器坐标系点云；
- 需要射线自由空间清理的稠密历史地图；
- 含大量未滤除地面点的点云。

---

## 22. 地图查询接口

### 22.1 原始占据查询

```cpp
int getOccupancy(Vector3d pos);
int getOccupancy(Vector3i idx);
```

返回：

```text
1  原始概率超过占据阈值
0  未超过占据阈值
-1 地图外
```

未知体素的原始值低于占据阈值，所以这里也返回 0。

### 22.2 膨胀占据查询

```cpp
int getInflateOccupancy(Vector3d pos);
```

返回：

```text
1  膨胀障碍
0  非膨胀障碍
-1 地图外
```

上层常将返回值转换为 `bool`，因此地图外 `-1` 会被视为障碍物，这是安全行为。

### 22.3 状态查询

```cpp
isUnknown()
isKnownFree()
isKnownOccupied()
```

适合需要区分未知与已知自由空间的规划逻辑。

### 22.4 地图属性

```cpp
getResolution()
getOrigin()
getRegion()
```

当前 `getVoxelNum()` 在头文件声明，但实现被注释，不能安全调用。

---

## 23. 可视化

### 23.1 原始地图

```text
/grid_map/occupancy
```

显示：

```text
occupancy_buffer > p_occ 阈值
```

的体素。

### 23.2 膨胀地图

```text
/grid_map/occupancy_inflate
```

显示规划器实际认为不可通行的区域。

调试规划问题时，这个话题通常比原始地图更重要。

### 23.3 未知地图

`publishUnknown()` 已实现，但默认：

```cpp
visCallback()
```

没有调用它。

需要调试未知空间时，可临时启用发布。

### 23.4 高度截断

发布点云时，超过：

```text
visualization_truncate_height
```

的体素不显示。

这只影响 RViz，不改变规划地图。

---

## 24. 默认参数详解

参数来自：

```text
planner/plan_manage/launch/advanced_param.xml
```

### 24.1 地图与局部范围

| 参数 | 默认值 | 作用 |
|---|---:|---|
| `grid_map/resolution` | `0.1` | 体素边长 |
| `grid_map/map_size_x` | launch 输入 | 全局缓冲区 x 尺寸 |
| `grid_map/map_size_y` | launch 输入 | 全局缓冲区 y 尺寸 |
| `grid_map/map_size_z` | launch 输入 | 全局缓冲区 z 尺寸 |
| `grid_map/local_update_range_x` | `5.5` | 局部更新半径 x |
| `grid_map/local_update_range_y` | `5.5` | 局部更新半径 y |
| `grid_map/local_update_range_z` | `4.5` | 局部更新半径 z |
| `grid_map/local_map_margin` | `30` | 局部边界额外体素 margin |
| `grid_map/ground_height` | `-0.01` | 地图底部高度 |

### 24.2 相机与深度过滤

| 参数 | 默认值 | 作用 |
|---|---:|---|
| `fx, fy, cx, cy` | launch 输入 | 相机内参 |
| `use_depth_filter` | `true` | 是否启用深度过滤和降采样 |
| `depth_filter_tolerance` | `0.15` | 前后帧一致性阈值，当前逻辑禁用 |
| `depth_filter_maxdist` | `5.0` | 深度过滤最大距离 |
| `depth_filter_mindist` | `0.2` | 深度过滤最小距离 |
| `depth_filter_margin` | `1` | 跳过图像边缘像素 |
| `k_depth_scaling_factor` | `1000` | 深度整数尺度 |
| `skip_pixel` | `2` | 像素降采样步长 |

### 24.3 概率融合

| 参数 | 默认值 | 作用 |
|---|---:|---|
| `p_hit` | `0.65` | hit 观测置信度 |
| `p_miss` | `0.35` | miss 观测置信度 |
| `p_min` | `0.12` | 最低占据概率 |
| `p_max` | `0.90` | 最高占据概率 |
| `p_occ` | `0.80` | 判定占据阈值 |
| `min_ray_length` | `0.1` | 当前代码中检查被注释 |
| `max_ray_length` | `4.5` | 最大射线融合距离 |

### 24.4 规划安全与可视化

| 参数 | 默认值 | 作用 |
|---|---:|---|
| `obstacles_inflation` | `0.099` | 障碍膨胀距离 |
| `virtual_ceil_height` | `2.5` | 虚拟天花板 |
| `visualization_truncate_height` | `2.4` | RViz 点云最高显示高度 |
| `pose_type` | `2` | 使用里程计同步深度图 |
| `frame_id` | `world` | 地图发布坐标系 |

---

## 25. 参数调优建议

### 25.1 地图噪点太多

检查：

- 深度尺度是否正确；
- 相机内参是否正确；
- 相机外参是否正确；
- 深度和位姿时间同步；
- 点云是否已转换到世界坐标系。

可以尝试：

```text
增大 p_occ
减小 p_hit
增大 depth_filter_mindist
减小 depth_filter_maxdist
增大 skip_pixel
```

### 25.2 障碍物很难从地图中消失

可以尝试：

```text
降低 p_max
减小 p_hit
增大 miss 更新影响，即选择更小 p_miss
```

还需确认射线自由空间是否正确经过旧障碍。

点云直接模式没有射线 miss 融合，而是依赖每帧清空局部膨胀区域。

### 25.3 规划器离障碍物太远

检查：

```text
obstacles_inflation
bspline_opt 的 dist0
传感器噪点
```

膨胀和 B 样条安全距离会叠加产生保守效果。

### 25.4 规划轨迹偶尔穿过障碍

可以考虑：

```text
减小地图 resolution
增大 obstacles_inflation
提高地图更新频率
降低飞行速度
改善同步和外参
```

但更小分辨率会以三次方增加体素数量和计算量。

### 25.5 地图更新太慢

可尝试：

```text
增大 skip_pixel
减小 local_update_range
增大 resolution
减少可视化发布
降低输入深度图分辨率
```

---

## 26. 调试方法

### 26.1 同时显示两种地图

在 RViz 中同时显示：

```text
/grid_map/occupancy
/grid_map/occupancy_inflate
```

可快速判断问题来自：

- 传感器概率地图；
- 还是障碍膨胀。

### 26.2 检查坐标系

确认：

- 发布点云的 `frame_id` 与实际世界坐标一致；
- 深度相机位姿是相机坐标系位姿；
- 点云模式输入已在世界系；
- 里程计方向与硬编码相机外参匹配。

### 26.3 检查深度尺度

若真实 2 米物体被投影到 2000 米或 0.002 米，通常是：

```text
k_depth_scaling_factor
```

与输入编码不匹配。

### 26.4 检查概率更新

可以临时打印某个固定体素每帧的：

```text
hit_count
miss_count
log_odds
occupancy state
```

以确认障碍出现和消失是否符合预期。

### 26.5 检查更新延迟

记录：

```text
深度消息时间戳
位姿消息时间戳
同步后回调时间
地图定时器处理时间
```

地图错位往往来自时序，而不是概率参数。

### 26.6 建议测试场景

1. 单个静态立方体；
2. 平面墙；
3. 障碍物出现后移走；
4. 相机快速旋转；
5. 地图边界附近障碍；
6. 虚拟天花板；
7. 超过最大深度的区域；
8. 点云与深度模式分别运行。

---

## 27. 当前源码中的重要风险与注意事项

以下结论针对当前工作空间中的具体实现。

### 27.1 深度过滤读取了下一个像素判断零值

过滤循环中：

```cpp
depth = (*row_ptr) * inv_factor;
row_ptr = row_ptr + skip_pixel;

if (*row_ptr == 0)
```

`depth` 来自当前像素，但零值判断发生在指针移动之后，检查的是下一个采样位置。

影响：

- 当前像素无效时可能未被正确识别；
- 下一个像素无效时可能误伤当前像素；
- 接近行尾时存在越界读取风险。

应在移动指针前保存当前原始深度值。

### 27.2 投影点缓冲区容量假设固定图像尺寸

初始化：

```cpp
proj_points_.resize(640 * 480 / skip_pixel / skip_pixel);
```

若：

- 输入图像大于 640×480；
- 关闭深度过滤后遍历全部像素；
- `skip_pixel` 参数不合理；

则：

```cpp
proj_points_[proj_points_cnt++]
```

可能越界写入。

更稳健的方式是按实际图像尺寸动态 `resize()`，或使用 `push_back()` 并预留容量。

### 27.3 `skip_pixel` 缺少有效性检查

默认参数读取失败为 `-1`。

它被用于：

- 除法计算缓冲区容量；
- 循环步长；

若为 0 或负数，会产生严重错误或死循环。

### 27.4 大部分关键参数默认值为负数且未校验

例如：

```text
resolution
map_size
local_update_range
obstacles_inflation
相机内参
depth scaling
skip_pixel
```

如果 launch 未正确加载，可能出现：

- 除零；
- 巨大内存申请；
- 空循环；
- 越界；
- 无意义地图。

`initMap()` 应显式验证参数。

### 27.5 障碍膨胀边界检查使用一维地址，可能跨轴回绕

深度模式膨胀时：

```cpp
int idx_inf = toAddress(inf_pt);
if (idx_inf < 0 || idx_inf >= total_size)
    continue;
```

只检查一维地址，没有逐轴调用：

```cpp
isInMap(inf_pt)
```

某个轴越界时，一维地址仍可能落在合法总范围内，从而错误写到另一行或另一层体素。

边界附近障碍可能产生地图回绕污染。

点云模式正确使用了 `isInMap(inf_pt)`，两条路径行为不一致。

### 27.6 虚拟天花板索引没有边界检查

```cpp
ceil_id = floor(...)
occupancy_buffer_inflate_[toAddress(x, y, ceil_id)] = 1;
```

若虚拟天花板位于地图 z 范围外，可能越界写入。

### 27.7 `raycast_num_` 和标记缓冲区使用 `char`

```cpp
char raycast_num_;
vector<char> flag_traverse_;
vector<char> flag_rayend_;
```

每次融合：

```cpp
raycast_num_ += 1;
```

`char` 很快会回绕。回绕后，某些旧标记可能恰好等于当前轮次编号，导致射线被错误跳过或提前终止。

20 Hz 下几秒到十几秒就可能发生一次数值回绕，具体取决于 `char` 符号范围。

更稳健的实现应使用更宽整数，或在回绕时清空标记缓冲区。

### 27.8 `min_ray_length` 当前没有实际生效

对应检查被注释，所以近距离射线仍会参与融合。

### 27.9 深度前后帧一致性检查被禁用

代码存在，但被：

```cpp
if (false)
```

完全关闭。

因此：

```text
depth_filter_tolerance
```

当前基本不影响地图。

### 27.10 点云和深度订阅始终同时创建

无论选择哪种模式，初始化都会订阅：

- 深度图；
- 点云；
- 里程计。

如果深度和点云话题同时有数据，两条地图更新链路可能同时修改膨胀缓冲区，行为难以预测。

建议显式配置输入模式，只启用一种主要传感器链路。

### 27.11 点云模式不更新概率地图

点云回调只写：

```text
occupancy_buffer_inflate_
```

所以：

- `/grid_map/occupancy` 可能为空或过期；
- `getOccupancy()` 与 `getInflateOccupancy()` 结果可能不一致；
- 未知/已知自由查询不反映点云模式真实状态。

### 27.12 点云模式不添加虚拟天花板

虚拟天花板只在：

```cpp
clearAndInflateLocalMap()
```

中添加，而点云回调不调用该函数。

若仅使用点云模式，`virtual_ceil_height` 可能不会限制规划。

### 27.13 `depthOdomCallback()` 状态更新不完整

与 `depthPoseCallback()` 相比，它没有：

- 检查相机是否在地图内；
- 设置 `has_odom_`；
- 增加 `update_num_`。

它会直接设置：

```cpp
occ_need_update_ = true;
```

真实流程通常还有独立里程计订阅，但两种位姿模式的状态行为不一致。

### 27.14 相机外参硬编码

`cam2body_` 不从参数读取。

换相机、换机架或改变安装角度后，若忘记修改源码，地图会系统性错位。

### 27.15 `body2world` 没有显式整体初始化

`depthOdomCallback()` 创建：

```cpp
Eigen::Matrix4d body2world;
```

随后设置旋转、平移和 `(3,3)`，但未显式设置底行前三项为 0。

当前计算相机位置和旋转主要依赖前三行，通常不会直接受影响，但齐次矩阵应完整初始化为：

```cpp
Eigen::Matrix4d::Identity()
```

### 27.16 `setOccupancy()` 与概率缓冲区语义不一致

该接口直接向 log-odds 缓冲区写入：

```text
0 或 1
```

而缓冲区其余位置保存的是 log-odds。

虽然当前工作空间中没有调用它，但接口语义容易误用。

### 27.17 `getVoxelNum()` 声明但未实现

头文件声明：

```cpp
int getVoxelNum();
```

源码实现被注释。调用会导致链接错误。

### 27.18 `publishDepth()` 声明但未实现

同样属于遗留接口。

### 27.19 `toAddress()` 存在重复的不可达 return

源码连续写了两次相同 `return`。

不影响运行，但应清理。

### 27.20 `isUnknown()` 会将地图外索引夹到边界

`isUnknown(Vector3i)` 先调用：

```cpp
boundIndex()
```

所以地图外查询实际返回最近边界体素的未知状态，而不是明确表示地图外。

### 27.21 `Raycast()` 批量接口存在距离单位不一致

第一个批量接口中：

```text
maxDist = squaredNorm
dist = Euclidean norm
```

随后直接比较，单位不一致。

当前地图主流程使用 `RayCaster`，不会触发该问题；但批量接口本身存在缺陷。

### 27.22 `RayCaster` 的零方向轴依赖浮点除零行为

当某轴无移动时：

```cpp
tDelta = step / delta = 0 / 0
```

可能产生 NaN。

当前分支逻辑通常仍可工作，但实现依赖 IEEE 浮点比较行为，不够明确。

### 27.23 没有独立测试

该包承担规划安全关键地图功能，但当前没有：

- 单元测试；
- 射线测试；
- 概率融合测试；
- 地图边界测试；
- 膨胀测试。

---

## 28. 推荐改进顺序

### 第一优先级：地图正确性

1. 修复深度像素指针移动后的零值判断；
2. 膨胀前逐轴检查 `isInMap(inf_pt)`；
3. 检查虚拟天花板索引；
4. 将射线轮次标记改为更宽整数；
5. 校验所有关键参数；
6. 动态调整投影点缓冲区。

### 第二优先级：配置与模式清晰化

1. 增加明确输入模式参数；
2. 相机外参改为 ROS 参数或 TF；
3. 统一深度位姿两种回调的状态更新；
4. 明确点云模式是否需要虚拟天花板；
5. 恢复或删除无效参数。

### 第三优先级：代码维护性

1. 补齐 `package.xml` 依赖；
2. 删除未实现接口；
3. 清理重复和不可达代码；
4. 将 `logit` 宏改为内联函数；
5. 避免全局 `using namespace std`；
6. 增加测试与性能统计。

---

## 29. 推荐源码阅读顺序

### 第一阶段：理解地图数据

阅读：

```text
include/plan_env/grid_map.h
```

重点：

- `MappingParameters`
- `MappingData`
- 坐标与索引转换
- 占据查询接口

### 第二阶段：理解初始化和 ROS 接口

阅读：

```cpp
GridMap::initMap()
```

理解：

- 参数来源；
- 地图尺寸；
- 缓冲区初始化；
- 话题与定时器；
- 位姿输入模式。

### 第三阶段：理解深度概率融合

按顺序阅读：

1. `depthPoseCallback()` / `depthOdomCallback()`
2. `updateOccupancyCallback()`
3. `projectDepthImage()`
4. `raycastProcess()`
5. `setCacheOccupancy()`
6. `clearAndInflateLocalMap()`

### 第四阶段：理解射线算法

阅读：

```text
include/plan_env/raycast.h
src/raycast.cpp
```

重点：

- `RayCaster::setInput()`
- `RayCaster::step()`

### 第五阶段：理解点云模式

阅读：

```cpp
odomCallback()
cloudCallback()
```

### 第六阶段：理解规划器如何消费地图

阅读：

```text
planner/path_searching/src/dyn_a_star.cpp
planner/bspline_opt/src/bspline_optimizer.cpp
planner/plan_manage/src/ego_replan_fsm.cpp
```

---

## 30. 建议学习实验

### 实验一：观察 log-odds 累积

固定一个体素，连续输入：

```text
hit hit hit miss miss ...
```

记录概率如何跨过占据阈值并逐步恢复自由。

### 实验二：验证射线自由空间

在相机前放一个平面墙，观察：

- 墙面端点成为占据；
- 相机与墙之间成为自由；
- 墙后区域保持未知。

### 实验三：改变膨胀距离

分别设置：

```text
0.0
0.099
0.2
0.4
```

观察膨胀地图、A* 路径和 B 样条轨迹的变化。

### 实验四：比较深度和点云模式

对同一场景分别输入：

- 深度图；
- 世界坐标点云；

比较：

- 原始概率地图；
- 膨胀地图；
- 障碍消失后的清理行为；
- 计算耗时。

### 实验五：测试地图边界

在地图 x/y/z 边界附近放置障碍，检查膨胀是否错误回绕到另一侧。

### 实验六：验证外参

缓慢旋转无人机，观察静态墙面是否在世界坐标中保持稳定。

若墙面随无人机旋转或漂移，优先检查相机外参和时间同步。

### 实验七：射线轮次回绕

长时间运行深度融合，观察 `char raycast_num_` 回绕时地图是否出现周期性异常。

---

## 31. 常见问题

### Q1：为什么规划器主要查询膨胀地图，而不是原始地图？

原始地图只描述观测到的障碍表面。

无人机具有尺寸，且存在定位和控制误差，所以规划必须在障碍周围保留禁行裕量。

### Q2：未知空间能否通行？

当前主要规划查询使用：

```cpp
getInflateOccupancy()
```

未知体素默认在膨胀缓冲区中为 0，因此通常被当作可通行。

地图外区域返回 `-1`，上层转换为 `bool` 后会被当作障碍物。

### Q3：为什么深度点超过最大距离后被记为 miss？

超过有效范围时，系统只知道最大射线长度以内没有观察到障碍，不应该在截断端点凭空创建障碍物。

### Q4：为什么要限制概率上下界？

如果概率无限接近 0 或 1，后续相反观测将很难改变状态，动态障碍会永久残留。

### Q5：为什么地图尺寸很大，但只更新局部区域？

固定全局缓冲区方便世界坐标索引；局部更新和清理则控制实时计算量并避免过期地图长期保留。

### Q6：点云模式是否需要里程计？

需要。里程计用于确定局部清空和筛选范围的中心。

但源码不会用里程计转换点云，所以输入点云必须已经处于世界坐标系。

### Q7：`obstacles_inflation = 0.099` 为什么在 0.1 米分辨率下仍膨胀一格？

因为：

```text
ceil(0.099 / 0.1) = 1
```

### Q8：为什么虚拟天花板看不见？

默认天花板高度为 2.5 米，而可视化截断高度为 2.4 米。它参与规划，但不会显示在默认点云中。

---

## 32. 总结

`plan_env` 是 EGO-Planner 的安全基础层。

它把原始传感器数据变成规划器能够快速查询的三维局部环境：

```text
深度图模式：
深度投影 -> 射线遍历 -> log-odds 融合 -> 局部清理 -> 障碍膨胀

点云模式：
局部点云 -> 清空旧膨胀地图 -> 直接膨胀
```

理解这个包时，最重要的六条主线是：

1. **`GridMap` 运行在 `ego_planner_node` 内部，不是独立节点。**
2. **原始概率地图负责融合证据，膨胀地图负责规划安全。**
3. **深度端点提供 hit，相机射线沿途提供 miss。**
4. **log-odds 让多帧概率更新转化为高效加法。**
5. **固定全局缓冲区只维护无人机附近的局部有效区域。**
6. **A*、B 样条优化和 FSM 最终都依赖膨胀地图的正确性。**

地图错误往往比优化器错误更隐蔽，也更危险。调试规划失败时，应首先确认：

```text
传感器坐标系
相机外参
时间同步
深度尺度
原始占据地图
膨胀占据地图
```

是否正确，再继续调整 A* 和轨迹优化参数。
