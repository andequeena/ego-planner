# `local_sensing` 功能包详细学习说明

> 适用源码：`/home/yxc/Desktop/ego-planner_ws/src/uav_simulator/local_sensing`
>
> ROS 功能包名称：`local_sensing_node`
>
> 所属系统：EGO-Planner 仿真环境

---

## 1. 一句话认识这个包

`local_sensing` 用全局障碍物点云和无人机里程计，模拟无人机在当前位置能够获得的局部传感器数据。

当前功能包有两条实现路径：

```text
默认 CPU 路径
全局点云 + 无人机位姿
        |
        v
KD 树半径搜索
        |
        v
距离、垂直范围和朝向过滤
        |
        v
世界坐标系局部点云


可选 CUDA 路径
全局点云 + 无人机位姿 + 相机内参
        |
        v
世界坐标变换到相机坐标
        |
        v
GPU 投影与深度缓冲
        |
        v
深度图、彩色深度图、相机位姿和反投影点云
```

需要首先明确：

```cmake
set(ENABLE_CUDA false)
```

所以当前默认编译和运行的是 CPU 点云裁剪版本，不是完整的深度相机渲染版本。

---

## 2. 它在 EGO-Planner 中的位置

默认仿真数据链路可以简化为：

```text
map_generator
生成完整障碍物点云
        |
        | /map_generator/global_cloud
        v
local_sensing_node/pcl_render_node
根据无人机当前位置裁剪局部可见点云
        |
        | /pcl_render_node/cloud
        v
plan_env::GridMap
更新概率占据地图和膨胀地图
        |
        v
plan_manage + bspline_opt
规划和优化局部轨迹
        |
        v
traj_server + 控制器
```

该包位于：

```text
理想完整地图
        和
规划器实际接收的局部观测
```

之间。

它的作用不是规划轨迹，也不是维护占据地图，而是模拟传感器有限的：

- 感知距离；
- 水平视场；
- 垂直视场；
- 相机投影；
- 遮挡关系。

其中 CPU 版本只做近似几何裁剪，CUDA 版本才进行像素级深度渲染。

---

## 3. 目录结构

```text
local_sensing/
├── CMakeLists.txt
├── package.xml
├── CMakeModules/
│   ├── FindCUDA.cmake
│   ├── FindEigen.cmake
│   └── FindCUDA/
├── cfg/
│   └── local_sensing_node.cfg
├── params/
│   └── camera.yaml
└── src/
    ├── pointcloud_render_node.cpp
    ├── pcl_render_node.cpp
    ├── depth_render.cu
    ├── depth_render.cuh
    ├── device_image.cuh
    ├── cuda_exception.cuh
    ├── helper_math.h
    ├── euroc.cpp
    ├── AlignError.h
    ├── ceres_extensions.h
    ├── csv_convert.py
    ├── empty.cpp
    └── empty.h
```

核心文件职责：

| 文件 | 职责 | 默认是否参与构建 |
|---|---|---|
| `pointcloud_render_node.cpp` | CPU 局部点云模拟 | 是 |
| `pcl_render_node.cpp` | CUDA 模式的 ROS 节点、位姿处理和消息发布 | 否 |
| `depth_render.cu` | GPU 点云投影、Z-buffer 和深度图生成 | 否 |
| `depth_render.cuh` | `DepthRender` 和相机参数定义 | 否 |
| `device_image.cuh` | CUDA 二维 pitched memory 封装 | 否 |
| `cuda_exception.cuh` | CUDA 异常类型 | 否 |
| `helper_math.h` | CUDA 向量数学辅助函数 | 否 |
| `camera.yaml` | CUDA 相机模型参数 | 启动时会加载 |
| `local_sensing_node.cfg` | 动态重配置参数声明 | 默认不生成 |
| `euroc.cpp` | 旧的 EuRoC/相机标定实验程序 | 否 |
| `AlignError.h` | Ceres 外参对齐残差 | 否 |
| `ceres_extensions.h` | Ceres 四元数辅助实现 | 否 |
| `csv_convert.py` | 旧数据集格式转换脚本 | 否 |

当前真正需要优先学习的是：

```text
CMakeLists.txt
pointcloud_render_node.cpp
pcl_render_node.cpp
depth_render.cuh
depth_render.cu
device_image.cuh
plan_manage/launch/simulator.xml
```

---

## 4. 功能包名、工程名和节点名

这里存在三个容易混淆的名字：

| 类型 | 名称 |
|---|---|
| 源码目录 | `local_sensing` |
| ROS package | `local_sensing_node` |
| CMake project | `local_sensing_node` |
| 可执行文件 | `pcl_render_node` |
| ROS 节点名 | `pcl_render_node` |

因此运行时使用：

```bash
rosrun local_sensing_node pcl_render_node
```

而不是：

```bash
rosrun local_sensing pcl_render_node
```

---

## 5. 构建模式与产物

### 5.1 默认 CPU 模式

当前：

```cmake
set(ENABLE_CUDA false)
```

进入：

```cmake
else(ENABLE_CUDA)
```

并构建：

```cmake
add_executable(
  pcl_render_node
  src/pointcloud_render_node.cpp
)
```

构建结果大致为：

```text
devel/lib/local_sensing_node/pcl_render_node
```

这个可执行文件：

- 接收全局点云；
- 接收里程计；
- 构建 KD 树；
- 周期性裁剪局部点云；
- 发布 `/pcl_render_node/cloud`。

### 5.2 可选 CUDA 模式

手动修改为：

```cmake
set(ENABLE_CUDA true)
```

会构建：

```cmake
CUDA_ADD_LIBRARY(
  depth_render_cuda
  src/depth_render.cu
)

add_executable(
  pcl_render_node
  src/pcl_render_node.cpp
)
```

构建产物包括：

```text
libdepth_render_cuda.so
pcl_render_node
```

此时同名 `pcl_render_node` 的内部实现完全不同。

### 5.3 两种模式不能同时生成同名节点

CMake 使用条件分支，所以一次构建只选择一种：

```text
ENABLE_CUDA = false -> pointcloud_render_node.cpp
ENABLE_CUDA = true  -> pcl_render_node.cpp + depth_render.cu
```

这也是阅读本包时最容易误解的地方：

```text
源码中虽然有深度图发布代码，
当前默认生成的节点却不会发布深度图。
```

---

## 6. 关键依赖

### 6.1 CPU 模式

| 依赖 | 用途 |
|---|---|
| `roscpp` | 节点、订阅、发布和定时器 |
| `nav_msgs` | 接收 `Odometry` |
| `sensor_msgs` | 接收和发布 `PointCloud2` |
| `pcl_ros` / PCL | 点云转换、体素滤波和 KD 树 |
| `Eigen3` | 四元数、旋转矩阵和三维向量 |

### 6.2 CUDA 模式

在 CPU 依赖之外还需要：

| 依赖 | 用途 |
|---|---|
| CUDA Runtime | GPU 内存、kernel 和原子最小值 |
| OpenCV | 深度矩阵与伪彩色图 |
| `cv_bridge` | OpenCV 图像与 ROS Image 转换 |
| `image_transport` | 图像相关依赖 |
| `geometry_msgs` | 发布相机位姿 |
| `quadrotor_msgs` | 当前源码中没有实际使用 |
| `dynamic_reconfigure` | 生成配置代码，但节点没有实际创建 server |

---

## 7. ROS 接口总览

### 7.1 默认 CPU 版本

私有订阅话题：

| 源码名 | 类型 | 默认完整名称 | 用途 |
|---|---|---|---|
| `~global_map` | `sensor_msgs/PointCloud2` | `/pcl_render_node/global_map` | 完整静态地图 |
| `~local_map` | `sensor_msgs/PointCloud2` | `/pcl_render_node/local_map` | 预留局部增量地图 |
| `~odometry` | `nav_msgs/Odometry` | `/pcl_render_node/odometry` | 无人机位置和朝向 |

发布话题：

| 话题 | 类型 | 用途 |
|---|---|---|
| `/pcl_render_node/cloud` | `sensor_msgs/PointCloud2` | 世界坐标系局部感知点云 |

注意发布话题在源码中以 `/` 开头，是绝对话题：

```cpp
nh.advertise<sensor_msgs::PointCloud2>(
    "/pcl_render_node/cloud", 10);
```

### 7.2 CUDA 版本

订阅话题与 CPU 版本相同。

私有发布话题：

| 源码名 | 默认完整名称 | 类型 | 用途 |
|---|---|---|---|
| `~depth` | `/pcl_render_node/depth` | `sensor_msgs/Image` | `32FC1` 米制深度图 |
| `~colordepth` | `/pcl_render_node/colordepth` | `sensor_msgs/Image` | 伪彩色深度图 |
| `~camera_pose` | `/pcl_render_node/camera_pose` | `geometry_msgs/PoseStamped` | 世界系相机位姿 |
| `~rendered_pcl` | `/pcl_render_node/rendered_pcl` | `sensor_msgs/PointCloud2` | 深度图反投影点云 |

CUDA 版本没有发布：

```text
/pcl_render_node/cloud
```

而是发布：

```text
/pcl_render_node/rendered_pcl
```

因此仅切换 `ENABLE_CUDA` 后，默认 `advanced_param.xml` 的点云输入重映射并不会自动改到 `rendered_pcl`。

CUDA 模式通常应使用：

```text
depth + camera_pose
```

这条地图输入链路，或者显式重新映射 `rendered_pcl`。

---

## 8. 默认 launch 中的真实连接

`plan_manage/launch/simulator.xml` 启动：

```xml
<node pkg="local_sensing_node"
      type="pcl_render_node"
      name="pcl_render_node">
```

并设置：

```xml
<param name="sensing_horizon" value="5.0"/>
<param name="sensing_rate" value="30.0"/>
<param name="estimation_rate" value="30.0"/>
```

主要重映射：

```xml
<remap from="~global_map"
       to="/map_generator/global_cloud"/>

<remap from="~odometry"
       to="$(arg odometry_topic)"/>
```

于是默认 CPU 链路是：

```text
/map_generator/global_cloud
        |
        v
/pcl_render_node/global_map

/visual_slam/odom
        |
        v
/pcl_render_node/odometry

/pcl_render_node/cloud
        |
        v
/grid_map/cloud
```

`advanced_param.xml` 中：

```xml
<remap from="/grid_map/cloud"
       to="$(arg cloud_topic)"/>
```

而 `run_in_sim.launch` 和 `simple_run.launch` 将：

```xml
<arg name="cloud_topic"
     value="/pcl_render_node/cloud"/>
```

所以当前默认规划器实际使用的是 CPU 点云接口。

---

## 9. CPU 版本全局状态

`pointcloud_render_node.cpp` 使用大量文件级全局变量。

关键数据：

| 变量 | 含义 |
|---|---|
| `_cloud_all_map` | 下采样后的完整全局地图 |
| `_local_map` | 当前筛选出的局部点云 |
| `_kdtreeLocalMap` | 全局地图 KD 树 |
| `_odom` | 最近一次无人机里程计 |
| `has_global_map` | 是否已经接收全局地图 |
| `has_odom` | 是否已经接收里程计 |
| `sensing_horizon` | 感知半径 |
| `sensing_rate` | 配置的感知频率 |
| `_pointIdxRadiusSearch` | KD 树半径搜索结果索引 |
| `_pointRadiusSquaredDistance` | 搜索结果平方距离 |

节点只有在：

```cpp
has_global_map && has_odom
```

都为真时才会发布局部点云。

---

## 10. CPU 版本初始化流程

`main()` 的核心流程：

```text
初始化 ROS 节点
        |
        v
读取 sensing_horizon、sensing_rate 等参数
        |
        v
订阅 global_map、local_map、odometry
        |
        v
创建 /pcl_render_node/cloud 发布器
        |
        v
创建局部感知定时器
        |
        v
100 Hz 执行 ros::spinOnce()
```

节点名：

```cpp
ros::init(argc, argv, "pcl_render");
```

但 launch 中显式指定：

```xml
name="pcl_render_node"
```

所以实际运行名称以 launch 设置为准。

---

## 11. 全局点云回调

入口：

```cpp
rcvGlobalPointCloudCallBack(...)
```

### 11.1 只接收第一帧

```cpp
if (has_global_map) return;
```

收到第一帧后：

```cpp
has_global_map = true;
```

以后所有全局点云都会被忽略。

这说明当前设计假设：

```text
global_map 是一次发布或内容固定的静态地图
```

如果地图源持续变化，本节点不会更新 KD 树。

### 11.2 ROS 点云转 PCL

```cpp
pcl::fromROSMsg(pointcloud_map, cloud_input);
```

得到：

```cpp
pcl::PointCloud<pcl::PointXYZ>
```

当前只保留 XYZ，不使用颜色、强度、法向量等字段。

### 11.3 体素下采样

固定叶子尺寸：

```cpp
_voxel_sampler.setLeafSize(0.1f, 0.1f, 0.1f);
```

意义：

```text
每个约 0.1 m × 0.1 m × 0.1 m 体素
只保留一个代表点
```

优点：

- 降低点数；
- 减少 KD 树内存；
- 加快半径搜索；
- 降低向规划地图发送的数据量。

代价：

- 小于 0.1 m 的几何细节可能丢失；
- 参数写死，无法从 launch 调整；
- 与 `GridMap` 分辨率存在隐式耦合。

### 11.4 构建 KD 树

```cpp
_kdtreeLocalMap.setInputCloud(
    _cloud_all_map.makeShared());
```

KD 树只构建一次，之后每一帧感知都重复利用。

这比每帧遍历完整地图更高效。

---

## 12. 里程计回调

入口：

```cpp
rcvOdometryCallbck(...)
```

当前 CPU 版本只执行：

```cpp
has_odom = true;
_odom = odom;
```

即保存最近一次里程计。

没有：

- 时间同步；
- 坐标变换查询；
- 位姿插值；
- 里程计有效性检查；
- 四元数归一化检查。

定时器触发时直接使用“最近收到的一帧”。

---

## 13. CPU 局部感知完整流程

入口：

```cpp
renderSensedPoints(...)
```

完整过程：

```text
检查是否已有全局地图和里程计
        |
        v
从里程计四元数计算旋转矩阵
        |
        v
提取机体 x 轴在世界系中的方向
        |
        v
以无人机位置为球心做 KD 树半径搜索
        |
        v
过滤垂直范围外的点
        |
        v
过滤机体后方和侧后方的点
        |
        v
发布世界坐标系局部点云
```

---

## 14. 无人机前向方向

源码将里程计四元数转换为旋转矩阵：

```cpp
Eigen::Matrix3d rot;
rot = q;
```

然后取：

```cpp
Eigen::Vector3d yaw_vec = rot.col(0);
```

旋转矩阵第一列表示：

```text
机体 x 轴在世界坐标系中的方向
```

因此本实现假设无人机或传感器的主要观察方向是：

```text
body x 正方向
```

变量名叫 `yaw_vec`，但它不只是平面 yaw：

- 它保留完整三维姿态；
- 如果无人机有 pitch 或 roll，向量也会倾斜。

---

## 15. KD 树半径搜索

搜索中心：

```cpp
pcl::PointXYZ searchPoint(
    odom_x, odom_y, odom_z);
```

搜索接口：

```cpp
_kdtreeLocalMap.radiusSearch(
    searchPoint,
    sensing_horizon,
    indices,
    squared_distances);
```

所以第一层感知范围是以无人机为球心、半径为：

```text
sensing_horizon
```

的球体。

默认 launch：

```text
sensing_horizon = 5.0 m
```

即先找出无人机周围 5 m 内的全部障碍点。

---

## 16. CPU 垂直范围过滤

源码：

```cpp
if ((fabs(pt.z - odom_z) / sensing_horizon)
      > tan(M_PI / 6.0))
  continue;
```

等价于：

```text
|点高度 - 无人机高度|
    <= sensing_horizon × tan(30°)
```

默认 `sensing_horizon = 5 m` 时：

```text
允许垂直差约为 ±2.887 m
```

需要注意，这不是严格的相机垂直视场角判断。

严格仰角通常应使用：

```text
|dz| / 水平距离
```

当前实现使用固定的：

```text
|dz| / sensing_horizon
```

所以它实际上构造了一个固定高度的水平带状区域，再与半径球相交。

---

## 17. CPU 水平前向过滤

对每个候选点构造：

```cpp
pt_vec = point - drone_position;
```

然后判断：

```cpp
if (pt_vec.normalized().dot(yaw_vec) < 0.5)
  continue;
```

点积关系：

```text
normalized(pt_vec) · yaw_vec = cos(theta)
```

保留条件：

```text
cos(theta) >= 0.5
```

所以：

```text
theta <= 60°
```

即以机体 x 轴为中心，形成半顶角约 60° 的前向圆锥：

```text
总水平张角近似 120°
```

组合全部过滤后，CPU 观测区域近似为：

```text
半径球
∩ 固定垂直高度带
∩ 前向 120° 圆锥
```

---

## 18. CPU 版本并不处理遮挡

假设同一方向上有：

```text
近处墙面
远处墙面
```

真实相机或激光雷达通常只能看到近处表面。

CPU 版本只检查：

- 距离；
- 高度；
- 朝向。

只要远处点仍满足这些条件，就会被一起发布。

因此它不模拟：

- 近物遮挡远物；
- 像素分辨率；
- 深度量化；
- 相机内参；
- 视锥边界；
- 传感器噪声；
- 漏检和误检。

它更准确的定位是：

```text
局部前向点云裁剪器
```

而不是严格的物理传感器模型。

---

## 19. CPU 点云发布

筛选完成后：

```cpp
_local_map.width = _local_map.points.size();
_local_map.height = 1;
_local_map.is_dense = true;
```

说明输出是无组织点云：

```text
height = 1
```

转换为 ROS 消息：

```cpp
pcl::toROSMsg(_local_map, _local_map_pcd);
```

并设置：

```cpp
_local_map_pcd.header.frame_id = "map";
```

输出点坐标没有变换到机体坐标系。

它们仍然是输入全局地图中的坐标，因此：

```text
输出是世界/地图坐标系局部点云
```

这正符合 `GridMap::cloudCallback()` 的使用方式。

---

## 20. 感知频率

CPU 版本定时器周期：

```cpp
double sensing_duration =
    1.0 / sensing_rate * 2.5;
```

所以真实发布频率约为：

```text
实际频率 = sensing_rate / 2.5
```

默认：

```text
sensing_rate = 30 Hz
```

得到：

```text
实际定时器频率约 12 Hz
```

这与参数名直觉不一致。

CUDA 版本则使用：

```cpp
1.0 / sensing_rate
```

所以相同参数在两种实现中的实际含义不同。

---

## 21. CUDA 版本总体流程

CUDA 节点的核心链路：

```text
接收全局地图点云
        |
        v
复制为连续 float3 数组
        |
        v
上传到 GPU
        |
        v
接收无人机 odometry
        |
        v
body pose × camera-to-body 外参
        |
        v
得到 camera-to-world
        |
        v
取逆得到 world-to-camera
        |
        v
GPU 将每个世界点变换并投影到图像
        |
        v
atomicMin 生成最近深度
        |
        +----------------------+
        |                      |
        v                      v
发布 32FC1 深度图       反投影为世界系局部点云
```

---

## 22. CUDA 相机模型

`params/camera.yaml`：

```yaml
cam_width:  640
cam_height: 480
cam_fx:     387.229248046875
cam_fy:     387.229248046875
cam_cx:     321.04638671875
cam_cy:     243.44969177246094
```

含义：

| 参数 | 含义 |
|---|---|
| `width` | 图像宽度 |
| `height` | 图像高度 |
| `fx` | x 方向焦距，单位为像素 |
| `fy` | y 方向焦距，单位为像素 |
| `cx` | 主点横坐标 |
| `cy` | 主点纵坐标 |

针孔投影：

```text
u = fx × Xc / Zc + cx
v = fy × Yc / Zc + cy
```

其中：

```text
(Xc, Yc, Zc)
```

是点在相机坐标系中的坐标。

---

## 23. 相机与机体坐标变换

源码固定外参：

```cpp
cam02body <<
   0,  0,  1, 0,
  -1,  0,  0, 0,
   0, -1,  0, 0,
   0,  0,  0, 1;
```

然后：

```cpp
cam2world = body_pose * cam02body;
```

从矩阵乘法含义看：

```text
T_world_camera =
    T_world_body × T_body_camera
```

虽然变量名 `cam02body` 不够规范，但它在这里被当作：

```text
相机坐标到机体坐标的变换
```

当前平移全为 0，所以相机光心与机体原点重合，只存在轴方向转换。

---

## 24. 为什么渲染时要取逆矩阵

地图点存储在世界坐标系。

投影需要相机坐标：

```text
P_camera = T_camera_world × P_world
```

而已有：

```text
T_world_camera = cam2world
```

所以：

```cpp
Matrix4d cam_pose = cam2world.inverse();
```

得到：

```text
T_camera_world
```

再将前三行旋转和平移复制到 `Parameter`：

```cpp
parameter.r[3][3]
parameter.t[3]
```

---

## 25. `DepthRender` 类

公开接口：

```cpp
class DepthRender
{
public:
  void set_para(...);
  void set_data(vector<float>& cloud_data);
  void render_pose(double* transformation,
                   int* host_ptr);
};
```

典型使用顺序：

```text
set_para()
设置相机内参与图像尺寸
        |
        v
set_data()
上传全局地图到 GPU
        |
        v
render_pose()
针对每个相机位姿渲染一帧深度图
```

关键成员：

| 成员 | 含义 |
|---|---|
| `cloud_size` | 地图点数 |
| `host_cloud_ptr` | CPU 端 `float3` 点数组 |
| `dev_cloud_ptr` | GPU 端点数组 |
| `parameter` | CPU 端相机与位姿参数 |
| `parameter_devptr` | GPU 端参数 |
| `has_devptr` | 是否已分配 GPU 数据 |

---

## 26. 地图上传到 GPU

输入 `cloud_data` 的布局：

```text
x0, y0, z0,
x1, y1, z1,
...
```

`set_data()` 将其整理为：

```cpp
float3
```

数组：

```cpp
host_cloud_ptr[i] =
    make_float3(x, y, z);
```

然后：

```cpp
cudaMalloc(&dev_cloud_ptr, ...);
cudaMemcpy(..., cudaMemcpyHostToDevice);
```

参数结构也被复制到 GPU：

```cpp
cudaMalloc(&parameter_devptr, sizeof(Parameter));
cudaMemcpy(parameter_devptr, &parameter, ...);
```

之后每帧只更新旋转和平移，不必重新上传静态全局地图。

---

## 27. CUDA 深度图内存

每次 `render_pose()` 创建：

```cpp
DeviceImage<int> depth_output(width, height);
```

`DeviceImage` 使用：

```cpp
cudaMallocPitch(...)
```

分配二维 pitched memory。

相比普通连续分配，pitch 会让每一行满足 GPU 对齐要求：

```text
真实行跨度 pitch
    >= width × sizeof(ElementType)
```

因此索引不能简单写成：

```text
y × width + x
```

而要使用：

```cpp
data[y * stride + x]
```

`DeviceImage::atXY()` 封装了这一逻辑。

---

## 28. 深度缓冲初始化

CUDA kernel：

```cpp
depth_initial<<<...>>>(...);
```

将每个像素初始化为：

```text
999999 mm
```

这是一个很大的深度值，用于后续：

```cpp
atomicMin(...)
```

没有任何点投影到的像素会保持该值。

拷回 CPU 后：

```cpp
999999 / 1000 = 999.999 m
```

随后因为大于 500 m 被改成 0，所以最终无观测像素为：

```text
0.0 m
```

---

## 29. GPU 点变换

每个 CUDA 线程处理一个地图点。

线程索引：

```cpp
index = threadIdx.x + blockIdx.x * blockDim.x;
```

世界点转换到相机坐标：

```text
P_camera = R × P_world + t
```

源码逐项计算：

```cpp
trans_point.x = ...
trans_point.y = ...
trans_point.z = ...
```

若：

```cpp
trans_point.z <= 0
```

说明点在相机后方或成像平面上，直接丢弃。

---

## 30. GPU 投影与图像边界

投影：

```cpp
projected.x =
    Xc / Zc * fx + cx + 0.5;

projected.y =
    Yc / Zc * fy + cy + 0.5;
```

`+0.5` 用于近似四舍五入到整数像素。

若像素不在：

```text
0 <= u < width
0 <= v < height
```

则丢弃。

这使 CUDA 版本具有由相机内参决定的真实矩形视锥，而不是 CPU 版本的近似前向圆锥。

---

## 31. 深度定义

源码：

```cpp
float dist = trans_point.z;
```

所以深度是：

```text
相机光轴方向 Z 深度
```

不是点到相机光心的欧氏距离：

```text
sqrt(X² + Y² + Z²)
```

这与常见针孔深度图定义一致，也与后续反投影公式匹配。

深度转换为整数毫米：

```cpp
int dist_mm = dist * 1000.0f + 0.5f;
```

因此 GPU 深度缓冲使用：

```text
整数毫米
```

便于执行原子最小值。

---

## 32. 点扩张与孔洞减少

一个稀疏地图点只投到一个像素，会产生大量孔洞。

源码根据深度计算扩张半径：

```cpp
int r = 0.0573 * fx / dist + 0.5f;
```

然后将该点写入：

```text
(2r + 1) × (2r + 1)
```

的像素方块。

因为：

```text
r ∝ 1 / depth
```

所以：

- 近处点覆盖更多像素；
- 远处点覆盖更少像素。

常数 `0.0573` 可以理解为对地图点实际表面尺寸的经验近似。

该方法并不是真正的三角网格光栅化，只是一种点云 splatting。

---

## 33. `atomicMin` 与遮挡

多个点可能投影到同一个像素。

源码：

```cpp
atomicMin(dist_ptr, dist_mm);
```

最终保存最小深度：

```text
离相机最近的点
```

这相当于一个简化的 Z-buffer。

因此 CUDA 版本可以模拟：

```text
近处物体遮挡远处物体
```

这也是它与 CPU 版本最本质的区别之一。

---

## 34. 深度图发布

GPU 深度拷回 CPU：

```cpp
depth_output.getDevData(host_ptr);
```

随后从整数毫米转换为浮点米：

```cpp
depth = depth_hostptr[...] / 1000.0f;
```

大于等于 500 m 的值被设为 0。

最终发布：

```text
编码：TYPE_32FC1
单位：m
无效值：0
frame_id：camera
时间戳：最近里程计时间
```

这与 `plan_env::GridMap` 的深度输入约定相符：

```text
32 位浮点单通道深度图
```

---

## 35. 彩色深度图

CUDA 节点还将深度映射到 8 位图：

```cpp
depth_mat.convertTo(
    adjMap, CV_8UC1, 255 / 13.0, -min);
```

再使用：

```cpp
cv::applyColorMap(
    adjMap,
    falseColorsMap,
    cv::COLORMAP_RAINBOW);
```

发布：

```text
/pcl_render_node/colordepth
```

该图主要用于可视化，不用于规划地图融合。

当前颜色缩放固定使用约 13 m 范围，并没有根据：

```text
sensing_horizon
```

自动调整。

---

## 36. 深度图反投影为点云

`render_pcl_world()` 遍历每个有效深度像素。

从像素坐标和深度恢复相机坐标：

```text
Xc = (u - cx) × depth / fx
Yc = (v - cy) × depth / fy
Zc = depth
```

再使用：

```cpp
pose_in_world = cam2world * pose_in_camera;
```

转换回世界坐标。

最后进行球形距离限制：

```cpp
if ((point_world - drone_position).norm()
      > sensing_horizon)
  continue;
```

发布：

```text
/pcl_render_node/rendered_pcl
```

其特点：

- 位于世界坐标系；
- 已经过相机视锥和遮挡处理；
- 一个有效像素最多产生一个点；
- 点数上限约为 `width × height`。

---

## 37. 相机位姿发布

`pubCameraPose()` 由独立定时器调用。

发布内容：

```text
position    = cam2world 平移
orientation = cam2world 旋转
frame_id    = /map
stamp       = 最近里程计时间
```

频率：

```text
estimation_rate
```

默认：

```text
30 Hz
```

在 CUDA 深度模式下，`GridMap` 可以同步订阅：

```text
depth + camera_pose
```

完成深度图反投影和概率地图更新。

---

## 38. `GridMap` 如何使用本包输出

`GridMap::initMap()` 同时创建：

```text
/grid_map/depth
/grid_map/pose 或 /grid_map/odom
/grid_map/cloud
/grid_map/odom
```

两类输入链路都存在：

### 点云链路

```text
local_sensing CPU cloud
        |
        v
GridMap::cloudCallback()
        |
        v
局部概率占据更新
```

### 深度链路

```text
local_sensing CUDA depth
        +
camera_pose / odometry
        |
        v
时间同步
        |
        v
GridMap::depthPoseCallback()
或 depthOdomCallback()
```

默认配置：

```xml
<param name="grid_map/pose_type" value="2"/>
```

表示深度图使用 odometry 同步路径。

但默认 CPU 节点不发布深度图，因此实际起作用的是独立点云链路。

---

## 39. 参数说明

| 参数 | 默认 launch 值 | CPU 使用 | CUDA 使用 |
|---|---:|---:|---:|
| `sensing_horizon` | `5.0` | 是 | 是 |
| `sensing_rate` | `30.0` | 是 | 是 |
| `estimation_rate` | `30.0` | 读取但不使用 | 是 |
| `map/x_size` | launch 参数 | 仅用于未实际使用的网格函数 | 同左 |
| `map/y_size` | launch 参数 | 同上 | 同上 |
| `map/z_size` | launch 参数 | 同上 | 同上 |
| `cam_width` | `640` | 否 | 是 |
| `cam_height` | `480` | 否 | 是 |
| `cam_fx` | `387.229...` | 否 | 是 |
| `cam_fy` | `387.229...` | 否 | 是 |
| `cam_cx` | `321.046...` | 否 | 是 |
| `cam_cy` | `243.449...` | 否 | 是 |

参数全部通过：

```cpp
nh.getParam(...)
```

读取。

`getParam()` 没有提供默认值，且返回值没有检查。

如果参数缺失，对应全局变量可能保持零初始化值，进而导致：

- 除零；
- 空图像；
- 定时器周期非法；
- 相机投影异常。

---

## 40. `local_map` 输入

CPU 版本回调：

```cpp
void rcvLocalPointCloudCallBack(...)
{
  // do nothing, fix later
}
```

所以 CPU 模式下：

```text
~local_map 完全不起作用
```

CUDA 版本会把每次收到的局部点云继续追加到：

```cpp
cloud_data
```

然后重新上传全部数据。

`map_generator` 中存在：

```text
/pcl_render_node/local_map
```

发布接口，但默认点击地图逻辑被注释，且 CPU 回调为空。

因此它属于未完成或遗留接口。

---

## 41. 动态重配置文件

`cfg/local_sensing_node.cfg` 定义：

```text
tx, ty, tz
axis_x, axis_y, axis_z
```

看起来原本用于在线调整传感器外参。

但是：

- 默认 CPU 分支不调用 `generate_dynamic_reconfigure_options()`；
- CUDA 分支虽然生成配置代码；
- `pcl_render_node.cpp` 没有创建 `dynamic_reconfigure::Server`；
- 没有注册配置回调；
- 参数也没有应用到 `cam02body`。

所以当前动态重配置文件没有实际运行效果。

---

## 42. 旧实验文件

### 42.1 `euroc.cpp`

这是一个旧的相机、Vicon、地图深度渲染和 PnP 对齐实验程序。

它包含：

- 图像和位姿近似时间同步；
- 地面真值文件读取；
- 深度图渲染；
- 鼠标选取 2D/3D 对应点；
- `solvePnP()` 外参估计；
- 图像与深度伪彩色叠加。

当前 CMake 没有构建它，并且它依赖：

```text
cloud_banchmark
```

及硬编码数据路径，所以不属于 EGO-Planner 默认运行链路。

### 42.2 `AlignError.h`

定义一个 Ceres 自动求导残差，用于对齐：

```text
相机位姿
激光雷达位姿
世界坐标变换
雷达到相机外参
```

当前没有被任何构建目标包含。

### 42.3 `ceres_extensions.h`

提供 Eigen 四元数局部参数化和四元数数学函数。

同样不参与当前节点构建。

### 42.4 `csv_convert.py`

用于将 EuRoC CSV 真值转换为逐项文本格式。

脚本包含开发者机器上的绝对路径，不能直接复用。

---

## 43. CPU 与 CUDA 版本对比

| 特性 | CPU 版本 | CUDA 版本 |
|---|---|---|
| 默认启用 | 是 | 否 |
| 输入全局点云 | 是 | 是 |
| 使用里程计位置 | 是 | 是 |
| 使用完整姿态 | 前向过滤 | 相机外参和投影 |
| KD 树半径搜索 | 是 | 否 |
| 相机内参 | 否 | 是 |
| 严格矩形视锥 | 否 | 是 |
| 遮挡处理 | 否 | 是 |
| 深度图 | 否 | 是 |
| 彩色深度图 | 否 | 是 |
| 相机位姿 | 否 | 是 |
| 世界系局部点云 | `/cloud` | `/rendered_pcl` |
| GPU 要求 | 无 | NVIDIA CUDA |
| 地图预处理 | 0.1 m 体素滤波 | 原始点直接上传 |
| 主要优点 | 简单、稳定、部署方便 | 传感器模型更接近深度相机 |
| 主要缺点 | 会“看穿”障碍物 | 构建和内存管理更复杂 |

---

## 44. 当前源码中的重要风险与注意事项

以下结论针对当前工作空间源码。

### 44.1 `_resolution` 从未读取或赋值

CPU 和 CUDA 节点都声明：

```cpp
double _resolution, _inv_resolution;
```

随后：

```cpp
_inv_resolution = 1.0 / _resolution;
```

但没有：

```cpp
nh.getParam("map/resolution", _resolution);
```

全局变量会被零初始化，所以这里实际发生：

```text
1.0 / 0.0
```

浮点结果通常为无穷大。

随后 `_GLX_SIZE` 等计算也会异常。

当前主流程之所以通常还能运行，是因为：

```text
gridIndex2coord()
coord2gridIndex()
```

在实际感知逻辑中没有被调用。

这仍然是明确的无效初始化，应删除整段遗留网格代码，或正确读取分辨率。

### 44.2 CPU 输出点云没有设置时间戳

CPU 版本只设置：

```cpp
header.frame_id = "map";
```

没有设置：

```cpp
header.stamp
```

因此消息时间戳通常为 0。

当前 `GridMap::cloudCallback()` 主要使用最近 odometry，可能仍能工作，但：

- 无法准确分析感知延迟；
- 无法进行严格时间同步；
- rosbag 回放和多传感器融合更难；
- 调试工具看到的时间信息不完整。

建议使用生成该局部点云时对应的 odometry 时间戳。

### 44.3 CPU `local_map` 回调为空

节点订阅了 `local_map`，但数据被完全忽略。

接口存在会让使用者误以为支持增量障碍物更新。

### 44.4 全局地图只接受第一帧

```cpp
if (has_global_map) return;
```

适合静态地图，但不支持：

- 动态地图源；
- 地图重新生成；
- 运行中加入新静态障碍；
- 多阶段地图加载。

### 44.5 CPU 模式不处理遮挡

远处障碍点可能穿过近处障碍被发布。

这会让规划器提前知道现实传感器看不到的地图区域，降低仿真的真实性。

### 44.6 CPU 垂直过滤不是严格视场角

当前使用：

```text
|dz| / sensing_horizon
```

而不是：

```text
|dz| / horizontal_distance
```

所以近处点和远处点共享相同高度限制。

### 44.7 CPU 感知频率被额外除以 2.5

参数填 30 Hz，实际约 12 Hz。

如果调参者不知道这个系数，会误判地图更新频率。

### 44.8 CPU 体素分辨率硬编码

```cpp
setLeafSize(0.1f, 0.1f, 0.1f);
```

无法根据：

- 地图分辨率；
- 障碍物尺度；
- 性能预算；
- 感知距离；

通过参数调整。

### 44.9 `getParam()` 返回值未检查

感知距离或频率缺失时，节点不会主动报错退出。

建议使用：

```cpp
nh.param(name, value, default_value);
```

或检查每个必需参数并输出明确错误。

### 44.10 坐标系名称不一致

工作区中同时出现：

```text
world
map
/map
```

CPU 输出：

```text
map
```

CUDA 相机位姿和点云：

```text
/map
```

全局地图源常使用：

```text
world
```

代码没有通过 TF 转换点云，只是修改 `frame_id`。

所以系统隐式假设：

```text
world 和 map 数值上是同一个坐标系
```

若未来两者不重合，必须进行真实坐标变换，而不是只改标签。

### 44.11 CUDA `set_data()` 重复调用会泄漏

每次 `set_data()` 都重新：

```cpp
malloc(host_cloud_ptr)
cudaMalloc(dev_cloud_ptr)
cudaMalloc(parameter_devptr)
```

但调用前没有释放旧内存。

全局地图只调用一次时问题不明显。

CUDA `local_map` 每来一帧都会再次调用，会持续泄漏：

- CPU 点云内存；
- GPU 点云内存；
- GPU 参数内存。

### 44.12 CUDA `cloud_data` 持续累积

局部点云回调不断：

```cpp
cloud_data.push_back(...)
```

从不清空，也不去重。

长期运行会导致：

- 地图点数持续增加；
- 重复点越来越多；
- 上传越来越慢；
- GPU 内存持续增加；
- 渲染耗时持续上升。

### 44.13 `depth_hostptr` 重复分配未释放

每次接收全局或局部地图都会：

```cpp
depth_hostptr =
    malloc(width * height * sizeof(int));
```

旧指针没有释放。

### 44.14 `CudaException::what()` 返回悬空指针

当前：

```cpp
std::stringstream description;
return description.str().c_str();
```

`description.str()` 产生临时 `std::string`。

函数返回后临时字符串被销毁，返回的 `const char*` 已失效。

正确做法是把完整错误文本保存为异常对象的成员字符串。

### 44.15 CUDA kernel 错误没有显式检查

调用：

```cpp
depth_initial<<<...>>>();
render<<<...>>>();
```

后没有立即检查：

```cpp
cudaGetLastError()
cudaDeviceSynchronize()
```

后续同步拷贝可能间接暴露错误，但日志位置不够准确。

### 44.16 每帧重新分配 GPU 深度图

`render_pose()` 每次创建和销毁：

```cpp
DeviceImage<int> depth_output(...)
```

这会在每一帧执行：

- `cudaMallocPitch`；
- `cudaMalloc`；
- 两次 `cudaFree`。

高频渲染时会产生不必要的分配开销和延迟抖动。

更合理的方式是在 `DepthRender` 初始化时分配一次并重复使用。

### 44.17 CUDA 架构写死为 `sm_75`

CMake：

```cmake
-gencode arch=compute_75,code=sm_75
```

只针对 Turing 级别的计算能力 7.5。

不同 GPU 可能：

- 无法运行；
- 无法发挥新架构能力；
- 编译器不再支持对应目标。

应根据实际 GPU 和 CUDA 版本配置。

### 44.18 同时指定 C++11 和 C++14

```cmake
ADD_COMPILE_OPTIONS(-std=c++11)
ADD_COMPILE_OPTIONS(-std=c++14)
```

通常后者生效，但建议只保留一个标准。

### 44.19 动态重配置声明但未使用

生成配置并不等于节点支持动态修改。

当前没有 server 和 callback，界面上的参数不会改变相机外参。

### 44.20 `package.xml` 与实际依赖不一致

包清单包含：

```text
svo_msgs
vikit_ros
```

当前主要节点并未使用。

同时现代 catkin 包通常应明确：

- `build_depend`；
- `exec_depend`；
- 或使用 package format 2。

依赖声明值得整理。

### 44.21 CPU 与 CUDA 输出接口不统一

两种实现编译出同名节点，但：

```text
CPU  -> /pcl_render_node/cloud
CUDA -> /pcl_render_node/rendered_pcl
```

切换构建选项可能导致规划器悄悄收不到点云。

### 44.22 无传感器噪声模型

即使 CUDA 模式考虑遮挡，它仍然没有模拟：

- 深度噪声随距离增长；
- 边缘飞点；
- 无反射区域；
- 随机漏测；
- 运动模糊；
- 时间延迟；
- 位姿误差。

因此它适合规划算法仿真，不应被当作完整的真实传感器数字孪生。

---

## 45. 推荐调试方法

### 45.1 确认实际构建的是哪种模式

检查：

```bash
grep ENABLE_CUDA \
  src/uav_simulator/local_sensing/CMakeLists.txt
```

当前应看到：

```text
set(ENABLE_CUDA false)
```

### 45.2 检查节点和话题

```bash
rosnode info /pcl_render_node
```

CPU 模式应看到发布：

```text
/pcl_render_node/cloud
```

CUDA 模式应看到：

```text
/pcl_render_node/depth
/pcl_render_node/colordepth
/pcl_render_node/camera_pose
/pcl_render_node/rendered_pcl
```

### 45.3 检查全局地图是否到达

```bash
rostopic hz /map_generator/global_cloud
rostopic echo -n 1 /map_generator/global_cloud/header
```

节点终端应输出：

```text
Global Pointcloud received..
```

### 45.4 检查里程计

```bash
rostopic hz /visual_slam/odom
rostopic echo -n 1 /visual_slam/odom/pose/pose
```

若没有 odometry，即使全局地图已经收到，也不会发布局部点云。

### 45.5 检查局部点云频率

```bash
rostopic hz /pcl_render_node/cloud
```

默认 CPU 模式预期约：

```text
12 Hz
```

不是 30 Hz。

### 45.6 在 RViz 中观察视场

显示：

```text
/map_generator/global_cloud
/pcl_render_node/cloud
```

为两者设置不同颜色。

重点观察：

- 局部点云是否在无人机 5 m 内；
- 无人机转向时前向扇区是否随之旋转；
- 后方点是否被过滤；
- 垂直范围是否符合预期；
- 是否出现穿透遮挡的远处点。

### 45.7 检查点云时间戳

```bash
rostopic echo -n 1 /pcl_render_node/cloud/header
```

当前 CPU 版本可能显示：

```text
stamp: 0
```

这属于源码现状，不一定表示消息没有发布。

### 45.8 CUDA 深度图检查

```bash
rostopic hz /pcl_render_node/depth
rostopic echo -n 1 /pcl_render_node/depth/encoding
```

应为：

```text
32FC1
```

可使用：

```text
rqt_image_view
```

查看：

```text
/pcl_render_node/colordepth
```

原始 `32FC1` 深度图在普通图像工具中可能显示较暗，伪彩色图更便于检查。

---

## 46. 常见故障

### 46.1 `/pcl_render_node/cloud` 没有数据

依次检查：

1. 是否默认 CPU 构建；
2. `/map_generator/global_cloud` 是否发布；
3. `~global_map` 重映射是否正确；
4. odometry 是否发布；
5. `sensing_rate` 是否大于 0；
6. 全局地图是否为空；
7. 无人机附近是否有满足视场条件的障碍点。

### 46.2 有全局地图但始终没有局部点

可能原因：

- 无人机朝向与地图障碍相反；
- 5 m 内没有点；
- 垂直高度差过大；
- odometry 四元数无效；
- 地图和 odometry 不在同一坐标系。

### 46.3 切换 CUDA 后规划器没有地图

原因通常是：

```text
规划器仍订阅 /pcl_render_node/cloud
CUDA 节点发布 /pcl_render_node/rendered_pcl
```

或者深度链路所需的：

```text
depth + pose/odom
```

没有正确同步。

### 46.4 CUDA 编译失败

检查：

- CUDA Toolkit 是否安装；
- 编译器版本是否兼容；
- GPU 架构是否为或支持 `sm_75`；
- OpenCV、cv_bridge 是否存在；
- CMake 的旧 `FindCUDA` 是否兼容当前 CUDA；
- catkin 是否能找到全部依赖。

### 46.5 CUDA 长时间运行后显存增长

重点检查：

```text
local_map 是否持续发布
DepthRender::set_data() 是否被重复调用
cloud_data 是否不断追加
```

当前实现对此确实存在内存增长风险。

---

## 47. 推荐改进方向

### 47.1 低风险修复

优先建议：

1. 为 CPU 点云设置 odometry 时间戳；
2. 删除未使用的网格转换变量，或正确读取 `map/resolution`；
3. 检查所有必需参数；
4. 将体素大小改为 ROS 参数；
5. 将 CPU 频率公式改为 `1.0 / sensing_rate`，或明确参数含义；
6. 统一 `map`、`world` 和 `/map`；
7. 删除空 `local_map` 订阅或补齐功能；
8. 清理无用 include 和包依赖。

### 47.2 CPU 视场模型改进

可以显式计算相机坐标：

```text
P_camera = R_camera_world ×
           (P_world - camera_position)
```

然后用：

```text
Z > 0
|X / Z| < tan(horizontal_fov / 2)
|Y / Z| < tan(vertical_fov / 2)
```

过滤。

这样即使不使用 CUDA，也能得到更准确的矩形视锥。

### 47.3 CPU 遮挡近似

可选方案：

- 按角度分桶，只保留每个方向最近点；
- 在低分辨率深度图上做 CPU Z-buffer；
- 使用 PCL range image；
- 对候选点做体素射线检查。

### 47.4 CUDA 内存复用

建议让 `DepthRender` 长期持有：

- GPU 深度图；
- CPU 输出缓冲；
- 参数缓冲；
- 地图点云缓冲。

地图更新时：

1. 若容量足够，复用旧内存；
2. 若容量不足，再扩容；
3. 上传前释放或覆盖旧数据；
4. 避免每帧 `cudaMalloc/cudaFree`。

### 47.5 统一两种模式接口

两种实现都应至少发布统一的：

```text
/pcl_render_node/cloud
```

可额外发布：

```text
depth
colordepth
camera_pose
```

这样切换 CPU/CUDA 不需要修改规划器 launch。

### 47.6 使用现代 CMake CUDA

旧的：

```cmake
find_package(CUDA)
CUDA_ADD_LIBRARY(...)
```

可以逐步迁移到：

```cmake
project(... LANGUAGES CXX CUDA)
add_library(... depth_render.cu)
```

并通过：

```cmake
CMAKE_CUDA_ARCHITECTURES
```

配置 GPU 架构。

---

## 48. 推荐源码阅读顺序

### 第一阶段：理解默认运行链路

阅读：

```text
CMakeLists.txt
plan_manage/launch/simulator.xml
plan_manage/launch/advanced_param.xml
```

先确认：

- 默认关闭 CUDA；
- 节点接收什么；
- `/pcl_render_node/cloud` 如何进入 `GridMap`。

### 第二阶段：理解 CPU 点云筛选

阅读：

```text
src/pointcloud_render_node.cpp
```

建议顺序：

1. `main()`
2. `rcvGlobalPointCloudCallBack()`
3. `rcvOdometryCallbck()`
4. `renderSensedPoints()`

重点理解：

- 体素下采样；
- KD 树半径搜索；
- `rot.col(0)`；
- 点积视场过滤；
- 输出仍在世界坐标系。

### 第三阶段：理解规划地图如何消费点云

阅读：

```text
planner/plan_env/src/grid_map.cpp
```

重点看：

```text
GridMap::initMap()
GridMap::cloudCallback()
GridMap::odomCallback()
GridMap::updateOccupancyCallback()
```

### 第四阶段：理解 CUDA 封装

阅读：

```text
src/depth_render.cuh
src/device_image.cuh
```

重点理解：

- `Parameter`；
- `DepthRender` 生命周期；
- pitched memory；
- CPU/GPU 数据复制。

### 第五阶段：理解 GPU 渲染

阅读：

```text
src/depth_render.cu
```

建议顺序：

1. `depth_initial()`
2. `render()`
3. `DepthRender::set_data()`
4. `DepthRender::render_pose()`

### 第六阶段：理解 ROS 图像输出

阅读：

```text
src/pcl_render_node.cpp
```

重点看：

```text
rcvOdometryCallbck()
render_currentpose()
render_pcl_world()
pubCameraPose()
```

### 第七阶段：最后再看遗留实验

阅读：

```text
euroc.cpp
AlignError.h
ceres_extensions.h
csv_convert.py
```

这些文件有助于理解代码来源和历史用途，但不是默认规划链路的必要知识。

---

## 49. 建议学习实验

### 实验一：观察 CPU 感知区域

在 RViz 同时显示：

```text
/map_generator/global_cloud
/pcl_render_node/cloud
无人机模型或 odometry
```

缓慢旋转无人机，验证局部点云是否形成约 120° 前向区域。

### 实验二：改变 `sensing_horizon`

尝试：

```text
3 m
5 m
8 m
```

记录：

- 局部点数；
- 发布耗时；
- `GridMap` 更新范围；
- 规划器对障碍物的提前感知距离。

注意当前垂直允许范围也会随 `sensing_horizon` 一起变化。

### 实验三：验证实际发布频率

分别设置：

```text
sensing_rate = 10
sensing_rate = 20
sensing_rate = 30
```

使用：

```bash
rostopic hz /pcl_render_node/cloud
```

验证实际频率约为参数的 `1 / 2.5`。

### 实验四：验证遮挡缺失

在同一观察方向布置两堵墙。

观察 CPU 局部点云中远墙是否仍然存在。

这可以直观看到：

```text
局部裁剪 != 深度相机渲染
```

### 实验五：修复时间戳

将输出：

```cpp
_local_map_pcd.header.stamp =
    _odom.header.stamp;
```

比较修改前后的：

```bash
rostopic echo /pcl_render_node/cloud/header
```

### 实验六：CUDA Z-buffer

启用 CUDA 后，在相同像素方向放置多个不同深度的点。

验证最终深度是否为最近点的：

```text
Z_camera
```

### 实验七：点扩张半径

修改：

```cpp
0.0573
```

观察：

- 深度图孔洞；
- 障碍边缘厚度；
- 远近物体的像素覆盖。

### 实验八：对比两种模式

在相同地图和轨迹下记录：

```text
CPU 点云数量
CUDA 反投影点云数量
地图融合耗时
规划成功率
遮挡区域差异
```

---

## 50. 常见问题

### Q1：这个包是真实传感器驱动吗？

不是。

它读取已经知道的完整地图，再根据无人机位姿生成局部观测，属于仿真感知节点。

### Q2：为什么默认不用 CUDA？

CPU 版本：

- 部署简单；
- 不要求 NVIDIA GPU；
- 编译依赖少；
- 对规划演示已经足够；
- 更适合不同开发环境。

代价是传感器真实性较低。

### Q3：CPU 版本是不是激光雷达模型？

也不完全是。

它有感知半径和前向视场，但没有激光扫描线、遮挡和量测噪声。

更准确地说，它是一个局部点云筛选模型。

### Q4：为什么输出点云是世界坐标系？

因为 `GridMap` 的独立点云输入会直接将点用于世界地图更新。

省去每帧在 `GridMap` 内根据传感器位姿转换点云。

### Q5：`sensing_horizon` 与规划距离是什么关系？

默认 launch 注释建议：

```text
planning_horizon ≈ 1.5 × sensing_horizon
```

当前：

```text
sensing_horizon = 5.0 m
planning_horizon = 7.5 m
```

规划轨迹可以略长于当前观测范围，但过长会增加在未知区域中规划的风险。

### Q6：CPU 版本为什么还读取相机参数？

它实际上不读取 `camera.yaml` 中的参数。

launch 会加载这些参数，但默认 CPU 源码没有调用对应 `getParam()`，所以它们只为 CUDA 版本预留。

### Q7：为什么 CMake 开启 CUDA 后节点名没有变化？

两种实现是同一功能的替代版本，因此都输出：

```text
pcl_render_node
```

但当前话题接口并未完全统一，这是需要注意的工程问题。

### Q8：`rendered_pcl` 是原始全局地图裁剪吗？

不是。

它先经过：

- 相机坐标变换；
- 像素投影；
- Z-buffer 遮挡；
- 深度图量化；
- 深度反投影；
- 感知半径过滤。

所以它更接近深度相机能够观察到的表面点。

### Q9：深度图中的值是欧氏距离吗？

不是。

它是相机坐标系的：

```text
Z
```

即光轴深度。

### Q10：为什么 CUDA 深度使用整数毫米？

因为 CUDA 对整数支持：

```cpp
atomicMin()
```

将米乘 1000 转为毫米整数后，可以高效实现像素级最近深度竞争。

---

## 51. 总结

`local_sensing` 是 EGO-Planner 仿真链路中的局部传感器模拟包。

它的核心任务是：

```text
不把完整世界地图直接交给规划器，
而是根据无人机当前位置和朝向，
生成有限范围内的局部障碍观测。
```

理解这个包时，最重要的六条主线是：

1. **ROS 包名是 `local_sensing_node`，目录名是 `local_sensing`。**
2. **默认 `ENABLE_CUDA=false`，实际运行的是 CPU 点云裁剪版本。**
3. **CPU 版本通过体素滤波、KD 树半径搜索和前向视场过滤生成 `/pcl_render_node/cloud`。**
4. **CPU 版本不处理遮挡，因此只是近似局部感知。**
5. **CUDA 版本通过世界到相机变换、针孔投影和 `atomicMin` Z-buffer 生成深度图。**
6. **输出最终进入 `plan_env::GridMap`，成为局部占据地图的传感器输入。**

默认 CPU 数据链路：

```text
全局地图
  -> 0.1 m 体素下采样
  -> KD 树
  -> 5 m 半径搜索
  -> 垂直范围过滤
  -> 前向 120° 过滤
  -> /pcl_render_node/cloud
  -> GridMap
```

CUDA 数据链路：

```text
全局地图
  -> 上传 GPU
  -> 世界点变换到相机系
  -> 针孔投影
  -> 点扩张
  -> atomicMin 遮挡
  -> 32FC1 深度图
  -> GridMap
```

当前实现已经满足 EGO-Planner 仿真演示需求，但在：

- 参数校验；
- 点云时间戳；
- 坐标系统一；
- CPU 视场准确性；
- CUDA 内存复用；
- 两种模式话题统一；
- 遗留代码清理；

方面仍有明确的改进空间。

阅读完本包后，再结合：

```text
map_generator
plan_env::GridMap
plan_manage/launch
```

就可以完整理解 EGO-Planner 如何把“完整仿真地图”转换成“规划器当前能够看到的局部障碍物”。
