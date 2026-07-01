# `map_generator` 功能包详细学习说明

> 适用源码：`/home/yxc/Desktop/ego-planner_ws/src/uav_simulator/map_generator`
>
> ROS 功能包名称：`map_generator`
>
> 所属系统：EGO-Planner 仿真环境

---

## 1. 一句话认识这个包

`map_generator` 用随机采样的方法生成三维障碍物点云，为 EGO-Planner 提供可重复测试规划算法的仿真环境。

当前源码主要生成两类障碍物：

```text
随机竖直柱体
        +
随机竖直椭圆环
        |
        v
合并为完整 PCL 点云
        |
        v
/map_generator/global_cloud
        |
        v
local_sensing
模拟无人机的局部感知
        |
        v
GridMap 与 EGO-Planner
```

虽然源码还保留：

- 基于里程计的局部点云查询；
- 鼠标点击添加障碍物；
- 方柱地图生成；

但这些路径目前都没有在实际执行流程中生效。

---

## 2. 它在 EGO-Planner 中的位置

如果使用本包作为地图源，仿真链路为：

```text
map_generator::random_forest
生成完整随机障碍物点云
        |
        | /map_generator/global_cloud
        v
local_sensing_node::pcl_render_node
根据无人机位姿生成局部感知点云
        |
        | /pcl_render_node/cloud
        v
plan_env::GridMap
构建概率占据地图和膨胀地图
        |
        v
plan_manage + bspline_opt
生成无碰撞轨迹
```

这个包只负责：

```text
环境几何生成
```

它不负责：

- 传感器视场模拟；
- 点云遮挡；
- 概率地图融合；
- 障碍物膨胀；
- 路径搜索；
- 轨迹优化；
- 无人机动力学仿真。

这些任务分别由：

```text
local_sensing
plan_env
path_searching
bspline_opt
so3_quadrotor_simulator
```

等包完成。

---

## 3. 当前默认 launch 并没有启动本包

这是理解当前工作区时最重要的一点。

`plan_manage/launch/simulator.xml` 中，`map_generator` 节点被写在：

```xml
<![CDATA[
  <node pkg="map_generator"
        name="random_forest"
        type="random_forest">
    ...
  </node>
]]>
```

CDATA 中的内容只是 XML 文本，不会被 roslaunch 当作节点执行。

当前真正启用的是：

```xml
<node pkg="mockamap"
      type="mockamap_node"
      name="mockamap_node">
```

并把：

```text
/mock_map
```

重映射成：

```text
/map_generator/global_cloud
```

所以当前默认运行链路实际是：

```text
mockamap
    |
    | /mock_map
    | remap
    v
/map_generator/global_cloud
    |
    v
local_sensing
```

也就是说：

```text
话题名看起来属于 map_generator，
实际发布者可能是 mockamap。
```

如果希望运行本包，需要将 `random_forest` 节点移出 CDATA，并停用或改名当前 `mockamap` 地图源。

---

## 4. 目录结构

```text
map_generator/
├── CMakeLists.txt
├── package.xml
├── .vscode/
│   └── c_cpp_properties.json
└── src/
    └── random_forest_sensing.cpp
```

源码规模很小：

| 文件 | 职责 |
|---|---|
| `random_forest_sensing.cpp` | 参数读取、随机障碍生成、点云发布和遗留局部感知逻辑 |
| `CMakeLists.txt` | 构建 `random_forest` 可执行节点 |
| `package.xml` | ROS 包元数据与依赖 |
| `.vscode/c_cpp_properties.json` | 旧开发环境的 IntelliSense 配置 |

核心算法全部集中在一个 C++ 文件中。

---

## 5. 构建产物

`CMakeLists.txt`：

```cmake
add_executable(
  random_forest
  src/random_forest_sensing.cpp
)
```

构建结果大致为：

```text
devel/lib/map_generator/random_forest
```

该包：

- 不生成库；
- 不生成消息；
- 不生成服务；
- 不包含 nodelet；
- 不包含测试目标。

手动运行：

```bash
rosrun map_generator random_forest
```

源码默认节点名：

```cpp
ros::init(argc, argv, "random_map_sensing");
```

如果通过 launch 指定：

```xml
name="random_forest"
```

则实际 ROS 节点名会被 launch 覆盖为：

```text
/random_forest
```

---

## 6. 构建依赖

### 6.1 代码实际使用

| 依赖 | 用途 |
|---|---|
| `roscpp` | 节点、参数、发布、订阅和日志 |
| `sensor_msgs` | 发布 `PointCloud2` |
| `nav_msgs` | 接收 `Odometry` |
| `geometry_msgs` | 遗留点击添加障碍物接口 |
| PCL | 点云容器和 KD 树 |
| `pcl_conversions` | PCL 与 ROS 点云消息转换 |
| Eigen3 | 二维距离、三维旋转和椭圆点计算 |
| C++ `<random>` | 随机数引擎与均匀分布 |

### 6.2 CMake 声明

```cmake
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  pcl_conversions
)
```

其中：

- `rospy` 没有使用；
- `std_msgs` 没有直接使用；
- `nav_msgs` 和 `sensor_msgs` 被源码使用，却没有列入 catkin components；
- 构建可能因为其他依赖间接导出头文件而成功，但声明不完整。

### 6.3 `package.xml`

当前包清单只明确声明：

```text
roscpp
std_msgs
```

没有完整声明：

- `geometry_msgs`；
- `nav_msgs`；
- `sensor_msgs`；
- `pcl_conversions`；
- `pcl_ros` 或 PCL 系统依赖；
- Eigen3。

包清单仍保留大量 catkin 模板注释，许可证也是：

```xml
<license>TODO</license>
```

因此该包可以在当前工作区中构建，不代表它已具备完整、可独立分发的依赖描述。

---

## 7. 编译配置

当前同时添加：

```cmake
ADD_COMPILE_OPTIONS(-std=c++11)
ADD_COMPILE_OPTIONS(-std=c++14)
```

通常后出现的：

```text
-std=c++14
```

最终生效。

Release 编译标志：

```cmake
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")
```

含义：

| 选项 | 作用 |
|---|---|
| `-O3` | 高等级优化 |
| `-Wall` | 开启常见警告 |
| `-g` | 保留调试符号 |

这是一个适合性能测试的组合：

```text
优化执行速度，同时保留调试符号。
```

建议只保留一个 C++ 标准，并使用现代：

```cmake
set(CMAKE_CXX_STANDARD 14)
```

表达。

---

## 8. ROS 接口总览

### 8.1 发布话题

| 话题 | 类型 | 当前实际状态 |
|---|---|---|
| `/map_generator/global_cloud` | `sensor_msgs/PointCloud2` | 正常循环发布 |
| `/map_generator/local_cloud` | `sensor_msgs/PointCloud2` | 发布代码不可达 |
| `/pcl_render_node/local_map` | `sensor_msgs/PointCloud2` | 点击订阅被注释，因此不发布 |

### 8.2 订阅话题

| 源码名 | 类型 | 用途 | 当前实际状态 |
|---|---|---|---|
| `~odometry` | `nav_msgs/Odometry` | 为局部半径查询提供无人机位置 | 能接收，但结果不会用于发布 |
| `/goal` | `geometry_msgs/PoseStamped` | 点击位置添加障碍物 | 订阅代码被注释 |

### 8.3 坐标系

发布点云统一设置：

```cpp
header.frame_id = "world";
```

点坐标本身也是按世界坐标直接生成的。

源码没有 TF 查询或坐标变换。

---

## 9. 全局状态与数据结构

### 9.1 地图点云

```cpp
pcl::PointCloud<pcl::PointXYZ> cloudMap;
```

保存完整随机地图。

每个点只有：

```text
x, y, z
```

没有：

- 颜色；
- 反射强度；
- 法向量；
- 障碍物编号；
- 语义类别。

### 9.2 KD 树

```cpp
pcl::KdTreeFLANN<pcl::PointXYZ>
    kdtreeLocalMap;
```

原本用于根据无人机位置查询一定半径内的障碍点。

当前虽然会构建 KD 树，但查询发布逻辑位于无条件 `return` 后，因此不会执行。

### 9.3 随机数

```cpp
random_device rd;
default_random_engine eng(rd());
```

地图每次启动通常不同。

还定义多组均匀分布：

```text
rand_x      障碍中心 x
rand_y      障碍中心 y
rand_w      柱体基础宽度
rand_h      柱体局部高度
rand_inf    柱体宽度缩放
rand_radius 椭圆第一半径
rand_radius2 椭圆第二半径
rand_theta  椭圆水平旋转角
rand_z      椭圆中心高度
```

---

## 10. 主函数完整流程

`main()`：

```text
初始化 ROS
        |
        v
创建全局、局部和点击点云发布器
        |
        v
订阅 odometry
        |
        v
读取地图与障碍参数
        |
        v
计算地图 x/y 边界
        |
        v
限制柱体数量上限
        |
        v
等待 0.5 秒
        |
        v
RandomMapGenerateCylinder()
一次性生成完整地图
        |
        v
按 _sense_rate 循环
        |
        v
pubSensedPoints()
反复发布完整地图
```

地图只生成一次。

循环中不会：

- 重新随机地图；
- 移动障碍物；
- 修改障碍物；
- 清空地图；
- 更新 KD 树。

因此这是：

```text
随机生成、随后保持静态的地图。
```

---

## 11. 参数读取方式

节点使用私有 NodeHandle：

```cpp
ros::NodeHandle n("~");
```

所以：

```cpp
n.param("map/x_size", ...)
```

读取的是：

```text
~map/x_size
```

如果节点名为 `/random_forest`，完整参数名为：

```text
/random_forest/map/x_size
```

`n.param()` 提供默认值，因此缺少参数时节点仍可运行。

这比只使用 `getParam()` 更健壮。

---

## 12. 参数总览

| 参数 | 源码默认值 | 作用 |
|---|---:|---|
| `init_state_x` | `0.0` | 初始安全区中心 x |
| `init_state_y` | `0.0` | 初始安全区中心 y |
| `map/x_size` | `50.0` | 随机中心采样区域 x 尺寸 |
| `map/y_size` | `50.0` | 随机中心采样区域 y 尺寸 |
| `map/z_size` | `5.0` | 读取后基本未参与生成 |
| `map/obs_num` | `30` | 随机柱体数量 |
| `map/resolution` | `0.1` | 障碍点采样间隔 |
| `map/circle_num` | `30` | 椭圆环数量 |
| `ObstacleShape/lower_rad` | `0.3` | 柱体基础直径下限 |
| `ObstacleShape/upper_rad` | `0.8` | 柱体基础直径上限 |
| `ObstacleShape/lower_hei` | `3.0` | 柱体高度下限 |
| `ObstacleShape/upper_hei` | `7.0` | 柱体高度上限 |
| `ObstacleShape/radius_l` | `7.0` | 椭圆第一半径分布下界 |
| `ObstacleShape/radius_h` | `7.0` | 椭圆第一半径分布上界 |
| `ObstacleShape/z_l` | `7.0` | 椭圆中心高度下界 |
| `ObstacleShape/z_h` | `7.0` | 椭圆中心高度上界 |
| `ObstacleShape/theta` | `7.0` | 椭圆绕 z 轴旋转范围 |
| `sensing/radius` | `10.0` | 遗留局部查询半径 |
| `min_distance` | `1.0` | 柱体中心最小距离 |

源码原本还需要：

```text
sensing/rate
```

但实际读取键写错，后文单独说明。

---

## 13. 地图边界

参数：

```text
map/x_size = X
map/y_size = Y
```

转换为：

```cpp
_x_l = -X / 2;
_x_h =  X / 2;
_y_l = -Y / 2;
_y_h =  Y / 2;
```

所以障碍物中心在：

```text
x ∈ [-X/2, X/2]
y ∈ [-Y/2, Y/2]
```

内均匀采样。

注意：

```text
限制的是障碍物中心，不是障碍物外轮廓。
```

靠近边界的柱体或椭圆环可能伸出配置地图范围。

`map/z_size` 被保存到：

```cpp
_z_limit = _z_size;
```

但 `_z_limit` 后续没有使用，因此 z 方向并没有真正按 `map/z_size` 裁剪。

---

## 14. 当前实际选择的地图生成函数

源码保留两个生成函数：

```cpp
RandomMapGenerate();
RandomMapGenerateCylinder();
```

主函数：

```cpp
// RandomMapGenerate();
RandomMapGenerateCylinder();
```

所以当前实际使用：

```text
圆柱体 + 椭圆环
```

而不是：

```text
方柱体 + 椭圆环
```

两个函数的大部分椭圆环代码重复，主要区别在柱状障碍物横截面：

| 函数 | 柱体横截面 |
|---|---|
| `RandomMapGenerate()` | 方形 |
| `RandomMapGenerateCylinder()` | 圆形 |

---

## 15. 随机柱体中心生成

对每个柱体：

```cpp
x = rand_x(eng);
y = rand_y(eng);
w = rand_w(eng);
inf = rand_inf(eng);
```

其中：

```text
x, y  障碍中心
w     基础直径
inf   额外缩放系数
```

当前：

```text
inf ∈ [0.5, 1.5]
```

最终直径：

```text
diameter = w × inf
```

如果 launch 设置：

```text
w ∈ [0.5, 0.7] m
```

则可能的最终直径约为：

```text
0.25 m 到 1.05 m
```

最终半径：

```cpp
radius = w * inf / 2;
```

---

## 16. 起点安全区

柱体中心生成后首先检查：

```cpp
sqrt((x - init_x)^2 +
     (y - init_y)^2) < 2.0
```

如果成立：

```cpp
i--;
continue;
```

即拒绝该样本并重新生成。

所以会在：

```text
(init_state_x, init_state_y)
```

附近保留半径 2 m 的无障碍中心区。

默认 `simulator.xml` 中：

```text
init_x = -18
init_y = 0
```

这个安全区用于避免无人机出生时直接位于随机障碍物内部。

注意它只限制障碍中心。

大半径障碍的边缘理论上仍可能进入 2 m 安全区。

---

## 17. 硬编码终点安全区

源码还检查：

```cpp
sqrt((x - 19.0)^2 +
     (y - 0.0)^2) < 2.0
```

即固定在：

```text
(19, 0)
```

附近保留半径 2 m 的区域。

这显然针对某个原始演示任务的终点。

它不是 ROS 参数，因此：

- 修改任务终点后安全区不会跟着变化；
- 小地图中 `(19, 0)` 可能在地图外；
- 多航点任务无法为所有目标保留空间；
- 代码用途不够通用。

更合理的设计是提供：

```text
goal_clearance_center
goal_clearance_radius
```

参数，或支持安全区列表。

---

## 18. 柱体最小中心距离

`RandomMapGenerateCylinder()` 维护：

```cpp
vector<Eigen::Vector2d> obs_position;
```

对新中心检查：

```cpp
(new_center - old_center).norm()
    < min_distance
```

如果过近则拒绝。

这保证的是：

```text
柱体中心之间至少相距 min_distance
```

并不保证柱体表面之间有这么大的间隙。

真实表面间距约为：

```text
中心距离 - 半径1 - 半径2
```

因此若希望无人机能穿过障碍之间的通道，`min_distance` 应考虑：

- 两个柱体最大半径；
- 无人机尺寸；
- 障碍膨胀半径；
- 控制误差。

---

## 19. 中心对齐到地图分辨率

通过安全区和距离检查后：

```cpp
x = floor(x / resolution) * resolution
    + resolution / 2;
```

y 同理。

这会将中心吸附到某个分辨率格子的中心。

概念上：

```text
grid_index = floor(position / resolution)
center = grid_index × resolution
       + resolution / 2
```

好处：

- 点云与体素地图更容易对齐；
- 障碍物边缘更规则；
- 减少浮点随机偏移。

需要注意负坐标使用 `floor()`：

```text
floor(-0.1) = -1
```

其行为与向 0 截断不同，但这里在正负方向上符合标准网格划分。

---

## 20. 圆柱横截面离散

最终直径：

```cpp
diameter = w * inf;
```

横向网格数量：

```cpp
widNum = ceil(diameter / resolution);
```

随后遍历一个方形候选区域：

```cpp
for r
  for s
```

候选点：

```text
temp_x = center_x + (r + 0.5) × resolution + 0.01
temp_y = center_y + (s + 0.5) × resolution + 0.01
```

只保留满足：

```cpp
distance((temp_x, temp_y), center)
    <= radius
```

的点。

因此横截面是由规则点阵近似的圆。

分辨率越小：

- 圆形更平滑；
- 点数更多；
- 生成和发布成本更高。

---

## 21. 柱体高度生成

对横截面中的每一个 `(r, s)` 单元，源码重新采样：

```cpp
h = rand_h(eng);
```

然后：

```cpp
heiNum = ceil(h / resolution);
```

这意味着同一柱体不同横向位置的高度可能不同。

所以顶部并不是严格平面，而是：

```text
随机起伏的粗糙顶部
```

这可以增加环境不规则性，但变量命名和注释容易让人误以为每根柱体只有一个统一高度。

如果希望标准圆柱，应在进入 `(r, s)` 循环前只采样一次 `h`。

---

## 22. 柱体向地下延伸

z 方向循环：

```cpp
for (int t = -30;
     t < heiNum;
     t++)
```

点高度：

```cpp
z = (t + 0.5) * resolution + 0.01;
```

当分辨率为 `0.1 m` 时，最低点约为：

```text
(-30 + 0.5) × 0.1 + 0.01
= -2.94 m
```

所以柱体会从地下约 3 m 一直延伸到随机高度。

这样做可能是为了确保：

- 地面附近没有点云缝隙；
- 地图原点略有偏差时柱体仍与地面相连；
- 障碍物不会看起来悬空。

代价是完整点云包含大量规划地图范围外的地下点。

`RandomMapGenerate()` 中使用 `t = -20`，即方柱版本约向地下延伸 2 m。

---

## 23. `+1e-2` 的作用

柱体点坐标都额外加：

```cpp
1e-2
```

即：

```text
0.01 m
```

这会让点避开精确的体素边界或整数栅格边界。

可能的工程目的：

- 避免浮点数正好落在边界造成索引歧义；
- 保证 `floor()` 或体素转换稳定；
- 减少边界点在相邻体素之间跳动。

但它也意味着点并不严格位于前面计算的格子中心。

更清晰的方式是统一整个系统的：

```text
position-to-index
index-to-position
```

规则，而不是分散加入经验偏移。

---

## 24. 方柱版本

`RandomMapGenerate()` 的柱体生成逻辑与圆柱版本相似，但没有圆形距离判断。

它遍历：

```text
widNum × widNum
```

的完整方形横截面。

结果近似：

```text
随机宽度的竖直方柱
```

该版本：

- 没有 `min_distance` 检查；
- 没有宽度缩放 `inf`；
- 向地下延伸 20 个栅格；
- 当前主函数没有调用。

切换方式是修改源码：

```cpp
RandomMapGenerate();
// RandomMapGenerateCylinder();
```

当前没有 ROS 参数用于运行时选择障碍形状。

---

## 25. 椭圆环生成

除柱体外，两个生成函数都会创建：

```text
circle_num
```

个椭圆环。

每个环随机采样：

```text
x, y    中心水平位置
z       中心高度
theta   绕世界 z 轴旋转角
radius1 第一半径
radius2 第二半径
```

中心同样：

- 避开初始位置 2 m 安全区；
- 避开固定 `(19, 0)` 终点安全区；
- 对齐到地图分辨率中心。

但是椭圆环之间没有 `min_distance` 检查，也不检查与柱体的距离。

所以环可能：

- 相互重叠；
- 与柱体相交；
- 伸出地图边界；
- 穿过安全区边缘。

---

## 26. 椭圆环的几何方程

局部坐标：

```cpp
cpt.x = 0;
cpt.y = radius1 * cos(angle);
cpt.z = radius2 * sin(angle);
```

所以未旋转前，椭圆位于：

```text
x = 0
```

的 y-z 平面。

其参数方程：

```text
x = 0
y = r1 cos(a)
z = r2 sin(a)
```

这是一条竖直椭圆曲线。

随后绕世界 z 轴旋转：

```cpp
rotate <<
  cos(theta), -sin(theta), 0,
  sin(theta),  cos(theta), 0,
  0,           0,          1;
```

因此环仍然位于一个竖直平面中，只是水平方向发生旋转。

它适合测试无人机：

```text
穿环、绕环和三维高度规划
```

能力。

---

## 27. 环的采样密度

角度循环：

```cpp
for (double angle = 0;
     angle < 6.282;
     angle += resolution / 2)
```

`6.282` 近似：

```text
2π
```

当：

```text
resolution = 0.1
```

角度步长：

```text
0.05 rad
```

每个环大约包含：

```text
2π / 0.05 ≈ 126 个点
```

这里存在一个量纲混合：

```text
resolution 的单位是 m
angle 的单位是 rad
```

直接使用 `resolution / 2` 作为角度步长只是经验设计。

更合理的方式是根据希望的弧长间隔计算：

```text
angle_step ≈ point_spacing / radius
```

---

## 28. 当前所谓的“inflate”没有实际膨胀

源码：

```cpp
for (int ifx = -0; ifx <= 0; ++ifx)
  for (int ify = -0; ify <= 0; ++ify)
    for (int ifz = -0; ifz <= 0; ++ifz)
```

`-0` 与 `0` 完全相同。

所以每个循环都只执行一次：

```text
ifx = ify = ifz = 0
```

最终只生成椭圆中心线上的单层点。

因此当前环不是：

- 有厚度的圆环；
- 管状 torus；
- 膨胀后的安全障碍。

而是：

```text
一条稀疏椭圆点线。
```

真正的安全膨胀由后续 `GridMap` 的障碍膨胀逻辑完成，但如果原始点线太稀，仍可能出现局部断裂。

---

## 29. `radius_l`、`radius_h` 和 `radius2`

第一半径：

```cpp
rand_radius_ =
    uniform_real_distribution<double>(
        radius_l_, radius_h_);
```

第二半径：

```cpp
rand_radius2_ =
    uniform_real_distribution<double>(
        radius_l_, 1.2);
```

注意第二半径没有使用单独的：

```text
radius2_lower
radius2_upper
```

而是复用了 `radius_l_` 并把上限写死为 `1.2`。

所以参数语义不直观：

- `radius_l` 同时影响两个轴；
- `radius_h` 只影响第一轴；
- 第二轴上限无法通过参数修改。

建议将两个半径分别参数化。

---

## 30. 地图点云收尾

生成完成后：

```cpp
cloudMap.width = cloudMap.points.size();
cloudMap.height = 1;
cloudMap.is_dense = true;
```

这表示：

```text
无组织点云
```

其中：

```text
width  = 点数
height = 1
```

`is_dense = true` 表示代码声称点云中没有 NaN。

随后：

```cpp
kdtreeLocalMap.setInputCloud(
    cloudMap.makeShared());
```

构建 KD 树，并设置：

```cpp
_map_ok = true;
```

---

## 31. 全局地图发布

`pubSensedPoints()` 每次循环执行：

```cpp
pcl::toROSMsg(cloudMap, globalMap_pcd);
globalMap_pcd.header.frame_id = "world";
_all_map_pub.publish(globalMap_pcd);
```

因此：

```text
完整地图会被反复转换并发布。
```

这有一个实际好处：

- 发布器不是 latched；
- 后启动的 `local_sensing` 仍有机会收到地图。

但也有明显成本：

- 静态点云每帧重复做 PCL 到 ROS 转换；
- 重复序列化完整地图；
- 占用 CPU 和 ROS 带宽；
- 地图越密，成本越高。

更合理的实现可以：

1. 生成后只转换一次；
2. 使用 latched publisher；
3. 只在地图变化时重新发布。

---

## 32. 为什么 `/local_cloud` 当前不会发布

全局点云发布后立刻执行：

```cpp
return;
```

所以后面的：

```text
检查 map 和 odometry
KD 树半径搜索
构造 localMap
发布 /map_generator/local_cloud
```

全部是不可达代码。

因此当前真实行为是：

```text
/map_generator/global_cloud 正常发布
/map_generator/local_cloud 永远不发布
```

这个无条件 `return` 可能是项目后期将局部感知职责移交给：

```text
local_sensing
```

之后留下的临时改动。

---

## 33. 遗留局部点云算法

如果移除 `return`，局部点云路径为：

```text
接收 odometry
        |
        v
取无人机当前位置
        |
        v
KD 树 radiusSearch
        |
        v
提取 sensing_range 内全部障碍点
        |
        v
/map_generator/local_cloud
```

它只按球形距离裁剪，不考虑：

- 无人机朝向；
- 相机视场；
- 遮挡；
- 深度图；
- 传感器噪声。

因此即使恢复，也比 `local_sensing` 的 CPU 版本更简单。

当前系统选择：

```text
map_generator 发布完整地图
local_sensing 负责局部感知
```

职责划分更清晰。

---

## 34. 里程计回调

```cpp
void rcvOdometryCallbck(
    const nav_msgs::Odometry odom)
```

首先忽略：

```cpp
child_frame_id == "X"
child_frame_id == "O"
```

这看起来是原项目中用于过滤特殊状态消息的约定，但当前代码没有解释。

随后保存：

```text
位置 x, y, z
线速度 vx, vy, vz
三个固定为 0 的量
```

到：

```cpp
vector<double> _state;
```

当前只有位置原本会被 KD 树局部查询使用。

由于局部发布代码不可达，odometry 对当前全局地图发布没有任何影响。

---

## 35. 点击添加障碍物

`clickCallback()` 接收：

```text
geometry_msgs/PoseStamped
```

使用消息中的：

```text
x, y
```

作为新障碍物中心。

然后：

- 随机生成宽度；
- 为每个横向格随机生成高度；
- 添加方柱点到 `clicked_cloud_`；
- 同时添加到完整 `cloudMap`；
- 发布 `/pcl_render_node/local_map`。

原订阅代码：

```cpp
// ros::Subscriber click_sub =
//     n.subscribe("/goal", 10, clickCallback);
```

已经注释，因此当前点击 RViz 目标不会添加障碍物。

---

## 36. 点击点云的累积行为

`clicked_cloud_` 从不清空。

每次点击都会：

```cpp
clicked_cloud_.points.push_back(...)
```

所以发布的是：

```text
从第一次点击到当前为止的全部点击障碍物
```

而不是仅发布本次新增障碍。

同样，`cloudMap` 也持续追加。

但添加后没有重新：

```cpp
kdtreeLocalMap.setInputCloud(...)
```

因此如果恢复旧局部 KD 树查询：

```text
新增障碍可能不会立即进入 KD 树索引。
```

---

## 37. 与 `local_sensing` 的关系

本包与 `local_sensing` 的职责可以概括为：

```text
map_generator
回答“世界中有哪些障碍物”

local_sensing
回答“无人机当前能看到哪些障碍物”
```

接口：

```text
/map_generator/global_cloud
        |
        v
local_sensing 的 ~global_map
```

默认 CPU `local_sensing`：

1. 只接收第一帧全局地图；
2. 做 0.1 m 体素下采样；
3. 构建 KD 树；
4. 根据无人机位置和朝向裁剪；
5. 发布 `/pcl_render_node/cloud`。

因此 `map_generator` 没有必要高频发布地图，但重复发布可避免节点启动顺序问题。

---

## 38. 与 `mockamap` 的关系

`map_generator` 和 `mockamap` 都可以作为全局地图源。

| 特性 | `map_generator` | `mockamap` |
|---|---|---|
| 主要地图 | 随机柱体和椭圆环 | Perlin、柱体、2D/3D 迷宫 |
| 核心表示 | PCL 点云 | 程序化体素/点云 |
| 默认启动 | 否 | 是 |
| 默认输出 | `/map_generator/global_cloud` | `/mock_map`，被重映射 |
| 随机种子 | 当前不可控 | launch 中可设置 |
| 穿环场景 | 支持 | 取决于地图类型 |

两者通常应二选一。

如果同时向：

```text
/map_generator/global_cloud
```

发布，订阅者会收到来自不同发布者的不同地图，行为很难预测。

---

## 39. 默认 `simulator.xml` 参数

被 CDATA 禁用的 `random_forest` 配置包括：

```text
init_state_x = -18
init_state_y = 0
map/resolution = 0.1
lower_rad = 0.5
upper_rad = 0.7
lower_hei = 0.0
upper_hei = 3.0
radius_l = 0.7
radius_h = 0.5
z_l = 0.7
z_h = 0.8
theta = 0.5
sensing/radius = 5.0
sensing/rate = 10.0
```

地图尺寸、柱体数量、环数量和最小距离由上层 launch 参数传入。

即使该节点当前没有启动，这些配置仍反映了原始设计意图。

---

## 40. 随机性与可复现性

launch 中设置：

```xml
<param name="ObstacleShape/seed"
       value="1"/>
```

但是源码没有读取：

```text
ObstacleShape/seed
```

随机引擎始终使用：

```cpp
default_random_engine eng(rd());
```

所以地图不受 launch seed 控制。

结果：

- 每次启动地图可能不同；
- 相同参数不保证复现实验；
- 很难稳定重现规划失败；
- 算法性能对比不公平。

源码中已有被注释的固定种子思路：

```cpp
// default_random_engine eng(0);
```

更好的方式是读取 ROS 参数：

```text
seed >= 0  使用固定种子
seed < 0   使用 random_device
```

---

## 41. 点数与性能估算

### 41.1 柱体点数

单根柱体点数大致为：

```text
横截面面积 / resolution²
×
(地下深度 + 高度) / resolution
```

所以点数对分辨率近似呈：

```text
O(1 / resolution³)
```

变化。

将分辨率从：

```text
0.1 m -> 0.05 m
```

单个实体障碍物的点数理论上可能增加约：

```text
2³ = 8 倍
```

### 41.2 椭圆环点数

当前环只是曲线，角度步长为 `resolution / 2`。

点数近似：

```text
4π / resolution
```

因此约为：

```text
O(1 / resolution)
```

### 41.3 循环发布成本

完整点云每一轮都执行：

```text
PCL -> ROS 消息转换
序列化
发布
```

地图点数和 `_sense_rate` 都会线性增加通信成本。

---

## 42. 分辨率的多重作用

`map/resolution` 同时影响：

1. 柱体中心吸附；
2. 柱体横向点间距；
3. 柱体纵向点间距；
4. 椭圆中心吸附；
5. 椭圆角度采样步长；
6. 向地下延伸的真实距离；
7. 障碍物点数；
8. 后续局部感知下采样效果。

因此改变它不仅仅是改变“地图精度”。

例如 `t = -30` 是固定栅格数：

```text
地下深度 ≈ 30 × resolution
```

分辨率变大时，障碍物向地下延伸更深。

---

## 43. 当前源码中的重要风险与注意事项

以下结论针对当前工作空间源码。

### 43.1 默认 launch 实际不运行 `map_generator`

`random_forest` 被放入 CDATA，当前地图来自 `mockamap`。

调试 `/map_generator/global_cloud` 时，不能仅凭话题名判断发布者。

应使用：

```bash
rostopic info /map_generator/global_cloud
```

确认真正的 publisher。

### 43.2 感知频率参数键写错

源码：

```cpp
n.param("sensing/radius",
        _sensing_range, 10.0);
n.param("sensing/radius",
        _sense_rate, 10.0);
```

第二行应很可能是：

```cpp
n.param("sensing/rate",
        _sense_rate, 10.0);
```

当前 `_sense_rate` 会读取半径值。

例如 launch：

```text
sensing/radius = 5
sensing/rate = 10
```

实际循环频率为：

```text
5 Hz
```

而不是 10 Hz。

### 43.3 局部点云代码不可达

`pubSensedPoints()` 中全局发布后无条件 `return`。

所以：

- `_local_map_pub` 没有实际输出；
- odometry 不影响当前行为；
- KD 树构建没有实际用途；
- `_sensing_range` 只间接错误控制循环频率。

### 43.4 点云时间戳没有设置

发布消息只设置：

```cpp
header.frame_id = "world";
```

没有设置：

```cpp
header.stamp
```

所以时间戳通常为 0。

对于静态地图问题较小，但会影响：

- rosbag 分析；
- 延迟统计；
- 可视化同步；
- 多源地图调试。

### 43.5 发布器不是 latched

全局静态地图使用普通 publisher。

因此代码只能通过循环重复发布来确保后启动订阅者收到数据。

更适合静态地图的方式是：

```cpp
advertise(..., queue_size, true);
```

即 latched publisher。

### 43.6 每轮重复转换完整点云

地图静态不变，却每次调用：

```cpp
pcl::toROSMsg(...)
```

可以在地图生成完成后只转换一次。

### 43.7 随机 seed 参数完全无效

launch 设置了 seed，源码不读取。

实验无法稳定复现。

### 43.8 椭圆第一半径上下界可能反向

默认禁用配置中：

```text
radius_l = 0.7
radius_h = 0.5
```

但均匀分布要求通常是：

```text
lower <= upper
```

当前配置违反该前提，生成结果不可靠。

参数加载后应显式检查并交换或报错。

### 43.9 源码默认环参数也不安全

若不提供参数，默认：

```text
radius_l = 7.0
radius_h = 7.0
```

第一半径是固定 7 m。

但第二半径分布构造为：

```text
[radius_l, 1.2] = [7.0, 1.2]
```

下界大于上界，同样违反分布前提。

因此直接 `rosrun` 使用全部默认值时，环生成参数存在明显问题。

### 43.10 `map/z_size` 不限制实际地图高度

`_z_limit` 赋值后未使用。

柱体高度由：

```text
ObstacleShape/lower_hei
ObstacleShape/upper_hei
```

控制，环高度由：

```text
z_l, z_h, radius2
```

控制。

障碍物可能超出 z 地图范围。

### 43.11 障碍物中心可能无限重采样

循环通过：

```cpp
i--;
continue;
```

拒绝不合格样本。

如果：

- 地图很小；
- 障碍数量很多；
- `min_distance` 很大；
- 安全区占据大部分地图；

可能长期找不到合法位置，生成过程近似无限循环。

应设置最大尝试次数并输出明确错误。

### 43.12 `_obs_num` 上限只依赖 x 尺寸

```cpp
_obs_num =
    min(_obs_num, (int)_x_size * 10);
```

它没有考虑：

- y 尺寸；
- 障碍物直径；
- `min_distance`；
- 安全区面积。

因此这个上限不能真正防止过密配置。

### 43.13 最小距离只约束柱体中心

环不参与 `obs_position`。

所以：

- 环与环可重叠；
- 环与柱体可重叠；
- 环可穿过柱体；
- 安全通道无法由 `min_distance` 保证。

### 43.14 障碍边缘可以越过地图边界

只约束中心：

```text
x ∈ [x_l, x_h]
y ∈ [y_l, y_h]
```

没有根据半径缩小采样范围。

### 43.15 固定终点 `(19, 0)` 写死

该安全区与当前规划目标没有参数关联。

更换地图尺寸和任务后可能毫无意义。

### 43.16 柱体高度在每个横向格重新随机

导致顶部粗糙。

如果设计目标是标准柱体，这是算法位置错误。

### 43.17 地下点数量较大

圆柱固定向下生成 30 层。

许多点可能永远不会进入规划地图，却仍占用：

- 生成时间；
- 点云内存；
- ROS 带宽；
- `local_sensing` 下采样时间。

### 43.18 环的“inflate”循环无效

`-0` 到 `0` 只执行一次。

注释与真实行为不一致。

### 43.19 环角度步长存在量纲混合

把米制 `resolution` 直接当弧度使用。

不同半径环的实际弧长采样密度不一致。

### 43.20 点击订阅被注释

`clickCallback()` 和点击发布器仍存在，但用户无法通过默认节点触发。

### 43.21 点击新增障碍后 KD 树不更新

如果未来恢复局部查询，新增点可能不在旧 KD 树中。

### 43.22 `cloudMap` 生成前没有清空

当前只调用一次，所以没有问题。

若未来支持重新生成地图，新的地图会追加到旧地图。

### 43.23 `translate` 变量未使用

椭圆环生成中创建：

```cpp
Eigen::Vector3d translate(x, y, z);
```

但后续直接再次构造：

```cpp
Eigen::Vector3d(x, y, z)
```

该变量是遗留代码。

### 43.24 全局变量和函数位于全局命名空间

状态分散且缺乏封装。

未来增加：

- 多地图实例；
- 动态重新生成；
- 多线程；
- 服务接口；

会比较困难。

### 43.25 依赖声明不完整

源码、CMake 和 `package.xml` 三者的依赖集合不一致。

在干净环境中可能出现构建失败。

### 43.26 C++ 标准重复设置

同时指定 C++11 和 C++14，应整理。

### 43.27 VS Code 配置已经过时

`.vscode/c_cpp_properties.json` 使用：

```text
/usr/include/pcl-1.7
/opt/ros/kinetic/include
compilerPath = /usr/bin/gcc
cppStandard = c++17
```

而 CMake 使用 C++14。

如果当前系统不是 ROS Kinetic 或 PCL 1.7，IDE 可能显示错误的头文件提示。

该文件只影响编辑器 IntelliSense，不影响 catkin 的实际编译。

---

## 44. 推荐调试方法

### 44.1 先确认地图发布者

```bash
rostopic info /map_generator/global_cloud
```

查看 publisher 是：

```text
/random_forest
```

还是：

```text
/mockamap_node
```

这是当前工作区最先应该确认的事情。

### 44.2 检查随机地图节点

```bash
rosnode info /random_forest
```

确认：

- 参数命名空间；
- odometry 重映射；
- 三个发布话题；
- 节点是否真的已启动。

### 44.3 检查全局点云

```bash
rostopic hz /map_generator/global_cloud
rostopic echo -n 1 \
  /map_generator/global_cloud/header
```

当前时间戳可能为 0。

### 44.4 统计点数

可以查看：

```bash
rostopic echo -n 1 \
  /map_generator/global_cloud/width
```

因为点云无组织：

```text
width = 点数
height = 1
```

改变分辨率、障碍数量和高度后，比较点数增长。

### 44.5 在 RViz 中观察

添加 `PointCloud2`：

```text
Topic: /map_generator/global_cloud
Fixed Frame: world
```

重点检查：

- 初始位置附近是否空旷；
- `(19, 0)` 附近是否空旷；
- 柱体顶部是否随机起伏；
- 柱体是否伸出地图边界；
- 椭圆环是否稀疏或断裂；
- 障碍物是否大量位于地下。

### 44.6 检查参数实际值

如果节点名为 `/random_forest`：

```bash
rosparam get /random_forest
```

尤其核对：

```text
sensing/radius
sensing/rate
ObstacleShape/radius_l
ObstacleShape/radius_h
```

### 44.7 测试可复现性

连续启动两次并保存点云：

```bash
rosrun pcl_ros pointcloud_to_pcd \
  input:=/map_generator/global_cloud
```

比较两个 PCD。

当前即使 launch seed 相同，地图通常仍不同。

---

## 45. 常见故障

### 45.1 启动整套仿真后地图形状与源码不符

最可能原因：

```text
当前运行的是 mockamap，不是 random_forest。
```

使用：

```bash
rostopic info /map_generator/global_cloud
```

确认发布者。

### 45.2 手动启动后生成过程卡住

可能原因：

- 地图范围过小；
- 障碍数量过多；
- `min_distance` 过大；
- 初始与终点安全区占据大部分地图；
- 合法中心难以采样。

当前没有最大重试次数。

### 45.3 椭圆环形状异常

检查：

```text
radius_l <= radius_h
radius_l <= 1.2
z_l <= z_h
resolution > 0
```

默认或旧 launch 参数可能违反半径上下界要求。

### 45.4 `/map_generator/local_cloud` 没有数据

这是当前源码预期行为。

发布代码位于无条件 `return` 后。

局部感知请查看：

```text
/pcl_render_node/cloud
```

### 45.5 修改 `sensing/rate` 没有效果

因为源码错误读取：

```text
sensing/radius
```

作为 `_sense_rate`。

### 45.6 `/goal` 点击不能添加障碍

因为对应 subscriber 被注释。

仅有 callback 函数存在并不足以接收话题。

### 45.7 地图每次启动都不同

因为 `ObstacleShape/seed` 没有被读取，随机引擎使用 `random_device`。

---

## 46. 推荐改进方向

### 46.1 低风险修复

优先建议：

1. 修复 `sensing/rate` 参数键；
2. 为全局点云设置时间戳；
3. 读取并使用随机种子；
4. 校验所有上下界；
5. 校验 `resolution > 0`；
6. 删除无用变量和 include；
7. 补齐 CMake 与 `package.xml` 依赖；
8. 只保留一个 C++ 标准；
9. 更新或删除过时 `.vscode` 配置。

### 46.2 使用 latched 静态地图发布器

```cpp
_all_map_pub =
    n.advertise<sensor_msgs::PointCloud2>(
        "/map_generator/global_cloud",
        1,
        true);
```

生成后：

1. 转换一次；
2. 发布一次；
3. `ros::spin()` 等待。

新订阅者仍会自动收到最后一帧地图。

### 46.3 将地图生成器封装为类

可以设计：

```cpp
class RandomMapGenerator
{
  void loadParameters();
  bool generate();
  void publish();
  void regenerateService(...);
};
```

好处：

- 减少全局变量；
- 更容易单元测试；
- 支持重新生成；
- 支持多个实例；
- 参数验证更集中。

### 46.4 增加地图类型参数

例如：

```text
map/type = square_column
map/type = cylinder
map/type = cylinder_and_ring
```

无需重新编译即可切换。

### 46.5 增加安全区列表

支持：

```yaml
clear_regions:
  - {x: -18, y: 0, radius: 2}
  - {x:  19, y: 0, radius: 2}
```

取代硬编码终点。

### 46.6 使用真正的障碍表面间距

生成新障碍时检查：

```text
center_distance
  >= radius_new
   + radius_old
   + requested_clearance
```

比仅检查中心距离更符合通道需求。

### 46.7 统一环的采样参数

建议分别配置：

```text
ring/radius_horizontal_min
ring/radius_horizontal_max
ring/radius_vertical_min
ring/radius_vertical_max
ring/thickness
ring/point_spacing
```

并按弧长决定角度步长。

### 46.8 避免无效地下点

可以只生成到：

```text
ground_height - 一到两个分辨率
```

或者让 `GridMap` 地面高度和地图生成器共享参数。

### 46.9 提供重新生成服务

例如：

```text
/map_generator/regenerate
```

调用时：

1. 清空点云；
2. 使用指定 seed；
3. 重新生成；
4. 重建 KD 树；
5. 发布新地图。

便于批量规划测试。

---

## 47. 推荐源码阅读顺序

### 第一阶段：确认构建和默认运行状态

阅读：

```text
CMakeLists.txt
package.xml
plan_manage/launch/simulator.xml
```

先理解：

- 生成哪个可执行文件；
- `random_forest` 默认被禁用；
- 当前 `/map_generator/global_cloud` 来自 `mockamap`。

### 第二阶段：读主函数

阅读：

```cpp
main()
```

掌握：

- 参数列表；
- 地图边界；
- 当前选择的生成函数；
- 发布循环。

### 第三阶段：理解圆柱生成

阅读：

```cpp
RandomMapGenerateCylinder()
```

建议顺序：

1. 随机分布初始化；
2. 初始与终点安全区；
3. 中心最小距离；
4. 圆形横截面；
5. 随机高度；
6. 椭圆环。

### 第四阶段：对比方柱版本

阅读：

```cpp
RandomMapGenerate()
```

重点找出与圆柱版本的不同。

### 第五阶段：理解发布和遗留逻辑

阅读：

```text
pubSensedPoints()
rcvOdometryCallbck()
clickCallback()
```

识别：

- 无条件 `return`；
- 不可达局部点云；
- 被注释的点击订阅。

### 第六阶段：继续跟踪下游

阅读：

```text
uav_simulator/local_sensing/
planner/plan_env/
```

理解全局点云如何最终转化为规划器的局部占据地图。

---

## 48. 建议学习实验

### 实验一：确认真实地图源

运行完整仿真后：

```bash
rostopic info /map_generator/global_cloud
```

记录发布者。

然后切换到 `random_forest`，再次确认。

### 实验二：固定随机种子

修改代码读取 seed。

使用相同 seed 连续启动两次，验证点云完全一致。

再改变 seed，观察地图变化。

### 实验三：改变分辨率

尝试：

```text
0.20 m
0.10 m
0.05 m
```

记录：

- 生成耗时；
- 点云点数；
- ROS 消息大小；
- RViz 圆柱平滑程度；
- 规划地图更新耗时。

### 实验四：验证起点安全区

显示：

```text
初始无人机位置
/map_generator/global_cloud
```

测量障碍中心与初始位置的关系。

进一步验证大障碍边缘是否可能进入 2 m 区域。

### 实验五：观察柱体顶部

将柱体数量设为 1、宽度增大。

从侧面观察不同横向格子的高度，确认顶部是随机起伏而非平面。

### 实验六：恢复方柱版本

切换：

```cpp
RandomMapGenerate();
```

对比方柱和圆柱：

- 点数；
- 几何形状；
- 规划难度；
- B 样条绕障行为。

### 实验七：恢复局部点云

临时移除 `pubSensedPoints()` 中的 `return`。

发布 odometry 后观察：

```text
/map_generator/local_cloud
```

并与：

```text
/pcl_render_node/cloud
```

比较。

### 实验八：验证频率参数错误

设置：

```text
sensing/radius = 5
sensing/rate = 20
```

观察全局点云实际约为 5 Hz。

修复键名后验证变为 20 Hz。

### 实验九：测试高密度失败

使用小地图、大 `obs_num` 和大 `min_distance`。

观察生成是否长时间停留。

然后增加最大采样尝试次数，验证能否安全报错退出。

### 实验十：椭圆环厚度

将无效的 `-0...0` 膨胀循环改为：

```text
-1...1
```

或按欧氏距离生成管状厚度。

观察原始点云与膨胀占据地图的连通性变化。

---

## 49. 常见问题

### Q1：为什么节点叫 `random_forest`？

这里的 forest 指由大量随机竖直柱状障碍构成的“随机森林”，不是机器学习中的随机森林算法。

### Q2：地图是二维还是三维？

输出是三维点云。

柱体沿 z 方向延伸，椭圆环也具有明显三维高度结构。

### Q3：障碍物会运动吗？

不会。

地图在启动时生成一次，之后保持静态。

### Q4：`map/z_size` 会裁剪障碍物吗？

当前不会。

参数被读取，但没有用于限制生成点的 z 坐标。

### Q5：为什么要反复发布静态地图？

因为发布器不是 latched，反复发布可以让晚启动的订阅者收到地图。

但这会浪费转换和通信资源。

### Q6：为什么还要构建 KD 树？

源码原本支持根据 odometry 发布局部半径点云。

当前这段代码不可达，因此 KD 树属于遗留功能。

### Q7：`/map_generator/local_cloud` 和 `/pcl_render_node/cloud` 有什么区别？

前者原本只按球形距离查询。

后者由 `local_sensing` 生成，还会根据无人机朝向和近似视场过滤，是当前规划器实际使用的接口。

### Q8：为什么地图每次不同？

随机引擎由 `random_device` 初始化，launch 的 seed 参数没有被使用。

### Q9：圆环是不是有实体厚度？

当前不是。

它只是椭圆中心线上的一圈点，后续依赖占据地图膨胀形成安全厚度。

### Q10：为什么完整仿真中看不到圆柱和圆环？

因为默认启动的是 `mockamap`，`random_forest` 被 CDATA 禁用。

### Q11：能否直接让规划器订阅全局地图？

技术上可以重映射，但这会让规划器立即知道整个环境，不再模拟有限局部感知。

当前设计通过 `local_sensing` 更接近在线规划场景。

### Q12：这个包与 `mockamap` 应该同时运行吗？

通常不应该让两者向同一个全局地图话题同时发布。

应根据测试目的选择一个地图源。

---

## 50. 总结

`map_generator` 是一个小型随机三维点云地图生成器。

它的核心任务是：

```text
在给定 x-y 范围内，
随机生成柱状障碍物和竖直椭圆环，
将它们离散为世界坐标系点云，
并发布给局部感知模拟器。
```

当前实际地图生成链路：

```text
读取参数
  -> 随机采样柱体中心
  -> 避开起点和固定终点
  -> 检查柱体中心间距
  -> 离散圆柱体
  -> 随机生成竖直椭圆点环
  -> 合并为 cloudMap
  -> /map_generator/global_cloud
```

理解本包时，最重要的七条主线是：

1. **包只生成一个 `random_forest` ROS 节点。**
2. **当前主函数使用圆柱体加椭圆环地图。**
3. **地图只生成一次，但完整点云会循环发布。**
4. **局部 KD 树点云发布代码目前不可达。**
5. **点击添加障碍物接口目前没有订阅者。**
6. **默认整套仿真实际使用 `mockamap`，不是本包。**
7. **输出 `/map_generator/global_cloud` 会进入 `local_sensing`，再变成规划器的局部观测。**

当前源码已经能生成适合规划演示的随机环境，但在：

- 随机种子；
- 参数键；
- 参数范围校验；
- 时间戳；
- 静态地图发布效率；
- 地图边界约束；
- 遗留不可达代码；
- 依赖声明；

方面仍有明确的改进空间。

阅读完本包后，再结合：

```text
mockamap
local_sensing
plan_env::GridMap
plan_manage/launch
```

就可以完整理解 EGO-Planner 仿真环境如何从“随机世界”一路形成“规划器当前看到的局部障碍地图”。
