# `mockamap` 功能包详细学习说明

> 适用源码：`/home/yxc/Desktop/ego-planner_ws/src/uav_simulator/mockamap`
>
> ROS 功能包名称：`mockamap`
>
> 所属系统：EGO-Planner 仿真环境

---

## 1. 一句话认识这个包

`mockamap` 是一个程序化三维点云地图生成器。

它根据 ROS 参数选择四种地图：

```text
type = 1  三维 Perlin 噪声地图
type = 2  随机方柱地图
type = 3  二维递归分割迷宫
type = 4  三维 Voronoi 风格迷宫
```

统一输出：

```text
/mock_map
```

在 EGO-Planner 默认仿真中，该话题被重映射为：

```text
/map_generator/global_cloud
```

然后交给 `local_sensing` 模拟局部感知。

---

## 2. 它在 EGO-Planner 中的位置

当前默认链路：

```text
mockamap_node
程序化生成完整静态地图
        |
        | /mock_map
        | remap
        v
/map_generator/global_cloud
        |
        v
local_sensing_node
根据无人机位姿筛选局部点云
        |
        | /pcl_render_node/cloud
        v
plan_env::GridMap
概率占据地图与障碍膨胀
        |
        v
EGO-Planner 局部轨迹规划
```

`mockamap` 负责回答：

```text
整个仿真世界中哪些位置是障碍物？
```

它不负责：

- 传感器视场；
- 遮挡模拟；
- 地图概率更新；
- 无人机定位；
- 轨迹规划；
- 飞行控制。

---

## 3. 它是当前默认地图源

`plan_manage/launch/simulator.xml` 中：

```xml
<node pkg="mockamap"
      type="mockamap_node"
      name="mockamap_node">
  <remap from="/mock_map"
         to="/map_generator/global_cloud"/>
```

相邻的 `map_generator/random_forest` 被放在 CDATA 中，不会启动。

因此当前整套 EGO-Planner 仿真真正使用：

```text
mockamap 的 Perlin 三维地图
```

而不是 `map_generator` 的随机圆柱地图。

默认关键参数：

```text
seed        = 127
resolution  = 0.1 m
type        = 1
complexity  = 0.05
fill        = 0.12
fractal     = 1
attenuation = 0.1
```

---

## 4. 目录结构

```text
mockamap/
├── CMakeLists.txt
├── package.xml
├── include/
│   ├── maps.hpp
│   └── perlinnoise.hpp
├── src/
│   ├── mockamap.cpp
│   ├── maps.cpp
│   ├── perlinnoise.cpp
│   └── ces_randommap.cpp
├── launch/
│   ├── mockamap.launch
│   ├── perlin3d.launch
│   ├── post2d.launch
│   ├── maze2d.launch
│   ├── maze3d.launch
│   └── yxc_test_*.launch
└── config/
    ├── rviz.rviz
    ├── rviz_simple.rviz
    └── rviz_simple_3d.rviz
```

核心文件职责：

| 文件 | 职责 |
|---|---|
| `mockamap.cpp` | ROS 节点入口、公共参数、地图发布和可选表面优化 |
| `maps.hpp` | `Maps`、`BasicInfo` 和 `MazePoint` 声明 |
| `maps.cpp` | 四类地图的生成算法 |
| `perlinnoise.hpp/.cpp` | Improved Perlin Noise 实现 |
| `ces_randommap.cpp` | 遗留固定地图与局部感知代码 |
| `launch/*.launch` | 各类地图示例参数 |
| `config/*.rviz` | 点云可视化配置 |

---

## 5. 构建产物

当前 CMake：

```cmake
file(GLOB mockamap_SRCS src/*.cpp)

add_executable(
  mockamap_node
  ${mockamap_SRCS}
)
```

所以 `src/` 下四个 `.cpp` 全部被编译进同一个节点：

```text
devel/lib/mockamap/mockamap_node
```

手动运行：

```bash
rosrun mockamap mockamap_node
```

没有生成：

- 独立库；
- 自定义消息；
- 服务；
- nodelet；
- 测试目标。

---

## 6. 一个容易忽略的构建特点

由于使用：

```cmake
file(GLOB ... src/*.cpp)
```

`ces_randommap.cpp` 也会参与编译。

它虽然没有 `main()`，但包含大量：

- 全局变量；
- 地图生成函数；
- 里程计回调；
- 发布函数。

这些符号会进入 `mockamap_node`，只是主流程没有调用。

因此它不是“完全未编译的参考文件”，而是：

```text
已链接进节点、但当前没有执行入口的遗留实现。
```

---

## 7. 关键依赖

| 依赖 | 用途 |
|---|---|
| `roscpp` | 节点、参数、日志和发布循环 |
| `sensor_msgs` | `PointCloud2` |
| PCL | 点云容器和 KD 树 |
| `pcl_ros` | PCL 的 ROS 集成 |
| `pcl_conversions` | PCL 点云转 ROS 消息 |
| Eigen | 二维迷宫矩阵和三维点运算 |
| C++ STL | 排序、随机数、向量和递归 |

`package.xml` 使用旧式：

```text
build_depend
run_depend
```

主要依赖与 CMake 基本一致。

---

## 8. 编译配置

CMake 先后添加：

```cmake
add_compile_options(-std=c++11)
ADD_COMPILE_OPTIONS(-std=c++11)
ADD_COMPILE_OPTIONS(-std=c++14)
```

通常最终使用 C++14。

Release 标志：

```cmake
-O3 -Wall -g
```

即：

- 高等级优化；
- 常见警告；
- 保留调试符号。

重复设置 C++ 标准没有必要，建议改为：

```cmake
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
```

---

## 9. ROS 接口

### 9.1 发布话题

源码：

```cpp
nh.advertise<sensor_msgs::PointCloud2>(
    "mock_map", 1);
```

因为 `nh` 是普通根 NodeHandle，节点通常发布：

```text
/mock_map
```

消息类型：

```text
sensor_msgs/PointCloud2
```

### 9.2 订阅、服务和动作

主节点没有订阅任何话题，也没有服务或 action。

地图完全由启动参数决定。

### 9.3 坐标系

所有生成方法最终调用：

```cpp
output.header.frame_id = "world";
```

因此点云声明在：

```text
world
```

坐标系。

没有使用 TF。

---

## 10. 主节点完整流程

`mockamap.cpp::main()`：

```text
初始化 ROS 节点
        |
        v
创建 /mock_map 发布器
        |
        v
读取公共参数
        |
        v
将物理尺寸转换为离散格子数量
        |
        v
构造 Maps::BasicInfo
        |
        v
Maps::generate(type)
一次性生成地图
        |
        v
按 update_freq 循环发布同一消息
```

地图生成后不会随时间变化。

---

## 11. 公共参数

| 参数 | 默认值 | 含义 |
|---|---:|---|
| `seed` | `4546` | 随机种子 |
| `update_freq` | `1.0 Hz` | 静态地图重复发布频率 |
| `resolution` | `0.38 m` | 点云离散分辨率 |
| `x_length` | `100 m` | 地图 x 物理长度 |
| `y_length` | `100 m` | 地图 y 物理长度 |
| `z_length` | `10 m` | 地图 z 物理长度 |
| `type` | `1` | 地图类型 |

全部从私有参数空间读取：

```text
~seed
~resolution
...
```

如果节点名是 `/mockamap_node`，完整名称例如：

```text
/mockamap_node/resolution
```

---

## 12. `resolution`、`scale` 和格子数量

源码先把参数 `resolution` 存入变量 `scale`：

```cpp
nh_private.param("resolution", scale, 0.38);
```

随后：

```cpp
scale = 1 / scale;
```

所以类内部：

```text
info.scale = 1 / resolution
```

它实际是：

```text
每米格子数
```

例如：

```text
resolution = 0.1 m
scale      = 10 cells/m
```

再计算：

```cpp
sizeX = x_length * scale;
```

得到离散格子数。

因此：

```text
物理坐标 = 格子索引 / scale
         = 格子索引 × resolution
```

变量名 `scale` 容易与分辨率混淆。

---

## 13. `BasicInfo` 数据结构

```cpp
struct BasicInfo
{
  ros::NodeHandle* nh_private;
  int sizeX;
  int sizeY;
  int sizeZ;
  int seed;
  double scale;
  sensor_msgs::PointCloud2* output;
  pcl::PointCloud<pcl::PointXYZ>* cloud;
};
```

字段含义：

| 字段 | 含义 |
|---|---|
| `nh_private` | 各地图算法读取专属参数 |
| `sizeX/Y/Z` | 三个方向离散格子数 |
| `seed` | 可复现随机种子 |
| `scale` | 每米格子数，即 `1/resolution` |
| `output` | 最终 ROS 点云消息 |
| `cloud` | 生成中的 PCL 点云 |

`Maps` 通过：

```cpp
setInfo(info);
```

复制该结构。

其中两个指针仍指向 `main()` 栈上的对象；这些对象在主循环期间一直有效。

---

## 14. 地图类型分派

```cpp
void Maps::generate(int type)
{
  switch (type)
  {
    default:
    case 1: perlin3D(); break;
    case 2: randomMapGenerate(); break;
    case 3:
      srand(seed);
      maze2D();
      break;
    case 4:
      srand(seed);
      Maze3DGen();
      break;
  }
}
```

类型映射：

| `type` | 函数 | 地图 |
|---:|---|---|
| `1` | `perlin3D()` | 三维连续噪声障碍 |
| `2` | `randomMapGenerate()` | 随机空心方柱 |
| `3` | `maze2D()` | 挤出到固定高度的二维迷宫 |
| `4` | `Maze3DGen()` | 三维等距分割面迷宫 |
| 其他 | `perlin3D()` | 默认回退到类型 1 |

---

## 15. 点云输出的共同格式

多数算法最终调用：

```cpp
Maps::pcl2ros()
```

其中：

```cpp
pcl::toROSMsg(*info.cloud, *info.output);
info.output->header.frame_id = "world";
```

输出只有 XYZ。

通常设置：

```text
height = 1
width  = 点数
```

所以是无组织点云。

当前没有设置：

```cpp
header.stamp
```

时间戳通常为 0。

---

## 16. 类型 1：Perlin 三维地图总览

Perlin 地图流程：

```text
为每个三维格子计算多尺度噪声
        |
        v
将全部噪声值排序
        |
        v
根据 fill 选择分位数阈值
        |
        v
再次计算每个格子的噪声
        |
        v
噪声高于阈值的格子变成障碍点
```

它会生成连贯、自然、非规则的三维障碍结构。

与独立随机占据相比，Perlin 噪声相邻位置高度相关，因此会形成：

- 块状障碍；
- 洞穴；
- 连续墙体；
- 弯曲通道。

---

## 17. Perlin Noise 基础

`PerlinNoise::noise(x, y, z)` 的主要步骤：

1. 找到输入点所在整数单位立方体；
2. 计算点在立方体内的局部坐标；
3. 对局部坐标应用平滑函数；
4. 根据 permutation table 选择八个角的梯度；
5. 对八个角贡献做三线性平滑插值；
6. 将结果映射到约 `[0, 1]`。

平滑函数：

```text
fade(t) = 6t^5 - 15t^4 + 10t^3
```

源码：

```cpp
t * t * t * (t * (t * 6 - 15) + 10)
```

该函数在格子边界处一阶和二阶导数为 0，使噪声连续平滑。

---

## 18. Perlin permutation table

无参构造函数使用经典参考排列。

实际地图使用：

```cpp
PerlinNoise noise(info.seed);
```

带 seed 构造函数：

```text
生成 0...255
使用 seed 初始化随机引擎
shuffle
复制一遍形成 512 长度表
```

因此：

```text
相同 seed + 相同参数
应生成相同 Perlin 地图。
```

这比 `map_generator` 当前未使用 seed 的实现更适合可复现实验。

---

## 19. Perlin 专属参数

| 参数 | 默认值 | 作用 |
|---|---:|---|
| `complexity` | `0.142857` | 输入坐标频率缩放 |
| `fill` | `0.38` | 期望障碍格比例 |
| `fractal` | `1` | 叠加噪声层数 |
| `attenuation` | `0.5` | 每层振幅基准 |

默认 EGO-Planner 配置：

```text
complexity  = 0.05
fill        = 0.12
fractal     = 1
attenuation = 0.1
```

所以只使用一层较低频、较低填充率的噪声。

---

## 20. Perlin 多尺度叠加

对每个格子：

```cpp
for (int it = 1; it <= fractal; ++it)
{
  int dfv = pow(2, it);
  double ta = attenuation / it;
  tnoise += ta * noise.noise(
      dfv * i * complexity,
      dfv * j * complexity,
      dfv * k * complexity);
}
```

第 `it` 层：

```text
频率倍数 = 2^it
振幅权重 = attenuation / it
```

注意第一层从 `it=1` 开始，因此最低频率倍数已经是：

```text
2
```

而不是常见分形噪声实现中的 `1`。

`fractal` 增大时：

- 增加更高频细节；
- 计算量线性增加；
- 障碍边界更复杂。

---

## 21. `complexity` 的影响

噪声输入：

```text
2^it × index × complexity
```

`complexity` 小：

- 相邻格子噪声变化慢；
- 障碍团块较大；
- 通道更平滑。

`complexity` 大：

- 噪声变化快；
- 障碍更碎；
- 小结构更多；
- 规划环境更复杂。

它作用在离散索引上，而不是直接作用在米制坐标上。

所以改变 `resolution` 时，即使 `complexity` 不变，物理空间中的结构尺度也会改变。

---

## 22. `fill` 如何控制占据比例

第一遍计算全部：

```text
N = sizeX × sizeY × sizeZ
```

个噪声值，并排序。

阈值索引：

```cpp
tpos = N * (1 - fill);
```

阈值：

```cpp
tmp = sorted_noise[tpos];
```

第二遍保留：

```cpp
tnoise > tmp
```

的格子。

所以理论上约有：

```text
fill × N
```

个障碍点。

例如：

```text
fill = 0.12
```

表示约 12% 的格子被选为障碍。

实际比例会因相同噪声值和严格大于比较略有差异。

---

## 23. Perlin 为什么计算两遍

第一遍只为求全局分位数阈值。

第二遍才真正写入点坐标。

这种做法避免同时长期保存：

```text
每个格子的噪声值 + 点云
```

但当前仍保存了全部噪声值用于排序，并且第二遍重复完整计算。

复杂度约为：

```text
两次 O(N × fractal) 噪声计算
+ O(N log N) 排序
```

更高效的方案可以：

- 第一遍保存噪声数组并复用；
- 使用 `nth_element` 求分位数；
- 避免全排序；
- 并行计算噪声。

---

## 24. Perlin 坐标转换

选中格子的世界坐标：

```cpp
x = i / scale - sizeX / (2 * scale);
y = j / scale - sizeY / (2 * scale);
z = k / scale;
```

因此：

```text
x ∈ [-x_length/2, x_length/2)
y ∈ [-y_length/2, y_length/2)
z ∈ [0, z_length)
```

地图在 x、y 方向以原点为中心。

z 从 0 开始，不以原点居中。

这与 `Maze3DGen()` 的 z 坐标设计不同，后者在 z 方向也以 0 为中心。

---

## 25. 类型 2：随机方柱地图总览

类型 2：

```text
随机选择柱体中心
        |
        v
随机选择宽度和高度
        |
        v
离散出长方体边界点
        |
        v
重复 obstacle_number 次
```

对应 launch：

```text
post2d.launch
```

名称中的 `post` 表示柱状障碍物。

---

## 26. 类型 2 参数

| 参数 | 默认值 | 含义 |
|---|---:|---|
| `width_min` | `0.6 m` | 柱体宽度下限 |
| `width_max` | `1.5 m` | 柱体宽度上限 |
| `obstacle_number` | `10` | 柱体数量 |

中心范围：

```text
x ∈ [-x_length/2, x_length/2]
y ∈ [-y_length/2, y_length/2]
```

高度：

```text
h ∈ [0, z_length]
```

均匀采样。

---

## 27. 类型 2 实际生成的是空心表面

算法遍历柱体包围盒中的格子，但只在：

```cpp
(r - rl) * (r - rh + 1)
* (s - sl) * (s - sh + 1)
* t * (t - heiNum + 1) == 0
```

时加入点。

乘积为 0 表示至少一个维度位于边界：

```text
r 在左右边界
或 s 在前后边界
或 t 在上下边界
```

因此它生成：

```text
长方体外壳
```

而不是填满内部的实体柱体。

这样可以大幅减少点数。

对于占据地图而言，只要表面封闭且采样足够密，内部是否填充通常不重要。

---

## 28. 类型 2 的几何特点

当前实现：

- 柱体中心不吸附到栅格中心；
- 柱体可以相互重叠；
- 不保留起点安全区；
- 不检查最小间距；
- 边缘可以超出地图范围；
- 高度可能接近 0；
- 宽度和高度独立均匀采样。

它适合快速生成柱林，但不保证存在可行通道。

高障碍密度时，规划任务可能无解。

---

## 29. 类型 3：二维迷宫总览

二维迷宫先在较粗的逻辑网格中生成墙：

```text
物理地图
        |
        v
按 road_width 划分迷宫单元
        |
        v
递归分割矩阵
        |
        v
值为 1 的单元变成墙
        |
        v
墙沿 z 方向挤出到 z_length
```

它叫二维迷宫，是因为拓扑只在 x-y 平面生成。

最终输出仍是三维墙体点云。

---

## 30. 类型 3 参数

| 参数 | 默认值 | 含义 |
|---|---:|---|
| `road_width` | `1.0 m` | 一个迷宫逻辑格的物理宽度 |
| `add_wall_x` | `0` | 是否添加一组外边界墙 |
| `add_wall_y` | `0` | 是否添加另一组外边界墙 |
| `maze_type` | `1` | 迷宫算法，目前只实现类型 1 |

逻辑迷宫尺寸：

```cpp
mx = sizeX / (road_width * scale);
my = sizeY / (road_width * scale);
```

因为：

```text
sizeX / scale = x_length
```

所以概念上：

```text
mx ≈ x_length / road_width
my ≈ y_length / road_width
```

---

## 31. 递归分割迷宫原理

`recursiveDivision()` 对当前矩形区域：

1. 随机选择内部 x 分割位置；
2. 随机选择内部 y 分割位置；
3. 画一条横墙和一条竖墙，形成十字；
4. 在四个墙段中选择三个开门；
5. 将区域分成四个子区域；
6. 对四个子区域递归。

核心思想：

```text
四个分区之间只保留三个连接门
```

可以减少闭环并形成迷宫结构。

---

## 32. 为什么四段墙只开三个门

十字墙把区域分成四部分。

若四段都开门，容易形成环路。

若少于三个门，至少一个区域可能与其他区域断开。

选择三个门相当于在四个区域之间建立一棵连接树：

```text
四个节点需要三条边才能连通且无环。
```

这是递归分割迷宫的经典思路。

---

## 33. 小区域特殊处理

`recursiveDivision()` 针对不同剩余尺寸分别处理：

```text
至少 5×5：十字墙并递归四个子区
至少 4×4：十字墙但不继续递归
3×4+：添加一条纵向墙
4+×3：添加一条横向墙
3×3：只添加中心墙点
更小：结束
```

这些分支用于避免：

- `rand() % 0`；
- 墙无处放置；
- 子区域尺寸非法。

实现较长，也包含大量调试日志和矩阵打印。

---

## 34. 二维迷宫墙体点云化

若：

```cpp
maze(i, j) == 1
```

则在该逻辑格覆盖的每个细分点上生成：

```text
x = i × road_width
  + ii / scale
  - x_length/2

y = j × road_width
  + jj / scale
  - y_length/2

z = k / scale
```

其中：

```text
ii, jj 覆盖一个 road_width 方块
k 覆盖完整 z_length
```

因此每个迷宫墙逻辑单元会变成：

```text
厚度约 road_width、高度 z_length 的实体墙柱。
```

与类型 2 不同，这里墙内部是填充的。

---

## 35. `add_wall_x` 和 `add_wall_y`

源码：

```cpp
if (addWallX)
{
  maze(i, 0) = 1;
  maze(i, my - 1) = 1;
}
```

这是固定 y 边界，即两条沿 x 方向延伸的墙。

```cpp
if (addWallY)
{
  maze(0, i) = 1;
  maze(mx - 1, i) = 1;
}
```

这是固定 x 边界，即两条沿 y 方向延伸的墙。

变量名可以理解为：

```text
墙延伸的方向
```

而不是固定坐标轴。

---

## 36. 类型 4：三维迷宫总览

三维迷宫不是把二维迷宫简单堆叠。

它先随机生成若干三维核心点：

```text
seed points / sites
```

然后对每个三维格子寻找最近和第二近的核心点。

若两者距离接近：

```text
该格子位于两个 Voronoi 区域的分界面附近
```

就将其作为墙。

所以类型 4 本质上接近：

```text
三维 Voronoi 分区边界点云。
```

---

## 37. `MazePoint` 的作用

每个测试格子临时使用：

```cpp
MazePoint mp;
```

记录：

| 字段 | 含义 |
|---|---|
| `point` | 当前格子的世界坐标 |
| `dist1` | 到最近核心点的距离 |
| `dist2` | 到第二近核心点的距离 |
| `point1` | 最近核心点编号 |
| `point2` | 第二近核心点编号 |
| `isdoor` | 声明但未使用 |

遍历全部核心点后即可判断当前格子是否靠近 Voronoi 分界面。

---

## 38. 类型 4 核心点生成

参数：

| 参数 | 默认值 | 含义 |
|---|---:|---|
| `numNodes` | `10` | 三维核心点数量 |
| `connectivity` | `0.5` | 哪些区域边界尝试开洞 |
| `nodeRad` | `3` | 读取但未使用 |
| `roadRad` | `2` | 边界开口判据尺度 |

核心点坐标在三个方向都以 0 为中心：

```text
x ∈ 约 [-x_length/2, x_length/2]
y ∈ 约 [-y_length/2, y_length/2]
z ∈ 约 [-z_length/2, z_length/2]
```

这与其他三种地图的：

```text
z 从 0 开始
```

不一致。

---

## 39. Voronoi 边界判断

对每个格子计算最近两点距离：

```text
d1 <= d2
```

若：

```cpp
abs(d2 - d1) < 1 / scale
```

则认为该格子接近两个核心点的等距面。

因为：

```text
1 / scale = resolution
```

所以墙厚大约由一个分辨率带控制。

这会生成把空间划分成多个区域的薄墙面。

---

## 40. 类型 4 的“连通性”处理

源码用最近两个核心点编号之和判断：

```cpp
point1 + point2
```

是否位于：

```text
(1-connectivity) × numNodes
到
(1+connectivity) × numNodes
```

之间。

若在范围内，将该边界视为：

```text
holed wall
```

并根据：

```text
d1 + d2 - 核心点间距
```

决定是否保留墙点。

从几何上：

```text
d1 + d2 - distance(site1, site2)
```

衡量当前点偏离两核心连线的程度。

靠近连线的点更可能被删除，从而在分界墙上形成通道。

---

## 41. `roadRad` 的作用

开洞条件：

```cpp
d1 + d2 - judge
    >= roadRad / (scale * 3)
```

满足时才保留墙点。

所以不满足条件的靠近核心连线区域会成为空洞。

一般而言：

- `roadRad` 增大，右侧阈值增大；
- 更多点不满足保留条件；
- 通道可能更宽。

但它并不是直接以米为单位的严格半径，公式还除以 3，属于经验参数。

---

## 42. `nodeRad` 当前没有作用

`Maze3DGen()` 读取并打印：

```cpp
nodeRad
```

但之后没有使用。

修改 launch 中的：

```text
nodeRad
```

不会改变地图。

这是一个遗留或尚未实现的参数。

---

## 43. 地图发布机制

地图生成后已经保存在：

```cpp
sensor_msgs::PointCloud2 output;
```

主循环只执行：

```cpp
pcl_pub.publish(output);
```

所以与 `map_generator` 不同，`mockamap` 不会每帧重复：

```text
PCL -> ROS
```

转换。

它仍然按：

```text
update_freq
```

重复发布静态地图，以保证晚启动订阅者可以收到。

---

## 44. 默认启动链路

默认 `simulator.xml`：

```text
mockamap_node
  seed = 127
  update_freq = 0.5 Hz
  resolution = 0.1 m
  type = 1
  complexity = 0.05
  fill = 0.12
  fractal = 1
  attenuation = 0.1
```

地图尺寸来自：

```text
map_size_x
map_size_y
map_size_z
```

输出重映射：

```text
/mock_map
    ->
/map_generator/global_cloud
```

`local_sensing` 订阅后只接收第一帧并构建自己的 KD 树。

---

## 45. 独立 launch 文件

| launch | 类型 | 主要用途 |
|---|---:|---|
| `mockamap.launch` | `1` | 综合参数示例，默认 Perlin |
| `perlin3d.launch` | `1` | 大型 Perlin 三维地图 |
| `post2d.launch` | `2` | 随机柱状地图 |
| `maze2d.launch` | `3` | 二维递归分割迷宫 |
| `maze3d.launch` | `4` | 三维 Voronoi 迷宫 |
| `yxc_test_*.launch` | 对应类型 | 使用简化 RViz 配置的测试版本 |

独立运行示例：

```bash
roslaunch mockamap perlin3d.launch
roslaunch mockamap post2d.launch
roslaunch mockamap maze2d.launch
roslaunch mockamap maze3d.launch
```

---

## 46. RViz 配置

简化配置主要显示：

```text
/mock_map
```

`rviz.rviz` 还包含：

```text
/mock_map/cut
```

相关显示或裁剪插件配置。

当前源码本身不发布：

```text
/mock_map/cut
```

所以该话题需要外部工具或插件生成。

---

## 47. `optimizeMap()` 的设计目的

`mockamap.cpp` 中存在全局函数：

```cpp
optimizeMap(info);
```

但调用被注释。

它的目标是删除实体障碍内部点，只保留表面。

流程：

```text
为点云建立 KD 树
        |
        v
对每个点搜索 sqrt(3)×resolution 邻域
        |
        v
若邻居数 >= 27
认为完整 3×3×3 邻域都被占据
        |
        v
删除该内部点
```

半径：

```cpp
1.75 / scale
```

其中 `1.75` 是略大于：

```text
sqrt(3) ≈ 1.732
```

的值。

---

## 48. 为什么默认关闭优化

该实现对每个点做 KD 树半径搜索，然后使用多次：

```cpp
vector.erase()
```

删除点。

即使从后向前删除，连续数组移动仍可能产生较大成本。

对于大 Perlin 地图：

- 点数可能很高；
- 每点 KD 查询成本显著；
- 删除过程也较慢。

而后续 `local_sensing` 本身会下采样，`GridMap` 也只需要障碍表面信息。

可能因此默认没有启用该优化。

---

## 49. `ces_randommap.cpp` 的定位

该文件是一套遗留的固定走廊地图与局部感知服务器代码。

它包含：

- `fixedMapGenerate()`；
- 固定长方体障碍列表；
- 全局地图发布；
- 基于 KD 树的局部点云；
- odometry 队列；
- 前 5～7 秒有限次发布全局地图。

但是主节点：

```cpp
mockamap.cpp::main()
```

没有调用其中任何函数。

因此默认运行行为完全由：

```text
maps.cpp
perlinnoise.cpp
mockamap.cpp
```

决定。

---

## 50. 固定地图遗留实现

`fixedMapGenerate()` 手工定义多个长方体：

```text
若干竖直隔墙
两条沿 x 方向的长边界墙
```

分辨率固定：

```text
1.0 m
```

同类型 2 一样，只生成长方体表面。

它看起来用于特定走廊或穿门实验，并不受主节点参数控制。

---

## 51. 与 `map_generator` 的对比

| 特性 | `mockamap` | `map_generator` |
|---|---|---|
| 当前默认使用 | 是 | 否 |
| 地图类型 | 4 种 | 圆柱/方柱加椭圆环 |
| 固定 seed | 支持 | 当前参数未接入 |
| Perlin 地图 | 支持 | 不支持 |
| 2D 迷宫 | 支持 | 不支持 |
| 3D 迷宫 | 支持 | 不支持 |
| 穿环场景 | 无专门环 | 支持 |
| 主输出 | `/mock_map` | `/map_generator/global_cloud` |
| 默认链路 | remap 到 global_cloud | 节点被禁用 |

两者是可替换地图源，不应同时向同一个全局地图话题发布。

---

## 52. 当前源码中的重要风险与注意事项

以下结论针对当前工作空间源码。

### 52.1 `resolution = 0` 会除零

```cpp
scale = 1 / scale;
```

没有检查输入分辨率是否大于 0。

应要求：

```text
resolution > 0
```

### 52.2 地图尺寸使用 `int` 接收

```cpp
int sizeX;
nh_private.param("x_length", sizeX, 100);
```

所以源码期望长度参数为整数。

若用户提供带小数的地图长度，ROS 参数类型可能不匹配，或无法表达亚米级长度。

### 52.3 离散格子数量会截断

```cpp
sizeX = sizeX * scale;
```

右侧是 double，赋给 int 会截断。

例如某些分辨率不能整除地图长度时，实际物理地图略小于请求尺寸。

### 52.4 点云时间戳为 0

只设置 `frame_id`，没有设置 `header.stamp`。

对于静态地图通常能工作，但不利于 rosbag 和同步调试。

### 52.5 发布器不是 latched

静态地图只能依赖循环重复发布。

使用 latched publisher 可以只发布一次。

### 52.6 `update_freq <= 0` 未校验

无效频率可能导致 `ros::Rate` 行为异常。

### 52.7 Perlin `fill` 未限制在 `[0,1]`

阈值位置：

```cpp
tpos = N * (1 - fill);
v->at(tpos);
```

若：

- `fill < 0`；
- `fill > 1`；

索引可能越界或语义异常。

即使 `fill = 0`：

```text
tpos = N
```

也会越界。

安全范围实际应接近：

```text
0 < fill <= 1
```

并专门处理边界。

### 52.8 Perlin `fractal <= 0` 会生成常数噪声

循环不执行，所有 `tnoise = 0`。

排序和阈值仍继续，最终可能生成空地图。

应校验：

```text
fractal >= 1
```

### 52.9 Perlin 噪声计算两遍

对大地图代价明显。

可以保存结果或使用更合适的分位数算法。

### 52.10 Perlin 使用完整排序

只需要一个分位数，却执行：

```cpp
std::sort()
```

可改为：

```cpp
std::nth_element()
```

将平均复杂度降至 O(N)。

### 52.11 Perlin 噪声数组使用裸 `new`

```cpp
std::vector<double>* v =
    new std::vector<double>;
```

函数结束前没有：

```cpp
delete v;
```

因此发生内存泄漏。

应直接使用栈对象：

```cpp
std::vector<double> v;
```

### 52.12 `pcl2ros()` 的百分比日志少乘 100

日志：

```cpp
ROS_INFO("finish: infill %lf%%",
  width / total_voxels);
```

若占据率为 0.12，它会打印：

```text
0.12%
```

但真正百分比是：

```text
12%
```

应乘 100，或删除 `%` 符号。

### 52.13 类型 2 参数未校验

需要检查：

```text
0 <= width_min <= width_max
obstacle_number >= 0
z_length > 0
```

### 52.14 类型 2 可能生成零高度障碍

高度分布从 0 开始。

当高度很小时：

```text
heiNum = 0
```

不会生成任何点，但仍消耗一个障碍编号。

### 52.15 类型 2 不保证起点安全

随机障碍可能生成在无人机初始位置附近。

默认 EGO-Planner 使用 Perlin 地图，也同样没有显式起点清空。

### 52.16 类型 2 障碍可能重叠或越界

没有中心间距和边界裕量检查。

### 52.17 `road_width` 可能导致除零或零尺寸矩阵

```cpp
mx = sizeX / (road_width * scale);
```

若 `road_width <= 0` 或比地图大，迷宫尺寸可能非法。

### 52.18 `road_width * scale` 被用于 int 循环上限

```cpp
ii < width * scale
```

右侧是 double。

若道路宽度不是分辨率的整数倍，实际墙厚通过比较隐式取整，行为不够明确。

### 52.19 二维迷宫索引维度使用混乱

矩阵构造：

```cpp
Eigen::MatrixXi maze(mx, my);
```

代码中有时将：

```text
rows/cols
```

与：

```text
x/y
```

交叉使用。

非方形地图需要特别测试，可能暴露越界或转置逻辑问题。

### 52.20 递归迷宫输出大量日志

每次递归都：

- 输出 ROS_INFO；
- 打印完整矩阵。

大地图会产生大量终端输出并显著拖慢生成。

### 52.21 `recursizeDivisionMaze()` 标记为有 bug

源码直接写：

```cpp
//! @todo all bugs here...
```

当前 `maze2D()` 不调用它，而是调用另一套 `recursiveDivision()`。

该遗留函数不应在未测试前启用。

### 52.22 `recursizeDivisionMaze()` 有可疑 block 尺寸

例如：

```cpp
maze.block(px, py, sy - px, sy - py)
```

行尺寸使用了 `sy - px`，很可能应与 `sx` 有关。

这与源码的 bug 注释一致。

### 52.23 3D 迷宫复杂度很高

对每个体素都遍历全部核心点：

```text
O(sizeX × sizeY × sizeZ × numNodes)
```

`20 m × 20 m × 20 m`、`0.1 m` 分辨率意味着：

```text
200 × 200 × 200 = 8,000,000 个格子
```

再乘 `64` 个核心点约为：

```text
5.12 亿次距离计算
```

生成可能非常慢。

### 52.24 3D 迷宫每次距离都调用 `sqrt`

寻找最近点时可比较平方距离，减少开方成本。

### 52.25 `nodeRad` 参数无效

读取后完全没有参与算法。

### 52.26 `connectivity` 未限制范围

若不在 `[0,1]`，编号和区间判据会产生难以理解的结果。

### 52.27 3D 迷宫的连通性判据依赖编号

是否开洞取决于：

```text
point1 + point2
```

而核心点编号本身没有几何意义。

这不是严格的图连通性控制，参数效果可能不直观。

### 52.28 类型 4 z 坐标系与其他类型不一致

Perlin、柱体和 2D 迷宫：

```text
z 从 0 到 z_length
```

3D 迷宫：

```text
z 以 0 为中心
```

切换地图类型时无人机与地面关系会变化。

### 52.29 `MazePoint::isdoor` 未初始化且未使用

该字段可以删除或补齐功能。

### 52.30 头文件声明了未实现的成员函数

`Maps` 私有声明：

```cpp
void optimizeMap();
```

但没有对应：

```cpp
Maps::optimizeMap()
```

真正实现的是 `mockamap.cpp` 中的全局函数。

当前未调用成员函数，所以不会链接失败，但接口具有误导性。

### 52.31 全局 `optimizeMap()` 默认关闭

地图可能包含大量内部点。

开启前又需考虑其性能和删除实现。

### 52.32 优化函数逐点 `erase` 效率低

即使逆序删除，每次仍可能移动后续元素。

建议构建新的表面点云，而不是原地多次擦除。

### 52.33 优化函数的 27 邻居判据依赖规则格点

它假设点云是完整 3×3×3 体素邻域。

对类型 2 的随机浮点中心、迷宫边界或重复点，效果可能不稳定。

### 52.34 CMake `GLOB` 会自动带入遗留源码

新增任何 `.cpp` 都会进入同一节点。

可能引入：

- 重复 `main()`；
- 全局符号冲突；
- 不需要的依赖；
- 隐式行为变化。

建议显式列出源文件。

### 52.35 `ces_randommap.cpp` 包含未使用全局状态

增加编译时间、二进制体积和维护负担。

更适合移出主目标或建立独立示例节点。

### 52.36 同时指定 C++11 和 C++14

构建配置冗余。

### 52.37 默认回退掩盖错误类型

任何非法 `type` 都静默生成 Perlin 地图。

更利于调试的做法是：

```text
输出错误并退出
```

或明确警告后回退。

### 52.38 没有清理起点附近障碍

默认 Perlin 地图可能让初始位置落在或靠近障碍物。

完整仿真是否成功依赖 seed 和参数。

### 52.39 没有地图重新生成接口

参数只在启动时读取。

要改变地图必须重启节点。

---

## 53. 推荐调试方法

### 53.1 确认地图节点

```bash
rosnode info /mockamap_node
```

确认发布：

```text
/mock_map
```

完整仿真中因重映射，应查看：

```text
/map_generator/global_cloud
```

### 53.2 确认真正发布者

```bash
rostopic info /map_generator/global_cloud
```

应看到：

```text
/mockamap_node
```

### 53.3 查看频率

独立 launch：

```bash
rostopic hz /mock_map
```

完整仿真：

```bash
rostopic hz /map_generator/global_cloud
```

### 53.4 查看点数

```bash
rostopic echo -n 1 /mock_map/width
```

无组织点云中：

```text
width = 点数
```

### 53.5 检查坐标范围

将点云保存为 PCD：

```bash
rosrun pcl_ros pointcloud_to_pcd \
  input:=/mock_map
```

使用 PCL 工具或脚本统计：

```text
min/max x
min/max y
min/max z
```

特别对比类型 4 的 z 范围。

### 53.6 查看参数

```bash
rosparam get /mockamap_node
```

确认：

- `type`；
- `resolution`；
- `fill`；
- 地图尺寸；
- 专属参数。

### 53.7 检查可复现性

使用同一 seed 连续启动两次并保存点云。

点顺序和坐标应一致。

改变 seed 后地图应变化。

### 53.8 Perlin 生成性能

记录从节点启动到出现：

```text
finish: infill
```

日志的时间。

依次改变：

- 分辨率；
- 地图体积；
- fractal；
- fill。

---

## 54. 常见故障

### 54.1 节点启动后长时间没有点云

地图是在第一次发布前同步生成的。

可能是：

- 地图体积太大；
- 分辨率太小；
- 3D 迷宫核心点太多；
- 递归迷宫大量打印；
- Perlin 排序耗时。

### 54.2 节点崩溃并出现越界

优先检查：

- `fill` 是否为 0 或超出 `[0,1]`；
- `resolution` 是否大于 0；
- `road_width` 是否合理；
- 非方形迷宫尺寸；
- 地图长度参数类型。

### 54.3 地图发布了但 EGO-Planner 收不到

独立运行时输出是：

```text
/mock_map
```

而默认 `local_sensing` 订阅：

```text
/map_generator/global_cloud
```

需要重映射：

```xml
<remap from="/mock_map"
       to="/map_generator/global_cloud"/>
```

### 54.4 修改 `nodeRad` 没效果

这是当前源码预期现象，因为参数没有被使用。

### 54.5 3D 迷宫出现在地下

类型 4 的 z 坐标以 0 为中心。

其他地图类型则从 z=0 向上生成。

### 54.6 Perlin 占据率日志不对

日志带 `%`，但源码没有乘 100。

例如打印 `0.12%` 实际约表示 12%。

### 54.7 相同参数地图不同

检查 seed 是否真的相同，以及是否使用同一种标准库实现环境。

标准 C++ 的随机引擎通常可复现，但跨编译器/标准库不一定保证完全相同 shuffle 结果。

### 54.8 迷宫终端刷屏

`recursiveDivision()` 会打印大量矩阵和日志。

这不是 ROS 日志等级配置能完全解决的，因为还使用了：

```cpp
std::cout
```

---

## 55. 推荐改进方向

### 55.1 低风险修复

优先建议：

1. 校验 `resolution > 0`；
2. 校验 `update_freq > 0`；
3. 限制 `fill`；
4. 限制 `fractal >= 1`；
5. 修复 Perlin 噪声向量泄漏；
6. 修复占据率日志；
7. 设置点云时间戳；
8. 删除无效 `nodeRad` 或实现其功能；
9. 删除未实现的成员 `optimizeMap()` 声明；
10. 统一 C++ 标准。

### 55.2 使用 latched publisher

静态地图更适合：

```cpp
advertise<PointCloud2>("mock_map", 1, true);
```

生成后发布一次，然后：

```cpp
ros::spin();
```

### 55.3 优化 Perlin 阈值计算

推荐：

```text
一次计算并保存噪声
        |
        v
nth_element 求分位数
        |
        v
遍历保存值生成点云
```

避免重复噪声计算和全排序。

### 55.4 并行化体素计算

Perlin 和 3D 迷宫中每个格子的计算高度独立。

可考虑：

- OpenMP；
- TBB；
- CUDA；
- 分块线程池。

### 55.5 3D 最近点查询优化

当前每个格子遍历所有核心点。

可为核心点建立 KD 树，一次查询最近两个邻居：

```text
O(Nvoxels × log(numNodes))
```

替代：

```text
O(Nvoxels × numNodes)
```

### 55.6 统一坐标约定

所有地图类型都采用：

```text
x/y 以 0 为中心
z 从 ground_height 向上
```

会更容易接入 `GridMap` 和无人机初始状态。

### 55.7 增加安全区

支持起点、终点或航点清空：

```text
clear_spheres
clear_boxes
```

避免随机地图直接堵住任务端点。

### 55.8 明确地图类型枚举

用枚举和字符串替代魔法数字：

```text
perlin3d
random_posts
maze2d
maze3d
```

非法值应输出错误。

### 55.9 拆分遗留代码

将 `ces_randommap.cpp`：

- 移出主可执行目标；
- 或做成独立 `fixed_map_server`；
- 或删除未使用实现。

### 55.10 增加单元测试

至少测试：

- 相同 seed 生成相同点云；
- 点坐标在预期边界；
- `fill` 近似正确；
- 二维迷宫连通；
- 类型 2 表面点判据；
- 类型 4 参数边界；
- 非方形地图不越界。

---

## 56. 推荐源码阅读顺序

### 第一阶段：理解节点入口

阅读：

```text
src/mockamap.cpp
```

重点看：

- 参数读取；
- `resolution -> scale`；
- `BasicInfo`；
- `map.generate(type)`；
- 发布循环。

### 第二阶段：理解公共接口

阅读：

```text
include/maps.hpp
```

掌握：

- `BasicInfo`；
- `Maps`；
- `MazePoint`；
- 四类生成函数。

### 第三阶段：理解默认 Perlin 地图

阅读：

```text
Maps::perlin3D()
perlinnoise.hpp
perlinnoise.cpp
```

重点理解：

- 多尺度噪声；
- 分位数阈值；
- fill；
- seed；
- 坐标转换。

### 第四阶段：理解随机柱体

阅读：

```text
Maps::randomMapGenerate()
```

重点看长方体表面判据。

### 第五阶段：理解二维迷宫

阅读：

```text
Maps::maze2D()
Maps::recursiveDivision()
```

先理解逻辑矩阵，再看点云挤出。

### 第六阶段：理解三维迷宫

阅读：

```text
MazePoint
Maps::Maze3DGen()
```

重点理解最近两核心点和等距面。

### 第七阶段：最后看遗留代码

阅读：

```text
optimizeMap()
ces_randommap.cpp
recursizeDivisionMaze()
```

这些不是默认主链路。

---

## 57. 建议学习实验

### 实验一：四种地图逐一运行

分别运行：

```bash
roslaunch mockamap perlin3d.launch
roslaunch mockamap post2d.launch
roslaunch mockamap maze2d.launch
roslaunch mockamap maze3d.launch
```

比较：

- 生成耗时；
- 点数；
- 空间结构；
- 规划难度。

### 实验二：验证 seed

固定全部参数，只改变 seed。

保存 PCD 并比较。

### 实验三：Perlin `fill`

尝试：

```text
0.05
0.12
0.25
0.40
```

记录实际点数比例和规划成功率。

### 实验四：Perlin `complexity`

尝试：

```text
0.02
0.05
0.10
0.20
```

观察障碍团块尺度和通道曲折程度。

### 实验五：Perlin 分形层数

比较：

```text
fractal = 1, 2, 3
```

记录生成时间和边界细节。

### 实验六：分辨率缩放

使用同一物理尺寸和 seed：

```text
0.20 m
0.10 m
0.05 m
```

观察点数近似如何随三次方增长。

### 实验七：随机柱体表面

只生成一个较大的柱体。

从 RViz 剖面观察内部是否为空。

### 实验八：二维迷宫连通性

将迷宫矩阵导出，使用 BFS 验证所有自由区域是否连通。

### 实验九：非方形迷宫

设置：

```text
x_length != y_length
```

测试递归索引是否安全。

### 实验十：3D 迷宫性能

逐步增加：

```text
numNodes
地图尺寸
分辨率
```

记录生成耗时。

### 实验十一：启用 `optimizeMap`

比较优化前后：

- 点数；
- 生成耗时；
- 内存；
- RViz 外观；
- `local_sensing` 性能。

### 实验十二：验证 z 坐标差异

分别生成类型 1 和类型 4，统计 z 最小值。

验证：

```text
type 1: z_min ≈ 0
type 4: z_min ≈ -z_length/2
```

---

## 58. 常见问题

### Q1：为什么叫 `mockamap`？

“mock a map”表示程序化构造一个用于测试的模拟地图。

### Q2：当前 EGO-Planner 默认用哪一种？

使用：

```text
type = 1
```

即三维 Perlin 地图。

### Q3：地图会动态变化吗？

不会。

只在节点启动时生成一次，之后重复发布同一消息。

### Q4：为什么还要重复发布静态地图？

发布器不是 latched，重复发布可让晚启动订阅者收到地图。

### Q5：`scale` 是分辨率吗？

不是。

类内部：

```text
scale = 1 / resolution
```

表示每米格子数。

### Q6：`fill=0.12` 是 0.12% 吗？

不是。

算法语义约为 12% 占据。日志中的百分号输出有误。

### Q7：类型 2 是实体柱体吗？

不是。

只生成长方体外表面点。

### Q8：二维迷宫为什么仍有 z_length？

二维只指拓扑在 x-y 平面生成。

墙会沿 z 方向挤出成为三维点云。

### Q9：三维迷宫是什么算法？

它接近随机核心点的三维 Voronoi 分区边界，并在部分边界上留洞。

### Q10：`nodeRad` 为什么改了没效果？

因为当前只读取和打印，没有参与计算。

### Q11：为什么 type 4 有负 z？

它在 z 方向也以原点为中心，而其他类型从 z=0 向上生成。

### Q12：`optimizeMap` 是否必须开启？

不是。

它用于删除实体内部点，但当前实现可能较慢，默认关闭。

### Q13：独立启动后为什么 EGO-Planner 收不到？

因为独立输出 `/mock_map`，而规划仿真链路期望 `/map_generator/global_cloud`，需要重映射。

### Q14：能否同时启动 `map_generator`？

可以使用不同话题进行比较，但不应让两者同时向同一个全局地图话题发布。

---

## 59. 总结

`mockamap` 是当前 EGO-Planner 默认使用的程序化地图生成包。

它完成：

```text
读取地图尺寸、分辨率、随机种子和类型参数，
一次性生成完整三维障碍物点云，
随后周期性发布给局部感知模拟器。
```

四种地图：

```text
type 1：Perlin 噪声三维障碍
type 2：随机空心方柱
type 3：递归分割二维迷宫墙
type 4：Voronoi 风格三维迷宫
```

默认系统链路：

```text
mockamap/type 1
  -> /mock_map
  -> remap /map_generator/global_cloud
  -> local_sensing
  -> /pcl_render_node/cloud
  -> GridMap
  -> EGO-Planner
```

理解本包时最重要的七条主线是：

1. **当前默认地图源就是 `mockamap`。**
2. **`resolution` 会被转换成 `scale = 1/resolution`。**
3. **地图通过 `type` 在四种算法间切换。**
4. **默认 Perlin 地图通过分位数阈值精确近似控制占据率。**
5. **二维迷宫是递归分割矩阵沿 z 方向挤出。**
6. **三维迷宫基于最近两个随机核心点的等距面。**
7. **地图静态生成一次，之后只是重复发布同一 ROS 消息。**

当前实现功能丰富，但在：

- 参数边界检查；
- Perlin 内存和性能；
- 3D 迷宫计算复杂度；
- 坐标约定统一；
- 遗留源码隔离；
- 无效参数；
- 时间戳与静态发布方式；

方面仍有清晰的改进空间。

结合：

```text
map_generator
local_sensing
plan_env
plan_manage/launch
```

一起阅读，就能完整理解 EGO-Planner 从程序化全局环境到无人机局部占据地图的整个仿真感知链路。
