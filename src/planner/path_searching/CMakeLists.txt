# 最低CMake版本要求
cmake_minimum_required(VERSION 2.8.3)
# 工程名称
project(path_searching)

# 设置编译类型为Release，优化编译参数
set(CMAKE_BUILD_TYPE "Release")
# 添加C++11和C++14编译选项（建议只保留一个，实际以最后一个为准）
ADD_COMPILE_OPTIONS(-std=c++11 )
ADD_COMPILE_OPTIONS(-std=c++14 )
# Release模式下的编译参数：O3优化，Wall显示所有警告，g生成调试信息
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

# 查找Eigen3库（线性代数）
find_package(Eigen3 REQUIRED)
# 查找PCL库（点云处理）
find_package(PCL 1.7 REQUIRED)

# 查找catkin及其依赖功能包
find_package(catkin REQUIRED COMPONENTS
  roscpp           # ROS C++接口
  rospy            # ROS Python接口
  std_msgs         # 标准消息类型
  visualization_msgs # 可视化消息类型
  plan_env         # 依赖的自定义地图环境包
  cv_bridge        # OpenCV与ROS图像转换
)

# 定义catkin包参数
catkin_package(
 INCLUDE_DIRS include           # 头文件路径
 LIBRARIES path_searching       # 生成的库名
 CATKIN_DEPENDS plan_env        # 依赖的catkin包
#  DEPENDS system_lib           # 可选，依赖系统库
)

# 设置头文件搜索路径
include_directories( 
    SYSTEM 
    include                    # 本包头文件
    ${catkin_INCLUDE_DIRS}     # catkin依赖包头文件
    ${Eigen3_INCLUDE_DIRS}     # Eigen3头文件
    ${PCL_INCLUDE_DIRS}        # PCL头文件
)

# 编译静态/动态库（path_searching），源文件为src/dyn_a_star.cpp
add_library( path_searching 
    src/dyn_a_star.cpp
    )
# 链接catkin依赖库
target_link_libraries( path_searching
    ${catkin_LIBRARIES} 
    )