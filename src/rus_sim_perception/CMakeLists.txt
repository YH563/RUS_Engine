cmake_minimum_required(VERSION 3.18)
project(rus_sim_perception)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED On)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ROS2 ament 构建
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rus_sim_interfaces REQUIRED)
find_package(rus_sim_utils REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)

# ── 感知核心库（感知节点 + 算法模块）──
# 节点仅做通信与调度；算法分层：
#   components/  公共组件（位姿插值、传感器帧编码）
#   pointcloud/  点云算法子模块（变换、滤波、建图）
add_library(perception_core SHARED
  src/rus_sim_perception/perception_node.cpp
  src/components/pose_interpolator.cpp
  src/components/sensor_encoder.cpp
  src/pointcloud/spatial_transformer.cpp
  src/pointcloud/cloud_filter_pipeline.cpp
  src/pointcloud/cloud_io.cpp
  src/pointcloud/map_manager.cpp
)

target_include_directories(perception_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
)
ament_target_dependencies(perception_core PUBLIC
  rclcpp std_msgs sensor_msgs geometry_msgs rus_sim_interfaces rus_sim_utils pcl_conversions)
target_link_libraries(perception_core PUBLIC Eigen3::Eigen ${PCL_LIBRARIES})

# ── zstd（压缩编码依赖；libzstd-dev 提供头文件与动态库）──
find_path(ZSTD_INCLUDE_DIR zstd.h)
find_library(ZSTD_LIBRARY zstd)
if(NOT ZSTD_INCLUDE_DIR OR NOT ZSTD_LIBRARY)
  message(FATAL_ERROR "libzstd-dev not found (zstd.h / libzstd)")
endif()
target_include_directories(perception_core PUBLIC ${ZSTD_INCLUDE_DIR})
target_link_libraries(perception_core PUBLIC ${ZSTD_LIBRARY})

# ── 可执行节点（仅入口，节点实现在 perception_core 库中）──
add_executable(rus_sim_perception_node src/main.cpp)
target_link_libraries(rus_sim_perception_node PUBLIC perception_core)
ament_target_dependencies(rus_sim_perception_node PUBLIC rclcpp)

# ── 测试点云生成工具（离线联调：生成 test_cloud.pcd）──
add_executable(rus_sim_gen_test_cloud src/tools/gen_test_cloud.cpp)
target_link_libraries(rus_sim_gen_test_cloud PUBLIC perception_core)
ament_target_dependencies(rus_sim_gen_test_cloud PUBLIC rclcpp)

# ── 安装规则 ──
install(TARGETS perception_core
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)
install(TARGETS rus_sim_perception_node DESTINATION lib/${PROJECT_NAME})
install(TARGETS rus_sim_gen_test_cloud DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch
  PATTERN "__pycache__" EXCLUDE)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)

ament_package()
