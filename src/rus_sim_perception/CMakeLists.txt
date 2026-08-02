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
find_package(rus_sim_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)

# ── 感知核心库（点云 / 视觉 / 融合组件）──
add_library(perception_core SHARED
  src/pointcloud_component.cpp
  src/vision_component.cpp
  src/fusion_component.cpp
)

target_include_directories(perception_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(perception_core PUBLIC rclcpp std_msgs sensor_msgs rus_sim_interfaces)
target_link_libraries(perception_core PUBLIC Eigen3::Eigen)

# ── 可执行节点 ──
add_executable(rus_sim_perception_node src/perception_main.cpp)
target_link_libraries(rus_sim_perception_node PUBLIC perception_core)
ament_target_dependencies(rus_sim_perception_node PUBLIC rclcpp)

# ── 安装规则 ──
install(TARGETS perception_core
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)
install(TARGETS rus_sim_perception_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME}/config)

ament_package()
