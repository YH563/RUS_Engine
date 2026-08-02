cmake_minimum_required(VERSION 3.18)
project(rus_sim_planning)

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

# ── 规划核心库（轨迹生成 / 伺服闭环 / 碰撞检测）──
add_library(planning_core SHARED
  src/trajectory_generator.cpp
  src/servo_loop.cpp
  src/collision_checker.cpp
)

target_include_directories(planning_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(planning_core PUBLIC rclcpp std_msgs sensor_msgs rus_sim_interfaces)
target_link_libraries(planning_core PUBLIC Eigen3::Eigen)

# ── 可执行节点 ──
add_executable(rus_sim_planning_node src/planning_main.cpp)
target_link_libraries(rus_sim_planning_node PUBLIC planning_core)
ament_target_dependencies(rus_sim_planning_node PUBLIC rclcpp)

# ── 安装规则 ──
install(TARGETS planning_core
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)
install(TARGETS rus_sim_planning_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME}/config)

ament_package()
