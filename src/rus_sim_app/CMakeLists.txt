cmake_minimum_required(VERSION 3.18)
project(rus_sim_app)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED On)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ROS2 ament 构建
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rus_sim_interfaces REQUIRED)

# ── 应用核心库（FSM / 用户接口 / 任务客户端）──
add_library(app_core SHARED
  src/fsm.cpp
  src/user_interface.cpp
  src/task_client.cpp
)

target_include_directories(app_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(app_core PUBLIC rclcpp std_msgs rus_sim_interfaces)

# ── 可执行节点 ──
add_executable(rus_sim_app_node src/app_main.cpp)
target_link_libraries(rus_sim_app_node PUBLIC app_core)
ament_target_dependencies(rus_sim_app_node PUBLIC rclcpp)

# ── 安装规则 ──
install(TARGETS app_core
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)
install(TARGETS rus_sim_app_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME}/config)

ament_package()
