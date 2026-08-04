cmake_minimum_required(VERSION 3.18)
project(rus_sim_app)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED On)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ROS2 ament 构建
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rus_sim_interfaces REQUIRED)
find_package(rus_sim_utils REQUIRED)
find_package(yasmin REQUIRED)
find_package(yasmin_ros REQUIRED)

# ── 应用核心库（FSM / 用户接口 / 任务客户端 / 协调器）──
# CONFIGURE_DEPENDS：新增/删除源文件时自动重新 GLOB，避免缓存缺文件
file(GLOB APP_CORE_SOURCES CONFIGURE_DEPENDS src/rus_sim_app/*.cpp)
add_library(app_core SHARED ${APP_CORE_SOURCES})

target_include_directories(app_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(app_core PUBLIC rclcpp std_msgs rus_sim_interfaces rus_sim_utils yasmin yasmin_ros)

# ── 可执行节点 ──
add_executable(rus_sim_app_node src/main.cpp)
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
