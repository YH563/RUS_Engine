cmake_minimum_required(VERSION 3.18)
project(rus_sim_bridge)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED On)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ROS2 ament 构建
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rus_sim_interfaces REQUIRED)
find_package(rus_sim_utils REQUIRED)

# libwebsockets（系统库）
find_library(WEBSOCKETS_LIB websockets REQUIRED)

# ── 核心库（协议 + WS 服务器 + 桥接节点） ──
file(GLOB BRIDGE_CORE_SOURCES CONFIGURE_DEPENDS src/rus_sim_bridge/*.cpp)
add_library(bridge_core SHARED ${BRIDGE_CORE_SOURCES})

target_include_directories(bridge_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(bridge_core PUBLIC rclcpp sensor_msgs rus_sim_interfaces rus_sim_utils)
target_link_libraries(bridge_core PUBLIC ${WEBSOCKETS_LIB})

# ── 可执行节点 ──
add_executable(rus_sim_bridge_node src/main.cpp)
target_link_libraries(rus_sim_bridge_node PUBLIC bridge_core)
ament_target_dependencies(rus_sim_bridge_node PUBLIC rclcpp)

# ── 安装规则 ──
install(TARGETS bridge_core
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)
install(TARGETS rus_sim_bridge_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch
  PATTERN "__pycache__" EXCLUDE)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)

ament_package()
