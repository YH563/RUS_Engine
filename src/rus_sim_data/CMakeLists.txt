cmake_minimum_required(VERSION 3.18)
project(rus_sim_data)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED On)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ROS2 ament 构建
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rus_sim_interfaces REQUIRED)
find_package(rus_sim_utils REQUIRED)
find_package(Eigen3 REQUIRED)

# ── 数据核心库（录制 / 回放 / 统一节点）──
file(GLOB DATA_CORE_SOURCES CONFIGURE_DEPENDS src/rus_sim_data/*.cpp)
add_library(data_core SHARED ${DATA_CORE_SOURCES})

target_include_directories(data_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(data_core PUBLIC rclcpp sensor_msgs rus_sim_interfaces rus_sim_utils)
target_link_libraries(data_core PUBLIC Eigen3::Eigen)

# ── 可执行节点 ──
add_executable(rus_sim_data_node src/main.cpp)
target_link_libraries(rus_sim_data_node PUBLIC data_core)
ament_target_dependencies(rus_sim_data_node PUBLIC rclcpp)

# ── 安装规则 ──
install(TARGETS data_core
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)
install(TARGETS rus_sim_data_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME}/config)

ament_package()
