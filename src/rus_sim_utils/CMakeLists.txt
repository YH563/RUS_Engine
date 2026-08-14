cmake_minimum_required(VERSION 3.18)
project(rus_sim_utils)
# 纯头文件工具包（header-only）：协议 / 指令 / 路由注册表全部 inline 实现于 include/，
# 无编译单元。下游通过 ament_target_dependencies(... rus_sim_utils) 使用。
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED On)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)

# INTERFACE 库：只导出头文件目录与依赖，不产生编译单元
add_library(${PROJECT_NAME} INTERFACE)

target_include_directories(${PROJECT_NAME} INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# robot_state.hpp 依赖 Eigen 头文件，透传给下游
target_link_libraries(${PROJECT_NAME} INTERFACE
  Eigen3::Eigen
)

# 安装头文件
install(DIRECTORY include/ DESTINATION include)

# 导出接口目标（让下游 find_package(rus_sim_utils) 能找到 target）
install(TARGETS ${PROJECT_NAME} EXPORT ${PROJECT_NAME})

# 导出头文件目录与依赖（兼容 ament_target_dependencies 下游用法）
ament_export_include_directories("include")
ament_export_dependencies(rclcpp)
ament_export_targets(${PROJECT_NAME})

# 最后必须调用 ament_package
ament_package()
