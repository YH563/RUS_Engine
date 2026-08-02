cmake_minimum_required(VERSION 3.18)
project(rus_sim_utils)
# 设置使用C++17标准，并启用编译命令导出
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED On)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

find_package(Eigen3 REQUIRED)

file(GLOB SRC_FILES src/${PROJECT_NAME}/*.cpp)

# 创建共享库
add_library(${PROJECT_NAME} SHARED
  ${SRC_FILES}
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME} PUBLIC
  rclcpp
  geometry_msgs
)

target_link_libraries(${PROJECT_NAME} PUBLIC
  Eigen3::Eigen
)

install(DIRECTORY include/ DESTINATION include)

# 安装库文件
install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  DESTINATION lib
)

# 导出目标（让下游 find_package 能找到我们的库）
ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)

# 导出头文件目录（可选，通常是显式导出更好）
ament_export_include_directories("include")

# 导出库的依赖（重要！下游包不需要再重复 find_package(rclcpp)）
ament_export_dependencies(rclcpp)

# 最后必须调用 ament_package
ament_package()
