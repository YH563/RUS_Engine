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

# ── 数据源：RealSense 直连（可选；缺 librealsense2 时自动降级为 stub，其余源照常）──
option(RUS_SIM_WITH_REALSENSE "编译 RealSense 直连数据源（需 librealsense2）" ON)
set(RUS_REALSENSE_SOURCE src/camera/realsense_source_stub.cpp)
if(RUS_SIM_WITH_REALSENSE)
  find_package(realsense2 QUIET)   # librealsense2-dev 提供的 CMake config
  if(realsense2_FOUND)
    set(RUS_REALSENSE_SOURCE src/camera/realsense_source.cpp)
    message(STATUS "rus_sim_perception: RealSense 直连数据源已启用（realsense2 ${realsense2_VERSION}）")
  else()
    find_path(REALSENSE_INCLUDE_DIR librealsense2/rs.hpp)
    find_library(REALSENSE_LIBRARY realsense2)
    if(REALSENSE_INCLUDE_DIR AND REALSENSE_LIBRARY)
      set(RUS_REALSENSE_SOURCE src/camera/realsense_source.cpp)
      message(STATUS "rus_sim_perception: RealSense 直连数据源已启用（${REALSENSE_LIBRARY}）")
    else()
      message(WARNING "rus_sim_perception: 未找到 librealsense2 → source=realsense 不可用"
                      "（ros_topic / replay 照常）。安装：sudo apt install librealsense2-dev")
    endif()
  endif()
endif()

# ── 感知核心库（感知节点 + 采集/算法模块）──
# 节点仅做通信与调度；分层：
#   camera/      数据采集（RealSense 直连 / ROS 话题 / 离线回放 + 最新帧槽）
#   components/  公共组件（位姿插值、传感器帧编码）
#   pointcloud/  点云算法子模块（变换、滤波、建图）
add_library(perception_core SHARED
  src/rus_sim_perception/perception_node.cpp
  src/camera/frame_slot.cpp
  src/camera/ros_topic_source.cpp
  src/camera/replay_source.cpp
  src/camera/source_factory.cpp
  ${RUS_REALSENSE_SOURCE}
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

# ── RealSense 链接（CMake config 优先，其次手工探测的 include/lib）──
if(RUS_REALSENSE_SOURCE STREQUAL "src/camera/realsense_source.cpp")
  if(realsense2_FOUND)
    target_link_libraries(perception_core PUBLIC realsense2::realsense2)
  else()
    target_include_directories(perception_core PUBLIC ${REALSENSE_INCLUDE_DIR})
    target_link_libraries(perception_core PUBLIC ${REALSENSE_LIBRARY})
  endif()
endif()

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
