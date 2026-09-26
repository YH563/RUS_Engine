cmake_minimum_required(VERSION 3.18)
project(rus_sim_recorder)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED On)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ROS2 ament 构建
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rus_sim_interfaces REQUIRED)
# Eigen3：rus_sim_utils（header-only）的导出接口里带 Eigen3::Eigen，
# 必须在本包 find_package 到它，否则 generate 阶段报 target 未找到
find_package(Eigen3 REQUIRED)
find_package(rus_sim_utils REQUIRED)

# ── 记录文件读写库（storage/）──
# 纯 std 实现：不依赖 ROS —— 离线单测与 rus_sim_recorder_inspect 可脱离 ROS 运行
add_library(rec_storage SHARED
  src/storage/rec_writer.cpp
  src/storage/rec_reader.cpp
)
target_include_directories(rec_storage PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# ── 记录核心库（节点）──
# 分层：components/ 格式与写入基元（header-only）、storage/ 文件读写、节点只做订阅与调度
add_library(recorder_core SHARED
  src/rus_sim_recorder/recorder_node.cpp
)
target_include_directories(recorder_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(recorder_core PUBLIC rec_storage)
# rus_sim_utils：指令名常量（/recorder/command 的三条 recorder_*，与 bridge 路由同源）
ament_target_dependencies(recorder_core PUBLIC rclcpp rus_sim_interfaces rus_sim_utils)

# ── 可执行节点（仅入口，节点实现在 recorder_core 库中）──
add_executable(rus_sim_recorder_node src/main.cpp)
target_link_libraries(rus_sim_recorder_node PUBLIC recorder_core)
ament_target_dependencies(rus_sim_recorder_node PUBLIC rclcpp)

# ── 回放核心库（节点）──
# 复用 storage/ 的 RecReader（BuildTimeline + ReadRecordAt），把录音按时间轴重发回话题；
# 指令名 / 事件名取自 rus_sim_utils（协议层唯一来源，与 bridge 路由同源）。
add_library(replay_core SHARED
  src/rus_sim_replayer/replay_node.cpp
)
target_include_directories(replay_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(replay_core PUBLIC rec_storage)
ament_target_dependencies(replay_core PUBLIC rclcpp rus_sim_interfaces rus_sim_utils)

# ── 可执行节点（回放）──
add_executable(rus_sim_recorder_replay src/replay_main.cpp)
target_link_libraries(rus_sim_recorder_replay PUBLIC replay_core)
ament_target_dependencies(rus_sim_recorder_replay PUBLIC rclcpp)

# ── 离线校验工具（只链 rec_storage：无需 ROS 运行时）──
add_executable(rus_sim_recorder_inspect src/tools/recorder_inspect.cpp)
target_link_libraries(rus_sim_recorder_inspect PRIVATE rec_storage)

# ── 安装规则 ──
install(TARGETS rec_storage recorder_core replay_core
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)
install(TARGETS rus_sim_recorder_node DESTINATION lib/${PROJECT_NAME})
install(TARGETS rus_sim_recorder_replay DESTINATION lib/${PROJECT_NAME})
install(TARGETS rus_sim_recorder_inspect DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch
  PATTERN "__pycache__" EXCLUDE)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)

ament_package()
