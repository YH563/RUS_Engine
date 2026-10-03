cmake_minimum_required(VERSION 3.18)
project(rus_sim_reconstruction)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED On)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 核心算法：仅 Eigen，无 ROS 运行时依赖（方便脱离运行时单测）
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Threads REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
# PCL 仅用于网格化工具（Poisson），核心库与单测不链接
find_package(PCL REQUIRED COMPONENTS common surface search io filters kdtree)
# 实时可视化用 GLFW + GLEW（避免 PCLVisualizer/VTK 在本机的不稳定）
find_package(PkgConfig REQUIRED)
find_package(OpenGL REQUIRED)
pkg_check_modules(GLFW REQUIRED glfw3)
pkg_check_modules(GLEW REQUIRED glew)

add_library(reconstruction_core SHARED
  src/surfel_map.cpp
  src/sharded_surfel_map.cpp
  src/tsdf_volume.cpp
)
target_include_directories(reconstruction_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(reconstruction_core PUBLIC Eigen3::Eigen Threads::Threads)

install(TARGETS reconstruction_core
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)
install(DIRECTORY include/ DESTINATION include)

# ── PCL 点云预处理库（均匀重采样）──
add_library(reconstruction_pcl SHARED src/cloud_resampler.cpp)
target_include_directories(reconstruction_pcl PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS})
target_link_libraries(reconstruction_pcl Eigen3::Eigen ${PCL_LIBRARIES})
install(TARGETS reconstruction_pcl LIBRARY DESTINATION lib ARCHIVE DESTINATION lib)

# 重采样工具（PCD → 间距分布 → PCD）
add_executable(rus_sim_recon_resample examples/resample_tool.cpp)
target_include_directories(rus_sim_recon_resample PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(rus_sim_recon_resample reconstruction_pcl ${PCL_LIBRARIES})
install(TARGETS rus_sim_recon_resample DESTINATION lib/${PROJECT_NAME})

# ── 离线 demo（合成场景 → PLY，供可视化对比）──
add_executable(rus_sim_recon_demo examples/fuse_demo.cpp)
target_link_libraries(rus_sim_recon_demo reconstruction_core)
install(TARGETS rus_sim_recon_demo DESTINATION lib/${PROJECT_NAME})

# ── 实时可视化（OpenGL/GLFW，逐帧刷新面元；不依赖 VTK）──
add_executable(rus_sim_recon_live examples/live_gl.cpp)
target_include_directories(rus_sim_recon_live PRIVATE ${GLFW_INCLUDE_DIRS} ${GLEW_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
target_link_libraries(rus_sim_recon_live reconstruction_core
  ${GLFW_LIBRARIES} ${GLEW_LIBRARIES} OpenGL::GL ${PCL_LIBRARIES} ${CMAKE_DL_LIBS})
install(TARGETS rus_sim_recon_live DESTINATION lib/${PROJECT_NAME})

# ── 精简版 SurfelMeshing（面元平滑+局部三角化+一致性，实时网格）──
add_executable(rus_sim_recon_live_surfel_mesh examples/live_surfel_mesh_gl.cpp)
target_include_directories(rus_sim_recon_live_surfel_mesh PRIVATE ${GLFW_INCLUDE_DIRS} ${GLEW_INCLUDE_DIRS})
target_link_libraries(rus_sim_recon_live_surfel_mesh reconstruction_core
  ${GLFW_LIBRARIES} ${GLEW_LIBRARIES} OpenGL::GL ${CMAKE_DL_LIBS})
install(TARGETS rus_sim_recon_live_surfel_mesh DESTINATION lib/${PROJECT_NAME})

# ── 实时：完整滤波链 → 稀疏 TSDF → 增量网格（PCL + OpenGL）──
add_executable(rus_sim_recon_live_tsdf_filtered examples/live_tsdf_filtered_gl.cpp)
target_include_directories(rus_sim_recon_live_tsdf_filtered PRIVATE
  ${GLFW_INCLUDE_DIRS} ${GLEW_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
target_link_libraries(rus_sim_recon_live_tsdf_filtered
  reconstruction_core reconstruction_pcl
  ${GLFW_LIBRARIES} ${GLEW_LIBRARIES} OpenGL::GL ${PCL_LIBRARIES} ${CMAKE_DL_LIBS})
install(TARGETS rus_sim_recon_live_tsdf_filtered DESTINATION lib/${PROJECT_NAME})

# ── 面元 → 网格（GP3）──
add_executable(rus_sim_recon_surfel_mesh examples/surfel_mesh.cpp)
target_include_directories(rus_sim_recon_surfel_mesh PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(rus_sim_recon_surfel_mesh reconstruction_core ${PCL_LIBRARIES})
install(TARGETS rus_sim_recon_surfel_mesh DESTINATION lib/${PROJECT_NAME})

# ── TSDF 网格导出（供拓扑/几何分析）──
add_executable(rus_sim_recon_tsdf_dump examples/tsdf_dump.cpp)
target_link_libraries(rus_sim_recon_tsdf_dump reconstruction_core)
install(TARGETS rus_sim_recon_tsdf_dump DESTINATION lib/${PROJECT_NAME})

# ── 实时管线（每帧降采样 → 增量 TSDF → 网格；无泊松）──
add_executable(rus_sim_recon_live_pipeline examples/live_pipeline_gl.cpp)
target_include_directories(rus_sim_recon_live_pipeline PRIVATE ${GLFW_INCLUDE_DIRS} ${GLEW_INCLUDE_DIRS})
target_link_libraries(rus_sim_recon_live_pipeline reconstruction_core
  ${GLFW_LIBRARIES} ${GLEW_LIBRARIES} OpenGL::GL ${CMAKE_DL_LIBS})
install(TARGETS rus_sim_recon_live_pipeline DESTINATION lib/${PROJECT_NAME})

# ── 网格查看器（加载网格 PLY 交互渲染；OpenGL，不依赖 matplotlib/VTK）──
add_executable(rus_sim_recon_meshview examples/mesh_gl.cpp)
target_include_directories(rus_sim_recon_meshview PRIVATE ${GLFW_INCLUDE_DIRS} ${GLEW_INCLUDE_DIRS})
target_link_libraries(rus_sim_recon_meshview
  ${GLFW_LIBRARIES} ${GLEW_LIBRARIES} OpenGL::GL ${CMAKE_DL_LIBS})
install(TARGETS rus_sim_recon_meshview DESTINATION lib/${PROJECT_NAME})

# ── 实时增量网格（TSDF + Marching Tetrahedra，OpenGL；纯 Eigen，无 PCL/VTK）──
add_executable(rus_sim_recon_live_tsdf examples/live_tsdf_gl.cpp)
target_include_directories(rus_sim_recon_live_tsdf PRIVATE ${GLFW_INCLUDE_DIRS} ${GLEW_INCLUDE_DIRS})
target_link_libraries(rus_sim_recon_live_tsdf reconstruction_core
  ${GLFW_LIBRARIES} ${GLEW_LIBRARIES} OpenGL::GL ${CMAKE_DL_LIBS})
install(TARGETS rus_sim_recon_live_tsdf DESTINATION lib/${PROJECT_NAME})

# ── 基准工具（只测 SurfelMap::Fuse 吞吐）──
add_executable(rus_sim_recon_bench examples/bench_fuse.cpp)
target_link_libraries(rus_sim_recon_bench reconstruction_core)
install(TARGETS rus_sim_recon_bench DESTINATION lib/${PROJECT_NAME})

# ── 网格工具（面元 PLY → Poisson → 三角网格 PLY；仅此目标链接 PCL）──
add_executable(rus_sim_recon_mesh examples/mesh_demo.cpp)
target_include_directories(rus_sim_recon_mesh PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(rus_sim_recon_mesh reconstruction_core ${PCL_LIBRARIES})
install(TARGETS rus_sim_recon_mesh DESTINATION lib/${PROJECT_NAME})

# ── ROS 节点：增量重建（嵌入感知层）──
add_library(reconstruction_ros SHARED src/reconstruction_node.cpp)
target_include_directories(reconstruction_ros PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS})
target_link_libraries(reconstruction_ros reconstruction_core ${PCL_LIBRARIES})
ament_target_dependencies(reconstruction_ros rclcpp sensor_msgs pcl_conversions)

add_executable(rus_sim_reconstruction_node src/reconstruction_main.cpp)
target_link_libraries(rus_sim_reconstruction_node reconstruction_ros)
ament_target_dependencies(rus_sim_reconstruction_node rclcpp)

# 实时网格查看器（订阅 /reconstructed_cloud）
add_executable(rus_sim_recon_view_ros examples/view_ros_gl.cpp)
target_include_directories(rus_sim_recon_view_ros PRIVATE
  ${GLFW_INCLUDE_DIRS} ${GLEW_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
target_link_libraries(rus_sim_recon_view_ros
  ${GLFW_LIBRARIES} ${GLEW_LIBRARIES} OpenGL::GL ${PCL_LIBRARIES} ${CMAKE_DL_LIBS})
ament_target_dependencies(rus_sim_recon_view_ros rclcpp sensor_msgs pcl_conversions)
install(TARGETS rus_sim_recon_view_ros DESTINATION lib/${PROJECT_NAME})

# 测试数据源：连续喂多帧点云
add_executable(rus_sim_recon_feed src/recon_feed.cpp)
target_include_directories(rus_sim_recon_feed PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(rus_sim_recon_feed Eigen3::Eigen ${PCL_LIBRARIES})
ament_target_dependencies(rus_sim_recon_feed rclcpp sensor_msgs pcl_conversions)

install(TARGETS reconstruction_ros
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)
install(TARGETS rus_sim_reconstruction_node rus_sim_recon_feed DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)

# ── 单元测试（合成几何，无需 ROS / PCL）──
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_surfel_map test/test_surfel_map.cpp)
  target_link_libraries(test_surfel_map reconstruction_core)
  ament_add_gtest(test_tsdf test/test_tsdf.cpp)
  target_link_libraries(test_tsdf reconstruction_core)
  ament_add_gtest(test_cloud_resampler test/test_cloud_resampler.cpp)
  target_include_directories(test_cloud_resampler PRIVATE ${PCL_INCLUDE_DIRS})
  target_link_libraries(test_cloud_resampler reconstruction_pcl ${PCL_LIBRARIES})
endif()

ament_package()
