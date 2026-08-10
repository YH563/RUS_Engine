cmake_minimum_required(VERSION 3.18)
project(rus_sim_driver)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED On)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_BUILD_TYPE Release)

# ROS2 ament 构建
find_package(ament_cmake REQUIRED)

# ========== 1. 定位你刚刚拷贝进来的 SDK 路径 ==========
set(SDK_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/SDK/libfairino)

# 告诉编译器去 SDK 的 include 文件夹找头文件
set(SDK_INCLUDE_DIR ${SDK_ROOT}/include)

# 告诉链接器去 SDK 的 lib 文件夹找 .so 文件
set(SDK_LIB_DIR ${SDK_ROOT}/lib)
set(EIGEN_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/third_party/EAIK/eigen)
set(IK_GEO_DIR ${CMAKE_CURRENT_SOURCE_DIR}/third_party/EAIK/ik-geo)

# ========== 2. 查找 SDK 库文件（直接找绝对路径下的 .so） ==========
find_library(FAIRINO_SDK_LIB
    NAMES fairino
    PATHS ${SDK_LIB_DIR}
    NO_DEFAULT_PATH  # 强制只在这个路径找，不去系统 /usr/lib 找
)

if(NOT FAIRINO_SDK_LIB)
    message(FATAL_ERROR "Fairino SDK library not found in ${SDK_LIB_DIR}")
else()
    message(STATUS "Found Fairino SDK: ${FAIRINO_SDK_LIB}")
endif()

# ========== 3. 编译你的驱动核心库 ==========
# 你的源文件（包含 Sim 和 Real）
file(GLOB_RECURSE DRIVER_SRC
    src/driver/*.cpp
    src/trajectory/*.cpp
    src/controller/*.cpp
    src/rus_sim_driver/*.cpp
    src/components/*.cpp
)

find_package(Eigen3 REQUIRED)
find_package(urdf REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rus_sim_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rus_sim_utils REQUIRED)

# MuJoCo（使用自定义 Find 模块，也支持 MUJOCO_DIR 环境变量）
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
find_package(mujoco REQUIRED)
add_subdirectory(third_party/EAIK)

add_library(driver_core SHARED ${DRIVER_SRC})

# 添加头文件目录（你自己的头文件 + SDK 的头文件）
target_include_directories(driver_core PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/third_party>
    $<BUILD_INTERFACE:${SDK_INCLUDE_DIR}>   # 这就是让 #include "robot.h" 起效的关键！
    $<INSTALL_INTERFACE:include>
)

# ── ROS2 相关库：用 ament_target_dependencies 链接（自动处理 include 与依赖传递）──
ament_target_dependencies(driver_core PUBLIC
    rclcpp
    ament_index_cpp
    sensor_msgs
    urdf
    rus_sim_interfaces
    rus_sim_utils
)

# ── 非 ROS2 库：用 target_link_libraries 链接 ──
target_link_libraries(driver_core PUBLIC
    ${FAIRINO_SDK_LIB}
    pthread
    dl
    mujoco::mujoco
    Eigen3::Eigen
    EAIK
)

# ========== 4. 直接链接外部的 rus_sim_interfaces 接口包 ==========
# 消息/服务定义由独立的 rus_sim_interfaces 包提供（src/rus_sim_interfaces），
# 本包不再本地生成接口，只消费外部包导出的类型。
# 该包已在上方通过 ament_target_dependencies 链接。

# 重命名可执行文件，避免与 rosidl 目标冲突
set(NODE_EXECUTABLE rus_sim_driver_node)

# ========== 5. 【重点】设置运行时 RPATH，让 .so 找得到 ==========
# 让生成的 libdriver_core.so 在运行时，去同一个 lib 目录下找 libfairino.so
set_target_properties(driver_core PROPERTIES
    BUILD_RPATH "${SDK_LIB_DIR}"                           # 编译时生效
    INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib"           # 安装后生效
    LINK_FLAGS "-Wl,-rpath-link,${SDK_LIB_DIR}"           # 链接时辅助
)

# ========== 5. 安装规则 ==========
install(TARGETS driver_core
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
)

# 安装 robot_model 所有文件（urdf + mjcf + meshes）
install(DIRECTORY robot_model/
    DESTINATION share/${PROJECT_NAME}/robot_model
    FILES_MATCHING
        PATTERN "*.xml"
        PATTERN "*.urdf"
        PATTERN "*.STL"
        PATTERN "*.stl"
)

install(DIRECTORY robot_model/meshes/fairino3_v6/
    DESTINATION share/${PROJECT_NAME}/meshes
)

# 【极其重要】必须把你的 SDK 库也安装到同一个 lib 目录下，否则部署到别的电脑会报错！
# 安装 SDK 库文件
install(FILES ${SDK_LIB_DIR}/libfairino.so.2.1.4
    DESTINATION lib
)

# 在安装目录的 lib 子目录下创建软链接
install(CODE "
    execute_process(COMMAND ${CMAKE_COMMAND} -E create_symlink 
        libfairino.so.2.1.4 libfairino.so.2
        WORKING_DIRECTORY ${CMAKE_INSTALL_PREFIX}/lib)
    execute_process(COMMAND ${CMAKE_COMMAND} -E create_symlink 
        libfairino.so.2 libfairino.so
        WORKING_DIRECTORY ${CMAKE_INSTALL_PREFIX}/lib)
")

# ament export（仅导出不需要跨包依赖的接口）
ament_export_dependencies(ament_index_cpp)

ament_package()

# 注意：可执行文件名 ≠ 项目名/包名，避免与 rosidl 目标冲突
set(NODE_EXECUTABLE rus_sim_driver_node)

add_executable(${NODE_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp)
# ROS2 库用 ament，非 ROS2 库用 link
ament_target_dependencies(${NODE_EXECUTABLE} PUBLIC rclcpp rus_sim_interfaces)
target_link_libraries(${NODE_EXECUTABLE} PUBLIC driver_core)
# RPATH：让可执行文件和 .so 能找到同目录下的库
set_target_properties(${NODE_EXECUTABLE} PROPERTIES
    INSTALL_RPATH "${SDK_LIB_DIR};${CMAKE_INSTALL_PREFIX}/lib"
    BUILD_RPATH "${SDK_LIB_DIR};${CMAKE_CURRENT_BINARY_DIR}/third_party/EAIK"
)
set_target_properties(driver_core PROPERTIES
    BUILD_RPATH "${SDK_LIB_DIR};${CMAKE_CURRENT_BINARY_DIR}/third_party/EAIK"
)
install(TARGETS ${NODE_EXECUTABLE}
    DESTINATION lib/${PROJECT_NAME}
)

# 安装 EAIK 库（第三方子模块）
install(TARGETS EAIK EAIK_Kinematics SUBPROBLEMS
    LIBRARY DESTINATION lib
)

# 安装 Python 脚本
install(PROGRAMS scripts/keyboard_control.py
    DESTINATION lib/${PROJECT_NAME}
)

# 安装 launch 文件
install(DIRECTORY launch/
    DESTINATION share/${PROJECT_NAME}/launch
)

# 安装 config 文件
install(DIRECTORY config/
    DESTINATION share/${PROJECT_NAME}/config
)