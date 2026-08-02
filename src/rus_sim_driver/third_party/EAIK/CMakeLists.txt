cmake_minimum_required(VERSION 3.9)
project(EAIK_Cpp)
set(CMAKE_POSITION_INDEPENDENT_CODE ON CACHE BOOL "Generate PIC")
# 使用顶层传入的变量，若未定义则提供合理的默认值（相对于当前目录）
if(NOT EIGEN_INCLUDE_DIR)
    set(EIGEN_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/eigen)
endif()

# 注意：顶层传入的是 IK_GEO_DIR，我们这里将其转换为 IK_GEO_INCLUDE_DIR 并添加默认
if(NOT IK_GEO_DIR)
    set(IK_GEO_DIR ${CMAKE_CURRENT_SOURCE_DIR}/ik-geo)
endif()
set(IK_GEO_INCLUDE_DIR ${IK_GEO_DIR}/cpp/include)   # 统一包含目录

# 添加 IK 子目录（使用相对路径，避免重复指定二进制目录）
add_subdirectory(IK)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(sources
    EAIK.cpp
    utils/kinematic_remodeling.cpp
    IK/utils/kinematic_utils.cpp
)

add_library(EAIK ${sources})

# 【关键】添加包含目录，并 PUBLIC 传播给依赖者
target_include_directories(EAIK PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/utils
    ${CMAKE_CURRENT_SOURCE_DIR}/IK
    ${CMAKE_CURRENT_SOURCE_DIR}/IK/utils
    ${EIGEN_INCLUDE_DIR}
    ${IK_GEO_INCLUDE_DIR}          # 使 EAIK 的用户自动获得 ik-geo 头文件
)

target_link_libraries(EAIK EAIK_Kinematics)