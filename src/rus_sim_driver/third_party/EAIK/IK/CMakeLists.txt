cmake_minimum_required(VERSION 3.9)
project(EAIK_Kin)

# 使用顶层传入的 IK_GEO_DIR
if(NOT IK_GEO_DIR)
    message(FATAL_ERROR "IK_GEO_DIR must be defined before including IK subdirectory")
endif()

# 添加 ik-geo 的子问题模块（如果必要）
add_subdirectory(${IK_GEO_DIR}/cpp/subproblems ${CMAKE_CURRENT_BINARY_DIR}/subproblems)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(sources
    6R_IK.cpp 5R_IK.cpp 4R_IK.cpp 3R_IK.cpp 2R_IK.cpp 1R_IK.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/utils/kinematic_utils.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../utils/kinematic_remodeling.cpp
)

add_library(EAIK_Kinematics ${sources})

# 【修正】使用变量替代硬编码路径
target_include_directories(EAIK_Kinematics PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/../utils
    ${IK_GEO_DIR}/cpp/subproblems   # 或 ${IK_GEO_DIR}/cpp/include，视你源码需要
)

target_link_libraries(EAIK_Kinematics SUBPROBLEMS)