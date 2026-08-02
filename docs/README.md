# RUS-Sim 超声机器人仿真平台后端

## 项目简介

RUS-Sim（Robotic Ultrasound Simulator）是一个面向超声扫查机械臂的数字孪生仿真平台，旨在为自动化扫查算法和AI辅助诊断模块的研发提供安全、可控的集成验证环境。

## 后端开发环境搭建

### 系统环境要求
- 操作系统：**Ubuntu 22.04**
- **CMake** 3.18+

### 安装 ROS2 Humble 
- 安装教程见 [ROS 2 Documentation](https://docs.ros.org/en/humble/index.html)

### 安装额外依赖：
- **libigl** 2.5.0 (已在项目内)
- **Eigen3** 3.4.0
- **PCL** 1.21.1
- **pcl_conversions** 2.4.5
- **Moveit** 2.5.9-Alpha
- **ros-humble-pinocchio** 3.9.0

## 项目结构
采用标准的 ROS2 工作空间布局，主要功能组织为多个功能包。
```
src
├── frcobot_ros2            # 机械臂官方sdk
├── rus_sim_bringup         # 启动文件
├── rus_sim_controller      # 机械臂控制模块，负责实时控制处理
├── rus_sim_force           # 机械臂末端力反馈模块
├── rus_sim_interfaces      # 接口文件
├── rus_sim_perception      # 感知模块，主要负责处理超声数据
├── rus_sim_planner         # 规划模块，主要负责机械臂运动规划
├── rus_sim_pointcloud      # 点云处理模块，主要负责点云数据的处理
├── rus_sim_task_executor   # 任务执行模块，主要负责机械臂运动控制
├── rus_sim_utils           # 工具模块，提供一些通用工具函数
└── third_party             # 第三方库
```

## 自定义包功能说明

### frcobot_ros2
机械臂官方sdk，提供机械臂运动控制、传感器数据获取等功能。

### rus_sim_bringup
启动文件，用于启动整个仿真平台。

### rus_sim_controller
机械臂控制模块，负责实时控制处理，将位姿、图像、受力等数据进行耦合处理，便于实时调整机械臂运动。

### rus_sim_force
机械臂末端力反馈模块，用于获取、处理机械臂末端力反馈数据。

### rus_sim_interfaces
接口文件，定义了各个模块之间的接口。

### rus_sim_perception
感知模块，主要负责处理超声数据，包括超声数据的采集、预处理、特征提取等。

### rus_sim_planner
规划模块，主要负责机械臂运动规划，包括路径规划、轨迹规划等。

### rus_sim_pointcloud
点云处理模块，主要负责点云数据的处理，包括点云的生成、滤波、分割等。

### rus_sim_task_executor
任务执行模块，主要负责机械臂运动控制，包括机械臂的运动控制、任务调度等。

### rus_sim_utils
工具模块，提供一些通用工具函数，如文件操作、日志记录等。

### third_party
第三方库，包括libigl等。

## ROS 开发规范
- [ROS 2 开发规范](./DevelopmentGuide.md)