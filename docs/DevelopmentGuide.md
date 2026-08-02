# ROS 开发代码规范

本项目遵循 ROS2 的开发规范，并在此基础上进行了一些扩展和优化，本文档仅针对 **cpp 代码**进行说明。

## 包命名与结构

### 包命名
包名应遵循 ROS2 的命名规范，使用小写字母和下划线，例如 `rus_sim_core`，所有自定义包均以 `rus_sim_` 开头。

### 包的目录结构
标准包的目录结构如下：

```
rus_sim_package/
  ├── CMakeLists.txt
  ├── package.xml
  ├── src/
  │   └── main.cpp             # 主函数入口
  │   └── rus_sim_package/     # 核心源代码目录，与包名保持一致
  ├── launch/
  │   └── package.launch.py    # 启动文件
  ├── include/
  │   └── rus_sim_package/     # 核心代码头文件目录，与包名保持一致
  ├── test/
  │   └── src/                 # 测试代码源码
  │   └── test_include/        # 测试代码头文件
  └── config/
```

### 自定义包文档
在 `docs` 文件夹下创建与包同名的文件夹，并在该文件夹内创建与包同名的 `.md` 文件，用于描述包的功能和使用方法，可参考 [示例文档](./DocExample.md)。

## CMakeLists.txt

### 基本模板

```
cmake_minimum_required(VERSION 3.18)
project(rus_sim_pointcloud)
# 设置使用C++17标准，并启用编译命令导出
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED On)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 测试选项设置
option(ENABLE_TEST "Include test code in executable" ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

set(LIBIGL_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../third_party/libigl/include)  # libigl 库目录
file(GLOB SRC_FILES src/${PROJECT_NAME}/*.cpp)  # 核心源文件
file(GLOB TEST_FILES ${CMAKE_CURRENT_SOURCE_DIR}/test/src/*.cpp)  # 测试源文件

# ------------------- 核心静态库 -------------------
add_library(pointcloud_core STATIC ${SRC_FILES})

target_include_directories(pointcloud_core PUBLIC
  ${LIBIGL_INCLUDE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

ament_target_dependencies(pointcloud_core PUBLIC
  rclcpp
  sensor_msgs
  shape_msgs
  pcl_conversions
)

target_link_libraries(pointcloud_core PUBLIC
  Eigen3::Eigen
  ${PCL_LIBRARIES}
)

# ------------------- 主程序 -------------------
set(MAIN_SOURCES src/main.cpp)

# 如果启用测试，将测试源文件加入主程序
if(ENABLE_TEST)
  # 将测试源文件加入主程序
  list(APPEND MAIN_SOURCES ${TEST_FILES})
  # 定义宏，让代码知道测试模式已启用
  add_definitions(-DENABLE_TESTING)
endif()

add_executable(rus_sim_pointcloud ${MAIN_SOURCES})

target_link_libraries(rus_sim_pointcloud PRIVATE
  pointcloud_core
)

ament_target_dependencies(rus_sim_pointcloud PUBLIC
  rclcpp
  sensor_msgs
  shape_msgs
  pcl_conversions
)

target_include_directories(rus_sim_pointcloud PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/test  # 链接测试用头文件
)

install(TARGETS rus_sim_pointcloud
  DESTINATION lib/${PROJECT_NAME}
)


# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   # the following line skips the linter which checks for copyrights
#   # comment the line when a copyright and license is added to all source files
#   set(ament_cmake_copyright_FOUND TRUE)
#   # the following line skips cpplint (only works in a git repo)
#   # comment the line when this package is in a git repo and when
#   # a copyright and license is added to all source files
#   set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
# endif()

ament_package()

```
以上仅作参考，具体实现请根据项目需求进行调整。

### 关键语句说明
- `cmake_minimum_required(VERSION 3.18)`：CMake 版本要求最低 3.18。
- `project(rus_sim_pointcloud)`：项目名称，与包名保持一致
- `set(CMAKE_CXX_STANDARD 17)`：设置使用 C++17 标准
- `find_package(ament_cmake REQUIRED)`：引入 ament_cmake 包
- `find_package(...)`：引入其他第三方库
- `add_library(pointcloud_core ...)`：添加静态库，库名字 `package_core`，将核心源代码编译为静态库

## 源代码命名与组织

### 文件命名
- 源文件：`.cpp`，使用蛇形命名（snake_case），例如 `trajectory_planner.cpp`。对于节点类，则需在文件末尾添加 `_node` 进行区分，例如 `trajectory_planner_node.cpp`。
- 头文件：`.hpp`，使用蛇形命名（snake_case），与对应源文件名字保持一致。
- 启动文件：`.launch.py`，使用蛇形命名（snake_case）。

### 头文件保护
- 在头文件开始添加 `#pragma once` 语句。
- 头文件内函数，简短函数可以添加 `inline` 关键字，对于长函数必须在头文件内声明，在源文件内进行实现。
- 头文件内类成员函数，简短函数可在头文件内进行实现，长函数必须在头文件内声明，在源文件内进行实现。

### 命名空间
为头文件内添加命名空间，例如 `namespace RusTrajectoryPlanner`，使用大驼峰命名（PascalCase），以 `Rus` 开头，表示为自定义库，对于测试文件，命名空间为 `Test` 开头，表示为测试文件。

## 代码风格

### 命名规范

- **类名**：大驼峰
- **函数名**：大驼峰，成员函数中的私有函数成员使用蛇形命名，以动词开头
- **变量名**：蛇形命名，成员变量以 `_` 结尾，全局变量以 `g_` 开头
- **常量**：使用 `kPascalCase`，以 `k` 开头，表示为常量；对于宏定义，使用**全大写+下划线**
- **命名空间**：见上文，不再赘述
- **话题回调函数**：接收方以 `on` 开头，用蛇形命名，如 `on_cloud`，发布方以 `publish` 开头，如 `publish_cloud`。话题订阅者用 `sub_` 结尾，如 `cloud_sub_`，发布者用 `pub_` 结尾，如 `cloud_pub_`。回调函数结尾不需要写 `callback`。变量末尾的下划线表示为私有成员变量。
- **服务回调函数**：服务方以 `handle` 开头，用蛇形命名，如 `handle_cloud`，客户方以 `response` 或 `request` 开头，如 `response_cloud`。服务端变量用 `server_` 结尾，如 `cloud_server_`，客户端变量用 `client_` 结尾，如 `cloud_client_`。回调函数结尾不需要写 `callback`。

### 注释规范


| 情况 | 示例 |
|------|------|
| **类/结构体**：一句话说明它是干什么的 | `// 点云处理器，负责滤波和分割` |
| **重要函数**：一句话说明功能，特别说明参数/返回值 | `// 体素滤波，leaf_size单位是米，返回滤波后点云` |
| **复杂的变量**：比如魔法数字、特殊含义的 flag | `int max_points = 5000; // 超过这个数就降采样，防止内存爆` |
| **算法/业务逻辑**：为什么这么写，而不是别的写法 | `// 这里用 KdTree 加速，因为点云数量很大` |
| **已知问题/TODO**：方便以后回来改 | `// TODO: 这里需要支持动态参数更新` |
| **容易混淆的依赖关系**：比如某个函数必须在另一个之前调用 | `// 注意：调用前必须 init() 已执行` |

---

## 测试

### CMake 控制测试编译
在 `CMakeLists.txt` 中使用 `option(ENABLE_TEST ...)` 开关控制是否将测试代码编译进可执行文件。  
- 默认开启测试（`ON`），方便开发调试。
- 正式发布时可关闭（`OFF`），减小可执行文件体积。
- 当 `ENABLE_TEST` 为 `ON` 时，CMake 会自动收集 `test/src/*.cpp` 文件，并添加到主程序的源文件列表中；同时定义宏 `ENABLE_TESTING`，供代码中条件编译。

### 主程序入口与测试模式选择
主函数（`src/main.cpp`）通过命令行参数 `--test` 决定是否进入测试模式。

- **正常模式**：不传 `--test`，启动核心节点（如点云处理节点），执行实际功能。
- **测试模式**：传入 `--test`，在核心节点之外，额外启动测试节点（如点云生成器、模拟传感器等），用于模拟数据或验证功能。

**示例代码框架**
```cpp
int main(int argc, char **argv)
{
    bool test_mode = false;
    for (int i = 1; i < argc; ++i) {
        if (std::string(argv[i]) == "--test") {
            test_mode = true;
            break;
        }
    }
    // ... 初始化节点等
#ifdef ENABLE_TESTING
    if (test_mode) {
        // 启动测试节点
        auto test_node = std::make_shared<TestNode>();
        executor.add_node(test_node);
    }
#endif
    executor.spin();
}
```

### 禁止事项
- **禁止手动编译测试文件**：所有测试必须通过 CMake 和 `colcon` 编译，不得在终端手动调用 `g++` 编译。
- **禁止在测试文件中包含独立的 `main` 函数**：测试逻辑必须写在测试节点的类中，由主程序统一调用。
- **禁止将测试代码混入 `src/` 目录**：测试代码必须放在 `test/` 下，与核心源码清晰分离。
- **禁止在测试节点中硬编码路径**：使用 ROS 2 参数或环境变量传递必要配置。

### 运行测试
编译后，通过以下命令运行测试模式：
```bash
ros2 run your_package your_node --test
```
如需正常功能，直接运行：
```bash
ros2 run your_package your_node
```
