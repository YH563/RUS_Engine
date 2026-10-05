# RUS_Engine 测试报告

> 范围：后端 8 个 ROS 2 功能包的**自动化单元测试**（gtest）。
> 目标：覆盖**纯逻辑 / 纯算法**模块（无 ROS 运行时、无硬件），作为回归基线；
> 硬件/ROS 运行时相关的集成验证另行说明（见文末"未覆盖"）。
> 结论：**10 个测试可执行、71 条 gtest 用例，0 失败**（`colcon test` 全绿）。

## 1. 测试方案

### 1.1 分层原则
| 层 | 内容 | 是否纳入本轮单测 |
|---|---|---|
| 纯算法 / 纯逻辑（无 ROS、无硬件）| 位姿插值、点云编码、空间变换、滤波链、地图累积、指令解析、轨迹生成、协议编解码、记录文件读写 | ✅ |
| ROS 节点 / 服务编排 | bridge dispatcher 扇出、各 node 的 handle_command | ⚠️ 仅测其可抽离的纯部分（如路由注册表）|
| 硬件 / 运行时 | RealSense 采集、MuJoCo 仿真、EIK 逆运动学（需 URDF 模型）| ❌ 需设备/模型，暂缓 |
| 端到端闭环 | `pre_scan_done → plan → execute → scan_done` | ❌ 需 ROS 运行时；现为手动脚本 |

### 1.2 用例清单
| 包 | 测试可执行 | 覆盖模块 | 用例数 |
|---|---|---|---|
| `rus_sim_utils` | `test_rus_sim_utils` | 协议编解码（command/reply/state/sensor/mesh）、指令解析、位姿工具、error_code | 16 |
| `rus_sim_recorder` | `test_rus_sim_recorder` | CRC32、BinWriter 小端、`.rusrec` 写读往返 + 崩溃扫描恢复 | 6 |
| `rus_sim_planning` | `test_trajectory_interpolator` | 插值（位置 lerp + 姿态 slerp）、推进/复位 | 8 |
| `rus_sim_planning` | `test_trajectory_generator` | 合成平面点云 → 起终点轨迹生成、贴面性、点有效性 | 1 |
| `rus_sim_perception` | `test_perception` | `pose_interpolator`、`sensor_encoder`、`spatial_transformer`、`map_manager`、`cloud_filter_pipeline`（含重采样阶段） | 14 |
| `rus_sim_perception` | `test_cloud_resampler` | 平面间距均匀化、球面法线径向、间距统计 | 3 |
| `rus_sim_reconstruction` | `test_surfel_map` | 面元融合（重复融合/置信度/多视角收敛/位姿/法线/上限/Clear） | 8 |
| `rus_sim_reconstruction` | `test_tsdf` | TSDF 网格、增量增长、无脏块、chunk 导出/快照、chunkId 打包 | 5 |
| `rus_sim_driver` | `test_cmd_parser` | 指令解析（movej/movel/servo/jog/驱动控制/未知回退） | 6 |
| `rus_sim_bridge` | `test_command_registry` | 指令→模块路由注册表（注册/查询/本地/改扇出/模块名） | 5 |
| **合计** | **10 个可执行** | | **72** |

### 1.3 本轮新增（本次工作）
新增 **29 条**用例，并为 `rus_sim_perception` / `rus_sim_driver` / `rus_sim_bridge` 补齐 gtest 基建（此前无 `BUILD_TESTING` 段）：
- `rus_sim_perception/test/test_perception.cpp`（14，含滤波链重采样阶段用例）
- `rus_sim_perception/test/test_cloud_resampler.cpp`（3；`cloud_resampler` 由 reconstruction 迁入 perception）
- `rus_sim_driver/test/test_cmd_parser.cpp`（6）
- `rus_sim_planning/test/test_trajectory_generator.cpp`（1）
- `rus_sim_bridge/test/test_command_registry.cpp`（5）

接入方式：各包 `CMakeLists.txt` 增加
```cmake
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(<target> test/<file>.cpp)
  target_link_libraries(<target> <pkg>_core)   # 或 ament_target_dependencies(<target> <依赖>)
endif()
```
与既有包（utils/recorder/planning/reconstruction）写法一致。

## 2. 测试结果

### 2.1 汇总
```
$ colcon test
Summary: 9 packages finished

$ colcon test-result
Summary: 82 tests, 0 errors, 0 failures, 0 skipped
```
> 说明：`colcon test-result` 的 82 = **72 条 gtest 用例** + 每个测试可执行的 **ctest 包装条目**（10 条）。
> 以 gtest 用例为准：**72 条，0 失败**。

### 2.2 明细（gtest）
| 包 | 可执行 | tests | failures |
|---|---|---|---|
| rus_sim_utils | test_rus_sim_utils | 16 | 0 |
| rus_sim_recorder | test_rus_sim_recorder | 6 | 0 |
| rus_sim_planning | test_trajectory_interpolator | 8 | 0 |
| rus_sim_planning | test_trajectory_generator | 1 | 0 |
| rus_sim_perception | test_perception | 14 | 0 |
| rus_sim_perception | test_cloud_resampler | 3 | 0 |
| rus_sim_reconstruction | test_surfel_map | 8 | 0 |
| rus_sim_reconstruction | test_tsdf | 5 | 0 |
| rus_sim_driver | test_cmd_parser | 6 | 0 |
| rus_sim_bridge | test_command_registry | 5 | 0 |
| **合计** | 10 | **72** | **0** |

### 2.3 测试发现并修复的问题
- **`PoseInterpolator::Add` 拒绝时间戳 0 的首帧**（`rus_sim_perception/src/components/pose_interpolator.cpp`）：
  原实现 `if (stamp <= Newest()) return false;`，空缓冲时 `Newest()` 为 `0.0`，导致 `stamp == 0` 的第一帧被误判为"乱序"而丢弃（ROS epoch 秒下不触发，属潜在边界缺陷）。
  已改为 `if (!buffer_.empty() && stamp <= buffer_.back().stamp) return false;`；新增用例 `PoseInterpolator.InterpolatesBetweenTwoPoses` 覆盖。

## 3. 运行方式
```bash
# 全量
colcon test
colcon test-result --all            # 汇总

# 单包
colcon test --packages-select rus_sim_perception

# 直接跑某个 gtest（看详细输出）
./build/rus_sim_perception/test_perception
./build/rus_sim_perception/test_perception --gtest_filter='PoseInterpolator.*'
```

## 4. 未覆盖（及原因）
| 项 | 原因 | 备注 |
|---|---|---|
| bridge `command_dispatcher` 扇出/聚合 | 依赖 ROS Service 客户端与运行时 | 其**纯路由表**部分已由 `CommandRegistry` 单测覆盖 |
| driver `kinematics`（FK/IK/Jacobian）| 需从 URDF/MuJoCo 模型构造 `EAIK::Robot` | 后续可抽出模型构造再补 |
| planning `collision_checker` | 当前阶段不做碰撞（已与需求确认暂缓）| — |
| perception 采集源（`realsense_source` / `ros_topic_source` / `frame_slot`）| 依赖设备 / ROS 话题 | 变换/编码/建图等下游已覆盖 |
| 端到端 `pre_scan_done→plan→execute→scan_done` | 需 ROS 运行时（多节点）| 现用 `rus_sim_bringup/scripts/test_prescan.sh` 手动验证；自动化 `launch_testing` 未做 |

## 5. 相关
- 代码规范（含测试约定）：[DevelopmentGuide.md](./DevelopmentGuide.md)
- 各模块测试现状：见各包 `docs/rus_sim_*/rus_sim_*.md` 的功能清单状态标记。
