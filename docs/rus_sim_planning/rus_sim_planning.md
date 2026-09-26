# rus_sim_planning

> 规划层：把感知点云表面变成一条扫查轨迹，插值后以 125 Hz 伺服下发到驱动。

## 功能概述

- ✅ 轨迹生成：点云表面建图 + 路径搜索 + 平滑（`trajectory_generator`）
- ✅ 轨迹插值：相邻路径点补点（`trajectory_interpolator`）
- ✅ 伺服执行：`movel` 到起点 → `servo_start` → 125 Hz `servo_cart`，可暂停 / 恢复 / 中止
- ✅ 事件上报：`plan_done` / `scan_done` / `error` → `/module_events` → 前端 event
- ✅ 轨迹可视化：`/planned_trajectory`（PoseArray）
- 🚧 碰撞检查（`collision_checker` 已建，策略待接入主流程）

> 状态标记：✅ 已完成 🚧 进行中 📝 待办 ❌ 废弃

## 结构

```
rus_sim_planning/
├── include/rus_sim_planning/
│   ├── planning_node.hpp        # 节点：指令分发、状态缓存、伺服循环
│   ├── trajectory_generator.hpp # 点云 → 路径点序列
│   ├── trajectory_interpolator.hpp # 路径点 → 稠密轨迹
│   └── collision_checker.hpp    # 碰撞检查
├── src/                         # 上述实现 + main.cpp
├── config/planning_params.yaml
└── launch/planning.launch.py
```

## 节点：planning_node

| 项 | 值 |
|----|----|
| 可执行 | `ros2 run rus_sim_planning rus_sim_planning_node` |
| 启动 | `ros2 launch rus_sim_planning planning.launch.py` |
| 服务 | `/planning/command`（`CommandService`） |

### 输入 / 输出

| 方向 | 名称 | 类型 | 说明 |
|------|------|------|------|
| 输入（话题） | `/preprocessed_cloud` | `PointCloud2` | 规划输入（perception 的地图快照或当前帧） |
| 输入（话题） | `/driver/state` | `RobotState` | 当前 TCP 位姿、伺服起点 |
| 输入（服务） | `/planning/command` | `CommandService` | 由 bridge 路由 |
| 输出（服务调用） | `/driver/command` | `CommandService` | `servo_start` / `servo_cart` / `movel` / `stop` 等直发驱动 |
| 输出（话题） | `/module_events` | `ModuleEvent` | `plan_done` / `scan_done` / `error`（带 `client_id` → 前端 `ack_id`） |
| 输出（话题） | `/planned_trajectory` | `geometry_msgs/PoseArray` | 每次 `plan` 一条，供 RViz / `traj_vis.py` 调试 |

### 指令（`/planning/command`）

| 指令 | args | 说明 |
|------|------|------|
| `set_start_pose` | ≥3（3 = 位置 / 6 = 位置+RPY / 7 = 位置+四元数） | 设置扫查起点 |
| `set_end_pose` | ≥3（同上） | 设置扫查终点 |
| `plan` | 无 | 预扫查门 + 起终点门 → 生成轨迹；成功发 `plan_done`，失败发 `error` |
| `execute` | 无 | `movel` 到起点 → `servo_start` → 125 Hz `servo_cart`；结束 / 中止发 `scan_done` |
| `pause` / `resume` | 无 | 暂停 / 恢复伺服下发 |
| `reset` | ≤2（`[mode?, enable?]`） | 停伺服 + `stop` + 清轨迹 |
| `query_motion_done` | 无 | `[0/1]`：执行中或暂停中 = 0，空闲 = 1 |
| `stop` | 无 | 中止执行（bridge 自动模式扇出 `{PLANNING, DRIVER}`） |
| `pre_scan_done` | — | 代码已实现分支，但 `command_types.hpp` 无结构体 + bridge 未注册 → 前端不可达 |

### 事件（→ `/module_events`）

| 事件 | 触发 | 说明 |
|------|------|------|
| `plan_done` | `plan` 成功 | `ack_id` = `plan` 的指令 id |
| `scan_done` | 伺服执行完毕 / 被 `stop` 中止 | 成功或中止由 `success` 区分 |
| `error` | 前置门失败、轨迹生成 / 插值失败 | 失败 message 形如 `plan failed: 未完成预扫查` |

## 参数（`config/planning_params.yaml`）

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `point_cloud_topic` | string | `/preprocessed_cloud` | 规划输入点云 |
| `driver_state_topic` | string | `/driver/state` | 机械臂状态 |
| `driver_command_service` | string | `/driver/command` | 伺服 / movel 下发目标 |
| `servo_rate_hz` | double | 125.0 | 伺服指令发布频率 |
| `interpolate_points` | int | 10 | 相邻路径点插值点数 |
| `movel_timeout_sec` | double | 10.0 | `execute` 前 `movel` 到起点的等待超时 |
| `alpha` | double | 1.0 | 椭圆 Gabriel 条件参数 |
| `graph_k` / `normal_k` / `projection_k` | int | 30 / 30 / 30 | 建图 k-NN / 法线估计 / 重投影参数 |
| `tol` / `max_iter` | double / int | 1e-6 / 40 | 长度变化容差 / 最大迭代轮次 |
| `use_smoothing` / `lambda` / `mu` | bool / double / double | true / 0.63 / −0.65 | Taubin 平滑开关与参数 |

> ⚠️ `end_hold_sec`（终点保持时长，成员初值 1.0 s）**未参数化**，yaml 无法覆盖 —— 见 [draft §8.2](../DevelopmentGuide.draft.md)。

## 启动 / 常用

```bash
ros2 launch rus_sim_planning planning.launch.py
python3 src/rus_sim_bringup/scripts/traj_vis.py       # RViz 轨迹可视化
python3 src/rus_sim_bringup/scripts/exec_monitor.py   # 执行过程 TCP vs 轨迹监控
```

## 依赖

| 依赖 | 说明 | 状态 |
|------|------|------|
| Boost（graph） | 建图 / 图搜索 | ✅ |
| PCL / pcl_conversions | 点云法线、邻域查询 | ✅ |
| Eigen3 | 位姿与插值运算 | ✅ |
| rclcpp / rus_sim_interfaces / rus_sim_utils | ROS 与协议 | ✅ |
