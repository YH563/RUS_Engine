# rus_sim_driver

> 机械臂驱动层：对外提供统一运动指令与 125 Hz 状态流，对内屏蔽「MuJoCo 仿真 / Fairino 真机」差异。

## 功能概述

- ✅ 双后端驱动：`sim`（MuJoCo）/ `real`（Fairino SDK），可运行期 `switch_driver` 切换
- ✅ 运动指令：`movej` / `movel` / `servoj` / `servo_cart` / 点动（`start_jog` 等）
- ✅ 状态发布：`/driver/state`（关节 + 法兰 + TCP，125 Hz）与 `/joint_states`
- ✅ 工具坐标系：六点标定计算、索引切换、运行时缓存与持久化
- ✅ 控制子层：CTC 计算力矩控制 / 重力补偿（`controller/`，由仿真驱动 `sim_driver` 使用）

> 状态标记：✅ 已完成 🚧 进行中 📝 待办 ❌ 废弃

## 结构

```
rus_sim_driver/
├── include/rus_sim_driver/driver_node.hpp   # 节点声明
├── include/driver/                          # robot_driver.hpp（IRobotDriver + DriverFactory）
│                                            # sim_driver（MuJoCo）/ real_driver（Fairino SDK）
├── include/trajectory/                      # trajectory_executor + servo/planned/jog segment
├── include/controller/                      # controller / ctc_controller / gravity_comp_controller
├── include/components/                      # command_defs（驱动私有指令别名）/ kinematics / types
├── src/                                     # 与 include 一一对应的实现 + main.cpp
├── launch/                                  # driver.launch.py、keyboard_control.launch.py
├── robot_model/                             # fairino3_v6.urdf + convert_urdf_to_mjcf.py
├── scripts/keyboard_control.py              # 键盘点动前端
├── config/driver_params.yaml
├── SDK/libfairino/                          # 厂商 SDK
└── cmake/Findmujoco.cmake
```

## 节点：driver_node

| 项 | 值 |
|----|----|
| 可执行 | `ros2 run rus_sim_driver rus_sim_driver_node` |
| 启动 | `ros2 launch rus_sim_driver driver.launch.py`（驱动 + `robot_state_publisher`，RViz 已注释） |
| 服务 | `/driver/command`（`CommandService`） |

### 输入 / 输出

| 方向 | 名称 | 类型 | 频率 | 说明 |
|------|------|------|------|------|
| 输入（服务） | `/driver/command` | `CommandService` | 事件触发 | 调用方：bridge（路由）、planning（`servo_cart` / `movel` / `stop` 直发） |
| 输出（话题） | `/driver/state` | `RobotState` | 125 Hz（8 ms 定时器） | 关节 + 法兰 `flange_pos` + TCP `tool_pose` |
| 输出（话题） | `/joint_states` | `sensor_msgs/JointState` | 125 Hz | 关节名 `j1..j6` → `robot_state_publisher` |

### 指令（`/driver/command`）

| 分组 | 指令 |
|------|------|
| 运动 | `movej` `movel` `servoj` `servo_cart` `start_jog` `stop_jog_decel` `stop_jog_immediate` |
| 伺服 | `servo_start` `servo_end` |
| 运动控制 | `stop` `pause` `resume` `reset` |
| 驱动 | `connect` `disconnect` `is_connected` `is_in_drag_teach` `robot_enable` `get_state` `is_motion_done` `query_motion_done` |
| 切换 / 查询 | `switch_driver` `get_driver_type` |
| 文件 | `run_file`（执行 `script_path` 指定的指令文件） |
| 仿真专用 | `set_time_speed` `get_time_speed` `get_sim_time` `step_once` `get_frame_rate` |
| 工具坐标系 | `set_tool_calib_point` `compute_tool_calib` `set_tool_coord` `set_tool_index` `get_tool_coords` |

> args 校验与单位约定见 [draft §6.3](../DevelopmentGuide.draft.md)（位置 m / 角度 rad；`movej`、`movel` 的
> `speed`、`acc` 为比例 [0~1]，`start_jog` 的 `speed%`、`acc%` 为百分比 [0~100]）。
> ⚠️ 工具坐标系 5 条指令驱动侧已实现、bridge 也注册了路由，但 `rus_sim_utils/command_types.hpp`
> 尚无对应结构体 → 前端暂不可达（当前只有 `rus_sim_bringup/scripts/tool_calib_six_point.py` 等 ROS 侧脚本使用）。

## 参数（`config/driver_params.yaml`）

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `driver_type` | string | `"sim"` | `sim` / `real`（⚠️ 仓库文件里是 `"real"`，切真机用） |
| `robot_ip` | string | `""` | `connect` / `switch_driver` 使用的机器人 IP |
| `script_path` | string | `""` | `run_file` 的指令文件 |
| `tool_coords` | double[] | `[0,0,0,0,0,0]` | 每 6 个一组 `[x,y,z,rx,ry,rz]`（TCP 相对法兰），0 = 法兰 |
| `tool_index` | int | 0 | 当前工具坐标系索引 |
| `tool_coords_file` | string | `~/.rus_sim/tool_coords.yaml` | 标定结果持久化文件（留空用默认） |
| `jog_max_dis_joint` | double | 1.5708 | 关节点动单次位移上限（rad）；0 = 不限 |
| `jog_max_dis_trans` | double | 0.15 | 笛卡尔平移点动上限（m）；0 = 不限 |
| `jog_max_dis_rot` | double | 1.5708 | 笛卡尔旋转点动上限（rad）；0 = 不限 |

> 点动上限**只由配置决定**：前端 `start_jog` 传入的 `max_dis` 仅解析、不参与控制（见 `driver_node::jog_limit_for`）。

## 启动 / 常用

```bash
ros2 launch rus_sim_driver driver.launch.py                # 驱动 + robot_state_publisher
ros2 launch rus_sim_driver keyboard_control.launch.py      # 键盘点动联调
ros2 service call /driver/command rus_sim_interfaces/srv/CommandService \
  "{command: 'get_driver_type', args: []}"
```

## 依赖

| 依赖 | 说明 | 状态 |
|------|------|------|
| MuJoCo | 仿真后端（自定义 `Findmujoco.cmake`） | ✅ |
| Fairino SDK `libfairino.so.2.1.4` | 真机后端（随包 `SDK/libfairino`） | ✅ |
| EAIK | 逆运动学求解（`third_party/EAIK`） | ✅ |
| Eigen3 / PCL | 数学与几何 | ✅ |
| `urdf` / `ament_index` | 模型与资源路径 | ✅ |
