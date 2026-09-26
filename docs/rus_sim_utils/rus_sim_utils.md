# rus_sim_utils

> 协议层公共库（**header-only**）：全系统的通道路径、指令名、事件名、指令参数校验、路由注册表、位姿工具都定义在这里。

## 功能概述

- ✅ 协议字符串常量：WS 通道路径 / 指令名 / 事件名 / 感知帧类型与语义（`command_defs.hpp`）
- ✅ 指令结构体 + `ParseArgs` 参数校验 + 编译期查找表 `ParseCommand()`（`command_types.hpp`）
- ✅ 路由注册表：`Module` 枚举 → 服务名、指令 → 目标模块（`command_registry.hpp`）
- ✅ 统一消息与线格式：`CommandMessage` / `ResultMessage` / SensorFrame 二进制编解码（`protocol.hpp`）
- ✅ 通用状态与位姿工具：位姿 ↔ 矩阵、`MakePose`、法兰 ↔ 探头变换（`robot_state.hpp` / `utils.hpp`）

> 状态标记：✅ 已完成 🚧 进行中 📝 待办 ❌ 废弃

## 结构

```
rus_sim_utils/
├── include/rus_sim_utils/
│   ├── command_defs.hpp     # 纯字符串常量：WsPath / CmdName / EventName / SensorType / SensorScope
│   ├── command_types.hpp    # 指令结构体 + ParseArgs + CommandVariant + ParseCommand
│   ├── command_registry.hpp # Module 枚举、module_service_name()、CommandRegistry（Register/SetTargets/TargetsOf/IsLocal）
│   ├── protocol.hpp         # Channel / CommandMessage / ResultMessage、JSON 序列化、SensorFrame 编解码
│   ├── robot_state.hpp      # ControlTarget、RobotState（无 SDK 依赖的通用结构）
│   └── utils.hpp            # 位姿数学：MakePose / PoseToRPY / FlangePosToPose / FlangeToProbe
└── CMakeLists.txt           # add_library(rus_sim_utils INTERFACE)
```

## 使用方式

无节点、无话题，是被其它包 `#include` 的静态依赖：

```cpp
#include "rus_sim_utils/command_defs.hpp"

RusUtils::WsPath::kControl;              // "/control"
RusUtils::CmdName::kRecorderStart;       // "recorder_start"
RusUtils::EventName::kReplayDone;        // "replay_done"
```

## 关键约定

| 约定 | 内容 |
|------|------|
| 单位 | 位置 **m**、角度 **rad**；RPY 为固定轴 XYZ（`R = Rz·Ry·Rx`），可直接回填 `/driver/state` |
| 指令名唯一 | 只在此处定义，其它模块不得写裸字符串（`rus_sim_driver` 内的 `Cmd` 常量为驱动私有别名） |
| 参数校验 | 有 `ParseArgs` 的指令才接受参数；无该函数的指令**不接受任何参数**，多传即 `invalid args` |
| 服务名推导 | `module_service_name(Module::X)` → `/<x>/command`，bridge 与各模块共用，避免硬编码不一致 |

## 修改须知

1. 新增指令：`command_defs.hpp` 加常量 → `command_types.hpp` 加结构体与 `ParseArgs` → 进 `CommandVariant` 清单；
   之后 bridge 的 `init_routing()` 注册路由，目标模块实现 `handle_command` 分支。
2. `CommandVariant` 是**编译期**生成的查找表，漏登记结构体会直接导致 `unknown command`（见 [draft §6.6](../DevelopmentGuide.draft.md)）。
3. 改动本包会影响全部模块，需全量 `colcon build` 后回归（header-only → 所有依赖方重编）。

## 依赖

| 依赖 | 说明 | 状态 |
|------|------|------|
| C++17 标准库 | 无第三方依赖（不含 ROS 消息头） | ✅ |
| Eigen3 | 仅 `utils.hpp` 的位姿运算使用 | ✅ |
