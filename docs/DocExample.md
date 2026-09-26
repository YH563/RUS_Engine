# 包名

> 一句话说明核心功能，例如：点云滤波与可视化节点。


## 功能概述

- ✅ 功能点1：体素滤波
- 🚧 功能点2：统计滤波
- 📝 功能点3：点云分割

> 状态标记：✅ 已完成  🚧 进行中  📝 待办  ❌ 废弃


## 结构

> 列关键文件 / 子目录即可（一句话说明每层职责），不必贴全量文件。

```
rus_sim_package/
├── include/rus_sim_package/xxx_node.hpp   # 节点声明
├── include/<子层>/                        # 算法 / 组件子层（可选，如 camera/、pointcloud/）
├── src/                                   # 与 include 一一对应的实现 + main.cpp
├── config/xxx_params.yaml                 # 参数文件
├── launch/xxx.launch.py                   # 启动文件
└── test/                                  # 测试（可选）
```

## 第三方依赖

| 依赖 | 说明 | 状态 |
|------|------|------|
| PCL | 点云处理库，用于滤波 | ✅ |
| Eigen3 | 线性代数库 | ✅ |

## 节点说明

### node_name_1

| 项 | 值 |
|----|----|
| 可执行 | `ros2 run rus_sim_package rus_sim_package_node` |
| 启动 | `ros2 launch rus_sim_package xxx.launch.py` |
| 服务 | `/xxx/command`（`CommandService`） |

**话题 / 服务（输入输出）**

| 话题名 | 订阅/发布 | 消息类型 | 说明 | 状态 |
|--------|-----------|----------|------|------|
| `/input_cloud` | 订阅 | PointCloud2 | 原始点云 | ✅ |
| `/output_cloud` | 发布 | PointCloud2 | 滤波后点云 | ✅ |

**参数**（`config/params.yaml`）

| 参数名 | 类型 | 默认值 | 说明 | 状态 |
|--------|------|--------|------|------|
| `leaf_size` | double | 0.05 | 体素边长（米） | ✅ |
| `max_points` | int | 10000 | 最大点数 | 🚧 |

## 启动 / 常用命令

```bash
ros2 launch rus_sim_package xxx.launch.py          # 单模块启动
ros2 launch rus_sim_bringup rus_sim.launch.py      # 全栈启动（含本模块）
```

## 注意 / 已知限制

> 写明「前端暂不可达」「参数未参数化」「与某模块的坐标系约定」这类坑，避免后人踩坑。

