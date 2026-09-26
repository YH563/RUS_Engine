# rus_sim_perception

> 感知层：从相机取帧 → 与机械臂位姿时间对齐 → 点云预处理 / 累积建图 → 发布「规划用点云」与「前端压缩帧」。

## 功能概述

- ✅ 四种数据源：`realsense`（直连）/ `ros_topic` / `replay`（离线 PCD）/ `auto`
- ✅ 位姿时间对齐（`/driver/state` 插值）与点云坐标变换到 `base_link`
- ✅ 点云滤波链：直通 / 体素 / 统计（各阶段可独立开关）
- ✅ 累积建图：`none` / `rolling`（上限降采样）/ `accumulate`
- ✅ 前端压缩帧：zstd + int16 量化 + `range_min/max`（`SensorFrame`）
- 🚧 超声图像通路（`SensorFrame::TYPE_ULTRASOUND` 预留）

> 状态标记：✅ 已完成 🚧 进行中 📝 待办 ❌ 废弃

## 结构

```
rus_sim_perception/
├── include/rus_sim_perception/perception_node.hpp
├── include/camera/          # point_cloud_source（接口）/ realsense / ros_topic / replay / source_factory / frame_slot
├── include/pointcloud/      # cloud_filter_pipeline / map_manager / spatial_transformer / cloud_io
├── include/components/      # pose_interpolator（位姿插值）/ sensor_encoder（zstd+量化）/ types
├── src/                     # 上述实现 + main.cpp + tools/gen_test_cloud.cpp
├── config/perception_params.yaml
└── launch/perception.launch.py
```

## 节点：perception_node

| 项 | 值 |
|----|----|
| 可执行 | `ros2 run rus_sim_perception rus_sim_perception_node` |
| 启动 | `ros2 launch rus_sim_perception perception.launch.py`（可叠加 `source:=` / `replay_path:=` / `load_test_cloud:=` / `input_pcd:=`） |
| 服务 | `/perception/command`（`CommandService`） |
| 工具 | `ros2 run rus_sim_perception rus_sim_gen_test_cloud`（生成联调测试点云） |

### 输入 / 输出

| 方向 | 名称 | 类型 | 频率 | 说明 |
|------|------|------|------|------|
| 输入（话题） | `/driver/state` | `RobotState` | 125 Hz | 位姿时间对齐基准 |
| 输入（话题） | `/camera/camera/depth/color/points` | `PointCloud2` | 相机帧率 | 仅 `source=ros_topic` 使用（`realsense` 直连不经过该话题） |
| 输出（话题） | `/preprocessed_cloud` | `PointCloud2` | `rolling` / `accumulate`：地图快照（`map_publish_period` 2 s）；`none`：处理周期 10 Hz 当前帧 | **planning 的规划输入**，`base_link` |
| 输出（话题） | `/perception/frame` | `PointCloud2` | 处理周期 10 Hz | 当前帧（RViz 实时可视化） |
| 输出（话题） | `/sensor/pointcloud` | `SensorFrame` | 与 `/preprocessed_cloud` 同频 | 压缩帧 → bridge 的 `/sensor`、recorder 落盘 |

> QoS：两条点云输出 + `SensorFrame` 均为 reliable + `transient_local`（bridge / recorder 订阅端同口径）。
> `scope` 区分语义：`frame`（当前帧）/ `map`（地图快照）——前端据此决定整帧替换还是地图渲染。

### 指令（`/perception/command`）

| 指令 | 实现 | bridge 注册 | 现状 |
|------|------|------------|------|
| `map_clear` | ✅ perception（清空累积地图） | ❌ | 服务可直接调用，前端暂不可达 |
| `load_cloud` | ✅ perception（`args[0]`=索引 → `pcd_dir` 第 N 个 `.pcd`；无参 → `input_pcd`） | ❌ | 同上 |
| `pre_scan_start` / `pre_scan_end` / `query_prescan_done` | ❌ 未实现 | ✅ 路由到 perception | 转发后返回 `unknown command` |

> 详见 [draft §6.4](../DevelopmentGuide.draft.md)；预扫查流程归属尚未定案（draft §12）。

## 参数（`config/perception_params.yaml`，此处只列关键项）

| 参数 | 说明 |
|------|------|
| `source` | `realsense`（仓库文件值）/ `ros_topic` / `replay` / `auto`（代码默认值；空值按 `auto`） |
| `input_cloud_topic` / `output_cloud_topic` / `frame_topic` / `sensor_cloud_topic` / `driver_state_topic` | 五个话题名 |
| `mapping_mode` | `none`（发当前帧）/ `rolling`（默认，累积 + 上限降采样）/ `accumulate` |
| `map_publish_period` / `map_max_points` | 地图快照周期（2.0 s）/ 累积点数上限（500000） |
| `max_allowed_diff_sec` / `max_pose_cache` / `allow_stale_pose` | 位姿对齐容差（0.05 s）/ 缓存帧数 / 是否允许用过期位姿 |
| `process_period` | 处理周期（0.1 s） |
| `camera_to_flange` | 相机 → 法兰标定矩阵（16 元素，行优先） |
| `rs_serial` / `rs_width` / `rs_height` / `rs_fps` | 直连相机选择与流配置（640×480@15 默认） |
| `rs_align_to` / `rs_color_mode` / `rs_point_stride` | 光学系对齐（默认 `color`）/ 彩色取 RGB / 像素抽稀（1 = 全分辨率） |
| `rs_min_depth` / `rs_max_depth` | 有效深度区间（0.15 ~ 6.0 m） |
| `rs_decimation` / `rs_spatial_filter` / `rs_temporal_filter` | SDK 原生滤波（贴合 realsense-viewer 效果） |
| `replay_path` / `replay_fps` / `replay_loop` / `replay_frame_id` | `source=replay` 的 PCD 来源与节奏 |
| `input_pcd` / `pcd_dir` | 启动即加载的点云（`base_link` 系场景）/ 可切换的 PCD 目录 |
| `enable_passthrough_filter` / `enable_voxel_filter` / `enable_statistical_filter` | 三级滤波开关（仓库当前均为 false，效果对齐阶段） |
| `voxel_leaf_size` / `passthrough_field` / `passthrough_limit_min,_max` / `statistical_mean_k` / `statistical_std_dev_mul` | 各级滤波参数（`passthrough` 为 `base_link` 系 ROI，不是相机 FOV） |

## 启动 / 常用

```bash
ros2 launch rus_sim_perception perception.launch.py                          # 默认（auto 选源）
ros2 launch rus_sim_perception perception.launch.py source:=replay replay_path:=/path/to.pcd
ros2 launch rus_sim_perception perception.launch.py load_test_cloud:=true input_pcd:=/abs/test_cloud.pcd
python3 tools/ws_probe/probe_sensor.py                                       # 查看发布元数据
```

## 依赖

| 依赖 | 说明 | 状态 |
|------|------|------|
| libzstd | 点云压缩（缺失时 CMake 直接报错） | ✅ |
| librealsense2 | 直连相机（`QUIET` 查找，缺失自动降级为 stub） | ✅ |
| PCL / pcl_conversions | 点云容器与算法 | ✅ |
| Eigen3 | 坐标变换与插值 | ✅ |
| rclcpp / rus_sim_interfaces / rus_sim_utils | ROS 与协议 | ✅ |
