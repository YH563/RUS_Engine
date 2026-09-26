# rus_sim_bringup

> 启动与联调层：一个 launch 拉起全系统，外加一组「不污染主代码」的联调 / 可视化脚本。

## 功能概述

- ✅ 一键启动：驱动 + 规划 + 感知 + 桥接（+ 可选录制），参数透传到子 launch
- ✅ 关键词开关：`record` / `record_autostart` / `load_test_cloud` / `input_pcd`
- ✅ 联调脚本集：全链路检测、执行监控、轨迹与坐标系可视化、点动 / movel / 标定测试
- 📝 模块级 gtest（`test/` 目录）尚未落地

> 状态标记：✅ 已完成 🚧 进行中 📝 待办 ❌ 废弃

## 结构

```
rus_sim_bringup/
├── launch/rus_sim.launch.py     # 组合子 launch（唯一入口）
└── scripts/                     # 联调脚本（不参与编译，手动 python3 运行）
    ├── check_pipeline.py        # 点云表面 vs 规划轨迹 vs 实际 TCP 全链路对齐检测
    ├── exec_monitor.py          # 伺服执行监控：实际 TCP z / RPY 曲线 vs 轨迹
    ├── traj_vis.py              # RViz 绘制 /planned_trajectory（PoseArray）
    ├── flange_vis.py            # RViz 显示法兰 / 相机 / 工具坐标系 TF（订阅 /driver/state）
    ├── tool_tf_vis.py           # 法兰 TF + 六点标定得到的工具坐标系 TF
    ├── movel_test.py            # 不经 planning，直接从点云表面取点逐段发 movel
    ├── movel_single_test.py     # 单目标点 movel 执行验证（目标 vs 实际）
    ├── tool_calib_six_point.py  # 交互式六点法工具坐标系标定（调 compute_tool_calib）
    ├── test_prescan.sh          # 预扫查流程控制（pre_scan_done → 起终点 → plan → execute 顺序调用）
    └── run_sensor_tf_rviz.sh    # 传感器坐标系可视化一键启动（driver(sim) + robot_state_publisher + flange_vis + RViz2）
```

无编译单元（`ament_cmake`，只 install `launch/` 与 `scripts/`）。

## 启动

```bash
ros2 launch rus_sim_bringup rus_sim.launch.py                        # 全栈（不含录制）
ros2 launch rus_sim_bringup rus_sim.launch.py record:=true           # 连录制（默认启动即录）
ros2 launch rus_sim_bringup rus_sim.launch.py record:=true record_autostart:=false
ros2 launch rus_sim_bringup rus_sim.launch.py load_test_cloud:=true  # 灌入联调测试点云
ros2 launch rus_sim_bringup rus_sim.launch.py --show-args            # 查看全部参数
```

### 参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `record` | `false` | 是否同时拉起 `rus_sim_recorder/launch/recorder.launch.py` |
| `record_dir` | `records` | 录制输出目录（转发给 recorder） |
| `record_autostart` | `true` | 录制是否启动即开始；`false` = 起来待命，由前端 `recorder_start` 开始 |
| `load_test_cloud` | `false` | 感知层启动时加载联调测试点云（仅联调） |
| `input_pcd` | `~/.rus_sim/test_data/test_cloud.pcd`（存在时） | 联调点云路径（`base_link` 系），受 `load_test_cloud` 控制 |

### 组合关系

```
rus_sim.launch.py
├── IncludeLaunchDescription → rus_sim_driver/launch/driver.launch.py
├── IncludeLaunchDescription → rus_sim_planning/launch/planning.launch.py
├── IncludeLaunchDescription → rus_sim_perception/launch/perception.launch.py   ← load_test_cloud / input_pcd
├── IncludeLaunchDescription → rus_sim_bridge/launch/bridge.launch.py
└── IncludeLaunchDescription → rus_sim_recorder/launch/recorder.launch.py       ← record:=true 时（autostart / output_dir 透传）
```

## 联调脚本速查

```bash
python3 src/rus_sim_bringup/scripts/traj_vis.py            # 轨迹可视化
python3 src/rus_sim_bringup/scripts/check_pipeline.py       # 全链路：点云 / 轨迹 / TCP 对齐
python3 src/rus_sim_bringup/scripts/exec_monitor.py        # 执行过程 TCP 监控
python3 src/rus_sim_bringup/scripts/tool_calib_six_point.py # 六点标定（交互式）
```

> 脚本用 ROS 2 CLI / rclpy 直连话题与服务，属**调试工具**：参数以各脚本头部注释为准，
> 不要把它们当作产品入口（前端侧的正式入口是 WS 协议）。

## 依赖

| 依赖 | 说明 | 状态 |
|------|------|------|
| `launch` / `launch_ros` / `ament_index_python` | launch 组合与 share 路径解析 | ✅ |
| `rclpy` / `sensor_msgs` / `visualization_msgs` | 脚本侧 | ✅ |
| `rus_sim_interfaces` | 脚本调用 CommandService 时使用 | ✅ |
