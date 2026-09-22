# ws_probe —— 感知流联调探针

验证「perception → bridge → 前端」链路上 `/sensor` 通道是否真的通，以及线格式是否自洽。

| 脚本 | 位置 | 作用 |
| --- | --- | --- |
| `probe_sensor.py` | ROS 侧 | 打印 `/sensor/pointcloud`(SensorFrame) 与 `/preprocessed_cloud` 的元数据 |
| `ws_sensor_check.py` | WS 侧 | 连 bridge 的 `/sensor` 通道，按线格式解析真实帧并校验自洽性 |

---

## 1. 起步：把点云发起来

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

# 全栈（默认即自动加载 ~/.rus_sim/test_data/test_cloud.pcd）
ros2 launch rus_sim_bringup rus_sim.launch.py

# 只想测感知 + 桥接：
ros2 launch rus_sim_perception perception.launch.py
ros2 launch rus_sim_bridge bridge.launch.py
```

感知层点云开关（`perception.launch.py` / `rus_sim.launch.py` 都支持）：

```bash
ros2 launch rus_sim_perception perception.launch.py load_test_cloud:=false      # 不灌测试点云
ros2 launch rus_sim_perception perception.launch.py input_pcd:=/abs/other.pcd   # 换一份点云
```

> 注意：不要用 `input_pcd:=""` 关闭。shell 会剥掉引号使其变成 `input_pcd:=`，
> `ros2 launch` 判定 malformed 会拒绝启动整个 launch；关闭请用 `load_test_cloud:=false`。

## 2. ROS 侧：帧探针

```bash
python3 tools/ws_probe/probe_sensor.py
python3 tools/ws_probe/probe_sensor.py --topic /sensor/pointcloud --duration 15
```

默认测试点云的预期输出：

```
[sensor] type=pointcloud points=12120 fields=['x', 'y', 'z', 'rgb'] dtype=int16 frame_id='base_link'
         scope='map' encoding='zstd' seq=... payload=88479B
```

## 3. WS 侧：线格式校验

```bash
pip install websockets zstandard numpy    # 首次

python3 tools/ws_probe/ws_sensor_check.py
python3 tools/ws_probe/ws_sensor_check.py --port 8765 --frames 5
```

校验链路：帧长自洽 → 按 `encoding` 解压后 `len == points*10` → 反量化落在
`range_min`/`range_max` 内（`int16` 归一化公式与协议文档同一份）；通过则打印首点与 xyz 包围盒。

```
帧 0: ✓ seq=7 scope=map frame_id=base_link enc=zstd points=12120 fields=['x','y','z','rgb'] dtype=int16
        ws_msg=88743B (4+头 260B+payload 88479B) raw=121200B
        xyz 范围: min=[-0.5, -0.3, -0.0545] max=[0.0, 0.3, 0.0227]
```

## 4. 排查顺序（收不到帧时）

1. `ros2 topic hz /sensor/pointcloud` —— 发布端有没有在发（约 0.5 Hz，mapping_mode=rolling 时 `scope=map`）
2. `python3 tools/ws_probe/probe_sensor.py` —— QoS 不匹配是最常见原因；本脚本已用 `transient_local + reliable`
3. `python3 tools/ws_probe/ws_sensor_check.py` —— ROS 侧有帧但前端没有，则看 bridge 日志
   `感知流：转发 <- /sensor/pointcloud`（若打印 `关闭`，检查 `forward_sensor` 参数）
4. 帧到但内容异常 —— 用本脚本的 `raw` / `xyz 范围` 与 `range_min`/`range_max` 比对

协议细节见 `docs/Protocol/WsProtocol.md` 的 `/sensor` 章节。
