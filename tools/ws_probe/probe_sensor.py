#!/usr/bin/env python3
"""ROS 侧探针：打印 /sensor/pointcloud（SensorFrame）+ /preprocessed_cloud 的关键元数据。

QoS 用 transient_local + reliable，与 perception 的 map_qos 对齐（不然只在发布时刻才能收到）。

用法：
  source install/setup.bash
  python3 probe_sensor.py                                  # 默认探 /sensor/pointcloud
  python3 probe_sensor.py --duration 15 --topic /sensor/pointcloud
"""
import argparse
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

from rus_sim_interfaces.msg import SensorFrame
from sensor_msgs.msg import PointCloud2

# SensorFrame.type 是 uint8 常量，打印成名字便于联调判读
TYPE_NAMES = {
    SensorFrame.TYPE_POINTCLOUD: "pointcloud",
    SensorFrame.TYPE_IMAGE: "image",
    SensorFrame.TYPE_ULTRASOUND: "ultrasound",
}


class Probe(Node):
    def __init__(self, topic, cloud_topic):
        super().__init__("probe_sensor")
        qos = QoSProfile(depth=1,
                         durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=QoSReliabilityPolicy.RELIABLE)
        self.create_subscription(SensorFrame, topic, self.on_sf, qos)
        self.create_subscription(PointCloud2, cloud_topic, self.on_pc, qos)
        self.got = False

    def on_sf(self, m):
        type_name = TYPE_NAMES.get(m.type, f"unknown({m.type})")
        print(f"[sensor] type={type_name} points={m.points} fields={list(m.fields)} dtype={m.dtype} "
              f"frame_id={m.frame_id!r} scope={m.scope!r} encoding={m.encoding!r} seq={m.seq} "
              f"range_min={[round(v, 4) for v in m.range_min]} "
              f"range_max={[round(v, 4) for v in m.range_max]} "
              f"payload={len(m.data)}B", flush=True)
        self.got = True

    def on_pc(self, m):
        print(f"[cloud ] width={m.width} height={m.height} point_step={m.point_step} "
              f"frame_id={m.header.frame_id!r} data={len(m.data)}B", flush=True)


def main():
    parser = argparse.ArgumentParser(description="感知帧 ROS 侧探针")
    parser.add_argument("--topic", default="/sensor/pointcloud", help="SensorFrame 话题")
    parser.add_argument("--cloud-topic", default="/preprocessed_cloud", help="PointCloud2 话题")
    parser.add_argument("--duration", type=float, default=8.0, help="等首帧最长秒数（默认 8）")
    args = parser.parse_args()

    rclpy.init()
    node = Probe(args.topic, args.cloud_topic)
    t0 = time.time()
    while time.time() - t0 < args.duration and not node.got:
        rclpy.spin_once(node, timeout_sec=0.5)
    # 再多收 1.5s：让 /preprocessed_cloud 的 transient_local 样本也打印出来
    t1 = time.time()
    while time.time() - t1 < 1.5:
        rclpy.spin_once(node, timeout_sec=0.3)
    print("probe done", flush=True)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
