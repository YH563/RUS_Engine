#!/usr/bin/env python3
"""
将 URDF 编译为 MuJoCo MJCF XML 格式
用法: python convert_urdf_to_mjcf.py
"""

import mujoco
import os

# 路径配置
script_dir = os.path.dirname(os.path.abspath(__file__))
urdf_path = os.path.join(script_dir, "fairino3_v6_mujoco.urdf")
output_path = os.path.join(script_dir, "robot_mujoco.xml")

print(f"📂 正在编译: {urdf_path}")

# 编译 URDF 为 MuJoCo 模型
model = mujoco.MjModel.from_xml_path(urdf_path)

# 导出为 MJCF XML 文件 (mj_saveLastXML 将最近编译的模型保存为 MJCF)
mujoco.mj_saveLastXML(output_path, model)

print(f"✅ 转换完成！输出文件: {output_path}")
print(f"   模型包含 {model.nq} 个自由度")
