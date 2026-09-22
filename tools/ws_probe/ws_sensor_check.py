#!/usr/bin/env python3
"""WS /sensor 二进制通道校验：连上 bridge 按线格式解析真实感知帧。

校验内容（不做夹具内容比对，只校验线格式与自洽性）：
  1. 帧长自洽：4 + 头长 + payload 长度 == WS 二进制消息长度（一条消息 = 一帧）
  2. payload 按 encoding 解压后 == points * 10 字节（int16 x|y|z + uint32 rgb，小端）
  3. 反量化结果落在 range_min / range_max 之间（公式见 docs/Protocol/WsProtocol.md）

用法：
  python3 ws_sensor_check.py                      # ws://127.0.0.1:8765/sensor，收 2 帧
  python3 ws_sensor_check.py --port 8765 --frames 5

依赖：pip install websockets zstandard numpy
"""
import argparse
import asyncio
import json
import time

import numpy as np
import websockets
import zstandard as zstd

POINT_STRIDE = 10  # 每点字节数：int16 x | int16 y | int16 z | uint32 rgb


def parse_raw(payload):
    """裸字节 → (q int16[N,3], rgb uint32[N])。"""
    n = len(payload) // POINT_STRIDE
    a = np.frombuffer(payload[: n * POINT_STRIDE], dtype=np.uint8).reshape(n, POINT_STRIDE)
    q = np.empty((n, 3), dtype=np.int16)
    q[:, 0] = a[:, 0:2].copy().view(np.int16).ravel()
    q[:, 1] = a[:, 2:4].copy().view(np.int16).ravel()
    q[:, 2] = a[:, 4:6].copy().view(np.int16).ravel()
    rgb = a[:, 6:10].copy().view(np.uint32).ravel()
    return q, rgb


def dequantize(q, mn, mx):
    """int16 → 真实坐标（与协议文档同一公式）。"""
    mn = np.asarray(mn, dtype=np.float64)
    mx = np.asarray(mx, dtype=np.float64)
    return mn + (q.astype(np.float64) + 32768.0) * (mx - mn) / 65535.0


def verify(buf, idx, decompressor):
    """校验一帧，打印关键字段；返回解析出的 JSON 头。"""
    head_len = int.from_bytes(buf[0:4], "little")
    head = json.loads(buf[4:4 + head_len].decode("utf-8"))
    payload = buf[4 + head_len:]
    assert 4 + head_len + len(payload) == len(buf), f"帧 {idx}: 帧长不自洽"

    enc = head.get("encoding", "")
    if enc == "zstd":
        raw = decompressor.decompress(payload)
    elif enc in ("raw", "none"):
        raw = payload          # 未压缩 payload 可直接按点布局解析
    else:
        # jpeg / png 属 image/ultrasound 通道，本脚本只校验点云
        raise AssertionError(f"帧 {idx}: 不支持的编码 {enc!r}（点云应为 zstd/raw）")

    assert len(raw) == head["points"] * POINT_STRIDE, \
        f"帧 {idx}: payload {len(raw)}B != points*10 = {head['points'] * POINT_STRIDE}B"
    q, rgb = parse_raw(raw)
    assert len(q) == head["points"], f"帧 {idx}: 点数不符"

    xyz = dequantize(q, head["range_min"], head["range_max"])
    mn, mx = np.asarray(head["range_min"]), np.asarray(head["range_max"])
    assert (xyz >= mn - 1e-6).all() and (xyz <= mx + 1e-6).all(), f"帧 {idx}: 反量化越界"
    assert (q >= -32768).all() and (q <= 32767).all()

    print(f"帧 {idx}: ✓ seq={head['seq']} scope={head['scope']} frame_id={head['frame_id']} "
          f"enc={head['encoding']} points={head['points']} fields={head['fields']} dtype={head['dtype']}")
    print(f"        ws_msg={len(buf)}B (4+头 {head_len}B+payload {len(payload)}B) raw={len(raw)}B")
    print(f"        range_min={[round(v, 4) for v in head['range_min']]} "
          f"range_max={[round(v, 4) for v in head['range_max']]}")
    print(f"        首点 q=({q[0][0]},{q[0][1]},{q[0][2]}) rgb=0x{rgb[0]:06X} "
          f"→ xyz=({xyz[0][0]:.4f},{xyz[0][1]:.4f},{xyz[0][2]:.4f})")
    print(f"        xyz 范围: min={xyz.min(axis=0).round(4).tolist()} max={xyz.max(axis=0).round(4).tolist()}")
    return head


async def main():
    parser = argparse.ArgumentParser(description="WS /sensor 感知帧线格式校验")
    parser.add_argument("--port", type=int, default=8765, help="bridge WS 端口（默认 8765）")
    parser.add_argument("--frames", type=int, default=2, help="校验帧数（默认 2）")
    parser.add_argument("--timeout", type=float, default=10.0, help="单帧等待超时秒数（默认 10）")
    args = parser.parse_args()

    uri = f"ws://127.0.0.1:{args.port}/sensor"
    decompressor = zstd.ZstdDecompressor()
    t0 = time.time()
    async with websockets.connect(uri, max_size=None, open_timeout=5) as ws:
        print(f"已连接 {uri}")
        got = 0
        while got < args.frames and time.time() - t0 < args.timeout:
            buf = await asyncio.wait_for(ws.recv(), timeout=args.timeout)
            verify(buf, got, decompressor)
            got += 1
        assert got > 0, f"{args.timeout}s 内未收到任何感知帧（检查 perception 是否在发 /sensor/pointcloud）"
        print(f"── 校验通过：{got} 帧（感知帧线格式自洽）──")


if __name__ == "__main__":
    asyncio.run(main())
