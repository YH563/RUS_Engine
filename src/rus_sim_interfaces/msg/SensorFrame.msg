# 感知传感器帧（感知 → bridge → 前端 /sensor 通道）
#
# payload 为压缩后的字节（encoding 声明压缩算法），元数据供前端解码/渲染。
# WS 线格式（JSON 头 + payload）见 rus_sim_utils/protocol.hpp（EncodeSensorFrame）。

# ── 帧类型 ──
uint8 TYPE_POINTCLOUD = 0
uint8 TYPE_IMAGE      = 1
uint8 TYPE_ULTRASOUND = 2

uint8   type          # TYPE_*
string  encoding      # "zstd" / "jpeg" / "png" / "raw"
builtin_interfaces/Time stamp
uint32  seq           # 每类型独立递增（前端检测丢帧）
string  frame_id

# ── pointcloud 元数据 ──
uint32   points        # 点数
string[] fields        # 分量顺序，如 ["x","y","z","rgb"]
string   dtype         # 量化后类型，如 "int16"
float64[] range_min    # 量化包围盒 min [x,y,z]
float64[] range_max    # 量化包围盒 max [x,y,z]

# ── image / ultrasound 元数据（预留）──
uint32  width
uint32  height
string  image_encoding # 压缩前格式 rgb8 / mono8
uint32  step           # 压缩前每行字节数

uint8[] data           # 压缩后 payload
