# 增量网格帧（重建 → bridge → 前端 /mesh 通道，协议 §3.4）
#
# 一条帧 = 一个批次：本重建 tick 的全部脏块（upsert + remove）。
# 线格式：uint32 LE 头长度 + JSON 头 + payload（与 SensorFrame 同构），
# 由 rus_sim_utils/protocol.hpp 的 EncodeMeshFrame/DecodeMeshFrame 编解码。
# payload（解压后）按 chunks 顺序拼接：每 upsert 块
#   int16 pos[verts*3]（块局部） + int8 normal[verts*3]（has_normals 时）
# 三角汤 + 隐式顺序索引。

builtin_interfaces/Time stamp
uint32  seq             # 帧序号（每帧 +1）
string  frame_id        # 顶点坐标系（base_link；已算好变换）
string  scope           # "delta"（增量）/ "map"（全量重同步）
string  encoding        # payload 压缩算法："raw" / "zstd"
string  pos_dtype       # 顶点量化类型（当前固定 "int16"）
string  normal_dtype    # 法线量化类型（当前固定 "int8"）

MeshChunkMeta[] chunks  # 本批次块元数据（按 payload 顺序）
uint8[] data            # 拼接 payload（已量化；按 encoding 压缩）
