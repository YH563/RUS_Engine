# 一个增量网格块的元数据（重建 → bridge → 前端 /mesh 通道）
#
# chunk = 一个 TSDF 数据块（8^3 体素），整块 upsert/remove（不是逐三角面 delta）。
# 顶点为【块局部】坐标，反量化需用 range_min/range_max（与点云同一公式）。
# 三角汤 + 隐式顺序索引：verts = 3 * tris。

uint8 KIND_UPSERT = 0
uint8 KIND_REMOVE = 1

int64   chunk_id        # 稳定块身份（后端 PackChunkId，块索引各 21 位打包）
uint8   kind            # KIND_*
float64[3] origin       # 块世界原点（顶点相对它，保 float 精度）
int64   revision        # 单调版本号（前端 <= 已应用则丢弃，乱序守卫）
uint32  verts           # 顶点数（= 3*tris；remove 时为 0）
uint32  tris            # 三角面数
float64[3] range_min    # 块局部量化包围盒 min
float64[3] range_max    # 块局部量化包围盒 max
bool    has_normals     # data 中该块是否含 int8 法线
