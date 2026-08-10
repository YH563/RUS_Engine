# 模块事件通知（子模块 → bridge，触发式）
#
# 子模块完成长任务 / 发生错误时发布到 /module_events topic，
# bridge 订阅后转为前端统一 event 消息（ResultMessage）。
#
# 事件名取值约定（与 rus_sim_utils/protocol.hpp 的 Event 命名空间一致）：
#   pre_scan_done / plan_done / scan_done / motion_done / error
string event          # 事件名
bool   success        # 是否成功
string message        # 错误描述 / 附加说明（成功可为空）
float64[] result      # 处理结果（查询类带数据，操作类为空）

# 关联的前端 command id：bridge 转发指令时透传（CommandService 请求的 client_id），
# 用于生成 event 的 ack_id；0 = 无关联（如模块自发事件）
uint32 client_id

# 来源模块（"driver" / "planning" / "perception"），便于调试与过滤
string module

builtin_interfaces/Time stamp
