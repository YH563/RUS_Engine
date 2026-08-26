# 机械臂状态（驱动发布，125Hz）
std_msgs/Header header          # 标准时间戳（仿真/驱动时间）
float64[] joint_pos
float64[] joint_vel
float64[] joint_acc
float64[] effort
float64[] flange_pos
int32 tool_index                # 当前工具坐标系索引（0~14，0 表示法兰坐标系）
float64[] tool_pose             # 当前 TCP 位姿（基坐标系下）XYZABC [m/rad]
