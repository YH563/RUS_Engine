# 机械臂状态（驱动发布，125Hz）
std_msgs/Header header          # 标准时间戳（仿真/驱动时间）
float64[] joint_pos
float64[] joint_vel
float64[] joint_acc
float64[] effort
float64[] flange_pos
