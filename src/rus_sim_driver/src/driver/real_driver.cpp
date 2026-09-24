#include "driver/real_driver.hpp"
#include "robot.h"
#include "robot_types.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace RusRealRobotDriver {

    namespace {
        // 单位换算常量
        constexpr double kDeg2Rad = M_PI / 180.0;   // 度 → 弧度
        constexpr double kRad2Deg = 180.0 / M_PI;   // 弧度 → 度
        constexpr double kMm2M    = 1.0 / 1000.0;   // 毫米 → 米
        constexpr double kM2Mm    = 1000.0;         // 米 → 毫米
        // 点动位移上限「不限制」的近似值：SDK 的 max_dis 必须 > 0（传 0 → 机械臂不动）；
        // 单位随点动类型解释：关节/笛卡尔旋转轴 = °，笛卡尔平移轴 = mm。
        // 实际点动会在到达该上限前被 stop_jog_decel / stop_jog_immediate 终止。
        // 用于 driver_params.yaml 中 jog_max_dis_* = 0（不限制）的情形。
        constexpr float  kJogMaxDisUnlimited = 3600.0f;

        // RPC/SDK 错误码 → 可读描述（用于连接失败诊断）
        const char* rpc_error_str(int err)
        {
            switch (err) {
                case 0:                return "ERR_SUCCESS 成功";
                case ERR_OTHER:        return "ERR_OTHER 其他错误";
                case ERR_SOCKET_COM_FAILED:  return "ERR_SOCKET_COM_FAILED 网络通讯异常（常见原因：SDK 与控制器固件版本不匹配，SDK 内部会报 'error SDK version'）";
                case ERR_XMLRPC_COM_FAILED:  return "ERR_XMLRPC_COM_FAILED XMLRPC 通讯失败，请检查网络连接以及服务器IP地址是否正确";
                case ERR_XMLRPC_CMD_FAILED:  return "ERR_XMLRPC_CMD_FAILED XMLRPC 接口执行失败";
                default: return "未知错误码";
            }
        }

        // DescPose (mm/deg) → 齐次矩阵 (m/rad)；固定轴 XYZ，R = Rz·Ry·Rx
        Eigen::Matrix4d desc_pose_to_matrix(const DescPose& p)
        {
            Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
            T.block<3, 3>(0, 0) =
                (Eigen::AngleAxisd(p.rpy.rz * kDeg2Rad, Eigen::Vector3d::UnitZ())
               * Eigen::AngleAxisd(p.rpy.ry * kDeg2Rad, Eigen::Vector3d::UnitY())
               * Eigen::AngleAxisd(p.rpy.rx * kDeg2Rad, Eigen::Vector3d::UnitX())).toRotationMatrix();
            T.block<3, 1>(0, 3) << p.tran.x * kMm2M, p.tran.y * kMm2M, p.tran.z * kMm2M;
            return T;
        }

        // 齐次矩阵 → [x,y,z,rx,ry,rz] (m/rad)
        void matrix_to_pose6(const Eigen::Matrix4d& T, Eigen::VectorXd& pose6)
        {
            const Eigen::Matrix3d R = T.block<3, 3>(0, 0);
            pose6.resize(6);
            pose6 << T(0, 3), T(1, 3), T(2, 3),
                std::atan2(R(2, 1), R(2, 2)),
                std::asin(-R(2, 0)),
                std::atan2(R(1, 0), R(0, 0));
        }

        // SDK 状态（fairino）→ 通用 RobotState；角度 度 → 弧度，位置 mm → m
        RusRobotDriver::RobotState convert_sdk_state(
            JointPos& jPos, float speed[6], float acc[6],
            float torques[6], DescPose& flange, int tool_index,
            const Eigen::VectorXd& tool_pose, float time_ms)
        {
            RusRobotDriver::RobotState s;

            // 法兰位姿 XYZABC：位置 [mm→m]，姿态 [deg→rad]
            s.flange_pos.resize(6);
            s.flange_pos << flange.tran.x * kMm2M, flange.tran.y * kMm2M, flange.tran.z * kMm2M,
                flange.rpy.rx * kDeg2Rad, flange.rpy.ry * kDeg2Rad,
                flange.rpy.rz * kDeg2Rad;

            // 工具坐标系信息：当前工具索引 + TCP 基座位姿（本地计算：法兰 × 工具变换）
            s.tool_index = tool_index;
            s.tool_pose = tool_pose;

            // 关节位置 / 速度 / 加速度，deg / (deg/s) / (deg/s²) → rad / (rad/s) / (rad/s²)
            s.joint_pos = Eigen::Map<Eigen::VectorXd>(jPos.jPos, 6) * kDeg2Rad;
            s.joint_vel = Eigen::Map<Eigen::VectorXf>(speed, 6).cast<double>() * kDeg2Rad;
            s.joint_acc = Eigen::Map<Eigen::VectorXf>(acc, 6).cast<double>() * kDeg2Rad;

            // 关节力矩 [Nm]
            s.effort = Eigen::Map<Eigen::VectorXf>(torques, 6).cast<double>();

            // SDK 系统时间 [ms] → 秒（与仿真时钟 sim_time_ 单位一致）
            s.timestamp = static_cast<double>(time_ms) * 0.001;
            return s;
        }
    }  // namespace

    RobotRealDriver::RobotRealDriver(const std::string& ip)
    {
        // 注意：FRRobot 成员由成员构造自动默认构造，绝不能再次 `robot = FRRobot();`。
        // FRRobot 内部持有原始指针（FRTcpClient*）与 socket 缓冲，默认拷贝赋值会
        // 造成临时对象析构后成员悬垂，导致堆内存损坏（malloc: unaligned tcache chunk）。

        // 对齐官方示例：先初始化 SDK 日志并设置过滤级别（1=error）。
        // 未调用 LoggerInit 时 SDK 内部错误（如 RecvPkg 的 "error SDK version"）
        // 不会输出，连接失败将无法定位根因。
        robot.LoggerInit();
        robot.SetLoggerLevel(1);

        // Connect 内部会更新 is_connected_ 与 last_rpc_error_，
        // 连接失败时调用方可通过 IsConnected() / LastRpcError() 检查原因
        Connect(ip);
        // 开启 SDK 自动重连：断线后每 500ms 尝试重连，最长 30s
        robot.SetReConnectParam(true, 30000, 500);
    }

    int RobotRealDriver::Connect(const std::string& ip)
    {
        int rtn = robot.RPC(ip.c_str());
        last_rpc_error_.store(rtn);
        is_connected_.store(rtn == 0);
        if (rtn != 0) {
            // 直接输出到 stderr：驱动库不依赖 rclcpp，连接失败信息必须可见
            fprintf(stderr,
                "[RobotRealDriver] RPC 连接失败 ip=%s 错误码=%d (%s)\n",
                ip.c_str(), rtn, rpc_error_str(rtn));
        }
        return rtn;
    }

    int RobotRealDriver::GetCurrentState(uint8_t flag, RobotState& robot_state)
    {
        if (!is_connected_.load()) return -1;  // 未连接时直接返回，避免 SDK 报错/脏数据
        JointPos jPos;
        float speed[6];
        float acc[6];
        float torques[6];
        DescPose flange;
        int tool_index = 0;
        float time_ms;

        // 任一查询失败立即返回错误码，避免把脏数据当真实状态发布
        int ret = robot.GetActualJointPosDegree(flag, &jPos);
        if (ret != 0) return ret;
        ret = robot.GetActualJointSpeedsDegree(flag, speed);
        if (ret != 0) return ret;
        ret = robot.GetActualJointAccDegree(flag, acc);
        if (ret != 0) return ret;
        ret = robot.GetJointTorques(flag, torques);
        if (ret != 0) return ret;
        ret = robot.GetActualToolFlangePose(flag, &flange);
        if (ret != 0) return ret;
        // 当前工具索引 = 驱动本地（set_tool_index 设置，MoveJ/MoveL 实际使用的工具号）
        tool_index = tool_index_.load();
        ret = robot.GetSystemClock(&time_ms);
        if (ret != 0) return ret;

        // tool_pose = 法兰 × 本地工具变换（与配置一致；控制器 GetActualTCPPose 恒按工具 0 计算，不可用）
        Eigen::Matrix4d T_flange = desc_pose_to_matrix(flange);
        Eigen::Matrix4d T_tcp;
        {
            std::lock_guard<std::mutex> lock(tool_mtx_);
            const int tidx = std::min<int>(tool_index, static_cast<int>(tool_transforms_.size()) - 1);
            T_tcp = T_flange * tool_transforms_[static_cast<size_t>(tidx)];
        }
        Eigen::VectorXd tcp_pose6;
        matrix_to_pose6(T_tcp, tcp_pose6);

        robot_state = convert_sdk_state(jPos, speed, acc, torques, flange, tool_index, tcp_pose6, time_ms);
        return 0;
    }

    // 关节空间运动
    int RobotRealDriver::MoveJ(MotionCommand& joint_command)
    {
        if (!is_connected_.load()) return -1;
        if (joint_command.target.size() < 6) return -1;
        JointPos jp;
        for (int i = 0; i < 6; ++i)
            jp.jPos[i] = joint_command.target(i) * kRad2Deg;  // rad → deg

        // SDK 旧版 MoveJ 要求 desc_pos 与 joint_pos 相互一致（控制器内部做 FK/IK 校验）。
        // 必须先 GetForwardKin 用目标关节角算出相符的笛卡尔位姿作为 desc_pos，
        // 否则 0 值 / 未初始化位姿与 FK 结果不符 → 控制器报错（如错误码 154）。
        DescPose desc;
        int fk_rtn = robot.GetForwardKin(&jp, &desc);
        if (fk_rtn != 0) {
            fprintf(stderr,
                "[RobotRealDriver] MoveJ 正运动学计算失败 错误码=%d (%s)，q=[%.2f %.2f %.2f %.2f %.2f %.2f]deg\n",
                fk_rtn, rpc_error_str(fk_rtn),
                jp.jPos[0], jp.jPos[1], jp.jPos[2],
                jp.jPos[3], jp.jPos[4], jp.jPos[5]);
            return fk_rtn;
        }
        ExaxisPos epos(0, 0, 0, 0);
        DescPose offset(0, 0, 0, 0, 0, 0);

        // vel/acc：比例 [0~1] → 百分比 [0~100]；ovl=100 不额外缩放；
        // blendT=0 非阻塞（立即返回，运动由控制器队列执行，完成状态用 IsMotionDone 轮询）
        // 注意：MoveJ 是关节空间运动，目标为关节角，与工具坐标系无关。
        // GetForwardKin 返回的是法兰（tool=0）位姿，因此 tool 必须固定为 0，
        // 与 desc_pos 坐标系一致（对齐官方示例：MoveJ 用 tool=0 + FK 位姿）。
        // 不能沿用 tool_index_（若非 0，控制器会把法兰位姿当 tool=N 位姿校验 → 不一致报错）。
        int rtn = robot.MoveJ(&jp, &desc, 0, 0,
            static_cast<float>(joint_command.speed * 100.0),
            static_cast<float>(joint_command.acceleration * 100.0),
            100.0f, &epos, 0.0f, 0, &offset);
        if (rtn != 0) {
            fprintf(stderr,
                "[RobotRealDriver] MoveJ 失败 错误码=%d (%s)，q=[%.2f %.2f %.2f %.2f %.2f %.2f]deg "
                "vel=%.1f acc=%.1f\n",
                rtn, rpc_error_str(rtn),
                jp.jPos[0], jp.jPos[1], jp.jPos[2],
                jp.jPos[3], jp.jPos[4], jp.jPos[5],
                static_cast<float>(joint_command.speed * 100.0),
                static_cast<float>(joint_command.acceleration * 100.0));
        }
        return rtn;
    }

    // 笛卡尔空间直线运动
    int RobotRealDriver::MoveL(MotionCommand& desc_command)
    {
        if (!is_connected_.load()) return -1;
        // 支持两种：>=6 为 [x,y,z,rx,ry,rz]（显式位姿）；==3 为 [x,y,z]（仅位置，姿态保持当前 TCP 姿态）
        if (desc_command.target.size() < 3) return -1;

        const int tool = tool_index_.load();

        // 工具变换 T_tool（TCP 相对法兰；约定与 GetCurrentState 一致：T_tcp = T_flange * T_tool）
        Eigen::Matrix4d T_tool;
        {
            std::lock_guard<std::mutex> lock(tool_mtx_);
            T_tool = (tool >= 0 && static_cast<size_t>(tool) < tool_transforms_.size())
                ? tool_transforms_[static_cast<size_t>(tool)]
                : Eigen::Matrix4d::Identity();
        }

        // ── 目标 TCP 位姿（基座系）──
        DescPose dp;
        dp.tran.x = desc_command.target(0) * kM2Mm;  // m → mm
        dp.tran.y = desc_command.target(1) * kM2Mm;
        dp.tran.z = desc_command.target(2) * kM2Mm;
        Eigen::Matrix4d T_tcp = desc_pose_to_matrix(dp);  // 位置已就位，旋转待定
        if (desc_command.target.size() >= 6) {
            dp.rpy.rx = desc_command.target(3) * kRad2Deg;  // rad → deg
            dp.rpy.ry = desc_command.target(4) * kRad2Deg;
            dp.rpy.rz = desc_command.target(5) * kRad2Deg;
            T_tcp = desc_pose_to_matrix(dp);
        } else {
            // 仅位置：姿态保持当前 TCP 姿态（读取当前法兰位姿 + 工具变换得到当前 TCP 位姿，取其旋转）
            DescPose flange_now;
            int ret = robot.GetActualToolFlangePose(1, &flange_now);
            if (ret != 0) {
                fprintf(stderr,
                    "[RobotRealDriver] MoveL 读取当前法兰位姿失败 错误码=%d (%s)，无法保持姿态\n",
                    ret, rpc_error_str(ret));
                return ret;
            }
            Eigen::Matrix4d T_tcp_now = desc_pose_to_matrix(flange_now) * T_tool;
            T_tcp.block<3, 3>(0, 0) = T_tcp_now.block<3, 3>(0, 0);  // 保持旋转，位置不变
            Eigen::VectorXd p6;
            matrix_to_pose6(T_tcp, p6);
            dp.rpy.rx = p6(3) * kRad2Deg;
            dp.rpy.ry = p6(4) * kRad2Deg;
            dp.rpy.rz = p6(5) * kRad2Deg;
        }

        // ── 换算成法兰目标：T_flange = T_tcp * T_tool⁻¹（右乘逆，与 T_tcp = T_flange * T_tool 自洽）──
        Eigen::Matrix4d T_flange = T_tcp * T_tool.inverse();

        DescPose flange_dp;
        Eigen::VectorXd pose6;
        matrix_to_pose6(T_flange, pose6);  // [x,y,z,rx,ry,rz]，m/rad，与 desc_pose_to_matrix 同为固定轴 XYZ
        flange_dp.tran.x = pose6(0) * kM2Mm;
        flange_dp.tran.y = pose6(1) * kM2Mm;
        flange_dp.tran.z = pose6(2) * kM2Mm;
        flange_dp.rpy.rx = pose6(3) * kRad2Deg;
        flange_dp.rpy.ry = pose6(4) * kRad2Deg;
        flange_dp.rpy.rz = pose6(5) * kRad2Deg;

        // GetInverseKin 求出与法兰目标自洽的关节角（FK(jp) == flange_dp）
        JointPos jp;
        int ik_rtn = robot.GetInverseKin(0, &flange_dp, -1, &jp);
        if (ik_rtn != 0) {
            fprintf(stderr,
                "[RobotRealDriver] MoveL 逆运动学计算失败 错误码=%d (%s)，flange_pos=[%.1f %.1f %.1f %.1f %.1f %.1f]，tcp_pos=[%.1f %.1f %.1f %.1f %.1f %.1f]\n",
                ik_rtn, rpc_error_str(ik_rtn),
                flange_dp.tran.x, flange_dp.tran.y, flange_dp.tran.z,
                flange_dp.rpy.rx, flange_dp.rpy.ry, flange_dp.rpy.rz,
                dp.tran.x, dp.tran.y, dp.tran.z,
                dp.rpy.rx, dp.rpy.ry, dp.rpy.rz);
            return ik_rtn;
        }
        ExaxisPos epos(0, 0, 0, 0);
        DescPose offset(0, 0, 0, 0, 0, 0);

        // SDK 要求 desc_pos 与 joint_pos 的 FK 严格一致（与 MoveJ 同构：MoveJ 用 GetForwardKin→FK(jp) 作为 desc_pos）。
        // 因此 desc_pos 必须用「法兰目标」flange_dp（= FK(jp)），不能传 TCP 目标 dp（否则 desc_pos≠FK(jp)→74）。
        // desc_pos 已是法兰目标，故 tool 固定传 0（法兰坐标系）——工具坐标系只在本地换算，不传入控制器内部。
        int rtn = robot.MoveL(&jp, &flange_dp, 0, 0,
            static_cast<float>(desc_command.speed * 100.0),
            static_cast<float>(desc_command.acceleration * 100.0),
            100.0f, 0.0f, &epos, 0, 0, &offset);
        if (rtn != 0) {
            fprintf(stderr,
                "[RobotRealDriver] MoveL 失败 错误码=%d (%s)，desc_pos(法兰)=[%.1f %.1f %.1f %.1f %.1f %.1f] tcp目标=[%.1f %.1f %.1f %.1f %.1f %.1f] "
                "tool=%d vel=%.1f acc=%.1f\n",
                rtn, rpc_error_str(rtn),
                flange_dp.tran.x, flange_dp.tran.y, flange_dp.tran.z,
                flange_dp.rpy.rx, flange_dp.rpy.ry, flange_dp.rpy.rz,
                dp.tran.x, dp.tran.y, dp.tran.z,
                dp.rpy.rx, dp.rpy.ry, dp.rpy.rz,
                tool,
                static_cast<float>(desc_command.speed * 100.0),
                static_cast<float>(desc_command.acceleration * 100.0));
        }
        return rtn;
    }

    // 伺服运动启动
    int RobotRealDriver::ServoMoveStart()
    {
        if (!is_connected_.load()) return -1;
        int ret = robot.ServoMoveStart();
        is_servo_enabled_.store(ret == 0);  // 仅成功开启时置位
        return ret;
    }

    // 伺服运动结束
    int RobotRealDriver::ServoMoveEnd()
    {
        if (!is_connected_.load()) return -1;
        int ret = robot.ServoMoveEnd();
        is_servo_enabled_.store(false);  // 无论 SDK 结果如何，本地伺服状态必须退出
        return ret;
    }

    // 关节空间伺服运动
    int RobotRealDriver::ServoJ(MotionCommand& joint_command)
    {
        if (!is_connected_.load() || !is_servo_enabled_.load()) return -1;
        if (joint_command.target.size() < 6) return -1;
        JointPos jp;
        for (int i = 0; i < 6; ++i)
            jp.jPos[i] = joint_command.target(i) * kRad2Deg;  // rad → deg

        ExaxisPos epos(0, 0, 0, 0);
        // cmdT=0.001s：与上层伺服下发周期（1ms 控制循环）匹配
        return robot.ServoJ(&jp, &epos, 0.0f, 0.0f, 0.001f, 0.0f, 0.0f);
    }

    // 笛卡尔空间伺服运动
    int RobotRealDriver::ServoCart(MotionCommand& cart_command)
    {
        if (!is_connected_.load() || !is_servo_enabled_.load()) return -1;
        if (cart_command.target.size() < 6) return -1;
        DescPose dp;
        dp.tran.x = cart_command.target(0) * kM2Mm;  // m → mm
        dp.tran.y = cart_command.target(1) * kM2Mm;
        dp.tran.z = cart_command.target(2) * kM2Mm;
        dp.rpy.rx = cart_command.target(3) * kRad2Deg;  // rad → deg
        dp.rpy.ry = cart_command.target(4) * kRad2Deg;
        dp.rpy.rz = cart_command.target(5) * kRad2Deg;

        float pos_gain[6] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
        // mode=0：基坐标系绝对运动；cmdT=0.001s 指令周期
        return robot.ServoCart(0, &dp, pos_gain, 0.0f, 0.0f, 0.001f, 0.0f, 0.0f);
    }

    // 点动
    int RobotRealDriver::StartJOG(MotionCommand& jog_command)
    {
        if (!is_connected_.load()) return -1;
        // 平台点动类型 → SDK ref：
        //   JOG_0(4) → 0 关节点动，JOG_1(5) → 2 基坐标系点动，JOG_2(6) → 4 工具坐标系点动
        uint8_t ref = static_cast<uint8_t>((jog_command.type - RusRobotDriver::MOTION_TYPE_JOG_0) * 2);
        last_jog_ref_ = ref;  // 记录本次点动参考系，供 StopJOGDecel 使用

        // ── max_dis 单位换算：驱动配置 m/rad → SDK °/mm（单位口径统一收束在驱动内部）──
        // 值来源：driver_params.yaml 的 jog_max_dis_joint / jog_max_dis_trans / jog_max_dis_rot
        //        （由 driver_node::jog_limit_for 注入；前端 start_jog 传入的 max_dis 不参与控制）。
        // 配置单位：rad（关节点动 JOG_0 与笛卡尔旋转轴 nb=4~6）、m（笛卡尔平移轴 nb=1~3），0 = 不限制；
        // SDK 单位：°（关节/旋转）、mm（平移），且必须 > 0（传 0 会被视为「位移上限 0」→ 机械臂不动）。
        const bool rot_like = (jog_command.type == RusRobotDriver::MOTION_TYPE_JOG_0) ||
                              (jog_command.jog_axis > 3);
        float max_dis;
        if (jog_command.jog_max_dis > 0.0) {
            max_dis = static_cast<float>(jog_command.jog_max_dis * (rot_like ? kRad2Deg : kM2Mm));
        } else {
            // 平台语义 0 = 不限制：SDK 无「不限制」表示，用足够大的上限近似
            max_dis = kJogMaxDisUnlimited;
            fprintf(stderr,
                "[RobotRealDriver] 提示: 点动上限配置为 %.3f（0 = 不限制），已按 SDK 上限 %.0f%s 下发\n",
                jog_command.jog_max_dis, max_dis, rot_like ? "°" : "mm");
        }

        float vel = static_cast<float>(jog_command.speed * 100.0);
        float acc = static_cast<float>(jog_command.acceleration * 100.0);

        int rtn = robot.StartJOG(ref, jog_command.jog_axis, jog_command.jog_dir, vel, acc, max_dis);
        if (rtn != 0) {
            fprintf(stderr,
                "[RobotRealDriver] StartJOG(ref=%u, nb=%u, dir=%u, vel=%.1f, acc=%.1f, max_dis=%.1f(%s)) 失败 错误码=%d\n",
                ref, jog_command.jog_axis, jog_command.jog_dir, vel, acc, max_dis,
                rot_like ? "°" : "mm", rtn);
        }
        return rtn;
    }

    // 减速停止点动
    int RobotRealDriver::StopJOGDecel()
    {
        if (!is_connected_.load()) return -1;
        // SDK 停止 ref = 启动 ref + 1：关节 1 / 基坐标系 3 / 工具坐标系 5
        uint8_t stop_ref = static_cast<uint8_t>(last_jog_ref_ + 1);
        int rtn = robot.StopJOG(stop_ref);
        if (rtn != 0)
            fprintf(stderr, "[RobotRealDriver] StopJOG(%u) 失败 错误码=%d\n", stop_ref, rtn);
        return rtn;
    }

    // 直接停止点动
    int RobotRealDriver::StopJOGImmediate()
    {
        if (!is_connected_.load()) return -1;
        int rtn = robot.ImmStopJOG();
        if (rtn != 0)
            fprintf(stderr, "[RobotRealDriver] ImmStopJOG 失败 错误码=%d\n", rtn);
        return rtn;
    }

    // 终止运动
    int RobotRealDriver::StopMotion()
    {
        if (!is_connected_.load()) return -1;
        is_servo_enabled_.store(false);  // 与仿真驱动一致：急停同时退出伺服模式
        return robot.StopMotion();
    }

    // 恢复运动
    int RobotRealDriver::ResumeMotion()
    {
        if (!is_connected_.load()) return -1;
        return robot.ResumeMotion();
    }

    // 暂停运动
    int RobotRealDriver::PauseMotion()
    {
        if (!is_connected_.load()) return -1;
        return robot.PauseMotion();
    }

    // 急停后重置：清错误 + 按参数重新使能（参数外部可配，便于适配不同 SDK 版本/产线）
    int RobotRealDriver::ResetMotion(const RusRobotDriver::ResetCmd& cmd)
    {
        // 无论软/硬复位，本地伺服状态都退出（与仿真驱动一致）
        is_servo_enabled_.store(false);

        if (cmd.mode < 1)
            return 0;  // 仅软复位：真实驱动无本地轨迹层状态，无需额外操作

        // 完整复位：先清除所有错误，再按参数决定是否重新上使能
        int ret = robot.ResetAllError();
        if (ret != 0) return -1;
        ret = robot.RobotEnable(cmd.enable);
        if (ret != 0) return -1;
        return 0;
    }

    bool RobotRealDriver::IsMotionDone() const
    {
        uint8_t state = 0;
        if (robot.GetRobotMotionDone(&state) != 0)
            return false;  // 查询失败视为运动未完成（保守处理）
        return state == 1;  // SDK: 0-未完成，1-完成
    }

    // ============================================================
    //  工具坐标系相关接口（六点标定计算完全在 SDK/控制器内部完成）
    // ============================================================

    // 六点法标定：记录第 point_num 个工具参考点（1~6）
    // 调用前需已移动机械臂使 TCP 对准同一标定尖点，SDK 采集当前位姿。
    int RobotRealDriver::SetToolCalibPoint(int point_num)
    {
        if (!is_connected_.load()) return -1;
        if (point_num < 1 || point_num > 6) {
            fprintf(stderr, "[RobotRealDriver] SetToolCalibPoint(%d) 参数越界（范围 1~6）\n", point_num);
            return -1;
        }
        int rtn = robot.SetToolPoint(point_num);
        if (rtn != 0)
            fprintf(stderr, "[RobotRealDriver] SetToolPoint(%d) 失败 错误码=%d\n", point_num, rtn);
        return rtn;
    }

    // 六点法标定：计算工具坐标系（标定计算由 SDK ComputeTool 在控制器内部完成，
    // 此处仅做单位换算 mm/deg → m/rad 并透传结果）
    int RobotRealDriver::ComputeToolCalib(std::vector<double>& tcp_pose)
    {
        if (!is_connected_.load()) return -1;
        DescPose pose;
        int rtn = robot.ComputeTool(&pose);
        if (rtn != 0) {
            fprintf(stderr, "[RobotRealDriver] ComputeTool 失败 错误码=%d\n", rtn);
            return rtn;
        }
        tcp_pose = {
            pose.tran.x * kMm2M, pose.tran.y * kMm2M, pose.tran.z * kMm2M,
            pose.rpy.rx * kDeg2Rad, pose.rpy.ry * kDeg2Rad, pose.rpy.rz * kDeg2Rad
        };
        return 0;
    }

    // 设置工具坐标系（TCP 相对法兰位姿）并立即生效
    int RobotRealDriver::SetToolCoord(int id, const std::vector<double>& coord)
    {
        if (!is_connected_.load()) return -1;
        if (id < 0 || id > 14 || coord.size() < 6) {
            fprintf(stderr, "[RobotRealDriver] SetToolCoord(id=%d, coord_size=%zu) 参数非法\n",
                    id, coord.size());
            return -1;
        }
        DescPose pose(
            coord[0] * kM2Mm, coord[1] * kM2Mm, coord[2] * kM2Mm,
            coord[3] * kRad2Deg, coord[4] * kRad2Deg, coord[5] * kRad2Deg);
        // 本地维护工具变换表（GetCurrentState 计算 tool_pose 用）
        {
            std::lock_guard<std::mutex> lock(tool_mtx_);
            if (tool_transforms_.size() <= static_cast<size_t>(id))
                tool_transforms_.resize(static_cast<size_t>(id) + 1, Eigen::Matrix4d::Identity());
            Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
            T.block<3, 3>(0, 0) =
                (Eigen::AngleAxisd(coord[5], Eigen::Vector3d::UnitZ())
               * Eigen::AngleAxisd(coord[4], Eigen::Vector3d::UnitY())
               * Eigen::AngleAxisd(coord[3], Eigen::Vector3d::UnitX())).toRotationMatrix();
            T.block<3, 1>(0, 3) << coord[0], coord[1], coord[2];
            tool_transforms_[static_cast<size_t>(id)] = T;
        }
        // type=0 工具坐标系, install=0 机器人末端, toolID=0, loadNum=0
        int rtn = robot.SetToolCoord(id, &pose, 0, 0, 0, 0);
        if (rtn != 0)
            fprintf(stderr,
                "[RobotRealDriver] SetToolCoord(id=%d) 控制器返回错误码=%d (%s)；请确认机器人已上使能、处于自动模式且空闲\n",
                id, rtn, rpc_error_str(rtn));
        return rtn;
    }
    // 切换当前工具坐标系索引（运动参考系随之切换：0=法兰，N=工具 N）
    int RobotRealDriver::SetToolIndex(int id)
    {
        if (id < 0 || id > 14) {
            fprintf(stderr, "[RobotRealDriver] SetToolIndex(%d) 参数越界（范围 0~14）", id);
            return -1;
        }
        tool_index_.store(id);
        return 0;
    }

}  // namespace RusRealRobotDriver
