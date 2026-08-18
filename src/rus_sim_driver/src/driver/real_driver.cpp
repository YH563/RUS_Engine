#include "driver/real_driver.hpp"
#include "robot.h"
#include "robot_types.h"

#include <cmath>

namespace RusRealRobotDriver {

    namespace {
        // 单位换算常量
        constexpr double kDeg2Rad = M_PI / 180.0;   // 度 → 弧度
        constexpr double kRad2Deg = 180.0 / M_PI;   // 弧度 → 度
        constexpr double kMm2M    = 1.0 / 1000.0;   // 毫米 → 米
        constexpr double kM2Mm    = 1000.0;         // 米 → 毫米

        // SDK 状态（fairino）→ 通用 RobotState；角度 度 → 弧度，位置 mm → m
        RusRobotDriver::RobotState convert_sdk_state(
            JointPos& jPos, float speed[6], float acc[6],
            float torques[6], DescPose& flange, float time_ms)
        {
            RusRobotDriver::RobotState s;

            // 法兰位姿 XYZABC：位置 [mm→m]，姿态 [deg→rad]
            s.flange_pos.resize(6);
            s.flange_pos << flange.tran.x * kMm2M, flange.tran.y * kMm2M, flange.tran.z * kMm2M,
                flange.rpy.rx * kDeg2Rad, flange.rpy.ry * kDeg2Rad,
                flange.rpy.rz * kDeg2Rad;

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
        // Connect 内部会更新 is_connected_，连接失败时调用方可通过 IsConnected() 检查
        Connect(ip);
        // 开启 SDK 自动重连：断线后每 500ms 尝试重连，最长 30s
        robot.SetReConnectParam(true, 30000, 500);
    }

    int RobotRealDriver::GetCurrentState(uint8_t flag, RobotState& robot_state)
    {
        if (!is_connected_.load()) return -1;  // 未连接时直接返回，避免 SDK 报错/脏数据
        JointPos jPos;
        float speed[6];
        float acc[6];
        float torques[6];
        DescPose flange;
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
        ret = robot.GetSystemClock(&time_ms);
        if (ret != 0) return ret;

        robot_state = convert_sdk_state(jPos, speed, acc, torques, flange, time_ms);
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

        // vel/acc：比例 [0~1] → 百分比 [0~100]；ovl=100 不额外缩放；
        // blendT=0 非阻塞（立即返回，运动由控制器队列执行，完成状态用 IsMotionDone 轮询）
        return robot.MoveJ(&jp, nullptr, 0, 0,
            static_cast<float>(joint_command.speed * 100.0),
            static_cast<float>(joint_command.acceleration * 100.0),
            100.0f, nullptr, 0.0f, 0, nullptr);
    }

    // 笛卡尔空间直线运动
    int RobotRealDriver::MoveL(MotionCommand& desc_command)
    {
        if (!is_connected_.load()) return -1;
        if (desc_command.target.size() < 6) return -1;
        DescPose dp;
        dp.tran.x = desc_command.target(0) * kM2Mm;  // m → mm
        dp.tran.y = desc_command.target(1) * kM2Mm;
        dp.tran.z = desc_command.target(2) * kM2Mm;
        dp.rpy.rx = desc_command.target(3) * kRad2Deg;  // rad → deg
        dp.rpy.ry = desc_command.target(4) * kRad2Deg;
        dp.rpy.rz = desc_command.target(5) * kRad2Deg;

        // blendR=0 非阻塞，与 MoveJ 一致
        return robot.MoveL(nullptr, &dp, 0, 0,
            static_cast<float>(desc_command.speed * 100.0),
            static_cast<float>(desc_command.acceleration * 100.0),
            100.0f, 0.0f, nullptr, 0, 0, nullptr);
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

        return robot.StartJOG(
            ref,
            jog_command.jog_axis,
            jog_command.jog_dir,
            static_cast<float>(jog_command.speed * 100.0),
            static_cast<float>(jog_command.acceleration * 100.0),
            static_cast<float>(jog_command.jog_max_dis)
        );
    }

    // 减速停止点动
    int RobotRealDriver::StopJOGDecel()
    {
        if (!is_connected_.load()) return -1;
        // SDK 停止 ref = 启动 ref + 1：关节 1 / 基坐标系 3 / 工具坐标系 5
        return robot.StopJOG(static_cast<uint8_t>(last_jog_ref_ + 1));
    }

    // 直接停止点动
    int RobotRealDriver::StopJOGImmediate()
    {
        if (!is_connected_.load()) return -1;
        return robot.ImmStopJOG();
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
}