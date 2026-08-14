#include "driver/real_driver.hpp"
#include "robot.h"
#include "robot_types.h"

#include <cmath>

namespace RusRealRobotDriver {

    namespace {
        // SDK 状态（fairino）→ 通用 RobotState；角度 度 → 弧度
        RusRobotDriver::RobotState convert_sdk_state(
            JointPos& jPos, float speed[6], float acc[6],
            float torques[6], DescPose& flange, float time)
        {
            RusRobotDriver::RobotState s;

            // 法兰位姿 XYZABC [m/rad]，rx/ry/rz 度 → 弧度
            s.flange_pos.resize(6);
            s.flange_pos << flange.tran.x, flange.tran.y, flange.tran.z,
                flange.rpy.rx / 180.0 * M_PI, flange.rpy.ry / 180.0 * M_PI,
                flange.rpy.rz / 180.0 * M_PI;

            // 关节位置 / 速度 / 加速度，度 → 弧度
            s.joint_pos = Eigen::Map<Eigen::VectorXd>(jPos.jPos, 6) / 180.0 * M_PI;
            s.joint_vel = Eigen::Map<Eigen::VectorXf>(speed, 6).cast<double>() / 180.0 * M_PI;
            s.joint_acc = Eigen::Map<Eigen::VectorXf>(acc, 6).cast<double>() / 180.0 * M_PI;

            // 关节力矩 [Nm]
            s.effort = Eigen::Map<Eigen::VectorXf>(torques, 6).cast<double>();

            s.timestamp = static_cast<double>(time);
            return s;
        }
    }  // namespace

    RobotRealDriver::RobotRealDriver(const std::string& ip)
    {
        robot = FRRobot();
        int rtn = Connect(ip);
        robot.SetReConnectParam(true, 30000, 500);

    }

    int RobotRealDriver::GetCurrentState(uint8_t flag, RobotState& robot_state)
    {
        JointPos jPos;
        float speed[6];
        float acc[6];
        float torques[6];
        DescPose flange;
        float time;
        robot.GetActualJointPosDegree(flag, &jPos);
        robot.GetActualJointSpeedsDegree(flag, speed);
        robot.GetActualJointAccDegree(flag, acc);
        robot.GetJointTorques(flag, torques);
        robot.GetActualToolFlangePose(flag, &flange);
        robot.GetSystemClock(&time);
        robot_state = convert_sdk_state(jPos, speed, acc, torques, flange, time);
        return 0;
    }

    // 关节空间运动
    int RobotRealDriver::MoveJ(MotionCommand& joint_command)
    {
        return -1;  // TODO: 调用 SDK MoveJ
    }

    // 笛卡尔空间直线运动
    int RobotRealDriver::MoveL(MotionCommand& desc_command)
    {
        return -1;  // TODO: 调用 SDK MoveL
    }

    // 伺服运动启动
    int RobotRealDriver::ServoMoveStart()
    {
        return -1;  // TODO: 调用 SDK ServoMoveStart
    }

    // 伺服运动结束
    int RobotRealDriver::ServoMoveEnd()
    {
        return -1;  // TODO: 调用 SDK ServoMoveEnd
    }

    // 关节空间伺服运动
    int RobotRealDriver::ServoJ(MotionCommand& joint_command)
    {
        return -1;  // TODO: 调用 SDK ServoJ
    }

    // 笛卡尔空间伺服运动
    int RobotRealDriver::ServoCart(MotionCommand& cart_command)
    {
        return -1;  // TODO: 调用 SDK ServoCart
    }

    // 点动
    int RobotRealDriver::StartJOG(MotionCommand& jog_command)
    {
        return robot.StartJOG(
            jog_command.type - 4,            // ref: JOG_0→0, JOG_1→2→... 需转换
            jog_command.jog_axis,
            jog_command.jog_dir,
            static_cast<float>(jog_command.speed * 100.0),
            static_cast<float>(jog_command.acceleration * 100.0),
            jog_command.jog_max_dis
        );
    }

    // 减速停止点动
    int RobotRealDriver::StopJOGDecel()
    {
        return -1;  // TODO: 调用 SDK 减速停止
    }

    // 直接停止点动
    int RobotRealDriver::StopJOGImmediate()
    {
        return -1;  // TODO: 调用 SDK 急停
    }

    // 终止运动
    int RobotRealDriver::StopMotion()
    {
        return robot.StopMotion();
    }

    // 恢复运动
    int RobotRealDriver::ResumeMotion()
    {
        return -1;  // TODO
    }

    // 暂停运动
    int RobotRealDriver::PauseMotion()
    {
        return -1;  // TODO
    }

    // 急停后重置：清错误 + 按参数重新使能（参数外部可配，便于适配不同 SDK 版本/产线）
    int RobotRealDriver::ResetMotion(const RusRobotDriver::ResetCmd& cmd)
    {
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
        return true;  // TODO: 查询 SDK 运动状态
    }
}