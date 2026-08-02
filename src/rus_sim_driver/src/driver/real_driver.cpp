#include "driver/real_driver.hpp"
#include "robot.h"
#include "robot_types.h"

namespace RusRealRobotDriver {
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
        robot_state.Convert(jPos, speed, acc, torques, flange, time);
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

    bool RobotRealDriver::IsMotionDone() const
    {
        return true;  // TODO: 查询 SDK 运动状态
    }
}