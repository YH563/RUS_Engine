#pragma once

#include <array>
#include <cmath>
#include <memory>
#include <string>
#include <Eigen/Dense>

#include "robot.h"
#include "robot_types.h"
#include "robot_error.h"
#include "trajectory/trajectory_executor.hpp"

namespace RusRobotDriver {
    // 虚基类接口，工厂模式
    class IRobotDriver
    {
    public:
        virtual ~IRobotDriver() = default;
        
        // 连接机械臂
        virtual int Connect(const std::string& ip) = 0;

        // 断开与机械臂的连接
        virtual int Disconnect() = 0;

        // 检查是否连接
        virtual bool IsConnected() const = 0;

        // 查询是否在示教模式，state 0-非拖动示教模式，1-拖动示教模式
        virtual int IsInDragTeach(uint8_t& state) = 0;

        // 控制机器人上使能或下使能，state 0-下使能，1-上使能
        virtual int RobotEnable(uint8_t state) = 0;

        // 获取当前的机械臂状态，flag 0-阻塞，1-非阻塞
        virtual int GetCurrentState(uint8_t flag, RobotState& robot_state) = 0;

        // 关节空间运动
        virtual int MoveJ(MotionCommand& joint_command) = 0;

        // 笛卡尔空间直线运动
        virtual int MoveL(MotionCommand& desc_command) = 0;

        // 伺服运动启动
        virtual int ServoMoveStart() = 0;

        // 伺服运动结束
        virtual int ServoMoveEnd() = 0;

        // 关节空间伺服运动
        virtual int ServoJ(MotionCommand& joint_command) = 0;

        // 笛卡尔空间伺服运动
        virtual int ServoCart(MotionCommand& joint_command) = 0;

        // 点动
        virtual int StartJOG(MotionCommand& jog_command) = 0;

        // 减速停止点动
        virtual int StopJOGDecel() = 0;

        // 直接停止点动
        virtual int StopJOGImmediate() = 0;

        // 终止运动
        virtual int StopMotion() = 0;

        // 恢复运动
        virtual int ResumeMotion() = 0;

        // 暂停运动
        virtual int PauseMotion() = 0;

        // 查询机械臂运动是否已完成
        virtual bool IsMotionDone() const = 0;
    };

    // 采用工厂模式
    class DriverFactory{
    public:
        enum Type{Sim, Real};
        static std::unique_ptr<IRobotDriver> Create(Type type, const std::string& ip);
    };
}