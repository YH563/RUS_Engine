#pragma once

#include <atomic>

#include "robot.h"
#include "robot_driver.hpp"

namespace RusRealRobotDriver {
    using RusRobotDriver::RobotState;
    using RusRobotDriver::MotionCommand;

    // 真实驱动，对实际SDK的进一步封装
    class RobotRealDriver : public RusRobotDriver::IRobotDriver
    {
    public:
        RobotRealDriver(const std::string& ip);
        ~RobotRealDriver() override = default;

        // 连接机械臂
        int Connect(const std::string& ip) override{
            int rtn = robot.RPC(ip.c_str());
            is_connected_.store(rtn == 0);
            return rtn;
        }

        // 断开与机械臂的连接
        int Disconnect() override{ 
            int rtn = robot.CloseRPC(); 
            is_connected_.store(rtn == 0);
            return rtn; 
        }

        // 检查是否连接
        // 说明：SDK 的 IsSockError() 为私有接口，无法直接调用；
        // 连接状态由 Connect/Disconnect 维护的 is_connected_ 标志位跟踪。
        bool IsConnected() const override{return is_connected_.load();}

        // 查询是否在示教模式，state 0-非拖动示教模式，1-拖动示教模式
        int IsInDragTeach(uint8_t& state) override{return robot.IsInDragTeach(&state);}

        // 控制机器人上使能或下使能，state 0-下使能，1-上使能
        int RobotEnable(uint8_t state) override{return robot.RobotEnable(state);}

        /**
         * @brief 获取当前的机械臂状态
         * 
         * @param flag 0-阻塞，1-非阻塞
         * @param robot_state 保存机械臂状态
         * @return int 
         */
        int GetCurrentState(uint8_t flag, RobotState& robot_state) override;

        // 关节空间运动
        int MoveJ(MotionCommand& joint_command) override;

        // 笛卡尔空间直线运动
        int MoveL(MotionCommand& desc_command) override;

        // 伺服运动启动
        int ServoMoveStart() override;

        // 伺服运动结束
        int ServoMoveEnd() override;

        // 关节空间伺服运动
        int ServoJ(MotionCommand& joint_command) override;

        // 笛卡尔空间伺服运动
        int ServoCart(MotionCommand& cart_command) override;

        // 点动
        int StartJOG(MotionCommand& jog_command) override;

        // 减速停止点动
        int StopJOGDecel() override;

        // 直接停止点动
        int StopJOGImmediate() override;

        // 终止运动
        int StopMotion() override;

        // 恢复运动
        int ResumeMotion() override;

        // 暂停运动
        int PauseMotion() override;

        // 急停后重置（清错误 + 按参数重新使能）
        int ResetMotion(const RusRobotDriver::ResetCmd& cmd) override;

        /**
         * @brief 查询运动是否已完成
         */
        bool IsMotionDone() const override;
    
    private:

        // 私有成员变量
        // robot 声明为 mutable：IsMotionDone 等 const 接口内部需要
        // 调用非 const 的 SDK 查询方法（GetRobotMotionDone）。
        mutable FRRobot robot;  // 机器人，用于获取真实机械臂状态
        std::atomic<bool> is_connected_{false};  // 是否连接
        std::atomic<bool> is_servo_enabled_{false};  // 伺服模式是否已开启（ServoMoveStart/End 维护）
        uint8_t last_jog_ref_{0};  // 最近一次点动的 SDK ref（StopJOGDecel 用），0=关节点动
    };
}