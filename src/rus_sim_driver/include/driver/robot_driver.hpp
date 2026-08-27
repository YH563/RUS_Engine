#pragma once

#include <array>
#include <cmath>
#include <memory>
#include <string>
#include <Eigen/Dense>
#include <vector>

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

        // 控制机器人手自动模式切换，mode 0-自动模式，1-手动模式
        // （部分配置类指令如 SetToolCoord 要求自动模式）
        virtual int SetMode(int mode) = 0;

        // 获取当前的机械臂状态，flag 0-阻塞，1-非阻塞
        virtual int GetCurrentState(uint8_t flag, RobotState& robot_state) = 0;

        //

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

        // 急停后重置（清错误 + 按参数重新使能）
        virtual int ResetMotion(const ResetCmd& cmd) = 0;

        // 查询机械臂运动是否已完成
        virtual bool IsMotionDone() const = 0;

        // === 工具坐标系相关接口 ===

        /**
         * @brief 六点法标定：记录第 point_num 个工具参考点（范围 1~6）。
         *
         * 调用前需已移动机械臂使 TCP 对准同一标定尖点，SDK 采集当前位姿。
         * 集齐 6 点后调用 ComputeToolCalib 完成计算。
         *
         * @param point_num 点编号 [1~6]
         * @return 错误码
         */
        virtual int SetToolCalibPoint(int point_num) = 0;

        /**
         * @brief 六点法标定：计算工具坐标系。
         *
         * 标定计算完全在驱动内部（真实驱动调用 SDK ComputeTool 完成六点拟合）。
         *
         * @param[out] tcp_pose 工具中心点相对末端法兰位姿 [x,y,z,rx,ry,rz]（m/rad）
         * @return 错误码
         */
        virtual int ComputeToolCalib(std::vector<double>& tcp_pose) = 0;

        /**
         * @brief 设置工具坐标系（工具中心点相对末端法兰位姿）并立即生效。
         *
         * @param id    坐标系编号 [0~14]
         * @param coord 工具相对法兰位姿 [x,y,z,rx,ry,rz]（m/rad）
         * @return 错误码
         */
        virtual int SetToolCoord(int id, const std::vector<double>& coord) = 0;

        /**
         * @brief 切换当前工具坐标系索引（运动参考系随之切换：0=法兰坐标系，N=工具坐标系 N）。
         *
         * 真实驱动：后续 MoveJ/MoveL/ServoCart 的 SDK tool 参数使用该索引；
         * 仿真驱动：运动学工具变换与状态 tool_pose 同步更新。
         *
         * @param id 工具坐标系编号 [0~14]
         * @return 错误码
         */
        virtual int SetToolIndex(int id) = 0;

    };

    // 采用工厂模式
    class DriverFactory{
    public:
        enum Type{Sim, Real};
        static std::unique_ptr<IRobotDriver> Create(Type type, const std::string& ip);
    };
}