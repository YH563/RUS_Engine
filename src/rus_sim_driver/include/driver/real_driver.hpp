#pragma once

#include <atomic>
#include <mutex>
#include <vector>

#include <Eigen/Dense>

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
        int Connect(const std::string& ip) override;

        // 最近一次 RPC 调用的错误码（0=成功，-2=网络通讯异常，-3=XMLRPC 通讯失败等）
        int LastRpcError() const { return last_rpc_error_; }

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

        // 控制机器人手自动模式切换，mode 0-自动模式，1-手动模式
        int SetMode(int mode) override{return robot.Mode(mode);}

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
        // jog_command.jog_max_dis：值来自 driver_params.yaml（由 driver_node::jog_limit_for 注入），
        // 指令层单位 rad（关节/笛卡尔旋转轴）、m（笛卡尔平移轴），0 = 不限制；
        // 内部按 SDK 语义换算为 °（关节/旋转轴）/ mm（平移轴）。
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

        // === 工具坐标系相关接口 ===

        // 六点法标定：记录第 point_num 个工具参考点（1~6，TCP 需对准同一尖点）
        int SetToolCalibPoint(int point_num) override;

        // 六点法标定：计算工具坐标系（标定计算由 SDK ComputeTool 在控制器内部完成）
        int ComputeToolCalib(std::vector<double>& tcp_pose) override;

        // 设置工具坐标系（TCP 相对法兰位姿）并生效
        int SetToolCoord(int id, const std::vector<double>& coord) override;

        // 切换当前工具坐标系索引（运动参考系随之切换）
        int SetToolIndex(int id) override;

    private:
        // 私有成员变量
        // robot 声明为 mutable：IsMotionDone 等 const 接口内部需要
        // 调用非 const 的 SDK 查询方法（GetRobotMotionDone）。
        mutable FRRobot robot;  // 机器人，用于获取真实机械臂状态
        std::atomic<bool> is_connected_{false};  // 是否连接
        std::atomic<bool> is_servo_enabled_{false};  // 伺服模式是否已开启（ServoMoveStart/End 维护）
        std::atomic<int> last_rpc_error_{0};  // 最近一次 RPC 错误码（供上层诊断连接失败原因）
        uint8_t last_jog_ref_{0};  // 最近一次点动的 SDK ref（StopJOGDecel 用），0=关节点动
        std::atomic<int> tool_index_{0};  // 当前工具坐标系索引（MoveJ/MoveL 的 SDK tool 参数）
        // 本地工具变换表（TCP 相对法兰；SetToolCoord 维护，GetCurrentState 用它计算 tool_pose，
        // 不依赖控制器 GetActualTCPPose——控制器无“当前工具号”设置接口，始终按工具 0 计算）
        mutable std::mutex tool_mtx_;
        std::vector<Eigen::Matrix4d> tool_transforms_{Eigen::Matrix4d::Identity()};
    };
}