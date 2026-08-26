#pragma once

#include <Eigen/Dense>
#include <cstdint>
#include <string>
#include <string_view>
#include <variant>
#include <vector>

#include "rus_sim_utils/robot_state.hpp"

namespace RusRobotDriver {
    using Eigen::VectorXd;
    using RusUtils::ControlTarget;  // 期望控制目标（通用类型）
    using RusUtils::RobotState;     // 机器人状态（通用类型）

    //  运动指令类型常量
    constexpr uint8_t MOTION_TYPE_JOINT  = 0;  // 关节空间运动 (MoveJ)
    constexpr uint8_t MOTION_TYPE_CART   = 1;  // 笛卡尔空间直线 (MoveL)
    constexpr uint8_t MOTION_TYPE_SERVOJ = 2;  // 关节伺服
    constexpr uint8_t MOTION_TYPE_SERVOC = 3;  // 笛卡尔空间伺服
    constexpr uint8_t MOTION_TYPE_JOG_0  = 4;  // 关节点动
    constexpr uint8_t MOTION_TYPE_JOG_1  = 5;  // 基坐标系点动
    constexpr uint8_t MOTION_TYPE_JOG_2  = 6;  // 工具坐标系点动

    //  运动指令
    struct MotionCommand{
        uint8_t type;     // MOTION_TYPE_JOINT / MOTION_TYPE_CART / MOTION_TYPE_SERVOJ / MOTION_TYPE_SERVOC / MOTION_TYPE_JOG_*
        VectorXd target;  // 目标，关节角或者笛卡尔位姿，单位 rad 或 m
        double speed = 0.5;  // 速度比例（JOG 时对应 vel 百分比/100）
        double acceleration = 0.5;  // 加速度比例（JOG 时对应 acc 百分比/100）

        // ---- 点动参数（仅 MOTION_TYPE_JOG_* 时使用） ----
        uint8_t jog_axis = 1;     // 轴号（nb）：1~6
        uint8_t jog_dir = 1;      // 方向（dir）：0-负方向，1-正方向
        double jog_max_dis = 0.0; // 单次最大位移（max_dis），单位 ° 或 mm，0 表示无限制
    };

    //  非运动指令参数

    // ── 驱动控制 ──
    struct ConnectCmd       {};
    struct DisconnectCmd    {};
    struct IsConnectedCmd   {};
    struct IsInDragTeachCmd {};
    struct RobotEnableCmd   { uint8_t state = 1; };
    struct GetStateCmd      { uint8_t flag = 1; };
    struct IsMotionDoneCmd  {};

    // ── 伺服模式 ──
    struct ServoStartCmd    {};
    struct ServoEndCmd      {};

    // ── 运动控制 ──
    struct StopCmd          {};
    struct PauseCmd         {};
    struct ResumeCmd        {};
    struct ResetCmd {
        uint8_t mode = 1;    // 复位模式：0=仅软复位（清运动状态），1=完整复位（清错误+重新使能）
        uint8_t enable = 1;  // 完整复位后是否重新上使能（mode=1 时有效）
    };
    struct StopJOGDecelCmd  {};
    struct StopJOGImmediateCmd {};

    // ── 文件执行 ──
    struct RunFileCmd       { std::string path; };

    // ── 工具坐标系 / 标定 ──
    struct SetToolCalibPointCmd { int point_num = 1; };       // 记录第 N 个六点法标定点（1~6）
    struct ComputeToolCalibCmd  {};                           // 计算工具坐标系（结果写入 result）
    struct SetToolCoordCmd {                                   // 设置工具坐标系并生效
        int id = 0;                                           // 坐标系编号 [0~14]
        std::vector<double> coord;                            // 工具相对法兰位姿 [x,y,z,rx,ry,rz]（m/rad）
    };

    // ── 仿真控制（仅 Sim 驱动） ──
    struct SetTimeSpeedCmd    { double speed; };
    struct GetTimeSpeedCmd    {};
    struct GetSimTimeCmd      {};
    struct StepOnceCmd        {};
    struct GetFrameRateCmd    {};

    //  统一指令承载
    using RobotCommand = std::variant<
        MotionCommand,

        // 驱动控制
        ConnectCmd,        DisconnectCmd,
        IsConnectedCmd,    IsInDragTeachCmd,
        RobotEnableCmd,    GetStateCmd,
        IsMotionDoneCmd,

        // 伺服模式
        ServoStartCmd,     ServoEndCmd,

        // 运动控制
        StopCmd,           PauseCmd,          ResumeCmd,         ResetCmd,
        StopJOGDecelCmd,   StopJOGImmediateCmd,

        // 文件执行
        RunFileCmd,

        // 工具坐标系 / 标定
        SetToolCalibPointCmd,  ComputeToolCalibCmd,  SetToolCoordCmd,

        // 仿真控制
        SetTimeSpeedCmd,   GetTimeSpeedCmd,
        GetSimTimeCmd,     StepOnceCmd,
        GetFrameRateCmd
    >;

    //  std::visit 辅助模板
    template<class... Ts> struct Overloaded : Ts... { using Ts::operator()...; };
    template<class... Ts> Overloaded(Ts...) -> Overloaded<Ts...>;

    /**
     * @brief 将指令名和参数转换为 RobotCommand
     *
     * @param name 指令名（如 "movej", "connect"）
     * @param args float64[] 参数数组
     * @return RobotCommand  解析结果
     *
     * 参数格式说明（运动指令）：
     *   movej    [q1..q6, speed?, acc?]
     *   movel    [x,y,z,rx,ry,rz, speed?, acc?]
     *   servoj   [q1..q6]
     *   servo_cart [x,y,z,rx,ry,rz]
     *   start_jog [ref, nb, dir, vel, acc, max_dis]
     *   reset     [mode, enable]
     */
    RobotCommand ParseCommand(std::string_view name, const std::vector<double>& args);

    // 控制目标 / 机器人状态为通用类型（见 rus_sim_utils/robot_state.hpp）
}