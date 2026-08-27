#pragma once

#include <memory>
#include <mutex>
#include <condition_variable>
#include <thread>
#include <atomic>
#include <chrono>
#include <vector>

#include <mujoco/mujoco.h>
#include <urdf/model.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "robot_driver.hpp"
#include "EAIK/EAIK.h"
#include "trajectory/trajectory_executor.hpp"
#include "controller/controller.hpp"

namespace RusSimRobotDriver {
    using RusRobotDriver::RobotState;
    using RusRobotDriver::MotionCommand;
    using Eigen::VectorXd;

    // MuJoCo 智能指针
    using MjModelPtr = std::unique_ptr<mjModel, decltype(&mj_deleteModel)>;
    using MjDataPtr  = std::unique_ptr<mjData,  decltype(&mj_deleteData)>;

    /**
     * @brief 基于 MuJoCo 仿真引擎的机器人模拟驱动
     *
     * 控制循环每帧执行：
     *   trajectory_executor_->Step() → ControlTarget
     *   controller_->ComputeTorque() → 关节力矩 → mj_data_->ctrl
     *   mj_step() → 物理推进
     *   sync_state() → 状态回读
     */
    class RobotSimDriver : public RusRobotDriver::IRobotDriver {
    public:
        /**
         * @brief 构造仿真驱动：加载模型、创建轨迹调度器与控制器、启动控制线程
         * 
         * @param ip 机器人 IP（仿真中未使用）
         */
        RobotSimDriver(const std::string& ip);

        /**
         * @brief 析构：停止控制线程
         */
        ~RobotSimDriver() override;

        // === IRobotDriver 接口 ===

        /**
         * @brief 连接到仿真机器人
         */
        int Connect(const std::string& ip) override { is_connected_.store(true); return 0; }

        /**
         * @brief 断开连接
         */
        int Disconnect() override { is_connected_.store(false); return 0; }

        /**
         * @brief 检查是否已连接
         */
        bool IsConnected() const override { return is_connected_; }

        /**
         * @brief 检查拖动示教模式
         * 
         * @param state 输出：1=拖动示教，0=非拖动示教
         */
        int IsInDragTeach(uint8_t& state) override { state = is_drag_teach_ ? 1 : 0; return 0; }

        /**
         * @brief 上使能/下使能
         * 
         * @param state 0=下使能，非0=上使能
         */
        int RobotEnable(uint8_t state) override { is_enabled_.store(state != 0); return 0; }

        /**
         * @brief 获取当前机器人状态
         * 
         * @param flag        0=阻塞等待更新，1=立即返回
         * @param robot_state 输出：当前状态
         */
        int GetCurrentState(uint8_t flag, RobotState& robot_state) override;

        /**
         * @brief 关节空间运动（MoveJ）：入队规划指令
         * 
         * @param joint_command 运动指令（target 为关节角）
         */
        int MoveJ(MotionCommand& joint_command) override;

        /**
         * @brief 笛卡尔空间直线运动（MoveL）：入队规划指令
         * 
         * @param desc_command 运动指令（target 为 [x,y,z,rx,ry,rz]）
         */
        int MoveL(MotionCommand& desc_command) override;

        /**
         * @brief 开启伺服模式
         */
        int ServoMoveStart() override;

        /**
         * @brief 结束伺服模式
         */
        int ServoMoveEnd() override;

        /**
         * @brief 关节空间伺服：入队伺服指令
         * 
         * @param joint_command 伺服指令（target 为关节角）
         */
        int ServoJ(MotionCommand& joint_command) override;

        /**
         * @brief 笛卡尔空间伺服：入队伺服指令
         * 
         * @param cart_command 伺服指令（target 为 [x,y,z,rx,ry,rz]）
         */
        int ServoCart(MotionCommand& cart_command) override;

        /**
         * @brief 点动：入队点动指令到队首
         * 
         * @param jog_command 点动指令
         */
        int StartJOG(MotionCommand& jog_command) override;

        /**
         * @brief 减速停止点动
         */
        int StopJOGDecel() override;

        /**
         * @brief 直接停止点动
         */
        int StopJOGImmediate() override;

        /**
         * @brief 停止所有运动，清空指令队列
         */
        int StopMotion() override;

        /**
         * @brief 恢复被暂停的运动
         */
        int ResumeMotion() override;

        /**
         * @brief 暂停当前运动
         */
        int PauseMotion() override;

        /**
         * @brief 急停后重置：复位轨迹调度器，可选清错误并重新上使能
         */
        int ResetMotion(const RusRobotDriver::ResetCmd& cmd) override;

        /**
         * @brief 查询运动是否已完成
         */
        bool IsMotionDone() const override;

        // === 工具坐标系相关接口 ===

        /**
         * @brief 六点法标定：记录第 point_num 个工具参考点（1~6）
         *
         * 仿真侧无法执行真实六点标定，仅记录当前法兰位姿（占位）。
         */
        int SetToolCalibPoint(int point_num) override;

        /**
         * @brief 六点法标定：计算工具坐标系
         *
         * 仿真侧占位：返回当前工具坐标系的变换（tool_transforms_[tool_index_]）。
         *
         * @param[out] tcp_pose 工具中心点相对末端法兰位姿 [x,y,z,rx,ry,rz]（m/rad）
         */
        int ComputeToolCalib(std::vector<double>& tcp_pose) override;

        /**
         * @brief 设置工具坐标系（工具中心点相对末端法兰位姿）并生效
         *
         * 更新本地 tool_transforms_ 变换矩阵表。
         *
         * @param id    坐标系编号 [0~14]
         * @param coord 工具相对法兰位姿 [x,y,z,rx,ry,rz]（m/rad）
         */
        int SetToolCoord(int id, const std::vector<double>& coord) override;

        /**
         * @brief 切换当前工具坐标系索引
         *
         * 同步更新运动学工具变换（KinematicsSolver::SetToolTransform）与状态 tool_index，
         * 使 MoveL 运动与 tool_pose 状态即时反映新工具坐标系。
         */
        int SetToolIndex(int id) override;

        // === 仿真控制 ===
        /**
         * @brief 设置仿真时间倍率
         * 
         * @param speed 倍率，1.0=实时，2.0=两倍速
         */
        void SetTimeSpeed(double speed) { sim_speed_ = speed; }

        /**
         * @brief 获取仿真时间倍率
         */
        double GetTimeSpeed() const { return sim_speed_; }

        /**
         * @brief 获取仿真已运行时间
         */
        double GetSimTime() const { return sim_time_; }

        /**
         * @brief 获取仿真帧率
         */
        double GetFrameRate() const { return frame_rate_; }

        /**
         * @brief 单步仿真（调试用，不依赖控制线程）
         */
        void StepOnce();

    private:
        /**
         * @brief 加载 MJCF/URDF，构建 MuJoCo 模型和 EAIK 运动学
         */
        void create_robot();

        /**
         * @brief 控制线程入口：compute_control → step_physics → sync_state
         */
        void control_loop();

        /**
         * @brief 控制层：推进轨迹 + 控制器算力矩 → 写入 ctrl
         */
        void compute_control();

        /**
         * @brief 物理层：MuJoCo 动力学积分
         */
        void step_physics();

        /**
         * @brief 同步层：MuJoCo 内部状态 → RobotState
         */
        void sync_state();

        /**
         * @brief 设置初始关节角
         *
         * @param q_init 初始关节角
         */
        void set_init_joints(const VectorXd& q_init);

        /**
         * @brief 到位判定：servo_end 请求后，等实际关节收敛到伺服目标（或超时）再清理伺服段
         */
        void check_servo_end();

        // === MuJoCo 仿真引擎 ===
        MjModelPtr mj_model_{nullptr, &mj_deleteModel};
        MjDataPtr  mj_data_{nullptr, &mj_deleteData};
        int end_effector_body_id_{-1};
        int num_dof_{0};

        // === EAIK 运动学 ===
        std::shared_ptr<EAIK::Robot> ki_model_;

        // === 机器人状态 ===
        RobotState current_state_;

        // === 运动学求解器（create_robot 中构造；工具变换随 SetToolIndex/SetToolCoord 更新） ===
        std::shared_ptr<RusRobotDriver::KinematicsSolver> kinematics_;

        // === 轨迹调度器与控制器（create_robot 后构造） ===
        std::unique_ptr<RusRobotDriver::TrajectoryExecutor> trajectory_executor_;
        std::unique_ptr<RusRobotDriver::IController> controller_;

        // === 控制线程 ===
        std::thread control_thread_;
        std::atomic<bool> stop_control_{false};
        double control_cycle_{0.001};

        // === 仿真时钟 ===
        std::atomic<double> sim_speed_{1.0};
        std::atomic<double> sim_time_{0.0};
        std::atomic<double> frame_rate_{0.0};

        // === 状态标志 ===
        std::atomic<bool> is_drag_teach_{false};
        std::atomic<bool> is_servo_enabled_{false};

        // === servo_end 到位判定（in-position）状态 ===
        std::atomic<bool> servo_end_requested_{false};   // 已收到 servo_end，等待到位后清理
        double servo_end_deadline_{-1.0};                // 到位判定超时时刻（sim_time）
        double servo_settle_tolerance_{0.005};           // 到位关节误差阈值 [rad]
        double servo_settle_timeout_{2.0};               // 到位判定超时 [s]

        std::atomic<bool> is_connected_{false};
        std::atomic<bool> is_enabled_{false};
        double flange_offset_{0.0938};  // 模型末端(wrist3_link)到法兰的 Z 向偏移 [m]（法兰坐标 = 末端 + 偏移）

        // === 工具坐标系变换矩阵 ===
        std::atomic<int> tool_index_{0};  // 工具坐标系索引，默认为0，表示法兰坐标系
        std::vector<Eigen::Matrix4d> tool_transforms_{Eigen::Matrix4d::Identity()};  // 工具坐标系变换矩阵
        std::vector<Eigen::Matrix4d> calib_points_;  // 六点法标定记录的位姿（仿真占位，SetToolCalibPoint 记录）

        // === 线程同步 ===
        uint64_t state_version_{0};
        std::condition_variable_any state_cv_;
        mutable std::recursive_mutex mtx_;
    };

}  // namespace RusSimRobotDriver