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
#include "components/playback_controller.hpp"
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
         * @brief 查询运动是否已完成
         */
        bool IsMotionDone() const override;

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

        // ── 回放接口（基于 PlaybackController） ──

        /**
         * @brief 开始回放录制的运动数据
         *
         * 进入回放模式，跳过控制器的计算，将状态直接写入 MuJoCo 并用
         * mj_forward 更新运动学，避免动力学积分偏差。
         *
         * @param frames 录制帧序列
         */
        void StartPlayback(const std::vector<RobotState>& frames);

        /**
         * @brief 停止回放，恢复正常控制模式
         */
        void StopPlayback();

        /** @brief 暂停回放（当前位置暂停） */
        void PlaybackPause();

        /** @brief 恢复回放 */
        void PlaybackResume();

        /**
         * @brief 设置回放速度倍率
         * @param speed [0.01, 100.0]
         */
        void PlaybackSetSpeed(double speed);

        /**
         * @brief 跳转到指定时间位置（秒）
         */
        void PlaybackSeek(double time_seconds);

        /**
         * @brief 逐帧步进
         * @param direction 1=前进一帧, -1=后退一帧
         */
        void PlaybackStep(int8_t direction);

        /**
         * @brief 设置循环播放
         * @param enable 1=循环, 0=不循环
         */
        void PlaybackSetLoop(uint8_t enable);

        /**
         * @brief 获取回放信息
         * @param[out] info 依次为：
         *   [current_frame, total_frames, current_time, total_time,
         *    speed, progress(0~1), playing(0/1)]
         */
        void GetPlaybackInfo(std::vector<double>& info) const;

        /**
         * @brief 是否正在回放
         */
        bool IsPlaybackActive() const { return playback_active_; }

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
         * @brief 回放步进：写入录制的关节状态 → mj_forward 更新运动学
         */
        void playback_step();

        /**
         * @brief 设置初始关节角
         *
         * @param q_init 初始关节角
         */
        void set_init_joints(const VectorXd& q_init);

        // === MuJoCo 仿真引擎 ===
        MjModelPtr mj_model_{nullptr, &mj_deleteModel};
        MjDataPtr  mj_data_{nullptr, &mj_deleteData};
        int end_effector_body_id_{-1};
        int num_dof_{0};

        // === EAIK 运动学 ===
        std::shared_ptr<EAIK::Robot> ki_model_;

        // === 机器人状态 ===
        RobotState current_state_;

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

        std::atomic<bool> is_connected_{false};
        std::atomic<bool> is_enabled_{false};
        double flange_offset_{0.0938};

        // === 回放状态 ===
        std::atomic<bool> playback_active_{false};
        std::vector<RobotState> playback_frames_;       ///< 帧数据持有者（PlaybackController 只持有指针）
        RusRobotDriver::PlaybackController playback_ctrl_;

        // === 线程同步 ===
        uint64_t state_version_{0};
        std::condition_variable_any state_cv_;
        mutable std::recursive_mutex mtx_;
    };

}  // namespace RusSimRobotDriver