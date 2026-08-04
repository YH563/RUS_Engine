#include "trajectory/trajectory_executor.hpp"
#include "trajectory/planned_segment.hpp"
#include "trajectory/servo_segment.hpp"
#include "trajectory/jog_segment.hpp"

namespace RusRobotDriver {

    // 判断是否为伺服/点动类型的辅助函数
    namespace {
        bool is_servo_type(uint8_t t) {
            return t == MOTION_TYPE_SERVOJ || t == MOTION_TYPE_SERVOC;
        }
        bool is_jog_type(uint8_t t) {
            return t == MOTION_TYPE_JOG_0 || t == MOTION_TYPE_JOG_1 || t == MOTION_TYPE_JOG_2;
        }
        bool is_continuous_type(uint8_t t) {
            return is_servo_type(t) || is_jog_type(t);
        }
    }

    TrajectoryExecutor::TrajectoryExecutor(std::shared_ptr<const KinematicsSolver> kinematics)
        : kinematics_(std::move(kinematics))
    {}

    void TrajectoryExecutor::PushBack(const MotionCommand& cmd)
    {
        command_queue_.push_back(cmd);
    }

    void TrajectoryExecutor::PushFront(const MotionCommand& cmd)
    {
        command_queue_.push_front(cmd);
    }

    void TrajectoryExecutor::Start()
    {
        exec_state_ = ExecState::RUNNING;
    }

    void TrajectoryExecutor::Pause()
    {
        exec_state_ = ExecState::PAUSED;
    }

    void TrajectoryExecutor::Resume()
    {
        if (exec_state_ == ExecState::PAUSED)
            exec_state_ = ExecState::RUNNING;
    }

    void TrajectoryExecutor::Stop()
    {
        command_queue_.clear();
        active_segment_.reset();
        exec_state_ = ExecState::STOPPED;
    }

    void TrajectoryExecutor::ClearQueue()
    {
        command_queue_.clear();
    }

    // ---- StopJOGDecel — 减速停止点动 ----
    void TrajectoryExecutor::StopJOGDecel()
    {
        if (!active_segment_ || !is_jog_type(active_segment_->GetType()))
            return;

        // 从最后一条实际执行的 JOG 指令获取当前速度
        jog_decel_speed_ = last_jog_cmd_.speed;
        jog_decel_active_ = true;
    }

    // ---- StopJOGImmediate — 直接停止点动 ----
    void TrajectoryExecutor::StopJOGImmediate()
    {
        if (active_segment_ && is_jog_type(active_segment_->GetType())) {
            active_segment_.reset();
            jog_decel_active_ = false;
        }
        clear_jog_commands();
    }

    bool TrajectoryExecutor::IsActive() const
    {
        return active_segment_ != nullptr && !active_segment_->IsFinished();
    }

    ControlTarget TrajectoryExecutor::Step(double sim_time, const RobotState& state)
    {
        if (exec_state_ != ExecState::RUNNING)
            return hold_target_;

        // 计算步长
        if (last_step_time_ < 0.0)
            last_step_time_ = sim_time;
        double dt = sim_time - last_step_time_;
        last_step_time_ = sim_time;
        if (dt <= 0.0 || dt > 0.1) dt = 0.001;

        // 先处理伺服指令（更新或创建 ServoSegment，不 step）
        process_servo_commands(state);

        // 再处理点动指令（若无活跃伺服段抢占）
        if (!active_segment_ || !is_servo_type(active_segment_->GetType()))
            process_jog_commands(state);

        // 处理点动减速：直接降低活跃段的速度
        if (jog_decel_active_ && active_segment_ && is_jog_type(active_segment_->GetType())) {
            jog_decel_speed_ -= kDecelRate * dt;
            if (jog_decel_speed_ <= 0.0) {
                jog_decel_active_ = false;
                // 把 hold_target 重置为当前状态，避免控制器继续追累积偏移后的位置
                hold_target_.q_des   = state.joint_pos;
                hold_target_.qd_des  = VectorXd::Zero(state.joint_pos.size());
                hold_target_.qdd_des = VectorXd::Zero(state.joint_pos.size());
                active_segment_.reset();
                clear_jog_commands();
            } else {
                // 不下发到队列，直接更新段速度，避免与新的点动指令冲突
                MotionCommand decel = last_jog_cmd_;
                decel.speed = jog_decel_speed_;
                active_segment_->UpdateTarget(decel, state);
            }
        }

        // 当前段结束（PlannedSegment 完成）→ 加载下一条规划指令
        if (!active_segment_ || active_segment_->IsFinished()) {
            uint8_t front_type = command_queue_.empty() ? 0xFF : command_queue_.front().type;
            if (!is_continuous_type(front_type))
                load_next_segment(state);
        }

        // 推进活跃段（PlannedSegment / ServoSegment / JogSegment 都在这里 step）
        if (active_segment_ && !active_segment_->IsFinished())
            hold_target_ = active_segment_->Step(sim_time, state);
        else if (!active_segment_) {
            // 无活跃段时重置为当前状态，防止 hold_target 残留旧目标导致控制器持续施力
            hold_target_.q_des   = state.joint_pos;
            hold_target_.qd_des  = VectorXd::Zero(state.joint_pos.size());
            hold_target_.qdd_des = VectorXd::Zero(state.joint_pos.size());
        }

        return hold_target_;
    }

    void TrajectoryExecutor::load_next_segment(const RobotState& state)
    {
        if (command_queue_.empty()) return;

        MotionCommand cmd = command_queue_.front();
        command_queue_.pop_front();

        if (cmd.type == MOTION_TYPE_JOINT) {
            active_segment_ = std::make_unique<PlannedSegment>(cmd, state);
        } else if (cmd.type == MOTION_TYPE_CART) {
            active_segment_ = std::make_unique<PlannedSegment>(cmd, state, kinematics_);
        }
    }

    void TrajectoryExecutor::process_servo_commands(const RobotState& state)
    {
        // 跳到最后一个伺服指令（中间堆积的直接丢弃，只需最新的）
        MotionCommand last_servo;
        bool has_servo = false;
        while (!command_queue_.empty() && is_servo_type(command_queue_.front().type)) {
            last_servo = command_queue_.front();
            command_queue_.pop_front();
            has_servo = true;
        }

        if (!has_servo) return;

        if (active_segment_ && is_servo_type(active_segment_->GetType())) {
            active_segment_->UpdateTarget(last_servo, state);
        } else {
            active_segment_ = std::make_unique<ServoSegment>(last_servo, state, kinematics_);
        }
    }

    // ---- process_jog_commands — 点动指令处理 ----
    void TrajectoryExecutor::process_jog_commands(const RobotState& state)
    {
        // 跳到最后一个点动指令（中间堆积的直接丢弃，只需最新的）
        bool has_jog = false;
        while (!command_queue_.empty() && is_jog_type(command_queue_.front().type)) {
            last_jog_cmd_ = command_queue_.front();
            command_queue_.pop_front();
            has_jog = true;
        }

        if (!has_jog) return;

        if (active_segment_ && is_jog_type(active_segment_->GetType())) {
            // 运动类型改变时（如 JOG_0↔JOG_1）重建段，确保 motion_type_ 正确
            if (active_segment_->GetType() != last_jog_cmd_.type) {
                active_segment_.reset();
                active_segment_ = std::make_unique<JogSegment>(last_jog_cmd_, state, kinematics_);
            } else {
                active_segment_->UpdateTarget(last_jog_cmd_, state);
            }
        } else {
            active_segment_ = std::make_unique<JogSegment>(last_jog_cmd_, state, kinematics_);
        }

        // 新点动指令到达时取消减速，直接接管
        jog_decel_active_ = false;
    }

    // ---- clear_jog_commands — 清空队列中的点动指令 ----
    void TrajectoryExecutor::clear_jog_commands()
    {
        while (!command_queue_.empty() && is_jog_type(command_queue_.front().type))
            command_queue_.pop_front();
    }

}  // namespace RusRobotDriver