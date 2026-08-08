#include "driver/sim_driver.hpp"

#include "components/kinematics.hpp"

// 模型文件路径
static const std::string kShareDir = []() -> std::string {
    try {
        return ament_index_cpp::get_package_share_directory("rus_sim_driver");
    } catch (const std::exception&) {
        return std::string(std::getenv("HOME")) + "/RobotSDK/rus_sim_driver";
    }
}();
static const std::string kModelDir  = kShareDir + "/robot_model";
static const std::string kMjcfPath = kModelDir + "/robot_mujoco.xml";
static const std::string kUrdfPath = kModelDir + "/fairino3_v6.urdf";

namespace RusSimRobotDriver {
    using RusRobotDriver::ControlTarget;

    RobotSimDriver::RobotSimDriver(const std::string& ip) {
        create_robot();
        Connect(ip);

        // 创建轨迹调度器与 CTC 控制器
        // 运动学统一封装在 KinematicsSolver 中，由调度器持有并下发给各轨迹段
        auto kinematics = std::make_shared<RusRobotDriver::KinematicsSolver>(ki_model_, flange_offset_);
        trajectory_executor_ = std::make_unique<RusRobotDriver::TrajectoryExecutor>(kinematics);
        trajectory_executor_->Start();

        auto dynamics = [this](const VectorXd& q, const VectorXd& qd,
                               Eigen::MatrixXd& M, VectorXd& bias) {
            int nv = num_dof_;
            mju_copy(mj_data_->qpos, q.data(), nv);
            mju_copy(mj_data_->qvel, qd.data(), nv);
            mju_zero(mj_data_->qacc, nv);
            mj_inverse(mj_model_.get(), mj_data_.get());
            bias = Eigen::Map<const VectorXd>(mj_data_->qfrc_inverse, nv);

            std::vector<double> M_buf(nv * nv);
            mj_fullM(mj_model_.get(), M_buf.data(), mj_data_->qM);
            M = Eigen::Map<Eigen::MatrixXd>(M_buf.data(), nv, nv);
        };

        controller_ = RusRobotDriver::ControllerFactory::Create(
            RusRobotDriver::ControllerFactory::CTC, dynamics, 800.0, 50.0);

        is_enabled_.store(true);  // 默认上使能

        control_thread_ = std::thread(&RobotSimDriver::control_loop, this);
    }

    RobotSimDriver::~RobotSimDriver() {
        stop_control_.store(true);
        state_cv_.notify_all();
        if (control_thread_.joinable())
            control_thread_.join();
    }

    int RobotSimDriver::GetCurrentState(uint8_t flag, RobotState& robot_state) {
        // 阻塞获取状态
        if (flag == 0) {
            std::unique_lock<std::recursive_mutex> lock(mtx_);
            uint64_t wait_ver = state_version_;
            state_cv_.wait(lock, [this, wait_ver] {
                return state_version_ > wait_ver || stop_control_.load();
            });
            robot_state = current_state_;
        } else {  // 非阻塞的场景
            std::lock_guard<std::recursive_mutex> lock(mtx_);
            robot_state = current_state_;
        }
        return 0;
    }

    int RobotSimDriver::MoveJ(MotionCommand& cmd) {
        if (!is_enabled_) return -1;
        std::lock_guard<std::recursive_mutex> lock(mtx_);
        trajectory_executor_->PushBack(cmd);
        return 0;
    }

    int RobotSimDriver::MoveL(MotionCommand& cmd) {
        if (!is_enabled_) return -1;
        std::lock_guard<std::recursive_mutex> lock(mtx_);
        trajectory_executor_->PushBack(cmd);
        return 0;
    }

    int RobotSimDriver::ServoMoveStart() {
        if (!is_enabled_) return -1;
        is_servo_enabled_.store(true);
        return 0;
    }

    int RobotSimDriver::ServoMoveEnd() {
        if (!is_enabled_) return -1;
        is_servo_enabled_.store(false);
        return 0;
    }

    int RobotSimDriver::ServoJ(MotionCommand& cmd) {
        if (!is_enabled_ || !is_servo_enabled_) return -1;
        std::lock_guard<std::recursive_mutex> lock(mtx_);
        trajectory_executor_->PushFront(cmd);
        return 0;
    }

    int RobotSimDriver::ServoCart(MotionCommand& cmd) {
        if (!is_enabled_ || !is_servo_enabled_) return -1;
        std::lock_guard<std::recursive_mutex> lock(mtx_);
        trajectory_executor_->PushFront(cmd);
        return 0;
    }

    int RobotSimDriver::StartJOG(MotionCommand& cmd) {
        if (!is_enabled_) return -1;
        std::lock_guard<std::recursive_mutex> lock(mtx_);
        trajectory_executor_->PushFront(cmd);
        return 0;
    }

    int RobotSimDriver::StopJOGDecel() {
        if (!is_enabled_) return -1;
        std::lock_guard<std::recursive_mutex> lock(mtx_);
        trajectory_executor_->StopJOGDecel();
        return 0;
    }

    int RobotSimDriver::StopJOGImmediate() {
        if (!is_enabled_) return -1;
        std::lock_guard<std::recursive_mutex> lock(mtx_);
        trajectory_executor_->StopJOGImmediate();
        return 0;
    }

    int RobotSimDriver::StopMotion() {
        if (!is_enabled_) return -1;
        std::lock_guard<std::recursive_mutex> lock(mtx_);
        trajectory_executor_->Stop();
        is_servo_enabled_ = false;
        return 0;
    }

    int RobotSimDriver::ResumeMotion() {
        if (!is_enabled_) return -1;
        std::lock_guard<std::recursive_mutex> lock(mtx_);
        trajectory_executor_->Resume();
        return 0;
    }

    int RobotSimDriver::PauseMotion() {
        if (!is_enabled_) return -1;
        std::lock_guard<std::recursive_mutex> lock(mtx_);
        trajectory_executor_->Pause();
        return 0;
    }

    bool RobotSimDriver::IsMotionDone() const {
        return !trajectory_executor_->IsActive();
    }

    // ---- create_robot — 加载模型 + 构建 EAIK 运动学 ----
    void RobotSimDriver::create_robot() {
        char err[1000];

        mj_model_.reset(mj_loadXML(kMjcfPath.c_str(), nullptr, err, 1000));
        if (!mj_model_)
            throw std::runtime_error("MuJoCo load MJCF failed: " + std::string(err));
        mj_data_.reset(mj_makeData(mj_model_.get()));
        mj_model_->opt.timestep = control_cycle_;
        num_dof_ = mj_model_->nv;

        end_effector_body_id_ = mj_name2id(mj_model_.get(), mjOBJ_BODY, "wrist3_link");
        if (end_effector_body_id_ < 0)
            throw std::runtime_error("Cannot find end-effector body 'wrist3_link' in MJCF");

        // 构建 EAIK 运动学模型（从 URDF 解析）
        urdf::Model model;
        if (!model.initFile(kUrdfPath))
            throw std::runtime_error("Cannot load URDF for EAIK: " + kUrdfPath);

        auto link = model.getRoot();
        if (!link) throw std::runtime_error("No root link in URDF");

        std::vector<urdf::JointSharedPtr> joints;
        while (link) {
            urdf::JointSharedPtr next_joint = nullptr;
            for (auto& j : link->child_joints) {
                if (j->type == urdf::Joint::REVOLUTE) {
                    next_joint = j;
                    break;
                }
            }
            if (!next_joint) break;
            joints.push_back(next_joint);
            link = model.getLink(next_joint->child_link_name);
            if (!link) throw std::runtime_error("Joint has no child link");
        }

        int n = joints.size();
        if (n == 0) throw std::runtime_error("No revolute joints found");

        Eigen::Matrix<double, 3, Eigen::Dynamic> H(3, n);
        Eigen::Matrix<double, 3, Eigen::Dynamic> P(3, n + 1);
        Eigen::Vector3d pos = Eigen::Vector3d::Zero();
        Eigen::Matrix3d rot = Eigen::Matrix3d::Identity();
        Eigen::Vector3d prev_pos;

        for (int i = 0; i < n; ++i) {
            auto& jnt = joints[i];
            const auto& ori = jnt->parent_to_joint_origin_transform;
            Eigen::Vector3d p(ori.position.x, ori.position.y, ori.position.z);
            Eigen::Quaterniond q(ori.rotation.w, ori.rotation.x, ori.rotation.y, ori.rotation.z);
            Eigen::Matrix3d R = q.toRotationMatrix();

            pos = pos + rot * p;
            rot = rot * R;

            Eigen::Vector3d axis(jnt->axis.x, jnt->axis.y, jnt->axis.z);
            H.col(i) = (rot * axis.normalized()).normalized();

            if (i == 0) P.col(0) = pos;
            else        P.col(i) = pos - prev_pos;
            prev_pos = pos;
        }
        P.col(n) = pos - prev_pos;

        ki_model_ = std::make_shared<EAIK::Robot>(H, P, Eigen::Matrix3d::Identity(),
                                                   std::vector<std::pair<int, double>>{});

        // 初始姿态
        VectorXd q_init(mj_model_->nq);
        q_init << 0.0, -0.785, 1.571, 0.0, 0.785, 0.0;
        set_init_joints(q_init);
    }

    // ---- set_init_joints — 设置初始姿态 ----
    void RobotSimDriver::set_init_joints(const VectorXd& q_init) {
        std::lock_guard<std::recursive_mutex> lock(mtx_);

        mju_copy(mj_data_->qpos, q_init.data(), mj_model_->nq);
        mju_zero(mj_data_->qvel, mj_model_->nv);
        mju_zero(mj_data_->qacc, mj_model_->nv);
        mju_zero(mj_data_->ctrl, mj_model_->nv);  // 控制器首帧自动算出力矩
        mj_forward(mj_model_.get(), mj_data_.get());

        sync_state();
        is_servo_enabled_ = false;
        sim_time_ = 0.0;
    }

    // ---- compute_control — 轨迹推进 + CTC 力矩输出 ----
    void RobotSimDriver::compute_control() {
        ControlTarget target = trajectory_executor_->Step(sim_time_, current_state_);
        VectorXd torque = controller_->ComputeTorque(target, current_state_);
        for (int i = 0; i < num_dof_; ++i)
            mj_data_->ctrl[i] = torque(i);
    }

    // ---- step_physics — MuJoCo 动力学积分 ----
    void RobotSimDriver::step_physics() {
        mj_step(mj_model_.get(), mj_data_.get());
        sim_time_.store(mj_data_->time, std::memory_order_relaxed);
    }

    // ---- sync_state — MuJoCo → RobotState ----
    void RobotSimDriver::sync_state() {
        int nq = mj_model_->nq;
        int nv = mj_model_->nv;

        current_state_.flange_pos.resize(6);
        current_state_.joint_pos = Eigen::Map<VectorXd>(mj_data_->qpos, nq);
        current_state_.joint_vel = Eigen::Map<VectorXd>(mj_data_->qvel, nv);
        current_state_.joint_acc = Eigen::Map<VectorXd>(mj_data_->qacc, nv);
        current_state_.effort    = Eigen::Map<VectorXd>(mj_data_->qfrc_actuator, nv);
        current_state_.timestamp = sim_time_;

        // 末端位姿
        const double* px = mj_data_->xpos + 3 * end_effector_body_id_;
        const double* R9 = mj_data_->xmat + 9 * end_effector_body_id_;
        Eigen::Vector3d pos(px[0], px[1], px[2]);
        Eigen::Matrix3d R;
        R << R9[0], R9[1], R9[2],
             R9[3], R9[4], R9[5],
             R9[6], R9[7], R9[8];
        if (flange_offset_ != 0.0) {
            pos(0) += flange_offset_ * R(0, 2);
            pos(1) += flange_offset_ * R(1, 2);
            pos(2) += flange_offset_ * R(2, 2);
        }
        current_state_.flange_pos(0) = pos(0);
        current_state_.flange_pos(1) = pos(1);
        current_state_.flange_pos(2) = pos(2);
        current_state_.flange_pos(3) = std::atan2(R(2, 1), R(2, 2));
        current_state_.flange_pos(4) = std::asin(-R(2, 0));
        current_state_.flange_pos(5) = std::atan2(R(1, 0), R(0, 0));
    }

    // ---- control_loop — 实时控制循环 ----
    void RobotSimDriver::control_loop() {
        auto last_fps_time = std::chrono::steady_clock::now();
        int fps_counter = 0;

        while (!stop_control_.load()) {
            auto t0 = std::chrono::steady_clock::now();

            {
                std::lock_guard<std::recursive_mutex> lock(mtx_);
                compute_control();
                step_physics();
                sync_state();
                ++state_version_;
            }
            state_cv_.notify_all();

            // 帧率统计
            fps_counter++;
            auto now = std::chrono::steady_clock::now();
            if (now - last_fps_time >= std::chrono::seconds(1)) {
                frame_rate_ = fps_counter;
                fps_counter = 0;
                last_fps_time = now;
            }

            auto elapsed = std::chrono::steady_clock::now() - t0;
            double target_sleep = control_cycle_ / sim_speed_.load();
            auto sleep_dur = std::chrono::duration<double>(target_sleep);
            if (elapsed < sleep_dur)
                std::this_thread::sleep_for(sleep_dur - elapsed);
        }
    }


    // ---- StepOnce — 单步调试 ----
    void RobotSimDriver::StepOnce() {
        std::lock_guard<std::recursive_mutex> lock(mtx_);
        compute_control();
        step_physics();
        sync_state();
        ++state_version_;
        state_cv_.notify_all();
    }

}  // namespace RusSimRobotDriver