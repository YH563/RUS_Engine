#include "rus_sim_planning/planning_node.hpp"

#include <cmath>

#include <geometry_msgs/msg/pose_array.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "rus_sim_utils/command_defs.hpp"

namespace RusSimPlanning {

    namespace {
        using PointCloud2 = sensor_msgs::msg::PointCloud2;
        using DriverStateMsg = rus_sim_interfaces::msg::RobotState;
        using CommandService = rus_sim_interfaces::srv::CommandService;
        using PclCloud = pcl::PointCloud<pcl::PointXYZ>;
    }  // namespace

    PlanningNode::PlanningNode() : Node("planning_node")
    {
        // ── 话题 / 服务参数 ──
        point_cloud_topic_       = declare_parameter<std::string>("point_cloud_topic", "/preprocessed_cloud");
        driver_state_topic_      = declare_parameter<std::string>("driver_state_topic", "/driver/state");
        driver_command_service_  = declare_parameter<std::string>("driver_command_service", "/driver/command");
        servo_rate_hz_           = declare_parameter<double>("servo_rate_hz", 125.0);
        interpolate_points_      = declare_parameter<int>("interpolate_points", 10);
        movel_timeout_sec_       = declare_parameter<double>("movel_timeout_sec", 10.0);

        // ── 轨迹生成模块参数 ──
        TrajectoryParameter param;
        param.alpha         = declare_parameter<double>("alpha", 1.0);
        param.graph_k       = declare_parameter<int>("graph_k", 30);
        param.normal_k      = declare_parameter<int>("normal_k", 30);
        param.projection_k  = declare_parameter<int>("projection_k", 30);
        param.tol           = declare_parameter<double>("tol", 1.0e-6);
        param.max_iter      = declare_parameter<int>("max_iter", 40);
        param.use_smoothing = declare_parameter<bool>("use_smoothing", true);
        param.lambda        = declare_parameter<double>("lambda", 0.63);
        param.mu            = declare_parameter<double>("mu", -0.65);

        // 工具坐标系变换由驱动内部统一管理（见驱动 tool_coords 配置），planning 不再维护探头标定
        generator_.SetParameter(param);

        // ── 数据获取：点云订阅 ──
        cloud_sub_ = create_subscription<PointCloud2>(
            point_cloud_topic_, 10,
            [this](const PointCloud2::SharedPtr msg) { on_cloud(msg); });

        // ── 数据获取：机械臂状态（驱动话题）──
        state_sub_ = create_subscription<DriverStateMsg>(
            driver_state_topic_, 10,
            [this](const DriverStateMsg::SharedPtr msg) { on_driver_state(msg); });

        // ── 对外指令服务（/planning/command，供 bridge/客户端设置起终点与启停）──
        cmd_server_ = create_service<CommandService>(
            "/planning/command",
            std::bind(&PlanningNode::handle_command, this,
                std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

        // ── 整合转发：/driver/command 服务客户端 ──
        driver_cmd_client_ = create_client<CommandService>(driver_command_service_);

        // ── 事件发布（/module_events，bridge 订阅后广播为前端 event）──
        event_pub_ = create_publisher<rus_sim_interfaces::msg::ModuleEvent>("/module_events", 10);

        // ── 规划轨迹可视化（/planned_trajectory，RViz PoseArray 调试用）──
        path_pub_ = create_publisher<geometry_msgs::msg::PoseArray>("/planned_trajectory", 10);

        RCLCPP_INFO(get_logger(),
            "PlanningNode 已启动：点云=%s 状态=%s 指令=/planning/command 伺服=%.0fHz",
            point_cloud_topic_.c_str(), driver_state_topic_.c_str(), servo_rate_hz_);
    }

    // ---- handle_command — 外部指令处理（/planning/command） ----

    void PlanningNode::handle_command(
        const std::shared_ptr<rmw_request_id_t> req_header,
        const std::shared_ptr<CommandService::Request> req,
        std::shared_ptr<CommandService::Response> res)
    {
        (void)req_header;
        using namespace RusUtils::CmdName;

        if (req->command == kSetStartPose) {
            if (req->args.size() < 3) {
                res->success = false;
                res->message = "set_start_pose 需要 ≥3 个参数 [x,y,z,...]";
                RCLCPP_WARN(get_logger(), "set_start_pose 参数不足");
                return;
            }
            start_pose_ = MakePose(req->args);
            res->success = true;
            res->message = "start_pose set";
            RCLCPP_INFO(get_logger(), "起点已设置: (%.3f, %.3f, %.3f)",
                start_pose_->position.x, start_pose_->position.y, start_pose_->position.z);
        }
        else if (req->command == kSetEndPose) {
            if (req->args.size() < 3) {
                res->success = false;
                res->message = "set_end_pose 需要 ≥3 个参数 [x,y,z,...]";
                RCLCPP_WARN(get_logger(), "set_end_pose 参数不足");
                return;
            }
            goal_pose_ = MakePose(req->args);
            res->success = true;
            res->message = "end_pose set";
            RCLCPP_INFO(get_logger(), "终点已设置: (%.3f, %.3f, %.3f)",
                goal_pose_->position.x, goal_pose_->position.y, goal_pose_->position.z);
        }
        else if (req->command == kPreScanDone) {
            // 外部（医生完成手动扫查）触发：取最新点云一次性初始化轨迹生成器
            if (!cloud_cache_ || cloud_cache_->data.empty()) {
                res->success = false;
                res->message = "pre_scan_done failed: 尚未收到点云";
                RCLCPP_WARN(get_logger(), "%s", res->message.c_str());
                return;
            }
            auto cloud = std::make_shared<PclCloud>();
            pcl::fromROSMsg(*cloud_cache_, *cloud);
            if (cloud->empty()) {
                res->success = false;
                res->message = "pre_scan_done failed: 点云为空";
                RCLCPP_WARN(get_logger(), "%s", res->message.c_str());
                return;
            }
            if (!generator_.LoadCloud(cloud)) {
                res->success = false;
                res->message = "pre_scan_done failed: 点云初始化失败";
                RCLCPP_ERROR(get_logger(), "%s", res->message.c_str());
                return;
            }
            prescan_done_ = true;
            res->success = true;
            res->message = "pre_scan done";
            RCLCPP_INFO(get_logger(), "预扫查完成，轨迹生成器已初始化（%zu 点）", cloud->size());
        }
        else if (req->command == kQueryPreScanDone) {
            // 查询预扫查状态（外部 pre_scan_done 指令驱动）
            res->success = true;
            res->result = {prescan_done_ ? 1.0 : 0.0};
            res->message = prescan_done_ ? "pre_scan done" : "pre_scan not done";
        }
        else if (req->command == kQueryMotionDone) {
            // planning 持有动作完成状态：执行中或暂停中都算未完成（driver 原始状态会误导）
            bool done = !executing_ && !paused_;
            res->success = true;
            res->result = {done ? 1.0 : 0.0};
            res->message = done ? "motion done" : (paused_ ? "paused" : "scanning");
        }
        else if (req->command == kPlan) {
            res->success = Plan(req->client_id);
            res->message = res->success ? "plan ok" : "plan failed";
        }
        else if (req->command == kExecute) {
            res->success = Execute(req->client_id);
            res->message = res->success ? "execute ok" : "execute failed";
        }
        else if (req->command == kStop) {
            StopExecution();
            res->success = true;
            res->message = "stopped";
        }
        else if (req->command == kPause) {
            res->success = Pause();
            res->message = res->success ? "paused" : "pause failed（未在执行中）";
        }
        else if (req->command == kResume) {
            res->success = Resume();
            res->message = res->success ? "resumed" : "resume failed（未在暂停中）";
        }
        else if (req->command == kReset) {
            ResetState();
            res->success = true;
            res->message = "planning state reset";
        }
        else {
            res->success = false;
            res->message = "unknown command: " + req->command;
            RCLCPP_WARN(get_logger(), "未知指令: %s", req->command.c_str());
            return;
        }

        if (res->success) {
            RCLCPP_INFO(get_logger(), "指令 %s 成功: %s", req->command.c_str(), res->message.c_str());
        } else {
            RCLCPP_WARN(get_logger(), "指令 %s 失败: %s", req->command.c_str(), res->message.c_str());
        }
    }

    // ---- 数据获取：点云 / 机械臂状态 ----

    void PlanningNode::on_cloud(const PointCloud2::SharedPtr msg)
    {
        // 只缓存最新点云；收到外部 pre_scan_done 指令后取一次做轨迹生成器初始化
        if (!msg || msg->data.empty()) return;
        cloud_cache_ = msg;
        RCLCPP_DEBUG(get_logger(), "缓存最新点云（等待 pre_scan_done 指令初始化）");
    }

    void PlanningNode::on_driver_state(const DriverStateMsg::SharedPtr msg)
    {
        // 驱动话题状态 → 通用状态 → 注入插值计算模块（供控制算法使用）
        RusUtils::RobotState state;
        auto to_eig = [](const std::vector<double>& v) {
            Eigen::VectorXd q(v.size());
            for (size_t i = 0; i < v.size(); ++i) q(i) = v[i];
            return q;
        };
        state.joint_pos  = to_eig(msg->joint_pos);
        state.joint_vel  = to_eig(msg->joint_vel);
        state.joint_acc  = to_eig(msg->joint_acc);
        state.effort     = to_eig(msg->effort);
        // 当前 TCP 位姿（驱动已按工具坐标系计算），作为规划起点参考
        state.flange_pos = to_eig(msg->tool_pose.size() >= 6 ? msg->tool_pose : msg->flange_pos);
        state.timestamp  = rclcpp::Time(msg->header.stamp).seconds();
        interpolator_.SetRobotState(state);
    }

    // ---- Plan — 规划主流程：生成 → 插值（只规划，不执行） ----

    bool PlanningNode::Plan(uint32_t client_id)
    {
        // 预扫查门：外部 pre_scan_done 指令完成点云初始化后，才允许规划
        if (!prescan_done_ || !generator_.IsInitialized()) {
            RCLCPP_WARN(get_logger(), "尚未完成预扫查（等待外部 pre_scan_done 指令）");
            publish_event(RusUtils::EventName::kError, false,
                "plan failed: 未完成预扫查", {}, client_id);
            return false;
        }
        if (!start_pose_ || !goal_pose_) {
            RCLCPP_WARN(get_logger(), "起终点未设置（先发 set_start_pose / set_end_pose）");
            publish_event(RusUtils::EventName::kError, false,
                "plan failed: 起终点未设置", {}, client_id);
            return false;
        }
        if (!generator_.GenerateTrajectory(*start_pose_, *goal_pose_)) {
            RCLCPP_ERROR(get_logger(), "轨迹生成失败");
            publish_event(RusUtils::EventName::kError, false,
                "plan failed: 轨迹生成失败", {}, client_id);
            return false;
        }
        auto sparse = generator_.GetTrajectory();
        if (!sparse) {
            RCLCPP_ERROR(get_logger(), "生成轨迹为空");
            publish_event(RusUtils::EventName::kError, false,
                "plan failed: 生成轨迹为空", {}, client_id);
            return false;
        }
        const Trajectory& waypoints = sparse.value().get();

        // 插值计算模块：路径稠密化
        interpolator_.SetWaypoints(waypoints);
        if (!interpolator_.Interpolate(interpolate_points_)) {
            RCLCPP_ERROR(get_logger(), "插值失败（路径点不足）");
            publish_event(RusUtils::EventName::kError, false,
                "plan failed: 插值失败", {}, client_id);
            return false;
        }
        RCLCPP_INFO(get_logger(), "规划完成：稀疏 %zu 点 → 稠密 %zu 点",
            waypoints.size(), interpolator_.DenseTrajectory().size());
        // 规划轨迹发布到 RViz（调试：对比实际执行位姿）
        publish_planned_path();
        // 规划完成事件（关联 plan 指令）
        publish_event(RusUtils::EventName::kPlanDone, true, "plan done", {}, client_id);
        return true;
    }

    void PlanningNode::publish_planned_path()
    {
        const auto& dense = interpolator_.DenseTrajectory();
        if (dense.empty()) return;
        // 纯数据话题：发布全部稠密轨迹点（PoseArray，frame=base_link）。
        // 绘制交给外部脚本（如 traj_vis.py），不在本节点做可视化。
        auto msg = std::make_shared<geometry_msgs::msg::PoseArray>();
        msg->header.stamp = this->now();
        msg->header.frame_id = "base_link";
        msg->poses.reserve(dense.size());
        for (const auto& p : dense)
            msg->poses.push_back(p);
        path_pub_->publish(*msg);
    }

    // ---- Execute / servo_tick — 伺服执行：按 servo_rate_hz 逐点下发 ----

    bool PlanningNode::Execute(uint32_t client_id)
    {
        if (executing_) {
            RCLCPP_WARN(get_logger(), "已在伺服执行中");
            return false;
        }
        const auto& dense = interpolator_.DenseTrajectory();
        if (dense.empty()) {
            RCLCPP_WARN(get_logger(), "无已规划的轨迹（先发 plan）");
            publish_event(RusUtils::EventName::kError, false,
                "execute failed: 无已规划轨迹（先发 plan）", {}, client_id);
            return false;
        }
        if (!driver_cmd_client_->service_is_ready()) {
            RCLCPP_WARN(get_logger(), "驱动服务 %s 不可用", driver_command_service_.c_str());
            return false;
        }

        // 记录本次 execute 的 client_id（scan_done 事件关联）
        execute_client_id_ = client_id;

        // 回到轨迹起点重新执行
        end_hold_remaining_ = 0;
        trajectory_done_ = false;
        interpolator_.Rewind();

        // 先 movel 到轨迹起点（异步等待到达，不阻塞服务回调），到达后自动进入伺服
        const Pose& start_pose = dense.front();
        double rx = 0.0, ry = 0.0, rz = 0.0;
        PoseToRPY(start_pose, rx, ry, rz);
        RCLCPP_INFO(get_logger(),
            "先 movel 到起点 (%.3f, %.3f, %.3f, %.3f, %.3f, %.3f)...",
            start_pose.position.x, start_pose.position.y, start_pose.position.z,
            rx, ry, rz);
        send_cmd_async("movel",
            {start_pose.position.x, start_pose.position.y, start_pose.position.z,
             rx, ry, rz});

        movel_deadline_ = this->now() + rclcpp::Duration::from_seconds(movel_timeout_sec_);
        if (!movel_wait_timer_) {
            movel_wait_timer_ = create_wall_timer(std::chrono::milliseconds(100),
                std::bind(&PlanningNode::on_movel_wait_tick, this));
        } else {
            movel_wait_timer_->reset();
        }
        return true;
    }

    // ---- Pause / Resume — 暂停 / 恢复伺服执行 ----

    bool PlanningNode::Pause()
    {
        if (paused_) {
            RCLCPP_WARN(get_logger(), "已在暂停中");
            return true;  // 幂等
        }
        if (!executing_) {
            RCLCPP_WARN(get_logger(), "未在执行中，无法暂停");
            return false;
        }
        // 停掉伺服下发（发 servo_end）；driver 之后可能上报运动完成，
        // 但 planning 知道是暂停 → query_motion_done 仍返回未完成
        stop_servo();
        paused_ = true;
        RCLCPP_INFO(get_logger(), "伺服执行已暂停");
        return true;
    }

    bool PlanningNode::Resume()
    {
        if (!paused_) {
            RCLCPP_WARN(get_logger(), "未在暂停中，无法恢复");
            return false;
        }
        if (interpolator_.IsFinished() && end_hold_remaining_ == 0) {
            RCLCPP_WARN(get_logger(), "轨迹已执行完，无需恢复");
            paused_ = false;
            return false;
        }
        // 重新开启伺服模式并继续逐点下发（从暂停位置继续）
        auto req = std::make_shared<CommandService::Request>();
        req->client_id = 0;
        req->command = "servo_start";
        driver_cmd_client_->async_send_request(req,
            [this](rclcpp::Client<CommandService>::SharedFuture future) {
                try {
                    auto res = future.get();
                    if (!res->success) {
                        RCLCPP_ERROR(get_logger(), "servo_start 失败：%s", res->message.c_str());
                        return;
                    }
                } catch (const std::exception& e) {
                    RCLCPP_ERROR(get_logger(), "servo_start 异常：%s", e.what());
                    return;
                }
                paused_ = false;
                executing_ = true;
                start_servo_timer();
                RCLCPP_INFO(get_logger(), "伺服执行已恢复，剩余 %zu 点",
                    interpolator_.DenseTrajectory().size() - interpolator_.CurrentIndex());
            });
        return true;
    }

    // ---- ResetState — 复位规划状态 ----

    void PlanningNode::ResetState()
    {
        stop_servo();
        paused_ = false;
        executing_ = false;
        end_hold_remaining_ = 0;
        trajectory_done_ = false;
        send_cmd_async("stop", {});   // 彻底停止驱动运动
        interpolator_.Reset();
        RCLCPP_INFO(get_logger(), "规划状态已复位");
    }

    void PlanningNode::servo_tick()
    {
        if (!executing_) return;

        // ── 正常轨迹下发阶段（轨迹未全部下发完才进入，避免保持期结束后死循环）──
        if (!trajectory_done_) {
            auto target = interpolator_.NextTarget();
            if (target) {
                send_servo_cart(*target);
                return;
            }
            // 轨迹点已全部下发 → 进入终点保持（dwell）：重复下发终点位姿，
            // 让驱动端滤波在终点收敛，避免 servo_end 时机械臂停在半路/下坠。
            // 工业界 CP 轨迹终点普遍采用到位判定（in-position）或保持时间处理。
            trajectory_done_ = true;
            const auto& dense = interpolator_.DenseTrajectory();
            if (!dense.empty()) {
                end_hold_remaining_ = static_cast<int>(servo_rate_hz_ * end_hold_sec_);
                RCLCPP_INFO(get_logger(), "轨迹点已全部下发，终点保持 %.1fs（%d 帧）...",
                    end_hold_sec_, end_hold_remaining_);
            }
        }

        // ── 终点保持阶段：重复下发终点位姿 ──
        if (end_hold_remaining_ > 0) {
            const auto& dense = interpolator_.DenseTrajectory();
            if (!dense.empty())
                send_servo_cart(dense.back());
            --end_hold_remaining_;
            return;
        }

        // 终点已稳定 → 结束伺服
        RCLCPP_INFO(get_logger(), "轨迹执行完毕（终点已稳定）");
        publish_event(RusUtils::EventName::kScanDone, true, "scan done", {}, execute_client_id_);
        paused_ = false;
        stop_servo();
    }

    void PlanningNode::StopExecution()
    {
        if (!executing_ && !paused_) return;
        // 外部 stop：扫查被中止
        publish_event(RusUtils::EventName::kScanDone, false, "scan stopped", {}, execute_client_id_);
        paused_ = false;
        stop_servo();
        send_cmd_async("stop", {});   // 彻底停止驱动运动
    }

    void PlanningNode::stop_servo()
    {
        executing_ = false;
        if (servo_timer_) servo_timer_->cancel();
        send_cmd_async("servo_end", {});
        RCLCPP_INFO(get_logger(), "伺服执行已停止");
    }

    void PlanningNode::start_servo_timer()
    {
        auto period = std::chrono::duration<double>(1.0 / servo_rate_hz_);
        if (!servo_timer_) {
            servo_timer_ = create_wall_timer(period, std::bind(&PlanningNode::servo_tick, this));
        } else {
            servo_timer_->reset();
        }
    }

    // ---- 指令下发（/driver/command 服务客户端） ----

    void PlanningNode::send_servo_cart(const geometry_msgs::msg::Pose& pose)
    {
        double rx, ry, rz;
        PoseToRPY(pose, rx, ry, rz);
        send_cmd_async("servo_cart",
            {pose.position.x, pose.position.y, pose.position.z, rx, ry, rz});
    }

    void PlanningNode::send_cmd_async(const std::string& cmd, const std::vector<double>& args)
    {
        if (!driver_cmd_client_->service_is_ready()) {
            RCLCPP_WARN(get_logger(), "驱动服务 %s 不可用", driver_command_service_.c_str());
            return;
        }
        auto req = std::make_shared<CommandService::Request>();
        req->client_id = 0;
        req->command = cmd;
        req->args = args;
        driver_cmd_client_->async_send_request(req);
    }

    void PlanningNode::on_movel_wait_tick()
    {
        // 超时：直接进入伺服（容错）
        if (this->now() > movel_deadline_) {
            if (movel_wait_timer_) movel_wait_timer_->cancel();
            RCLCPP_WARN(get_logger(), "movel 到达起点超时（%.1fs），仍尝试伺服执行",
                        movel_timeout_sec_);
            start_servo_sequence();
            return;
        }

        // 异步查询驱动 is_motion_done（响应由 executor 处理，不阻塞任何回调）
        auto req = std::make_shared<CommandService::Request>();
        req->client_id = 0;
        req->command = std::string(RusUtils::CmdName::kIsMotionDone);
        driver_cmd_client_->async_send_request(req,
            [this](rclcpp::Client<CommandService>::SharedFuture future) {
                try {
                    auto res = future.get();
                    if (res->success && !res->result.empty() && res->result[0] > 0.5) {
                        if (movel_wait_timer_) movel_wait_timer_->cancel();
                        RCLCPP_INFO(get_logger(), "movel 已到达起点");
                        start_servo_sequence();
                    }
                } catch (const std::exception& e) {
                    RCLCPP_WARN(get_logger(), "is_motion_done 查询异常：%s", e.what());
                }
            });
    }

    void PlanningNode::start_servo_sequence()
    {
        auto req = std::make_shared<CommandService::Request>();
        req->client_id = 0;
        req->command = "servo_start";
        driver_cmd_client_->async_send_request(req,
            [this](rclcpp::Client<CommandService>::SharedFuture future) {
                try {
                    auto res = future.get();
                    if (!res->success) {
                        RCLCPP_ERROR(get_logger(), "servo_start 失败：%s", res->message.c_str());
                        return;
                    }
                } catch (const std::exception& e) {
                    RCLCPP_ERROR(get_logger(), "servo_start 异常：%s", e.what());
                    return;
                }
                executing_ = true;
                start_servo_timer();
                RCLCPP_INFO(get_logger(), "伺服执行开始 @ %.0fHz，轨迹 %zu 点",
                    servo_rate_hz_, interpolator_.DenseTrajectory().size());
            });
    }

    // ---- publish_event — 发布模块事件（→ /module_events，bridge 转发前端 event） ----

    void PlanningNode::publish_event(std::string_view event, bool success,
                                     std::string_view message,
                                     const std::vector<double>& result,
                                     uint32_t client_id)
    {
        rus_sim_interfaces::msg::ModuleEvent evt;
        evt.event = std::string(event);
        evt.success = success;
        evt.message = std::string(message);
        evt.result = result;
        evt.client_id = client_id;   // 关联触发指令的前端 command id（→ event.ack_id）
        evt.module = "planning";
        evt.stamp = now();
        event_pub_->publish(evt);
    }

}  // namespace RusSimPlanning
