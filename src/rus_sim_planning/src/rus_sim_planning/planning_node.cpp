#include "rus_sim_planning/planning_node.hpp"

#include <cmath>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "rus_sim_utils/command_defs.hpp"
#include "rus_sim_planning/planning_utils.hpp"

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

        // 法兰→探头 变换矩阵（16 元素行优先；探头安装在机械臂末端待标定，未配置则用单位矩阵）
        std::vector<double> empty_vec;  // 空参数默认值（避免 {} 歧义为 ParameterDescriptor）
        std::vector<double> probe_to_flange = declare_parameter<std::vector<double>>("probe_to_flange", empty_vec);
        if (probe_to_flange.size() == 16) {
            param.probe_to_flange.row(0) << probe_to_flange[0], probe_to_flange[1], probe_to_flange[2], probe_to_flange[3];
            param.probe_to_flange.row(1) << probe_to_flange[4], probe_to_flange[5], probe_to_flange[6], probe_to_flange[7];
            param.probe_to_flange.row(2) << probe_to_flange[8], probe_to_flange[9], probe_to_flange[10], probe_to_flange[11];
            param.probe_to_flange.row(3) << probe_to_flange[12], probe_to_flange[13], probe_to_flange[14], probe_to_flange[15];
        } else {
            RCLCPP_WARN(get_logger(), "probe_to_flange 参数不足 16 个（实际 %zu），使用单位矩阵（探头与法兰重合）",
                probe_to_flange.size());
        }
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
        else if (req->command == kPreScanStart) {
            // 预扫查开始：重置完成标记，等待点云数据
            prescan_done_ = false;
            res->success = true;
            res->message = "pre_scan started";
            RCLCPP_INFO(get_logger(), "预扫查开始");
        }
        else if (req->command == kPreScanEnd) {
            // 预扫查结束：点云已加载才算完成
            if (generator_.IsInitialized()) {
                prescan_done_ = true;
                res->success = true;
                res->message = "pre_scan done";
                publish_event(RusUtils::EventName::kPreScanDone, true, "pre_scan done", {}, req->client_id);
                RCLCPP_INFO(get_logger(), "预扫查完成（点云已就绪）");
            } else {
                res->success = false;
                res->message = "pre_scan failed: 未收到点云数据";
                publish_event(RusUtils::EventName::kError, false, res->message, {}, req->client_id);
                RCLCPP_WARN(get_logger(), "预扫查结束但未收到点云数据");
            }
        }
        else if (req->command == kQueryPreScanDone) {
            // 查询预扫查是否完成
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
        auto cloud = std::make_shared<PclCloud>();
        pcl::fromROSMsg(*msg, *cloud);
        RCLCPP_INFO(get_logger(), "收到点云：%zu 点", cloud->size());
        generator_.LoadCloud(cloud);
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
        state.flange_pos = to_eig(msg->flange_pos);
        state.timestamp  = msg->timestamp;
        interpolator_.SetRobotState(state);
    }

    // ---- Plan — 规划主流程：生成 → 插值（只规划，不执行） ----

    bool PlanningNode::Plan(uint32_t client_id)
    {
        // 预扫查门：未完成预扫查则无点云数据，禁止规划
        if (!prescan_done_ || !generator_.IsInitialized()) {
            RCLCPP_WARN(get_logger(), "未完成预扫查（无点云数据），请先 pre_scan_start → pre_scan_end");
            publish_event(RusUtils::EventName::kError, false,
                "plan failed: 未完成预扫查（无点云数据）", {}, client_id);
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
        // 规划完成事件（关联 plan 指令）
        publish_event(RusUtils::EventName::kPlanDone, true, "plan done", {}, client_id);
        return true;
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
        interpolator_.Rewind();

        // 开启驱动伺服模式；成功后启动逐点下发定时器
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
        if (interpolator_.IsFinished()) {
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
        send_cmd_async("stop", {});   // 彻底停止驱动运动
        interpolator_.Reset();
        RCLCPP_INFO(get_logger(), "规划状态已复位");
    }

    void PlanningNode::servo_tick()
    {
        if (!executing_) return;

        auto target = interpolator_.NextTarget();
        if (!target) {
            // 轨迹执行完毕 → 扫查完成事件 + 停止伺服
            RCLCPP_INFO(get_logger(), "轨迹执行完毕");
            publish_event(RusUtils::EventName::kScanDone, true, "scan done", {}, execute_client_id_);
            paused_ = false;
            stop_servo();
            return;
        }
        send_servo_cart(*target);
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
