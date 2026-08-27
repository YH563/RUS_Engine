#include "rus_sim_perception/perception_node.hpp"

#include <chrono>
#include <functional>
#include <utility>
#include <algorithm>
#include <filesystem>

#include <geometry_msgs/msg/pose.hpp>
#include <pcl_conversions/pcl_conversions.h>

#include "rus_sim_utils/command_defs.hpp"
#include "rus_sim_utils/utils.hpp"
#include "pointcloud/cloud_io.hpp"

namespace RusPerception {

    PerceptionNode::PerceptionNode() : Node("perception_node")
    {
        // ── 话题 / 调度参数 ──
        input_cloud_topic_    = declare_parameter<std::string>("input_cloud_topic", "/camera/camera/depth/color/points");
        output_cloud_topic_   = declare_parameter<std::string>("output_cloud_topic", "/preprocessed_cloud");
        sensor_cloud_topic_   = declare_parameter<std::string>("sensor_cloud_topic", "/sensor/pointcloud");
        driver_state_topic_   = declare_parameter<std::string>("driver_state_topic", "/driver/state");
        max_allowed_diff_sec_ = declare_parameter<double>("max_allowed_diff_sec", 0.05);
        process_period_       = declare_parameter<double>("process_period", 1.0);
        max_pose_cache_       = static_cast<size_t>(declare_parameter<int>("max_pose_cache", 256));
        map_max_points_       = static_cast<size_t>(declare_parameter<int>("map_max_points", 0));
        input_pcd_            = declare_parameter<std::string>("input_pcd", "");
        pcd_dir_              = declare_parameter<std::string>("pcd_dir", "");

        // ── 滤波参数（perception_params.yaml 注入）──
        PointCloud::FilterParameter param;
        param.voxel_leaf_size         = static_cast<float>(declare_parameter<double>("voxel_leaf_size", 0.003));
        param.passthrough_field       = declare_parameter<std::string>("passthrough_field", "z");
        param.passthrough_limit_min   = static_cast<float>(declare_parameter<double>("passthrough_limit_min", -0.5));
        param.passthrough_limit_max   = static_cast<float>(declare_parameter<double>("passthrough_limit_max", 0.5));
        param.passthrough_negative    = declare_parameter<bool>("passthrough_negative", false);
        param.statistical_mean_k      = declare_parameter<int>("statistical_mean_k", 50);
        param.statistical_std_dev_mul = static_cast<float>(declare_parameter<double>("statistical_std_dev_mul", 1.0));

        // ── 相机→法兰 标定矩阵（16 元素行优先；未配置则单位阵）──
        // 点云变换：T_base = T_base_flange * camera_to_flange
        Eigen::Matrix4f camera_to_flange = Eigen::Matrix4f::Identity();
        std::vector<double> empty_vec;
        std::vector<double> calib = declare_parameter<std::vector<double>>("camera_to_flange", empty_vec);
        if (calib.size() == 16) {
            for (int r = 0; r < 4; ++r) {
                for (int c = 0; c < 4; ++c) {
                    camera_to_flange(r, c) = static_cast<float>(calib[static_cast<size_t>(r * 4 + c)]);
                }
            }
        } else {
            RCLCPP_WARN(get_logger(), "camera_to_flange 参数不足 16 个（实际 %zu），使用单位矩阵",
                        calib.size());
        }

        // ── 算法模块配置 ──
        pose_interp_ = PoseInterpolator(max_pose_cache_);
        transformer_.SetCameraToFlange(camera_to_flange);
        filter_.SetParameter(param);
        map_.SetVoxelLeafSize(param.voxel_leaf_size);
        map_.SetMaxPoints(map_max_points_);

        // ── 回调组：处理定时器独立，避免阻塞数据订阅回调 ──
        process_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        process_timer_ = create_wall_timer(
            std::chrono::duration<double>(process_period_),
            std::bind(&PerceptionNode::process_frame, this),
            process_cb_group_);

        // ── 订阅点云 / 机械臂状态 ──
        cloud_sub_ = create_subscription<PointCloud2>(
            input_cloud_topic_, 10,
            [this](const PointCloud2::SharedPtr msg) { on_cloud(msg); });
        state_sub_ = create_subscription<RobotStateMsg>(
            driver_state_topic_, 10,
            [this](const RobotStateMsg::SharedPtr msg) { on_driver_state(msg); });

        // ── 发布：raw 点云（planning）+ 压缩帧（前端）──
        // TransientLocal：保留最新一帧，晚订阅的 RViz / 下游也能立即拿到地图
        rclcpp::QoS map_qos(10);
        map_qos.transient_local();
        cloud_pub_  = create_publisher<PointCloud2>(output_cloud_topic_, map_qos);
        sensor_pub_ = create_publisher<SensorFrame>(sensor_cloud_topic_, map_qos);

        // ── 指令服务（/perception/command，仅 map_clear）──
        cmd_server_ = create_service<CommandService>(
            "/perception/command",
            std::bind(&PerceptionNode::handle_command, this,
                      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

        RCLCPP_INFO(get_logger(),
            "PerceptionNode 已启动：点云=%s 状态=%s raw=%s sensor=%s 处理周期=%.2fs",
            input_cloud_topic_.c_str(), driver_state_topic_.c_str(),
            output_cloud_topic_.c_str(), sensor_cloud_topic_.c_str(), process_period_);

        // 启动即加载离线点云（input_pcd 参数，联调用）
        if (!input_pcd_.empty()) {
            if (load_cloud_from_file(input_pcd_)) {
                RCLCPP_INFO(get_logger(), "已加载离线点云: %s", input_pcd_.c_str());
            } else {
                RCLCPP_ERROR(get_logger(), "离线点云加载失败: %s", input_pcd_.c_str());
            }
        }
    }
    // ================================================================
    //  数据获取：点云 / 机械臂状态
    // ================================================================

    void PerceptionNode::on_cloud(const PointCloud2::SharedPtr msg)
    {
        if (!msg || msg->data.empty()) {
            RCLCPP_ERROR(get_logger(), "传入的点云数据为空");
            return;
        }
        cloud_cache_ = msg;
        cloud_time_ = msg->header.stamp;  // 生产者时间戳（采集时刻），拒绝到达时刻
    }

    void PerceptionNode::on_driver_state(const RobotStateMsg::SharedPtr msg)
    {
        // 深度相机变换矩阵相对法兰：直接用法兰位姿（驱动已含模型法兰偏移补偿）。
        // 感知模块不依赖工具坐标系（TCP）；相机位姿 = 法兰 × 相机相对法兰（感知侧另行标定）。
        if (!msg || msg->flange_pos.size() < 6) return;

        const geometry_msgs::msg::Pose pose = RusUtils::FlangePosToPose(msg->flange_pos);
        const Eigen::Quaterniond q(pose.orientation.w, pose.orientation.x,
                                   pose.orientation.y, pose.orientation.z);
        const Eigen::Isometry3d T =
            Eigen::Translation3d(pose.position.x, pose.position.y, pose.position.z) * q;
        if (!pose_interp_.Add(rclcpp::Time(msg->header.stamp).seconds(), T)) {
            RCLCPP_DEBUG(get_logger(), "位姿时间戳非单调，已忽略");
        }
    }

    // ================================================================
    //  处理：时间对齐 → 变换 → 滤波 → 增量入图 → 双路发布
    // ================================================================

    void PerceptionNode::process_frame()
    {
        // 1) 实时管线：有输入点云且位姿足够时，处理并累积进地图
        const bool have_input = cloud_cache_ && !cloud_cache_->data.empty() && pose_interp_.Size() >= 2;
        if (have_input) {
            // 时间对齐：按点云采集时间戳插值法兰位姿
            const double t = cloud_time_.seconds();
            Eigen::Isometry3d T_base_flange;
            double nearest_diff = 0.0;
            if (!pose_interp_.Sample(t, T_base_flange, &nearest_diff)) {
                RCLCPP_WARN(get_logger(), "位姿插值失败（时间越界 t=%.3f），丢弃本帧", t);
            } else if (nearest_diff > max_allowed_diff_sec_) {
                RCLCPP_WARN(get_logger(), "点云与位姿时间差过大（%.4fs > %.4fs），丢弃本帧",
                            nearest_diff, max_allowed_diff_sec_);
            } else {
                // PointCloud2 → CloudRGB → 变换到 base_link → 滤波
                auto cloud_rgb = std::make_shared<CloudRGB>();
                pcl::fromROSMsg(*cloud_cache_, *cloud_rgb);
                CloudRGBPtr base_cloud;
                if (!cloud_rgb->empty() &&
                    transformer_.Transform(cloud_rgb, T_base_flange.matrix().cast<float>(), base_cloud)) {
                    std::vector<PointCloud::FilterStageStat> stats;
                    if (filter_.Apply(*base_cloud, &stats)) {
                        for (const auto& s : stats) {
                            RCLCPP_DEBUG(get_logger(), "滤波 %s: %zu → %zu", s.stage.c_str(), s.in, s.out);
                        }
                        map_.AddFrame(base_cloud);
                        RCLCPP_DEBUG(get_logger(), "地图更新：%zu 帧，%zu 点",
                                     map_.FrameCount(), map_.PointCount());
                    } else {
                        RCLCPP_WARN(get_logger(), "滤波后点云为空");
                    }
                } else {
                    RCLCPP_WARN(get_logger(), "点云变换失败");
                }
            }
        }

        // 2) 统一从地图发布（定时器持续驱动，RViz 始终有数据）
        if (map_.PointCount() > 0) {
            map_.Compact();
            publish_cloud(map_.Snapshot());
        } else {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                "地图为空（等待 /camera 输入，或 load_cloud 加载 PCD）");
        }
    }

    bool PerceptionNode::publish_cloud(const CloudRGBPtr& cloud)
    {
        if (!cloud || cloud->empty()) return false;

        // 1) raw 发布（planning 规划输入）
        auto cloud_msg = std::make_shared<PointCloud2>();
        pcl::toROSMsg(*cloud, *cloud_msg);
        // toROSMsg 会用 PCL 点云 header 整体覆盖，frame_id/stamp 必须在其后设置
        cloud_msg->header.frame_id = "base_link";
        cloud_msg->header.stamp = cloud_time_;
        cloud_pub_->publish(*cloud_msg);

        // 2) 压缩帧发布（前端 /sensor 通路）
        EncodedFrame encoded;
        if (encoder_.EncodePointCloud(*cloud, encoded)) {
            SensorFrame frame;
            frame.type = SensorFrame::TYPE_POINTCLOUD;
            frame.encoding = encoded.encoding;
            frame.stamp = cloud_time_;
            frame.seq = sensor_seq_++;
            frame.frame_id = "base_link";
            frame.points = encoded.points;
            frame.fields = encoded.fields;
            frame.dtype = encoded.dtype;
            frame.range_min = {encoded.range_min[0], encoded.range_min[1], encoded.range_min[2]};
            frame.range_max = {encoded.range_max[0], encoded.range_max[1], encoded.range_max[2]};
            frame.data = std::move(encoded.payload);
            sensor_pub_->publish(frame);
            RCLCPP_DEBUG(get_logger(), "点云已发布：%zu 点, sensor=%.1f KB",
                         cloud->size(), frame.data.size() / 1024.0);
        }
        return true;
    }

    // ================================================================
    //  离线点云加载（load_cloud 指令 / input_pcd 参数）
    // ================================================================

    CloudRGBPtr PerceptionNode::load_cloud_from_file(const std::string& path)
    {
        auto cloud = std::make_shared<CloudRGB>();
        if (!PointCloud::LoadPcd(path, *cloud)) {
            RCLCPP_ERROR(get_logger(), "PCD 加载失败: %s", path.c_str());
            return nullptr;
        }
        // PCD 视为 base_link 系：不执行坐标变换，只需滤波后作为初始地图
        std::vector<PointCloud::FilterStageStat> stats;
        if (!filter_.Apply(*cloud, &stats)) {
            RCLCPP_ERROR(get_logger(), "PCD 滤波后为空: %s", path.c_str());
            return nullptr;
        }
        map_.Clear();
        map_.AddFrame(cloud);              // 统一进入 map（唯一出口）
        map_.Compact();
        publish_cloud(map_.Snapshot());    // 立即发布一次，RViz 立即可见
        return cloud;
    }

    std::string PerceptionNode::pcd_path_by_index(int idx) const
    {
        if (idx < 0 || pcd_dir_.empty()) return "";
        std::vector<std::string> files;
        for (const auto& entry : std::filesystem::directory_iterator(pcd_dir_)) {
            if (entry.is_regular_file() && entry.path().extension() == ".pcd") {
                files.push_back(entry.path().string());
            }
        }
        std::sort(files.begin(), files.end());
        if (static_cast<size_t>(idx) >= files.size()) return "";
        return files[static_cast<size_t>(idx)];
    }

    // ================================================================
    //  指令处理（/perception/command）
    // ================================================================

    void PerceptionNode::handle_command(
        const std::shared_ptr<rmw_request_id_t> req_header,
        const std::shared_ptr<CommandService::Request> req,
        std::shared_ptr<CommandService::Response> res)
    {
        (void)req_header;
        using namespace RusUtils::CmdName;

        if (req->command == kMapClear) {
            map_.Clear();
            sensor_seq_ = 0;
            res->success = true;
            res->message = "map cleared";
        } else if (req->command == kLoadCloud) {
            // 无参数 → 加载 input_pcd_；args[0] 为整数索引 → 加载 pcd_dir_ 下第 N 个 .pcd
            std::string path = input_pcd_;
            if (!req->args.empty()) {
                const int idx = static_cast<int>(req->args[0]);
                const std::string resolved = pcd_path_by_index(idx);
                if (resolved.empty()) {
                    res->success = false;
                    res->message = "load_cloud failed: pcd_dir 未配置或索引越界";
                    RCLCPP_WARN(get_logger(), "%s", res->message.c_str());
                    return;
                }
                path = resolved;
            }
            if (path.empty()) {
                res->success = false;
                res->message = "load_cloud failed: 未配置 input_pcd，且无 pcd_dir 索引";
                RCLCPP_WARN(get_logger(), "%s", res->message.c_str());
                return;
            }
            const CloudRGBPtr loaded = load_cloud_from_file(path);
            if (!loaded) {
                res->success = false;
                res->message = "load_cloud failed: " + path;
                RCLCPP_ERROR(get_logger(), "%s", res->message.c_str());
                return;
            }
            res->success = true;
            res->result = {static_cast<double>(loaded->size())};
            res->message = "cloud loaded: " + path;
        } else {
            res->success = false;
            res->message = "unknown command: " + req->command;
            RCLCPP_WARN(get_logger(), "未知指令: %s", req->command.c_str());
            return;
        }

        RCLCPP_INFO(get_logger(), "指令 %s: %s", req->command.c_str(), res->message.c_str());
    }

}  // namespace RusPerception

