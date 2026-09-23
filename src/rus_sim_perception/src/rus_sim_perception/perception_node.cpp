#include "rus_sim_perception/perception_node.hpp"

#include <chrono>
#include <cstdint>
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
namespace {

    /// 秒 → ROS 时间（RCL_ROS_TIME：与 node->now() / header.stamp 同一时基）
    rclcpp::Time ToRosTime(double sec)
    {
        return rclcpp::Time(static_cast<int64_t>(sec * 1e9), RCL_ROS_TIME);
    }

}  // namespace

    PerceptionNode::PerceptionNode() : Node("perception_node")
    {
        // ── 话题 / 调度参数 ──
        input_cloud_topic_    = declare_parameter<std::string>("input_cloud_topic", "/camera/camera/depth/color/points");
        output_cloud_topic_   = declare_parameter<std::string>("output_cloud_topic", "/preprocessed_cloud");
        frame_topic_          = declare_parameter<std::string>("frame_topic", "/perception/frame");
        sensor_cloud_topic_   = declare_parameter<std::string>("sensor_cloud_topic", "/sensor/pointcloud");
        driver_state_topic_   = declare_parameter<std::string>("driver_state_topic", "/driver/state");
        max_allowed_diff_sec_ = declare_parameter<double>("max_allowed_diff_sec", 0.05);
        process_period_       = declare_parameter<double>("process_period", 0.1);
        allow_stale_pose_     = declare_parameter<bool>("allow_stale_pose", false);
        map_publish_period_   = declare_parameter<double>("map_publish_period", 2.0);
        max_pose_cache_       = static_cast<size_t>(declare_parameter<int>("max_pose_cache", 256));
        map_max_points_       = static_cast<size_t>(declare_parameter<int>("map_max_points", 500000));
        input_pcd_            = declare_parameter<std::string>("input_pcd", "");
        pcd_dir_              = declare_parameter<std::string>("pcd_dir", "");

        // ── 数据源参数（source 决定用哪一路采集；实现在 src/camera/）──
        source_type_  = declare_parameter<std::string>("source", "auto");
        mapping_mode_ = declare_parameter<std::string>("mapping_mode", "rolling");
        // 前端 /sensor 通路语义（与 planning 解耦的关键）：
        //   map   与 /preprocessed_cloud 同源（rolling/accumulate → 地图快照；none → 当前帧）
        //   frame 恒发当前单帧 —— 前端拿到的等价于 realsense-viewer 画面（单视角、无累积
        //         重影、无噪声沉淀），而 /preprocessed_cloud 仍按 mapping_mode 供 planning
        sensor_scope_ = declare_parameter<std::string>("sensor_scope", "map");
        // 空值 = auto（launch 传空占位时用探测：有设备走直连，否则回落话题）
        if (source_type_.empty()) source_type_ = "auto";
        source_cfg_.type  = source_type_;
        source_cfg_.topic = input_cloud_topic_;   // ros_topic 源输入话题
        // RealSense 直连（rs_*）
        source_cfg_.rs.width        = declare_parameter<int>("rs_width", 640);
        source_cfg_.rs.height       = declare_parameter<int>("rs_height", 480);
        source_cfg_.rs.fps          = declare_parameter<int>("rs_fps", 15);
        source_cfg_.rs.serial       = declare_parameter<std::string>("rs_serial", "");
        source_cfg_.rs.align_to     = declare_parameter<std::string>("rs_align_to", "none");
        source_cfg_.rs.color_mode   = declare_parameter<std::string>("rs_color_mode", "rgb");
        source_cfg_.rs.point_stride = declare_parameter<int>("rs_point_stride", 2);
        source_cfg_.rs.min_depth    = static_cast<float>(declare_parameter<double>("rs_min_depth", 0.15));
        source_cfg_.rs.max_depth    = static_cast<float>(declare_parameter<double>("rs_max_depth", 2.0));
        source_cfg_.rs.decimation      = declare_parameter<bool>("rs_decimation", false);
        source_cfg_.rs.spatial_filter  = declare_parameter<bool>("rs_spatial_filter", false);
        source_cfg_.rs.temporal_filter = declare_parameter<bool>("rs_temporal_filter", false);
        // 离线回放（replay_*）
        source_cfg_.replay.path     = declare_parameter<std::string>("replay_path", "");
        source_cfg_.replay.fps      = declare_parameter<double>("replay_fps", 5.0);
        source_cfg_.replay.loop     = declare_parameter<bool>("replay_loop", true);
        source_cfg_.replay.frame_id = declare_parameter<std::string>("replay_frame_id", "camera_optical_frame");
        if (mapping_mode_ != "none" && mapping_mode_ != "rolling" && mapping_mode_ != "accumulate") {
            RCLCPP_WARN(get_logger(), "未知 mapping_mode=%s（可选 none/rolling/accumulate），按 rolling 处理",
                        mapping_mode_.c_str());
            mapping_mode_ = "rolling";
        }
        if (sensor_scope_ != "map" && sensor_scope_ != "frame") {
            RCLCPP_WARN(get_logger(), "未知 sensor_scope=%s（可选 map/frame），按 map 处理",
                        sensor_scope_.c_str());
            sensor_scope_ = "map";
        }

        // ── 滤波参数（perception_params.yaml 注入）──
        PointCloud::FilterParameter param;
        param.voxel_leaf_size         = static_cast<float>(declare_parameter<double>("voxel_leaf_size", 0.005));
        param.passthrough_field       = declare_parameter<std::string>("passthrough_field", "z");
        param.passthrough_limit_min   = static_cast<float>(declare_parameter<double>("passthrough_limit_min", -0.5));
        param.passthrough_limit_max   = static_cast<float>(declare_parameter<double>("passthrough_limit_max", 0.5));
        param.passthrough_negative    = declare_parameter<bool>("passthrough_negative", false);
        param.enable_passthrough      = declare_parameter<bool>("enable_passthrough_filter", true);
        param.enable_statistical      = declare_parameter<bool>("enable_statistical_filter", false);
        param.enable_voxel            = declare_parameter<bool>("enable_voxel_filter", true);
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
        // 点数上限仅 rolling 生效（accumulate = 只累积不降采样；none = 不建图）
        map_.SetMaxPoints(mapping_mode_ == "rolling" ? map_max_points_ : 0);

        // 滤波链生效值（效果排查用：与 realsense-viewer 对照时先确认这里全为"关"）
        RCLCPP_INFO(get_logger(), "滤波链：直通=%s 统计=%s(%d 近邻/%.1fσ) 体素=%s(叶=%.4fm)",
                    param.enable_passthrough ? "开" : "关",
                    param.enable_statistical ? "开" : "关",
                    param.statistical_mean_k, param.statistical_std_dev_mul,
                    param.enable_voxel ? "开" : "关",
                    param.voxel_leaf_size);

        // ── 回调组：处理定时器独立，避免阻塞数据订阅回调 ──
        process_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        process_timer_ = create_wall_timer(
            std::chrono::duration<double>(process_period_),
            std::bind(&PerceptionNode::process_frame, this),
            process_cb_group_);

        // ── 订阅机械臂状态（点云改由数据源 camera/ 提供）──
        state_sub_ = create_subscription<RobotStateMsg>(
            driver_state_topic_, 10,
            [this](const RobotStateMsg::SharedPtr msg) { on_driver_state(msg); });

        // ── 发布：当前帧（RViz）+ /preprocessed_cloud（地图快照 or 当前帧，planning 输入）
        //          + 压缩帧（前端 /sensor 通路，待 bridge 订阅转发）──
        // TransientLocal：保留最新一帧，晚订阅的 RViz / 下游也能立即拿到数据
        rclcpp::QoS map_qos(10);
        map_qos.transient_local();
        frame_pub_  = create_publisher<PointCloud2>(frame_topic_, map_qos);
        cloud_pub_  = create_publisher<PointCloud2>(output_cloud_topic_, map_qos);
        sensor_pub_ = create_publisher<SensorFrame>(sensor_cloud_topic_, map_qos);

        // ── 指令服务（/perception/command，仅 map_clear）──
        cmd_server_ = create_service<CommandService>(
            "/perception/command",
            std::bind(&PerceptionNode::handle_command, this,
                      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

        // ── 数据源装配（source：auto / realsense / ros_topic / replay）──
        // auto：探测到 RealSense 设备走直连，否则回落 ROS 话题（仿真 / 包装节点）
        if (source_cfg_.type == "auto") {
            std::string detail;
            if (Camera::RealSenseAvailable(detail)) {
                RCLCPP_INFO(get_logger(), "source=auto：检测到 RealSense 设备（%s）→ 直连", detail.c_str());
                source_cfg_.type = "realsense";
            } else {
                RCLCPP_WARN(get_logger(), "source=auto：%s → 回落 ros_topic（%s）",
                            detail.c_str(), input_cloud_topic_.c_str());
                source_cfg_.type = "ros_topic";
            }
        }
        std::string source_error;
        source_ = Camera::CreateSource(source_cfg_, this, source_error);
        if (!source_) {
            RCLCPP_ERROR(get_logger(), "数据源创建失败：%s", source_error.c_str());
        } else {
            const std::string src_name = source_->Name();  // 采集线程日志用（不跨线程读成员）
            if (!source_->Start(
                    [this](Camera::CloudFrame&& f) { on_cloud_frame(std::move(f)); },
                    [this]() { return this->now().seconds(); },  // RealSense 必须用 ROS 时间（非设备时钟）
                    [this, src_name](const std::string& m) {
                        RCLCPP_WARN(get_logger(), "[%s] %s", src_name.c_str(), m.c_str());
                    },
                    source_error)) {
                RCLCPP_ERROR(get_logger(), "数据源启动失败：%s", source_error.c_str());
                source_.reset();
            }
        }

        RCLCPP_INFO(get_logger(),
            "PerceptionNode 已启动：数据源=%s 状态=%s 实时帧=%s 地图=%s 建图=%s 前端=%s "
            "处理周期=%.0fms 地图发布周期=%.1fs",
            source_ ? source_->Describe().c_str() : "无",
            driver_state_topic_.c_str(), frame_topic_.c_str(), output_cloud_topic_.c_str(),
            mapping_mode_.c_str(), sensor_scope_.c_str(),
            process_period_ * 1000.0, map_publish_period_);

        // 启动即加载离线点云（input_pcd 参数，联调用）
        if (!input_pcd_.empty()) {
            if (load_cloud_from_file(input_pcd_)) {
                RCLCPP_INFO(get_logger(), "已加载离线点云: %s", input_pcd_.c_str());
            } else {
                RCLCPP_ERROR(get_logger(), "离线点云加载失败: %s", input_pcd_.c_str());
            }
        }
    }
    PerceptionNode::~PerceptionNode()
    {
        // 先停数据源（join 采集线程），再析构 ROS 成员，
        // 避免采集线程在节点销毁过程中回调（now() / slot_ / 日志都是节点成员）
        if (source_) source_->Stop();
    }

    // ================================================================
    //  数据获取：数据源帧槽 / 机械臂状态
    // ================================================================

    void PerceptionNode::on_cloud_frame(Camera::CloudFrame&& frame)
    {
        // 只入槽（覆盖式留最新一帧），不做变换/滤波：处理全部在定时器线程，
        // 采集线程的回调必须足够短（RealSense 直连时这里是相机线程）
        slot_.Push(std::move(frame));
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
        // ── 实时处理：取最新帧 → 时间对齐 → 变换 → 滤波 → 入图 → 发布 ──
        Camera::CloudFrame frame;
        const bool have_input = slot_.TakeNewerThan(last_processed_stamp_, frame);

        if (have_input) {
            const double t = frame.stamp;
            Eigen::Isometry3d T_base_flange;
            double nearest_diff = 0.0;
            bool aligned = pose_interp_.Sample(t, T_base_flange, &nearest_diff);

            // 真机相机与驱动时钟不同步时允许降级：用最近位姿近似，避免持续丢帧
            if (!aligned && allow_stale_pose_ && pose_interp_.LatestPose(T_base_flange)) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                    "位姿对齐失败（与最近位姿差 %.4fs），已降级用最近位姿", nearest_diff);
                aligned = true;
            }

            if (!aligned) {
                // 对齐失败且未降级：丢弃本帧（帧槽取走即消费，不会反复重试同一帧）
                if (pose_interp_.Size() >= 2) {
                    // ⚠️ 措辞说明：Sample 返回 false 有两种原因 ——
                    //   ① 差值确实超容差（nearest_diff > max_allowed_diff_sec）
                    //   ② 点云时刻晚于最新位姿、位姿缓存尚未覆盖该时刻
                    //      （此时差值可能远小于容差，如 0.0024s；125Hz 位姿 vs 15fps 点云
                    //      存在这种时序竞态：处理该帧时对应位姿还在路上）
                    //   两者都按"丢帧"处理（保守，防错位数据）；要容忍 ② 可开 allow_stale_pose。
                    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                        "位姿对齐失败（差值 %.4fs，容差 %.4fs；或位姿尚未覆盖该时刻），"
                        "丢弃本帧（源=%s seq=%u）",
                        nearest_diff, max_allowed_diff_sec_, source_type_.c_str(), frame.seq);
                }
                last_processed_stamp_ = t;
                ++dropped_frames_;
            } else if (!frame.cloud || frame.cloud->empty()) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                    "采集到的点云为空（源=%s）", source_type_.c_str());
                last_processed_stamp_ = t;
                ++dropped_frames_;
            } else {
                // 相机光学系 → base_link（位姿 × 相机-法兰标定）→ 滤波
                CloudRGBPtr base_cloud;
                if (transformer_.Transform(frame.cloud, T_base_flange.matrix().cast<float>(), base_cloud)) {
                    std::vector<PointCloud::FilterStageStat> stats;
                    if (filter_.Apply(*base_cloud, &stats)) {
                        if (mapping_enabled()) map_.AddFrame(base_cloud);
                        ++processed_frames_;
                        last_processed_stamp_ = t;
                        if (!mapping_enabled()) {
                            // 单帧模式（mapping_mode=none）：无累积地图，当前帧即对外数据，
                            // 直接全量发布 /preprocessed_cloud（planning / 前端每帧整体替换）
                            publish_cloud(base_cloud, t);
                        } else if (sensor_scope_ == "frame") {
                            // 前端要单帧、planning 要地图：/preprocessed_cloud 仍按
                            // map_publish_period 发地图快照（地图节流发布见下方），
                            // /sensor/pointcloud 改发当前帧（scope=frame，无累积重影/噪声沉淀）
                            publish_sensor_frame(base_cloud, t,
                                                 std::string(RusUtils::SensorScope::kFrame));
                        }
                        // 实时帧发布（高频，RViz 订阅 /perception/frame 查看流畅画面）
                        publish_frame(base_cloud, t);
                    } else {
                        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "滤波后点云为空");
                        last_processed_stamp_ = t;
                        ++dropped_frames_;
                    }
                } else {
                    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "点云变换失败");
                    last_processed_stamp_ = t;
                    ++dropped_frames_;
                }
            }
        }

        // ── 发布 ──
        // mapping_mode=none：当前帧已随处理发布（见上），此处不重复发；
        // rolling/accumulate：全图 Compact + 快照拷贝 + zstd 编码开销大，若随 10Hz
        // 处理会阻塞实时帧，故独立节流（planning 仅在 pre_scan_done 时取一次）。
        const double wall = std::chrono::duration<double>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
        if (mapping_enabled()) {
            if (map_.PointCount() > 0) {
                if ((wall - last_map_publish_time_) >= map_publish_period_) {
                    map_.Compact();
                    // 地图快照时间戳 = 最近处理帧的采集时刻（尚无帧时用当前时刻）
                    const double stamp = (last_processed_stamp_ > 0.0)
                        ? last_processed_stamp_ : this->now().seconds();
                    publish_cloud(map_.Snapshot(), stamp);
                    last_map_publish_time_ = wall;
                }
            } else {
                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
                    "地图为空：等待 %s 输入（mapping_mode=%s）",
                    source_ ? source_->Name().c_str() : "数据源", mapping_mode_.c_str());
            }
        }

        // ── 帧率统计日志（每 2s 输出一次，便于真机排查）──
        if (wall - last_process_log_time_ >= 2.0) {
            process_rate_hz_ = processed_frames_ / std::max(0.001, wall - last_process_log_time_);
            RCLCPP_INFO(get_logger(),
                "处理统计：%.1fHz | 地图 %zu 帧 %zu 点 | 丢弃 %zu | 帧槽 收 %lu 覆盖 %lu 重复 %lu",
                process_rate_hz_, map_.FrameCount(), map_.PointCount(), dropped_frames_,
                static_cast<unsigned long>(slot_.Pushed()),
                static_cast<unsigned long>(slot_.Overwritten()),
                static_cast<unsigned long>(slot_.Stale()));
            processed_frames_ = 0;
            dropped_frames_ = 0;
            last_process_log_time_ = wall;
        }
    }

    /// 实时帧发布（高频；stamp = 该帧采集时刻，ROS 时间）
    bool PerceptionNode::publish_frame(const CloudRGBPtr& cloud, double stamp)
    {
        if (!cloud || cloud->empty()) return false;

        auto msg = std::make_shared<PointCloud2>();
        pcl::toROSMsg(*cloud, *msg);
        // toROSMsg 会用 PCL 点云 header 整体覆盖，frame_id/stamp 必须在其后设置
        msg->header.frame_id = "base_link";
        msg->header.stamp = ToRosTime(stamp);
        frame_pub_->publish(*msg);
        return true;
    }

    /// 点云发布：raw（planning 输入）+ 压缩帧（前端 /sensor 通路）
    /// 调用方：rolling/accumulate 传地图快照 + 最近帧时刻；none 传当前帧 + 采集时刻
    bool PerceptionNode::publish_cloud(const CloudRGBPtr& cloud, double stamp)
    {
        if (!cloud || cloud->empty()) return false;

        // 1) raw 发布（planning 规划输入）
        auto cloud_msg = std::make_shared<PointCloud2>();
        pcl::toROSMsg(*cloud, *cloud_msg);
        // toROSMsg 会用 PCL 点云 header 整体覆盖，frame_id/stamp 必须在其后设置
        cloud_msg->header.frame_id = "base_link";
        cloud_msg->header.stamp = ToRosTime(stamp);
        cloud_pub_->publish(*cloud_msg);

        // 2) 压缩帧发布（前端 /sensor 通路）
        //    同一话题两种语义：rolling/accumulate 发的是地图快照，none 发的是当前单视角帧。
        //    ⚠️ rolling/accumulate + sensor_scope=frame 时【不发】：前端通路已由 process_frame
        //    的单帧路径负责；若此处再发地图快照，/sensor 上会混进 map/frame 两种 scope，
        //    前端表现为周期性收到一个点数/范围突变的大帧。
        const bool emit_sensor_frame = !mapping_enabled() || (sensor_scope_ != "frame");
        if (emit_sensor_frame) {
            publish_sensor_frame(cloud, stamp,
                                 mapping_enabled() ? std::string(RusUtils::SensorScope::kMap)
                                                   : std::string(RusUtils::SensorScope::kFrame));
        }
        return true;
    }

    /// 压缩帧发布（/sensor/pointcloud → 前端 /sensor 通路）
    /// scope 由调用方指定：kMap = 地图快照（与 planning 同源），kFrame = 当前单帧（前端可视化）
    bool PerceptionNode::publish_sensor_frame(const CloudRGBPtr& cloud, double stamp,
                                              const std::string& scope)
    {
        if (!cloud || cloud->empty()) return false;

        EncodedFrame encoded;
        if (!encoder_.EncodePointCloud(*cloud, encoded)) return false;

        SensorFrame frame;
        frame.type = SensorFrame::TYPE_POINTCLOUD;
        frame.encoding = encoded.encoding;
        frame.stamp = ToRosTime(stamp);
        frame.seq = sensor_seq_++;
        frame.frame_id = "base_link";
        frame.scope = scope;
        frame.points = encoded.points;
        frame.fields = encoded.fields;
        frame.dtype = encoded.dtype;
        frame.range_min = {encoded.range_min[0], encoded.range_min[1], encoded.range_min[2]};
        frame.range_max = {encoded.range_max[0], encoded.range_max[1], encoded.range_max[2]};
        frame.data = std::move(encoded.payload);
        sensor_pub_->publish(frame);
        RCLCPP_DEBUG(get_logger(), "压缩帧已发布：%zu 点, scope=%s, %.1f KB",
                     cloud->size(), scope.c_str(), frame.data.size() / 1024.0);
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
        if (mapping_enabled()) {
            map_.AddFrame(cloud);              // 统一进入 map（唯一出口）
            map_.Compact();
        }
        // 直发一次（不占地图节流配额）：none 模式 = 本次加载即对外数据；
        // rolling/accumulate = 地图种子，RViz / planning 立即可见
        publish_cloud(cloud, this->now().seconds());
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

