#include "rus_sim_reconstruction/reconstruction_node.hpp"

#include <chrono>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

namespace RusReconstruction {

    ReconstructionNode::ReconstructionNode() : rclcpp::Node("reconstruction_node")
    {
        input_topic_ = declare_parameter<std::string>("cloud_topic", "/perception/frame");
        output_topic_ = declare_parameter<std::string>("output_topic", "/reconstructed_cloud");
        const double voxel = declare_parameter<double>("voxel_size", 0.004);
        const int shards = declare_parameter<int>("shards", 12);
        const int threads = declare_parameter<int>("threads", 0);
        min_confidence_ = declare_parameter<double>("min_confidence", 2.0);
        publish_period_ = declare_parameter<double>("publish_period", 1.0);
        estimate_normals_ = declare_parameter<bool>("estimate_normals", true);
        normal_k_ = declare_parameter<int>("normal_k", 10);
        save_pcd_ = declare_parameter<std::string>("save_pcd", "");

        SurfelFusionOptions opt;
        opt.voxel_size = voxel;
        map_ = std::make_unique<ShardedSurfelMap>(opt, shards, threads);

        auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
        sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            input_topic_, qos, std::bind(&ReconstructionNode::OnCloud, this, std::placeholders::_1));
        pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, qos);
        timer_ = create_wall_timer(std::chrono::duration<double>(publish_period_),
            std::bind(&ReconstructionNode::PublishReconstruction, this));

        RCLCPP_INFO(get_logger(), "重建节点启动：输入=%s 输出=%s voxel=%.4fm shards=%d 法线估计=%d",
                    input_topic_.c_str(), output_topic_.c_str(), voxel, shards,
                    estimate_normals_ ? 1 : 0);
    }

    void ReconstructionNode::OnCloud(sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::fromROSMsg(*msg, *cloud);
        if (cloud->empty()) { ++frames_dropped_; return; }

        pcl::PointCloud<pcl::Normal>::Ptr normals;
        if (estimate_normals_) {
            normals.reset(new pcl::PointCloud<pcl::Normal>);
            pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
            ne.setInputCloud(cloud);
            pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
            ne.setSearchMethod(tree);
            ne.setKSearch(normal_k_);
            ne.compute(*normals);
            if (normals->size() != cloud->size()) normals.reset();
        }

        Frame f;
        f.T_base_sensor = Mat4::Identity();   // 输入已是 base_link
        f.stamp = rclcpp::Time(msg->header.stamp).seconds();
        f.points.reserve(cloud->size());
        f.normals.reserve(cloud->size());
        f.colors.reserve(cloud->size());
        for (size_t i = 0; i < cloud->size(); ++i) {
            const auto& p = (*cloud)[i];
            if (!pcl::isFinite(p)) continue;
            f.points.emplace_back(p.x, p.y, p.z);
            f.colors.push_back((uint32_t(p.r) << 16) | (uint32_t(p.g) << 8) | p.b);
            if (normals) {
                const auto& n = (*normals)[i];
                Vec3 v(n.normal_x, n.normal_y, n.normal_z);
                f.normals.push_back(v.norm() > 1e-6 ? v.normalized() : Vec3::UnitZ());
            }
        }
        if (f.points.empty()) { ++frames_dropped_; return; }

        map_->Fuse(f);
        ++frames_in_;
        points_in_ += f.points.size();
    }

    void ReconstructionNode::PublishReconstruction()
    {
        auto surfels = map_->Snapshot(static_cast<float>(min_confidence_));

        pcl::PointCloud<pcl::PointXYZRGB> out;
        out.reserve(surfels.size());
        for (const auto& s : surfels) {
            pcl::PointXYZRGB q;
            q.x = static_cast<float>(s.position.x());
            q.y = static_cast<float>(s.position.y());
            q.z = static_cast<float>(s.position.z());
            q.r = (s.color >> 16) & 0xFF;
            q.g = (s.color >> 8) & 0xFF;
            q.b = s.color & 0xFF;
            out.push_back(q);
        }
        out.width = static_cast<uint32_t>(out.size());
        out.height = 1;

        sensor_msgs::msg::PointCloud2 msg;
        pcl::toROSMsg(out, msg);
        msg.header.frame_id = "base_link";
        msg.header.stamp = now();
        pub_->publish(msg);

        if (!save_pcd_.empty() && !out.empty()) {
            pcl::io::savePCDFileBinary(save_pcd_, out);
        }

        RCLCPP_INFO(get_logger(),
            "增量重建：帧=%llu 点=%llu (丢弃=%llu) | 面元 全=%zu 过滤(conf>=%.1f)=%zu",
            static_cast<unsigned long long>(frames_in_),
            static_cast<unsigned long long>(points_in_),
            static_cast<unsigned long long>(frames_dropped_),
            map_->SurfaceCount(), min_confidence_, surfels.size());
    }

}  // namespace RusReconstruction
