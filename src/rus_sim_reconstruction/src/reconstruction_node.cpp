#include "rus_sim_reconstruction/reconstruction_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <vector>

#include <zstd.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

#include <rus_sim_utils/protocol.hpp>

namespace RusReconstruction {

    namespace {
        bool ZstdCompress(const std::vector<uint8_t>& in, std::vector<uint8_t>& out)
        {
            if (in.empty()) return false;
            const size_t bound = ZSTD_compressBound(in.size());
            out.resize(bound);
            const size_t cs = ZSTD_compress(out.data(), bound, in.data(), in.size(), 3);
            if (ZSTD_isError(cs)) return false;
            out.resize(cs);
            return true;
        }
    }  // namespace

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

        // ── 面元点云图（前端 /pcmap 通道；点云图 = 融合面元，去噪/带置信度）──
        pcmap_topic_ = declare_parameter<std::string>("pcmap_topic", "/sensor/pcmap");
        pcmap_period_ = declare_parameter<double>("pcmap_period", 1.0);
        pcmap_min_confidence_ = static_cast<float>(
            declare_parameter<double>("pcmap_min_confidence", 2.0));
        pcmap_compress_ = declare_parameter<bool>("pcmap_compress", true);

        // ── 增量网格（/mesh，默认关：前端当前只收面元点云）──
        enable_mesh_ = declare_parameter<bool>("enable_mesh", false);
        mesh_topic_ = declare_parameter<std::string>("mesh_topic", "/sensor/mesh");
        mesh_period_ = declare_parameter<double>("mesh_period", 0.3);
        mesh_full_period_ = declare_parameter<double>("mesh_full_period", 5.0);
        mesh_compress_ = declare_parameter<bool>("mesh_compress", true);
        const std::vector<double> so = declare_parameter<std::vector<double>>(
            "sensor_origin", std::vector<double>{0.0, 0.0, 0.0});
        if (so.size() == 3) sensor_origin_ = Vec3(so[0], so[1], so[2]);

        TsdfVolume::Options topt;
        topt.voxel_size = declare_parameter<double>("tsdf_voxel_size", 0.006);
        topt.truncation = declare_parameter<double>("tsdf_truncation", 0.018);
        tsdf_ = std::make_unique<TsdfVolume>(topt);

        auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
        sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            input_topic_, qos, std::bind(&ReconstructionNode::OnCloud, this, std::placeholders::_1));
        pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, qos);
        timer_ = create_wall_timer(std::chrono::duration<double>(publish_period_),
            std::bind(&ReconstructionNode::PublishReconstruction, this));

        // 面元点云图：SensorFrame（点云 wire 格式，前端复用 /sensor 解码）→ bridge /pcmap 通道
        {
            auto pqos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
            pcmap_pub_ = create_publisher<rus_sim_interfaces::msg::SensorFrame>(pcmap_topic_, pqos);
            pcmap_timer_ = create_wall_timer(std::chrono::duration<double>(pcmap_period_),
                std::bind(&ReconstructionNode::PublishPcMap, this));
        }

        if (enable_mesh_) {
            auto mqos = rclcpp::QoS(rclcpp::KeepLast(8)).reliable();
            mesh_pub_ = create_publisher<rus_sim_interfaces::msg::MeshFrame>(mesh_topic_, mqos);
            mesh_timer_ = create_wall_timer(std::chrono::duration<double>(mesh_period_),
                std::bind(&ReconstructionNode::PublishMesh, this));
        }

        RCLCPP_INFO(get_logger(), "重建节点启动：输入=%s 输出=%s voxel=%.4fm shards=%d 法线估计=%d",
                    input_topic_.c_str(), output_topic_.c_str(), voxel, shards,
                    estimate_normals_ ? 1 : 0);
        RCLCPP_INFO(get_logger(),
                    "面元点云图：话题=%s 周期=%.1fs min_conf=%.1f 压缩=%d（前端 /pcmap）",
                    pcmap_topic_.c_str(), pcmap_period_, pcmap_min_confidence_,
                    pcmap_compress_ ? 1 : 0);
        RCLCPP_INFO(get_logger(),
                    "增量网格：enable=%d 话题=%s tsdf=%.4fm/trunc%.4f 周期=%.2fs 全量=%.1fs 压缩=%d",
                    enable_mesh_ ? 1 : 0, mesh_topic_.c_str(), topt.voxel_size, topt.truncation,
                    mesh_period_, mesh_full_period_, mesh_compress_ ? 1 : 0);
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
            // 用传感器原点定向法线（PCL 法线无向；TSDF 需要朝外的确定方向）
            ne.setViewPoint(static_cast<float>(sensor_origin_.x()),
                            static_cast<float>(sensor_origin_.y()),
                            static_cast<float>(sensor_origin_.z()));
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

        // 面元融合（点云快照）
        map_->Fuse(f);
        // 稀疏 TSDF 积分（增量网格）
        if (enable_mesh_ && tsdf_ && !f.normals.empty()) {
            tsdf_->Integrate(f.points, f.normals, sensor_origin_, 1.0, /*orient_to_origin=*/true);
        }
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

    void ReconstructionNode::PublishPcMap()
    {
        auto surfels = map_->Snapshot(pcmap_min_confidence_);
        if (surfels.empty()) return;

        // 包围盒 + 量化打包（int16 xyz + uint32 rgb，10 字节/点；与感知 /sensor 点云格式一致，
        // 前端可复用同一解码路径）
        double mn[3] = {1e300, 1e300, 1e300};
        double mx[3] = {-1e300, -1e300, -1e300};
        for (const auto& s : surfels) {
            mn[0] = std::min(mn[0], s.position.x()); mx[0] = std::max(mx[0], s.position.x());
            mn[1] = std::min(mn[1], s.position.y()); mx[1] = std::max(mx[1], s.position.y());
            mn[2] = std::min(mn[2], s.position.z()); mx[2] = std::max(mx[2], s.position.z());
        }
        double range[3];
        for (int a = 0; a < 3; ++a) {
            if (mx[a] - mn[a] < 0.001) mx[a] = mn[a] + 0.001;   // 退化范围补 1mm
            range[a] = mx[a] - mn[a];
        }

        auto quant = [](double v, double m, double r) -> int16_t {
            double q = (v - m) * (65535.0 / r) - 32768.0;
            if (q < -32768.0) q = -32768.0;
            if (q > 32767.0) q = 32767.0;
            return static_cast<int16_t>(std::lround(q));
        };

        std::vector<uint8_t> raw;
        raw.reserve(surfels.size() * 10);
        for (const auto& s : surfels) {
            uint8_t b[10];
            const int16_t qx = quant(s.position.x(), mn[0], range[0]);
            const int16_t qy = quant(s.position.y(), mn[1], range[1]);
            const int16_t qz = quant(s.position.z(), mn[2], range[2]);
            const uint32_t c = s.color & 0x00FFFFFFu;
            std::memcpy(b, &qx, 2); std::memcpy(b + 2, &qy, 2);
            std::memcpy(b + 4, &qz, 2); std::memcpy(b + 6, &c, 4);
            raw.insert(raw.end(), b, b + 10);
        }
        std::string encoding = "raw";
        if (pcmap_compress_) {
            std::vector<uint8_t> comp;
            if (ZstdCompress(raw, comp)) { raw.swap(comp); encoding = "zstd"; }
        }

        rus_sim_interfaces::msg::SensorFrame msg;
        msg.type = rus_sim_interfaces::msg::SensorFrame::TYPE_POINTCLOUD;
        msg.encoding = encoding;
        msg.stamp = now();
        msg.seq = pcmap_seq_++;
        msg.frame_id = "base_link";
        msg.scope = std::string(RusUtils::SensorScope::kMap);
        msg.points = static_cast<uint32_t>(surfels.size());
        msg.fields = {"x", "y", "z", "rgb"};
        msg.dtype = "int16";
        msg.range_min = {mn[0], mn[1], mn[2]};
        msg.range_max = {mx[0], mx[1], mx[2]};
        msg.data = std::move(raw);
        pcmap_pub_->publish(msg);

        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
            "面元点云图：%zu 面元 payload=%zu B enc=%s → %s",
            surfels.size(), msg.data.size(), encoding.c_str(), pcmap_topic_.c_str());
    }

    void ReconstructionNode::PublishMesh()
    {
        if (!enable_mesh_ || !tsdf_) return;

        tsdf_->UpdateMesh();
        const double t = now().seconds();
        const bool full = (t - last_mesh_full_) >= mesh_full_period_;
        std::vector<ChunkMesh> chunks = full ? tsdf_->SnapshotChunks() : tsdf_->DrainChunkDeltas();
        if (chunks.empty()) return;
        if (full) last_mesh_full_ = t;

        // 组装协议帧（量化在 protocol.hpp 中，单一来源）
        RusUtils::MeshFrame frame;
        frame.timestamp = t;
        frame.seq = mesh_seq_++;
        frame.frame_id = "base_link";
        frame.scope = full ? "map" : "delta";
        frame.encoding = "raw";
        size_t n_upsert = 0, n_remove = 0;
        for (const auto& c : chunks) {
            RusUtils::MeshChunkMeta m;
            m.id = c.chunk_id;
            if (c.removed) {
                m.kind = RusUtils::MeshChunkMeta::kRemove;
                ++n_remove;
            } else {
                m.kind = RusUtils::MeshChunkMeta::kUpsert;
                m.origin[0] = c.origin.x();
                m.origin[1] = c.origin.y();
                m.origin[2] = c.origin.z();
                m.revision = static_cast<int64_t>(c.revision);
                RusUtils::QuantizeMeshChunkPayload(c.verts, c.normals, m, frame.payload);
                ++n_upsert;
            }
            frame.chunks.push_back(m);
        }
        if (mesh_compress_ && !frame.payload.empty()) {
            std::vector<uint8_t> comp;
            if (ZstdCompress(frame.payload, comp)) { frame.payload.swap(comp); frame.encoding = "zstd"; }
        }

        rus_sim_interfaces::msg::MeshFrame msg;
        msg.stamp = now();
        msg.frame_id = "base_link";
        msg.seq = frame.seq;
        msg.scope = frame.scope;
        msg.encoding = frame.encoding;
        msg.pos_dtype = frame.pos_dtype;
        msg.normal_dtype = frame.normal_dtype;
        msg.chunks.reserve(frame.chunks.size());
        for (const auto& m : frame.chunks) {
            rus_sim_interfaces::msg::MeshChunkMeta rc;
            rc.chunk_id = m.id;
            rc.kind = (m.kind == RusUtils::MeshChunkMeta::kRemove)
                ? rus_sim_interfaces::msg::MeshChunkMeta::KIND_REMOVE
                : rus_sim_interfaces::msg::MeshChunkMeta::KIND_UPSERT;
            rc.origin[0] = m.origin[0];
            rc.origin[1] = m.origin[1];
            rc.origin[2] = m.origin[2];
            rc.revision = m.revision;
            rc.verts = m.verts;
            rc.tris = m.tris;
            rc.range_min[0] = m.range_min[0];
            rc.range_min[1] = m.range_min[1];
            rc.range_min[2] = m.range_min[2];
            rc.range_max[0] = m.range_max[0];
            rc.range_max[1] = m.range_max[1];
            rc.range_max[2] = m.range_max[2];
            rc.has_normals = m.has_normals;
            msg.chunks.push_back(rc);
        }
        msg.data = frame.payload;
        mesh_pub_->publish(msg);

        RCLCPP_INFO(get_logger(),
            "网格增量：seq=%u %s 块=%zu (upsert=%zu remove=%zu) payload=%zu B enc=%s 总块=%zu dirty=%zu",
            frame.seq, frame.scope.c_str(), frame.chunks.size(), n_upsert, n_remove,
            frame.payload.size(), frame.encoding.c_str(), tsdf_->BlockCount(),
            tsdf_->DirtyBlockCount());
    }

}  // namespace RusReconstruction
