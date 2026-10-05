#include "pointcloud/cloud_resampler.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <thread>
#include <unordered_map>
#include <vector>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/surface/mls.h>
#include <pcl/kdtree/kdtree_flann.h>

namespace RusPerception::PointCloud {

    namespace {
        // ── 网格贪心均匀采样：保证任意两点间距 ≥ min_dist（近似 Poisson-disk）──
        struct CellKey {
            int32_t x = 0, y = 0, z = 0;
            bool operator==(const CellKey& o) const { return x == o.x && y == o.y && z == o.z; }
        };
        struct CellKeyHash {
            size_t operator()(const CellKey& k) const {
                uint64_t h = 1469598103934665603ull;
                auto mix = [&h](int32_t v) { h ^= static_cast<uint32_t>(v); h *= 1099511628211ull; };
                mix(k.x); mix(k.y); mix(k.z);
                return static_cast<size_t>(h);
            }
        };

        void UniformFilter(const CloudRGB& in, const std::vector<Eigen::Vector3f>* normals,
                           float min_dist, CloudRGB& out, std::vector<Eigen::Vector3f>& out_normals)
        {
            out.clear();
            out_normals.clear();
            if (in.empty()) return;

            if (min_dist <= 0.f) {
                out = in;
                if (normals) out_normals = *normals;
                return;
            }

            const float cell = min_dist;
            const float md2 = min_dist * min_dist;
            std::unordered_map<CellKey, std::vector<int>, CellKeyHash> grid;
            out.reserve(in.size());
            if (normals) out_normals.reserve(in.size());

            auto cell_of = [cell](const pcl::PointXYZRGB& p) {
                return CellKey{static_cast<int32_t>(std::floor(p.x / cell)),
                               static_cast<int32_t>(std::floor(p.y / cell)),
                               static_cast<int32_t>(std::floor(p.z / cell))};
            };

            for (size_t i = 0; i < in.size(); ++i) {
                const auto& p = in[i];
                if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
                const CellKey c = cell_of(p);
                bool ok = true;
                for (int dx = -1; dx <= 1 && ok; ++dx)
                    for (int dy = -1; dy <= 1 && ok; ++dy)
                        for (int dz = -1; dz <= 1 && ok; ++dz) {
                            auto it = grid.find(CellKey{c.x + dx, c.y + dy, c.z + dz});
                            if (it == grid.end()) continue;
                            for (int idx : it->second) {
                                const auto& q = out[static_cast<size_t>(idx)];
                                const float ddx = q.x - p.x, ddy = q.y - p.y, ddz = q.z - p.z;
                                if (ddx * ddx + ddy * ddy + ddz * ddz < md2) { ok = false; break; }
                            }
                        }
                if (!ok) continue;
                out.push_back(p);
                if (normals) out_normals.push_back((*normals)[i]);
                grid[c].push_back(static_cast<int>(out.size() - 1));
            }
            out.width = static_cast<uint32_t>(out.size());
            out.height = 1;
        }
    }  // namespace

    void Resample(const CloudRGB& in, ResampleResult& out, const ResampleOptions& opt)
    {
        out.cloud.clear();
        out.normals.clear();
        if (in.empty()) return;

        CloudRGB work = in;

        // 1) 粗降密：VoxelGrid（取质心）
        if (opt.enable_voxel && opt.voxel_leaf > 0.f) {
            pcl::VoxelGrid<pcl::PointXYZRGB> vg;
            vg.setInputCloud(work.makeShared());
            vg.setLeafSize(opt.voxel_leaf, opt.voxel_leaf, opt.voxel_leaf);
            CloudRGB tmp;
            vg.filter(tmp);
            work = std::move(tmp);
            if (work.empty()) return;
        }

        // 2) 去离群
        if (opt.enable_sor && opt.sor_mean_k > 0) {
            pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
            sor.setInputCloud(work.makeShared());
            sor.setMeanK(opt.sor_mean_k);
            sor.setStddevMulThresh(opt.sor_std);
            CloudRGB tmp;
            sor.filter(tmp);
            work = std::move(tmp);
            if (work.empty()) return;
        }

        // 3) MLS：投影到局部拟合平面做平滑 + 求法线（不开上采样）
        CloudRGB smooth = work;
        std::vector<Eigen::Vector3f> smooth_normals;
        bool have_normals = false;
        if (opt.enable_mls && opt.mls_search_radius > 0.f) {
            pcl::MovingLeastSquares<pcl::PointXYZRGB, pcl::PointXYZRGBNormal> mls;
            mls.setInputCloud(work.makeShared());
            mls.setSearchRadius(opt.mls_search_radius);
            mls.setPolynomialOrder(opt.mls_order);
            mls.setComputeNormals(true);
            // MLS 默认单线程（threads_=1）；PCL 带 OpenMP，设多线程显著加速
            mls.setNumberOfThreads(std::max(1u, std::thread::hardware_concurrency()));
            mls.setUpsamplingMethod(
                pcl::MovingLeastSquares<pcl::PointXYZRGB, pcl::PointXYZRGBNormal>::NONE);

            pcl::PointCloud<pcl::PointXYZRGBNormal> mls_out;
            mls.process(mls_out);

            smooth.clear();
            smooth_normals.clear();
            smooth.reserve(mls_out.size());
            smooth_normals.reserve(mls_out.size());
            for (const auto& p : mls_out) {
                if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
                pcl::PointXYZRGB q;
                q.x = p.x; q.y = p.y; q.z = p.z; q.r = p.r; q.g = p.g; q.b = p.b;
                smooth.push_back(q);
                Eigen::Vector3f n(p.normal_x, p.normal_y, p.normal_z);
                const float nn = n.norm();
                smooth_normals.push_back(nn > 1e-9f ? n / nn : Eigen::Vector3f(0, 0, 1));
            }
            have_normals = true;
        }

        // 4) 网格贪心均匀采样：保证间距 ≥ target_spacing（VoxelGrid 保证不了）
        UniformFilter(smooth, have_normals ? &smooth_normals : nullptr,
                      opt.mls_target_spacing, out.cloud, out.normals);
    }

    SpacingStats ComputeSpacingStats(const CloudRGB& cloud)
    {
        SpacingStats st;
        st.n = cloud.size();
        if (cloud.size() < 3) return st;

        pcl::KdTreeFLANN<pcl::PointXYZRGB> tree;
        tree.setInputCloud(cloud.makeShared());

        std::vector<float> nn;
        nn.reserve(cloud.size());
        std::vector<int> idx(2);
        std::vector<float> sq(2);
        for (size_t i = 0; i < cloud.size(); ++i) {
            if (tree.nearestKSearch(cloud[i], 2, idx, sq) == 2) {
                const float d = std::sqrt(sq[1]);
                if (d > 0.f) nn.push_back(d);
            }
        }
        if (nn.empty()) return st;
        std::sort(nn.begin(), nn.end());
        auto pct = [&](double p) { return nn[static_cast<size_t>(p * (nn.size() - 1))]; };

        double sum = 0; for (float d : nn) sum += d;
        st.mean = static_cast<float>(sum / nn.size());
        st.median = pct(0.5);
        st.p5 = pct(0.05);
        st.p95 = pct(0.95);
        st.min = nn.front();
        st.max = nn.back();
        return st;
    }

}  // namespace RusPerception::PointCloud
