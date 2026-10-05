#include "pointcloud/cloud_filter_pipeline.hpp"

#include <algorithm>
#include <cmath>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>

#include "pointcloud/cloud_resampler.hpp"

namespace RusPerception::PointCloud {

    bool CloudFilterPipeline::Apply(CloudRGB& cloud, std::vector<FilterStageStat>* stats)
    {
        if (cloud.empty()) return false;
        if (stats) stats->clear();

        auto run = [&](const std::string& name,
                       bool (CloudFilterPipeline::*fn)(CloudRGB&)) -> bool {
            const size_t in = cloud.size();
            const bool ok = (this->*fn)(cloud);
            if (stats) stats->push_back({name, in, cloud.size()});
            return ok && !cloud.empty();
        };

        // nan_remove 无开关：只剔 NaN/Inf（RealSense 无效深度），
        // 是 planning 建图 KDTree 的保险，不改变有效点的观感
        if (!run("nan_remove", &CloudFilterPipeline::remove_nan)) return false;
        if (param_.enable_passthrough) {
            if (!run("passthrough", &CloudFilterPipeline::passthrough)) return false;
        }
        // 统计滤波（KDTree 近邻开销大）：按参数开关，高频实时处理时建议关闭
        if (param_.enable_statistical) {
            if (!run("statistical", &CloudFilterPipeline::statistical)) return false;
        }
        if (param_.enable_voxel) {
            if (!run("voxel", &CloudFilterPipeline::voxel)) return false;
        }
        // 均匀重采样（链尾）：MLS 平滑 + 网格贪心均匀采样，保证间距 ≥ target
        if (param_.enable_resample) {
            if (!run("resample", &CloudFilterPipeline::resample)) return false;
        }
        return true;
    }

    bool CloudFilterPipeline::remove_nan(CloudRGB& cloud)
    {
        if (cloud.empty()) return false;
        // 清除 NaN / Inf 点：RealSense 深度无效区域会产生 NaN 坐标，
        // 若不剔除会污染后续 KDTree（planning 建图）与体素化。
        cloud.erase(std::remove_if(cloud.begin(), cloud.end(),
            [](const pcl::PointXYZRGB& p) {
                return !std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z);
            }), cloud.end());
        return true;
    }

    bool CloudFilterPipeline::passthrough(CloudRGB& cloud)
    {
        if (cloud.empty()) return false;
        pcl::PassThrough<pcl::PointXYZRGB> pass;
        pass.setInputCloud(cloud.makeShared());
        pass.setFilterFieldName(param_.passthrough_field);
        pass.setFilterLimits(param_.passthrough_limit_min, param_.passthrough_limit_max);
        pass.setNegative(param_.passthrough_negative);
        pass.filter(cloud);
        return true;
    }

    bool CloudFilterPipeline::statistical(CloudRGB& cloud)
    {
        if (cloud.empty()) return false;
        pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
        sor.setInputCloud(cloud.makeShared());
        sor.setMeanK(param_.statistical_mean_k);
        sor.setStddevMulThresh(param_.statistical_std_dev_mul);
        sor.filter(cloud);
        return true;
    }

    bool CloudFilterPipeline::voxel(CloudRGB& cloud)
    {
        if (cloud.empty()) return false;
        // 叶大小 ≤ 0 = 未配置：直接跳过（PCL 内部按 1/leaf_size 建索引，0 会除零出 inf）
        if (param_.voxel_leaf_size <= 0.0f) return true;
        pcl::VoxelGrid<pcl::PointXYZRGB> vg;
        vg.setInputCloud(cloud.makeShared());
        vg.setLeafSize(param_.voxel_leaf_size, param_.voxel_leaf_size, param_.voxel_leaf_size);
        vg.filter(cloud);
        return true;
    }

    bool CloudFilterPipeline::resample(CloudRGB& cloud)
    {
        if (cloud.empty()) return false;
        // 自含重采样链：体素预降密(到 target) → MLS 平滑+法线 → 网格贪心均匀采样。
        // 体素预降密用于**约束 MLS 的计算量**；SOR 不复用（链上已有 statistical 阶段）。
        ResampleOptions ro;
        ro.enable_voxel = true;
        ro.voxel_leaf = param_.resample_target_spacing;
        ro.enable_sor = false;
        ro.enable_mls = true;
        ro.mls_search_radius = param_.resample_mls_radius;
        ro.mls_target_spacing = param_.resample_target_spacing;
        ro.mls_order = param_.resample_mls_order;

        ResampleResult r;
        Resample(cloud, r, ro);
        if (r.cloud.empty()) return false;
        cloud = std::move(r.cloud);   // 位置+颜色；法线在此阶段丢弃（pipeline 输出为 CloudRGB）
        return true;
    }

}  // namespace RusPerception::PointCloud
