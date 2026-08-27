#include "pointcloud/cloud_filter_pipeline.hpp"

#include <algorithm>
#include <cmath>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>

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

        if (!run("nan_remove", &CloudFilterPipeline::remove_nan)) return false;
        if (!run("passthrough", &CloudFilterPipeline::passthrough)) return false;
        // 统计滤波（KDTree 近邻开销大）：按参数开关，高频实时处理时建议关闭
        if (param_.enable_statistical) {
            if (!run("statistical", &CloudFilterPipeline::statistical)) return false;
        }
        if (!run("voxel", &CloudFilterPipeline::voxel)) return false;
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
        pcl::VoxelGrid<pcl::PointXYZRGB> vg;
        vg.setInputCloud(cloud.makeShared());
        vg.setLeafSize(param_.voxel_leaf_size, param_.voxel_leaf_size, param_.voxel_leaf_size);
        vg.filter(cloud);
        return true;
    }

}  // namespace RusPerception::PointCloud
