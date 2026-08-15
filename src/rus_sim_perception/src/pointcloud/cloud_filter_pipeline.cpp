#include "pointcloud/cloud_filter_pipeline.hpp"

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

        if (!run("passthrough", &CloudFilterPipeline::passthrough)) return false;
        if (!run("statistical", &CloudFilterPipeline::statistical)) return false;
        if (!run("voxel", &CloudFilterPipeline::voxel)) return false;
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
