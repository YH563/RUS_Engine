#include "pointcloud/map_manager.hpp"

#include <memory>
#include <utility>

#include <pcl/filters/voxel_grid.h>

namespace RusPerception::PointCloud {

    void MapManager::AddFrame(const CloudRGBPtr& frame)
    {
        if (!frame || frame->empty()) return;
        map_ += *frame;
        ++frame_count_;
        if (max_points_ > 0 && map_.size() > max_points_) voxelize();
    }

    void MapManager::Compact()
    {
        if (!map_.empty()) voxelize();
    }

    void MapManager::Clear()
    {
        map_.clear();
        frame_count_ = 0;
    }

    CloudRGBPtr MapManager::Snapshot() const
    {
        auto out = std::make_shared<CloudRGB>();
        *out = map_;
        return out;
    }

    void MapManager::voxelize()
    {
        if (voxel_leaf_size_ <= 0.0f || map_.empty()) return;
        pcl::VoxelGrid<pcl::PointXYZRGB> vg;
        vg.setInputCloud(map_.makeShared());
        vg.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
        CloudRGB filtered;
        vg.filter(filtered);
        map_ = std::move(filtered);
    }

}  // namespace RusPerception::PointCloud
