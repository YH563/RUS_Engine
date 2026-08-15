#include "pointcloud/spatial_transformer.hpp"

#include <memory>

#include <pcl/common/transforms.h>

namespace RusPerception::PointCloud {

    bool SpatialTransformer::Transform(const CloudRGBPtr& in,
                                       const Eigen::Matrix4f& T_base_flange,
                                       CloudRGBPtr& out) const
    {
        if (!in || in->empty()) return false;
        const Eigen::Matrix4f T_base_camera = T_base_flange * camera_to_flange_;
        out = std::make_shared<CloudRGB>();
        pcl::transformPointCloud(*in, *out, T_base_camera);
        return !out->empty();
    }

}  // namespace RusPerception::PointCloud
