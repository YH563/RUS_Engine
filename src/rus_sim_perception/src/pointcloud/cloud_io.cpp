#include "pointcloud/cloud_io.hpp"

#include <pcl/conversions.h>
#include <pcl/io/pcd_io.h>

namespace RusPerception::PointCloud {

    bool LoadPcd(const std::string& path, CloudRGB& out)
    {
        // 非模板 loadPCDFile(PCLPointCloud2)：避免 clang 下 traits::asEnum constexpr 问题
        pcl::PCLPointCloud2 cloud2;
        if (pcl::io::loadPCDFile(path, cloud2) == -1) return false;
        pcl::fromPCLPointCloud2(cloud2, out);
        return !out.empty();
    }

    bool SavePcd(const std::string& path, const CloudRGB& cloud)
    {
        pcl::PCLPointCloud2 cloud2;
        pcl::toPCLPointCloud2(cloud, cloud2);
        if (pcl::io::savePCDFile(path, cloud2, Eigen::Vector4f::Zero(),
                                 Eigen::Quaternionf::Identity(), true) == -1) {
            return false;
        }
        return true;
    }

}  // namespace RusPerception::PointCloud
