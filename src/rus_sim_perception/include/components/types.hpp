#pragma once

// ════════════════════════════════════════════════════════════════════
//  感知层公共类型（components：跨点云 / 图像 / 超声复用的基础定义）
//  ────────────────────────────────────────────────────────────────────
//  仅定义类型别名，不包含任何逻辑。
//  依赖：PCL（点云内部表示统一为 PCL 类型，算法层与节点层共用）。
// ════════════════════════════════════════════════════════════════════

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace RusPerception {

    using CloudRGB   = pcl::PointCloud<pcl::PointXYZRGB>;  // 彩色点云（XYZ + RGB）
    using CloudRGBPtr = CloudRGB::Ptr;                      // 点云共享指针

}  // namespace RusPerception
