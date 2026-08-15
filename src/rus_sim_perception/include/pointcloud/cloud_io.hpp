#pragma once

// ════════════════════════════════════════════════════════════════════
//  点云文件读写（pointcloud：点云算法子模块）
//  ────────────────────────────────────────────────────────────────────
//  PCD 文件加载 / 保存（二进制）。供：
//    - load_cloud 指令：加载离线点云 → 发布 /preprocessed_cloud（联调）
//    - 场景保存 / 测试数据生成（后续）
//
//  走非模板 loadPCDFile / savePCDFile(PCLPointCloud2) 路径：
//  模板版在 clang 引擎下 traits::asEnum 有 constexpr 问题
//  （gcc 可编译但 clangd 报错），非模板路径行为一致。
//
//  纯算法实现：仅依赖 PCL / std，无 ROS 类型。
// ════════════════════════════════════════════════════════════════════

#include <string>

#include "components/types.hpp"

namespace RusPerception::PointCloud {

    /**
     * @brief 从 PCD 文件加载点云（支持 PointXYZ / PointXYZRGB 等）
     *
     * @param path PCD 文件路径
     * @param out  输出的彩色点云
     * @return true 加载成功且非空
     */
    bool LoadPcd(const std::string& path, CloudRGB& out);

    /**
     * @brief 保存点云到 PCD（二进制）
     *
     * @param path 目标路径
     * @param cloud 点云
     * @return true 保存成功
     */
    bool SavePcd(const std::string& path, const CloudRGB& cloud);

}  // namespace RusPerception::PointCloud
