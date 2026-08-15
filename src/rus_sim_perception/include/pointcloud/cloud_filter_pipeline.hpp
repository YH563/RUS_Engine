#pragma once

// ════════════════════════════════════════════════════════════════════
//  点云滤波链（pointcloud：点云算法子模块）
//  ────────────────────────────────────────────────────────────────────
//  可插拔滤波流水线，当前阶段：passthrough → statistical → voxel。
//  每阶段输出进出点统计，供健康监控 / 日志使用。
//
//  纯算法实现：仅依赖 PCL / std，无 ROS 类型。
// ════════════════════════════════════════════════════════════════════

#include <string>
#include <vector>

#include "components/types.hpp"

namespace RusPerception::PointCloud {

    /// 滤波链参数（由 perception_params.yaml 注入）
    struct FilterParameter {
        float voxel_leaf_size = 0.003f;       // 体素大小（米）
        std::string passthrough_field = "z";  // 直通字段 x / y / z
        float passthrough_limit_min = -0.5f;  // 直通最小值
        float passthrough_limit_max = 0.5f;   // 直通最大值
        bool passthrough_negative = false;    // 取反（提取范围外）
        int statistical_mean_k = 50;          // 统计滤波邻域点数
        float statistical_std_dev_mul = 1.0f; // 统计滤波标准差倍数
    };

    /// 单个滤波阶段的进出点统计
    struct FilterStageStat {
        std::string stage;  // "passthrough" / "statistical" / "voxel"
        size_t in = 0;
        size_t out = 0;
    };

    /// 点云滤波链
    class CloudFilterPipeline {
    public:
        void SetParameter(const FilterParameter& p) { param_ = p; }
        const FilterParameter& Parameter() const { return param_; }

        /**
         * @brief 应用整条滤波链（cloud 原地覆盖）
         *
         * @param cloud 输入输出点云（原地滤波）
         * @param stats 可选输出：每阶段进出点统计
         * @return true 全部阶段通过且结果非空
         */
        bool Apply(CloudRGB& cloud, std::vector<FilterStageStat>* stats = nullptr);

    private:
        bool passthrough(CloudRGB& cloud);
        bool statistical(CloudRGB& cloud);
        bool voxel(CloudRGB& cloud);

        FilterParameter param_;
    };

}  // namespace RusPerception::PointCloud
