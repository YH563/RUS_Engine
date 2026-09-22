#pragma once

// ════════════════════════════════════════════════════════════════════
//  点云滤波链（pointcloud：点云算法子模块）
//  ────────────────────────────────────────────────────────────────────
//  可插拔滤波流水线，链顺序：nan_remove → passthrough → statistical → voxel。
//  每阶段独立开关（除 nan_remove：它是无效点保险，不算"观感滤波"）。
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
        // ── 各阶段开关 ──
        // 效果对照 RealSense 直出（realsense-viewer 单帧画面）时可全部关闭：
        // 此时留存的只有无副作用的 nan_remove。
        // 注意：不要用「叶大小=0 / 直通限值=±1e9」当关闭手段——PCL 体素滤波器内部
        //       按 1/leaf_size 建索引，0 会除零；必须显式置 false。
        bool enable_passthrough = true;       // 直通滤波（base_link 系场景 ROI 裁剪）
        bool enable_statistical = false;      // 统计滤波（KDTree k 近邻开销大，高频处理建议关闭）
        bool enable_voxel = true;             // 体素降采样（降带宽，点变稀）

        // ── 各阶段参数（关闭时忽略）──
        float voxel_leaf_size = 0.005f;       // 体素大小（米）；≤0 视为未配置并跳过
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
        bool remove_nan(CloudRGB& cloud);  // 清除 NaN / Inf 点（RealSense 无效深度）
        bool passthrough(CloudRGB& cloud);
        bool statistical(CloudRGB& cloud);
        bool voxel(CloudRGB& cloud);

        FilterParameter param_;
    };

}  // namespace RusPerception::PointCloud
