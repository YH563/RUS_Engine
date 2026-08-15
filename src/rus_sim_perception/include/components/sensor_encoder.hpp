#pragma once

// ════════════════════════════════════════════════════════════════════
//  传感器帧编码器（components：通用压缩编码）
//  ────────────────────────────────────────────────────────────────────
//  把感知数据编码为"压缩 payload + 元数据"，供面向前端的 /sensor 通路
//  （ROS 侧 SensorFrame.msg → bridge 转发 → 前端解码）。
//  本期实现点云路径：坐标量化（float32 → int16，包围盒反量化）+ zstd；
//  图像 / 超声路径预留（将来复用同一接口，只加编码函数）。
//
//  纯算法实现：无 ROS 类型，可独立单测（编码 → 解码回环）。
// ════════════════════════════════════════════════════════════════════

#include <cstdint>
#include <string>
#include <vector>

#include "components/types.hpp"

namespace RusPerception {

    /// 编码后的传感器帧（通用结构，type 区分数据类别）
    struct EncodedFrame {
        std::string type;                  // "pointcloud"（见 SensorType 约定）
        std::string encoding;              // "zstd" / "jpeg" / "png" / "raw"
        double timestamp = 0.0;            // 采集时间戳（秒）
        uint32_t seq = 0;                  // 每类型独立递增（前端检测丢帧）
        uint32_t points = 0;               // 点数（pointcloud）

        // 点云分量顺序与量化包围盒（反量化必需）
        std::vector<std::string> fields;   // 如 {"x","y","z","rgb"}
        std::string dtype;                 // "int16"
        float range_min[3] = {0.0f, 0.0f, 0.0f};
        float range_max[3] = {0.0f, 0.0f, 0.0f};

        // 图像 / 超声元数据（预留）
        uint32_t width = 0;
        uint32_t height = 0;
        std::string image_encoding;        // 压缩前格式 rgb8 / mono8
        uint32_t step = 0;

        std::vector<uint8_t> payload;      // 压缩后字节
    };

    /// 传感器帧编码器（点云：坐标量化 + zstd）
    class SensorEncoder {
    public:
        /**
         * @brief 点云 → 编码帧
         *
         * 每个点 8 字节布局：int16 x/y/z（包围盒量化）+ uint8 r/g/b + 1 字节填充，
         * 整体经 zstd 压缩。
         *
         * @param cloud 输入点云（XYZ+RGB）
         * @param out   输出编码帧（payload 已压缩）
         * @return true 成功；false 输入为空
         */
        bool EncodePointCloud(const CloudRGB& cloud, EncodedFrame& out);

        /**
         * @brief 解码回点云（编码/解码回环测试用）
         *
         * @param in  编码帧（encoding 须为 "zstd"）
         * @param out 输出的彩色点云
         * @return true 成功
         */
        static bool DecodePointCloud(const EncodedFrame& in, CloudRGB& out);
    };

}  // namespace RusPerception
