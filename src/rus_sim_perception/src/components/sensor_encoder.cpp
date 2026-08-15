#include "components/sensor_encoder.hpp"

#include <algorithm>
#include <cfloat>
#include <cmath>
#include <cstring>

#include <zstd.h>

namespace RusPerception {

    // 每点 10 字节：int16 x/y/z（6）+ uint32 rgb（4）
    static constexpr size_t kPointStride = 10;

    namespace {
        inline int16_t quantize(float v, float mn, float range)
        {
            // v ∈ [mn, mn+range] → [-32768, 32767]
            const float q = std::clamp((v - mn) * (65535.0f / range) - 32768.0f,
                                       -32768.0f, 32767.0f);
            return static_cast<int16_t>(std::lround(q));
        }

        inline float dequantize(int16_t q, float mn, float range)
        {
            return mn + (static_cast<float>(q) + 32768.0f) * (range / 65535.0f);
        }
    }  // namespace

    bool SensorEncoder::EncodePointCloud(const CloudRGB& cloud, EncodedFrame& out)
    {
        if (cloud.empty()) return false;

        // 1. 包围盒
        float mn[3] = {FLT_MAX, FLT_MAX, FLT_MAX};
        float mx[3] = {-FLT_MAX, -FLT_MAX, -FLT_MAX};
        for (const auto& p : cloud) {
            mn[0] = std::min(mn[0], p.x);  mx[0] = std::max(mx[0], p.x);
            mn[1] = std::min(mn[1], p.y);  mx[1] = std::max(mx[1], p.y);
            mn[2] = std::min(mn[2], p.z);  mx[2] = std::max(mx[2], p.z);
        }
        // 退化范围（单点 / 共面）至少 1mm，防除零
        float range[3];
        for (int i = 0; i < 3; ++i) {
            if (mx[i] - mn[i] < 0.001f) mx[i] = mn[i] + 0.001f;
            range[i] = mx[i] - mn[i];
        }

        // 2. 量化打包：int16 xyz + uint32 rgb
        std::vector<uint8_t> raw;
        raw.reserve(cloud.size() * kPointStride);
        for (const auto& p : cloud) {
            uint8_t bytes[kPointStride];
            const int16_t qx = quantize(p.x, mn[0], range[0]);
            const int16_t qy = quantize(p.y, mn[1], range[1]);
            const int16_t qz = quantize(p.z, mn[2], range[2]);
            std::memcpy(bytes, &qx, 2);
            std::memcpy(bytes + 2, &qy, 2);
            std::memcpy(bytes + 4, &qz, 2);
            // rgb 打包为 uint32（r<<16 | g<<8 | b），与 PCL 的 float rgb 位模式一致
            const uint32_t c = (static_cast<uint32_t>(p.r) << 16) |
                               (static_cast<uint32_t>(p.g) << 8) |
                               static_cast<uint32_t>(p.b);
            std::memcpy(bytes + 6, &c, 4);
            raw.insert(raw.end(), bytes, bytes + kPointStride);
        }

        // 3. zstd 压缩
        const size_t bound = ZSTD_compressBound(raw.size());
        out.payload.resize(bound);
        const size_t csize = ZSTD_compress(out.payload.data(), bound,
                                           raw.data(), raw.size(), 3);
        if (ZSTD_isError(csize)) return false;
        out.payload.resize(csize);

        // 4. 元数据
        out.type = "pointcloud";
        out.encoding = "zstd";
        out.points = static_cast<uint32_t>(cloud.size());
        out.fields = {"x", "y", "z", "rgb"};
        out.dtype = "int16";
        for (int i = 0; i < 3; ++i) {
            out.range_min[i] = mn[i];
            out.range_max[i] = mx[i];
        }
        return true;
    }

    bool SensorEncoder::DecodePointCloud(const EncodedFrame& in, CloudRGB& out)
    {
        if (in.encoding != "zstd" || in.payload.empty() || in.points == 0) return false;

        const size_t raw_size = static_cast<size_t>(in.points) * kPointStride;
        std::vector<uint8_t> raw(raw_size);
        const size_t dsize = ZSTD_decompress(raw.data(), raw_size,
                                             in.payload.data(), in.payload.size());
        if (ZSTD_isError(dsize) || dsize != raw_size) return false;

        out.clear();
        out.reserve(in.points);
        const float range[3] = {
            in.range_max[0] - in.range_min[0],
            in.range_max[1] - in.range_min[1],
            in.range_max[2] - in.range_min[2],
        };
        for (uint32_t i = 0; i < in.points; ++i) {
            const uint8_t* p = raw.data() + static_cast<size_t>(i) * kPointStride;
            int16_t qx, qy, qz;
            uint32_t c;
            std::memcpy(&qx, p, 2);
            std::memcpy(&qy, p + 2, 2);
            std::memcpy(&qz, p + 4, 2);
            std::memcpy(&c, p + 6, 4);

            pcl::PointXYZRGB pt;
            pt.x = dequantize(qx, in.range_min[0], range[0]);
            pt.y = dequantize(qy, in.range_min[1], range[1]);
            pt.z = dequantize(qz, in.range_min[2], range[2]);
            float f;
            std::memcpy(&f, &c, 4);
            pt.rgb = f;  // PCL 内部 rgb 字段的位模式即打包后的 uint32
            out.push_back(pt);
        }
        return true;
    }

}  // namespace RusPerception
