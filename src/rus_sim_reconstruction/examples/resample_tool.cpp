// 点云均匀重采样工具：输入 PCD → 打印 原始/重采样 的最近邻间距分布 → 输出 PCD。
//   用法: rus_sim_recon_resample <in.pcd> [out.pcd] [target_spacing=0.004]

#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <string>

#include <pcl/io/pcd_io.h>

#include "rus_sim_reconstruction/cloud_resampler.hpp"

using namespace RusReconstruction;

// 写出带法线的 ASCII PLY（x y z nx ny nz red green blue），供 Poisson 重建用
static void WritePly(const std::string& path, const ResampleResult& r)
{
    std::ofstream os(path);
    os << "ply\nformat ascii 1.0\n";
    os << "element vertex " << r.cloud.size() << "\n";
    os << "property float x\nproperty float y\nproperty float z\n";
    os << "property float nx\nproperty float ny\nproperty float nz\n";
    os << "property uchar red\nproperty uchar green\nproperty uchar blue\n";
    os << "end_header\n";
    for (size_t i = 0; i < r.cloud.size(); ++i) {
        const auto& p = r.cloud[i];
        Eigen::Vector3f n = i < r.normals.size() ? r.normals[i] : Eigen::Vector3f(0, 0, 1);
        os << p.x << ' ' << p.y << ' ' << p.z << ' '
           << n.x() << ' ' << n.y() << ' ' << n.z() << ' '
           << static_cast<int>(p.r) << ' ' << static_cast<int>(p.g) << ' '
           << static_cast<int>(p.b) << '\n';
    }
}

static void PrintStats(const char* tag, const SpacingStats& s)
{
    if (s.n == 0) { std::printf("%s: (空)\n", tag); return; }
    std::printf("%s: n=%zu  最近邻间距[mm]  mean=%.2f median=%.2f p5=%.2f p95=%.2f min=%.2f max=%.2f\n",
                tag, s.n, s.mean * 1000, s.median * 1000, s.p5 * 1000, s.p95 * 1000,
                s.min * 1000, s.max * 1000);
}

int main(int argc, char** argv)
{
    if (argc < 2) {
        std::printf("用法: %s <in.pcd> [out.pcd] [target_spacing=0.004]\n", argv[0]);
        return 1;
    }
    const std::string in_path = argv[1];
    const std::string out_path = argc > 2 ? argv[2] : "/tmp/opencode/resampled.pcd";
    const float spacing = argc > 3 ? std::atof(argv[3]) : 0.004f;

    CloudRGB cloud;
    if (pcl::io::loadPCDFile(in_path, cloud) < 0) {
        std::printf("打不开 %s\n", in_path.c_str());
        return 1;
    }
    std::printf("输入 %s: %zu 点\n", in_path.c_str(), cloud.size());
    PrintStats("  原始", ComputeSpacingStats(cloud));

    ResampleOptions opt;
    opt.voxel_leaf = spacing;
    opt.mls_target_spacing = spacing;
    opt.mls_search_radius = spacing * 3.0f;

    ResampleResult out;
    Resample(cloud, out, opt);
    std::printf("重采样后: %zu 点\n", out.cloud.size());
    PrintStats("  重采样", ComputeSpacingStats(out.cloud));

    if (out_path.size() > 4 && out_path.substr(out_path.size() - 4) == ".ply") {
        WritePly(out_path, out);   // 带法线，供 Poisson 重建
    } else {
        pcl::io::savePCDFileBinary(out_path, out.cloud);
    }
    std::printf("已写出 %s（法线 %zu）\n", out_path.c_str(), out.normals.size());
    return 0;
}
