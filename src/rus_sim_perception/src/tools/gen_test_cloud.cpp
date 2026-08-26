// ════════════════════════════════════════════════════════════════════
//  测试点云生成工具
//  ────────────────────────────────────────────────────────────────────
//  生成模拟超声扫查表面（起伏平面，PointXYZRGB，无障碍）：
//    x ∈ [-0.5, 0]，y ∈ [-0.3, 0.3]，z ∈ [-0.1, 0.1]
//  并随机挑两个点作为起始点（间距保证 > 0.15m），写入起终点文件。
//
//  用法：
// ════════════════════════════════════════════════════════════════════

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <random>
#include <string>

#include "pointcloud/cloud_io.hpp"

int main(int argc, char** argv)
{
    const std::string cloud_path = argc > 1 ? argv[1] : "/tmp/test_cloud.pcd";
    const std::string poses_path = argc > 2 ? argv[2] : "/tmp/test_cloud_poses.txt";

    RusPerception::CloudRGB cloud;

    // 起伏表面（模拟被扫查对象表面），无障碍：
    //   x ∈ [-0.5, 0]，y ∈ [-0.3, 0.3]
    //   z = 起伏函数，范围保持在 [-0.1, 0.1] 内
    for (float x = -0.50f; x <= 0.0f; x += 0.005f) {
        for (float y = -0.30f; y <= 0.30f; y += 0.005f) {
            pcl::PointXYZRGB p;
            p.x = x;
            p.y = y;
            p.z = 0.03f * std::sin(x * 8.0f) + 0.02f * std::cos(y * 6.0f) - 0.02f;
            p.r = static_cast<uint8_t>((x + 0.5f) / 0.5f * 255.0f);
            p.g = static_cast<uint8_t>((y + 0.3f) / 0.6f * 255.0f);
            p.b = 128;
            cloud.push_back(p);
        }
    }

    const bool ok = RusPerception::PointCloud::SavePcd(cloud_path, cloud);
    std::printf("已生成 %s（%zu 点，无障碍，x∈[-0.5,0] y∈[-0.3,0.3] z∈[-0.1,0.1]）\n",
                cloud_path.c_str(), cloud.size());
    if (!ok) return 1;

    // 随机挑两个点作为起始点（保证间距足够，避免路径退化）
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<size_t> dist(0, cloud.size() - 1);
    size_t i_start = 0, i_goal = 0;
    for (int attempt = 0; attempt < 200; ++attempt) {
        i_start = dist(gen);
        i_goal = dist(gen);
        const auto& a = cloud[i_start];
        const auto& b = cloud[i_goal];
        const float dx = a.x - b.x, dy = a.y - b.y, dz = a.z - b.z;
        if (std::sqrt(dx * dx + dy * dy + dz * dz) > 0.15f) break;
    }

    // 起终点文件：第 1 行起点，第 2 行终点（"x y z"）
    std::ofstream poses(poses_path);
    poses << cloud[i_start].x << " " << cloud[i_start].y << " " << cloud[i_start].z << "\n";
    poses << cloud[i_goal].x << " " << cloud[i_goal].y << " " << cloud[i_goal].z << "\n";
    poses.close();
    std::printf("起始点已写入 %s\n", poses_path.c_str());
    std::printf("  START: %.4f %.4f %.4f\n",
                cloud[i_start].x, cloud[i_start].y, cloud[i_start].z);
    std::printf("  GOAL : %.4f %.4f %.4f\n",
                cloud[i_goal].x, cloud[i_goal].y, cloud[i_goal].z);
    return 0;
}
