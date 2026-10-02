// 基准：只测 SurfelMap::Fuse 的吞吐（不含场景生成 / I/O）。
//   用法: rus_sim_recon_bench [点数/帧] [帧数]

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <random>
#include <thread>
#include <vector>

#include "rus_sim_reconstruction/sharded_surfel_map.hpp"
#include "rus_sim_reconstruction/surfel_map.hpp"

using namespace RusReconstruction;

namespace {

Frame MakePlaneFrame(size_t target_points, uint32_t seed, double noise)
{
    const int n = static_cast<int>(std::sqrt(static_cast<double>(target_points)));
    const double half = 0.30;
    const double step = 2.0 * half / (n - 1);
    std::mt19937 rng(seed);
    std::normal_distribution<double> nd(0.0, noise);

    Frame f;
    f.points.reserve(static_cast<size_t>(n) * n);
    f.normals.reserve(static_cast<size_t>(n) * n);
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            const double x = -half + i * step;
            const double y = -half + j * step;
            f.points.emplace_back(x, y, nd(rng));     // 平面 z=0 + 噪声
            f.normals.emplace_back(0, 0, 1);
        }
    }
    return f;
}

double BenchOne(double voxel, const Frame& frame, int frames)
{
    SurfelFusionOptions opt;
    opt.voxel_size = voxel;
    SurfelMap map(opt);
    map.Fuse(frame);                                  // 预热

    const auto t0 = std::chrono::steady_clock::now();
    for (int k = 0; k < frames; ++k) map.Fuse(frame);
    const auto t1 = std::chrono::steady_clock::now();

    const double ms = std::chrono::duration<double, std::milli>(t1 - t0).count() / frames;
    const double pts = static_cast<double>(frame.points.size());
    std::printf("  voxel=%.0fmm  融合 %6zu 点/帧 : %7.1f ms/帧  %6.2f M点/s  "
                "最多 %.0f FPS  (面元 %zu)\n",
                voxel * 1000, frame.points.size(), ms, pts / ms / 1000.0,
                1000.0 / ms, map.SurfaceCount());
    return ms;
}

}  // namespace

int main(int argc, char** argv)
{
    const size_t points = argc > 1 ? std::stoul(argv[1]) : 300000;
    const int frames = argc > 2 ? std::stoi(argv[2]) : 20;

    const Frame frame = MakePlaneFrame(points, 7, 0.001);
    std::printf("每帧 %zu 点，计时 %d 帧（单线程 CPU）\n", frame.points.size(), frames);
    for (double v : {0.008, 0.004, 0.002}) BenchOne(v, frame, frames);

    std::printf("\n并行（分片 + 线程池）：\n");
    const int shards = static_cast<int>(std::thread::hardware_concurrency());
    for (double v : {0.008, 0.004, 0.002}) {
        SurfelFusionOptions opt;
        opt.voxel_size = v;
        ShardedSurfelMap map(opt, shards, 0);
        map.Fuse(frame);  // 预热
        const auto t0 = std::chrono::steady_clock::now();
        for (int k = 0; k < frames; ++k) map.Fuse(frame);
        const auto t1 = std::chrono::steady_clock::now();
        const double ms = std::chrono::duration<double, std::milli>(t1 - t0).count() / frames;
        const double pts = static_cast<double>(frame.points.size());
        std::printf("  voxel=%.0fmm  融合 %6zu 点/帧 : %7.1f ms/帧  %6.2f M点/s  "
                    "最多 %.0f FPS  (shards=%d 面元 %zu)\n",
                    v * 1000, frame.points.size(), ms, pts / ms / 1000.0, 1000.0 / ms,
                    shards, map.SurfaceCount());
    }
    return 0;
}
