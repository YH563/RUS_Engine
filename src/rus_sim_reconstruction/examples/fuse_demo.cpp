// 离线 demo：用真实 SurfelMap 融合合成场景（球 + 平面，多视角 + 噪声），
// 导出「原始累积点云」与「融合面元」两份 PLY，供可视化对比。
//   用法: rus_sim_recon_demo [输出目录=/tmp/opencode]

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <random>
#include <string>
#include <vector>

#include "rus_sim_reconstruction/surfel_map.hpp"

using namespace RusReconstruction;

namespace {

constexpr double kPi = 3.14159265358979323846;

struct ScenePoint { Vec3 p; Vec3 n; uint32_t color; };

void AddSphere(std::vector<ScenePoint>& out, const Vec3& c, double r, int nu, int nv)
{
    for (int i = 1; i < nu; ++i) {
        const double theta = kPi * i / nu;
        for (int j = 0; j < nv; ++j) {
            const double phi = 2.0 * kPi * j / nv;
            const Vec3 nrm(std::sin(theta) * std::cos(phi),
                           std::sin(theta) * std::sin(phi),
                           std::cos(theta));
            out.push_back({c + r * nrm, nrm, 0x33CC66});  // 绿
        }
    }
}

void AddPlane(std::vector<ScenePoint>& out, double half, int n)
{
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            const double x = -half + 2.0 * half * j / (n - 1);
            const double y = -half + 2.0 * half * i / (n - 1);
            out.push_back({Vec3(x, y, 0.0), Vec3(0, 0, 1), 0x4488FF});  // 蓝
        }
    }
}

void WritePly(const std::string& path, const std::vector<ScenePoint>& pts, bool with_normals)
{
    std::ofstream os(path);
    os << "ply\nformat ascii 1.0\n";
    os << "element vertex " << pts.size() << "\n";
    os << "property float x\nproperty float y\nproperty float z\n";
    if (with_normals) os << "property float nx\nproperty float ny\nproperty float nz\n";
    os << "property uchar red\nproperty uchar green\nproperty uchar blue\n";
    os << "end_header\n";
    for (const auto& p : pts) {
        os << p.p.x() << ' ' << p.p.y() << ' ' << p.p.z() << ' ';
        if (with_normals) os << p.n.x() << ' ' << p.n.y() << ' ' << p.n.z() << ' ';
        const uint8_t r = (p.color >> 16) & 0xFF, g = (p.color >> 8) & 0xFF, b = p.color & 0xFF;
        os << static_cast<int>(r) << ' ' << static_cast<int>(g) << ' ' << static_cast<int>(b) << '\n';
    }
}

}  // namespace

int main(int argc, char** argv)
{
    const std::string outdir = argc > 1 ? argv[1] : "/tmp/opencode";

    // ── 场景：球 + 地面（贴近 RealSense 密度：~30 万点/帧）──
    std::vector<ScenePoint> scene;
    AddSphere(scene, Vec3(0, 0, 0.20), 0.07, 100, 150);
    AddPlane(scene, 0.16, 300);

    // ── 传感器位姿：绕场景中心环绕一圈 ──
    const Vec3 target(0, 0, 0.12);
    const double orbit_r = 0.45;
    const int K = 6;
    std::vector<Mat4> poses;
    for (int k = 0; k < K; ++k) {
        const double a = 2.0 * kPi * k / K;
        const Vec3 eye(orbit_r * std::cos(a), orbit_r * std::sin(a), 0.32);
        const Vec3 z = (eye - target).normalized();
        const Vec3 up(0, 0, 1);
        const Vec3 x = up.cross(z).normalized();
        const Vec3 y = z.cross(x);
        Mat4 T = Mat4::Identity();
        T.block<3, 3>(0, 0) << x, y, z;
        T.block<3, 1>(0, 3) = eye;
        poses.push_back(T);
    }

    // ── 融合 + 原始累积对比 ──
    std::mt19937 rng(42);
    std::normal_distribution<double> nd(0.0, 0.001);   // 深度噪声（≈ RealSense 近距 1mm）
    SurfelFusionOptions opt;
    opt.voxel_size = 0.004;                            // 4mm 体素（匹配 30 万点/帧的密度）
    opt.max_normal_angle_deg = 30.0;
    SurfelMap map(opt);
    std::vector<ScenePoint> raw;   // 朴素累积（带噪、带重复）

    for (int k = 0; k < K; ++k) {
        const Mat4& T = poses[k];
        const Mat4 Tinv = T.inverse();
        const Vec3 eye = T.block<3, 1>(0, 3);

        Frame f;
        f.T_base_sensor = T;
        f.stamp = k;
        for (const auto& sp : scene) {
            const Vec3 dir = (eye - sp.p).normalized();
            if (sp.n.dot(dir) <= 0.1) continue;   // 背向 / 掠射剔除（可见性近似）
            const Vec3 p_base = sp.p + nd(rng) * sp.n;   // 深度噪声沿表面法线（贴近深度相机）
            const Vec3 n_sensor = Tinv.block<3, 3>(0, 0) * sp.n;
            f.points.push_back(Tinv.block<3, 3>(0, 0) * p_base + Tinv.block<3, 1>(0, 3));
            f.normals.push_back(n_sensor.normalized());
            f.colors.push_back(sp.color);
            raw.push_back({p_base, sp.n, sp.color});
        }
        map.Fuse(f);
    }

    // ── 调试：平面面元的真实置信度分布与 z 方差 ──
    {
        const auto all = map.Snapshot(0.0f);
        int n = 0; double sum = 0, sum2 = 0, csum = 0; float cmin = 1e9f, cmax = 0.f;
        for (const auto& s : all) {
            if (std::abs(s.position.x()) < 0.12 && std::abs(s.position.y()) < 0.12 &&
                std::abs(s.position.z()) < 0.03) {
                sum += s.position.z(); sum2 += s.position.z() * s.position.z();
                csum += s.confidence; cmin = std::min(cmin, s.confidence);
                cmax = std::max(cmax, s.confidence); ++n;
            }
        }
        const double mean = sum / n, var = sum2 / n - mean * mean;
        std::printf("plane: n=%d conf[min=%.0f mean=%.1f max=%.0f] z_sigma=%.3f mm\n",
                    n, cmin, csum / n, cmax, std::sqrt(std::max(0.0, var)) * 1000.0);
        const int edges[] = {1, 2, 6, 21, 61, 100000};
        for (int e = 0; e < 5; ++e) {
            int bn = 0; double bz = 0, bz2 = 0;
            for (const auto& s : all) {
                if (std::abs(s.position.x()) >= 0.12 || std::abs(s.position.y()) >= 0.12 ||
                    std::abs(s.position.z()) >= 0.03) continue;
                if (s.confidence >= edges[e] && s.confidence < edges[e + 1]) {
                    bz += s.position.z(); bz2 += s.position.z() * s.position.z(); ++bn;
                }
            }
            if (bn) {
                const double m = bz / bn, v = bz2 / bn - m * m;
                std::printf("   conf[%d,%d): n=%d z_mean=%.3f z_sigma=%.3f mm\n",
                            edges[e], edges[e + 1], bn, m * 1000, std::sqrt(std::max(0.0, v)) * 1000);
            }
        }
    }

    // ── 导出 ──
    std::vector<ScenePoint> fused;
    for (const auto& s : map.Snapshot(6.0f)) {  // 滤掉低置信边界碎面元
        // 用面元自身颜色（球绿 / 地面蓝）；无颜色时按法线朝向上色
        uint32_t c = s.color;
        if (c == 0) {
            c = (uint32_t(static_cast<uint8_t>(128 + 120 * s.normal.x())) << 16) |
                (uint32_t(static_cast<uint8_t>(128 + 120 * s.normal.y())) << 8) |
                uint32_t(static_cast<uint8_t>(128 + 120 * s.normal.z()));
        }
        fused.push_back({s.position, s.normal, c});
    }

    const std::string raw_path = outdir + "/recon_raw.ply";
    const std::string fused_path = outdir + "/recon_surfels.ply";
    WritePly(raw_path, raw, false);
    WritePly(fused_path, fused, true);

    std::printf("raw    : %zu 点 -> %s\n", raw.size(), raw_path.c_str());
    std::printf("surfel : %zu 面元（全图 %zu，体素 %zu）-> %s\n",
                fused.size(), map.SurfaceCount(), map.VoxelCount(), fused_path.c_str());
    std::printf("压缩比 : %.1fx\n", fused.empty() ? 0.0
                : static_cast<double>(raw.size()) / static_cast<double>(fused.size()));
    return 0;
}
