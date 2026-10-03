// 面元 → 三角网格：多视角面元融合 → 有向点 → GreedyProjectionTriangulation → 网格 PLY。
//   用法: rus_sim_recon_surfel_mesh <out.ply>

#include <cmath>
#include <cstdio>
#include <cstdint>
#include <fstream>
#include <random>
#include <string>
#include <vector>
#include <algorithm>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PolygonMesh.h>
#include <pcl/conversions.h>
#include <pcl/search/kdtree.h>
#include <pcl/surface/gp3.h>

#include "rus_sim_reconstruction/sharded_surfel_map.hpp"

using namespace RusReconstruction;
namespace {
constexpr double kPi = 3.14159265358979323846;
struct SP { Vec3 p, n; };
void AddSphere(std::vector<SP>& o, const Vec3& c, double r, int nu, int nv) {
    for (int i = 1; i < nu; ++i) { double th = kPi*i/nu;
        for (int j = 0; j < nv; ++j) { double ph = 2*kPi*j/nv;
            Vec3 n(std::sin(th)*std::cos(ph), std::sin(th)*std::sin(ph), std::cos(th));
            o.push_back({c + r*n, n}); } }
}
void AddPlane(std::vector<SP>& o, double h, int n) {
    for (int i=0;i<n;i++) for(int j=0;j<n;j++){
        double x=-h+2*h*j/(n-1), y=-h+2*h*i/(n-1);
        o.push_back({Vec3(x,y,0), Vec3(0,0,1)}); }
}
}

int main(int argc, char** argv)
{
    const std::string out = argc > 1 ? argv[1] : "/tmp/opencode/surfel_mesh.ply";

    // 1) 多视角面元融合（面元法）
    std::vector<SP> scene;
    AddSphere(scene, Vec3(0,0,0.20), 0.07, 60, 120);
    AddPlane(scene, 0.16, 120);
    const Vec3 sc(0,0,0.20);
    const double R = 0.40;
    const int NVIEW = 200;

    std::mt19937 rng(7);
    std::normal_distribution<double> nd(0.0, 0.0015);
    SurfelFusionOptions opt; opt.voxel_size = 0.004;
    ShardedSurfelMap map(opt, 12, 0);

    for (int k = 0; k < NVIEW; ++k) {
        const double ct = 1.0 - 2.0*(k+0.5)/NVIEW;
        const double st = std::sqrt(std::max(0.0, 1.0-ct*ct));
        const double phi = k*(kPi*(1.0+std::sqrt(5.0)));
        const Vec3 eye = sc + R*Vec3(st*std::cos(phi), st*std::sin(phi), ct);
        Frame f; f.T_base_sensor = Mat4::Identity(); f.stamp = k;
        for (const auto& sp : scene) {
            const Vec3 dir = (eye - sp.p).normalized();
            if (sp.n.dot(dir) <= 0.1) continue;
            f.points.push_back(sp.p + nd(rng)*sp.n);
            f.normals.push_back(sp.n);
        }
        map.Fuse(f);
    }
    auto surfels = map.Snapshot(2.0f);
    std::printf("面元 %zu\n", surfels.size());

    // 2) 有向点 → 网格（GreedyProjectionTriangulation）
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud(new pcl::PointCloud<pcl::PointNormal>);
    cloud->reserve(surfels.size());
    for (const auto& s : surfels) {
        pcl::PointNormal q;
        q.x = static_cast<float>(s.position.x());
        q.y = static_cast<float>(s.position.y());
        q.z = static_cast<float>(s.position.z());
        q.normal_x = static_cast<float>(s.normal.x());
        q.normal_y = static_cast<float>(s.normal.y());
        q.normal_z = static_cast<float>(s.normal.z());
        cloud->push_back(q);
    }
    cloud->width = static_cast<uint32_t>(cloud->size());
    cloud->height = 1;
    cloud->is_dense = true;

    pcl::search::KdTree<pcl::PointNormal>::Ptr tree(new pcl::search::KdTree<pcl::PointNormal>);
    pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
    gp3.setInputCloud(cloud);
    gp3.setSearchMethod(tree);
    gp3.setSearchRadius(0.012);                 // 邻域半径 ≈ 3×面元间距
    gp3.setMu(2.5);                             // 最大边 = mu × 最近邻距离
    gp3.setMaximumNearestNeighbors(50);
    gp3.setMaximumSurfaceAngle(kPi / 4.0);      // 法线夹角上限
    gp3.setMinimumAngle(kPi / 18.0);
    gp3.setMaximumAngle(2.0 * kPi / 3.0);
    gp3.setNormalConsistency(true);
    pcl::PolygonMesh mesh;
    gp3.reconstruct(mesh);

    pcl::PointCloud<pcl::PointNormal> verts;
    pcl::fromPCLPointCloud2(mesh.cloud, verts);
    std::printf("GP3 网格: %zu 顶点 / %zu 面\n", verts.size(), mesh.polygons.size());

    // 3) 写 PLY（三角形 + 面法线）
    std::ofstream os(out);
    os << "ply\nformat ascii 1.0\n";
    os << "element vertex " << verts.size() << "\n";
    os << "property float x\nproperty float y\nproperty float z\n";
    os << "element face " << mesh.polygons.size() << "\n";
    os << "property list uchar int vertex_indices\nend_header\n";
    for (const auto& p : verts) os << p.x << ' ' << p.y << ' ' << p.z << '\n';
    for (auto poly : mesh.polygons) {
        // 按面元法线统一绕序（GP3 不保证朝向）：面法线·平均顶点法线 < 0 → 翻转
        if (poly.vertices.size() == 3) {
            const auto& a = verts[poly.vertices[0]];
            const auto& b = verts[poly.vertices[1]];
            const auto& c = verts[poly.vertices[2]];
            const float ux = b.x - a.x, uy = b.y - a.y, uz = b.z - a.z;
            const float vx = c.x - a.x, vy = c.y - a.y, vz = c.z - a.z;
            const float fx = uy * vz - uz * vy, fy = uz * vx - ux * vz, fz = ux * vy - uy * vx;
            const float ax = a.normal_x + b.normal_x + c.normal_x;
            const float ay = a.normal_y + b.normal_y + c.normal_y;
            const float az = a.normal_z + b.normal_z + c.normal_z;
            if (fx * ax + fy * ay + fz * az < 0.f) std::swap(poly.vertices[1], poly.vertices[2]);
        }
        os << poly.vertices.size();
        for (auto i : poly.vertices) os << ' ' << i;
        os << '\n';
    }
    std::printf("已写出 %s\n", out.c_str());
    return 0;
}
