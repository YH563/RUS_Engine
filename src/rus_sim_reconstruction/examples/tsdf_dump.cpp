// 离线：跑一遍 TSDF 增量重建，把网格导出为带索引的 PLY（供拓扑分析）。
//   用法: rus_sim_recon_tsdf_dump <out.ply>

#include <cmath>
#include <cstdio>
#include <cstdint>
#include <fstream>
#include <random>
#include <string>
#include <unordered_map>
#include <vector>

#include "rus_sim_reconstruction/tsdf_volume.hpp"

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
struct VKey { int32_t x,y,z; bool operator==(const VKey&o)const{return x==o.x&&y==o.y&&z==o.z;} };
struct VKeyH { size_t operator()(const VKey&k)const{ uint64_t h=1469598103934665603ull;
    auto m=[&](int32_t v){h^=(uint32_t)v;h*=1099511628211ull;}; m(k.x);m(k.y);m(k.z); return (size_t)h; } };
}

int main(int argc, char** argv)
{
    const std::string out = argc > 1 ? argv[1] : "/tmp/opencode/tsdf_mesh_indexed.ply";

    std::vector<SP> scene;
    AddSphere(scene, Vec3(0,0,0.20), 0.07, 60, 120);
    AddPlane(scene, 0.16, 80);
    const Vec3 sc(0,0,0.20);
    const double R = 0.40;
    const int NVIEW = 200;

    std::mt19937 rng(7);
    std::normal_distribution<double> nd(0.0, 0.0008);
    TsdfVolume::Options opt; opt.voxel_size = 0.006; opt.truncation = 0.012;
    TsdfVolume vol(opt);

    for (int k = 0; k < NVIEW; ++k) {
        const double ct = 1.0 - 2.0*(k+0.5)/NVIEW;
        const double st = std::sqrt(std::max(0.0, 1.0-ct*ct));
        const double phi = k*(kPi*(1.0+std::sqrt(5.0)));
        const Vec3 eye = sc + R*Vec3(st*std::cos(phi), st*std::sin(phi), ct);
        std::vector<Vec3> p, n;
        for (const auto& sp : scene) {
            const Vec3 dir = (eye - sp.p).normalized();
            if (sp.n.dot(dir) <= 0.1) continue;
            p.push_back(sp.p + nd(rng)*sp.n);
            n.push_back(sp.n);
        }
        vol.Integrate(p, n, eye);
    }
    vol.UpdateMesh();
    std::printf("blocks=%zu tris(soup)=%zu\n", vol.BlockCount(), vol.TriangleCount());

    // 三角汤 → 带去重的索引网格（按量化位置合并顶点，便于拓扑分析）
    const auto& V = vol.MeshVertices();
    const double q = 1e-5;
    std::unordered_map<VKey, int, VKeyH> vmap;
    std::vector<float> verts;
    std::vector<uint32_t> faces;
    auto vid = [&](float x,float y,float z)->uint32_t {
        VKey k{ static_cast<int32_t>(std::llround(x/q)),
                static_cast<int32_t>(std::llround(y/q)),
                static_cast<int32_t>(std::llround(z/q)) };
        auto it = vmap.find(k);
        if (it != vmap.end()) return static_cast<uint32_t>(it->second);
        int id = static_cast<int>(verts.size()/3);
        verts.push_back(x); verts.push_back(y); verts.push_back(z);
        vmap.emplace(k, id);
        return static_cast<uint32_t>(id);
    };
    for (size_t t = 0; t + 8 < V.size(); t += 9) {
        faces.push_back(vid(V[t],   V[t+1], V[t+2]));
        faces.push_back(vid(V[t+3], V[t+4], V[t+5]));
        faces.push_back(vid(V[t+6], V[t+7], V[t+8]));
    }

    std::ofstream os(out);
    os << "ply\nformat ascii 1.0\n";
    os << "element vertex " << verts.size()/3 << "\n";
    os << "property float x\nproperty float y\nproperty float z\n";
    os << "element face " << faces.size()/3 << "\n";
    os << "property list uchar int vertex_indices\nend_header\n";
    for (size_t i = 0; i + 2 < verts.size(); i += 3)
        os << verts[i] << ' ' << verts[i+1] << ' ' << verts[i+2] << '\n';
    for (size_t i = 0; i + 2 < faces.size(); i += 3)
        os << "3 " << faces[i] << ' ' << faces[i+1] << ' ' << faces[i+2] << '\n';
    std::printf("导出: %zu 顶点 / %zu 三角面 -> %s\n", verts.size()/3, faces.size()/3, out.c_str());
    return 0;
}
