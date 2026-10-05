// 一次性复杂点云重建：多形状场景(地面+球+立方体+圆柱+凸包) + 噪声/离群
//   → 完整滤波链(NaN/ROR/体素/SOR/MLS/均匀) → 稀疏 TSDF → Marching Tetrahedra 网格。
//   用法: rus_sim_recon_complex <out_dir=/tmp/opencode>
//   产物: complex_raw.ply / complex_filtered.ply / complex_mesh.ply

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <random>
#include <string>
#include <unordered_map>
#include <vector>

#include <pcl/filters/radius_outlier_removal.h>
#include "pointcloud/cloud_resampler.hpp"   // 重采样实现位于 rus_sim_perception
#include "rus_sim_reconstruction/tsdf_volume.hpp"

using namespace RusReconstruction;
using CloudRGB = RusPerception::CloudRGB;   // PCL 彩色点云（perception 定义）
// 重采样（perception）与本文件空间隔离，显式引入
using RusPerception::PointCloud::Resample;
using RusPerception::PointCloud::ResampleOptions;
using RusPerception::PointCloud::ResampleResult;

namespace {
constexpr double kPi = 3.14159265358979323846;
struct P { Vec3 p, n; };

void AddPlane(std::vector<P>& o, double h, int n) {
    for (int i=0;i<n;i++) for(int j=0;j<n;j++){
        double x=-h+2*h*j/(n-1), y=-h+2*h*i/(n-1);
        o.push_back({Vec3(x,y,0), Vec3(0,0,1)}); }
}
void AddSphere(std::vector<P>& o, const Vec3& c, double r, int nu, int nv) {
    for (int i=1;i<nu;i++){ double th=kPi*i/nu;
        for(int j=0;j<nv;j++){ double ph=2*kPi*j/nv;
            Vec3 n(std::sin(th)*std::cos(ph),std::sin(th)*std::sin(ph),std::cos(th));
            o.push_back({c+r*n, n}); } }
}
void AddBox(std::vector<P>& o, const Vec3& c, const Vec3& half, int n) {
    // 6 面，外向法线
    const Vec3 ax[3] = {Vec3(1,0,0),Vec3(0,1,0),Vec3(0,0,1)};
    for (int d=0; d<3; ++d) for (int s=-1; s<=1; s+=2) {
        Vec3 nrm = s*ax[d];
        Vec3 u = ax[(d+1)%3], v = ax[(d+2)%3];
        for (int i=0;i<n;i++) for(int j=0;j<n;j++){
            Vec3 p = c + half[d]*s*ax[d] + half[(d+1)%3]*(-1+2.0*i/(n-1))*u
                       + half[(d+2)%3]*(-1+2.0*j/(n-1))*v;
            o.push_back({p,nrm}); }
    }
}
void AddCylinder(std::vector<P>& o, const Vec3& c, double r, double h, int na, int nh) {
    for (int i=0;i<nh;i++){ double z=-h/2+h*i/(nh-1);
        for(int j=0;j<na;j++){ double a=2*kPi*j/na;
            Vec3 n(std::cos(a),std::sin(a),0);
            o.push_back({Vec3(c.x()+r*n.x(), c.y()+r*n.y(), c.z()+z), n}); } }
    for (int cap=-1; cap<=1; cap+=2)
        for(int i=0;i<na;i++) for(int j=0;j<na;j++){ // 端面圆盘
            double rr=r*std::sqrt((i+0.5)/na), a=2*kPi*j/na;
            Vec3 n(0,0,cap);
            o.push_back({Vec3(c.x()+rr*std::cos(a), c.y()+rr*std::sin(a), c.z()+cap*h/2), n}); }
}
void AddBump(std::vector<P>& o, const Vec3& c, double r, int nu, int nv) { // 半球凸包
    for (int i=0;i<nu;i++){ double th=kPi/2*i/nu;
        for(int j=0;j<nv;j++){ double ph=2*kPi*j/nv;
            Vec3 n(std::sin(th)*std::cos(ph),std::sin(th)*std::sin(ph),std::cos(th));
            o.push_back({c+r*n, n}); } }
}

void WritePlyPoints(const std::string& path, const std::vector<P>& pts, bool normals)
{
    std::ofstream os(path);
    os << "ply\nformat ascii 1.0\nelement vertex " << pts.size() << "\n";
    os << "property float x\nproperty float y\nproperty float z\n";
    if (normals) os << "property float nx\nproperty float ny\nproperty float nz\n";
    os << "end_header\n";
    for (const auto& p : pts) {
        os << p.p.x() << ' ' << p.p.y() << ' ' << p.p.z();
        if (normals) os << ' ' << p.n.x() << ' ' << p.n.y() << ' ' << p.n.z();
        os << '\n';
    }
}

void WritePlyMesh(const std::string& path, const std::vector<float>& V, const std::vector<float>& N)
{
    const size_t nt = V.size()/9;
    std::ofstream os(path);
    os << "ply\nformat ascii 1.0\nelement vertex " << V.size()/3 << "\n";
    os << "property float x\nproperty float y\nproperty float z\n";
    os << "property float nx\nproperty float ny\nproperty float nz\n";
    os << "element face " << nt << "\nproperty list uchar int vertex_indices\nend_header\n";
    for (size_t i=0;i+2<V.size();i+=3)
        os << V[i] << ' ' << V[i+1] << ' ' << V[i+2] << ' '
           << N[i] << ' ' << N[i+1] << ' ' << N[i+2] << '\n';
    for (size_t t=0;t<nt;++t) os << "3 " << t*3 << ' ' << t*3+1 << ' ' << t*3+2 << '\n';
}

// 网格清理：过长边 + 小连通分量
void CleanMesh(std::vector<float>& V, std::vector<float>& N, double max_edge, size_t min_comp)
{
    const size_t nt=V.size()/9; if(!nt) return;
    std::vector<uint8_t> keep(nt,1);
    for(size_t t=0;t<nt;++t){ const float* v=&V[t*9];
        for(int e=0;e<3;++e){ const float* a=v+e*3; const float* b=v+((e+1)%3)*3;
            double dx=a[0]-b[0],dy=a[1]-b[1],dz=a[2]-b[2];
            if(dx*dx+dy*dy+dz*dz>max_edge*max_edge){keep[t]=0;break;} } }
    struct K{int32_t x,y,z;bool operator==(const K&o)const{return x==o.x&&y==o.y&&z==o.z;}};
    struct KH{size_t operator()(const K&k)const{uint64_t h=1469598103934665603ull;
        auto m=[&](int32_t v){h^=(uint32_t)v;h*=1099511628211ull;};m(k.x);m(k.y);m(k.z);return(size_t)h;}};
    std::unordered_map<K,int,KH> vmap; std::vector<int> par(nt); for(size_t i=0;i<nt;++i)par[i]=(int)i;
    auto find=[&](int x){while(par[x]!=x){par[x]=par[par[x]];x=par[x];}return x;};
    auto uni=[&](int a,int b){a=find(a);b=find(b);if(a!=b)par[a]=b;};
    auto key=[&](const float* p){return K{(int32_t)std::llround(p[0]*1e4),(int32_t)std::llround(p[1]*1e4),(int32_t)std::llround(p[2]*1e4)};};
    for(size_t t=0;t<nt;++t){ if(!keep[t])continue; const float* v=&V[t*9];
        for(int e=0;e<3;++e){ K k=key(v+e*3); auto it=vmap.find(k);
            if(it==vmap.end()) vmap.emplace(k,(int)t); else uni((int)t,it->second); } }
    std::unordered_map<int,int> comp; for(size_t t=0;t<nt;++t) if(keep[t]) comp[find((int)t)]++;
    std::vector<float> ov, on;
    for(size_t t=0;t<nt;++t){ if(!keep[t]||comp[find((int)t)]<(int)min_comp)continue;
        for(int i=0;i<9;++i){ov.push_back(V[t*9+i]);on.push_back(N[t*9+i]);} }
    V.swap(ov); N.swap(on);
}
}  // namespace

int main(int argc, char** argv)
{
    const std::string dir = argc>1 ? argv[1] : "/tmp/opencode";

    // ── 复杂场景 ──
    std::vector<P> scene;
    AddPlane(scene, 0.25, 200);                                   // 地面
    AddSphere(scene, Vec3(-0.10,0,0.14), 0.06, 50, 100);          // 球
    AddBox(scene, Vec3(0.14,0.02,0.05), Vec3(0.05,0.04,0.05), 30);// 立方体
    AddCylinder(scene, Vec3(0.02,-0.16,0.05), 0.04, 0.10, 60, 30); // 圆柱
    AddBump(scene, Vec3(0.10,0.14,0), 0.04, 30, 60);              // 地面凸包
    std::printf("场景: %zu 点\n", scene.size());

    // 加噪 + 离群
    std::mt19937 rng(11);
    std::normal_distribution<double> nd(0.0, 0.0015);
    std::uniform_real_distribution<double> uni(-0.3,0.3);
    std::vector<P> noisy; noisy.reserve(scene.size()+2000);
    for (const auto& s : scene) noisy.push_back({s.p + nd(rng)*s.n, s.n});
    for (int k=0;k<2000;++k) noisy.push_back({Vec3(uni(rng),uni(rng),uni(rng)), Vec3(0,0,1)});
    WritePlyPoints(dir+"/complex_raw.ply", noisy, true);
    std::printf("带噪+离群: %zu 点\n", noisy.size());

    // ── 滤波链（体素/SOR/MLS/均匀；NaN 由 PCL 过滤天然处理）──
    CloudRGB work;
    for (const auto& p : noisy){ pcl::PointXYZRGB q; q.x=p.p.x();q.y=p.p.y();q.z=p.p.z();
        q.r=q.g=q.b=180; work.push_back(q); }
    { pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> ror;      // ROR 去离群
      ror.setInputCloud(work.makeShared()); ror.setRadiusSearch(0.012); ror.setMinNeighborsInRadius(3);
      CloudRGB tmp; ror.filter(tmp); work=std::move(tmp); }

    ResampleOptions opt;
    opt.voxel_leaf = 0.005f;
    opt.enable_sor = true; opt.sor_mean_k = 20; opt.sor_std = 3.0f;
    opt.enable_mls = true; opt.mls_search_radius = 0.015f; opt.mls_target_spacing = 0.004f;
    ResampleResult rr; Resample(work, rr, opt);
    std::printf("滤波后: %zu 点\n", rr.cloud.size());

    std::vector<P> filtered;
    filtered.reserve(rr.cloud.size());
    for (size_t i=0;i<rr.cloud.size();++i){
        Vec3 n = i<rr.normals.size()? Vec3(rr.normals[i].x(),rr.normals[i].y(),rr.normals[i].z()) : Vec3(0,0,1);
        if(n.norm()>1e-9) n.normalize();
        filtered.push_back({Vec3(rr.cloud[i].x,rr.cloud[i].y,rr.cloud[i].z), n});
    }
    WritePlyPoints(dir+"/complex_filtered.ply", filtered, true);

    // ── 稀疏 TSDF（一次性整套：信任输入法线）→ 网格 ──
    TsdfVolume::Options topt; topt.voxel_size=0.005; topt.truncation=0.015;
    TsdfVolume vol(topt);
    std::vector<Vec3> P, N; P.reserve(filtered.size()); N.reserve(filtered.size());
    for (const auto& p : filtered){ P.push_back(p.p); N.push_back(p.n); }
    vol.Integrate(P, N, Vec3(0,0,0), 1.0, /*orient_to_origin=*/false);
    vol.UpdateMesh();
    std::printf("TSDF: blocks=%zu tris(原始)=%zu\n", vol.BlockCount(), vol.TriangleCount());

    std::vector<float> V=vol.MeshVertices(), NM=vol.MeshNormals();
    CleanMesh(V, NM, 0.03, 40);
    std::printf("清理后: %zu 三角面\n", V.size()/9);
    WritePlyMesh(dir+"/complex_mesh.ply", V, NM);
    std::printf("写出: complex_raw.ply / complex_filtered.ply / complex_mesh.ply\n");
    return 0;
}
