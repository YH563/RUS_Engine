// 离线 demo：把融合出的带法向面元（PLY）做 Poisson 重建 → 三角网格 PLY。
//   用法: rus_sim_recon_mesh <输入面元PLY> <输出网格PLY>
//   例:   rus_sim_recon_mesh /tmp/opencode/recon_surfels.ply /tmp/opencode/recon_mesh.ply

#include <cstdint>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PolygonMesh.h>
#include <pcl/conversions.h>
#include <pcl/surface/poisson.h>

namespace {

struct SurfelPoint { float x, y, z, nx, ny, nz; };

std::vector<SurfelPoint> ReadSurfelPly(const std::string& path)
{
    std::ifstream in(path);
    if (!in) { std::cerr << "打不开 " << path << "\n"; std::exit(1); }
    std::string line;
    std::getline(in, line);              // ply
    std::getline(in, line);              // format
    size_t n = 0;
    std::vector<std::string> props;
    while (std::getline(in, line)) {
        if (line.rfind("element vertex", 0) == 0) n = std::stoul(line.substr(14));
        else if (line.rfind("property", 0) == 0) props.push_back(line.substr(line.find_last_of(' ') + 1));
        else if (line.rfind("end_header", 0) == 0) break;
    }
    std::vector<SurfelPoint> pts;
    pts.reserve(n);
    for (size_t i = 0; i < n; ++i) {
        std::getline(in, line);
        std::istringstream ss(line);
        std::vector<float> v;
        float x;
        while (ss >> x) v.push_back(x);
        SurfelPoint p{};
        for (size_t k = 0; k < props.size() && k < v.size(); ++k) {
            if (props[k] == "x") p.x = v[k];
            else if (props[k] == "y") p.y = v[k];
            else if (props[k] == "z") p.z = v[k];
            else if (props[k] == "nx") p.nx = v[k];
            else if (props[k] == "ny") p.ny = v[k];
            else if (props[k] == "nz") p.nz = v[k];
        }
        pts.push_back(p);
    }
    return pts;
}

void WriteMeshPly(const std::string& path,
                  const pcl::PointCloud<pcl::PointXYZ>& verts,
                  const std::vector<pcl::Vertices>& polys)
{
    std::ofstream os(path);
    os << "ply\nformat ascii 1.0\n";
    os << "element vertex " << verts.size() << "\n";
    os << "property float x\nproperty float y\nproperty float z\n";
    os << "element face " << polys.size() << "\n";
    os << "property list uchar int vertex_indices\n";
    os << "end_header\n";
    for (const auto& p : verts) os << p.x << ' ' << p.y << ' ' << p.z << '\n';
    for (const auto& f : polys) {
        os << f.vertices.size();
        for (auto idx : f.vertices) os << ' ' << idx;
        os << '\n';
    }
}

}  // namespace

int main(int argc, char** argv)
{
    const std::string in_path = argc > 1 ? argv[1] : "/tmp/opencode/recon_surfels.ply";
    const std::string out_path = argc > 2 ? argv[2] : "/tmp/opencode/recon_mesh.ply";

    const auto pts = ReadSurfelPly(in_path);
    std::cout << "输入面元: " << pts.size() << "\n";

    pcl::PointCloud<pcl::PointNormal>::Ptr cloud(new pcl::PointCloud<pcl::PointNormal>);
    cloud->reserve(pts.size());
    for (const auto& p : pts) {
        pcl::PointNormal q;
        q.x = p.x; q.y = p.y; q.z = p.z;
        q.normal_x = p.nx; q.normal_y = p.ny; q.normal_z = p.nz;
        cloud->push_back(q);
    }
    cloud->width = cloud->size();
    cloud->height = 1;
    cloud->is_dense = true;

    pcl::Poisson<pcl::PointNormal> poisson;
    poisson.setDepth(9);          // 八叉树深度（越大越细，成本越高）
    poisson.setSamplesPerNode(2.0f);
    poisson.setInputCloud(cloud);
    pcl::PolygonMesh mesh;
    poisson.reconstruct(mesh);

    pcl::PointCloud<pcl::PointXYZ> verts;
    pcl::fromPCLPointCloud2(mesh.cloud, verts);
    WriteMeshPly(out_path, verts, mesh.polygons);

    std::cout << "输出网格: " << verts.size() << " 顶点, "
              << mesh.polygons.size() << " 三角面 -> " << out_path << "\n";
    return 0;
}
