// 精简版 SurfelMeshing（参照 Schöps et al., arXiv:1810.00729）：
//   面元增量融合 → 面元平滑 → 局部（切平面扇形）三角化 + 一致性 → Laplacian 平滑 → 网格。
//   全程无体素、无泊松；网格周期性异步重建并实时渲染。
//   操作：左键拖动=旋转，滚轮=缩放，M=网格/面元，S=网格/面元叠加
//   用法: rus_sim_recon_live_surfel_mesh

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <cstdio>
#include <mutex>
#include <random>
#include <thread>
#include <unordered_map>
#include <vector>

#include <GL/glew.h>
#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>

#include "rus_sim_reconstruction/sharded_surfel_map.hpp"

using namespace RusReconstruction;

namespace {
constexpr double kPi = 3.14159265358979323846;

struct SP { Vec3 p, n; uint32_t col; };
void AddSphere(std::vector<SP>& o, const Vec3& c, double r, int nu, int nv) {
    for (int i = 1; i < nu; ++i) { double th = kPi*i/nu;
        for (int j = 0; j < nv; ++j) { double ph = 2*kPi*j/nv;
            Vec3 n(std::sin(th)*std::cos(ph), std::sin(th)*std::sin(ph), std::cos(th));
            o.push_back({c + r*n, n, 0x33CC66}); } }
}
void AddPlane(std::vector<SP>& o, double h, int n) {
    for (int i=0;i<n;i++) for(int j=0;j<n;j++){
        double x=-h+2*h*j/(n-1), y=-h+2*h*i/(n-1);
        o.push_back({Vec3(x,y,0), Vec3(0,0,1), 0x4488FF}); }
}

// ── 面元网格（带索引 + 顶点法线）──
struct Mesh {
    std::vector<float> verts;    // 3 per vertex
    std::vector<float> normals;  // 3 per vertex
    std::vector<uint32_t> tris;  // 3 per triangle
    void clear() { verts.clear(); normals.clear(); tris.clear(); }
};
Mesh g_mesh;
std::mutex g_mesh_mtx;
std::atomic<double> g_build_ms{0.0};
std::atomic<size_t> g_nsurfel{0};
std::atomic<size_t> g_tri_count{0};

// 后台网格重建
std::mutex g_in_mtx;
std::condition_variable g_cv;
std::vector<Surfel> g_pending;
bool g_has_pending = false;
bool g_stop = false;
double g_voxel = 0.004;

// ── 邻域网格哈希 ──
struct CK { int32_t x,y,z; bool operator==(const CK&o)const{return x==o.x&&y==o.y&&z==o.z;} };
struct CKH { size_t operator()(const CK&k)const{ uint64_t h=1469598103934665603ull;
    auto m=[&](int32_t v){h^=(uint32_t)v;h*=1099511628211ull;}; m(k.x);m(k.y);m(k.z); return (size_t)h; } };
CK cell_of(const Vec3& p, double c) {
    return CK{static_cast<int32_t>(std::floor(p.x()/c)),
              static_cast<int32_t>(std::floor(p.y()/c)),
              static_cast<int32_t>(std::floor(p.z()/c))};
}

// ── 面元平滑（位置/法线邻居平均）──
void SmoothSurfels(std::vector<Vec3>& P, std::vector<Vec3>& N, double radius, int iters)
{
    const int n = static_cast<int>(P.size());
    for (int it = 0; it < iters; ++it) {
        std::unordered_map<CK, std::vector<int>, CKH> grid;
        for (int i = 0; i < n; ++i) grid[cell_of(P[i], radius)].push_back(i);
        std::vector<Vec3> nP = P, nN = N;
        for (int i = 0; i < n; ++i) {
            const CK c = cell_of(P[i], radius);
            Vec3 sump = P[i], sumn = N[i]; int cnt = 1;
            for (int dx=-1;dx<=1;++dx) for (int dy=-1;dy<=1;++dy) for (int dz=-1;dz<=1;++dz) {
                auto it2 = grid.find(CK{c.x+dx,c.y+dy,c.z+dz});
                if (it2==grid.end()) continue;
                for (int j : it2->second) {
                    if (j==i) continue;
                    if ((P[j]-P[i]).squaredNorm() <= radius*radius) { sump+=P[j]; sumn+=N[j]; ++cnt; }
                }
            }
            nP[i] = sump / cnt;
            if (sumn.norm() > 1e-9) nN[i] = sumn.normalized();
        }
        P.swap(nP); N.swap(nN);
    }
}

// ── 局部切平面扇形三角化 + 一致性 ──
void Triangulate(const std::vector<Vec3>& P, const std::vector<Vec3>& N, double radius,
                 double max_edge, double cos_normal, Mesh& mesh)
{
    const int n = static_cast<int>(P.size());
    std::unordered_map<CK, std::vector<int>, CKH> grid;
    for (int i = 0; i < n; ++i) grid[cell_of(P[i], radius)].push_back(i);

    std::unordered_map<uint64_t, uint32_t> tri_set;  // 排序后的顶点键 → 占位
    mesh.clear();
    mesh.verts.reserve(n * 3);
    mesh.tris.reserve(n * 2 * 3);

    auto vid = [&](int i) { return static_cast<uint32_t>(i); };
    auto emit = [&](uint64_t key, uint32_t a, uint32_t b, uint32_t c) {
        if (tri_set.count(key)) return;
        tri_set[key] = static_cast<uint32_t>(mesh.tris.size() / 3);
        mesh.tris.push_back(a); mesh.tris.push_back(b); mesh.tris.push_back(c);
    };

    for (int i = 0; i < n; ++i) {
        const CK c = cell_of(P[i], radius);
        std::vector<int> nb;
        for (int dx=-1;dx<=1;++dx) for (int dy=-1;dy<=1;++dy) for (int dz=-1;dz<=1;++dz) {
            auto it = grid.find(CK{c.x+dx,c.y+dy,c.z+dz});
            if (it==grid.end()) continue;
            for (int j : it->second) if (j!=i && (P[j]-P[i]).squaredNorm() <= radius*radius) nb.push_back(j);
        }
        if (nb.size() < 3) continue;
        // 只保留最近的 K 个邻居（去掉远处点，避免外接圆测试被远处点否决）
        std::sort(nb.begin(), nb.end(), [&](int a, int b) {
            return (P[a]-P[i]).squaredNorm() < (P[b]-P[i]).squaredNorm(); });
        if (nb.size() > 16) nb.resize(16);

        // 切平面基
        Vec3 z = N[i];
        Vec3 x0 = std::abs(z.z()) < 0.9 ? Vec3(0,0,1) : Vec3(1,0,0);
        Vec3 x = x0.cross(z).normalized();
        Vec3 y = z.cross(x);
        struct Item { double ang, px, py; int idx; };
        std::vector<Item> items;
        items.reserve(nb.size());
        for (int j : nb) {
            if (N[j].dot(z) < cos_normal) continue;              // 法线一致
            const Vec3 d = P[j] - P[i];
            if (d.squaredNorm() > max_edge*max_edge) continue;
            const double px = d.dot(x), py = d.dot(y);
            items.push_back({std::atan2(py, px), px, py, j});
        }
        if (items.size() < 3) continue;
        std::sort(items.begin(), items.end(), [](const Item&a,const Item&b){return a.ang<b.ang;});

        // 切平面内 2D Delaunay：三角形 (0,0),(ax,ay),(bx,by) 的外接圆内无其它邻居
        auto empty_circle = [&](double ax, double ay, double bx, double by) -> bool {
            const double det = 2.0 * (ax*by - ay*bx);
            if (std::abs(det) < 1e-12) return false;
            const double a2 = ax*ax + ay*ay, b2 = bx*bx + by*by;
            const double cx = (by*a2 - ay*b2) / det;
            const double cy = (ax*b2 - bx*a2) / det;
            const double r2 = cx*cx + cy*cy;
            for (const auto& q : items) {
                const double dx = q.px - cx, dy = q.py - cy;
                if (dx*dx + dy*dy < r2 - 1e-9) return false;
            }
            return true;
        };

        const double max_gap = 100.0 * kPi / 180.0;
        for (size_t k = 0; k < items.size(); ++k) {
            const Item& A = items[k];
            const Item& B = items[(k+1) % items.size()];
            double gap = B.ang - A.ang;
            if (k+1 == items.size()) gap += 2*kPi;
            if (gap > max_gap) continue;
            if (N[A.idx].dot(N[B.idx]) < cos_normal) continue;
            const Vec3 dab = P[B.idx]-P[A.idx];
            if (dab.squaredNorm() > max_edge*max_edge) continue;
            if (!empty_circle(A.px, A.py, B.px, B.py)) continue;   // Delaunay → 只留外接圆空的三角形
            // 绕序：面法线与 z(=N[i]) 一致
            uint32_t a = vid(i), b = vid(A.idx), cc = vid(B.idx);
            uint32_t s[3] = {a, b, cc};
            std::sort(s, s + 3);
            uint64_t key = (uint64_t)s[0]*100000ull*100000ull + (uint64_t)s[1]*100000ull + s[2];
            emit(key, a, b, cc);
        }
    }
}

void LaplacianSmooth(Mesh& mesh, int iters, double lambda)
{
    if (mesh.verts.empty()) return;
    const size_t nv = mesh.verts.size()/3;
    std::vector<std::vector<uint32_t>> adj(nv);
    for (size_t t = 0; t+2 < mesh.tris.size(); t += 3) {
        uint32_t a=mesh.tris[t], b=mesh.tris[t+1], c=mesh.tris[t+2];
        adj[a].push_back(b); adj[a].push_back(c);
        adj[b].push_back(a); adj[b].push_back(c);
        adj[c].push_back(a); adj[c].push_back(b);
    }
    std::vector<float> nv3 = mesh.verts;
    for (int it=0; it<iters; ++it) {
        for (size_t i=0;i<nv;++i) {
            if (adj[i].empty()) continue;
            float sx=0,sy=0,sz=0;
            for (uint32_t j : adj[i]) { sx+=nv3[j*3]; sy+=nv3[j*3+1]; sz+=nv3[j*3+2]; }
            const float k = static_cast<float>(lambda / adj[i].size());
            mesh.verts[i*3]   = nv3[i*3]   * (1-lambda) + sx * k;
            mesh.verts[i*3+1] = nv3[i*3+1] * (1-lambda) + sy * k;
            mesh.verts[i*3+2] = nv3[i*3+2] * (1-lambda) + sz * k;
        }
        nv3 = mesh.verts;
    }
}

void ComputeVertexNormals(Mesh& mesh)
{
    mesh.normals.assign(mesh.verts.size(), 0.f);
    for (size_t t = 0; t+2 < mesh.tris.size(); t += 3) {
        const uint32_t ia=mesh.tris[t], ib=mesh.tris[t+1], ic=mesh.tris[t+2];
        const float* a=&mesh.verts[ia*3]; const float* b=&mesh.verts[ib*3]; const float* c=&mesh.verts[ic*3];
        float ux=b[0]-a[0],uy=b[1]-a[1],uz=b[2]-a[2];
        float vx=c[0]-a[0],vy=c[1]-a[1],vz=c[2]-a[2];
        float nx=uy*vz-uz*vy, ny=uz*vx-ux*vz, nz=ux*vy-uy*vx;
        for (uint32_t ii : {ia,ib,ic}) { mesh.normals[ii*3]+=nx; mesh.normals[ii*3+1]+=ny; mesh.normals[ii*3+2]+=nz; }
    }
    for (size_t i=0;i<mesh.normals.size(); i+=3) {
        float x=mesh.normals[i],y=mesh.normals[i+1],z=mesh.normals[i+2];
        float l=std::sqrt(x*x+y*y+z*z); if (l>1e-9f){ mesh.normals[i]=x/l; mesh.normals[i+1]=y/l; mesh.normals[i+2]=z/l; }
    }
}

Mesh BuildMeshFromSurfels(const std::vector<Surfel>& surfels, double voxel)
{
    Mesh mesh;
    const int n = static_cast<int>(surfels.size());
    if (n < 8) return mesh;
    std::vector<Vec3> P(n), N(n);
    for (int i=0;i<n;++i){ P[i]=surfels[i].position; N[i]=surfels[i].normal; }
    const double radius = voxel*2.5;
    SmoothSurfels(P, N, radius, 2);                       // 平滑面元（SurfelMeshing 关键）
    Triangulate(P, N, radius, voxel*3.5, std::cos(45*kPi/180), mesh);
    // 顶点即用平滑后的面元位置（索引 = 面元索引）→ 直接铺顶点
    mesh.verts.assign(n*3, 0.f);
    for (int i=0;i<n;++i){ mesh.verts[i*3]=P[i].x(); mesh.verts[i*3+1]=P[i].y(); mesh.verts[i*3+2]=P[i].z(); }
    LaplacianSmooth(mesh, 3, 0.5);
    ComputeVertexNormals(mesh);
    return mesh;
}

void MeshWorker()
{
    for (;;) {
        std::vector<Surfel> in;
        {
            std::unique_lock<std::mutex> lk(g_in_mtx);
            g_cv.wait(lk, [] { return g_has_pending || g_stop; });
            if (g_stop) return;
            in = std::move(g_pending);
            g_has_pending = false;
        }
        const auto t0 = std::chrono::steady_clock::now();
        Mesh m = BuildMeshFromSurfels(in, g_voxel);
        {
            std::lock_guard<std::mutex> lk(g_mesh_mtx);
            g_mesh = std::move(m);
            g_tri_count = g_mesh.tris.size() / 3;
        }
        g_build_ms = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - t0).count();
    }
}

struct Camera { float yaw=30.f, pitch=25.f, dist=0.9f; } g_cam;
bool g_drag=false; double g_lx=0,g_ly=0; bool g_show_mesh=false;   // 默认面元圆盘 splatting（快、实心）
}
static void OnMouse(GLFWwindow* w,int b,int a,int){ if(b!=GLFW_MOUSE_BUTTON_LEFT)return; if(a==GLFW_PRESS){g_drag=true;glfwGetCursorPos(w,&g_lx,&g_ly);}else g_drag=false; }
static void OnCursor(GLFWwindow*,double x,double y){ if(!g_drag)return; g_cam.yaw+=float((x-g_lx)*0.4); g_cam.pitch=std::max(-89.f,std::min(89.f,g_cam.pitch+float((y-g_ly)*0.4))); g_lx=x;g_ly=y; }
static void OnScroll(GLFWwindow*,double,double dy){ g_cam.dist=std::max(0.15f,std::min(3.f,g_cam.dist*float(std::pow(0.9,dy)))); }
static void OnKey(GLFWwindow*,int key,int,int action,int){ if(key==GLFW_KEY_M&&action==GLFW_PRESS) g_show_mesh=!g_show_mesh; }

int main()
{
    if(!glfwInit()){std::fprintf(stderr,"glfwInit 失败\n");return 1;}
    GLFWwindow* win=glfwCreateWindow(1000,720,"RUS SurfelMeshing-lite (drag=rotate, wheel=zoom, M=mesh/surfels)",nullptr,nullptr);
    glfwMakeContextCurrent(win); glfwSwapInterval(1);
    glewExperimental=GL_TRUE; if(glewInit()!=GLEW_OK){return 1;}
    glfwSetMouseButtonCallback(win,OnMouse); glfwSetCursorPosCallback(win,OnCursor);
    glfwSetScrollCallback(win,OnScroll); glfwSetKeyCallback(win,OnKey);
    glDisable(GL_CULL_FACE); glLightModeli(GL_LIGHT_MODEL_TWO_SIDE,1); glEnable(GL_DEPTH_TEST);

    std::vector<SP> scene;
    AddSphere(scene,Vec3(0,0,0.20),0.07,60,120);
    AddPlane(scene,0.16,120);
    const Vec3 sc(0,0,0.20); const double R=0.40; const int NVIEW=200;
    std::mt19937 rng(7); std::normal_distribution<double> nd(0.0,0.0015);

    SurfelFusionOptions opt; opt.voxel_size=0.004;
    g_voxel = opt.voxel_size;
    ShardedSurfelMap map(opt,12,0);
    std::vector<Surfel> shown;

    int frame_no=0; double last_fuse=0,last_mesh=0; const double fuse_dt=0.05, mesh_dt=0.6;
    std::thread worker(MeshWorker);

    while(!glfwWindowShouldClose(win)){
        glfwPollEvents(); if(glfwWindowShouldClose(win))break;
        const double now=glfwGetTime();

        if(now-last_fuse>=fuse_dt){
            last_fuse=now;
            const int kk=frame_no%NVIEW;
            const double ct=1.0-2.0*(kk+0.5)/NVIEW;
            const double st=std::sqrt(std::max(0.0,1.0-ct*ct));
            const double phi=kk*(kPi*(1.0+std::sqrt(5.0)));
            const Vec3 eye=sc+R*Vec3(st*std::cos(phi),st*std::sin(phi),ct);
            Frame f; f.T_base_sensor=Mat4::Identity(); f.stamp=frame_no;
            for(const auto& sp:scene){ const Vec3 d=(eye-sp.p).normalized(); if(sp.n.dot(d)<=0.1)continue;
                f.points.push_back(sp.p+nd(rng)*sp.n); f.normals.push_back(sp.n); f.colors.push_back(sp.col); }
            map.Fuse(f); ++frame_no;
        }

        if(now-last_mesh>=mesh_dt){
            last_mesh=now;
            shown=map.Snapshot(2.0f);
            g_nsurfel=shown.size();
            std::lock_guard<std::mutex> lk(g_in_mtx);
            if(!g_has_pending){ g_pending=shown; g_has_pending=true; g_cv.notify_one(); }
        }

        int fbw,fbh; glfwGetFramebufferSize(win,&fbw,&fbh);
        const float aspect=fbh>0?float(fbw)/fbh:1.f;
        glViewport(0,0,fbw,fbh);
        glClearColor(0.04f,0.05f,0.09f,1.f); glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
        glMatrixMode(GL_PROJECTION); glLoadIdentity();
        const float near_=0.02f,far_=10.f,top=near_*std::tan(30.f*kPi/180.0);
        glFrustum(-top*aspect,top*aspect,-top,top,near_,far_);
        glMatrixMode(GL_MODELVIEW); glLoadIdentity();
        glTranslatef(0,0,-g_cam.dist); glRotatef(g_cam.pitch,1,0,0); glRotatef(g_cam.yaw,0,1,0); glTranslatef(0,0,-0.12f);

        if(g_show_mesh){
            std::lock_guard<std::mutex> lk(g_mesh_mtx);
            if(!g_mesh.tris.empty()){
                const float lp[]={0.4f,-0.6f,1.0f,0.0f};
                glEnable(GL_LIGHTING); glEnable(GL_LIGHT0); glLightfv(GL_LIGHT0,GL_POSITION,lp);
                glEnable(GL_COLOR_MATERIAL); glColorMaterial(GL_FRONT_AND_BACK,GL_AMBIENT_AND_DIFFUSE);
                glColor3f(0.72f,0.76f,0.82f);
                glBegin(GL_TRIANGLES);
                for(size_t t=0;t+2<g_mesh.tris.size();t+=3){
                    for(int v=0;v<3;++v){ uint32_t idx=g_mesh.tris[t+v];
                        glNormal3f(g_mesh.normals[idx*3],g_mesh.normals[idx*3+1],g_mesh.normals[idx*3+2]);
                        glVertex3f(g_mesh.verts[idx*3],g_mesh.verts[idx*3+1],g_mesh.verts[idx*3+2]); }
                }
                glEnd();
                glDisable(GL_COLOR_MATERIAL); glDisable(GL_LIGHT0); glDisable(GL_LIGHTING);
            }
        } else {
            // 面元有向圆盘 splatting（原生面元法显示：实心、无洞）
            // 不依赖 GL 固定管线光照（本机驱动下会全黑）：CPU 朗伯着色
            glDisable(GL_LIGHTING);
            const Vec3 L = Vec3(0.4,-0.6,1.0).normalized();
            glBegin(GL_QUADS);
            const double h = g_voxel*1.4;
            for(const auto& s:shown){
                const Vec3 n=s.normal.normalized();
                Vec3 t=(std::abs(n.z())<0.9?Vec3(0,0,1):Vec3(1,0,0)).cross(n).normalized();
                Vec3 b=n.cross(t);
                const Vec3 p=s.position;
                const Vec3 c0=p+h*t-h*b, c1=p+h*t+h*b, c2=p-h*t+h*b, c3=p-h*t-h*b;
                const double k = 0.35 + 0.65*std::abs(n.dot(L));
                auto ch=[&](int shift){ int v=int(((s.color>>shift)&0xFF)*k); return v>255?255:v; };
                glColor3ub(ch(16), ch(8), ch(0));
                glVertex3d(c0.x(),c0.y(),c0.z()); glVertex3d(c1.x(),c1.y(),c1.z());
                glVertex3d(c2.x(),c2.y(),c2.z()); glVertex3d(c3.x(),c3.y(),c3.z());
            }
            glEnd();
        }
        glBegin(GL_LINES);
        glColor3ub(220,60,60); glVertex3f(0,0,0); glVertex3f(0.08f,0,0);
        glColor3ub(60,220,60); glVertex3f(0,0,0); glVertex3f(0,0.08f,0);
        glColor3ub(60,120,255); glVertex3f(0,0,0); glVertex3f(0,0,0.08f);
        glEnd();

        char title[220];
        std::snprintf(title,sizeof(title),"RUS SurfelMeshing-lite | frames=%d surfels=%zu tris=%zu build=%.0fms [%s]",
            frame_no,g_nsurfel.load(),g_tri_count.load(),g_build_ms.load(),g_show_mesh?"mesh":"surfels");
        glfwSetWindowTitle(win,title); glfwSwapBuffers(win);
    }
    { std::lock_guard<std::mutex> lk(g_in_mtx); g_stop=true; g_cv.notify_one(); }
    worker.join();
    glfwDestroyWindow(win); glfwTerminate();
    return 0;
}
