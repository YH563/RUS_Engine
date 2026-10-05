// 实时（多线程）：原始稠密带噪点云 → 完整滤波链 → 稀疏 TSDF → 增量网格。
//   滤波链：直通(ROI) → 半径离群去除(ROR) → 体素 → 统计去离群 → MLS(平滑+法线) → 均匀采样
//   工作线程做「取帧→滤波→TSDF→网格」，主线程只渲染（60fps）。
//   操作：左键拖动=旋转，滚轮=缩放，P=网格/当前滤波点云
//   用法: rus_sim_recon_live_tsdf_filtered

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

#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include "pointcloud/cloud_resampler.hpp"   // 重采样实现位于 rus_sim_perception
#include "rus_sim_reconstruction/tsdf_volume.hpp"

using namespace RusReconstruction;
using CloudRGB = RusPerception::CloudRGB;   // PCL 彩色点云（perception 定义）
using RusPerception::PointCloud::Resample;
using RusPerception::PointCloud::ResampleOptions;
using RusPerception::PointCloud::ResampleResult;

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

// ── 线程间共享 ──
struct TriSoup { std::vector<float> verts; std::vector<float> normals; };
struct Stats { int frame=0; size_t raw=0, filt=0, blocks=0, tris=0, dirty=0; double filter_ms=0; };
std::mutex g_mtx;
TriSoup g_mesh;
std::vector<Vec3> g_fpts;
Stats g_stats;
std::atomic<bool> g_stop{false};

struct Camera { float yaw=30.f, pitch=25.f, dist=0.9f; } g_cam;
bool g_drag=false; double g_lx=0,g_ly=0; bool g_show_mesh=true;
}

// ── 网格清理：丢弃过长边三角形 + 小连通分量（清漂浮碎面）──
static void CleanMesh(TriSoup& m, double max_edge, size_t min_comp)
{
    const size_t nt = m.verts.size()/9;
    if (nt == 0) return;
    std::vector<uint8_t> keep(nt, 1);
    for (size_t t=0;t<nt;++t){                 // 过长边 → 丢弃
        const float* v=&m.verts[t*9];
        for(int e=0;e<3;++e){
            const float* a=v+e*3; const float* b=v+((e+1)%3)*3;
            const double dx=a[0]-b[0],dy=a[1]-b[1],dz=a[2]-b[2];
            if (dx*dx+dy*dy+dz*dz > max_edge*max_edge){ keep[t]=0; break; }
        }
    }
    struct K{int32_t x,y,z;bool operator==(const K&o)const{return x==o.x&&y==o.y&&z==o.z;}};
    struct KH{size_t operator()(const K&k)const{uint64_t h=1469598103934665603ull;
        auto m2=[&](int32_t v){h^=(uint32_t)v;h*=1099511628211ull;};m2(k.x);m2(k.y);m2(k.z);return(size_t)h;}};
    std::unordered_map<K,int,KH> vmap;
    std::vector<int> parent(nt); for(size_t i=0;i<nt;++i)parent[i]=(int)i;
    auto find=[&](int x){while(parent[x]!=x){parent[x]=parent[parent[x]];x=parent[x];}return x;};
    auto uni=[&](int a,int b){a=find(a);b=find(b);if(a!=b)parent[a]=b;};
    auto key_of=[&](const float* p){ return K{ (int32_t)std::llround(p[0]*1e4),
        (int32_t)std::llround(p[1]*1e4), (int32_t)std::llround(p[2]*1e4) }; };
    for(size_t t=0;t<nt;++t){
        if(!keep[t])continue;
        const float* v=&m.verts[t*9];
        for(int e=0;e<3;++e){
            const K k=key_of(v+e*3);
            auto it=vmap.find(k);
            if(it==vmap.end()) vmap.emplace(k,(int)t);
            else uni((int)t, it->second);
        }
    }
    std::unordered_map<int,int> comp;          // 连通分量 → 三角形数
    for(size_t t=0;t<nt;++t) if(keep[t]) comp[find((int)t)]++;
    TriSoup out; out.verts.reserve(m.verts.size()); out.normals.reserve(m.normals.size());
    for(size_t t=0;t<nt;++t){
        if(!keep[t] || comp[find((int)t)] < (int)min_comp) continue;
        for(int i=0;i<9;++i){ out.verts.push_back(m.verts[t*9+i]); out.normals.push_back(m.normals[t*9+i]); }
    }
    m = std::move(out);
}

// ── 工作线程：取帧 → 滤波链 → 稀疏 TSDF → 增量网格 ──
static void Worker()
{
    std::vector<SP> scene;
    AddSphere(scene,Vec3(0,0,0.20),0.07,60,120);
    AddPlane(scene,0.16,120);
    const Vec3 sc(0,0,0.20); const double R=0.40; const int NVIEW=200;
    std::mt19937 rng(7); std::normal_distribution<double> nd(0.0,0.002);

    const double spacing = 0.005;
    TsdfVolume::Options topt; topt.voxel_size = 0.006; topt.truncation = 0.014;
    TsdfVolume vol(topt);

    ResampleOptions ropt;
    ropt.voxel_leaf = float(spacing);
    ropt.enable_sor = true; ropt.sor_mean_k = 20; ropt.sor_std = 1.0f;
    ropt.enable_mls = true; ropt.mls_search_radius = float(spacing*3.0);
    ropt.mls_target_spacing = float(spacing);

    int frame_no=0;
    double last_fuse=0, last_mesh=0;
    const double fuse_dt=0.1, mesh_dt=0.3;
    auto t_start = std::chrono::steady_clock::now();

    while(!g_stop.load()){
        const double now = std::chrono::duration<double>(
            std::chrono::steady_clock::now()-t_start).count();
        if(now-last_fuse>=fuse_dt){
            last_fuse=now;
            const int kk=frame_no%NVIEW;
            const double ct=1.0-2.0*(kk+0.5)/NVIEW;
            const double st=std::sqrt(std::max(0.0,1.0-ct*ct));
            const double phi=kk*(kPi*(1.0+std::sqrt(5.0)));
            const Vec3 eye=sc+R*Vec3(st*std::cos(phi),st*std::sin(phi),ct);

            // 1) 原始帧（稠密 + 噪声 + 离群）
            CloudRGB raw;
            for(const auto& sp:scene){ const Vec3 d=(eye-sp.p).normalized(); if(sp.n.dot(d)<=0.1)continue;
                pcl::PointXYZRGB q; Vec3 p=sp.p+nd(rng)*sp.n;
                q.x=p.x(); q.y=p.y(); q.z=p.z(); q.r=q.g=q.b=180; raw.push_back(q); }
            std::uniform_real_distribution<double> uni(-0.2,0.2);
            for(int o=0;o<150;++o){ pcl::PointXYZRGB q; q.x=uni(rng); q.y=uni(rng); q.z=uni(rng);
                q.r=q.g=q.b=255; raw.push_back(q); }
            const size_t raw_n=raw.size();

            // 2) 滤波链
            const auto tf0=std::chrono::steady_clock::now();
            CloudRGB work;
            { pcl::PassThrough<pcl::PointXYZRGB> pass; pass.setInputCloud(raw.makeShared());
              pass.setFilterFieldName("z"); pass.setFilterLimits(-0.05,0.5); pass.filter(work); }
            { pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> ror;   // 清孤立离群 → 防碎三角
              ror.setInputCloud(work.makeShared());
              ror.setRadiusSearch(spacing*3.0); ror.setMinNeighborsInRadius(3); ror.filter(work); }
            ResampleResult rr; Resample(work, rr, ropt);
            const double filt_ms=std::chrono::duration<double,std::milli>(
                std::chrono::steady_clock::now()-tf0).count();

            // 3) 转 Vec3 喂稀疏 TSDF
            std::vector<Vec3> fpts, fnrm;
            fpts.reserve(rr.cloud.size()); fnrm.reserve(rr.cloud.size());
            for(size_t i=0;i<rr.cloud.size();++i){
                fpts.emplace_back(rr.cloud[i].x, rr.cloud[i].y, rr.cloud[i].z);
                Vec3 n = i<rr.normals.size()? Vec3(rr.normals[i].x(),rr.normals[i].y(),rr.normals[i].z()) : Vec3(0,0,1);
                if(n.norm()>1e-9) n.normalize();
                fnrm.push_back(n);
            }
            if(!fpts.empty()) vol.Integrate(fpts, fnrm, eye);

            {
                std::lock_guard<std::mutex> lk(g_mtx);
                g_fpts = std::move(fpts);
                g_stats.frame=frame_no; g_stats.raw=raw_n; g_stats.filt=rr.cloud.size();
                g_stats.blocks=vol.BlockCount(); g_stats.dirty=vol.DirtyBlockCount();
                g_stats.filter_ms=filt_ms;
            }
            ++frame_no;
        }

        if(now-last_mesh>=mesh_dt){
            last_mesh=now;
            vol.UpdateMesh();
            TriSoup m;
            m.verts=vol.MeshVertices(); m.normals=vol.MeshNormals();
            CleanMesh(m, topt.voxel_size*5.0, 30);   // 清漂浮碎面 / 过长边
            std::lock_guard<std::mutex> lk(g_mtx);
            g_mesh=std::move(m);
            g_stats.tris=g_mesh.verts.size()/9;
            g_stats.blocks=vol.BlockCount();
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
}

static void OnMouse(GLFWwindow* w,int b,int a,int){ if(b!=GLFW_MOUSE_BUTTON_LEFT)return; if(a==GLFW_PRESS){g_drag=true;glfwGetCursorPos(w,&g_lx,&g_ly);}else g_drag=false; }
static void OnCursor(GLFWwindow*,double x,double y){ if(!g_drag)return; g_cam.yaw+=float((x-g_lx)*0.4); g_cam.pitch=std::max(-89.f,std::min(89.f,g_cam.pitch+float((y-g_ly)*0.4))); g_lx=x;g_ly=y; }
static void OnScroll(GLFWwindow*,double,double dy){ g_cam.dist=std::max(0.15f,std::min(3.f,g_cam.dist*float(std::pow(0.9,dy)))); }
static void OnKey(GLFWwindow*,int key,int,int action,int){ if(key==GLFW_KEY_P&&action==GLFW_PRESS) g_show_mesh=!g_show_mesh; }

int main()
{
    if(!glfwInit()){std::fprintf(stderr,"glfwInit 失败\n");return 1;}
    GLFWwindow* win=glfwCreateWindow(1000,720,"RUS sparse TSDF (multithread) drag/wheel, P=toggle",nullptr,nullptr);
    glfwMakeContextCurrent(win); glfwSwapInterval(1);
    glewExperimental=GL_TRUE; if(glewInit()!=GLEW_OK) return 1;
    glfwSetMouseButtonCallback(win,OnMouse); glfwSetCursorPosCallback(win,OnCursor);
    glfwSetScrollCallback(win,OnScroll); glfwSetKeyCallback(win,OnKey);
    glDisable(GL_CULL_FACE); glLightModeli(GL_LIGHT_MODEL_TWO_SIDE,1); glEnable(GL_DEPTH_TEST);

    std::thread worker(Worker);

    while(!glfwWindowShouldClose(win)){
        glfwPollEvents(); if(glfwWindowShouldClose(win))break;
        int fbw,fbh; glfwGetFramebufferSize(win,&fbw,&fbh);
        const float aspect=fbh>0?float(fbw)/fbh:1.f;
        glViewport(0,0,fbw,fbh);
        glClearColor(0.04f,0.05f,0.09f,1.f); glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
        glMatrixMode(GL_PROJECTION); glLoadIdentity();
        const float near_=0.02f,far_=10.f,top=near_*std::tan(30.f*kPi/180.0);
        glFrustum(-top*aspect,top*aspect,-top,top,near_,far_);
        glMatrixMode(GL_MODELVIEW); glLoadIdentity();
        glTranslatef(0,0,-g_cam.dist); glRotatef(g_cam.pitch,1,0,0); glRotatef(g_cam.yaw,0,1,0); glTranslatef(0,0,-0.12f);

        Stats st;
        if(g_show_mesh){
            std::lock_guard<std::mutex> lk(g_mtx);
            st=g_stats;
            const auto& V=g_mesh.verts; const auto& N=g_mesh.normals;
            if(!V.empty()){
                const float lp[]={0.4f,-0.6f,1.0f,0.0f};
                glEnable(GL_LIGHTING); glEnable(GL_LIGHT0); glLightfv(GL_LIGHT0,GL_POSITION,lp);
                glEnable(GL_COLOR_MATERIAL); glColorMaterial(GL_FRONT_AND_BACK,GL_AMBIENT_AND_DIFFUSE);
                glColor3f(0.72f,0.76f,0.82f);
                glBegin(GL_TRIANGLES);
                for(size_t t=0;t+8<V.size();t+=9)
                    for(int v=0;v<3;++v){ glNormal3f(N[t+v*3],N[t+v*3+1],N[t+v*3+2]);
                        glVertex3f(V[t+v*3],V[t+v*3+1],V[t+v*3+2]); }
                glEnd();
                glDisable(GL_COLOR_MATERIAL); glDisable(GL_LIGHT0); glDisable(GL_LIGHTING);
            }
        } else {
            std::lock_guard<std::mutex> lk(g_mtx);
            st=g_stats;
            glPointSize(2.0f);
            glBegin(GL_POINTS);
            for(const auto& p:g_fpts){ glColor3ub(120,200,255); glVertex3d(p.x(),p.y(),p.z()); }
            glEnd();
        }
        glBegin(GL_LINES);
        glColor3ub(220,60,60); glVertex3f(0,0,0); glVertex3f(0.08f,0,0);
        glColor3ub(60,220,60); glVertex3f(0,0,0); glVertex3f(0,0.08f,0);
        glColor3ub(60,120,255); glVertex3f(0,0,0); glVertex3f(0,0,0.08f);
        glEnd();

        char title[240];
        std::snprintf(title,sizeof(title),
            "RUS sparse TSDF (multithread) | frames=%d raw=%zu->filt=%zu (%.0fms) blocks=%zu tris=%zu dirty=%zu [%s]",
            st.frame, st.raw, st.filt, st.filter_ms, st.blocks, st.tris, st.dirty,
            g_show_mesh?"mesh":"points");
        glfwSetWindowTitle(win,title); glfwSwapBuffers(win);
    }
    g_stop.store(true); worker.join();
    glfwDestroyWindow(win); glfwTerminate();
    return 0;
}
