// 实时重建「网格」可视化（OpenGL/GLFW + 后台 Poisson 重建）：
//   - 主线程：模拟传感器环绕出图 → 分片并行面元融合 → 实时渲染三角形网格（带光照）
//   - 后台线程：定期把当前面元快照做 Poisson 重建，产出三角网格
//   操作：左键拖动=旋转，滚轮=缩放，M=切换 网格/点云
//   用法: rus_sim_recon_live

#include <algorithm>
#include <atomic>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <cstdio>
#include <mutex>
#include <random>
#include <thread>
#include <vector>

#include <GL/glew.h>
#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PolygonMesh.h>
#include <pcl/conversions.h>
#include <pcl/surface/poisson.h>

#include "rus_sim_reconstruction/sharded_surfel_map.hpp"

using namespace RusReconstruction;

namespace {
constexpr double kPi = 3.14159265358979323846;

struct ScenePoint { Vec3 p; Vec3 n; uint32_t color; };

void AddSphere(std::vector<ScenePoint>& out, const Vec3& c, double r, int nu, int nv)
{
    for (int i = 1; i < nu; ++i) {
        const double th = kPi * i / nu;
        for (int j = 0; j < nv; ++j) {
            const double ph = 2.0 * kPi * j / nv;
            const Vec3 n(std::sin(th) * std::cos(ph), std::sin(th) * std::sin(ph), std::cos(th));
            out.push_back({c + r * n, n, 0x33CC66});
        }
    }
}

void AddPlane(std::vector<ScenePoint>& out, double half, int n)
{
    for (int i = 0; i < n; ++i)
        for (int j = 0; j < n; ++j) {
            const double x = -half + 2.0 * half * j / (n - 1);
            const double y = -half + 2.0 * half * i / (n - 1);
            out.push_back({Vec3(x, y, 0.0), Vec3(0, 0, 1), 0x4488FF});
        }
}

// ── 网格缓冲（三角形汤：每个三角形 9 个 float 位置 + 9 个 float 法线）──
struct MeshBuffers {
    std::vector<float> verts;
    std::vector<float> normals;
    size_t triangles() const { return verts.size() / 9; }
};

MeshBuffers BuildMesh(const std::vector<Surfel>& surfels)
{
    MeshBuffers mb;
    if (surfels.size() < 100) return mb;

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

    pcl::Poisson<pcl::PointNormal> poisson;
    poisson.setDepth(8);
    poisson.setSamplesPerNode(2.0f);
    poisson.setInputCloud(cloud);
    pcl::PolygonMesh mesh;
    poisson.reconstruct(mesh);

    pcl::PointCloud<pcl::PointNormal> verts;
    pcl::fromPCLPointCloud2(mesh.cloud, verts);

    mb.verts.reserve(mesh.polygons.size() * 9);
    mb.normals.reserve(mesh.polygons.size() * 9);
    for (const auto& poly : mesh.polygons) {
        if (poly.vertices.size() != 3) continue;
        const auto& a = verts[poly.vertices[0]];
        const auto& b = verts[poly.vertices[1]];
        const auto& c = verts[poly.vertices[2]];
        const float ux = b.x - a.x, uy = b.y - a.y, uz = b.z - a.z;
        const float vx = c.x - a.x, vy = c.y - a.y, vz = c.z - a.z;
        float nx = uy * vz - uz * vy;
        float ny = uz * vx - ux * vz;
        float nz = ux * vy - uy * vx;
        const float len = std::sqrt(nx * nx + ny * ny + nz * nz);
        if (len > 1e-12f) { nx /= len; ny /= len; nz /= len; }
        for (const auto* p : {&a, &b, &c}) {
            mb.verts.push_back(p->x); mb.verts.push_back(p->y); mb.verts.push_back(p->z);
            mb.normals.push_back(nx); mb.normals.push_back(ny); mb.normals.push_back(nz);
        }
    }
    return mb;
}

// ── 线程间共享 ──
std::mutex g_mesh_mtx;
MeshBuffers g_mesh;

std::mutex g_in_mtx;
std::condition_variable g_in_cv;
std::vector<Surfel> g_pending;
bool g_has_pending = false;
bool g_stop = false;

void MeshWorker()
{
    for (;;) {
        std::vector<Surfel> in;
        {
            std::unique_lock<std::mutex> lk(g_in_mtx);
            g_in_cv.wait(lk, [] { return g_has_pending || g_stop; });
            if (g_stop) return;
            in = std::move(g_pending);
            g_has_pending = false;
        }
        MeshBuffers mb = BuildMesh(in);
        {
            std::lock_guard<std::mutex> lk(g_mesh_mtx);
            g_mesh = std::move(mb);
        }
    }
}

// ── 相机 / 交互 ──
struct Camera { float yaw = 30.f, pitch = 25.f, dist = 0.9f; } g_cam;
bool g_dragging = false;
double g_last_x = 0, g_last_y = 0;
bool g_show_mesh = true;
}

static void OnMouseButton(GLFWwindow* w, int button, int action, int)
{
    if (button != GLFW_MOUSE_BUTTON_LEFT) return;
    if (action == GLFW_PRESS) { g_dragging = true; glfwGetCursorPos(w, &g_last_x, &g_last_y); }
    else g_dragging = false;
}
static void OnCursor(GLFWwindow* w, double x, double y)
{
    if (!g_dragging) return;
    g_cam.yaw += static_cast<float>((x - g_last_x) * 0.4);
    g_cam.pitch += static_cast<float>((y - g_last_y) * 0.4);
    g_cam.pitch = std::max(-89.f, std::min(89.f, g_cam.pitch));
    g_last_x = x; g_last_y = y;
}
static void OnScroll(GLFWwindow*, double, double dy)
{
    g_cam.dist = std::max(0.15f, std::min(3.0f, g_cam.dist * static_cast<float>(std::pow(0.9, dy))));
}
static void OnKey(GLFWwindow*, int key, int, int action, int)
{
    if (key == GLFW_KEY_M && action == GLFW_PRESS) g_show_mesh = !g_show_mesh;
}

int main()
{
    if (!glfwInit()) { std::fprintf(stderr, "glfwInit 失败\n"); return 1; }
    GLFWwindow* win = glfwCreateWindow(1000, 720,
        "RUS live MESH reconstruction (drag=rotate, wheel=zoom, M=toggle)", nullptr, nullptr);
    if (!win) { std::fprintf(stderr, "创建窗口失败\n"); glfwTerminate(); return 1; }
    glfwMakeContextCurrent(win);
    glfwSwapInterval(1);
    glewExperimental = GL_TRUE;
    if (glewInit() != GLEW_OK) { std::fprintf(stderr, "glewInit 失败\n"); return 1; }

    glfwSetMouseButtonCallback(win, OnMouseButton);
    glfwSetCursorPosCallback(win, OnCursor);
    glfwSetScrollCallback(win, OnScroll);
    glfwSetKeyCallback(win, OnKey);
    glEnable(GL_DEPTH_TEST);
    glPointSize(2.5f);

    std::thread worker(MeshWorker);

    // 场景 + 位姿
    std::vector<ScenePoint> scene;
    AddSphere(scene, Vec3(0, 0, 0.20), 0.07, 40, 60);
    AddPlane(scene, 0.16, 80);
    const Vec3 target(0, 0, 0.12);
    const double orbit_r = 0.45;
    const int K = 24;

    std::mt19937 rng(7);
    std::normal_distribution<double> nd(0.0, 0.001);

    SurfelFusionOptions opt;
    opt.voxel_size = 0.004;
    ShardedSurfelMap map(opt, 12, 0);

    std::vector<Surfel> shown;           // 用于点云模式
    int frame_no = 0;
    double last_fuse = 0.0, last_mesh = 0.0;
    const double fuse_dt = 0.05;         // 融合限速
    const double mesh_dt = 1.5;          // 网格重建周期（Poisson 较重）

    while (!glfwWindowShouldClose(win)) {
        glfwPollEvents();  // 处理完事件（含窗口关闭）后继续
        if (glfwWindowShouldClose(win)) break;

        const double now = glfwGetTime();

        // 逐帧融合
        if (now - last_fuse >= fuse_dt) {
            last_fuse = now;
            const double a = 2.0 * kPi * (frame_no % K) / K;
            const Vec3 eye(orbit_r * std::cos(a), orbit_r * std::sin(a), 0.32);
            const Vec3 z = (eye - target).normalized();
            const Vec3 x = Vec3(0, 0, 1).cross(z).normalized();
            const Vec3 y = z.cross(x);
            Mat4 T = Mat4::Identity();
            T.block<3, 3>(0, 0) << x, y, z;
            T.block<3, 1>(0, 3) = eye;
            const Mat4 Tinv = T.inverse();

            Frame f;
            f.T_base_sensor = T;
            f.stamp = frame_no++;
            for (const auto& sp : scene) {
                const Vec3 dir = (eye - sp.p).normalized();
                if (sp.n.dot(dir) <= 0.1) continue;
                const Vec3 p_base = sp.p + nd(rng) * sp.n;
                f.points.push_back(Tinv.block<3, 3>(0, 0) * p_base + Tinv.block<3, 1>(0, 3));
                f.normals.push_back((Tinv.block<3, 3>(0, 0) * sp.n).normalized());
                f.colors.push_back(sp.color);
            }
            map.Fuse(f);
            if (!g_show_mesh) shown = map.Snapshot(2.0f);
        }

        // 定期触发网格重建（后台）
        if (now - last_mesh >= mesh_dt) {
            last_mesh = now;
            std::lock_guard<std::mutex> lk(g_in_mtx);
            if (!g_has_pending) {
                g_pending = map.Snapshot(3.0f);
                g_has_pending = true;
                g_in_cv.notify_one();
            }
        }

        // 渲染
        int fbw = 0, fbh = 0;
        glfwGetFramebufferSize(win, &fbw, &fbh);
        const float aspect = fbh > 0 ? static_cast<float>(fbw) / fbh : 1.0f;
        glViewport(0, 0, fbw, fbh);
        glClearColor(0.04f, 0.05f, 0.09f, 1.f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        const float near_ = 0.02f, far_ = 10.f;
        const float top = near_ * std::tan(30.f * kPi / 180.0);
        glFrustum(-top * aspect, top * aspect, -top, top, near_, far_);

        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
        glTranslatef(0.f, 0.f, -g_cam.dist);
        glRotatef(g_cam.pitch, 1.f, 0.f, 0.f);
        glRotatef(g_cam.yaw, 0.f, 1.f, 0.f);
        glTranslatef(0.f, 0.f, -0.12f);

        size_t ntri = 0;
        if (g_show_mesh) {
            std::lock_guard<std::mutex> lk(g_mesh_mtx);
            ntri = g_mesh.triangles();
            if (ntri > 0) {
                const float lp[] = {0.4f, -0.6f, 1.0f, 0.0f};
                glEnable(GL_LIGHTING);
                glEnable(GL_LIGHT0);
                glLightfv(GL_LIGHT0, GL_POSITION, lp);
                glEnable(GL_COLOR_MATERIAL);
                glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
                glBegin(GL_TRIANGLES);
                for (size_t t = 0; t < ntri; ++t) {
                    const float* v = &g_mesh.verts[t * 9];
                    const float* n = &g_mesh.normals[t * 9];
                    glNormal3f(n[0], n[1], n[2]);
                    const float zc = (v[2] + v[5] + v[8]) / 3.f;
                    const float tt = std::max(0.f, std::min(1.f, (zc + 0.02f) / 0.30f));
                    glColor3f(0.2f + 0.6f * tt, 0.5f + 0.3f * (1.f - std::abs(tt - 0.5f) * 2.f), 0.9f - 0.6f * tt);
                    glVertex3f(v[0], v[1], v[2]);
                    glVertex3f(v[3], v[4], v[5]);
                    glVertex3f(v[6], v[7], v[8]);
                }
                glEnd();
                glDisable(GL_COLOR_MATERIAL);
                glDisable(GL_LIGHT0);
                glDisable(GL_LIGHTING);
            }
        } else {
            glBegin(GL_POINTS);
            for (const auto& s : shown) {
                glColor3ub((s.color >> 16) & 0xFF, (s.color >> 8) & 0xFF, s.color & 0xFF);
                glVertex3d(s.position.x(), s.position.y(), s.position.z());
            }
            glEnd();
        }

        glBegin(GL_LINES);
        glColor3ub(220, 60, 60);  glVertex3f(0, 0, 0); glVertex3f(0.08f, 0, 0);
        glColor3ub(60, 220, 60);  glVertex3f(0, 0, 0); glVertex3f(0, 0.08f, 0);
        glColor3ub(60, 120, 255); glVertex3f(0, 0, 0); glVertex3f(0, 0, 0.08f);
        glEnd();

        char title[160];
        std::snprintf(title, sizeof(title),
                      "RUS live MESH  |  frames=%d  surfels=%zu  tris=%zu  [%s]",
                      frame_no, map.SurfaceCount(), ntri, g_show_mesh ? "mesh" : "points");
        glfwSetWindowTitle(win, title);
        glfwSwapBuffers(win);
    }

    {
        std::lock_guard<std::mutex> lk(g_in_mtx);
        g_stop = true;
        g_in_cv.notify_one();
    }
    worker.join();
    glfwDestroyWindow(win);
    glfwTerminate();
    return 0;
}
