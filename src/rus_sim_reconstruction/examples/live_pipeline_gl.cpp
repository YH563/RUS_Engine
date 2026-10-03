// 实时重建管线（无泊松）：每帧点云 → 均匀降采样 → 增量 TSDF → Marching Tetrahedra 分块网格。
//   操作：左键拖动=旋转，滚轮=缩放
//   用法: rus_sim_recon_live_pipeline

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <random>
#include <unordered_map>
#include <vector>

#include <GL/glew.h>
#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>

#include "rus_sim_reconstruction/tsdf_volume.hpp"

using namespace RusReconstruction;

namespace {
constexpr double kPi = 3.14159265358979323846;

struct SP { Vec3 p, n; };
void AddSphere(std::vector<SP>& o, const Vec3& c, double r, int nu, int nv)
{
    for (int i = 1; i < nu; ++i) {
        const double th = kPi * i / nu;
        for (int j = 0; j < nv; ++j) {
            const double ph = 2.0 * kPi * j / nv;
            const Vec3 n(std::sin(th) * std::cos(ph), std::sin(th) * std::sin(ph), std::cos(th));
            o.push_back({c + r * n, n});
        }
    }
}
void AddPlane(std::vector<SP>& o, double half, int n)
{
    for (int i = 0; i < n; ++i)
        for (int j = 0; j < n; ++j) {
            const double x = -half + 2.0 * half * j / (n - 1);
            const double y = -half + 2.0 * half * i / (n - 1);
            o.push_back({Vec3(x, y, 0.0), Vec3(0, 0, 1)});
        }
}

// 网格贪心均匀降采样（保证间距 ≥ min_dist），保留法线
struct CK { int32_t x, y, z; bool operator==(const CK& o) const { return x==o.x&&y==o.y&&z==o.z; } };
struct CKH { size_t operator()(const CK& k) const {
    uint64_t h = 1469598103934665603ull;
    auto m=[&h](int32_t v){ h^=(uint32_t)v; h*=1099511628211ull; }; m(k.x);m(k.y);m(k.z); return (size_t)h; } };

void UniformDownsample(const std::vector<Vec3>& ip, const std::vector<Vec3>& in,
                       double min_dist, std::vector<Vec3>& op, std::vector<Vec3>& on)
{
    op.clear(); on.clear();
    if (min_dist <= 0) { op = ip; on = in; return; }
    const double cell = min_dist, md2 = min_dist * min_dist;
    std::unordered_map<CK, std::vector<int>, CKH> grid;
    auto cell_of = [cell](const Vec3& p) {
        return CK{static_cast<int32_t>(std::floor(p.x()/cell)),
                  static_cast<int32_t>(std::floor(p.y()/cell)),
                  static_cast<int32_t>(std::floor(p.z()/cell))};
    };
    for (size_t i = 0; i < ip.size(); ++i) {
        const Vec3& p = ip[i];
        const CK c = cell_of(p);
        bool ok = true;
        for (int dx=-1; dx<=1 && ok; ++dx) for (int dy=-1; dy<=1 && ok; ++dy) for (int dz=-1; dz<=1 && ok; ++dz) {
            auto it = grid.find(CK{c.x+dx, c.y+dy, c.z+dz});
            if (it == grid.end()) continue;
            for (int idx : it->second) if ((op[idx]-p).squaredNorm() < md2) { ok=false; break; }
        }
        if (!ok) continue;
        op.push_back(p); on.push_back(in[i]);
        grid[c].push_back(static_cast<int>(op.size()-1));
    }
}

struct Camera { float yaw = 30.f, pitch = 25.f, dist = 0.9f; } g_cam;
bool g_drag = false; double g_lx = 0, g_ly = 0;
}
static void OnMouse(GLFWwindow* w, int b, int a, int) {
    if (b != GLFW_MOUSE_BUTTON_LEFT) return;
    if (a == GLFW_PRESS) { g_drag = true; glfwGetCursorPos(w, &g_lx, &g_ly); } else g_drag = false;
}
static void OnCursor(GLFWwindow*, double x, double y) {
    if (!g_drag) return;
    g_cam.yaw += static_cast<float>((x-g_lx)*0.4f);
    g_cam.pitch = std::max(-89.f, std::min(89.f, g_cam.pitch + static_cast<float>((y-g_ly)*0.4f)));
    g_lx = x; g_ly = y;
}
static void OnScroll(GLFWwindow*, double, double dy) {
    g_cam.dist = std::max(0.15f, std::min(3.0f, g_cam.dist * static_cast<float>(std::pow(0.9, dy))));
}

int main()
{
    if (!glfwInit()) { std::fprintf(stderr, "glfwInit 失败\n"); return 1; }
    GLFWwindow* win = glfwCreateWindow(1000, 720,
        "RUS live pipeline: per-frame downsample -> incremental TSDF (no Poisson)", nullptr, nullptr);
    glfwMakeContextCurrent(win);
    glfwSwapInterval(1);
    glewExperimental = GL_TRUE;
    if (glewInit() != GLEW_OK) { std::fprintf(stderr, "glewInit 失败\n"); return 1; }
    glfwSetMouseButtonCallback(win, OnMouse);
    glfwSetCursorPosCallback(win, OnCursor);
    glfwSetScrollCallback(win, OnScroll);
    glEnable(GL_DEPTH_TEST);

    std::vector<SP> scene;
    AddSphere(scene, Vec3(0, 0, 0.20), 0.07, 80, 160);  // 密（模拟真机多点数）
    AddPlane(scene, 0.16, 160);
    const Vec3 sphere_c(0, 0, 0.20);
    const double R = 0.40;
    const int NVIEW = 240;

    std::mt19937 rng(7);
    std::normal_distribution<double> nd(0.0, 0.002);   // 2mm 噪声

    TsdfVolume::Options opt;
    opt.voxel_size = 0.006;
    opt.truncation = 0.014;
    TsdfVolume vol(opt);

    const double spacing = 0.006;   // 每帧重采样目标间距
    int frame_no = 0; size_t raw_n = 0, ds_n = 0;
    double last_fuse = 0, last_mesh = 0;
    const double fuse_dt = 0.05, mesh_dt = 0.15;
    size_t tris = 0;

    while (!glfwWindowShouldClose(win)) {
        glfwPollEvents();
        if (glfwWindowShouldClose(win)) break;
        const double now = glfwGetTime();

        if (now - last_fuse >= fuse_dt) {
            last_fuse = now;
            const int kk = frame_no % NVIEW;
            const double ct = 1.0 - 2.0 * (kk + 0.5) / NVIEW;
            const double st = std::sqrt(std::max(0.0, 1.0 - ct * ct));
            const double phi = kk * (kPi * (1.0 + std::sqrt(5.0)));
            const Vec3 eye = sphere_c + R * Vec3(st * std::cos(phi), st * std::sin(phi), ct);

            std::vector<Vec3> rp, rn;
            for (const auto& sp : scene) {
                const Vec3 dir = (eye - sp.p).normalized();
                if (sp.n.dot(dir) <= 0.1) continue;
                rp.push_back(sp.p + nd(rng) * sp.n);
                rn.push_back(sp.n);
            }
            raw_n = rp.size();

            // 每帧均匀降采样（保留法线）→ 更均匀、更少的点喂给 TSDF
            std::vector<Vec3> dp, dn;
            UniformDownsample(rp, rn, spacing, dp, dn);
            ds_n = dp.size();

            vol.Integrate(dp, dn, eye);
            ++frame_no;
        }

        if (now - last_mesh >= mesh_dt) {
            last_mesh = now;
            vol.UpdateMesh();
            tris = vol.TriangleCount();
        }

        int fbw = 0, fbh = 0;
        glfwGetFramebufferSize(win, &fbw, &fbh);
        const float aspect = fbh > 0 ? static_cast<float>(fbw) / fbh : 1.f;
        glViewport(0, 0, fbw, fbh);
        glClearColor(0.04f, 0.05f, 0.09f, 1.f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        glMatrixMode(GL_PROJECTION); glLoadIdentity();
        const float near_ = 0.02f, far_ = 10.f;
        const float top = near_ * std::tan(30.f * kPi / 180.0);
        glFrustum(-top * aspect, top * aspect, -top, top, near_, far_);
        glMatrixMode(GL_MODELVIEW); glLoadIdentity();
        glTranslatef(0, 0, -g_cam.dist);
        glRotatef(g_cam.pitch, 1, 0, 0);
        glRotatef(g_cam.yaw, 0, 1, 0);
        glTranslatef(0, 0, -0.12f);

        const auto& V = vol.MeshVertices();
        const auto& N = vol.MeshNormals();
        if (!V.empty()) {
            const float lp[] = {0.4f, -0.6f, 1.0f, 0.0f};
            glEnable(GL_LIGHTING); glEnable(GL_LIGHT0);
            glLightfv(GL_LIGHT0, GL_POSITION, lp);
            glEnable(GL_COLOR_MATERIAL);
            glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
            glColor3f(0.72f, 0.76f, 0.82f);
            glBegin(GL_TRIANGLES);
            for (size_t t = 0; t + 8 < V.size(); t += 9) {
                for (int v = 0; v < 3; ++v) {
                    glNormal3f(N[t + v*3], N[t + v*3 + 1], N[t + v*3 + 2]);
                    glVertex3f(V[t + v*3], V[t + v*3 + 1], V[t + v*3 + 2]);
                }
            }
            glEnd();
            glDisable(GL_COLOR_MATERIAL); glDisable(GL_LIGHT0); glDisable(GL_LIGHTING);
        }

        glBegin(GL_LINES);
        glColor3ub(220,60,60); glVertex3f(0,0,0); glVertex3f(0.08f,0,0);
        glColor3ub(60,220,60); glVertex3f(0,0,0); glVertex3f(0,0.08f,0);
        glColor3ub(60,120,255); glVertex3f(0,0,0); glVertex3f(0,0,0.08f);
        glEnd();

        char title[220];
        std::snprintf(title, sizeof(title),
            "RUS live pipeline | frames=%d  raw=%zu -> ds=%zu  blocks=%zu tris=%zu dirty=%zu",
            frame_no, raw_n, ds_n, vol.BlockCount(), tris, vol.DirtyBlockCount());
        glfwSetWindowTitle(win, title);
        glfwSwapBuffers(win);
    }
    glfwDestroyWindow(win);
    glfwTerminate();
    return 0;
}
