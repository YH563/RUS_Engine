// 真正的增量网格：TSDF 体素融合 + Marching Tetrahedra 分块增量重建，OpenGL 实时渲染。
//   主循环：模拟传感器环绕出图 → Integrate 进 TSDF（增量）→ 定期只重算脏块出网格 → 渲染。
//   操作：左键拖动=旋转，滚轮=缩放
//   用法: rus_sim_recon_live_tsdf

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <random>
#include <vector>

#include <GL/glew.h>
#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>

#include "rus_sim_reconstruction/tsdf_volume.hpp"

using namespace RusReconstruction;

namespace {
constexpr double kPi = 3.14159265358979323846;
struct SP { Vec3 p, n; };
void AddSphere(std::vector<SP>& o, const Vec3& c, double rad, int nu, int nv)
{
    for (int i = 1; i < nu; ++i) {
        const double th = kPi * i / nu;
        for (int j = 0; j < nv; ++j) {
            const double ph = 2.0 * kPi * j / nv;
            const Vec3 n(std::sin(th) * std::cos(ph), std::sin(th) * std::sin(ph), std::cos(th));
            o.push_back({c + rad * n, n});
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
struct Camera { float yaw = 30.f, pitch = 25.f, dist = 0.9f; } g_cam;
bool g_dragging = false; double g_lx = 0, g_ly = 0;
}
static void OnMouse(GLFWwindow* w, int b, int a, int)
{
    if (b != GLFW_MOUSE_BUTTON_LEFT) return;
    if (a == GLFW_PRESS) { g_dragging = true; glfwGetCursorPos(w, &g_lx, &g_ly); } else g_dragging = false;
}
static void OnCursor(GLFWwindow*, double x, double y)
{
    if (!g_dragging) return;
    g_cam.yaw += static_cast<float>((x - g_lx) * 0.4);
    g_cam.pitch += static_cast<float>((y - g_ly) * 0.4);
    g_cam.pitch = std::max(-89.f, std::min(89.f, g_cam.pitch));
    g_lx = x; g_ly = y;
}
static void OnScroll(GLFWwindow*, double, double dy)
{
    g_cam.dist = std::max(0.15f, std::min(3.0f, g_cam.dist * static_cast<float>(std::pow(0.9, dy))));
}

int main()
{
    if (!glfwInit()) { std::fprintf(stderr, "glfwInit 失败\n"); return 1; }
    GLFWwindow* win = glfwCreateWindow(1000, 720,
        "RUS incremental TSDF mesh (drag=rotate, wheel=zoom)", nullptr, nullptr);
    glfwMakeContextCurrent(win);
    glfwSwapInterval(1);
    glewExperimental = GL_TRUE;
    if (glewInit() != GLEW_OK) { std::fprintf(stderr, "glewInit 失败\n"); return 1; }
    glfwSetMouseButtonCallback(win, OnMouse);
    glfwSetCursorPosCallback(win, OnCursor);
    glfwSetScrollCallback(win, OnScroll);
    glEnable(GL_DEPTH_TEST);

    std::vector<SP> scene;
    AddSphere(scene, Vec3(0, 0, 0.20), 0.07, 60, 120);  // 更密，保证覆盖
    AddPlane(scene, 0.16, 80);
    const Vec3 sphere_c(0, 0, 0.20);
    const double R = 0.40;
    const int NVIEW = 200;   // 全球面（含底部）环绕，覆盖完整 → 才有闭合表面

    std::mt19937 rng(7);
    std::normal_distribution<double> nd(0.0, 0.0008);

    TsdfVolume::Options opt;
    opt.voxel_size = 0.006;
    opt.truncation = 0.012;   // ≈2×体素
    TsdfVolume vol(opt);

    int frame_no = 0;
    double last_fuse = 0.0, last_mesh = 0.0;
    const double fuse_dt = 0.05;   // 每 50ms 一帧
    const double mesh_dt = 0.15;   // 每 150ms 只重算脏块
    size_t tris = 0;

    while (!glfwWindowShouldClose(win)) {
        glfwPollEvents();
        if (glfwWindowShouldClose(win)) break;
        const double now = glfwGetTime();

        if (now - last_fuse >= fuse_dt) {
            last_fuse = now;
            // 球面螺旋视角：覆盖整球的上下四周（含底部）
            const int kk = frame_no % NVIEW;
            const double ct = 1.0 - 2.0 * (kk + 0.5) / NVIEW;   // cos(theta): +1 → -1
            const double st = std::sqrt(std::max(0.0, 1.0 - ct * ct));
            const double phi = kk * (kPi * (1.0 + std::sqrt(5.0)));
            const Vec3 eye = sphere_c + R * Vec3(st * std::cos(phi), st * std::sin(phi), ct);

            std::vector<Vec3> pts, nrm;
            pts.reserve(scene.size()); nrm.reserve(scene.size());
            for (const auto& sp : scene) {
                const Vec3 dir = (eye - sp.p).normalized();
                if (sp.n.dot(dir) <= 0.1) continue;      // 可见性
                pts.push_back(sp.p + nd(rng) * sp.n);    // 沿法线噪声
                nrm.push_back(sp.n);
            }
            vol.Integrate(pts, nrm, eye);
            ++frame_no;
        }

        if (now - last_mesh >= mesh_dt) {
            last_mesh = now;
            vol.UpdateMesh();
            tris = vol.TriangleCount();
        }

        int fbw = 0, fbh = 0;
        glfwGetFramebufferSize(win, &fbw, &fbh);
        const float aspect = fbh > 0 ? static_cast<float>(fbw) / fbh : 1.0f;
        glViewport(0, 0, fbw, fbh);
        glClearColor(0.04f, 0.05f, 0.09f, 1.f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        glMatrixMode(GL_PROJECTION); glLoadIdentity();
        const float near_ = 0.02f, far_ = 10.f;
        const float top = near_ * std::tan(30.f * kPi / 180.0);
        glFrustum(-top * aspect, top * aspect, -top, top, near_, far_);
        glMatrixMode(GL_MODELVIEW); glLoadIdentity();
        glTranslatef(0.f, 0.f, -g_cam.dist);
        glRotatef(g_cam.pitch, 1.f, 0.f, 0.f);
        glRotatef(g_cam.yaw, 0.f, 1.f, 0.f);
        glTranslatef(0.f, 0.f, -0.12f);

        const auto& V = vol.MeshVertices();
        const auto& N = vol.MeshNormals();
        if (!V.empty()) {
            const float lp[] = {0.4f, -0.6f, 1.0f, 0.0f};
            glEnable(GL_LIGHTING); glEnable(GL_LIGHT0);
            glLightfv(GL_LIGHT0, GL_POSITION, lp);
            glEnable(GL_COLOR_MATERIAL);
            glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
            glBegin(GL_TRIANGLES);
            for (size_t t = 0; t + 8 < V.size(); t += 9) {
                glColor3f(0.72f, 0.76f, 0.82f);   // 单色材质 + 平滑顶点法线
                glNormal3f(N[t], N[t + 1], N[t + 2]);
                glVertex3f(V[t], V[t + 1], V[t + 2]);
                glNormal3f(N[t + 3], N[t + 4], N[t + 5]);
                glVertex3f(V[t + 3], V[t + 4], V[t + 5]);
                glNormal3f(N[t + 6], N[t + 7], N[t + 8]);
                glVertex3f(V[t + 6], V[t + 7], V[t + 8]);
            }
            glEnd();
            glDisable(GL_COLOR_MATERIAL); glDisable(GL_LIGHT0); glDisable(GL_LIGHTING);
        }

        glBegin(GL_LINES);
        glColor3ub(220, 60, 60);  glVertex3f(0, 0, 0); glVertex3f(0.08f, 0, 0);
        glColor3ub(60, 220, 60);  glVertex3f(0, 0, 0); glVertex3f(0, 0.08f, 0);
        glColor3ub(60, 120, 255); glVertex3f(0, 0, 0); glVertex3f(0, 0, 0.08f);
        glEnd();

        char title[180];
        std::snprintf(title, sizeof(title),
            "RUS incremental TSDF mesh | frames=%d blocks=%zu tris=%zu dirty=%zu",
            frame_no, vol.BlockCount(), tris, vol.DirtyBlockCount());
        glfwSetWindowTitle(win, title);
        glfwSwapBuffers(win);
    }

    glfwDestroyWindow(win);
    glfwTerminate();
    return 0;
}
