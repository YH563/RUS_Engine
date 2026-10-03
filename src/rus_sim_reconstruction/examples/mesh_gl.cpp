// 三角网格查看器（OpenGL/GLFW，不依赖 matplotlib/VTK）：加载网格 PLY 并实时渲染。
//   操作：左键拖动=旋转，滚轮=缩放
//   用法: rus_sim_recon_meshview <mesh.ply>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include <GL/glew.h>
#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>

namespace {
constexpr double kPi = 3.14159265358979323846;

struct Mesh {
    std::vector<float> verts;    // 9 per tri（三角汤）
    std::vector<float> normals;  // 9 per tri
    float cx = 0, cy = 0, cz = 0, radius = 1.f;
};
Mesh g_mesh;

bool LoadMesh(const std::string& path)
{
    std::ifstream in(path);
    if (!in) return false;
    std::string line;
    std::getline(in, line);   // ply
    std::getline(in, line);   // format
    size_t nv = 0, nf = 0;
    while (std::getline(in, line)) {
        if (line.rfind("element vertex", 0) == 0) nv = std::stoul(line.substr(14));
        else if (line.rfind("element face", 0) == 0) nf = std::stoul(line.substr(12));
        else if (line.rfind("end_header", 0) == 0) break;
    }
    std::vector<std::array<float, 3>> v(nv);
    double mn[3] = {1e9, 1e9, 1e9}, mx[3] = {-1e9, -1e9, -1e9};
    for (size_t i = 0; i < nv; ++i) {
        std::getline(in, line);
        std::istringstream ss(line);
        float x, y, z;
        ss >> x >> y >> z;
        v[i] = {x, y, z};
        for (int a = 0; a < 3; ++a) { mn[a] = std::min<double>(mn[a], v[i][a]); mx[a] = std::max<double>(mx[a], v[i][a]); }
    }
    g_mesh.verts.reserve(nf * 9);
    g_mesh.normals.reserve(nf * 9);
    for (size_t f = 0; f < nf; ++f) {
        std::getline(in, line);
        std::istringstream ss(line);
        int k;
        ss >> k;
        std::vector<int> idx(k);
        for (int j = 0; j < k; ++j) ss >> idx[j];
        if (k < 3) continue;
        const auto& a = v[idx[0]];
        for (int j = 1; j + 1 < k; ++j) {   // 扇形三角化
            const auto& b = v[idx[j]];
            const auto& c = v[idx[j + 1]];
            float ux = b[0] - a[0], uy = b[1] - a[1], uz = b[2] - a[2];
            float wx = c[0] - a[0], wy = c[1] - a[1], wz = c[2] - a[2];
            float nx = uy * wz - uz * wy, ny = uz * wx - ux * wz, nz = ux * wy - uy * wx;
            float nl = std::sqrt(nx * nx + ny * ny + nz * nz);
            if (nl < 1e-12f) continue;
            nx /= nl; ny /= nl; nz /= nl;
            for (const auto* p : {&a, &b, &c}) {
                g_mesh.verts.insert(g_mesh.verts.end(), {(*p)[0], (*p)[1], (*p)[2]});
                g_mesh.normals.insert(g_mesh.normals.end(), {nx, ny, nz});
            }
        }
    }
    g_mesh.cx = 0.5f * (mn[0] + mx[0]);
    g_mesh.cy = 0.5f * (mn[1] + mx[1]);
    g_mesh.cz = 0.5f * (mn[2] + mx[2]);
    g_mesh.radius = 0.5f * std::sqrt((mx[0]-mn[0])*(mx[0]-mn[0]) + (mx[1]-mn[1])*(mx[1]-mn[1]) + (mx[2]-mn[2])*(mx[2]-mn[2]));
    if (g_mesh.radius < 1e-3f) g_mesh.radius = 0.1f;
    return !g_mesh.verts.empty();
}

struct Camera { float yaw = 30.f, pitch = 25.f, dist = 1.f; } g_cam;
bool g_drag = false; double g_lx = 0, g_ly = 0;
}
static void OnMouse(GLFWwindow* w, int b, int a, int)
{
    if (b != GLFW_MOUSE_BUTTON_LEFT) return;
    if (a == GLFW_PRESS) { g_drag = true; glfwGetCursorPos(w, &g_lx, &g_ly); } else g_drag = false;
}
static void OnCursor(GLFWwindow*, double x, double y)
{
    if (!g_drag) return;
    g_cam.yaw += static_cast<float>((x - g_lx) * 0.4);
    g_cam.pitch = std::max(-89.f, std::min(89.f, g_cam.pitch + static_cast<float>((y - g_ly) * 0.4)));
    g_lx = x; g_ly = y;
}
static void OnScroll(GLFWwindow*, double, double dy)
{
    g_cam.dist = std::max(0.2f, std::min(5.0f, g_cam.dist * static_cast<float>(std::pow(0.9, dy))));
}

int main(int argc, char** argv)
{
    const std::string path = argc > 1 ? argv[1] : "/tmp/opencode/recon_mesh.ply";
    if (!LoadMesh(path)) { std::fprintf(stderr, "加载网格失败: %s\n", path.c_str()); return 1; }
    g_cam.dist = g_mesh.radius * 3.0f;

    if (!glfwInit()) { std::fprintf(stderr, "glfwInit 失败\n"); return 1; }
    GLFWwindow* win = glfwCreateWindow(1000, 720, "RUS mesh (drag=rotate, wheel=zoom)", nullptr, nullptr);
    glfwMakeContextCurrent(win);
    glfwSwapInterval(1);
    glewExperimental = GL_TRUE;
    if (glewInit() != GLEW_OK) { std::fprintf(stderr, "glewInit 失败\n"); return 1; }
    glfwSetMouseButtonCallback(win, OnMouse);
    glfwSetCursorPosCallback(win, OnCursor);
    glfwSetScrollCallback(win, OnScroll);
    glEnable(GL_DEPTH_TEST);

    while (!glfwWindowShouldClose(win)) {
        glfwPollEvents();
        int fbw = 0, fbh = 0;
        glfwGetFramebufferSize(win, &fbw, &fbh);
        const float aspect = fbh > 0 ? static_cast<float>(fbw) / fbh : 1.f;
        glViewport(0, 0, fbw, fbh);
        glClearColor(0.04f, 0.05f, 0.09f, 1.f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        glMatrixMode(GL_PROJECTION); glLoadIdentity();
        const float near_ = g_mesh.radius * 0.02f, far_ = g_mesh.radius * 50.f;
        const float top = near_ * std::tan(30.f * kPi / 180.0);
        glFrustum(-top * aspect, top * aspect, -top, top, near_, far_);
        glMatrixMode(GL_MODELVIEW); glLoadIdentity();
        glTranslatef(0, 0, -g_cam.dist);
        glRotatef(g_cam.pitch, 1, 0, 0);
        glRotatef(g_cam.yaw, 0, 1, 0);
        glTranslatef(-g_mesh.cx, -g_mesh.cy, -g_mesh.cz);

        const float lp[] = {0.4f, -0.6f, 1.0f, 0.0f};
        glEnable(GL_LIGHTING); glEnable(GL_LIGHT0);
        glLightfv(GL_LIGHT0, GL_POSITION, lp);
        glEnable(GL_COLOR_MATERIAL);
        glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
        glColor3f(0.72f, 0.76f, 0.82f);
        glBegin(GL_TRIANGLES);
        for (size_t t = 0; t + 8 < g_mesh.verts.size(); t += 9) {
            glNormal3f(g_mesh.normals[t], g_mesh.normals[t + 1], g_mesh.normals[t + 2]);
            glVertex3f(g_mesh.verts[t], g_mesh.verts[t + 1], g_mesh.verts[t + 2]);
            glNormal3f(g_mesh.normals[t + 3], g_mesh.normals[t + 4], g_mesh.normals[t + 5]);
            glVertex3f(g_mesh.verts[t + 3], g_mesh.verts[t + 4], g_mesh.verts[t + 5]);
            glNormal3f(g_mesh.normals[t + 6], g_mesh.normals[t + 7], g_mesh.normals[t + 8]);
            glVertex3f(g_mesh.verts[t + 6], g_mesh.verts[t + 7], g_mesh.verts[t + 8]);
        }
        glEnd();
        glDisable(GL_COLOR_MATERIAL); glDisable(GL_LIGHT0); glDisable(GL_LIGHTING);

        char title[200];
        std::snprintf(title, sizeof(title), "RUS mesh | tris=%zu  %s", g_mesh.verts.size() / 9, path.c_str());
        glfwSetWindowTitle(win, title);
        glfwSwapBuffers(win);
    }
    glfwDestroyWindow(win);
    glfwTerminate();
    return 0;
}
