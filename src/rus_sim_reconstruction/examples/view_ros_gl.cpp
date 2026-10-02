// 实时「网格」可视化（订阅 ROS 重建结果）：
//   订阅 /reconstructed_cloud（PointCloud2）→ 后台 Poisson 重建三角网格 → OpenGL 实时渲染。
//   用于观察增量重建过程（feed → reconstruction_node → 本窗口）。
//   操作：左键拖动=旋转，滚轮=缩放，M=切换 网格/点云
//   用法: ros2 run rus_sim_reconstruction rus_sim_recon_view_ros

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <cstdio>
#include <mutex>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>
#include <pcl/PolygonMesh.h>
#include <pcl/conversions.h>
#include <pcl/surface/poisson.h>
#include <pcl_conversions/pcl_conversions.h>

#include <GL/glew.h>
#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>

namespace {
constexpr double kPi = 3.14159265358979323846;

struct MeshBuffers {
    std::vector<float> verts;
    std::vector<float> normals;
    size_t triangles() const { return verts.size() / 9; }
};

// ── 最新点云（ROS 回调写，工作线程读）──
std::mutex g_cloud_mtx;
std::condition_variable g_cloud_cv;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr g_cloud;
bool g_has_new = false;
bool g_stop = false;
std::atomic<int> g_depth{8};

// ── 当前网格（工作线程写，渲染读）──
std::mutex g_mesh_mtx;
MeshBuffers g_mesh;

MeshBuffers BuildMesh(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud)
{
    MeshBuffers mb;
    if (!cloud || cloud->size() < 100) return mb;

    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
    ne.setInputCloud(cloud);
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
    ne.setSearchMethod(tree);
    ne.setKSearch(10);
    ne.compute(*normals);

    pcl::PointCloud<pcl::PointNormal>::Ptr pn(new pcl::PointCloud<pcl::PointNormal>);
    pn->resize(cloud->size());
    for (size_t i = 0; i < cloud->size(); ++i) {
        const auto& p = (*cloud)[i];
        const auto& n = (*normals)[i];
        (*pn)[i].x = p.x; (*pn)[i].y = p.y; (*pn)[i].z = p.z;
        (*pn)[i].normal_x = n.normal_x; (*pn)[i].normal_y = n.normal_y; (*pn)[i].normal_z = n.normal_z;
    }
    pn->width = static_cast<uint32_t>(pn->size());
    pn->height = 1;
    pn->is_dense = false;

    pcl::Poisson<pcl::PointNormal> poisson;
    poisson.setDepth(g_depth.load());
    poisson.setSamplesPerNode(2.0f);
    poisson.setInputCloud(pn);
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
        float nx = uy * vz - uz * vy, ny = uz * vx - ux * vz, nz = ux * vy - uy * vx;
        const float len = std::sqrt(nx * nx + ny * ny + nz * nz);
        if (len > 1e-12f) { nx /= len; ny /= len; nz /= len; }
        for (const auto* p : {&a, &b, &c}) {
            mb.verts.push_back(p->x); mb.verts.push_back(p->y); mb.verts.push_back(p->z);
            mb.normals.push_back(nx); mb.normals.push_back(ny); mb.normals.push_back(nz);
        }
    }
    return mb;
}

void MeshWorker()
{
    for (;;) {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr in;
        {
            std::unique_lock<std::mutex> lk(g_cloud_mtx);
            g_cloud_cv.wait(lk, [] { return g_has_new || g_stop; });
            if (g_stop) return;
            in = g_cloud;
            g_has_new = false;
        }
        MeshBuffers mb = BuildMesh(in);
        {
            std::lock_guard<std::mutex> lk(g_mesh_mtx);
            g_mesh = std::move(mb);
        }
    }
}

struct Camera { float yaw = 30.f, pitch = 25.f, dist = 0.9f; } g_cam;
bool g_dragging = false;
double g_last_x = 0, g_last_y = 0;
bool g_show_mesh = true;
}  // namespace

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
    if (key == GLFW_KEY_1 && action == GLFW_PRESS) g_depth = 7;
    if (key == GLFW_KEY_2 && action == GLFW_PRESS) g_depth = 8;
    if (key == GLFW_KEY_3 && action == GLFW_PRESS) g_depth = 9;
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("recon_view");
    const std::string topic = node->declare_parameter<std::string>("cloud_topic", "/reconstructed_cloud");

    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    auto sub = node->create_subscription<sensor_msgs::msg::PointCloud2>(
        topic, qos, [](sensor_msgs::msg::PointCloud2::SharedPtr msg) {
            auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
            pcl::fromROSMsg(*msg, *cloud);
            if (cloud->empty()) return;
            {
                std::lock_guard<std::mutex> lk(g_cloud_mtx);
                g_cloud = cloud;
                g_has_new = true;
            }
            g_cloud_cv.notify_one();
        });
    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(node);
    std::thread ros_thread([&exec] { exec.spin(); });

    std::thread worker(MeshWorker);

    if (!glfwInit()) { std::fprintf(stderr, "glfwInit 失败\n"); return 1; }
    GLFWwindow* win = glfwCreateWindow(1000, 720,
        "RUS live MESH (ROS) drag=rotate wheel=zoom M=toggle 1/2/3=depth", nullptr, nullptr);
    glfwMakeContextCurrent(win);
    glfwSwapInterval(1);
    glewExperimental = GL_TRUE;
    if (glewInit() != GLEW_OK) { std::fprintf(stderr, "glewInit 失败\n"); return 1; }
    glfwSetMouseButtonCallback(win, OnMouseButton);
    glfwSetCursorPosCallback(win, OnCursor);
    glfwSetScrollCallback(win, OnScroll);
    glfwSetKeyCallback(win, OnKey);
    glEnable(GL_DEPTH_TEST);

    // 点云模式直接用最新点云
    std::vector<float> pt_xyz, pt_rgb;

    while (!glfwWindowShouldClose(win)) {
        glfwPollEvents();
        if (glfwWindowShouldClose(win)) break;

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
                glEnable(GL_LIGHTING); glEnable(GL_LIGHT0);
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
                glDisable(GL_COLOR_MATERIAL); glDisable(GL_LIGHT0); glDisable(GL_LIGHTING);
            }
        } else {
            std::lock_guard<std::mutex> lk(g_cloud_mtx);
            if (g_cloud) {
                glBegin(GL_POINTS);
                for (const auto& p : *g_cloud) {
                    glColor3ub(p.r, p.g, p.b);
                    glVertex3f(p.x, p.y, p.z);
                }
                glEnd();
            }
        }

        glBegin(GL_LINES);
        glColor3ub(220, 60, 60);  glVertex3f(0, 0, 0); glVertex3f(0.08f, 0, 0);
        glColor3ub(60, 220, 60);  glVertex3f(0, 0, 0); glVertex3f(0, 0.08f, 0);
        glColor3ub(60, 120, 255); glVertex3f(0, 0, 0); glVertex3f(0, 0, 0.08f);
        glEnd();

        char title[180];
        std::snprintf(title, sizeof(title), "RUS live MESH (ROS) | tris=%zu depth=%d [%s]  %s",
                      ntri, g_depth.load(), g_show_mesh ? "mesh" : "points", topic.c_str());
        glfwSetWindowTitle(win, title);
        glfwSwapBuffers(win);
    }

    {
        std::lock_guard<std::mutex> lk(g_cloud_mtx);
        g_stop = true;
        g_cloud_cv.notify_one();
    }
    worker.join();
    exec.cancel();
    ros_thread.join();
    glfwDestroyWindow(win);
    glfwTerminate();
    rclcpp::shutdown();
    return 0;
}
