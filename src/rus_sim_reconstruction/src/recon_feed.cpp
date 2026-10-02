// 测试用数据源：模拟传感器环绕场景，连续发布多帧点云（base_link），
// 供 reconstruction_node 做增量重建。用法：
//   ros2 run rus_sim_reconstruction rus_sim_recon_feed --ros-args -p frames:=0 -p rate:=10.0

#include <chrono>
#include <cmath>
#include <cstdint>
#include <random>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

namespace {
constexpr double kPi = 3.14159265358979323846;
using Vec3 = Eigen::Vector3d;

struct SP { Vec3 p, n; uint8_t r, g, b; };

void AddSphere(std::vector<SP>& o, const Vec3& c, double rad, int nu, int nv)
{
    for (int i = 1; i < nu; ++i) {
        const double th = kPi * i / nu;
        for (int j = 0; j < nv; ++j) {
            const double ph = 2.0 * kPi * j / nv;
            const Vec3 n(std::sin(th) * std::cos(ph), std::sin(th) * std::sin(ph), std::cos(th));
            o.push_back({c + rad * n, n, 0x33, 0xCC, 0x66});
        }
    }
}
void AddPlane(std::vector<SP>& o, double half, int n)
{
    for (int i = 0; i < n; ++i)
        for (int j = 0; j < n; ++j) {
            const double x = -half + 2.0 * half * j / (n - 1);
            const double y = -half + 2.0 * half * i / (n - 1);
            o.push_back({Vec3(x, y, 0.0), Vec3(0, 0, 1), 0x44, 0x88, 0xFF});
        }
}
}  // namespace

class Feed : public rclcpp::Node {
public:
    Feed() : Node("recon_feed")
    {
        topic_ = declare_parameter<std::string>("topic", "/perception/frame");
        rate_ = declare_parameter<double>("rate", 10.0);
        max_frames_ = declare_parameter<int>("frames", 0);          // 0=无限
        sphere_points_ = declare_parameter<int>("sphere_nu", 40);
        plane_n_ = declare_parameter<int>("plane_n", 80);
        noise_ = declare_parameter<double>("noise", 0.001);

        AddSphere(scene_, Vec3(0, 0, 0.20), 0.07, sphere_points_, sphere_points_ * 3 / 2);
        AddPlane(scene_, 0.16, plane_n_);

        auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
        pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(topic_, qos);
        const auto period = std::chrono::duration<double>(1.0 / std::max(0.1, rate_));
        timer_ = create_wall_timer(period, std::bind(&Feed::Tick, this));

        RCLCPP_INFO(get_logger(), "测试数据源：topic=%s rate=%.1fHz frames=%d（0=无限）场景=%zu 点",
                    topic_.c_str(), rate_, max_frames_, scene_.size());
    }

private:
    void Tick()
    {
        if (max_frames_ > 0 && frame_no_ >= max_frames_) {
            if (!done_) { RCLCPP_INFO(get_logger(), "已发布 %d 帧，停止", frame_no_); done_ = true; }
            return;
        }

        const double a = 2.0 * kPi * (frame_no_ % 24) / 24.0;
        const Vec3 target(0, 0, 0.12);
        const double R = 0.45;
        const Vec3 eye(R * std::cos(a), R * std::sin(a), 0.32);

        pcl::PointCloud<pcl::PointXYZRGB> cloud;
        for (const auto& sp : scene_) {
            const Vec3 dir = (eye - sp.p).normalized();
            if (sp.n.dot(dir) <= 0.1) continue;
            const Vec3 p = sp.p + nd_(rng_) * sp.n;
            pcl::PointXYZRGB q;
            q.x = static_cast<float>(p.x()); q.y = static_cast<float>(p.y()); q.z = static_cast<float>(p.z());
            q.r = sp.r; q.g = sp.g; q.b = sp.b;
            cloud.push_back(q);
        }
        cloud.width = static_cast<uint32_t>(cloud.size());
        cloud.height = 1;

        sensor_msgs::msg::PointCloud2 msg;
        pcl::toROSMsg(cloud, msg);
        msg.header.frame_id = "base_link";
        msg.header.stamp = now();
        pub_->publish(msg);

        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "喂第 %d 帧：%zu 点",
                             frame_no_, cloud.size());
        ++frame_no_;
    }

    std::string topic_;
    double rate_ = 10.0, noise_ = 0.001;
    int max_frames_ = 0, frame_no_ = 0, sphere_points_ = 40, plane_n_ = 80;
    bool done_ = false;
    std::vector<SP> scene_;
    std::mt19937 rng_{7};
    std::normal_distribution<double> nd_{0.0, 0.001};
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Feed>());
    rclcpp::shutdown();
    return 0;
}
