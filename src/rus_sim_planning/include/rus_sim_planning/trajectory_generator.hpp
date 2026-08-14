#pragma once

// ════════════════════════════════════════════════════════════════════
//  点云轨迹生成器（整合自 RUS_Backend/rus_sim_planner）
//  ────────────────────────────────────────────────────────────────────
//  核心算法：
//    1. 点云 k-NN 建图 + 椭圆 Gabriel 条件滤边
//    2. Dijkstra 求起点→终点的初始路径
//    3. 法线估计 + 牛顿迭代优化（M Δp = -M p）
//    4. 定向投影回点云 + Taubin 平滑
//  输出：末端（法兰）位姿轨迹（Trajectory = std::vector<Pose>），
//        由调用方交给插值计算模块稠密化后执行。
//  说明：轨迹按探头扫查设计。探头安装在机械臂末端（待标定，probe_to_flange）：
//        起终点（法兰坐标）经 法兰→探头 变换后在点云上找最近点规划路径；
//        输出路径点（探头接触点）经 探头→法兰 变换后作为法兰位姿交给驱动执行。
//        仅去掉 moveit 时代额外的末端偏移（flange_offset）。
// ════════════════════════════════════════════════════════════════════

#include <functional>
#include <optional>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <Eigen/Core>
#include <Eigen/Sparse>
#include <Eigen/IterativeLinearSolvers>
#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/conversions.h>
#include <pcl/memory.h>
#include <pcl/features/normal_3d.h>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>

namespace RusSimPlanning {

    using Vector3d = Eigen::Vector3d;                 // 3D 向量
    using Matrix4d = Eigen::Matrix4d;                 // 4x4 矩阵
    using Quaterniond = Eigen::Quaterniond;           // 四元数
    using SparseMatrixd = Eigen::SparseMatrix<double>;// 稀疏矩阵
    using Pose = geometry_msgs::msg::Pose;            // ROS2 位姿
    using Trajectory = std::vector<Pose>;             // 位姿轨迹（规划器输出 → 插值/执行）
    using Point = pcl::PointXYZ;                      // PCL 点
    using CloudPtr = pcl::PointCloud<pcl::PointXYZ>::Ptr;        // PCL 点云指针
    using CloudNormalsPtr = pcl::PointCloud<pcl::PointNormal>::Ptr;  // 带法向量点云
    using Graph = boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS,
                                        boost::no_property,
                                        boost::property<boost::edge_weight_t, float>>;

    // Point 转 Vector3d
    inline Vector3d PointToVector3d(const Point& p) { return Vector3d(p.x, p.y, p.z); }

    // 两点间距离
    inline double Distance(const Point& p1, const Point& p2) {
        double dx = p1.x - p2.x;
        double dy = p1.y - p2.y;
        double dz = p1.z - p2.z;
        return std::sqrt(dx * dx + dy * dy + dz * dz);
    }

    // 从 PointNormal 提取 Point
    inline Point PointNormalToPoint(const pcl::PointNormal& p) { return Point(p.x, p.y, p.z); }

    // 根据法向量 + 切向量生成四元数，保持末端姿态固定
    Quaterniond GenerateQuaternion(Vector3d& normal, Vector3d& tangent);

    /**
     * @brief 轨迹生成参数（全部可外部配置，见 config/planning_params.yaml）
     */
    struct TrajectoryParameter {
        double alpha = 1;            // 椭圆 Gabriel 条件检查参数
        int graph_k = 30;            // 建图 k-NN 参数
        int normal_k = 30;           // 法线估计参数
        int projection_k = 30;       // 重投影参数
        double tol = 1e-6;           // 长度变化容差
        int max_iter = 40;           // 最大迭代轮次
        bool use_smoothing = true;   // 是否开启平滑
        double lambda = 0.63;        // Taubin 平滑参数
        double mu = -0.65;           // Taubin 平滑参数
        Matrix4d probe_to_flange = Matrix4d::Identity();  // 法兰→探头 变换矩阵（探头待标定）
    };

    /**
     * @brief 点云轨迹生成器
     *
     * 基于点云做路径规划，输出起点→终点的末端位姿轨迹。
     * 生成结果通过 GetTrajectory() 取出，交给插值计算模块稠密化。
     */
    class TrajectoryGenerator {
    public:
        TrajectoryGenerator() = default;
        ~TrajectoryGenerator() = default;

        /**
         * @brief 加载点云并构建 KDTree / 法线 / 连接图（重复调用幂等）
         *
         * @param cloud 输入点云（XYZ，单位米）
         * @return true 初始化成功；false 点云为空或法线计算失败
         */
        bool LoadCloud(const CloudPtr& cloud);

        /**
         * @brief 设置生成参数
         *
         * @param parameter 生成参数（见 TrajectoryParameter）
         */
        void SetParameter(const TrajectoryParameter& parameter) { parameter_ = parameter; }

        /**
         * @brief 生成起点→终点的轨迹
         *
         * 按探头扫查设计：起终点为法兰坐标，经 法兰→探头 变换后在点云上找
         * 最近点规划路径；输出路径点（探头接触点）经 探头→法兰 变换后
         * 作为法兰位姿交给驱动执行。
         *
         * @param start 起点位姿（法兰坐标）
         * @param goal  终点位姿（法兰坐标）
         * @return true 生成成功；false 未初始化 / 初始路径失败 / 终点不可达
         */
        bool GenerateTrajectory(const Pose& start, const Pose& goal);

        /**
         * @brief 获取生成的轨迹
         *
         * @return 轨迹引用；未初始化或未生成时返回 nullopt
         */
        std::optional<std::reference_wrapper<const Trajectory>> GetTrajectory() const;

        /**
         * @brief 是否已加载点云并完成初始化
         */
        bool IsInitialized() const { return is_initialized_; }

    private:
        // 生成连接图
        void generate_graph();
        // Dijkstra 求初始路径
        bool generate_origin_path(int start_idx, int end_idx);
        // 椭圆 Gabriel 条件检查
        bool elliptic_gabriel_condition(int i, int j);
        bool point_inside_rotated_ellipsoid(const Point& pi, const Point& pj, const Point& pk, double alpha);
        // 全局点云法线估计
        bool compute_global_normals(const CloudPtr& cloud);
        // 路径点法线（近邻加权）
        void compute_path_normals();
        // 路径总长度
        double compute_path_length(const std::vector<Vector3d> path);
        // 构造优化矩阵 M
        void build_m();
        // 定向投影
        Vector3d project_point(const Vector3d& p, const Vector3d& n);
        // Taubin 平滑
        void taubin_smooth();
        // 查询点云最近点索引
        int find_nearest_point(const Point& p);

        // ============ 私有成员 ============
        bool is_initialized_ = false;                 // 是否已加载点云
        std::string class_name_ = "trajectory_generator";  // 日志名
        Pose start_pose_{};                           // 起始位姿
        Pose goal_pose_{};                            // 目标位姿
        Trajectory trajectory_;                       // 生成的轨迹

        pcl::KdTreeFLANN<Point> tree_;                // KDTree
        CloudNormalsPtr cloud_normals_ptr_;           // 带法线点云
        std::shared_ptr<Graph> graph_ptr_;            // 连接图
        std::vector<std::pair<Point, int>> origin_path_;  // 初始路径（点+索引）
        SparseMatrixd M;                              // 优化矩阵
        std::vector<Vector3d> result_path_;           // 优化后路径
        std::vector<Vector3d> result_path_normals_;   // 路径法线
        TrajectoryParameter parameter_;               // 生成参数
    };

}  // namespace RusSimPlanning
