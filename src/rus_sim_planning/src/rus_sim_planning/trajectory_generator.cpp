#include "rus_sim_planning/trajectory_generator.hpp"

namespace RusSimPlanning {

    // ---- GenerateQuaternion — 法向量+切向量 → 四元数（保持末端姿态固定） ----
    Quaterniond GenerateQuaternion(Vector3d& normal, Vector3d& tangent)
    {
        Vector3d z = -normal.normalized();
        Vector3d x = tangent.normalized();

        // 正交化：保证 y 垂直于 z 和 x
        Vector3d y = z.cross(x).normalized();
        // 重新计算 x，保证三轴严格正交
        x = y.cross(z);

        // 旋转矩阵
        Eigen::Matrix3d R;
        R.col(0) = x;
        R.col(1) = y;
        R.col(2) = z;

        Quaterniond q(R);
        q.normalize();
        return q;
    }

    // ---- LoadCloud — 加载点云：KDTree + 法线 + 连接图 ----
    bool TrajectoryGenerator::LoadCloud(const CloudPtr& cloud)
    {
        if (is_initialized_) return true;
        if (cloud->size() == 0) {
            RCLCPP_ERROR(rclcpp::get_logger(class_name_), "请检查传入的点云数据");
            return false;
        }
        tree_.setInputCloud(cloud);
        if (!compute_global_normals(cloud)) {
            RCLCPP_ERROR(rclcpp::get_logger(class_name_), "点云法向量计算失败");
            return false;
        }
        graph_ptr_ = std::make_shared<Graph>();
        generate_graph();
        RCLCPP_INFO(rclcpp::get_logger(class_name_), "轨迹生成器已加载点云数据，完成初始化");
        is_initialized_ = true;
        return true;
    }

    // ---- GetTrajectory — 获取生成的轨迹 ----
    std::optional<std::reference_wrapper<const Trajectory>>
    TrajectoryGenerator::GetTrajectory() const
    {
        if (!is_initialized_) {
            RCLCPP_ERROR(rclcpp::get_logger(class_name_), "轨迹生成器未初始化");
            return std::nullopt;
        }
        if (trajectory_.empty()) {
            RCLCPP_ERROR(rclcpp::get_logger(class_name_), "轨迹未生成");
            return std::nullopt;
        }
        return std::cref(trajectory_);
    }

    // ---- GenerateTrajectory — 生成起点→终点的探头扫查轨迹（法兰↔探头变换） ----
    bool TrajectoryGenerator::GenerateTrajectory(const Pose& start, const Pose& goal)
    {
        // 清空内部轨迹
        trajectory_.clear();
        result_path_.clear();
        result_path_normals_.clear();
        origin_path_.clear();

        if (!is_initialized_) {
            RCLCPP_ERROR(rclcpp::get_logger(class_name_), "请先加载点云数据进行初始化");
            return false;
        }

        start_pose_ = start;
        goal_pose_  = goal;

        // 起终点为法兰坐标；探头安装在机械臂末端，点云由探头扫查得到，
        // 先把法兰位姿转探头位姿，用探头位置在点云上找最近点
        auto start_probe = FlangeToProbe(start, parameter_.probe_to_flange);
        auto goal_probe  = FlangeToProbe(goal,  parameter_.probe_to_flange);
        RCLCPP_INFO(rclcpp::get_logger(class_name_),
            "start_probe: (%.3f, %.3f, %.3f)",
            start_probe.position.x, start_probe.position.y, start_probe.position.z);
        RCLCPP_INFO(rclcpp::get_logger(class_name_),
            "goal_probe: (%.3f, %.3f, %.3f)",
            goal_probe.position.x, goal_probe.position.y, goal_probe.position.z);
        int start_idx = find_nearest_point(Point(start_probe.position.x, start_probe.position.y, start_probe.position.z));
        int end_idx   = find_nearest_point(Point(goal_probe.position.x,  goal_probe.position.y,  goal_probe.position.z));

        // 1. 生成初始轨迹
        if (!generate_origin_path(start_idx, end_idx)) {
            RCLCPP_ERROR(rclcpp::get_logger(class_name_), "初始轨迹生成失败");
            return false;
        }
        for (const auto& p : origin_path_) {
            result_path_.push_back(PointToVector3d(p.first));
            result_path_normals_.push_back(Vector3d(
                cloud_normals_ptr_->points[p.second].normal_x,
                cloud_normals_ptr_->points[p.second].normal_y,
                cloud_normals_ptr_->points[p.second].normal_z));
        }
        int m = static_cast<int>(origin_path_.size());

        // 路径点（点云表面）即探头接触点：法线/切向量生成探头姿态，
        // 再经 探头→法兰 变换后作为法兰位姿输出给驱动
        auto convert = [m, this]() {
            // 先计算每点原始切线（前后差分），再做滑动平均消除方向突变
            // （局部差分在急转弯处切线突变 → GenerateQuaternion 姿态突变 → 伺服 IK 解跳变）
            std::vector<Vector3d> tangent_raw(m);
            for (int i = 0; i < m; ++i) {
                if (i == 0) {
                    tangent_raw[i] = this->result_path_[1] - this->result_path_[0];
                } else if (i == m - 1) {
                    tangent_raw[i] = this->result_path_[m-1] - this->result_path_[m-2];
                } else {
                    Vector3d forward  = this->result_path_[i+1] - this->result_path_[i];
                    Vector3d backward = this->result_path_[i] - this->result_path_[i-1];
                    tangent_raw[i] = forward + backward;
                }
            }
            // 滑动平均（窗口 ±2，共 5 点），消除局部方向突变
            const int kWin = 2;
            std::vector<Vector3d> tangent(m);
            for (int i = 0; i < m; ++i) {
                Vector3d sum(0, 0, 0);
                int cnt = 0;
                for (int k = -kWin; k <= kWin; ++k) {
                    int j = i + k;
                    if (j >= 0 && j < m) { sum += tangent_raw[j]; ++cnt; }
                }
                if (sum.norm() > 1e-9)
                    tangent[i] = sum.normalized();
                else
                    tangent[i] = tangent_raw[i].normalized();
            }

            this->trajectory_.reserve(static_cast<size_t>(m));
            for (int i = 0; i < m; i++) {
                Pose temp_pose;
                temp_pose.position.x = this->result_path_[i].x();
                temp_pose.position.y = this->result_path_[i].y();
                temp_pose.position.z = this->result_path_[i].z();

                Quaterniond q = GenerateQuaternion(this->result_path_normals_[i], tangent[i]);
                temp_pose.orientation.x = q.x();
                temp_pose.orientation.y = q.y();
                temp_pose.orientation.z = q.z();
                temp_pose.orientation.w = q.w();
                // 探头接触点 → 法兰位姿
                this->trajectory_.push_back(ProbeToFlange(temp_pose, parameter_.probe_to_flange));

                // debug：首尾路径点（位置/法线/姿态），核对是否贴合表面、朝向是否正确
                if (i == 0 || i == m - 1) {
                    const Vector3d& n = this->result_path_normals_[i];
                    double rx = 0.0, ry = 0.0, rz = 0.0;
                    RusUtils::PoseToRPY(temp_pose, rx, ry, rz);
                    RCLCPP_INFO(rclcpp::get_logger(class_name_),
                        "路径点[%d/%d]: pos(%.3f, %.3f, %.3f) normal(%.3f, %.3f, %.3f) "
                        "姿态(%.3f,%.3f,%.3f,%.3f) RPY(%.3f, %.3f, %.3f)",
                        i, m, temp_pose.position.x, temp_pose.position.y, temp_pose.position.z,
                        n.x(), n.y(), n.z(),
                        q.x(), q.y(), q.z(), q.w(), rx, ry, rz);
                }
            }
        };

        if (m < 3) {
            convert();
            return true;
        }
        double prev_len = compute_path_length(result_path_);

        // 2. 牛顿迭代优化
        for (int iter = 0; iter < parameter_.max_iter; ++iter) {
            // 计算路径法向量，构建优化矩阵 M
            compute_path_normals();
            build_m();
            Eigen::VectorXd p_vec(3 * m);
            for (int i = 0; i < m; ++i) {
                p_vec.segment<3>(3 * i) = result_path_[i];
            }
            // 3. 求解牛顿步: M Δp = -M p
            Eigen::LeastSquaresConjugateGradient<SparseMatrixd> solver;
            solver.setTolerance(1e-8);
            solver.setMaxIterations(200);
            solver.compute(M);
            Eigen::VectorXd rhs   = -(M * p_vec);   // -M p
            Eigen::VectorXd dp_vec = solver.solve(rhs);

            // 4. 更新路径点（端点固定）
            for (int i = 1; i < m - 1; ++i) {
                result_path_[i] += dp_vec.segment<3>(3 * i);
            }

            // 5. 定向投影到点云
            for (int i = 1; i < m - 1; ++i) {
                result_path_[i] = project_point(result_path_[i], result_path_normals_[i]);
            }

            // 6. Taubin 平滑
            if (parameter_.use_smoothing) taubin_smooth();

            // 7. 收敛判断
            double cur_len = compute_path_length(result_path_);
            if (std::abs(cur_len - prev_len) < parameter_.tol) {
                break;
            }
            prev_len = cur_len;
        }
        convert();
        return true;
    }

    // ---- generate_graph — 点云 k-NN 建图 + 椭圆 Gabriel 条件滤边 ----
    void TrajectoryGenerator::generate_graph()
    {
        // 对每个点查找 k 个最近邻
        for (std::size_t i = 0; i < cloud_normals_ptr_->size(); ++i) {
            std::vector<int> knn_indices;
            std::vector<float> knn_distances;
            tree_.nearestKSearch(
                PointNormalToPoint(cloud_normals_ptr_->points[i]),
                parameter_.graph_k + 1, knn_indices, knn_distances);
            // 第一个是自身，跳过
            for (size_t t = 1; t < knn_indices.size(); ++t) {
                std::size_t j = knn_indices[t];
                if (i >= j) continue;  // 避免重复添加无向边
                Point pi = PointNormalToPoint(cloud_normals_ptr_->points[i]);
                Point pj = PointNormalToPoint(cloud_normals_ptr_->points[j]);
                // 检查椭圆 Gabriel 条件
                if (elliptic_gabriel_condition(static_cast<int>(i), static_cast<int>(j))) {
                    boost::add_edge(i, j, Distance(pi, pj), *graph_ptr_);
                }
            }
        }
    }

    // ---- generate_origin_path — Dijkstra 求初始路径 ----
    bool TrajectoryGenerator::generate_origin_path(int start_idx, int end_idx)
    {
        // 1. 顶点数量
        int n = static_cast<int>(boost::num_vertices(*graph_ptr_));
        if (start_idx < 0 || start_idx >= n || end_idx < 0 || end_idx >= n) {
            RCLCPP_ERROR(rclcpp::get_logger(class_name_), "请输入正确的起始点以及终点坐标!");
            return false;
        }

        // 2. 存储距离和前驱
        std::vector<float> distances(n);
        std::vector<int> predecessors(n);

        // 3. 运行 Dijkstra
        boost::dijkstra_shortest_paths(*graph_ptr_, start_idx,
            boost::distance_map(boost::make_iterator_property_map(distances.begin(),
                                    boost::get(boost::vertex_index, *graph_ptr_)))
            .predecessor_map(boost::make_iterator_property_map(predecessors.begin(),
                                    boost::get(boost::vertex_index, *graph_ptr_))));

        // 4. 检查终点是否可达
        if (distances[end_idx] == std::numeric_limits<float>::max()) {
            RCLCPP_ERROR(rclcpp::get_logger(class_name_), "不存在有效轨迹!");
            return false;
        }

        // 5. 回溯路径（从 end_idx 倒推到 start_idx）
        std::vector<int> vertex_path;
        for (int v = end_idx; v != start_idx; v = predecessors[v]) {
            vertex_path.push_back(v);
            if (v == predecessors[v]) {  // 防止死循环
                RCLCPP_ERROR(rclcpp::get_logger(class_name_), "轨迹前驱链断裂!");
                return false;
            }
        }
        vertex_path.push_back(start_idx);
        std::reverse(vertex_path.begin(), vertex_path.end());

        // 6. 将索引映射为点坐标
        for (int idx : vertex_path) {
            origin_path_.push_back(std::make_pair(PointNormalToPoint(cloud_normals_ptr_->points[idx]), idx));
        }
        return true;
    }

    // ---- elliptic_gabriel_condition — 椭圆 Gabriel 条件检查 ----
    bool TrajectoryGenerator::elliptic_gabriel_condition(int i, int j)
    {
        Point pi = PointNormalToPoint(cloud_normals_ptr_->points[i]);
        Point pj = PointNormalToPoint(cloud_normals_ptr_->points[j]);

        // 计算中点
        Point mid;
        mid.x = (pi.x + pj.x) * 0.5;
        mid.y = (pi.y + pj.y) * 0.5;
        mid.z = (pi.z + pj.z) * 0.5;

        // 半长轴长度 = 线段长度的一半
        double half_len = Distance(pi, pj) * 0.5 * 1.01;

        // 在中点附近搜索半径 = half_len 的邻域点（保守半径，可略放大）
        double radius = half_len * 1.01;   // 避免数值误差
        std::vector<int> idx_radius;
        std::vector<float> dist_radius;
        tree_.radiusSearch(mid, radius, idx_radius, dist_radius);

        // 遍历这些候选点，检查是否落在椭球内（缩放空间中的球）
        for (int idx : idx_radius) {
            if (idx == i || idx == j) continue;
            const Point& pk = PointNormalToPoint(cloud_normals_ptr_->points[idx]);
            // 如果有点落在旋转椭球内，则边 (i,j) 不满足 Gabriel 条件
            if (point_inside_rotated_ellipsoid(pi, pj, pk, parameter_.alpha)) {
                return false;
            }
        }
        return true;
    }

    // ---- point_inside_rotated_ellipsoid — 点是否落在旋转椭球内 ----
    bool TrajectoryGenerator::point_inside_rotated_ellipsoid(
        const Point& pi, const Point& pj, const Point& pk, double alpha)
    {
        // 1. 线段向量和长度
        double L = Distance(pi, pj);
        if (L < 1e-12) return false;
        Eigen::Vector3d v(pj.x - pi.x, pj.y - pi.y, pj.z - pi.z);
        Eigen::Vector3d u = v.normalized();  // 长轴方向单位向量

        // 2. 计算 pk 相对于中点 mid 的坐标
        Eigen::Vector3d mid((pi.x + pj.x) / 2.0, (pi.y + pj.y) / 2.0, (pi.z + pj.z) / 2.0);
        Eigen::Vector3d pk_vec(pk.x, pk.y, pk.z);
        Eigen::Vector3d w = pk_vec - mid;   // 向量：中点到 pk

        // 3. 将 w 投影到长轴方向 (u) 和垂直于 u 的平面
        double w_parallel = w.dot(u);        // 沿长轴的分量
        Eigen::Vector3d w_perp = w - w_parallel * u;
        double w_perp_norm = w_perp.norm();  // 垂直于长轴的距离

        // 4. 椭球方程： (w_parallel)^2 / (L/2)^2 + (w_perp_norm)^2 / ((L/2)*alpha)^2 < 1
        double a = L / 2.0;                  // 半长轴
        double b = a * alpha;                // 半短轴（垂直于长轴方向）
        double left = (w_parallel * w_parallel) / (a * a)
                    + (w_perp_norm * w_perp_norm) / (b * b);
        return left < 1.0;                   // 点在椭球内部返回 true（违反 Gabriel 条件）
    }

    // ---- compute_global_normals — 全局点云法线估计 ----
    bool TrajectoryGenerator::compute_global_normals(const CloudPtr& cloud)
    {
        pcl::NormalEstimation<Point, pcl::Normal> ne;
        ne.setInputCloud(cloud);
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
        tree->setInputCloud(cloud);
        ne.setSearchMethod(tree);
        ne.setKSearch(parameter_.normal_k);

        // 法线朝向：视点设在点云上方（探头/传感器来向），使法线统一朝表面外
        // （表面朝上场景：法线朝 +z → GenerateQuaternion 中 z 轴 = -normal 朝 -z，
        //   即法兰 z 轴朝下压向表面，符合扫查约定）
        float z_max = -1.0e6f;
        for (const auto& p : cloud->points) {
            if (p.z > z_max) z_max = p.z;
        }
        ne.setViewPoint(0.0f, 0.0f, z_max + 1.0f);
        pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
        ne.compute(*normals);

        if (normals->size() != cloud->size()) {
            RCLCPP_ERROR(rclcpp::get_logger(class_name_), "法向量计算失败");
            return false;
        }

        // 合并为 PointNormal 点云
        cloud_normals_ptr_.reset(new pcl::PointCloud<pcl::PointNormal>);
        cloud_normals_ptr_->resize(cloud->size());
        for (size_t i = 0; i < cloud->size(); ++i) {
            auto& p  = cloud->points[i];
            auto& n  = normals->points[i];
            auto& pn = cloud_normals_ptr_->points[i];
            pn.x = p.x; pn.y = p.y; pn.z = p.z;
            pn.normal_x = n.normal_x; pn.normal_y = n.normal_y; pn.normal_z = n.normal_z;
            pn.curvature = n.curvature;
        }

        RCLCPP_INFO(rclcpp::get_logger(class_name_),
            "已成功为数量为%zu的点云计算法向量", cloud_normals_ptr_->size());
        return true;
    }

    // ---- compute_path_normals — 路径点法线（近邻加权） ----
    void TrajectoryGenerator::compute_path_normals()
    {
        int m = static_cast<int>(result_path_.size());
        result_path_normals_.resize(m);

        for (int i = 0; i < m; ++i) {
            const Vector3d& pos = result_path_[i];

            // 搜索 k 个最近邻（原始点云）
            std::vector<int> indices(parameter_.normal_k);
            std::vector<float> sqr_dists(parameter_.normal_k);
            Point search_pt;
            search_pt.x = pos.x(); search_pt.y = pos.y(); search_pt.z = pos.z();
            tree_.nearestKSearch(search_pt, parameter_.normal_k, indices, sqr_dists);

            Vector3d normal_sum(0, 0, 0);
            double weight_sum = 0.0;
            for (int j = 0; j < parameter_.normal_k; ++j) {
                const auto& pt = cloud_normals_ptr_->points[indices[j]];
                double dist = std::sqrt(sqr_dists[j]);
                double w = std::exp(-dist);   // 权重 e^{-d}
                Vector3d n(pt.normal_x, pt.normal_y, pt.normal_z);
                normal_sum += w * n;
                weight_sum += w;
            }
            Vector3d normal = normal_sum / weight_sum;
            normal.normalize();
            result_path_normals_[i] = normal;
        }
    }

    // ---- compute_path_length — 路径总长度 ----
    double TrajectoryGenerator::compute_path_length(const std::vector<Vector3d> path)
    {
        double length = 0;
        for (std::size_t i = 1; i < path.size(); i++) {
            double dx = path[i].x() - path[i-1].x();
            double dy = path[i].y() - path[i-1].y();
            double dz = path[i].z() - path[i-1].z();
            length += sqrt(dx * dx + dy * dy + dz * dz);
        }
        return length;
    }

    // ---- build_m — 构造优化矩阵 M = K - NNT·K ----
    void TrajectoryGenerator::build_m()
    {
        int m = static_cast<int>(origin_path_.size());
        SparseMatrixd K(3 * m, 3 * m);
        K.reserve(9 * (m - 2));
        for (int i = 1; i < m - 1; ++i) {
            for (int d = 0; d < 3; ++d) {
                int row = 3 * i + d;
                // p_{i-1}
                K.insert(row, 3 * (i - 1) + d) = -1.0;
                // p_i
                K.insert(row, 3 * i + d)       =  2.0;
                // p_{i+1}
                K.insert(row, 3 * (i + 1) + d) = -1.0;
            }
        }
        K.makeCompressed();

        SparseMatrixd NNT(3 * m, 3 * m);
        NNT.reserve(9 * m);  // 每个点贡献 9 个非零

        for (int i = 0; i < m; ++i) {
            Vector3d ni = result_path_normals_[i];
            Eigen::Matrix3d block = ni * ni.transpose();
            for (int r = 0; r < 3; ++r) {
                for (int c = 0; c < 3; ++c) {
                    NNT.insert(3 * i + r, 3 * i + c) = block(r, c);
                }
            }
        }
        NNT.makeCompressed();

        M = K - NNT * K;
    }

    // ---- project_point — 沿法线定向投影到点云 ----
    Vector3d TrajectoryGenerator::project_point(const Vector3d& p, const Vector3d& n)
    {
        std::vector<int> indices(parameter_.projection_k);
        std::vector<float> sqr_dists(parameter_.projection_k);
        Point search_point;
        search_point.x = p.x();
        search_point.y = p.y();
        search_point.z = p.z();
        tree_.nearestKSearch(search_point, parameter_.projection_k, indices, sqr_dists);

        double sum_w = 0.0;
        Vector3d weighted_sum(0, 0, 0);
        for (int i = 0; i < parameter_.projection_k; ++i) {
            const auto& pt = cloud_normals_ptr_->points[indices[i]];
            double d = std::sqrt(sqr_dists[i]);
            double w = std::exp(-d);
            weighted_sum += w * Vector3d(pt.x, pt.y, pt.z);
            sum_w += w;
        }
        Vector3d centroid = weighted_sum / sum_w;

        // 投影公式： p' = p + t * n, 其中 t = (centroid - p)·n / (n·n)
        double t = (centroid - p).dot(n) / n.squaredNorm();
        return p + t * n;
    }

    // ---- taubin_smooth — Taubin 平滑（λ/μ 交替） ----
    void TrajectoryGenerator::taubin_smooth()
    {
        const double lambda = parameter_.lambda;
        const double mu = parameter_.mu;
        const int smooth_iters = 8;   // 8 次平滑迭代

        int m = static_cast<int>(result_path_.size());
        for (int iter = 0; iter < smooth_iters; ++iter) {
            double beta = (iter % 2 == 0) ? lambda : mu;
            std::vector<Vector3d> new_path = result_path_;
            for (int i = 1; i < m - 1; ++i) {
                Vector3d laplacian = result_path_[i-1] + result_path_[i+1] - 2.0 * result_path_[i];
                new_path[i] = result_path_[i] + beta * laplacian / 2.0;
            }
            result_path_ = new_path;

            // 平滑后重新投影到点云
            compute_path_normals();   // 基于新位置计算法向量
            for (int i = 1; i < m - 1; ++i) {
                result_path_[i] = project_point(result_path_[i], result_path_normals_[i]);
            }
        }
    }

    // ---- find_nearest_point — 查询点云最近点索引 ----
    int TrajectoryGenerator::find_nearest_point(const Point& p)
    {
        std::vector<int> nearest_index(1);      // 存储最近点索引
        std::vector<float> nearest_distance(1); // 存储距离
        tree_.nearestKSearch(p, 1, nearest_index, nearest_distance);
        return nearest_index[0];
    }

}  // namespace RusSimPlanning
