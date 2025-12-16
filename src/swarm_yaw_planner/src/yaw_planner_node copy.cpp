#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <ceres/ceres.h>
#include <Eigen/Core>
#include <cmath>
#include <vector>

struct DroneState {
    bool has_odom = false;
    Eigen::Vector3d position;
    double prev_yaw = 0.0;  // 上一时刻 yaw（做平滑用）
};

// 工具函数：从 yaw 得到 2D 单位朝向向量 d(psi)
inline Eigen::Vector2d yawToDir(double yaw) {
    return Eigen::Vector2d(std::cos(yaw), std::sin(yaw));
}

// ----------------- Ceres 代价函数 -----------------

// 1) Outward alignment: || d(psi_i) - n_i ||^2
struct OutwardAlignmentCost {
    OutwardAlignmentCost(const Eigen::Vector2d& outward_dir, double weight) : outward_dir_(outward_dir), weight_(weight) {}

    template <typename T>
    bool operator()(const T* const yaw, T* residuals) const {
        T dx = ceres::cos(yaw[0]);
        T dy = ceres::sin(yaw[0]);

        // residual = sqrt(w) * (d - n)
        residuals[0] = T(std::sqrt(weight_)) * (dx - T(outward_dir_(0)));
        residuals[1] = T(std::sqrt(weight_)) * (dy - T(outward_dir_(1)));
        return true;
    }

    Eigen::Vector2d outward_dir_;
    double weight_;
};

// 2) Visibility cost: max(0, gamma - phi_max)^2
// gamma = arccos( d(psi_i)^T l_i )
struct VisibilityCost {
    VisibilityCost(const Eigen::Vector2d& los_dir, double phi_max_rad, double weight) : los_dir_(los_dir), phi_max_(phi_max_rad), weight_(weight) {}

    template <typename T>
    bool operator()(const T* const yaw, T* residuals) const {
        T dx = ceres::cos(yaw[0]);
        T dy = ceres::sin(yaw[0]);

        T dot = dx * T(los_dir_(0)) + dy * T(los_dir_(1));

        // clamp [-1, 1]
        if (dot > T(1.0))
            dot = T(1.0);
        if (dot < T(-1.0))
            dot = T(-1.0);

        T gamma = ceres::acos(dot);

        T diff = gamma - T(phi_max_);
        T penalty = (diff > T(0.0)) ? diff : T(0.0);

        residuals[0] = T(std::sqrt(weight_)) * penalty;

        return true;
    }

    Eigen::Vector2d los_dir_;
    double phi_max_;
    double weight_;
};

// 3) Smoothness cost: (psi_i - psi_prev_i)^2
struct SmoothnessCost {
    SmoothnessCost(double prev_yaw, double weight) : prev_yaw_(prev_yaw), weight_(weight) {}

    template <typename T>
    bool operator()(const T* const yaw, T* residuals) const {
        residuals[0] = T(std::sqrt(weight_)) * (yaw[0] - T(prev_yaw_));
        return true;
    }

    double prev_yaw_;
    double weight_;
};

// ----------------- 颜色结构体 -----------------
struct Color {
    double r, g, b, a;
    Color(double r_, double g_, double b_, double a_ = 1.0) : r(r_), g(g_), b(b_), a(a_) {}
};

// ----------------- 主类 -----------------

class YawPlannerNode {
   public:
    YawPlannerNode(ros::NodeHandle& nh) : nh_(nh) {
        // 从参数服务器读取所有参数
        nh_.param<double>("yaw_weight", w_yaw_, 1.0);
        nh_.param<double>("vis_weight", w_vis_, 10.0);
        nh_.param<double>("sm_weight", w_sm_, 0.1);
        nh_.param<double>("fov_horizontal_deg", fov_horizontal_deg_, 86.0);
        nh_.param<double>("fov_vertical_deg", fov_vertical_deg_, 57.0);  // 垂直FOV，用于锥体
        nh_.param<double>("safety_margin_deg", safety_margin_deg_, 5.0);
        nh_.param<double>("fov_ray_length", fov_ray_length_, 5.0);
        nh_.param<int>("num_drones", num_drones_, 4);
        nh_.param<int>("max_iterations", max_iterations_, 50);
        nh_.param<double>("timer_period", timer_period_, 0.1);
        nh_.param<std::string>("world_frame", world_frame_, "world");
        nh_.param<double>("fov_line_width", fov_line_width_, 0.05);
        nh_.param<double>("cone_alpha", cone_alpha_, 0.3);  // 锥体透明度

        // 话题名称参数
        nh_.param<std::string>("drone_odom_topic_prefix", drone_odom_topic_prefix_, "/drone_");
        nh_.param<std::string>("drone_odom_topic_suffix", drone_odom_topic_suffix_, "_visual_slam/odom");
        nh_.param<std::string>("target_topic", target_topic_, "/fxxzxec_estimator_target_target");
        nh_.param<std::string>("center_topic", center_topic_, "/mean_center_target");
        nh_.param<std::string>("yaw_cmd_topic_suffix", yaw_cmd_topic_suffix_, "/yaw_cmd");

        // FOV 相关计算
        double phi_half_deg = fov_horizontal_deg_ / 2.0 - safety_margin_deg_;
        phi_max_rad_ = phi_half_deg * M_PI / 180.0;

        // 定义每架无人机的颜色：黑色、红色、绿色、蓝色
        drone_colors_.push_back(Color(0.0, 0.0, 0.0));  // drone 0: 黑色
        drone_colors_.push_back(Color(1.0, 0.0, 0.0));  // drone 1: 红色
        drone_colors_.push_back(Color(0.0, 1.0, 0.0));  // drone 2: 绿色
        drone_colors_.push_back(Color(0.0, 0.0, 1.0));  // drone 3: 蓝色

        // 订阅无人机的里程计
        for (int i = 0; i < num_drones_; ++i) {
            std::string topic = drone_odom_topic_prefix_ + std::to_string(i) + drone_odom_topic_suffix_;
            ros::Subscriber sub = nh_.subscribe<nav_msgs::Odometry>(topic, 1, boost::bind(&YawPlannerNode::droneOdomCallback, this, _1, i));
            drone_odom_subs_.push_back(sub);
            drone_states_.push_back(DroneState());

            std::string yaw_topic = drone_odom_topic_prefix_ + std::to_string(i) + yaw_cmd_topic_suffix_;
            yaw_pubs_.push_back(nh_.advertise<std_msgs::Float64>(yaw_topic, 1));
        }

        // 目标
        target_sub_ = nh_.subscribe(target_topic_, 1, &YawPlannerNode::targetOdomCallback, this);
        // 虚拟质心
        center_sub_ = nh_.subscribe(center_topic_, 1, &YawPlannerNode::centerOdomCallback, this);

        // RViz Marker
        marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("yaw_fov_markers", 1);

        // 定时器
        timer_ = nh_.createTimer(ros::Duration(timer_period_), &YawPlannerNode::timerCallback, this);

        ROS_INFO("YawPlannerNode initialized with %d drones", num_drones_);
        ROS_INFO("FOV: horizontal=%.1f deg, vertical=%.1f deg", fov_horizontal_deg_, fov_vertical_deg_);
    }

   private:
    void droneOdomCallback(const nav_msgs::Odometry::ConstPtr& msg, int idx) {
        if (idx < 0 || idx >= num_drones_)
            return;
        drone_states_[idx].has_odom = true;
        drone_states_[idx].position = Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);

        // 从四元数提取当前 yaw 作为 prev_yaw 初值
        const auto& q = msg->pose.pose.orientation;
        double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
        double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
        double yaw = std::atan2(siny_cosp, cosy_cosp);
        drone_states_[idx].prev_yaw = yaw;
    }

    void targetOdomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        has_target_ = true;
        target_pos_ = Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    }

    void centerOdomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        has_center_ = true;
        center_pos_ = Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    }

    bool ready() const {
        if (!has_target_ || !has_center_)
            return false;
        for (int i = 0; i < num_drones_; ++i) {
            if (!drone_states_[i].has_odom)
                return false;
        }
        return true;
    }

    void timerCallback(const ros::TimerEvent&) {
        if (!ready())
            return;

        // 1) 构建 Ceres 问题：变量为 num_drones_ 个 yaw
        ceres::Problem problem;

        std::vector<double> yaws(num_drones_);
        for (int i = 0; i < num_drones_; ++i) {
            // 初值：用上一帧 yaw
            yaws[i] = drone_states_[i].prev_yaw;
            problem.AddParameterBlock(&yaws[i], 1);
        }

        // 2) 为每架无人机添加代价项
        for (int i = 0; i < num_drones_; ++i) {
            const auto& state = drone_states_[i];

            // outward direction: n_i = (p_i - C) / ||...||
            Eigen::Vector3d diff = state.position - center_pos_;
            Eigen::Vector2d outward_dir(diff.x(), diff.y());
            if (outward_dir.norm() < 1e-6) {
                outward_dir = Eigen::Vector2d(1.0, 0.0);  // 避免除零
            }
            outward_dir.normalize();

            // 目标方向 line-of-sight: l_i = (T - p_i)/||...||
            Eigen::Vector3d tdiff = target_pos_ - state.position;
            Eigen::Vector2d los_dir(tdiff.x(), tdiff.y());
            if (los_dir.norm() < 1e-6) {
                los_dir = Eigen::Vector2d(1.0, 0.0);
            }
            los_dir.normalize();

            // 2.1 J_yaw: outward alignment
            ceres::CostFunction* yaw_cost = new ceres::AutoDiffCostFunction<OutwardAlignmentCost, 2, 1>(new OutwardAlignmentCost(outward_dir, w_yaw_));
            problem.AddResidualBlock(yaw_cost, nullptr, &yaws[i]);

            // 2.2 J_vis: visibility penalty
            ceres::CostFunction* vis_cost = new ceres::AutoDiffCostFunction<VisibilityCost, 1, 1>(new VisibilityCost(los_dir, phi_max_rad_, w_vis_));
            problem.AddResidualBlock(vis_cost, nullptr, &yaws[i]);

            // 2.3 J_sm: smoothness (相对上一帧 yaw)
            ceres::CostFunction* sm_cost = new ceres::AutoDiffCostFunction<SmoothnessCost, 1, 1>(new SmoothnessCost(state.prev_yaw, w_sm_));
            problem.AddResidualBlock(sm_cost, nullptr, &yaws[i]);
        }

        // 3) 求解
        ceres::Solver::Options options;
        options.linear_solver_type = ceres::DENSE_QR;
        options.max_num_iterations = max_iterations_;
        options.minimizer_progress_to_stdout = false;

        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);

        // 4) 发布 yaw 与 Marker
        publishYawAndMarkers(yaws);
    }

    void publishYawAndMarkers(std::vector<double>& yaws) {
        visualization_msgs::MarkerArray ma;
        ros::Time now = ros::Time::now();

        double half_fov_h_rad = (fov_horizontal_deg_ * M_PI / 180.0) / 2.0;
        double half_fov_v_rad = (fov_vertical_deg_ * M_PI / 180.0) / 2.0;

        for (int i = 0; i < num_drones_; ++i) {
            // 记录最新 yaw
            drone_states_[i].prev_yaw = yaws[i];

            // 4.1 发布 yaw_cmd
            std_msgs::Float64 yaw_msg;
            yaw_msg.data = yaws[i];
            yaw_pubs_[i].publish(yaw_msg);

            // 获取颜色
            Color color = (i < (int)drone_colors_.size()) ? drone_colors_[i] : Color(1.0, 1.0, 1.0);

            // 4.2 创建FOV锥体的边框线（LINE_LIST）
            visualization_msgs::Marker line_marker;
            line_marker.header.stamp = now;
            line_marker.header.frame_id = world_frame_;
            line_marker.ns = "drone_fov_lines";
            line_marker.id = i;
            line_marker.type = visualization_msgs::Marker::LINE_LIST;
            line_marker.action = visualization_msgs::Marker::ADD;
            line_marker.scale.x = fov_line_width_;
            line_marker.color.r = color.r;
            line_marker.color.g = color.g;
            line_marker.color.b = color.b;
            line_marker.color.a = 1.0;

            // 无人机位置
            geometry_msgs::Point p0;
            p0.x = drone_states_[i].position.x();
            p0.y = drone_states_[i].position.y();
            p0.z = drone_states_[i].position.z();

            double yaw_center = yaws[i];

            // 计算锥体的4个角点（左上、右上、左下、右下）
            // 在无人机局部坐标系中，z轴向前，然后旋转到世界坐标系
            std::vector<geometry_msgs::Point> corners(4);

            // 角点偏移计算
            // 左上角: yaw + half_h, pitch + half_v
            // 右上角: yaw - half_h, pitch + half_v
            // 左下角: yaw + half_h, pitch - half_v
            // 右下角: yaw - half_h, pitch - half_v

            double angles_h[] = {half_fov_h_rad, -half_fov_h_rad, half_fov_h_rad, -half_fov_h_rad};
            double angles_v[] = {half_fov_v_rad, half_fov_v_rad, -half_fov_v_rad, -half_fov_v_rad};

            for (int j = 0; j < 4; ++j) {
                double yaw_j = yaw_center + angles_h[j];
                double pitch_j = angles_v[j];

                // 从yaw和pitch计算方向向量
                double dx = std::cos(pitch_j) * std::cos(yaw_j);
                double dy = std::cos(pitch_j) * std::sin(yaw_j);
                double dz = std::sin(pitch_j);

                corners[j].x = p0.x + fov_ray_length_ * dx;
                corners[j].y = p0.y + fov_ray_length_ * dy;
                corners[j].z = p0.z + fov_ray_length_ * dz;
            }

            // 从原点到4个角点的边（4条线）
            for (int j = 0; j < 4; ++j) {
                line_marker.points.push_back(p0);
                line_marker.points.push_back(corners[j]);
            }

            // 锥体底面的4条边（连接角点）
            // 左上-右上
            line_marker.points.push_back(corners[0]);
            line_marker.points.push_back(corners[1]);
            // 右上-右下
            line_marker.points.push_back(corners[1]);
            line_marker.points.push_back(corners[3]);
            // 右下-左下
            line_marker.points.push_back(corners[3]);
            line_marker.points.push_back(corners[2]);
            // 左下-左上
            line_marker.points.push_back(corners[2]);
            line_marker.points.push_back(corners[0]);

            ma.markers.push_back(line_marker);

            // 4.3 创建FOV锥体的半透明面（TRIANGLE_LIST）
            visualization_msgs::Marker cone_marker;
            cone_marker.header.stamp = now;
            cone_marker.header.frame_id = world_frame_;
            cone_marker.ns = "drone_fov_cone";
            cone_marker.id = i;
            cone_marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
            cone_marker.action = visualization_msgs::Marker::ADD;
            cone_marker.scale.x = 1.0;
            cone_marker.scale.y = 1.0;
            cone_marker.scale.z = 1.0;
            cone_marker.color.r = color.r;
            cone_marker.color.g = color.g;
            cone_marker.color.b = color.b;
            cone_marker.color.a = cone_alpha_;

            // 4个侧面三角形（从原点到底面边）
            // 顶面：p0, corners[0], corners[1]
            cone_marker.points.push_back(p0);
            cone_marker.points.push_back(corners[0]);
            cone_marker.points.push_back(corners[1]);

            // 右面：p0, corners[1], corners[3]
            cone_marker.points.push_back(p0);
            cone_marker.points.push_back(corners[1]);
            cone_marker.points.push_back(corners[3]);

            // 底面：p0, corners[3], corners[2]
            cone_marker.points.push_back(p0);
            cone_marker.points.push_back(corners[3]);
            cone_marker.points.push_back(corners[2]);

            // 左面：p0, corners[2], corners[0]
            cone_marker.points.push_back(p0);
            cone_marker.points.push_back(corners[2]);
            cone_marker.points.push_back(corners[0]);

            // 底面（封闭锥体底部）- 2个三角形
            // 三角形1：corners[0], corners[1], corners[2]
            cone_marker.points.push_back(corners[0]);
            cone_marker.points.push_back(corners[1]);
            cone_marker.points.push_back(corners[2]);

            // 三角形2：corners[1], corners[3], corners[2]
            cone_marker.points.push_back(corners[1]);
            cone_marker.points.push_back(corners[3]);
            cone_marker.points.push_back(corners[2]);

            ma.markers.push_back(cone_marker);
        }

        marker_pub_.publish(ma);
    }

   private:
    ros::NodeHandle nh_;
    std::vector<ros::Subscriber> drone_odom_subs_;
    ros::Subscriber target_sub_;
    ros::Subscriber center_sub_;
    ros::Publisher marker_pub_;

    std::vector<DroneState> drone_states_;
    bool has_target_ = false;
    bool has_center_ = false;
    Eigen::Vector3d target_pos_;
    Eigen::Vector3d center_pos_;

    std::vector<ros::Publisher> yaw_pubs_;

    ros::Timer timer_;

    // 参数
    double w_yaw_;
    double w_vis_;
    double w_sm_;
    double phi_max_rad_;
    double fov_horizontal_deg_;
    double fov_vertical_deg_;
    double safety_margin_deg_;
    double fov_ray_length_;
    int num_drones_;
    int max_iterations_;
    double timer_period_;
    std::string world_frame_;
    double fov_line_width_;
    double cone_alpha_;

    // 话题名称
    std::string drone_odom_topic_prefix_;
    std::string drone_odom_topic_suffix_;
    std::string target_topic_;
    std::string center_topic_;
    std::string yaw_cmd_topic_suffix_;

    // 颜色
    std::vector<Color> drone_colors_;
};

// ----------------- main -----------------
int main(int argc, char** argv) {
    ros::init(argc, argv, "yaw_planner_node");
    ros::NodeHandle nh("~");

    YawPlannerNode node(nh);

    ros::spin();
    return 0;
}