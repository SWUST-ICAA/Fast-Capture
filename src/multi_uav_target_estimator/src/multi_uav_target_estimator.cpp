// 修改版本：滑动窗口存储每个时间戳下所有无人机的平均观测
#include <ceres/ceres.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

#include <Eigen/Dense>
#include <deque>
#include <map>
#include <mutex>
#include <random>
#include <string>
#include <vector>

// 单个时间戳的平均观测
struct AveragedObservation {
    double timestamp;          // 观测时间戳(秒)
    Eigen::Vector3d position;  // 该时间戳下所有无人机的平均位置
    int num_drones;            // 参与平均的无人机数量

    AveragedObservation(double t, const Eigen::Vector3d& pos, int n) : timestamp(t), position(pos), num_drones(n) {}
};

// 临时存储：用于收集同一时间戳附近的观测
struct TemporaryObservation {
    double timestamp;
    Eigen::Vector3d position;
    int drone_id;

    TemporaryObservation(double t, const Eigen::Vector3d& pos, int id) : timestamp(t), position(pos), drone_id(id) {}
};

/// 相对运动约束: 约束 (pos_j - pos_i) 接近测得的 relative_motion
struct RelativeMotionCostFunctor {
    RelativeMotionCostFunctor(const Eigen::Vector3d& relative_motion, const Eigen::Matrix3d& information) : relative_motion_(relative_motion) {
        sqrt_information_ = information.llt().matrixL();
    }

    template <typename T>
    bool operator()(const T* const pos_i, const T* const pos_j, T* residuals) const {
        Eigen::Matrix<T, 3, 1> pred_motion;
        pred_motion[0] = pos_j[0] - pos_i[0];
        pred_motion[1] = pos_j[1] - pos_i[1];
        pred_motion[2] = pos_j[2] - pos_i[2];

        Eigen::Matrix<T, 3, 1> error = pred_motion - relative_motion_.template cast<T>();
        Eigen::Matrix<T, 3, 1> weighted_error = sqrt_information_.template cast<T>() * error;

        residuals[0] = weighted_error[0];
        residuals[1] = weighted_error[1];
        residuals[2] = weighted_error[2];

        return true;
    }

    static ceres::CostFunction* Create(const Eigen::Vector3d& relative_motion, const Eigen::Matrix3d& information) {
        return new ceres::AutoDiffCostFunction<RelativeMotionCostFunctor, 3, 3, 3>(new RelativeMotionCostFunctor(relative_motion, information));
    }

   private:
    Eigen::Vector3d relative_motion_;
    Eigen::Matrix3d sqrt_information_;
};

/// 先验约束: 固定第一个节点到一个先验位置
struct PriorCostFunctor {
    PriorCostFunctor(const Eigen::Vector3d& prior_pos, const Eigen::Matrix3d& information) : prior_pos_(prior_pos) { sqrt_information_ = information.llt().matrixL(); }

    template <typename T>
    bool operator()(const T* const pos, T* residuals) const {
        Eigen::Matrix<T, 3, 1> error;
        error[0] = pos[0] - T(prior_pos_[0]);
        error[1] = pos[1] - T(prior_pos_[1]);
        error[2] = pos[2] - T(prior_pos_[2]);

        Eigen::Matrix<T, 3, 1> weighted_error = sqrt_information_.template cast<T>() * error;

        residuals[0] = weighted_error[0];
        residuals[1] = weighted_error[1];
        residuals[2] = weighted_error[2];

        return true;
    }

    static ceres::CostFunction* Create(const Eigen::Vector3d& prior_pos, const Eigen::Matrix3d& information) {
        return new ceres::AutoDiffCostFunction<PriorCostFunctor, 3, 3>(new PriorCostFunctor(prior_pos, information));
    }

   private:
    Eigen::Vector3d prior_pos_;
    Eigen::Matrix3d sqrt_information_;
};

/// 单个观测约束
struct ObservationCostFunctor {
    ObservationCostFunctor(const Eigen::Vector3d& observed_pos, const Eigen::Matrix3d& information) : observed_pos_(observed_pos) {
        sqrt_information_ = information.llt().matrixL();
    }

    template <typename T>
    bool operator()(const T* const pos, T* residuals) const {
        Eigen::Matrix<T, 3, 1> error;
        error[0] = pos[0] - T(observed_pos_[0]);
        error[1] = pos[1] - T(observed_pos_[1]);
        error[2] = pos[2] - T(observed_pos_[2]);

        Eigen::Matrix<T, 3, 1> weighted_error = sqrt_information_.template cast<T>() * error;

        residuals[0] = weighted_error[0];
        residuals[1] = weighted_error[1];
        residuals[2] = weighted_error[2];

        return true;
    }

    static ceres::CostFunction* Create(const Eigen::Vector3d& observed_pos, const Eigen::Matrix3d& information) {
        return new ceres::AutoDiffCostFunction<ObservationCostFunctor, 3, 3>(new ObservationCostFunctor(observed_pos, information));
    }

   private:
    Eigen::Vector3d observed_pos_;
    Eigen::Matrix3d sqrt_information_;
};

/// 二阶平滑约束
struct MotionSmoothnessCostFunctor {
    MotionSmoothnessCostFunctor(const Eigen::Matrix3d& information) { sqrt_information_ = information.llt().matrixL(); }

    template <typename T>
    bool operator()(const T* const p_prev, const T* const p_curr, const T* const p_next, T* residuals) const {
        Eigen::Matrix<T, 3, 1> v1, v2, acc;
        v1[0] = p_curr[0] - p_prev[0];
        v1[1] = p_curr[1] - p_prev[1];
        v1[2] = p_curr[2] - p_prev[2];

        v2[0] = p_next[0] - p_curr[0];
        v2[1] = p_next[1] - p_curr[1];
        v2[2] = p_next[2] - p_curr[2];

        acc = v2 - v1;

        Eigen::Matrix<T, 3, 1> weighted = sqrt_information_.template cast<T>() * acc;

        residuals[0] = weighted[0];
        residuals[1] = weighted[1];
        residuals[2] = weighted[2];

        return true;
    }

    static ceres::CostFunction* Create(const Eigen::Matrix3d& information) {
        return new ceres::AutoDiffCostFunction<MotionSmoothnessCostFunctor, 3, 3, 3, 3>(new MotionSmoothnessCostFunctor(information));
    }

   private:
    Eigen::Matrix3d sqrt_information_;
};

/// 时序连续性约束
struct TemporalConsistencyCostFunctor {
    TemporalConsistencyCostFunctor(const Eigen::Vector3d& previous_optimized_pos, const Eigen::Matrix3d& information) : previous_optimized_pos_(previous_optimized_pos) {
        sqrt_information_ = information.llt().matrixL();
    }

    template <typename T>
    bool operator()(const T* const pos, T* residuals) const {
        Eigen::Matrix<T, 3, 1> error;
        error[0] = pos[0] - T(previous_optimized_pos_[0]);
        error[1] = pos[1] - T(previous_optimized_pos_[1]);
        error[2] = pos[2] - T(previous_optimized_pos_[2]);

        Eigen::Matrix<T, 3, 1> weighted_error = sqrt_information_.template cast<T>() * error;

        residuals[0] = weighted_error[0];
        residuals[1] = weighted_error[1];
        residuals[2] = weighted_error[2];

        return true;
    }

    static ceres::CostFunction* Create(const Eigen::Vector3d& previous_optimized_pos, const Eigen::Matrix3d& information) {
        return new ceres::AutoDiffCostFunction<TemporalConsistencyCostFunctor, 3, 3>(new TemporalConsistencyCostFunctor(previous_optimized_pos, information));
    }

   private:
    Eigen::Vector3d previous_optimized_pos_;
    Eigen::Matrix3d sqrt_information_;
};

class MultiDroneTargetEstimator {
   public:
    MultiDroneTargetEstimator(ros::NodeHandle& nh) : nh_(nh) {
        // ========= 基本参数 =========
        nh_.param<int>("window_size", window_size_, 20);
        nh_.param<int>("num_drones", num_drones_, 4);
        nh_.param<double>("publish_rate", publish_rate_, 20.0);

        // 新增：时间戳同步参数
        nh_.param<double>("timestamp_tolerance", timestamp_tolerance_, 0.05);  // 50ms容差

        // 噪声用于模拟原始观测
        nh_.param<double>("noise_std_x", noise_std_x_, 0.0);
        nh_.param<double>("noise_std_y", noise_std_y_, 0.0);
        nh_.param<double>("noise_std_z", noise_std_z_, 0.0);

        // ========= 优化/噪声模型参数 =========
        nh_.param<double>("lambda_mot", lambda_mot_, 800.0);
        nh_.param<double>("huber_delta", huber_delta_, 1.0);
        nh_.param<double>("measurement_noise_std", meas_std_, 0.1);
        nh_.param<double>("motion_noise_std", motion_std_, 0.3);
        nh_.param<bool>("enable_verbose", enable_verbose_, false);

        nh_.param<double>("prior_weight", prior_weight_, 10.0);
        nh_.param<double>("observation_weight", observation_weight_, 10.0);
        nh_.param<double>("relative_motion_base_weight", relative_motion_base_weight_, 2.0);
        nh_.param<int>("relative_motion_max_gap", relative_motion_max_gap_, 3);

        nh_.param<double>("temporal_consistency_weight", temporal_consistency_weight_, 10.0);
        nh_.param<int>("temporal_overlap_size", temporal_overlap_size_, 20);

        // 随机数
        std::random_device rd;
        rng_ = std::mt19937(rd());
        noise_dist_x_ = std::normal_distribution<double>(0.0, noise_std_x_);
        noise_dist_y_ = std::normal_distribution<double>(0.0, noise_std_y_);
        noise_dist_z_ = std::normal_distribution<double>(0.0, noise_std_z_);

        previous_optimized_trajectory_.clear();

        // 订阅每架无人机的目标观测
        for (int i = 0; i < num_drones_; ++i) {
            std::string topic = "/drone_" + std::to_string(i) + "_target";
            drone_subs_.push_back(nh_.subscribe<nav_msgs::Odometry>(topic, 10, boost::bind(&MultiDroneTargetEstimator::droneCallback, this, _1, i)));
            ROS_INFO("Subscribed to %s", topic.c_str());
        }

        target_pub_ = nh_.advertise<nav_msgs::Odometry>("/fxxzxec_estimator_target_target", 10);
        avg_target_pub_ = nh_.advertise<nav_msgs::Odometry>("/all_ave_traget", 10);

        timer_ = nh_.createTimer(ros::Duration(1.0 / publish_rate_), &MultiDroneTargetEstimator::optimizationCallback, this);

        ROS_INFO("Multi-Drone Target Estimator initialized (averaged observations per timestamp)");
        ROS_INFO("Window size: %d, Number of drones: %d, Timestamp tolerance: %.3fs", window_size_, num_drones_, timestamp_tolerance_);
        ROS_INFO("Weights - Prior: %.2f, Obs: %.2f, Lambda_mot: %.2f, Temporal: %.2f", prior_weight_, observation_weight_, lambda_mot_, temporal_consistency_weight_);
    }

   private:
    void droneCallback(const nav_msgs::Odometry::ConstPtr& msg, int drone_id) {
        std::lock_guard<std::mutex> lock(mutex_);

        Eigen::Vector3d original_pos(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);

        Eigen::Vector3d noisy_pos;
        noisy_pos.x() = original_pos.x() + noise_dist_x_(rng_);
        noisy_pos.y() = original_pos.y() + noise_dist_y_(rng_);
        noisy_pos.z() = original_pos.z() + noise_dist_z_(rng_);

        double timestamp = msg->header.stamp.toSec();

        // 将观测加入临时buffer
        temp_observations_.emplace_back(timestamp, noisy_pos, drone_id);

        // 定期清理临时buffer，处理成平均观测
        processTemporaryObservations();
    }

    // 处理临时观测，将同一时间戳（容差内）的观测平均后存入滑动窗口
    void processTemporaryObservations() {
        if (temp_observations_.empty())
            return;

        // 按时间戳排序
        std::sort(temp_observations_.begin(), temp_observations_.end(), [](const TemporaryObservation& a, const TemporaryObservation& b) { return a.timestamp < b.timestamp; });

        // 使用map来分组：key是代表时间戳，value是该时间段内的所有观测
        std::map<double, std::vector<TemporaryObservation>> timestamp_groups;

        for (const auto& obs : temp_observations_) {
            bool found_group = false;

            // 查找是否存在相近的时间戳组
            for (auto& group : timestamp_groups) {
                if (std::abs(group.first - obs.timestamp) <= timestamp_tolerance_) {
                    group.second.push_back(obs);
                    found_group = true;
                    break;
                }
            }

            // 如果没有找到相近的组，创建新组
            if (!found_group) {
                timestamp_groups[obs.timestamp] = {obs};
            }
        }

        // 对每个时间组，检查是否有足够的无人机数据，如果有则计算平均值并加入滑动窗口
        std::vector<double> processed_timestamps;

        for (const auto& group : timestamp_groups) {
            const auto& observations = group.second;

            // 检查是否有来自所有无人机的数据（或至少大部分）
            std::set<int> drone_ids;
            for (const auto& obs : observations) {
                drone_ids.insert(obs.drone_id);
            }

            // 如果这个时间戳有足够多的无人机数据（至少3个），则计算平均
            if ((int)drone_ids.size() >= std::min(3, num_drones_)) {
                Eigen::Vector3d avg_pos = Eigen::Vector3d::Zero();
                double avg_timestamp = 0.0;

                for (const auto& obs : observations) {
                    avg_pos += obs.position;
                    avg_timestamp += obs.timestamp;
                }

                avg_pos /= observations.size();
                avg_timestamp /= observations.size();

                // 添加到滑动窗口
                averaged_observation_buffer_.emplace_back(avg_timestamp, avg_pos, drone_ids.size());

                // 维护滑动窗口大小
                if ((int)averaged_observation_buffer_.size() > window_size_ + 10) {
                    averaged_observation_buffer_.pop_front();
                }

                processed_timestamps.push_back(group.first);

                ROS_DEBUG("Created averaged observation at t=%.3f with %zu drones", avg_timestamp, drone_ids.size());
            }
        }

        // 清理已处理的临时观测
        if (!processed_timestamps.empty()) {
            double oldest_processed = *std::min_element(processed_timestamps.begin(), processed_timestamps.end());

            temp_observations_.erase(std::remove_if(temp_observations_.begin(), temp_observations_.end(),
                                                    [oldest_processed, this](const TemporaryObservation& obs) { return obs.timestamp <= oldest_processed + timestamp_tolerance_; }),
                                     temp_observations_.end());
        }

        // 清理过老的临时观测（超过1秒）
        if (!temp_observations_.empty()) {
            double current_time = temp_observations_.back().timestamp;
            temp_observations_.erase(std::remove_if(temp_observations_.begin(), temp_observations_.end(),
                                                    [current_time](const TemporaryObservation& obs) { return current_time - obs.timestamp > 1.0; }),
                                     temp_observations_.end());
        }
    }

    void optimizationCallback(const ros::TimerEvent&) {
        std::lock_guard<std::mutex> lock(mutex_);

        // 确保有足够的平均观测数据
        if ((int)averaged_observation_buffer_.size() < window_size_) {
            ROS_WARN_THROTTLE(5.0, "Not enough averaged observations: %zu / %d", averaged_observation_buffer_.size(), window_size_);
            return;
        }

        std::vector<Eigen::Vector3d> estimated_trajectory;
        double last_timestamp = 0.0;

        if (optimizeTrajectory(estimated_trajectory, last_timestamp)) {
            publishEstimate(estimated_trajectory.back(), last_timestamp);
            previous_optimized_trajectory_ = estimated_trajectory;
        }

        publishAverageObservation();
    }

    bool optimizeTrajectory(std::vector<Eigen::Vector3d>& trajectory, double& last_timestamp) {
        int N = std::min(window_size_, (int)averaged_observation_buffer_.size());
        if (N < 2)
            return false;

        // 从buffer末尾取最近的N个平均观测
        std::vector<AveragedObservation> window_observations;
        auto it = averaged_observation_buffer_.end() - N;
        for (; it != averaged_observation_buffer_.end(); ++it) {
            window_observations.push_back(*it);
        }

        last_timestamp = window_observations.back().timestamp;

        // ========== 初始化状态 ==========
        std::vector<double*> states(N);

        for (int i = 0; i < N; ++i) {
            states[i] = new double[3];
            const auto& obs = window_observations[i];

            states[i][0] = obs.position[0];
            states[i][1] = obs.position[1];
            states[i][2] = obs.position[2];

            ROS_DEBUG("Node %d init: (%.3f, %.3f, %.3f) from %d drones", i, obs.position[0], obs.position[1], obs.position[2], obs.num_drones);
        }

        ceres::Problem problem;

        ceres::LossFunction* loss = nullptr;
        if (huber_delta_ > 0.0) {
            loss = new ceres::HuberLoss(huber_delta_);
        }

        // ========= 先验约束 =========
        Eigen::Matrix3d prior_info = Eigen::Matrix3d::Identity() * prior_weight_;
        problem.AddResidualBlock(PriorCostFunctor::Create(window_observations[0].position, prior_info), loss, states[0]);

        // ========= 观测约束 =========
        double meas_var = std::max(meas_std_ * meas_std_, 1e-6);
        Eigen::Matrix3d obs_info = Eigen::Matrix3d::Identity() * (observation_weight_ / meas_var);

        for (int i = 0; i < N; ++i) {
            problem.AddResidualBlock(ObservationCostFunctor::Create(window_observations[i].position, obs_info), loss, states[i]);
        }

        // ========= 二阶平滑约束 =========
        double mot_var = std::max(motion_std_ * motion_std_, 1e-6);
        Eigen::Matrix3d mot_info = Eigen::Matrix3d::Identity() * (lambda_mot_ / mot_var);

        if (N >= 3) {
            for (int i = 1; i < N - 1; ++i) {
                problem.AddResidualBlock(MotionSmoothnessCostFunctor::Create(mot_info), nullptr, states[i - 1], states[i], states[i + 1]);
            }
        }

        // ========= 相对运动约束 =========
        int constraint_count = 0;
        for (int i = 0; i < N; ++i) {
            for (int j = i + 1; j < N && j <= i + relative_motion_max_gap_; ++j) {
                Eigen::Vector3d rel = window_observations[j].position - window_observations[i].position;
                int gap = j - i;
                double w = relative_motion_base_weight_ / std::pow(double(gap), 1.0);
                Eigen::Matrix3d edge_info = Eigen::Matrix3d::Identity() * w;

                problem.AddResidualBlock(RelativeMotionCostFunctor::Create(rel, edge_info), nullptr, states[i], states[j]);
                ++constraint_count;
            }
        }

        // ========= 时序连续性约束 =========
        int temporal_constraint_count = 0;
        if (!previous_optimized_trajectory_.empty()) {
            int prev_size = previous_optimized_trajectory_.size();
            int overlap_size = std::min(temporal_overlap_size_, std::min(prev_size, N));

            Eigen::Matrix3d temporal_info = Eigen::Matrix3d::Identity() * temporal_consistency_weight_;

            for (int i = 0; i < overlap_size; ++i) {
                int prev_idx = prev_size - overlap_size + i;
                int curr_idx = i;

                const Eigen::Vector3d& prev_optimized_pos = previous_optimized_trajectory_[prev_idx];
                problem.AddResidualBlock(TemporalConsistencyCostFunctor::Create(prev_optimized_pos, temporal_info), nullptr, states[curr_idx]);
                ++temporal_constraint_count;
            }

            ROS_INFO_THROTTLE(2.0, "Added %d temporal consistency constraints (overlap: %d)", temporal_constraint_count, overlap_size);
        }

        ROS_INFO_THROTTLE(2.0, "Optimization: %d nodes, %d relative constraints, %d temporal constraints", N, constraint_count, temporal_constraint_count);

        // ========= 求解器配置 =========
        ceres::Solver::Options options;
        options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
        options.max_num_iterations = 20;
        options.function_tolerance = 1e-6;
        options.gradient_tolerance = 1e-8;
        options.minimizer_progress_to_stdout = enable_verbose_;

        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);

        if (summary.IsSolutionUsable()) {
            ROS_INFO_THROTTLE(5.0, "Optimization succeeded: cost %.6f -> %.6f, iters = %d", summary.initial_cost, summary.final_cost, (int)summary.iterations.size());
        } else {
            ROS_WARN("Optimization failed: %s", summary.message.c_str());
        }

        trajectory.clear();
        for (int i = 0; i < N; ++i) {
            trajectory.emplace_back(states[i][0], states[i][1], states[i][2]);
        }

        for (auto* ptr : states) {
            delete[] ptr;
        }

        return summary.IsSolutionUsable();
    }

    void publishEstimate(const Eigen::Vector3d& position, double timestamp) {
        nav_msgs::Odometry msg;
        msg.header.stamp = ros::Time(timestamp);
        msg.header.frame_id = "world";

        msg.pose.pose.position.x = position[0];
        msg.pose.pose.position.y = position[1];
        msg.pose.pose.position.z = position[2];

        msg.pose.pose.orientation.w = 1.0;
        msg.pose.pose.orientation.x = 0.0;
        msg.pose.pose.orientation.y = 0.0;
        msg.pose.pose.orientation.z = 0.0;

        target_pub_.publish(msg);
    }

    void publishAverageObservation() {
        if (averaged_observation_buffer_.empty())
            return;

        const auto& latest = averaged_observation_buffer_.back();

        nav_msgs::Odometry msg;
        msg.header.stamp = ros::Time(latest.timestamp);
        msg.header.frame_id = "world";

        msg.pose.pose.position.x = latest.position[0];
        msg.pose.pose.position.y = latest.position[1];
        msg.pose.pose.position.z = latest.position[2];

        msg.pose.pose.orientation.w = 1.0;
        msg.pose.pose.orientation.x = 0.0;
        msg.pose.pose.orientation.y = 0.0;
        msg.pose.pose.orientation.z = 0.0;

        avg_target_pub_.publish(msg);
    }

   private:
    ros::NodeHandle nh_;
    std::vector<ros::Subscriber> drone_subs_;
    ros::Publisher target_pub_;
    ros::Publisher avg_target_pub_;
    ros::Timer timer_;

    // 新的数据结构
    std::vector<TemporaryObservation> temp_observations_;          // 临时存储原始观测
    std::deque<AveragedObservation> averaged_observation_buffer_;  // 滑动窗口：存储平均观测
    std::mutex mutex_;

    // 参数
    int window_size_;
    int num_drones_;
    double publish_rate_;
    double timestamp_tolerance_;  // 时间戳容差

    double noise_std_x_;
    double noise_std_y_;
    double noise_std_z_;
    std::mt19937 rng_;
    std::normal_distribution<double> noise_dist_x_;
    std::normal_distribution<double> noise_dist_y_;
    std::normal_distribution<double> noise_dist_z_;

    double lambda_mot_;
    double huber_delta_;
    double meas_std_;
    double motion_std_;
    bool enable_verbose_;

    double prior_weight_;
    double observation_weight_;
    double relative_motion_base_weight_;
    int relative_motion_max_gap_;

    double temporal_consistency_weight_;
    int temporal_overlap_size_;
    std::vector<Eigen::Vector3d> previous_optimized_trajectory_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "multi_drone_target_estimator");
    ros::NodeHandle nh("~");

    MultiDroneTargetEstimator estimator(nh);

    ros::spin();
    return 0;
}