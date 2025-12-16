#ifndef ROUNDUP_OPTIMIZER_H
#define ROUNDUP_OPTIMIZER_H

#include <ceres/ceres.h>
#include <nav_msgs/Odometry.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <Eigen/Dense>
#include <memory>
#include <vector>

namespace roundup_formation {

// 无人机数量
constexpr int NUM_DRONES = 4;

/**
 * @brief 围捕编队优化器参数结构
 */
struct OptimizerParams {
    // 权重参数
    double w_fit = 1.0;    // 位置拟合权重
    double w_dist = -0.5;  // 距离正则化权重 (负值鼓励紧密包围)
    double w_obs = 10.0;   // 障碍物避障权重
    double w_col = 10.0;   // 碰撞避免权重
    double w_reg = 0.1;    // 编队规则性权重

    // 约束参数
    double R_min = 3.2;   // 最小捕获半径 (m)
    double R_max = 20.0;  // 最大捕获半径 (m)
    double d_obs = 1.5;   // 障碍物安全距离 (m)
    double d_col = 1.0;   // 无人机间最小间距 (m)

    // 优化器参数
    int max_iterations = 100;
    double function_tolerance = 1e-6;
    double gradient_tolerance = 1e-10;
    double parameter_tolerance = 1e-8;
};

/**
 * @brief 位置拟合代价函数 J_fit
 * 衡量实际无人机位置与期望捕获点之间的偏差
 */
struct FittingCostFunctor {
    FittingCostFunctor(const Eigen::Vector2d& uav_pos, const Eigen::Vector2d& centroid, double weight) : uav_pos_(uav_pos), centroid_(centroid), weight_(weight) {}

    template <typename T>
    bool operator()(const T* const theta, const T* const R, T* residual) const {
        // 期望捕获点: P_j* = C + R * [cos(theta), sin(theta)]^T
        T desired_x = T(centroid_(0)) + R[0] * ceres::cos(theta[0]);
        T desired_y = T(centroid_(1)) + R[0] * ceres::sin(theta[0]);

        // 残差: sqrt(w) * (P_actual - P_desired)
        T sqrt_weight = ceres::sqrt(T(weight_));
        residual[0] = sqrt_weight * (T(uav_pos_(0)) - desired_x);
        residual[1] = sqrt_weight * (T(uav_pos_(1)) - desired_y);

        return true;
    }

    static ceres::CostFunction* Create(const Eigen::Vector2d& uav_pos, const Eigen::Vector2d& centroid, double weight) {
        return new ceres::AutoDiffCostFunction<FittingCostFunctor, 2, 1, 1>(new FittingCostFunctor(uav_pos, centroid, weight));
    }

   private:
    Eigen::Vector2d uav_pos_;
    Eigen::Vector2d centroid_;
    double weight_;
};

/**
 * @brief 距离正则化代价函数 J_dist
 * 鼓励捕获半径接近最小安全半径
 */
struct DistanceRegularizationCostFunctor {
    DistanceRegularizationCostFunctor(double R_min, double weight) : R_min_(R_min), weight_(weight) {}

    template <typename T>
    bool operator()(const T* const R, T* residual) const {
        // J_dist = (R - R_min)^2
        // 注意: weight可能为负值，需要特殊处理
        T diff = R[0] - T(R_min_);
        if (weight_ >= 0) {
            residual[0] = ceres::sqrt(T(weight_)) * diff;
        } else {
            // 负权重表示奖励，转换为等效形式
            residual[0] = ceres::sqrt(T(-weight_)) * diff;
        }
        return true;
    }

    static ceres::CostFunction* Create(double R_min, double weight) {
        return new ceres::AutoDiffCostFunction<DistanceRegularizationCostFunctor, 1, 1>(new DistanceRegularizationCostFunctor(R_min, weight));
    }

   private:
    double R_min_;
    double weight_;
};

/**
 * @brief 障碍物避障代价函数 J_obs
 * 使用铰链型惩罚函数
 */
struct ObstacleAvoidanceCostFunctor {
    ObstacleAvoidanceCostFunctor(const Eigen::Vector2d& centroid, const std::vector<Eigen::Vector2d>& obstacle_points, double d_obs, double weight)
        : centroid_(centroid), obstacle_points_(obstacle_points), d_obs_(d_obs), weight_(weight) {}

    template <typename T>
    bool operator()(const T* const theta, const T* const R, T* residual) const {
        // 期望捕获点
        T capture_x = T(centroid_(0)) + R[0] * ceres::cos(theta[0]);
        T capture_y = T(centroid_(1)) + R[0] * ceres::sin(theta[0]);

        // 累计所有障碍物的惩罚
        T total_penalty = T(0.0);

        for (const auto& obs_pt : obstacle_points_) {
            T dx = capture_x - T(obs_pt(0));
            T dy = capture_y - T(obs_pt(1));
            T dist = ceres::sqrt(dx * dx + dy * dy + T(1e-6));

            // 铰链惩罚: max(0, d_obs - d)^2
            T violation = T(d_obs_) - dist;
            if (violation > T(0.0)) {
                total_penalty += violation * violation;
            }
        }

        residual[0] = ceres::sqrt(T(weight_)) * ceres::sqrt(total_penalty + T(1e-10));
        return true;
    }

    static ceres::CostFunction* Create(const Eigen::Vector2d& centroid, const std::vector<Eigen::Vector2d>& obstacle_points, double d_obs, double weight) {
        return new ceres::AutoDiffCostFunction<ObstacleAvoidanceCostFunctor, 1, 1, 1>(new ObstacleAvoidanceCostFunctor(centroid, obstacle_points, d_obs, weight));
    }

   private:
    Eigen::Vector2d centroid_;
    std::vector<Eigen::Vector2d> obstacle_points_;
    double d_obs_;
    double weight_;
};

/**
 * @brief 群内碰撞避免代价函数 J_col
 * 惩罚捕获点之间过近的距离
 */
struct CollisionAvoidanceCostFunctor {
    CollisionAvoidanceCostFunctor(const Eigen::Vector2d& centroid, double d_col, double weight) : centroid_(centroid), d_col_(d_col), weight_(weight) {}

    template <typename T>
    bool operator()(const T* const theta_j, const T* const theta_l, const T* const R, T* residual) const {
        // 两个捕获点
        T p_j_x = T(centroid_(0)) + R[0] * ceres::cos(theta_j[0]);
        T p_j_y = T(centroid_(1)) + R[0] * ceres::sin(theta_j[0]);
        T p_l_x = T(centroid_(0)) + R[0] * ceres::cos(theta_l[0]);
        T p_l_y = T(centroid_(1)) + R[0] * ceres::sin(theta_l[0]);

        // 两点之间的距离
        T dx = p_j_x - p_l_x;
        T dy = p_j_y - p_l_y;
        T dist = ceres::sqrt(dx * dx + dy * dy + T(1e-6));

        // 铰链惩罚: max(0, d_col - d)^2
        T violation = T(d_col_) - dist;
        if (violation > T(0.0)) {
            residual[0] = ceres::sqrt(T(weight_)) * violation;
        } else {
            residual[0] = T(0.0);
        }

        return true;
    }

    static ceres::CostFunction* Create(const Eigen::Vector2d& centroid, double d_col, double weight) {
        return new ceres::AutoDiffCostFunction<CollisionAvoidanceCostFunctor, 1, 1, 1, 1>(new CollisionAvoidanceCostFunctor(centroid, d_col, weight));
    }

   private:
    Eigen::Vector2d centroid_;
    double d_col_;
    double weight_;
};

/**
 * @brief 编队规则性代价函数 J_reg
 * 鼓励角度均匀分布
 */
struct RegularityCostFunctor {
    RegularityCostFunctor(int num_drones, double weight) : num_drones_(num_drones), weight_(weight) { ideal_gap_ = 2.0 * M_PI / num_drones_; }

    template <typename T>
    bool operator()(const T* const theta_j, const T* const theta_jp1, T* residual) const {
        // 角度间隙
        T gap = theta_jp1[0] - theta_j[0];

        // 处理周期性 (确保gap在[0, 2*pi]范围内)
        // 简化处理：直接计算与理想间隙的差异
        T deviation = gap - T(ideal_gap_);

        residual[0] = ceres::sqrt(T(weight_)) * deviation;

        return true;
    }

    static ceres::CostFunction* Create(int num_drones, double weight) {
        return new ceres::AutoDiffCostFunction<RegularityCostFunctor, 1, 1, 1>(new RegularityCostFunctor(num_drones, weight));
    }

   private:
    int num_drones_;
    double ideal_gap_;
    double weight_;
};

/**
 * @brief 最后一个角度间隙的规则性代价 (theta_1 + 2*pi - theta_N)
 */
struct RegularityWrapCostFunctor {
    RegularityWrapCostFunctor(int num_drones, double weight) : num_drones_(num_drones), weight_(weight) { ideal_gap_ = 2.0 * M_PI / num_drones_; }

    template <typename T>
    bool operator()(const T* const theta_1, const T* const theta_N, T* residual) const {
        // 环绕间隙: theta_1 + 2*pi - theta_N
        T gap = theta_1[0] + T(2.0 * M_PI) - theta_N[0];
        T deviation = gap - T(ideal_gap_);

        residual[0] = ceres::sqrt(T(weight_)) * deviation;

        return true;
    }

    static ceres::CostFunction* Create(int num_drones, double weight) {
        return new ceres::AutoDiffCostFunction<RegularityWrapCostFunctor, 1, 1, 1>(new RegularityWrapCostFunctor(num_drones, weight));
    }

   private:
    int num_drones_;
    double ideal_gap_;
    double weight_;
};

/**
 * @brief 围捕编队优化器类
 */
class RoundupOptimizer {
   public:
    RoundupOptimizer();
    ~RoundupOptimizer() = default;

    /**
     * @brief 设置优化参数
     */
    void setParams(const OptimizerParams& params);

    /**
     * @brief 更新无人机位置
     */
    void updateDronePositions(const std::vector<Eigen::Vector3d>& positions);

    /**
     * @brief 更新目标位置
     */
    void updateTargetPosition(const Eigen::Vector3d& target_pos);

    /**
     * @brief 更新障碍物点云
     */
    void updateObstacles(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);

    /**
     * @brief 执行优化，计算围捕点
     * @return 优化后的围捕点位置
     */
    std::vector<Eigen::Vector3d> optimize();

    /**
     * @brief 获取围捕无人机质心
     */
    Eigen::Vector3d getCentroid() const;

    /**
     * @brief 检查数据是否就绪
     */
    bool isDataReady() const;

   private:
    /**
     * @brief 从点云中提取附近的障碍物点
     */
    std::vector<Eigen::Vector2d> extractNearbyObstacles(const Eigen::Vector2d& center, double radius);

    /**
     * @brief 初始化角度参数
     */
    void initializeAngles(std::vector<double>& thetas, double& R);

   private:
    OptimizerParams params_;

    // 无人机位置
    std::vector<Eigen::Vector3d> drone_positions_;
    bool drone_positions_valid_ = false;

    // 目标位置
    Eigen::Vector3d target_position_;
    bool target_position_valid_ = false;

    // 障碍物点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr obstacle_cloud_;
    pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtree_;
    bool obstacles_valid_ = false;

    // 优化结果
    std::vector<double> optimal_thetas_;
    double optimal_R_;
};

}  // namespace roundup_formation

#endif  // ROUNDUP_OPTIMIZER_H