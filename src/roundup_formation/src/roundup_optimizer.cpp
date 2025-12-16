#include "roundup_formation/roundup_optimizer.h"
#include <algorithm>
#include <cmath>

namespace roundup_formation {

RoundupOptimizer::RoundupOptimizer() : obstacle_cloud_(new pcl::PointCloud<pcl::PointXYZ>()), kdtree_(new pcl::KdTreeFLANN<pcl::PointXYZ>()) {
    drone_positions_.resize(NUM_DRONES);
    optimal_thetas_.resize(NUM_DRONES);
    optimal_R_ = params_.R_min;
}

void RoundupOptimizer::setParams(const OptimizerParams& params) {
    params_ = params;
}

void RoundupOptimizer::updateDronePositions(const std::vector<Eigen::Vector3d>& positions) {
    if (positions.size() != NUM_DRONES) {
        ROS_WARN("Expected %d drone positions, got %zu", NUM_DRONES, positions.size());
        return;
    }
    drone_positions_ = positions;
    drone_positions_valid_ = true;
}

void RoundupOptimizer::updateTargetPosition(const Eigen::Vector3d& target_pos) {
    target_position_ = target_pos;
    target_position_valid_ = true;
}

void RoundupOptimizer::updateObstacles(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
    if (cloud && !cloud->empty()) {
        *obstacle_cloud_ = *cloud;
        kdtree_->setInputCloud(obstacle_cloud_);
        obstacles_valid_ = true;
    }
}

bool RoundupOptimizer::isDataReady() const {
    return drone_positions_valid_ && target_position_valid_;
}

Eigen::Vector3d RoundupOptimizer::getCentroid() const {
    // 使用目标位置作为围捕中心
    return target_position_;
}

std::vector<Eigen::Vector2d> RoundupOptimizer::extractNearbyObstacles(const Eigen::Vector2d& center, double radius) {
    std::vector<Eigen::Vector2d> nearby_obstacles;

    if (!obstacles_valid_ || obstacle_cloud_->empty()) {
        return nearby_obstacles;
    }

    // 使用KD树搜索附近的障碍物点
    pcl::PointXYZ search_point;
    search_point.x = center(0);
    search_point.y = center(1);
    search_point.z = target_position_(2);

    std::vector<int> point_indices;
    std::vector<float> point_distances;

    // 搜索半径范围内的所有点
    double search_radius = radius + params_.d_obs * 2.0;
    kdtree_->radiusSearch(search_point, search_radius, point_indices, point_distances);

    // 提取2D障碍物点 (投影到XY平面)
    for (int idx : point_indices) {
        const auto& pt = obstacle_cloud_->points[idx];
        // 只考虑高度接近目标高度的障碍物
        if (std::abs(pt.z - target_position_(2)) < 2.0) {
            nearby_obstacles.emplace_back(pt.x, pt.y);
        }
    }

    // 下采样以减少计算量
    const size_t max_obstacles = 100;
    if (nearby_obstacles.size() > max_obstacles) {
        std::vector<Eigen::Vector2d> downsampled;
        size_t step = nearby_obstacles.size() / max_obstacles;
        for (size_t i = 0; i < nearby_obstacles.size(); i += step) {
            downsampled.push_back(nearby_obstacles[i]);
        }
        return downsampled;
    }

    return nearby_obstacles;
}

void RoundupOptimizer::initializeAngles(std::vector<double>& thetas, double& R) {
    Eigen::Vector2d centroid(target_position_(0), target_position_(1));

    // 根据当前无人机位置初始化角度
    for (int j = 0; j < NUM_DRONES; ++j) {
        Eigen::Vector2d uav_pos(drone_positions_[j](0), drone_positions_[j](1));
        Eigen::Vector2d diff = uav_pos - centroid;
        thetas[j] = std::atan2(diff(1), diff(0));

        // 确保角度在[0, 2*pi)范围内
        if (thetas[j] < 0) {
            thetas[j] += 2.0 * M_PI;
        }
    }

    // 按角度排序，确保顺序一致
    std::vector<std::pair<double, int>> angle_indices;
    for (int j = 0; j < NUM_DRONES; ++j) {
        angle_indices.emplace_back(thetas[j], j);
    }
    std::sort(angle_indices.begin(), angle_indices.end());

    std::vector<double> sorted_thetas(NUM_DRONES);
    for (int j = 0; j < NUM_DRONES; ++j) {
        sorted_thetas[j] = angle_indices[j].first;
    }
    thetas = sorted_thetas;

    // 初始化半径为当前平均距离
    double avg_dist = 0.0;
    for (int j = 0; j < NUM_DRONES; ++j) {
        Eigen::Vector2d uav_pos(drone_positions_[j](0), drone_positions_[j](1));
        avg_dist += (uav_pos - centroid).norm();
    }
    avg_dist /= NUM_DRONES;

    R = std::max(params_.R_min, std::min(avg_dist, params_.R_max));
}

std::vector<Eigen::Vector3d> RoundupOptimizer::optimize() {
    std::vector<Eigen::Vector3d> capture_points(NUM_DRONES);

    if (!isDataReady()) {
        ROS_WARN("Data not ready for optimization");
        return capture_points;
    }

    // 质心 (使用目标位置)
    Eigen::Vector2d centroid(target_position_(0), target_position_(1));

    // 初始化优化变量
    std::vector<double> thetas(NUM_DRONES);
    double R;
    initializeAngles(thetas, R);

    // 提取附近障碍物
    std::vector<Eigen::Vector2d> nearby_obstacles = extractNearbyObstacles(centroid, params_.R_max);

    // 构建Ceres问题
    ceres::Problem problem;

    // 添加位置拟合代价 J_fit
    for (int j = 0; j < NUM_DRONES; ++j) {
        Eigen::Vector2d uav_pos(drone_positions_[j](0), drone_positions_[j](1));

        ceres::CostFunction* fitting_cost = FittingCostFunctor::Create(uav_pos, centroid, params_.w_fit);
        problem.AddResidualBlock(fitting_cost, nullptr, &thetas[j], &R);
    }

    // 添加距离正则化代价 J_dist
    ceres::CostFunction* dist_cost = DistanceRegularizationCostFunctor::Create(params_.R_min, params_.w_dist);
    problem.AddResidualBlock(dist_cost, nullptr, &R);

    // 添加障碍物避障代价 J_obs
    if (!nearby_obstacles.empty()) {
        for (int j = 0; j < NUM_DRONES; ++j) {
            ceres::CostFunction* obs_cost = ObstacleAvoidanceCostFunctor::Create(centroid, nearby_obstacles, params_.d_obs, params_.w_obs);
            problem.AddResidualBlock(obs_cost, nullptr, &thetas[j], &R);
        }
    }

    // 添加碰撞避免代价 J_col
    for (int j = 0; j < NUM_DRONES; ++j) {
        for (int l = j + 1; l < NUM_DRONES; ++l) {
            ceres::CostFunction* col_cost = CollisionAvoidanceCostFunctor::Create(centroid, params_.d_col, params_.w_col);
            problem.AddResidualBlock(col_cost, nullptr, &thetas[j], &thetas[l], &R);
        }
    }

    // 添加编队规则性代价 J_reg
    for (int j = 0; j < NUM_DRONES - 1; ++j) {
        ceres::CostFunction* reg_cost = RegularityCostFunctor::Create(NUM_DRONES, params_.w_reg);
        problem.AddResidualBlock(reg_cost, nullptr, &thetas[j], &thetas[j + 1]);
    }

    // 最后一个环绕间隙 (theta_1 + 2*pi - theta_N)
    ceres::CostFunction* reg_wrap_cost = RegularityWrapCostFunctor::Create(NUM_DRONES, params_.w_reg);
    problem.AddResidualBlock(reg_wrap_cost, nullptr, &thetas[0], &thetas[NUM_DRONES - 1]);

    // 设置参数边界
    // R的边界
    problem.SetParameterLowerBound(&R, 0, params_.R_min);
    problem.SetParameterUpperBound(&R, 0, params_.R_max);

    // theta的边界 [0, 2*pi)
    for (int j = 0; j < NUM_DRONES; ++j) {
        problem.SetParameterLowerBound(&thetas[j], 0, 0.0);
        problem.SetParameterUpperBound(&thetas[j], 0, 2.0 * M_PI);
    }

    // 配置求解器
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = false;
    options.max_num_iterations = params_.max_iterations;
    options.function_tolerance = params_.function_tolerance;
    options.gradient_tolerance = params_.gradient_tolerance;
    options.parameter_tolerance = params_.parameter_tolerance;

    // 求解
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    if (summary.termination_type == ceres::CONVERGENCE || summary.termination_type == ceres::USER_SUCCESS) {
        ROS_DEBUG("Optimization converged. Final cost: %f", summary.final_cost);
    } else {
        ROS_WARN("Optimization may not have converged. Status: %s", summary.BriefReport().c_str());
    }

    // 保存优化结果
    optimal_thetas_ = thetas;
    optimal_R_ = R;

    // 计算捕获点
    for (int j = 0; j < NUM_DRONES; ++j) {
        capture_points[j](0) = centroid(0) + R * std::cos(thetas[j]);
        capture_points[j](1) = centroid(1) + R * std::sin(thetas[j]);
        capture_points[j](2) = target_position_(2);  // 保持与目标相同的高度
    }

    ROS_INFO("Optimal R: %.2f, Thetas: [%.2f, %.2f, %.2f, %.2f] deg", R, thetas[0] * 180.0 / M_PI, thetas[1] * 180.0 / M_PI, thetas[2] * 180.0 / M_PI, thetas[3] * 180.0 / M_PI);

    return capture_points;
}

}  // namespace roundup_formation