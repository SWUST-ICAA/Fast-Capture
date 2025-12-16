#include <nav_msgs/Odometry.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <Eigen/Dense>
#include <mutex>
#include <vector>

#include "roundup_formation/roundup_optimizer.h"

using namespace roundup_formation;

class RoundupFormationNode {
   public:
    RoundupFormationNode(ros::NodeHandle& nh, ros::NodeHandle& pnh) : nh_(nh), pnh_(pnh) {
        // 加载参数
        loadParams();

        // 初始化优化器
        optimizer_.setParams(params_);

        // 初始化数据
        drone_positions_.resize(NUM_DRONES);
        drone_received_.resize(NUM_DRONES, false);

        // 订阅无人机位置
        for (int i = 0; i < NUM_DRONES; ++i) {
            std::string topic = "/drone_" + std::to_string(i) + "_visual_slam/odom";
            drone_subs_.push_back(nh_.subscribe<nav_msgs::Odometry>(topic, 10, boost::bind(&RoundupFormationNode::droneOdomCallback, this, _1, i)));
            ROS_INFO("Subscribed to: %s", topic.c_str());
        }

        // 订阅目标位置
        // target_sub_ = nh_.subscribe("/fxxzxec_estimator_target_target", 10, &RoundupFormationNode::targetCallback, this);
        target_sub_ = nh_.subscribe("/mean_predict_target", 10, &RoundupFormationNode::targetCallback, this);
        ROS_INFO("Subscribed to: /mean_predict_target");

        // 订阅点云
        pointcloud_sub_ = nh_.subscribe("/global_map", 1, &RoundupFormationNode::pointcloudCallback, this);
        ROS_INFO("Subscribed to: /global_map");

        // 发布围捕目标点
        for (int i = 0; i < NUM_DRONES; ++i) {
            std::string topic = "/drone_" + std::to_string(i) + "_roundup_target";
            target_pubs_.push_back(nh_.advertise<nav_msgs::Odometry>(topic, 10));
            ROS_INFO("Publishing to: %s", topic.c_str());
        }

        // 发布质心位置
        centroid_pub_ = nh_.advertise<nav_msgs::Odometry>("/mean_center_target", 10);
        ROS_INFO("Publishing to: /mean_center_target");

        // 定时器
        double update_rate;
        pnh_.param<double>("update_rate", update_rate, 10.0);
        timer_ = nh_.createTimer(ros::Duration(1.0 / update_rate), &RoundupFormationNode::timerCallback, this);

        ROS_INFO("Roundup Formation Node initialized with update rate: %.1f Hz", update_rate);
    }

   private:
    void loadParams() {
        pnh_.param<double>("w_fit", params_.w_fit, 1.0);
        pnh_.param<double>("w_dist", params_.w_dist, -0.5);
        pnh_.param<double>("w_obs", params_.w_obs, 10.0);
        pnh_.param<double>("w_col", params_.w_col, 10.0);
        pnh_.param<double>("w_reg", params_.w_reg, 0.1);

        pnh_.param<double>("R_min", params_.R_min, 3.0);
        pnh_.param<double>("R_max", params_.R_max, 20.0);
        pnh_.param<double>("d_obs", params_.d_obs, 1.5);
        pnh_.param<double>("d_col", params_.d_col, 1.0);

        pnh_.param<int>("max_iterations", params_.max_iterations, 100);
        pnh_.param<double>("function_tolerance", params_.function_tolerance, 1e-6);

        ROS_INFO("Parameters loaded:");
        ROS_INFO("  w_fit: %.2f, w_dist: %.2f, w_obs: %.2f, w_col: %.2f, w_reg: %.2f", params_.w_fit, params_.w_dist, params_.w_obs, params_.w_col, params_.w_reg);
        ROS_INFO("  R_min: %.2f, R_max: %.2f, d_obs: %.2f, d_col: %.2f", params_.R_min, params_.R_max, params_.d_obs, params_.d_col);
    }

    void droneOdomCallback(const nav_msgs::Odometry::ConstPtr& msg, int drone_id) {
        std::lock_guard<std::mutex> lock(data_mutex_);

        drone_positions_[drone_id] = Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
        drone_received_[drone_id] = true;

        ROS_DEBUG("Drone %d position: (%.2f, %.2f, %.2f)", drone_id, drone_positions_[drone_id](0), drone_positions_[drone_id](1), drone_positions_[drone_id](2));
    }

    void targetCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);

        target_position_ = Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
        target_received_ = true;

        ROS_DEBUG("Target position: (%.2f, %.2f, %.2f)", target_position_(0), target_position_(1), target_position_(2));
    }

    void pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
        std::lock_guard<std::mutex> lock(cloud_mutex_);

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(*msg, *cloud);

        optimizer_.updateObstacles(cloud);

        ROS_DEBUG("Received point cloud with %zu points", cloud->size());
    }

    void timerCallback(const ros::TimerEvent& event) {
        // 检查数据是否就绪
        bool all_drones_ready = true;
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            for (int i = 0; i < NUM_DRONES; ++i) {
                if (!drone_received_[i]) {
                    all_drones_ready = false;
                    break;
                }
            }
        }

        if (!all_drones_ready) {
            ROS_WARN_THROTTLE(5.0, "Waiting for all drone positions...");
            return;
        }

        if (!target_received_) {
            ROS_WARN_THROTTLE(5.0, "Waiting for target position...");
            return;
        }

        // 更新优化器数据
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            optimizer_.updateDronePositions(drone_positions_);
            optimizer_.updateTargetPosition(target_position_);
        }

        // 执行优化
        std::vector<Eigen::Vector3d> capture_points = optimizer_.optimize();

        // 获取质心
        Eigen::Vector3d centroid = optimizer_.getCentroid();

        // 发布围捕目标点
        ros::Time now = ros::Time::now();
        for (int i = 0; i < NUM_DRONES; ++i) {
            nav_msgs::Odometry target_msg;
            target_msg.header.stamp = now;
            target_msg.header.frame_id = "world";
            target_msg.child_frame_id = "drone_" + std::to_string(i) + "_target";

            target_msg.pose.pose.position.x = capture_points[i](0);
            target_msg.pose.pose.position.y = capture_points[i](1);
            target_msg.pose.pose.position.z = capture_points[i](2);

            // 设置朝向目标的方向
            double yaw = std::atan2(centroid(1) - capture_points[i](1), centroid(0) - capture_points[i](0));
            target_msg.pose.pose.orientation.x = 0;
            target_msg.pose.pose.orientation.y = 0;
            target_msg.pose.pose.orientation.z = std::sin(yaw / 2.0);
            target_msg.pose.pose.orientation.w = std::cos(yaw / 2.0);

            target_pubs_[i].publish(target_msg);
        }

        // 发布质心
        nav_msgs::Odometry centroid_msg;
        centroid_msg.header.stamp = now;
        centroid_msg.header.frame_id = "world";
        centroid_msg.child_frame_id = "centroid";

        centroid_msg.pose.pose.position.x = centroid(0);
        centroid_msg.pose.pose.position.y = centroid(1);
        centroid_msg.pose.pose.position.z = centroid(2);
        centroid_msg.pose.pose.orientation.w = 1.0;

        centroid_pub_.publish(centroid_msg);

        ROS_DEBUG("Published capture points and centroid");
    }

   private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    // 订阅者
    std::vector<ros::Subscriber> drone_subs_;
    ros::Subscriber target_sub_;
    ros::Subscriber pointcloud_sub_;

    // 发布者
    std::vector<ros::Publisher> target_pubs_;
    ros::Publisher centroid_pub_;

    // 定时器
    ros::Timer timer_;

    // 优化器
    RoundupOptimizer optimizer_;
    OptimizerParams params_;

    // 数据
    std::vector<Eigen::Vector3d> drone_positions_;
    std::vector<bool> drone_received_;
    Eigen::Vector3d target_position_;
    bool target_received_ = false;

    // 互斥锁
    std::mutex data_mutex_;
    std::mutex cloud_mutex_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "roundup_formation_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    RoundupFormationNode node(nh, pnh);

    ros::spin();

    return 0;
}