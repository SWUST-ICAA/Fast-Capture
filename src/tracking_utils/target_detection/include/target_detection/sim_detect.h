#ifndef _SIM_DETECT_H
#define _SIM_DETECT_H
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <Eigen/Dense>
#include <Eigen/Eigen>
#include <Eigen/StdVector>
#include <boost/functional/hash.hpp>
#include <deque>
#include <iostream>
#include <map>
#include <memory>
#include <mutex>
#include <queue>
#include <random>
#include <string>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

class Sim_detect {
   private:
    // 地图相关参数
    Eigen::Vector3d origin_;              // 地图原点坐标（左下角）
    Eigen::Vector3d map_size_3d_;         // 地图三维尺寸（x, y, z）
    Eigen::Vector3d dim_;                 // 每个维度的网格数
    std::vector<signed char> map_buffer;  // 地图数据缓冲区（0/1表示空闲/占用）
    double resolution;                    // 网格分辨率（单位：米）
    int buffer_size;                      // 缓冲区总大小
    bool has_map = false;                 // 地图是否已构建

    // ROS 通信
    ros::Publisher detection_pub;  // 发布检测到的目标（话题名：target）
    ros::Subscriber map_sub;       // 订阅全局地图（话题名：global_map）
    ros::Subscriber target_sub;    // 订阅车辆状态（话题名：car_state_detect）
    ros::Subscriber drone_sub;     // 订阅无人机状态（话题名：drone_odom）
    ros::Publisher detection_if;

    Eigen::Vector3d drone_pos;  // 无人机当前位置

    // 高斯噪声相关
    double noise_std_x_;
    double noise_std_y_;
    double noise_std_z_;
    std::mt19937 rng_;
    std::normal_distribution<double> noise_dist_x_;
    std::normal_distribution<double> noise_dist_y_;
    std::normal_distribution<double> noise_dist_z_;

   public:
    Sim_detect(ros::NodeHandle& nh);  // 构造函数
    ~Sim_detect() {};                 // 析构函数

    // 地图构建与坐标转换
    void GlobalMapBuild(const sensor_msgs::PointCloud2& pointcloud_map);
    void setObs(Eigen::Vector3d pt);                        // 设置障碍物
    bool isOutside(Eigen::Vector3i idx);                    // 检查索引是否越界
    int getIndex(Eigen::Vector3i idx);                      // 三维索引转一维数组下标
    bool isOccupied(Eigen::Vector3i idx);                   // 检查网格是否被占用
    Eigen::Vector3d intToFloat(const Eigen::Vector3i idx);  // 网格索引转实际坐标
    Eigen::Vector3i FloatToint(const Eigen::Vector3d pt);   // 实际坐标转网格索引

    // 核心功能
    bool is_block(Eigen::Vector3d pos1, Eigen::Vector3d pos2);  // 检测两点间是否被阻挡
    void car_state_cb(const nav_msgs::Odometry& car_state);     // 车辆状态回调
    void drone_odom_cb(const nav_msgs::Odometry& odom);         // 无人机状态回调
};
#endif