#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <map>
#include <string>
#include <vector>

class MultiTopicVisualizer {
   public:
    MultiTopicVisualizer() : nh_("~"), snapshot_count_(0) {
        // 获取统一的frame_id参数(默认为world)
        nh_.param<std::string>("global_frame_id", global_frame_id_, "world");

        // ============ 第一部分：订阅Odometry话题用于轨迹显示 ============
        sub_car_state_ = nh_.subscribe("/car_state", 10, &MultiTopicVisualizer::carStateCallback, this);
        sub_drone0_odom_ = nh_.subscribe("/drone_0_visual_slam/odom", 10, &MultiTopicVisualizer::drone0OdomCallback, this);
        sub_drone1_odom_ = nh_.subscribe("/drone_1_visual_slam/odom", 10, &MultiTopicVisualizer::drone1OdomCallback, this);
        sub_drone2_odom_ = nh_.subscribe("/drone_2_visual_slam/odom", 10, &MultiTopicVisualizer::drone2OdomCallback, this);
        sub_drone3_odom_ = nh_.subscribe("/drone_3_visual_slam/odom", 10, &MultiTopicVisualizer::drone3OdomCallback, this);

        // 发布历史轨迹
        pub_trajectories_ = nh_.advertise<visualization_msgs::MarkerArray>("/history_trajectories", 10);

        // ============ 第二部分：订阅需要定时记录的话题 ============
        // PointCloud2
        sub_drone0_cloud_ = nh_.subscribe("/drone_0_grid_map/occupancy_inflate", 10, &MultiTopicVisualizer::drone0CloudCallback, this);
        sub_drone1_cloud_ = nh_.subscribe("/drone_1_grid_map/occupancy_inflate", 10, &MultiTopicVisualizer::drone1CloudCallback, this);
        sub_drone2_cloud_ = nh_.subscribe("/drone_2_grid_map/occupancy_inflate", 10, &MultiTopicVisualizer::drone2CloudCallback, this);
        sub_drone3_cloud_ = nh_.subscribe("/drone_3_grid_map/occupancy_inflate", 10, &MultiTopicVisualizer::drone3CloudCallback, this);

        // Marker
        sub_car_model_ = nh_.subscribe("/visualization/car_model", 10, &MultiTopicVisualizer::carModelCallback, this);
        sub_drone0_robot_ = nh_.subscribe("/drone_0_odom_visualization/robot", 10, &MultiTopicVisualizer::drone0RobotCallback, this);
        sub_drone1_robot_ = nh_.subscribe("/drone_1_odom_visualization/robot", 10, &MultiTopicVisualizer::drone1RobotCallback, this);
        sub_drone2_robot_ = nh_.subscribe("/drone_2_odom_visualization/robot", 10, &MultiTopicVisualizer::drone2RobotCallback, this);
        sub_drone3_robot_ = nh_.subscribe("/drone_3_odom_visualization/robot", 10, &MultiTopicVisualizer::drone3RobotCallback, this);

        // MarkerArray
        sub_yaw_fov_ = nh_.subscribe("/yaw_planner_node/yaw_fov_markers", 10, &MultiTopicVisualizer::yawFovCallback, this);

        // 发布器 - 发布所有历史累积数据
        pub_history_clouds_ = nh_.advertise<sensor_msgs::PointCloud2>("/history/occupancy_clouds", 10);
        pub_history_markers_ = nh_.advertise<visualization_msgs::MarkerArray>("/history/all_markers", 10);

        // 定时器
        timer_trajectory_ = nh_.createTimer(ros::Duration(0.1), &MultiTopicVisualizer::publishTrajectories, this);
        timer_snapshot_ = nh_.createTimer(ros::Duration(9.0), &MultiTopicVisualizer::takeSnapshotAndPublish, this);

        initColors();

        ROS_INFO("========================================");
        ROS_INFO("Multi-Topic Visualizer Node Started");
        ROS_INFO("Global frame_id: %s", global_frame_id_.c_str());
        ROS_INFO("Publishing history trajectories to: /history_trajectories");
        ROS_INFO("Taking snapshot every 9 seconds and publishing cumulative data");
        ROS_INFO("Publishing to: /history/occupancy_clouds and /history/all_markers");
        ROS_INFO("========================================");
    }

   private:
    ros::NodeHandle nh_;
    int snapshot_count_;
    std::string global_frame_id_;

    // ============ 轨迹相关 ============
    std::map<std::string, std::vector<geometry_msgs::Point>> trajectories_;
    std::map<std::string, std_msgs::ColorRGBA> colors_;
    std::map<std::string, std::string> frame_ids_;
    std::map<std::string, int> message_count_;  // 统计每个话题接收到的消息数

    ros::Subscriber sub_car_state_, sub_drone0_odom_, sub_drone1_odom_, sub_drone2_odom_, sub_drone3_odom_;
    ros::Publisher pub_trajectories_;
    ros::Timer timer_trajectory_;

    // ============ 快照记录相关 ============
    sensor_msgs::PointCloud2 latest_drone0_cloud_, latest_drone1_cloud_, latest_drone2_cloud_, latest_drone3_cloud_;
    visualization_msgs::Marker latest_car_model_, latest_drone0_robot_, latest_drone1_robot_, latest_drone2_robot_, latest_drone3_robot_;
    visualization_msgs::MarkerArray latest_yaw_fov_;

    struct Snapshot {
        ros::Time timestamp;
        sensor_msgs::PointCloud2 drone0_cloud;
        sensor_msgs::PointCloud2 drone1_cloud;
        sensor_msgs::PointCloud2 drone2_cloud;
        sensor_msgs::PointCloud2 drone3_cloud;
        visualization_msgs::Marker car_model;
        visualization_msgs::Marker drone0_robot;
        visualization_msgs::Marker drone1_robot;
        visualization_msgs::Marker drone2_robot;
        visualization_msgs::Marker drone3_robot;
        visualization_msgs::MarkerArray yaw_fov;
        bool has_drone0_cloud = false;
        bool has_drone1_cloud = false;
        bool has_drone2_cloud = false;
        bool has_drone3_cloud = false;
        bool has_car_model = false;
        bool has_drone0_robot = false;
        bool has_drone1_robot = false;
        bool has_drone2_robot = false;
        bool has_drone3_robot = false;
        bool has_yaw_fov = false;
    };

    std::vector<Snapshot> snapshots_;

    bool received_drone0_cloud_ = false, received_drone1_cloud_ = false, received_drone2_cloud_ = false, received_drone3_cloud_ = false;
    bool received_car_model_ = false, received_drone0_robot_ = false, received_drone1_robot_ = false, received_drone2_robot_ = false, received_drone3_robot_ = false;
    bool received_yaw_fov_ = false;

    ros::Subscriber sub_drone0_cloud_, sub_drone1_cloud_, sub_drone2_cloud_, sub_drone3_cloud_;
    ros::Subscriber sub_car_model_, sub_drone0_robot_, sub_drone1_robot_, sub_drone2_robot_, sub_drone3_robot_;
    ros::Subscriber sub_yaw_fov_;

    ros::Publisher pub_history_clouds_, pub_history_markers_;
    ros::Timer timer_snapshot_;

    void initColors() {
        colors_["car"].r = 0.0;
        colors_["car"].g = 0.0;
        colors_["car"].b = 0.0;
        colors_["car"].a = 1.0;

        colors_["drone_0"].r = 1.0;
        colors_["drone_0"].g = 0.0;
        colors_["drone_0"].b = 0.0;
        colors_["drone_0"].a = 0.2;

        colors_["drone_1"].r = 0.0;
        colors_["drone_1"].g = 1.0;
        colors_["drone_1"].b = 0.0;
        colors_["drone_1"].a = 0.2;

        colors_["drone_2"].r = 0.0;
        colors_["drone_2"].g = 0.0;
        colors_["drone_2"].b = 1.0;
        colors_["drone_2"].a = 0.2;

        colors_["drone_3"].r = 1.0;
        colors_["drone_3"].g = 0.0;
        colors_["drone_3"].b = 1.0;
        colors_["drone_3"].a = 0.2;

        // 初始化消息计数
        message_count_["car"] = 0;
        message_count_["drone_0"] = 0;
        message_count_["drone_1"] = 0;
        message_count_["drone_2"] = 0;
        message_count_["drone_3"] = 0;
    }

    // ============ Odometry回调函数 ============
    void carStateCallback(const nav_msgs::Odometry::ConstPtr& msg) { processOdometry(msg, "car"); }

    void drone0OdomCallback(const nav_msgs::Odometry::ConstPtr& msg) { processOdometry(msg, "drone_0"); }

    void drone1OdomCallback(const nav_msgs::Odometry::ConstPtr& msg) { processOdometry(msg, "drone_1"); }

    void drone2OdomCallback(const nav_msgs::Odometry::ConstPtr& msg) { processOdometry(msg, "drone_2"); }

    void drone3OdomCallback(const nav_msgs::Odometry::ConstPtr& msg) { processOdometry(msg, "drone_3"); }

    void processOdometry(const nav_msgs::Odometry::ConstPtr& msg, const std::string& name) {
        geometry_msgs::Point pt = msg->pose.pose.position;
        trajectories_[name].push_back(pt);

        // 使用全局frame_id而不是消息中的frame_id
        frame_ids_[name] = global_frame_id_;

        // 增加消息计数
        message_count_[name]++;

        // 只在第一次接收到消息时打印
        if (message_count_[name] == 1) {
            ROS_INFO("✓ First message received from %s (frame: %s -> %s)", name.c_str(), msg->header.frame_id.c_str(), global_frame_id_.c_str());
            ROS_INFO("  Position: [%.2f, %.2f, %.2f]", pt.x, pt.y, pt.z);
        }

        // 每100个消息打印一次统计
        if (message_count_[name] % 100 == 0) {
            ROS_INFO("%s: %d messages, %lu trajectory points", name.c_str(), message_count_[name], trajectories_[name].size());
        }
    }

    // ============ PointCloud2回调函数 ============
    void drone0CloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
        latest_drone0_cloud_ = *msg;
        if (!received_drone0_cloud_) {
            ROS_INFO("✓ First cloud received from drone_0");
            received_drone0_cloud_ = true;
        }
    }

    void drone1CloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
        latest_drone1_cloud_ = *msg;
        if (!received_drone1_cloud_) {
            ROS_INFO("✓ First cloud received from drone_1");
            received_drone1_cloud_ = true;
        }
    }

    void drone2CloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
        latest_drone2_cloud_ = *msg;
        if (!received_drone2_cloud_) {
            ROS_INFO("✓ First cloud received from drone_2");
            received_drone2_cloud_ = true;
        }
    }

    void drone3CloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
        latest_drone3_cloud_ = *msg;
        if (!received_drone3_cloud_) {
            ROS_INFO("✓ First cloud received from drone_3");
            received_drone3_cloud_ = true;
        }
    }

    // ============ Marker回调函数 ============
    void carModelCallback(const visualization_msgs::Marker::ConstPtr& msg) {
        latest_car_model_ = *msg;
        if (!received_car_model_) {
            ROS_INFO("✓ First marker received from car_model");
            received_car_model_ = true;
        }
    }

    void drone0RobotCallback(const visualization_msgs::Marker::ConstPtr& msg) {
        latest_drone0_robot_ = *msg;
        if (!received_drone0_robot_) {
            ROS_INFO("✓ First marker received from drone_0_robot");
            received_drone0_robot_ = true;
        }
    }

    void drone1RobotCallback(const visualization_msgs::Marker::ConstPtr& msg) {
        latest_drone1_robot_ = *msg;
        if (!received_drone1_robot_) {
            ROS_INFO("✓ First marker received from drone_1_robot");
            received_drone1_robot_ = true;
        }
    }

    void drone2RobotCallback(const visualization_msgs::Marker::ConstPtr& msg) {
        latest_drone2_robot_ = *msg;
        if (!received_drone2_robot_) {
            ROS_INFO("✓ First marker received from drone_2_robot");
            received_drone2_robot_ = true;
        }
    }

    void drone3RobotCallback(const visualization_msgs::Marker::ConstPtr& msg) {
        latest_drone3_robot_ = *msg;
        if (!received_drone3_robot_) {
            ROS_INFO("✓ First marker received from drone_3_robot");
            received_drone3_robot_ = true;
        }
    }

    // ============ MarkerArray回调函数 ============
    void yawFovCallback(const visualization_msgs::MarkerArray::ConstPtr& msg) {
        latest_yaw_fov_ = *msg;
        if (!received_yaw_fov_) {
            ROS_INFO("✓ First marker array received from yaw_fov");
            received_yaw_fov_ = true;
        }
    }

    // ============ 发布轨迹 ============
    void publishTrajectories(const ros::TimerEvent& event) {
        visualization_msgs::MarkerArray marker_array;
        int marker_id = 0;

        for (const auto& traj_pair : trajectories_) {
            const std::string& name = traj_pair.first;
            const std::vector<geometry_msgs::Point>& points = traj_pair.second;

            if (points.empty())
                continue;

            // 创建LINE_STRIP
            visualization_msgs::Marker line_marker;
            line_marker.header.frame_id = global_frame_id_;  // 使用统一的frame_id
            line_marker.header.stamp = ros::Time::now();
            line_marker.ns = name + "_trajectory";
            line_marker.id = marker_id++;
            line_marker.type = visualization_msgs::Marker::LINE_STRIP;
            line_marker.action = visualization_msgs::Marker::ADD;
            line_marker.pose.orientation.w = 1.0;
            line_marker.scale.x = 0.05;
            line_marker.color = colors_[name];
            line_marker.lifetime = ros::Duration(0);

            for (const auto& pt : points) {
                line_marker.points.push_back(pt);
            }

            marker_array.markers.push_back(line_marker);

            // 创建POINTS
            visualization_msgs::Marker points_marker;
            points_marker.header.frame_id = global_frame_id_;  // 使用统一的frame_id
            points_marker.header.stamp = ros::Time::now();
            points_marker.ns = name + "_points";
            points_marker.id = marker_id++;
            points_marker.type = visualization_msgs::Marker::POINTS;
            points_marker.action = visualization_msgs::Marker::ADD;
            points_marker.pose.orientation.w = 1.0;
            points_marker.scale.x = 0.08;
            points_marker.scale.y = 0.08;
            points_marker.color = colors_[name];
            points_marker.lifetime = ros::Duration(0);

            for (const auto& pt : points) {
                points_marker.points.push_back(pt);
            }

            marker_array.markers.push_back(points_marker);
        }

        if (!marker_array.markers.empty()) {
            pub_trajectories_.publish(marker_array);
        }
    }

    // ============ 每9秒拍摄快照并发布所有历史数据 ============
    void takeSnapshotAndPublish(const ros::TimerEvent& event) {
        Snapshot snapshot;
        snapshot.timestamp = ros::Time::now();

        if (received_drone0_cloud_) {
            snapshot.drone0_cloud = latest_drone0_cloud_;
            snapshot.has_drone0_cloud = true;
        }
        if (received_drone1_cloud_) {
            snapshot.drone1_cloud = latest_drone1_cloud_;
            snapshot.has_drone1_cloud = true;
        }
        if (received_drone2_cloud_) {
            snapshot.drone2_cloud = latest_drone2_cloud_;
            snapshot.has_drone2_cloud = true;
        }
        if (received_drone3_cloud_) {
            snapshot.drone3_cloud = latest_drone3_cloud_;
            snapshot.has_drone3_cloud = true;
        }

        if (received_car_model_) {
            snapshot.car_model = latest_car_model_;
            snapshot.has_car_model = true;
        }
        if (received_drone0_robot_) {
            snapshot.drone0_robot = latest_drone0_robot_;
            snapshot.has_drone0_robot = true;
        }
        if (received_drone1_robot_) {
            snapshot.drone1_robot = latest_drone1_robot_;
            snapshot.has_drone1_robot = true;
        }
        if (received_drone2_robot_) {
            snapshot.drone2_robot = latest_drone2_robot_;
            snapshot.has_drone2_robot = true;
        }
        if (received_drone3_robot_) {
            snapshot.drone3_robot = latest_drone3_robot_;
            snapshot.has_drone3_robot = true;
        }

        if (received_yaw_fov_) {
            snapshot.yaw_fov = latest_yaw_fov_;
            snapshot.has_yaw_fov = true;
        }

        snapshots_.push_back(snapshot);
        snapshot_count_++;

        ROS_INFO("========================================");
        ROS_INFO("Snapshot #%d taken at time: %.2f", snapshot_count_, snapshot.timestamp.toSec());
        ROS_INFO("Total snapshots: %lu", snapshots_.size());
        ROS_INFO("========================================");

        publishAllHistoryData();
    }

    void publishAllHistoryData() {
        ros::Time publish_time = ros::Time::now();

        // ============ 发布所有历史点云(合并) ============
        pcl::PointCloud<pcl::PointXYZ> merged_cloud;

        for (size_t i = 0; i < snapshots_.size(); ++i) {
            const Snapshot& snap = snapshots_[i];

            if (snap.has_drone0_cloud) {
                pcl::PointCloud<pcl::PointXYZ> temp_cloud;
                pcl::fromROSMsg(snap.drone0_cloud, temp_cloud);
                merged_cloud += temp_cloud;
            }

            if (snap.has_drone1_cloud) {
                pcl::PointCloud<pcl::PointXYZ> temp_cloud;
                pcl::fromROSMsg(snap.drone1_cloud, temp_cloud);
                merged_cloud += temp_cloud;
            }

            if (snap.has_drone2_cloud) {
                pcl::PointCloud<pcl::PointXYZ> temp_cloud;
                pcl::fromROSMsg(snap.drone2_cloud, temp_cloud);
                merged_cloud += temp_cloud;
            }

            if (snap.has_drone3_cloud) {
                pcl::PointCloud<pcl::PointXYZ> temp_cloud;
                pcl::fromROSMsg(snap.drone3_cloud, temp_cloud);
                merged_cloud += temp_cloud;
            }
        }

        if (!merged_cloud.empty()) {
            sensor_msgs::PointCloud2 output_cloud;
            pcl::toROSMsg(merged_cloud, output_cloud);
            output_cloud.header.stamp = publish_time;
            output_cloud.header.frame_id = global_frame_id_;
            pub_history_clouds_.publish(output_cloud);

            ROS_INFO("Published merged cloud with %lu points", merged_cloud.size());
        }

        // ============ 发布所有历史Marker ============
        visualization_msgs::MarkerArray all_markers;
        int marker_id = 0;
        int count = 0;

        for (size_t i = 0; i < snapshots_.size(); ++i) {
            count++;
            if (count == 5) {
                continue;
            }

            const Snapshot& snap = snapshots_[i];
            std::string snapshot_ns = "snapshot_" + std::to_string(i);

            if (snap.has_car_model) {
                visualization_msgs::Marker marker = snap.car_model;
                marker.header.stamp = publish_time;
                marker.header.frame_id = global_frame_id_;
                marker.ns = snapshot_ns + "_car";
                marker.id = marker_id++;
                marker.lifetime = ros::Duration(0);
                all_markers.markers.push_back(marker);
            }

            if (snap.has_drone0_robot) {
                visualization_msgs::Marker marker = snap.drone0_robot;
                marker.header.stamp = publish_time;
                marker.header.frame_id = global_frame_id_;
                marker.ns = snapshot_ns + "_drone0";
                marker.id = marker_id++;
                marker.lifetime = ros::Duration(0);
                all_markers.markers.push_back(marker);
            }

            if (snap.has_drone1_robot) {
                visualization_msgs::Marker marker = snap.drone1_robot;
                marker.header.stamp = publish_time;
                marker.header.frame_id = global_frame_id_;
                marker.ns = snapshot_ns + "_drone1";
                marker.id = marker_id++;
                marker.lifetime = ros::Duration(0);
                all_markers.markers.push_back(marker);
            }

            if (snap.has_drone2_robot) {
                visualization_msgs::Marker marker = snap.drone2_robot;
                marker.header.stamp = publish_time;
                marker.header.frame_id = global_frame_id_;
                marker.ns = snapshot_ns + "_drone2";
                marker.id = marker_id++;
                marker.lifetime = ros::Duration(0);
                all_markers.markers.push_back(marker);
            }

            if (snap.has_drone3_robot) {
                visualization_msgs::Marker marker = snap.drone3_robot;
                marker.header.stamp = publish_time;
                marker.header.frame_id = global_frame_id_;
                marker.ns = snapshot_ns + "_drone3";
                marker.id = marker_id++;
                marker.lifetime = ros::Duration(0);
                all_markers.markers.push_back(marker);
            }

            if (snap.has_yaw_fov) {
                for (size_t j = 0; j < snap.yaw_fov.markers.size(); ++j) {
                    visualization_msgs::Marker marker = snap.yaw_fov.markers[j];
                    marker.header.stamp = publish_time;
                    marker.header.frame_id = global_frame_id_;
                    marker.ns = snapshot_ns + "_fov_" + std::to_string(j);
                    marker.id = marker_id++;
                    marker.lifetime = ros::Duration(0);
                    all_markers.markers.push_back(marker);
                }
            }
        }

        if (!all_markers.markers.empty()) {
            pub_history_markers_.publish(all_markers);
            ROS_INFO("Published %lu historical markers", all_markers.markers.size());
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "multi_topic_visualizer");

    MultiTopicVisualizer visualizer;

    ros::spin();

    return 0;
}