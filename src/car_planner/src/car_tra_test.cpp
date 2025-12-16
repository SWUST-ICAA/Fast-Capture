#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/Marker.h>
#include <cmath>
#include <random>
#include <string>

class Figure8Trajectory {
   public:
    Figure8Trajectory()
        : rng_(std::random_device{}()),
          noise_std_(0.05),             // 高斯噪声标准差，可根据需要调整
          noise_max_(0.15),             // 噪声绝对值最大不超过 0.15
          noise_dist_(0.0, noise_std_)  // 0 均值高斯分布
    {
        ros::NodeHandle nh;

        // 发布器：带噪声的 /car_state
        odom_pub_ = nh.advertise<nav_msgs::Odometry>("/car_state", 10);
        marker_pub_ = nh.advertise<visualization_msgs::Marker>("/trajectory_marker", 10);

        // 订阅器：订阅自己发布的 /car_state（此时已经带有 4.8 秒更新一次的噪声）
        car_state_sub_ = nh.subscribe("/car_state", 10, &Figure8Trajectory::carStateCallback, this);

        // 四个 drone target 话题发布器
        drone_target_pub_[0] = nh.advertise<nav_msgs::Odometry>("/drone_0_target", 10);
        drone_target_pub_[1] = nh.advertise<nav_msgs::Odometry>("/drone_1_target", 10);
        drone_target_pub_[2] = nh.advertise<nav_msgs::Odometry>("/drone_2_target", 10);
        drone_target_pub_[3] = nh.advertise<nav_msgs::Odometry>("/drone_3_target", 10);

        // 空间边界
        min_point_[0] = 0.0;
        min_point_[1] = 0.0;
        min_point_[2] = 0.0;

        max_point_[0] = 4.0;
        max_point_[1] = 4.0;
        max_point_[2] = 4.0;

        // 计算中心点
        center_x_ = (min_point_[0] + max_point_[0]) / 2.0;  // 2.0
        center_y_ = (min_point_[1] + max_point_[1]) / 2.0;  // 2.0
        center_z_ = (min_point_[2] + max_point_[2]) / 2.0;  // 2.0

        // "8" 字的半径
        radius_x_ = (max_point_[0] - min_point_[0]) / 4.0;  // 1.0
        radius_y_ = (max_point_[1] - min_point_[1]) / 4.0;  // 1.0
        radius_z_ = (max_point_[2] - min_point_[2]) / 4.0;  // 1.0

        // 运动周期 (秒)
        period_ = 10.0;

        // 发布频率 (Hz)
        rate_ = 50.0;

        // 记录起始时间
        start_time_ = ros::Time::now();

        // 初始化 4 个 drone target 的 0.03 秒噪声
        for (int i = 0; i < 4; ++i) {
            drone_last_noise_update_[i] = start_time_;
            drone_noise_x_[i] = sampleNoise();
            drone_noise_y_[i] = sampleNoise();
            drone_noise_z_[i] = sampleNoise();
        }

        // 初始化 /car_state 用的慢变化噪声（4.8 秒更新一次）
        last_noise_update_ = start_time_;
        noise_x_ = sampleNoise();
        noise_y_ = sampleNoise();
        noise_z_ = sampleNoise();

        ROS_INFO("Figure-8 trajectory publisher initialized");
        ROS_INFO("Trajectory bounds: (%.1f, %.1f, %.1f) to (%.1f, %.1f, %.1f)", min_point_[0], min_point_[1], min_point_[2], max_point_[0], max_point_[1], max_point_[2]);
    }

    // 计算 8 字轨迹位置（无噪声）
    void calculateFigure8Position(double t, double& x, double& y, double& z) {
        double omega = 2.0 * M_PI / period_;
        x = center_x_ + radius_x_ * std::sin(omega * t);
        y = center_y_ + radius_y_ * std::sin(2.0 * omega * t);  // 频率比 2:1
        z = center_z_ + radius_z_ * std::sin(omega * t);
    }

    // 计算速度（无噪声）
    void calculateVelocity(double t, double& vx, double& vy, double& vz) {
        double omega = 2.0 * M_PI / period_;
        vx = radius_x_ * omega * std::cos(omega * t);
        vy = radius_y_ * 2.0 * omega * std::cos(2.0 * omega * t);
        vz = radius_z_ * omega * std::cos(omega * t);
    }

    // 采样一个带截断的高斯噪声（0均值，绝对值 <= noise_max_）
    double sampleNoise() {
        double n = noise_dist_(rng_);
        if (n > noise_max_) {
            n = noise_max_;
        } else if (n < -noise_max_) {
            n = -noise_max_;
        }
        return n;
    }

    // 回调：在已经带慢噪声的 /car_state 上，再叠加一层“每 0.03 秒才改变一次”的噪声，发布到 4 个话题
    void carStateCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        // /car_state 本身已经包含了 4.8 秒更新一次的慢变化噪声
        const double drone_noise_interval = 0.03;  // 4 个 drone 话题的噪声每 0.03 秒更新一次
        ros::Time now = ros::Time::now();

        for (int i = 0; i < 4; ++i) {
            nav_msgs::Odometry noisy_odom = *msg;

            // 区分不同 drone 的 child_frame_id
            noisy_odom.child_frame_id = "drone_" + std::to_string(i) + "_base";

            double tx = noisy_odom.pose.pose.position.x;
            double ty = noisy_odom.pose.pose.position.y;
            double tz = noisy_odom.pose.pose.position.z;

            // ----- 0.03 秒更新一次的噪声 -----
            double dt = (now - drone_last_noise_update_[i]).toSec();
            if (dt >= drone_noise_interval) {
                drone_noise_x_[i] = sampleNoise();
                drone_noise_y_[i] = sampleNoise();
                drone_noise_z_[i] = sampleNoise();
                drone_last_noise_update_[i] = now;
            }

            // 在已有噪声的基础上，再加一层“保持 0.03 秒不变”的噪声
            noisy_odom.pose.pose.position.x += drone_noise_x_[i];
            noisy_odom.pose.pose.position.y += drone_noise_y_[i];
            noisy_odom.pose.pose.position.z += drone_noise_z_[i];

            // 如果你想速度也再次加噪，可以在这里对 twist.twist.linear.* 再加噪声（同样按 0.03 秒更新）

            double fabsx = (noisy_odom.pose.pose.position.x - tx) / fabs(noisy_odom.pose.pose.position.x);
            double fabsy = (noisy_odom.pose.pose.position.y - ty) / fabs(noisy_odom.pose.pose.position.y);
            double fabsz = (noisy_odom.pose.pose.position.z - tz) / fabs(noisy_odom.pose.pose.position.z);

            noisy_odom.pose.pose.position.x += 0.6 * fabsx;
            noisy_odom.pose.pose.position.y += 0.6 * fabsy;
            noisy_odom.pose.pose.position.z += 0.6 * fabsz;

            drone_target_pub_[i].publish(noisy_odom);
        }
    }

    void run() {
        ros::Rate loop_rate(rate_);

        // 轨迹线 Marker（使用 /car_state 的位置进行可视化，这里用带慢噪声的位置）
        visualization_msgs::Marker trajectory_line;
        trajectory_line.header.frame_id = "world";
        trajectory_line.ns = "trajectory";
        trajectory_line.id = 0;
        trajectory_line.type = visualization_msgs::Marker::LINE_STRIP;
        trajectory_line.action = visualization_msgs::Marker::ADD;
        trajectory_line.pose.orientation.w = 1.0;
        trajectory_line.scale.x = 0.05;  // 线宽
        trajectory_line.color.r = 0.0;
        trajectory_line.color.g = 1.0;
        trajectory_line.color.b = 0.0;
        trajectory_line.color.a = 1.0;

        const double noise_update_interval = 4.8;  // /car_state 噪声每 4.8 秒更新一次

        while (ros::ok()) {
            ros::Time current_time = ros::Time::now();
            double t = (current_time - start_time_).toSec();

            // ------------- 更新 /car_state 的慢变化噪声 -------------
            double dt_noise = (current_time - last_noise_update_).toSec();
            if (dt_noise >= noise_update_interval) {
                noise_x_ = sampleNoise();
                noise_y_ = sampleNoise();
                noise_z_ = sampleNoise();
                last_noise_update_ = current_time;
            }

            // ------------- 计算理想位置 & 速度（无噪声） -------------
            double x, y, z;
            double vx, vy, vz;
            calculateFigure8Position(t, x, y, z);
            calculateVelocity(t, vx, vy, vz);

            // 在理想位置基础上叠加“慢变化噪声” -> 这是 /car_state 的位置
            double x_noisy = x + noise_x_;
            double y_noisy = y + noise_y_;
            double z_noisy = z + noise_z_;

            // 朝向仍然根据无噪声速度计算
            double yaw = std::atan2(vy, vx);
            double pitch = std::atan2(vz, std::sqrt(vx * vx + vy * vy));
            double roll = 0.0;

            tf::Quaternion quaternion;
            quaternion.setRPY(roll, pitch, yaw);

            // ------------- 组装并发布 /car_state（带慢噪声） -------------
            nav_msgs::Odometry odom;
            odom.header.stamp = current_time;
            odom.header.frame_id = "world";
            odom.child_frame_id = "base_link";

            odom.pose.pose.position.x = x_noisy;
            odom.pose.pose.position.y = y_noisy;
            odom.pose.pose.position.z = z_noisy;

            odom.pose.pose.orientation.x = quaternion.x();
            odom.pose.pose.orientation.y = quaternion.y();
            odom.pose.pose.orientation.z = quaternion.z();
            odom.pose.pose.orientation.w = quaternion.w();

            odom.twist.twist.linear.x = vx;
            odom.twist.twist.linear.y = vy;
            odom.twist.twist.linear.z = vz;

            odom.twist.twist.angular.x = 0.0;
            odom.twist.twist.angular.y = 0.0;
            odom.twist.twist.angular.z = 0.0;

            odom_pub_.publish(odom);

            // ------------- 可视化：轨迹线（用带慢噪声的位置） -------------
            geometry_msgs::Point p;
            p.x = x_noisy;
            p.y = y_noisy;
            p.z = z_noisy;
            trajectory_line.points.push_back(p);

            if (trajectory_line.points.size() > 1000) {
                trajectory_line.points.erase(trajectory_line.points.begin());
            }

            trajectory_line.header.stamp = current_time;
            marker_pub_.publish(trajectory_line);

            // 当前位置小球 Marker
            visualization_msgs::Marker current_pos;
            current_pos.header.stamp = current_time;
            current_pos.header.frame_id = "world";
            current_pos.ns = "current_position";
            current_pos.id = 1;
            current_pos.type = visualization_msgs::Marker::SPHERE;
            current_pos.action = visualization_msgs::Marker::ADD;
            current_pos.pose.position.x = x_noisy;
            current_pos.pose.position.y = y_noisy;
            current_pos.pose.position.z = z_noisy;
            current_pos.pose.orientation.w = 1.0;
            current_pos.scale.x = 0.2;
            current_pos.scale.y = 0.2;
            current_pos.scale.z = 0.2;
            current_pos.color.r = 1.0;
            current_pos.color.g = 0.0;
            current_pos.color.b = 0.0;
            current_pos.color.a = 1.0;
            marker_pub_.publish(current_pos);

            // 速度箭头 Marker（起点用带慢噪声的位置，方向用真实速度）
            visualization_msgs::Marker velocity_arrow;
            velocity_arrow.header.stamp = current_time;
            velocity_arrow.header.frame_id = "world";
            velocity_arrow.ns = "velocity";
            velocity_arrow.id = 2;
            velocity_arrow.type = visualization_msgs::Marker::ARROW;
            velocity_arrow.action = visualization_msgs::Marker::ADD;

            geometry_msgs::Point start;
            start.x = x_noisy;
            start.y = y_noisy;
            start.z = z_noisy;

            geometry_msgs::Point end;
            end.x = x_noisy + vx * 0.3;
            end.y = y_noisy + vy * 0.3;
            end.z = z_noisy + vz * 0.3;

            velocity_arrow.points.push_back(start);
            velocity_arrow.points.push_back(end);
            velocity_arrow.scale.x = 0.05;
            velocity_arrow.scale.y = 0.1;
            velocity_arrow.color.r = 0.0;
            velocity_arrow.color.g = 0.0;
            velocity_arrow.color.b = 1.0;
            velocity_arrow.color.a = 1.0;
            marker_pub_.publish(velocity_arrow);

            // 处理订阅回调（必须有这个，/car_state 才能被订阅到）
            ros::spinOnce();
            loop_rate.sleep();
        }
    }

   private:
    // 发布器
    ros::Publisher odom_pub_;
    ros::Publisher marker_pub_;
    ros::Publisher drone_target_pub_[4];

    // 订阅器
    ros::Subscriber car_state_sub_;

    double min_point_[3];
    double max_point_[3];

    double center_x_;
    double center_y_;
    double center_z_;

    double radius_x_;
    double radius_y_;
    double radius_z_;

    double period_;
    double rate_;

    ros::Time start_time_;

    ros::Time start_time2_;  // 未使用，可以根据需要删除

    // /car_state 用的慢变化噪声
    ros::Time last_noise_update_;
    double noise_x_, noise_y_, noise_z_;

    // 4 个 drone target 的 0.03 秒更新一次的噪声
    ros::Time drone_last_noise_update_[4];
    double drone_noise_x_[4], drone_noise_y_[4], drone_noise_z_[4];

    // 随机数与高斯分布
    std::mt19937 rng_;
    double noise_std_;
    double noise_max_;
    std::normal_distribution<double> noise_dist_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "figure8_trajectory_publisher");

    try {
        Figure8Trajectory trajectory;
        trajectory.run();
    } catch (const std::exception& e) {
        ROS_ERROR("Exception: %s", e.what());
        return 1;
    }

    return 0;
}
