/*
功能：
这是一个 ROS 节点程序，用于实现无人机的路径规划与状态更新。
其核心功能包括：
  接收目标点（waypoints），调用路径规划算法生成轨迹；
  实时更新无人机状态（位置、方向）并发布到 /car_state 话题（这里仍沿用 Odometry 消息格式）；
  在 RViz 中可视化无人机模型（3D 网格模型，颜色为红色）。

输入：
  目标点话题 /waypoints：接收 geometry_msgs/PoseStamped 消息，指定无人机的目标位置和方向。
  参数服务器配置：无人机初始位置、尺寸（这里仅作为可视化缩放参数）、模型资源路径等。

输出：
  无人机状态话题 /car_state：发布 nav_msgs/Odometry 消息，包含无人机当前位置和方向。
  可视化话题 /visualization/car_model：发布 visualization_msgs/Marker 消息，显示无人机 3D 模型（红色）。
*/

#include <car_planner/car_search.h>  // 保留路径规划模块（如果该模块用于规划轨迹，不必改动）
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <Eigen/Dense>
#include <iostream>

using namespace car_planner;
// using namespace Eigen;

// 全局变量
std::string mesh_resource;
ros::Subscriber waypoints_sub;
ros::Publisher viscar_pub, state_pub;
ros::Timer state_timer, state_update_timer;
ros::Publisher traj_vis_pub;

// UAV 当前状态：x, y, yaw
Eigen::Vector3d current_state, target_pt;
// 规划算法对象（假设与车辆搜索代码类似）
Car_KinoSearch* kinosearch;
bool update_flag = false;
double update_time;

// 这里无人机模型可视化时只用一个缩放参数
double car_scale;

visualization_msgs::Marker traj_marker;
const size_t MAX_TRAJ_POINTS = 500;

// 回调函数声明
void rcvWaypointsCallback(const geometry_msgs::PoseStamped& msg);
void vis_car_model(Eigen::Vector3d cur_state);
void pub_carstate(const ros::TimerEvent& event);  // 这里依然发布 Odometry 消息作为状态
void state_update(const ros::TimerEvent& event);
void update_trajectory();

int main(int argc, char** argv) {
    ros::init(argc, argv, "car_search_node");
    ros::NodeHandle nh_priv("~");

    // 读取无人机初始状态参数：x, y, yaw（z 固定为1）
    nh_priv.param("car/init_x", current_state[0], -16.0);
    nh_priv.param("car/init_y", current_state[1], 4.0);
    nh_priv.param("car/init_yaw", current_state[2], 0.0);

    // 使用无人机模型（例如 hummingbird.mesh），并固定颜色为红色
    nh_priv.param("car/mesh_resource", mesh_resource, std::string("package://odom_visualization/meshes/hummingbird.mesh"));
    // 缩放参数
    nh_priv.param("car/scale", car_scale, 1.2);  // 根据实际情况调整

    ros::MultiThreadedSpinner spinner(8);
    // 创建订阅与定时器（与车辆代码类似）
    waypoints_sub = nh_priv.subscribe("waypoints", 1, rcvWaypointsCallback);
    state_timer = nh_priv.createTimer(ros::Duration(0.05), pub_carstate);
    state_update_timer = nh_priv.createTimer(ros::Duration(0.01), state_update);
    // 修改可视化话题名称（依然使用 /visualization/car_model 但内容为无人机模型）
    viscar_pub = nh_priv.advertise<visualization_msgs::Marker>("/visualization/car_model", 100, true);
    state_pub = nh_priv.advertise<nav_msgs::Odometry>("car_state", 1, true);

    traj_vis_pub = nh_priv.advertise<visualization_msgs::Marker>("/car_trajectory_vis", 1, true);

    // 初始化路径规划对象（保留车辆规划算法，如果适用于无人机则直接复用）
    kinosearch = new Car_KinoSearch(nh_priv);

    traj_marker.header.frame_id = "world";
    traj_marker.ns = "car_trajectory";
    traj_marker.id = 0;
    traj_marker.type = visualization_msgs::Marker::LINE_STRIP;
    traj_marker.action = visualization_msgs::Marker::ADD;
    // 轨迹线宽
    traj_marker.scale.x = 0.1;
    // 轨迹颜色（绿色示例）
    traj_marker.color.r = 0.0;
    traj_marker.color.g = 0.0;
    traj_marker.color.b = 0.0;
    traj_marker.color.a = 1.0;
    // 轨迹本身不需要姿态变换，使用单位姿态
    traj_marker.pose.orientation.w = 1.0;
    traj_marker.pose.orientation.x = 0.0;
    traj_marker.pose.orientation.y = 0.0;
    traj_marker.pose.orientation.z = 0.0;
    traj_marker.points.clear();

    update_trajectory();

    spinner.spin();
    return 0;
}

// 接收到目标点后更新目标状态，并调用规划算法
void rcvWaypointsCallback(const geometry_msgs::PoseStamped& msg) {
    Eigen::Quaterniond tq;
    tq.x() = msg.pose.orientation.x;
    tq.y() = msg.pose.orientation.y;
    tq.z() = msg.pose.orientation.z;
    tq.w() = msg.pose.orientation.w;
    Eigen::Matrix3d trot(tq);
    // 计算目标 yaw，采用与车辆代码相同方法
    double tyaw = atan2(trot.col(0)[1], trot.col(0)[0]);
    target_pt << msg.pose.position.x, msg.pose.position.y, tyaw;
    ROS_INFO_STREAM("TARGET=" << target_pt);
    ROS_INFO("[node] receive the planning target");
    kinosearch->reset();
    kinosearch->car_search(current_state, target_pt);  // 如果规划算法适用，可直接使用
    kinosearch->visualize(0.1);
    update_flag = true;
    update_time = ros::Time::now().toSec();
}

// 定时器回调：更新状态并可视化无人机模型
void state_update(const ros::TimerEvent& event) {
    if (update_flag) {
        double now_time = ros::Time::now().toSec();
        double delta_time = now_time - update_time;
        if (delta_time <= kinosearch->get_totalT()) {
            current_state = kinosearch->evaluate_state(delta_time);
        } else {
            update_flag = false;
        }
    }
    vis_car_model(current_state);
    update_trajectory();
}

void update_trajectory() {
    // 当前状态转换为轨迹点
    geometry_msgs::Point p;
    p.x = current_state[0];
    p.y = current_state[1];
    p.z = 0.29;  // 固定高度

    // 将新点加入轨迹
    traj_marker.points.push_back(p);

    // 控制最多 4000 个点
    if (traj_marker.points.size() > MAX_TRAJ_POINTS) {
        // 移除最早的点
        traj_marker.points.erase(traj_marker.points.begin());
    }

    // 更新时间戳并发布
    traj_marker.header.stamp = ros::Time::now();
    traj_vis_pub.publish(traj_marker);
}

// 可视化无人机模型函数，修改自车辆模型函数
void vis_car_model(Eigen::Vector3d cur_state) {
    Eigen::Vector3d pos;
    Eigen::Matrix3d rota_M;
    // 对于无人机模型，不再使用车辆尺寸偏移，仅直接使用当前位置，设置 z 为1
    double x = cur_state[0], y = cur_state[1], z = 1.0;
    double yaw = cur_state[2];

    // 构造旋转矩阵：无人机绕 z 轴旋转 yaw
    rota_M << cos(yaw), -sin(yaw), 0, sin(yaw), cos(yaw), 0, 0, 0, 1;

    // 修改：不再添加车辆尺寸偏移，直接使用当前位置作为模型显示位置
    pos = Eigen::Vector3d(x, y, z);

    // 构造 Marker 消息，采用 Mesh 类型
    visualization_msgs::Marker carMarker;
    carMarker.id = 0;
    carMarker.header.stamp = ros::Time::now();
    carMarker.header.frame_id = "world";
    carMarker.action = visualization_msgs::Marker::ADD;
    carMarker.type = visualization_msgs::Marker::MESH_RESOURCE;
    carMarker.ns = "car_mesh";
    carMarker.mesh_use_embedded_materials = true;
    // 设置颜色为红色
    carMarker.color.r = 1.0;
    carMarker.color.g = 0.0;
    carMarker.color.b = 0.0;
    carMarker.color.a = 1.0;
    // 设置缩放参数（统一缩放）
    carMarker.scale.x = car_scale;
    carMarker.scale.y = car_scale;
    carMarker.scale.z = car_scale;
    // 无人机模型的姿态：采用绕 z 轴旋转 yaw 得到的四元数
    Eigen::Matrix3d rot_c, rot_f;
    rot_c << 0, -1, 0, 1, 0, 0, 0, 0, 1;
    rot_f = rota_M * rot_c;
    Eigen::Quaterniond q(rot_f);
    carMarker.pose.orientation.w = q.w();
    carMarker.pose.orientation.x = q.x();
    carMarker.pose.orientation.y = q.y();
    carMarker.pose.orientation.z = q.z();
    // 设置模型位置
    carMarker.pose.position.x = pos(0);
    carMarker.pose.position.y = pos(1);
    carMarker.pose.position.z = pos(2);
    // 模型资源路径（无人机模型）
    carMarker.mesh_resource = mesh_resource;

    // 发布无人机模型 Marker
    viscar_pub.publish(carMarker);
}

// 定时器回调：发布无人机状态消息
void pub_carstate(const ros::TimerEvent& event) {
    // 构造 Odometry 消息，填入无人机当前状态
    nav_msgs::Odometry car_odom;
    car_odom.header.frame_id = "world";
    car_odom.header.stamp = ros::Time::now();
    car_odom.pose.pose.position.x = current_state[0];
    car_odom.pose.pose.position.y = current_state[1];
    car_odom.pose.pose.position.z = 1.0;  // 固定高度
    // 根据 yaw 构造四元数
    Eigen::Matrix3d rota_M;
    double yaw = current_state[2];
    rota_M << cos(yaw), -sin(yaw), 0, sin(yaw), cos(yaw), 0, 0, 0, 1;
    Eigen::Quaterniond q(rota_M);
    car_odom.pose.pose.orientation.x = q.x();
    car_odom.pose.pose.orientation.y = q.y();
    car_odom.pose.pose.orientation.z = q.z();
    car_odom.pose.pose.orientation.w = q.w();
    state_pub.publish(car_odom);
}
