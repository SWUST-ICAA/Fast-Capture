#include <target_detection/sim_detect.h>

// 构造函数
Sim_detect::Sim_detect(ros::NodeHandle& nh) {
    //     <param name="grid_map/resolution"      value="0.1" />
    // <param name="grid_map/map_size_x"   value="$(arg map_size_x_)" />
    // <param name="grid_map/map_size_y"   value="$(arg map_size_y_)" />
    // <param name="grid_map/map_size_z"   value="$(arg map_size_z_)" />
    nh.param("grid_map/resolution", resolution, 0.1);
    nh.param("grid_map/map_size_x", map_size_3d_[0], 100.0);
    nh.param("grid_map/map_size_y", map_size_3d_[1], 100.0);
    nh.param("grid_map/map_size_z", map_size_3d_[2], 3.0);
    nh.param("detection/noise_std_x", noise_std_x_, 0.07);  // 默认 0.1m
    nh.param("detection/noise_std_y", noise_std_y_, 0.07);
    nh.param("detection/noise_std_z", noise_std_z_, 0.07);  // z轴噪声通常更小

    // 计算地图原点（地图中心为原点，z 轴略微下移）
    origin_[0] = -map_size_3d_(0) / 2;  // x 原点 = -50m
    origin_[1] = -map_size_3d_(1) / 2;  // y 原点 = -50m
    origin_[2] = -0.1;                  // z 原点 = -0.1m（略低于地面）

    // 计算每个维度的网格数
    dim_(0) = map_size_3d_(0) / resolution;  // x 方向网格数 = 1000（分辨率 0.1m）
    dim_(1) = map_size_3d_(1) / resolution;  // y 方向网格数 = 1000
    dim_(2) = map_size_3d_(2) / resolution;  // z 方向网格数 = 30

    // 初始化地图缓冲区（总大小 = 1000*1000*30 = 30,000,000）
    buffer_size = dim_(0) * dim_(1) * dim_(2);
    map_buffer.resize(buffer_size);  // 初始值为 0（空闲）

    // 初始化随机数生成器
    std::random_device rd;
    rng_ = std::mt19937(rd());
    noise_dist_x_ = std::normal_distribution<double>(0.0, noise_std_x_);
    noise_dist_y_ = std::normal_distribution<double>(0.0, noise_std_y_);
    noise_dist_z_ = std::normal_distribution<double>(0.0, noise_std_z_);

    // 订阅与发布
    map_sub = nh.subscribe("global_map", 10, &Sim_detect::GlobalMapBuild, this);        // 订阅点云地图
    target_sub = nh.subscribe("car_state_detect", 1, &Sim_detect::car_state_cb, this);  // 订阅车辆状态
    drone_sub = nh.subscribe("drone_odom", 1, &Sim_detect::drone_odom_cb, this);        // 订阅无人机位姿
    detection_pub = nh.advertise<nav_msgs::Odometry>("target", 1, true);                // 发布目标检测结果
    detection_if = nh.advertise<std_msgs::Float64>("detection_if", 1, true);
}
// 地图构建 GlobalMapBuild
void Sim_detect::GlobalMapBuild(const sensor_msgs::PointCloud2& pointcloud_map) {
    if (has_map)  // 地图已构建则直接返回
        return;
    // 将 ROS 点云消息转换为 PCL 格式
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(pointcloud_map, cloud);
    // 空点云直接返回
    if ((int)cloud.points.size() == 0)
        return;
    pcl::PointXYZ pt;
    ROS_INFO("cloud points size=%d\n", (int)cloud.points.size());
    // 遍历点云中的每个点，设置障碍物
    for (int idx = 0; idx < (int)cloud.points.size(); idx++) {
        pt = cloud.points[idx];
        setObs(Eigen::Vector3d(pt.x, pt.y, pt.z));  // 调用 setObs 标记障碍物
    }
    has_map = true;
    // ROS_WARN("Sim Detection Node1111: Finish gridmap built");
}

// 设置障碍物 setObs
void Sim_detect::setObs(Eigen::Vector3d pt) {
    int expand_size = 0;                     // 障碍物扩展范围（当前为 0，不扩展）
    Eigen::Vector3i index = FloatToint(pt);  // 实际坐标转网格索引
    if (isOutside(index))                    // 越界则忽略
        return;
    // 在 expand_size 范围内标记障碍物
    for (int i = -expand_size; i <= expand_size; i++)
        for (int j = -expand_size; j <= expand_size; j++)
            for (int k = -expand_size; k <= expand_size; k++) {
                Eigen::Vector3i temp_index;
                temp_index(0) = index(0) + i;
                temp_index(1) = index(1) + j;
                temp_index(2) = index(2) + k;
                if (isOutside(temp_index))  // 越界跳过
                    continue;
                map_buffer[getIndex(temp_index)] = 1;
            }
}
bool Sim_detect::isOutside(Eigen::Vector3i idx) {
    for (int i = 0; i < 3; i++)
        if (idx(i) < 0 || idx(i) >= dim_(i))
            return true;
    return false;
}
int Sim_detect::getIndex(Eigen::Vector3i idx) {
    return idx(0) + dim_(0) * idx(1) + dim_(0) * dim_(1) * idx(2);
}
bool Sim_detect::isOccupied(Eigen::Vector3i idx) {
    if (isOutside(idx)) {
        // ROS_INFO_STREAM("IDX: "<<idx );
        return true;
    }
    return map_buffer[getIndex(idx)];
}
// 坐标转换 FloatToint 和 intToFloat
Eigen::Vector3d Sim_detect::intToFloat(const Eigen::Vector3i idx) {
    return (idx.template cast<double>() + Eigen::Vector3d::Constant(0.5)) * resolution + origin_;
}
Eigen::Vector3i Sim_detect::FloatToint(const Eigen::Vector3d pt) {
    Eigen::Vector3i pn;
    for (int i = 0; i < 3; i++)
        pn(i) = std::round((pt(i) - origin_(i)) / resolution - 0.5);
    return pn;
}
// 路径阻挡检测 is_block
bool Sim_detect::is_block(Eigen::Vector3d pos1, Eigen::Vector3d pos2) {
    // 步长 0.01：在两点之间采样 100 个点，可能在高分辨率地图中效率较低，但保证检测精度。
    double t;
    for (t = 0.0; t <= 1; t += 0.01) {  // 在两点间线性插值
        Eigen::Vector3d tmp_pos;
        tmp_pos = t * pos1 + (1 - t) * pos2;    // 插值点坐标
        if (isOccupied(FloatToint(tmp_pos))) {  // 检查是否被占用
            return true;
        }
    }
    return false;
}
// 回调函数 car_state_cb 和 drone_odom_cb
void Sim_detect::car_state_cb(const nav_msgs::Odometry& car_state) {
    if (!has_map)
        return;
    Eigen::Vector3d car_pos;
    car_pos << car_state.pose.pose.position.x, car_state.pose.pose.position.y, car_state.pose.pose.position.z;
    // if((car_pos-drone_pos).norm()>3.0) return;

    std_msgs::Float64 yaw_deg_msg;

    if (!is_block(car_pos, drone_pos)) {
        nav_msgs::Odometry state_pub = car_state;
        state_pub.pose.pose.position.x += noise_dist_x_(rng_);
        state_pub.pose.pose.position.y += noise_dist_y_(rng_);
        state_pub.pose.pose.position.z += noise_dist_z_(rng_);
        state_pub.header.stamp = ros::Time::now();
        detection_pub.publish(state_pub);
        yaw_deg_msg.data = 1.0;
        detection_if.publish(yaw_deg_msg);
    } else {
        yaw_deg_msg.data = 0.0;
        detection_if.publish(yaw_deg_msg);
    }
}
void Sim_detect::drone_odom_cb(const nav_msgs::Odometry& odom) {
    drone_pos << odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z;
}