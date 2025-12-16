/*
该程序基于 ROS
框架实现了一个轨迹服务器（TrajectoryServer），主要用于控制四旋翼（或无人机）按照预先规划的多项式轨迹飞行。

输入：
程序通过接收里程计（Odometry）消息、轨迹消息（PolynomialTrajectory）以及预测位置消息，

输出：
实时计算无人机的期望位置、速度和加速度，并发布控制命令（PositionCommand）给无人机。同时，还发布各种可视化消息（Marker、PointCloud2）用于调试和显示。

程序中还定义了一个
yaw_controller函数，用于计算期望偏航角，使无人机在运动过程中能平滑调整朝向。
程序主要分为两个部分：
全局辅助函数（例如 yaw_controller）
主类TrajectoryServer：负责订阅消息、处理轨迹和发布控制命令，其内部包含状态机（INIT、TRAJ、HOVER）以区分不同的控制状态。
*/
#include <geometry_msgs/PoseStamped.h>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <quadrotor_msgs/PolynomialTrajectory.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <qpOASES.hpp>
#include <vector>
USING_NAMESPACE_QPOASES
using namespace std;
#define PI acos(-1)
#define HOVER_SEG_NUM 1000  // 用于悬停时的分段数量

// 定义三维空间中 x, y, z 的索引，方便在数组或向量中访问对应维度。
const int _DIM_x = 0;
const int _DIM_y = 1;
const int _DIM_z = 2;

int hover_init_flag = 0;  // 用于悬停状态标志；
int hover_stop_flag = 0;
double destination_yaw = 0;               // 目标偏航角
double last_detect_time = 0;              // 上次检测到目标的时间；
bool emegency_flag = false;               // 紧急状态标志（当发生异常时触发）
vector<Eigen::Vector3d> history_poslist;  // 存储历史位置，用于紧急情况下回溯控制

Eigen::Vector4d target_detected_pos;  // 目标检测位置（4 维向量，可能包含位置及附加信息）

int vis_id = 0;  // 可视化标记的标识
using namespace std;
int emer_cur = 0;
int _poly_order_min, _poly_order_max;
int ec = 0;

// 新增全局或类内常量（例如在 TrajectoryServer 类私有成员中添加）：
const double dt = 0.05;         // 时间步长
const int Np = 10;              // 预测时域步数（可调参数）
Eigen::Matrix<double, 2, 2> Q;  // 位置误差权重（2x2）
Eigen::Matrix<double, 2, 2> R;  // 控制输入权重（2x2）
Eigen::Matrix<double, 2, 2> F;  // 终端状态权重（2x2）

/*
目标：计算无人机期望的偏航角，使无人机在运动过程中平滑调整朝向。
基本原理：根据目标偏航角与当前里程计中获得的无人机朝向之间的差值，利用简单的比例控制（P 控制）来生成修正量。

为处理角度环绕问题（例如，当两个角度差值跨越 ±𝜋,±π 时），代码判断误差是否大于 PI，并做相应调整。
为了防止控制输出过大，输出值经过幅值限制（阈值 0.15）。
最后返回的期望偏航角等于当前朝向加上修正量。
*/
double yaw_controller(double yaw, double odom_yaw) {
    double k = 1;  // 比例控制系数
    double error;
    // 根据两角差值处理环绕问题
    if (fabs(odom_yaw - yaw) <= PI) {
        error = yaw - odom_yaw;
    } else if (yaw - odom_yaw < -PI) {
        error = yaw - odom_yaw + 2 * PI;
    } else {
        error = yaw - odom_yaw - 2 * PI;
    }
    double c_yaw;
    // cout<<"k*error : "<<k*error<<endl;

    // proportional control
    // 限制控制量，防止过大
    double ampl_threshold = 0.15;
    if (k * error > ampl_threshold)
        c_yaw = ampl_threshold;
    else if (k * error < (-1 * ampl_threshold))
        c_yaw = -1 * ampl_threshold;
    else
        c_yaw = k * error;

    // 1-level control
    //  double time_taw = 0.05;
    //  if(error < 0.2){
    //      c_yaw = error * (1 - exp(-1 * yaw_pass_time / time_taw));
    //  }
    // 返回修正后的偏航角（加上当前里程计偏航角，使得输出为实际角度）
    return c_yaw + odom_yaw;
    // return k*error+odom_yaw;
}

// 该类封装了整个轨迹服务器的功能，包括消息订阅、状态处理、轨迹解析、状态更新与控制指令发布。
class TrajectoryServer {
   private:
    // Subscribers
    ros::Subscriber _odom_sub;
    ros::Subscriber _traj_sub;
    ros::Subscriber _predict_sub;

    // publishers
    ros::Publisher _cmd_pub;
    ros::Publisher _vis_cmd_pub;
    ros::Publisher _vis_vel_pub;
    ros::Publisher _vis_acc_pub;
    ros::Publisher _vis_traj_pub;
    ros::Publisher _vis_traj_points;
    ros::Publisher _vis_desired_pos;  // 发布期望位置的可视化 Marker。

    // configuration for trajectory
    int _n_segment = 0;                           // 轨迹段数
    int _traj_id = 0;                             // 轨迹编号
    uint32_t _traj_flag = 0;                      // 轨迹状态标志，依据 quadrotor_msgs::PositionCommand 定义
    Eigen::VectorXd _time;                        // 记录每段轨迹的持续时间（Eigen 向量）
    Eigen::Vector3d final_pos;                    // 轨迹终点位置
    vector<Eigen::MatrixXd> _normalizedcoeflist;  // 存储各段轨迹的归一化多项式系数矩阵，每个矩阵对应一段轨迹
    vector<int> _order;                           // 每段轨迹的多项式阶数
    double _vis_traj_width = 0.2;                 // 轨迹可视化宽度
    double mag_coeff;                             // 轨迹时间缩放系数
    ros::Time _final_time = ros::TIME_MIN;        // 轨迹起止时间
    ros::Time _start_time = ros::TIME_MAX;
    double _start_yaw = 0.0, _final_yaw = 0.0;  // 轨迹起始和终止时的偏航角

    // 初始化权重矩阵（可根据实际情况调整）
    Q = Eigen::Matrix2d::Identity() * 10.0;  // 对位置误差加大权重
    R = Eigen::Matrix2d::Identity() * 1.0;   // 对加速度消耗较小惩罚
    F = Eigen::Matrix2d::Identity() * 50.0;  // 终端状态要求更严格

    // state of the server
    // enum ServerState{INIT, TRAJ, HOVER} state = INIT;
    /*
    INIT：初始化状态，尚未进入轨迹执行；
    TRAJ：处于执行轨迹状态；
    HOVER：悬停状态（通常在轨迹执行结束或异常时进入）。
    */
    enum ServerState { INIT = 0,
                       TRAJ,
                       HOVER } state = INIT;
    ;
    nav_msgs::Odometry _odom;              // 存储最新的里程计信息
    quadrotor_msgs::PositionCommand _cmd;  // 存储发布的控制命令
    geometry_msgs::PoseStamped _vis_cmd;   // 用于发布可视化期望位置的消息

    visualization_msgs::Marker _vis_vel, _vis_acc, _vis_pos;  // 各种可视化 Marker 对象
    visualization_msgs::Marker _vis_traj;

    sensor_msgs::PointCloud2 traj_pts;  // 用于轨迹点云的消息和 PCL 点云对象
    pcl::PointCloud<pcl::PointXYZ> traj_pts_pcd;

   public:
    vector<Eigen::VectorXd> CList;   // Position coefficients vector, used to record all the pre-compute 'n choose k' combinatorial for the bernstein coefficients .
    vector<Eigen::VectorXd> CvList;  // Velocity coefficients vector.
    vector<Eigen::VectorXd> CaList;  // Acceleration coefficients vector.
    // CList, CvList, CaList：分别存储位置、速度和加速度的多项式系数向量，用于记录各类预计算的 Bernstein 系数组合。

    /*
    创建所有需要的订阅者和发布者，并设定消息队列大小和传输提示（例如 TCP_NO_DELAY）。
    调用 setGains 函数设置位置和速度控制增益。这里增益值存储在数组 pos_gain 和 vel_gain 中，分别用于 x、y、z 三个维度。
    初始化可视化 Marker 对象，如轨迹 Marker、期望位置 Marker，并设定颜色、比例、帧 ID 等参数。
    */
    TrajectoryServer(ros::NodeHandle& handle) {
        // 在rcvOdometryCallback回调内发布cmd命令，内有pubPositionCommand函数
        _odom_sub = handle.subscribe("odometry", 50, &TrajectoryServer::rcvOdometryCallback, this, ros::TransportHints().tcpNoDelay());

        _traj_sub = handle.subscribe("trajectory", 2, &TrajectoryServer::rcvTrajectoryCallabck, this);

        _predict_sub = handle.subscribe("front_pos_forpredict", 1, &TrajectoryServer::frontPosPredictCallback, this);

        _cmd_pub = handle.advertise<quadrotor_msgs::PositionCommand>("position_command", 50);

        _vis_cmd_pub = handle.advertise<geometry_msgs::PoseStamped>("desired_position", 50);

        _vis_vel_pub = handle.advertise<visualization_msgs::Marker>("desired_velocity", 50);

        _vis_acc_pub = handle.advertise<visualization_msgs::Marker>("desired_acceleration", 50);

        _vis_traj_pub = handle.advertise<visualization_msgs::Marker>("trajectory_vis", 1);
        _vis_desired_pos = handle.advertise<visualization_msgs::Marker>("desired_pos_vis", 1);

        _vis_traj_points = handle.advertise<sensor_msgs::PointCloud2>("trajectory_vis_points", 1);

        double pos_gain[3] = {5.7, 5.7, 6.2};
        double vel_gain[3] = {3.4, 3.4, 4.0};
        setGains(pos_gain, vel_gain);  // 调用setGains 函数设置位置和速度控制增益

        _vis_traj.header.stamp = ros::Time::now();
        _vis_traj.header.frame_id = "world";
        _vis_traj.ns = "trajectory/trajectory";
        _vis_traj.id = vis_id;
        _vis_traj.type = visualization_msgs::Marker::SPHERE_LIST;
        _vis_traj.action = visualization_msgs::Marker::ADD;
        _vis_traj.scale.x = _vis_traj_width / 1.3;
        _vis_traj.scale.y = _vis_traj_width / 1.3;
        _vis_traj.scale.z = _vis_traj_width / 1.3;
        _vis_traj.pose.orientation.x = 0.0;
        _vis_traj.pose.orientation.y = 0.0;
        _vis_traj.pose.orientation.z = 0.0;
        _vis_traj.pose.orientation.w = 1.0;
        _vis_traj.color.r = 1.0;
        _vis_traj.color.g = 1.0;
        _vis_traj.color.b = 0.0;
        _vis_traj.color.a = 0.0;
        _vis_traj.points.clear();

        _vis_pos.header.stamp = ros::Time::now();
        _vis_pos.header.frame_id = "world";
        _vis_pos.ns = "/desired_pos";
        _vis_pos.id = 0;
        _vis_pos.type = visualization_msgs::Marker::SPHERE_LIST;
        _vis_pos.action = visualization_msgs::Marker::ADD;
        _vis_pos.scale.x = _vis_traj_width;
        _vis_pos.scale.y = _vis_traj_width;
        _vis_pos.scale.z = _vis_traj_width;
        _vis_pos.pose.orientation.x = 0.0;
        _vis_pos.pose.orientation.y = 0.0;
        _vis_pos.pose.orientation.z = 0.0;
        _vis_pos.pose.orientation.w = 1.0;
        _vis_pos.color.r = 1.0;
        _vis_pos.color.g = 0.0;
        _vis_pos.color.b = 1.0;
        _vis_pos.color.a = 1.0;
        _vis_pos.points.clear();
    }

    void setGains(double pos_gain[3], double vel_gain[3]) {
        // 该函数用于设置控制命令中位置和速度控制增益，便于调整无人机控制性能。它将传入的 pos_gain 和 vel_gain 数组的值赋值给 _cmd.kx 和 _cmd.kv 的相应维度。
        _cmd.kx[_DIM_x] = pos_gain[_DIM_x];
        _cmd.kx[_DIM_y] = pos_gain[_DIM_y];
        _cmd.kx[_DIM_z] = pos_gain[_DIM_z];

        _cmd.kv[_DIM_x] = vel_gain[_DIM_x];
        _cmd.kv[_DIM_y] = vel_gain[_DIM_y];
        _cmd.kv[_DIM_z] = vel_gain[_DIM_z];
    }

    bool cmd_flag = false;
    void frontPosPredictCallback(const nav_msgs::Path& front_pos) {
        target_detected_pos << front_pos.poses[0].pose.position.x, front_pos.poses[0].pose.position.y, front_pos.poses[0].pose.position.z, front_pos.poses[0].pose.orientation.x;
    }
    void rcvOdometryCallback(const nav_msgs::Odometry& odom) {
        // ROS_WARN("state = %d",state);

        if (odom.child_frame_id == "X" || odom.child_frame_id == "O")
            return;
        // #1. store the odometry
        _odom = odom;
        _vis_cmd.header = _odom.header;
        _vis_cmd.header.frame_id = "world";

        if (state == INIT) {
            // ROS_WARN("[TRAJ SERVER] Pub initial pos command");
            //_cmd.position   = _odom.pose.pose.position;
            _cmd.position.x = _odom.pose.pose.position.x;
            _cmd.position.y = _odom.pose.pose.position.y;
            _cmd.position.z = 1.0;

            _cmd.header.stamp = _odom.header.stamp;
            _cmd.header.frame_id = "world";
            //_cmd.trajectory_flag = _traj_flag;
            _cmd.trajectory_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;

            _cmd.velocity.x = 0.0;
            _cmd.velocity.y = 0.0;
            _cmd.velocity.z = 0.0;

            _cmd.acceleration.x = 0.0;
            _cmd.acceleration.y = 0.0;
            _cmd.acceleration.z = 0.0;
            _cmd_pub.publish(_cmd);

            _vis_cmd.pose.position.x = _cmd.position.x;
            _vis_cmd.pose.position.y = _cmd.position.y;
            _vis_cmd.pose.position.z = _cmd.position.z;
            _vis_cmd_pub.publish(_vis_cmd);

            return;
        }

        // change the order between #2 and #3. zxzxzxzx

        // #2. try to calculate the new state
        if (state == TRAJ && ((odom.header.stamp - _start_time).toSec() / mag_coeff > (_final_time - _start_time).toSec())) {
            state = HOVER;
            _traj_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_COMPLETED;
        }

        // #3. try to publish command
        pubPositionCommand();  // 最后调用 pubPositionCommand() 函数发布实时的控制命令。
    }

    void rcvTrajectoryCallabck(const quadrotor_msgs::PolynomialTrajectory& traj) {
        // ROS_WARN("[SERVER] Recevied The Trajectory with %.3lf.", _start_time.toSec());
        // ROS_WARN("[SERVER] Now the odom time is : ");
        //  #1. try to execuse the action

        /*
        接收到新的轨迹规划，重置紧急标志（emegency_flag）、设置状态为 TRAJ，保存轨迹 ID、段数、时间数组、偏航角等信息；同时，将每段轨迹的多项式系数存入 _normalizedcoeflist。
        */
        if (traj.action == quadrotor_msgs::PolynomialTrajectory::ACTION_ADD) {
            if (emegency_flag) {
                emegency_flag = false;
                history_poslist.clear();
                emer_cur = -1;
            }
            state = TRAJ;
            _traj_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;
            _traj_id = traj.trajectory_id;
            _n_segment = traj.num_segment;
            _final_time = _start_time = traj.header.stamp;
            _time.resize(_n_segment);

            _order.clear();
            _normalizedcoeflist.clear();
            for (int idx = 0; idx < _n_segment; ++idx) {
                _final_time += ros::Duration(traj.time[idx]);
                _time(idx) = traj.time[idx];
                _order.push_back(traj.order[idx]);
            }

            _start_yaw = traj.start_yaw;
            _final_yaw = traj.final_yaw;
            mag_coeff = traj.mag_coeff;

            // ROS_WARN("stack the coefficients");
            int shift = 0;
            for (int idx = 0; idx < traj.num_segment; idx++) {
                int order = traj.order[idx];
                Eigen::MatrixXd coefmat;
                coefmat = Eigen::MatrixXd::Zero(3, order + 1);

                for (int j = 0; j <= order; j++) {
                    coefmat(0, j) = traj.coef_x[shift + j];
                    coefmat(1, j) = traj.coef_y[shift + j];
                    coefmat(2, j) = traj.coef_z[shift + j];
                }
                _normalizedcoeflist.push_back(coefmat);
                shift += (order + 1);
            }
            hover_stop_flag = 0;
        } else if (traj.action == quadrotor_msgs::PolynomialTrajectory::ACTION_ABORT) {
            ROS_WARN("[SERVER] Aborting the trajectory.");
            state = HOVER;
            _traj_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_COMPLETED;
        } else if (traj.action == quadrotor_msgs::PolynomialTrajectory::ACTION_WARN_IMPOSSIBLE) {
            state = HOVER;
            hover_stop_flag = 0;
            _traj_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_IMPOSSIBLE;
        } else if (traj.action == quadrotor_msgs::PositionCommand::ACTION_STOP) {
            ROS_WARN("Emergency!!!");
            state = HOVER;
            emegency_flag = true;
            _traj_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_COMPLETED;
        }
    }

    /*
    这是控制器的核心函数，负责根据当前状态计算并发布控制命令。
    该函数的主要作用是根据当前无人机的状态（由状态机 state 控制）计算并发布期望控制命令，控制无人机沿预规划轨迹飞行，并同时发布可视化信息。函数分为几个部分：
（1）状态检查
INIT 状态
如果状态处于 INIT（初始化状态），则不发布控制命令（直接返回）。
HOVER 状态
当状态为 HOVER（悬停状态）时，函数将期望位置设置为当前里程计位置，同时将速度和加速度设为零。如果处于紧急状态（emegency_flag 为 true），则利用保存的历史位置（history_poslist）进行回溯控制，即选择历史中较早的位置信息作为期望位置。

（2）轨迹段定位与期望状态计算（TRAJ 状态）
当状态为 TRAJ（执行轨迹状态）时，函数首先计算当前时刻与轨迹开始时间（_start_time）的差值 t，并通过遍历存储各段持续时间的数组 _time 来确定当前所在的轨迹段（seg_idx）。
将当前段内时间归一化后，利用预先存储的归一化多项式系数（_normalizedcoeflist）以及每段的多项式阶数（_order）采用逐项累加的方式计算期望位置、速度和加速度。
其中，用 tn、tnvel、tnacc 分别作为 t 的幂次系数，配合组合系数（n, k, l）计算各项对位置、速度、加速度的贡献。
计算出的速度与加速度进一步归一化，除以当前段的持续时间（以及时间平方）得到正确量纲。

（3）偏航角计算
利用计算得到的期望速度分量，通过 atan2 得到无人机期望的运动方向，然后调用 yaw_controller 函数对偏航角进行修正，使无人机运动中保持平滑转向。
同时还设置了一个固定的偏航角速度（yaw_dot）。

（4）命令发布与可视化
将计算得到的期望状态（位置、速度、加速度、偏航角等）打包到 _cmd 消息中，并通过 _cmd_pub 发布控制命令。
同时构造一系列用于可视化调试的 ROS 消息（例如 _vis_cmd、_vis_vel、_vis_acc、_vis_traj 以及点云消息 traj_pts），发布到各自的主题上，方便观察当前期望状态和轨迹情况。
    */
    void pubPositionCommand() {
        // #1. check if it is right state
        if (state == INIT)
            return;
        if (state == HOVER) {
            // 如果状态为 HOVER，则将控制命令设置为当前 odom 的位置，速度、加速度均置零。若处于紧急状态，还会利用 history_poslist 回溯历史位置进行控制。
            if (_cmd.header.frame_id != "world") {
                _cmd.position = _odom.pose.pose.position;
            }
            // 更新命令头部信息，设置时戳和参考坐标系
            _cmd.header.stamp = _odom.header.stamp;
            _cmd.header.frame_id = "world";
            _cmd.trajectory_flag = _traj_flag;  // 设置轨迹状态标志
            // 紧急状态下，利用历史位置回溯控制
            if (emegency_flag) {
                // 通过变量 ec 来决定回溯步长
                if (ec == 0) {
                    emer_cur -= 1;
                    ec++;
                } else {
                    ec--;
                }
                // 确保索引不越界
                if (emer_cur < 0)
                    emer_cur = 0;
                // 设置期望位置为历史记录中较早的位置信息
                _cmd.position.x = history_poslist[emer_cur][0];
                _cmd.position.y = history_poslist[emer_cur][1];
                _cmd.position.z = history_poslist[emer_cur][2];
            }
            // 悬停时速度和加速度全部置零
            _cmd.velocity.x = 0.0;
            _cmd.velocity.y = 0.0;
            _cmd.velocity.z = 0.0;

            _cmd.acceleration.x = 0.0;
            _cmd.acceleration.y = 0.0;
            _cmd.acceleration.z = 0.0;
        }
        // #2. locate the trajectory segment
        /*
        根据当前时间（odom 时间与轨迹开始时间之差）和预先存储的各段时间 _time，确定当前所在的轨迹段（seg_idx）。
        将时间 t 归一化为当前段的比例，然后利用 Bernstein 多项式法（或直接利用多项式系数）计算当前的期望位置、速度和加速度。
        对速度和加速度进行归一化（根据当前段时间）。
        更新 _cmd 中的各个字段：位置、速度、加速度，并计算偏航角（利用 atan2 从速度分量中求出方向），调用 yaw_controller 修正偏航角。
        同时，将计算得到的期望状态存入历史记录 history_poslist。

        发布控制命令与可视化消息：
        最后，将构造好的 _cmd 消息发布给无人机，同时发布各类 Marker（用于速度、加速度、期望位置和轨迹）和点云消息，便于调试和可视化。
        */
        // 如果状态为 TRAJ，采用 MPC 算法计算最优控制序列
        if (state == TRAJ) {
            // 当前状态 X0 从最新里程计中获得（仅取 x,y, vx,vy）
            Eigen::Vector4d X0;
            X0 << _odom.pose.pose.position.x,
                _odom.pose.pose.position.y,
                _odom.twist.twist.linear.x,
                _odom.twist.twist.linear.y;

            // 期望目标 Q*（二维位置），假设 Q* 为由预测消息得到的目标位置
            // 这里我们假设 target_detected_pos 的前两个元素为期望位置
            Eigen::Vector2d Qstar;
            Qstar << target_detected_pos[0], target_detected_pos[1];

            // 定义系统矩阵 A 和 B（与上面公式相同）
            Eigen::Matrix4d A;
            A << 1, 0, dt, 0,
                0, 1, 0, dt,
                0, 0, 1, 0,
                0, 0, 0, 1;
            Eigen::Matrix<double, 4, 2> B;
            B << dt * dt / 2, 0,
                0, dt * dt / 2,
                dt, 0,
                0, dt;

            // 我们只关注位置误差，因此定义输出矩阵 C
            Eigen::Matrix<double, 2, 4> C;
            C << 1, 0, 0, 0,
                0, 1, 0, 0;

            // 预测模型：构造预测矩阵 S and T 使得：
            // X_pred = S*X0 + T*U, 其中 U 是 [u0; u1; ...; u_{Np-1}] ∈ R^(2*Np)
            const int n = 4;  // 状态维数
            const int m = 2;  // 控制输入维数
            Eigen::MatrixXd S = Eigen::MatrixXd::Zero(n * Np, n);
            Eigen::MatrixXd T = Eigen::MatrixXd::Zero(n * Np, m * Np);
            for (int i = 0; i < Np; i++) {
                S.block(i * n, 0, n, n) = A.pow(i + 1);
                for (int j = 0; j <= i; j++) {
                    T.block(i * n, j * m, n, m) = A.pow(i - j) * B;
                }
            }
            // 输出预测位置：P = C * X. 定义预测输出矩阵:
            Eigen::MatrixXd Cbar = Eigen::MatrixXd::Zero(2 * Np, n * Np);
            for (int i = 0; i < Np; i++) {
                Cbar.block(i * 2, i * n, 2, n) = C;
            }
            // 构造 QP cost：变量 U ∈ R^(2*Np)
            // 预测位置 = Cbar*(S*X0 + T*U).令 e = Cbar*(S*X0) - Qbar + Cbar*T*U,
            // 其中 Qbar 是由期望位置 Qstar 重复 Np 次构成的向量。
            Eigen::VectorXd Qbar = Eigen::VectorXd::Zero(2 * Np);
            for (int i = 0; i < Np; i++) {
                Qbar.segment(i * 2, 2) = Qstar;
            }
            // 定义权重矩阵 Qbar_cost 为 block-diagonal由 Q 重复 Np-1 次，终端用 F
            Eigen::MatrixXd Qbar_cost = Eigen::MatrixXd::Zero(2 * Np, 2 * Np);
            for (int i = 0; i < Np - 1; i++) {
                Qbar_cost.block(i * 2, i * 2, 2, 2) = Q;
            }
            Qbar_cost.block((Np - 1) * 2, (Np - 1) * 2, 2, 2) = F;  // 终端权重

            // 同理构造 Rbar 为 block-diagonal由 R 重复 Np 次
            Eigen::MatrixXd Rbar = Eigen::MatrixXd::Zero(2 * Np, 2 * Np);
            for (int i = 0; i < Np; i++) {
                Rbar.block(i * 2, i * 2, 2, 2) = R;
            }

            // 构造 Hessian H 和梯度 g
            Eigen::MatrixXd H = T.transpose() * Cbar.transpose() * Qbar_cost * Cbar * T + Rbar;
            Eigen::VectorXd g = T.transpose() * Cbar.transpose() * Qbar_cost * (Cbar * S * X0 - Qbar);

            // 设置加速度限幅（如果需要，可加入约束）；本示例暂不加速度约束
            // 注意：还可以增加速度约束、碰撞约束等

            // 求解 QP 问题：min 0.5*Uᵀ H U + gᵀ U, 变量 U ∈ R^(2*Np)
            // 使用 qpOASES 库求解
            int nV = 2 * Np;  // 变量数
            int nC = 0;       // 这里我们不考虑不等式约束（约束可以后续添加）
            QProblem qp(nV, nC);
            Options options;
            options.printLevel = PL_NONE;
            qp.setOptions(options);
            int nWSR = 100;
            qp.init(H.data(), g.data(), nullptr, nullptr, nullptr, nullptr, nullptr, nWSR);
            Eigen::VectorXd Uopt(nV);
            qp.getPrimalSolution(Uopt.data());

            // 提取第一个控制量 u0* = [ax, ay]
            Eigen::Vector2d u0 = Uopt.segment(0, 2);

            // 预测下一时刻状态 X1 = A * X0 + B * u0
            Eigen::Vector4d X1 = A * X0 + B * u0;

            // 将计算结果赋值给控制命令 _cmd：
            // 期望位置取 X1 前两个分量
            _cmd.position.x = X1(0);
            _cmd.position.y = X1(1);
            // 对于 z 方向，我们可直接沿用原来的命令（或固定一个高度）
            _cmd.position.z = _cmd.position.z;  // 或者设定为某个常数高度

            // 期望速度取 X1 后两个分量
            _cmd.velocity.x = X1(2);
            _cmd.velocity.y = X1(3);
            // 同理，z 方向速度置零或采用其他控制
            _cmd.velocity.z = 0.0;

            // 期望加速度即 u0
            _cmd.acceleration.x = u0(0);
            _cmd.acceleration.y = u0(1);
            _cmd.acceleration.z = 0.0;  // z方向控制另行设计

            // 利用期望速度计算无人机运动方向（偏航角），并利用 yaw_controller 进行修正
            _cmd.yaw = atan2(_cmd.velocity.y, _cmd.velocity.x);
            double odom_yaw = 0;
            Eigen::Quaterniond q;
            q.w() = _odom.pose.pose.orientation.w;
            q.x() = _odom.pose.pose.orientation.x;
            q.y() = _odom.pose.pose.orientation.y;
            q.z() = _odom.pose.pose.orientation.z;
            q.normalize();
            odom_yaw = atan2(2 * (q.w() * q.z() + q.x() * q.y()), 1 - 2 * (q.y() * q.y() + q.z() * q.z()));
            _cmd.yaw = yaw_controller(_cmd.yaw, odom_yaw);
            _cmd.yaw_dot = 0.01;  // 固定偏航速率
        }

        // #4. 发布控制命令及可视化消息（后续可沿用原有可视化代码）
        _cmd_pub.publish(_cmd);
        _vis_cmd.header = _cmd.header;
        _vis_cmd.pose.position = _cmd.position;
        tf::Quaternion q_ = tf::createQuaternionFromYaw(_cmd.yaw);
        geometry_msgs::Quaternion odom_quat;
        tf::quaternionTFToMsg(q_, odom_quat);
        _vis_cmd.pose.orientation = odom_quat;
        _vis_cmd_pub.publish(_vis_cmd);

        // 构造速度可视化 Marker（箭头），表示期望速度方向
        _vis_vel.ns = "vel";
        _vis_vel.id = 0;
        _vis_vel.header.frame_id = "world";
        _vis_vel.type = visualization_msgs::Marker::ARROW;
        _vis_vel.action = visualization_msgs::Marker::ADD;
        _vis_vel.color.a = 1.0;
        _vis_vel.color.r = 0.0;
        _vis_vel.color.g = 1.0;
        _vis_vel.color.b = 0.0;
        _vis_vel.header.stamp = _odom.header.stamp;
        _vis_vel.points.clear();

        // 用当前期望位置构造箭头起点
        geometry_msgs::Point pt;
        pt.x = _cmd.position.x;
        pt.y = _cmd.position.y;
        pt.z = _cmd.position.z;
        // 清空并更新轨迹 Marker 中的点（显示路径点）
        _vis_traj.points.clear();
        _vis_traj.points.push_back(pt);
        _vis_traj_pub.publish(_vis_traj);

        // 构造期望位置 Marker（Sphere List）
        _vis_pos.points.clear();
        _vis_pos.points.push_back(pt);
        _vis_desired_pos.publish(_vis_pos);

        // 构造轨迹点云信息
        pcl::PointXYZ point(pt.x, pt.y, pt.z);
        traj_pts_pcd.points.clear();
        traj_pts_pcd.points.push_back(point);
        traj_pts_pcd.width = traj_pts_pcd.points.size();
        traj_pts_pcd.height = 1;
        traj_pts_pcd.is_dense = true;
        pcl::toROSMsg(traj_pts_pcd, traj_pts);
        traj_pts.header.frame_id = "world";
        _vis_traj_points.publish(traj_pts);

        // 更新速度 Marker的点，箭头从期望位置指向期望位置加上期望速度
        _vis_vel.points.push_back(pt);
        pt.x = _cmd.position.x + _cmd.velocity.x;
        pt.y = _cmd.position.y + _cmd.velocity.y;
        pt.z = _cmd.position.z + _cmd.velocity.z;
        _vis_vel.points.push_back(pt);
        _vis_vel.scale.x = 0.2;
        _vis_vel.scale.y = 0.4;
        _vis_vel.scale.z = 0.4;
        _vis_vel_pub.publish(_vis_vel);

        // 构造加速度 Marker（箭头），从期望位置指向期望位置加上期望加速度
        _vis_acc.ns = "acc";
        _vis_acc.id = 0;
        _vis_acc.header.frame_id = "world";
        _vis_acc.type = visualization_msgs::Marker::ARROW;
        _vis_acc.action = visualization_msgs::Marker::ADD;
        _vis_acc.color.a = 1.0;
        _vis_acc.color.r = 1.0;
        _vis_acc.color.g = 1.0;
        _vis_acc.color.b = 0.0;
        _vis_acc.header.stamp = _odom.header.stamp;
        _vis_acc.points.clear();
        pt.x = _cmd.position.x;
        pt.y = _cmd.position.y;
        pt.z = _cmd.position.z;
        _vis_acc.points.push_back(pt);
        pt.x = _cmd.position.x + _cmd.acceleration.x;
        pt.y = _cmd.position.y + _cmd.acceleration.y;
        pt.z = _cmd.position.z + _cmd.acceleration.z;
        _vis_acc.points.push_back(pt);
        _vis_acc.scale.x = 0.2;
        _vis_acc.scale.y = 0.4;
        _vis_acc.scale.z = 0.4;
        _vis_acc_pub.publish(_vis_acc);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "gradient_trajectory_server_mpc_node");
    ros::NodeHandle handle("~");

    TrajectoryServer server(handle);

    ros::spin();

    return 0;
}
