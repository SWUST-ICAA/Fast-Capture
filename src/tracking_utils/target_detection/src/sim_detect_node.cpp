/*
该ROS节点用于模拟无人机（drone）与车辆（car）之间的视线检测。主要功能包括：

构建三维网格地图：通过点云数据构建障碍物地图。
实时检测：当无人机与车辆之间的视线未被障碍物阻挡时，发布车辆状态。
输入：全局点云地图、车辆状态（位置）、无人机状态（位置）。
输出：未被遮挡的车辆状态（通过/target话题发布）。
*/
#include <target_detection/sim_detect.h>
int main(int argc, char** argv) {
    ros::init(argc, argv, "tracking_fsm_node");
    // ros::MultiThreadedSpinner spinner(16);
    ros::NodeHandle nh_priv("~");
    Sim_detect sim_detection(nh_priv);
    // spinner.spin();
    ros::spin();
    return 0;
}