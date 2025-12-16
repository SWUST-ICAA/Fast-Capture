#!/bin/bash


# 启动 d2vins
gnome-terminal -- bash -c "cd ~/Fast_Capture && source devel/setup.bash && roslaunch plan_manage R_swarm.launch; exec bash"

# 稍等片刻再播放 bag
# sleep 0

# 播放 rosbag
# gnome-terminal -- bash -c "cd ~/Fast_Capture && source devel/setup.bash && roslaunch multi_uav_target_estimator estimator.launch ; exec bash"
