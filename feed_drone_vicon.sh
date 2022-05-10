#!/bin/bash
echo "Feed vicon pose directly to the drone ..."

# [IMPORTANT] change your drone IP address accordingly
export ROS_MASTER_URI=http://192.168.0.60:11311
export ROS_IP=192.168.0.21


# [IMPORTANT] change the catkin_ws to match with your ws
source ~/catkin_ws/devel/setup.bash
roslaunch vicon_feeder vicon_feed.launch