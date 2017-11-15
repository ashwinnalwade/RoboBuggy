#!/bin/bash
source /opt/ros/kinetic/setup.bash
cd ~/RoboBuggy/Software/real_time/ROS_RoboBuggy/
source devel/setup.bash
ls
cd ../../
roslaunch robobuggy transistor.launch &
rosbag record -a &
