#! /usr/bin/bash

# $1 None or rf
cp -p /common/ros_launcher/launch_rplidar/* /root/ros2_ws/install/rplidar_ros/share/rplidar_ros/launch/
#export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
#export ROS_STATIC_PEERS=$(ip a | grep 192 | grep -o 'inet [0-9]\+\.[0-9]\+\.[0-9]\+\.[0-9]\+' | grep -o [0-9].*)
if [ $1 = "None" ]; then
  $(echo "exec ros2 launch rplidar_ros rplidar_s1_launch.py __ns:=/${HOSTNAME}")
else
  printenv | grep ROS
  #sleep 5
  ros2 launch rplidar_ros rplidar_s1_rf_${1}_launch.py frame_id:=${1}_lidar_link __ns:=/${HOSTNAME}
  #ros2 launch rplidar_ros rplidar_s1_rf_launch.py frame_id:=${1}_lidar_link __ns:=/${HOSTNAME}
  #$(echo "exec ros2 launch rplidar_ros rplidar_s1_rf_${1}_launch.py __ns:=/${HOSTNAME}")
fi
