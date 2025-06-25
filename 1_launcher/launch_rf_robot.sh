#! /usr/bin/bash

# target=$1
# # target
# #  rf : building os 用 RP向け
# #  robot : robot in the building
source /opt/ros/${ROS_DISTRO}/setup.bash
case $1 in
"matlab")
  # export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
  # export ROS_STATIC_PEERS=$(ip a | grep 192 | grep -o 'inet [0-9]\+\.[0-9]\+\.[0-9]\+\.[0-9]\+' | grep -o [0-9].*)
  ros2 run rf robot_node --log-opt max-size=100m --log-opt max-file=10 --ros-args -p rid:=1 -p send_input:=0 --remap __ns:=/$HOSTNAME
  ;;
"kill")
  ros2 topic pub --once /rover_twist geometry_msgs/msg/Twist '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
  pkill ros2
  ;;
"set_target")
  # dup rf_robot set_target [1,1]
  ros2 topic pub --once /Robot1/console2robot std_msgs/msg/Int8MultiArray "{data: $2}" --qos-durability volatile --qos-depth 1
  ;;
"hb")
  ros2 topic echo /Robot1/Heartbeat
  ;;
*)
  ros2 run rf robot_node --log-opt max-size=100m --log-opt max-file=10 --ros-args -p rid:=1 -p send_input:=1 --remap __ns:=/$HOSTNAME
  ;;
esac
