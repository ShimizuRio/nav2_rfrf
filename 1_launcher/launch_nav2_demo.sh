#! /usr/bin/bash

#if [[ $1 == "save_map" ]]; then
source /opt/ros/${ROS_DISTRO}/setup.bash
case $1 in
"save_map")
  echo "Save map"
  $(echo "exec ros2 run nav2_map_server map_saver_cli -f /common/ros_launcher/launch_slam_toolbox/map")
  ;;
"slam")
  # ros2 run ros_gz_bridge parameter_bridge /cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist
  # ros2 run ros_gz_bridge parameter_bridge /scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan
  # ros2 run ros_gz_bridge parameter_bridge /model/Robot1/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V --ros-args -r /model/Robot1/tf:=/tf
  # ros2 run ros_gz_bridge parameter_bridge /model/Robot1/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry --ros-args -r /model/Robot1/odometry:=/odom
  ros2 launch slam_toolbox online_async_launch.py slam_params_file:=/common/ros_launcher/launch_slam_toolbox/rf_robot_slam.yaml
  ;;
"localization")
  if [[ $# -ge 2 ]]; then
    map="map_file_name:=/common/ros_launcher/launch_slam_toolbox/$2"
  else
    map="map_file_name:=/common/ros_launcher/launch_slam_toolbox/bld10_4F"
  fi
  if [[ $# -ge 3 ]]; then
    start="map_start_pose:=${@:3:($# - 1)}"
  else
    start="map_start_pose:=[0.0, 0.0, 1.5708]"
  fi
  #echo "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
  #echo "start: "$start "@" ${@:3:($# - 1)}
  cp -f /common/ros_launcher/launch_slam_toolbox/rf_robot_localization_launch.py /opt/ros/${ROS_DISTRO}/share/slam_toolbox/launch/localization_launch.py
  # ros2 launch nav2_bringup localization_launch.py params_file:=/common/ros_launcher/launch_nav2_demo/localization.yaml "$map" "$start"
  ros2 launch nav2_bringup localization_launch.py \
  params_file:=/common/ros_launcher/launch_nav2_demo/localization.yaml \
  map_file_name:=/common/ros_launcher/launch_slam_toolbox/bld10_4F.yaml \
  map_start_pose:="[0.0, 0.0, 1.5708]"
  
  # cp /common/ros_launcher/launch_slam_toolbox/rf_robot_localization_launch.py /opt/ros/${ROS_DISTRO}/share/slam_toolbox/launch/
  #ros2 launch slam_toolbox /common/ros_launcher/launch_slam_toolbox/rf_robot_localization_launch.py slam_params_file:=/common/ros_launcher/launch_slam_toolbox/rf_robot_localization.yaml $map $start
  ;;
"rviz2")
  rviz2 -d /common/ros_launcher/launch_slam_toolbox/slam_toolbox.rviz
  ;;
esac
