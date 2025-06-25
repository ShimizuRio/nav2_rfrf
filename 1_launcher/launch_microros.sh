#! /usr/bin/bash

# cd /root/ros2_ws/
#micro-ros agent 起動コマンド
# if [[ ! "${TAG}" == image_* ]]; then
#   echo "Build first"
#   bash
# else
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export ROS_STATIC_PEERS=$(ip a | grep 192 | grep -o 'inet [0-9]\+\.[0-9]\+\.[0-9]\+\.[0-9]\+' | grep -o [0-9].*)
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB-megarover --log-opt max-size=100m --log-opt max-file=10 -r /common/ros_launcher/launch_microros/qos.xml --ros-args --remap __node:=microros_node --remap __ns:=/${HOSTNAME} -v6 --namespace-remapping /${HOSTNAME}
#$(echo "exec ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB-megarover --log-opt max-size=100m --log-opt max-file=10 --ros-args --remap __node:=microros_node --remap __ns:=/${HOSTNAME} -v6 --namespace-remapping /${HOSTNAME}")
# fi

#docker run -it --rm --net=host microros/micro-ros-agent:jazzy serial --dev /dev/ttyUSB-megarover
