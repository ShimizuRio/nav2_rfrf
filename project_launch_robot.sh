#! /usr/bin/bash +x

source /home/dstudent/nav2_rfrf/.acsl/bashrc
echo ${PROJECT}${TARGET}${x86}
echo ${ROS_DOMAIN_ID}

cd /home/dstudent/nav2_rfrf/.acsl/0_host_commands/scripts
dup microros

dup rf_tf
TAG=rplidar${x86} CONTAINER_NAME=rplidar_front COMPOSE_PROJECT_NAME=rplidar_front_rf ROS_LAUNCH=launch_rplidar.sh LARGS=front HOSTNAME=$(hostname | sed -e 's/-/_/g') docker compose -f /home/dstudent/nav2_rfrf/.acsl/4_docker/docker-compose.yml up common -d
TAG=rplidar${x86} CONTAINER_NAME=rplidar_back COMPOSE_PROJECT_NAME=rplidar_back_rf ROS_LAUNCH=launch_rplidar.sh LARGS=back HOSTNAME=$(hostname | sed -e 's/-/_/g') docker compose -f /home/dstudent/nav2_rfrf/.acsl/4_docker/docker-compose.yml up common -d

dup rf_robot
dup slam_toolbox localization bld10_4F
# docker run -it --rm -v /dev:/dev --privileged --net=host microros/micro-ros-agent:jazzy serial --dev /dev/ttyUSB-megarover -v4 --ros-args --remap __node:=microros_node --remap __ns:=/${HOSTNAME}
#-v6 --namespace-remapping /hogehoge --baud 115200
##complete
### slam_toolbox ###
#/home/dstudent/nav2_rfrf/.acsl/0_host_commands/scripts/dup slam_toolbox slam
### slam_toolbox END ###
