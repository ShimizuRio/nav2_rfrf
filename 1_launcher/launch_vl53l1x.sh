#! /usr/bin/bash

# cd /root/ros2_ws/

# echo "build vl53l1x"
# colcon build --symlink-install --packages-select acsl_interfaces vl53l1x
# source install/setup.bash
#/common/scripts/set_path_in_container.sh

# フライトコントローラとの通信ノード実行
echo $TAG
if [[ ! "${TAG}" == image_* ]]; then
  echo "Build first"
  bash
else
  $(echo "exec ros2 run vl53l1x vl53l1x_node --log-opt max-size=100m --log-opt max-file=10 --ros-args --remap __node:=vl53l1x_node --remap __ns:=/$HOSTNAME")
fi
#$(echo "exec ros2 run multi_vl53l1x main --log-opt max-size=100m --log-opt max-file=10 --ros-args -p vl_max_num:=1 --remap __ns:=$HOSTNAME")
#$(echo "exec ros2 run multi_vl53l1x quatro_main --ros-args --remap __ns:=$HOSTNAME")
#$(echo "exec ros2 run vl53l1x vl53l1x_node --ros-args --remap __ns:=$HOSTNAME")
