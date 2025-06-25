#! /usr/bin/bash

# target=$1
# # target
# #  rf : building os 用 RP向け
# #  robot : robot in the building
if [[ ! "${TAG}" == image_* ]]; then
  echo "Build first"
  bash
else
  $(echo "exec ros2 run rf building_node --log-opt max-size=100m --log-opt max-file=10 --ros-args --remap __ns:=/$HOSTNAME")
fi
