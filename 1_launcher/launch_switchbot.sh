#! /usr/bin/bash
$(echo "exec ros2 run switchbot switchbot_node --ros-args --remap __ns:=/$HOSTNAME")
