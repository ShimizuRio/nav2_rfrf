#! /usr/bin/bash +x

source /home/dstudent/nav2_rfrf/.acsl/bashrc
echo ${PROJECT}${TARGET}${x86}
echo ${ROS_DOMAIN_ID}

cd /home/dstudent/nav2_rfrf/.acsl/0_host_commands/scripts/

#dup vl53l1x
dup switchbot
dup rf_building
