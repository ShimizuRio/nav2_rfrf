#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()
    channel_type = LaunchConfiguration("channel_type", default="serial")
    serial_port_f = LaunchConfiguration(
        "serial_port_f", default="/dev/ttyUSB-RPLiDAR_f"
    )
    serial_port_b = LaunchConfiguration(
        "serial_port_b", default="/dev/ttyUSB-RPLiDAR_b"
    )
    serial_baudrate = LaunchConfiguration("serial_baudrate", default="256000")
    frame_id = LaunchConfiguration("frame_id", default="laser")
    inverted = LaunchConfiguration("inverted", default="false")
    angle_compensate = LaunchConfiguration("angle_compensate", default="true")
    namespace = LaunchConfiguration("__ns", default="")

    arg0 = DeclareLaunchArgument(
        "channel_type",
        default_value=channel_type,
        description="Specifying channel type of lidar",
    )

    arg1 = DeclareLaunchArgument(
        "serial_port_f",
        default_value=serial_port_f,
        description="Specifying usb port to connected lidar",
    )
    arg2 = DeclareLaunchArgument(
        "serial_port_b",
        default_value=serial_port_b,
        description="Specifying usb port to connected lidar",
    )

    arg3 = DeclareLaunchArgument(
        "serial_baudrate",
        default_value=serial_baudrate,
        description="Specifying usb port baudrate to connected lidar",
    )

    arg4 = DeclareLaunchArgument(
        "frame_id", default_value=frame_id, description="Specifying frame_id of lidar"
    )

    arg5 = DeclareLaunchArgument(
        "inverted",
        default_value=inverted,
        description="Specifying whether or not to invert scan data",
    )

    arg6 = DeclareLaunchArgument(
        "angle_compensate",
        default_value=angle_compensate,
        description="Specifying whether or not to enable angle_compensate of scan data",
    )

    front_rplidar = Node(
        package="rplidar_ros",
        executable="rplidar_node",
        name="front_rplidar_node",
        namespace=namespace,
        remappings=[("scan", "~/scan_front")],
        parameters=[
            {
                "channel_type": channel_type,
                "serial_port": serial_port_f,
                "serial_baudrate": serial_baudrate,
                "frame_id": frame_id,
                "inverted": inverted,
                "angle_compensate": angle_compensate,
            }
        ],
        output="screen",
    )

    back_rplidar = Node(
        package="rplidar_ros",
        executable="rplidar_node",
        name="back_rplidar_node",
        namespace=namespace,
        remappings=[("scan", "~/scan_back")],
        parameters=[
            {
                "channel_type": channel_type,
                "serial_port": serial_port_b,
                "serial_baudrate": serial_baudrate,
                "frame_id": frame_id,
                "inverted": inverted,
                "angle_compensate": angle_compensate,
            }
        ],
        output="screen",
    )

    # LaunchDescriptionに、起動したいノードを追加する
    ld.add_action(arg0)
    ld.add_action(arg1)
    ld.add_action(arg2)
    ld.add_action(arg3)
    ld.add_action(arg4)
    ld.add_action(arg5)
    ld.add_action(arg6)
    ld.add_action(front_rplidar)
    ld.add_action(back_rplidar)

    # launch構成を返すようにする
    return ld
