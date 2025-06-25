import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_params_file = LaunchConfiguration('slam_params_file')
    map_file_name = LaunchConfiguration('map_file_name')
    map_start_pose = LaunchConfiguration('map_start_pose')

    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation/Gazebo clock')
    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(get_package_share_directory("slam_toolbox"),
                                   'config', 'mapper_params_localization.yaml'),
        description='Full path to the ROS2 parameters file to use for the slam_toolbox node')
    declare_map_file_name_argument = DeclareLaunchArgument(
        'map_file_name',
        default_value='bld10_4F',
        description='Select map')
    declare_map_start_pose_argument = DeclareLaunchArgument(
        'map_start_pose',
        default_value="[0.0, 0.0, 1.5708]",
        description='Start pose on map')

    start_localization_slam_toolbox_node = Node(
        parameters=[
          slam_params_file,
          {'use_sim_time': use_sim_time},
          {'map_file_name': map_file_name},
          {"map_start_pose": map_start_pose}
        ],
        package='slam_toolbox',
        executable='localization_slam_toolbox_node',
        name='slam_toolbox',
        output='screen')

    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time_argument)
    ld.add_action(declare_slam_params_file_cmd)
    ld.add_action(declare_map_file_name_argument)
    ld.add_action(declare_map_start_pose_argument)
    ld.add_action(start_localization_slam_toolbox_node)

    return ld