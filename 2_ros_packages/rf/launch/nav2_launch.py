import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    my_main_pkg_name = 'rf'
    #my_main_pkg_path = get_package_share_directory(my_main_pkg_name)

    # Nav2のbringupパッケージのパス
    nav2_bringup_pkg_path = get_package_share_directory('nav2_bringup')

    use_sim_time = LaunchConfiguration('use_sim_time', default='False')
    
    map_path = LaunchConfiguration('map', 
        default='/common/ros_launcher/launch_slam_toolbox/bld10_4F.yaml')

    params_file = LaunchConfiguration('params_file', 
        default='/common/ros_launcher/launch_nav2_demo/localization.yaml')


    # 1. localization（AMCL + map_server）を起動
    start_localization_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_pkg_path, 'launch', 'localization_launch.py')),
        launch_arguments={
            'map': map_path,
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': 'true'
        }.items()
    )

    # 2. navigation（controller, planner など）を起動
    start_navigation_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_pkg_path, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': 'true'
        }.items()
    )

    # robot_node.pyを起動
    start_robot_node_cmd = Node(
        package=my_main_pkg_name, 
        executable='robot_node',
        name='robot_node',
        output='screen'
    )

    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='False', description='Use simulation (Gazebo) clock if true'))
    ld.add_action(DeclareLaunchArgument('map', default_value=map_path, description='Full path to map file'))
    ld.add_action(DeclareLaunchArgument('params_file', default_value=params_file, description='Full path to Nav2 params file'))

    ld.add_action(start_localization_cmd)
    ld.add_action(start_navigation_cmd)
    ld.add_action(start_robot_node_cmd)

    return ld

