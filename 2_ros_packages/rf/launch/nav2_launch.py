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
    
    # map_path: /common/... にある地図ファイルへの絶対パスを指定
    map_path = LaunchConfiguration('map', 
        default='/common/ros_launcher/launch_slam_toolbox/bld10_4F.yaml')

    # params_file: /common/... にあるNav2パラメータファイルへの絶対パスを指定
    params_file = LaunchConfiguration('params_file', 
        default='/common/ros_launcher/launch_nav2/localization.yaml')


    # --- 実行するアクションの定義 ---

    # 1. Nav2スタック全体を起動（AMCL, Map Server, Controller, Plannerなど）
    # bringup_launch.pyをインクルードして、必要な設定を渡します
    start_nav2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_pkg_path, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'map': map_path,
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': 'true'
        }.items()
    )

    # 2. 【最重要】あなたの「頭脳」ノード (robot_node.py) を起動
    # これにより、Nav2へのゴール指示やタスク管理が行われます
    start_robot_node_cmd = Node(
        package=my_main_pkg_name,             # robot_node.pyが含まれるパッケージ名
        executable='robot_node', # setup.pyのentry_pointsで設定した実行可能ファイル名
        name='robot_node',
        output='screen'
    )

    # --- LaunchDescriptionの構築 ---
    ld = LaunchDescription()

    # 作成したアクションをLaunchDescriptionに追加
    ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='False', description='Use simulation (Gazebo) clock if true'))
    ld.add_action(DeclareLaunchArgument('map', default_value=map_path, description='Full path to map file'))
    ld.add_action(DeclareLaunchArgument('params_file', default_value=params_file, description='Full path to Nav2 params file'))

    ld.add_action(start_nav2_cmd)
    ld.add_action(start_robot_node_cmd)

    return ld

