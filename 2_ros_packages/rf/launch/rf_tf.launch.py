import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    urdf_file = os.path.join(get_package_share_directory('rf'), 'urdf', 'megarover.urdf')
    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': open(urdf_file).read()}]
        ),
        # Node(
        #     package='rf',
        #     executable='robot_node',
        #     name='robot_node',
        #     output='screen',
        #     parameters=[
        #         {'log-opt': ['max-size=100m', 'max-file=10']}
        #     ],
        #     remappings=[
        #         ('__ns', f'/{os.getenv("HOSTNAME")}')
        #     ]
        # )
        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     name='rviz2',
        #     output='screen'
        # )
    ])
