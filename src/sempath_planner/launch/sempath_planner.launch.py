import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('sempath_planner')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('config', default_value=os.path.join(share, 'config', 'sempath_planner.yaml'),
                              description='sempath_planner parameter file'),
        Node(package='sempath_planner', executable='sempath_planner_node', name='sempath_planner',
             output='screen',
             parameters=[LaunchConfiguration('config'),
                         {'use_sim_time': LaunchConfiguration('use_sim_time')}]),
    ])
