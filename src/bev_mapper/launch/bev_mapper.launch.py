import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('bev_mapper')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('config', default_value=os.path.join(share, 'config', 'bev_mapper.yaml'),
                              description='bev_mapper parameter file'),
        Node(package='bev_mapper', executable='bev_mapper_node', name='bev_mapper',
             output='screen',
             parameters=[LaunchConfiguration('config'),
                         {'use_sim_time': LaunchConfiguration('use_sim_time')}]),
    ])
