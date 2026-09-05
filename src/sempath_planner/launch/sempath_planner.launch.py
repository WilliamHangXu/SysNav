import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('sempath_planner')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('config', default_value=os.path.join(share, 'config', 'sempath_planner.yaml'),
                              description='sempath_planner parameter file'),
        DeclareLaunchArgument('run_group', default_value='sim',
                              description='platform subfolder for exported maps: '
                                          'real/<run_group>/<timestamp>_train (sim | robot)'),
        DeclareLaunchArgument('map_viz_z', default_value='0.0',
                              description='height (m, map frame) of the RViz semantic map overlay; '
                                          'sim floor is at 0, the real robot floor sits below the '
                                          'SLAM origin (the robot launch passes -0.35)'),
        Node(package='sempath_planner', executable='sempath_planner_node', name='sempath_planner',
             output='screen',
             parameters=[LaunchConfiguration('config'),
                         {'use_sim_time': LaunchConfiguration('use_sim_time'),
                          'run_group': LaunchConfiguration('run_group'),
                          'map_viz.z': ParameterValue(LaunchConfiguration('map_viz_z'),
                                                      value_type=float)}]),
    ])
