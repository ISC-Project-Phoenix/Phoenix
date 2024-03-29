import os

from ament_index_python.packages import get_package_share_directory

from launch.conditions import IfCondition, UnlessCondition
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    debug = LaunchConfiguration('debug', default='true')

    track = Node(
        package='obj_tracker',
        executable='obj_tracker',
        name='obj_tracker',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'measure_cov': 0.001,
            'prediction_cov': 4.0,
            'max_frames_missed': 2,
            'debug': debug
        }]
    )

    return LaunchDescription([
        # Launch Arguments
        DeclareLaunchArgument('use_sim_time',
                              default_value='true',
                              description='Use simulation clock if true'),
        DeclareLaunchArgument('debug',
                              default_value = 'true',
                              description = 'Flag for whether or not we publish our rviz visualizations'),
        
        
        # Nodes
        track,
    ])
