import os

from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='phnx_io_ros',
            executable='phnx_io_ros',
            parameters=[PathJoinSubstitution([
                FindPackageShare('phoenix_robot'), 'config', 'phnx_io_ros', 'phnx_io_ros.yaml'])
            ],
        ),     
    ])
