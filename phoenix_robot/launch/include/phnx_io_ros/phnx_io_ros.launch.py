import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

def generate_launch_description():

    pir = Node(package='phnx_io_ros',
               executable='phnx_io_ros',
               parameters=[{
               }]
               )

    return LaunchDescription([
        # Launch Arguments
        pir
    ])
