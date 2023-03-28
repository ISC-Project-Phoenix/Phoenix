import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

def generate_launch_description():
    max_braking_speed = LaunchConfiguration('max_braking_speed', default='-10.0')
    max_throttle_speed = LaunchConfiguration('max_throttle_speed', default='10.0')
    max_steering_rad = LaunchConfiguration('max_steering_rad', default='2.0')
    wheelbase = LaunchConfiguration('wheelbase', default='1.08')

    pir = Node(package='phnx_io_ros',
               executable='phnx_io_ros',
               parameters=[{
                   'max_braking_speed': max_braking_speed,
                   'max_throttle_speed': max_throttle_speed,
                   'max_steering_rad': max_steering_rad,
                   'wheelbase': wheelbase
               }]
               )

    return LaunchDescription([
        # Launch Arguments
        DeclareLaunchArgument('max_braking_speed',
                              default_value='-10.0',
                              description='Maximum braking speed'),
        DeclareLaunchArgument('max_throttle_speed',
                              default_value='10.0',
                              description='Maximum throttle speed'),
        DeclareLaunchArgument('wheelbase',
                              default_value='1.08',
                              description='Maximum throttle speed'),
        DeclareLaunchArgument('max_steering_rad',
                              default_value='2.0',
                              description='Maximum wheel angle'),
        pir
    ])
