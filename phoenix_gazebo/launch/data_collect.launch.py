import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # ROS packages
    pkg_phoenix_gazebo = get_package_share_directory('phoenix_gazebo')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # TODO make these correct
    max_braking_speed = LaunchConfiguration('max_braking_speed', default='-10.0')
    max_throttle_speed = LaunchConfiguration('max_throttle_speed', default='10.0')
    max_steering_rad = LaunchConfiguration('max_steering_rad', default='0.34')
    wheelbase = LaunchConfiguration('wheelbase', default='1.8')

    # path we write run data to
    data_path = LaunchConfiguration('data_path', default='./training_data')

    common = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_phoenix_gazebo, 'launch'),
            '/common.launch.py'
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'max_braking_speed': max_braking_speed,
            'max_throttle_speed': max_throttle_speed,
            'max_steering_rad': max_steering_rad,
            'wheelbase': wheelbase
        }.items()
    )

    data_logger = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_phoenix_gazebo, 'launch'),
            '/include/data_logger/data_logger.launch.py'
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'max_braking_speed': max_braking_speed,
            'max_throttle_speed': max_throttle_speed,
            'max_steering_rad': max_steering_rad,
            'data_path': data_path
        }.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),

        # Nodes
        common,
        data_logger
    ])