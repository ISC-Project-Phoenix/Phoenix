import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # This just tells the script where your new folder is
    config = os.path.join(
        get_package_share_directory('phoenix_robot'),
        'config', 'phoenix_io_ros', 'phoenix_io_ros.yaml'
    )

    return LaunchDescription([
        Node(
            package='phnx_io_ros',
            executable='phnx_io_ros',
            name='phnx_io_ros',
            parameters=[config],
            output='screen'
        )
    ])