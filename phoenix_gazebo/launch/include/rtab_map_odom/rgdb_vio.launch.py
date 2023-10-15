import os

from ament_index_python.packages import get_package_share_directory

from launch.conditions import IfCondition, UnlessCondition
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# Experimental vio. Note that this is not integrated into the ekf yet, and needs more detail than sim can give.
def generate_launch_description():
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    vio = Node(
        package='rtabmap_odom',
        executable='rgbd_odometry',
        name='vio',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'publish_tf': False
        }],
        remappings=[
            ('/rgb/image', '/camera/mid/rgb'),
            ('/depth/image', '/camera/mid/depth'),
            ('/rgb/camera_info', '/camera/mid/rgb/camera_info'),
            ('/odom', '/odom_vio'),
        ],
    )

    return LaunchDescription([
        # Launch Arguments
        DeclareLaunchArgument('use_sim_time',
                              default_value='true',
                              description='Use simulation clock if true'),
        # Nodes
        vio,
    ])
