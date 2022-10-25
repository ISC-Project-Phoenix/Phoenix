import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    # ROS packages
    pkg_tinykart_description = get_package_share_directory(
        'tinykart_description')

    # launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    use_rviz = LaunchConfiguration('use_rviz', default='true')

    # Nodes
    rviz = Node(package='rviz2',
                executable='rviz2',
                arguments=[
                    '-d',
                    os.path.join(pkg_tinykart_description, 'rviz',
                                 'tinykart_gazebo.rviz')
                ],
                parameters=[{
                    'use_sim_time': use_sim_time
                }],
                condition=IfCondition(use_rviz))

    return LaunchDescription([
        # Launch Arguments
        DeclareLaunchArgument('use_rviz',
                              default_value='false',
                              description='Use simulation time if true'),

        # Node
        rviz,
    ])
