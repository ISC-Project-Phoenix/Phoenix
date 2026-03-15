# MIT License
#
# Copyright (c) 2021 Intelligent Systems Club
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import os

from ament_index_python.packages import get_package_share_directory

from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import PathJoinSubstitution
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 1. Define Arguments
    # We change the default to include .yaml so we can just join paths easily
    location_arg = DeclareLaunchArgument(
        'location_file',
        default_value='dearborn.yaml', 
        description='Name of location file (e.g., dearborn.yaml)'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    # 2. Setup Configuration Variables
    pkg_phoenix_robot = get_package_share_directory('phoenix_robot')
    location_file = LaunchConfiguration('location_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # 3. Build Paths using Substitutions (No Python string manipulation needed)
    common_yaml_path = PathJoinSubstitution([
        pkg_phoenix_robot, 
        'config', 'navsat_transform', 'navsat_common.yaml'
    ])
    
    location_yaml_path = PathJoinSubstitution([
        pkg_phoenix_robot, 
        'config', 'navsat_transform', 'locations', location_file
    ])

    # 4. Define the Node
    navsat_node = Node(
        package='robot_localization', # Fixed package name
        executable='navsat_transform_node',
        name='navsat_transform_node',
        output='screen',
        parameters=[
            common_yaml_path,   # Defaults
            location_yaml_path, # Location Specifics
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('imu/data', '/vectornav/imu'), 
            ('gps/fix', '/gps/fix'), 
            ('odometry/filtered', '/odometry/global'),
            ('odometry/gps', '/odometry/gps'),
            ('gps/filtered', '/gps/filtered')
        ]
    )

    return LaunchDescription([
        location_arg,
        use_sim_time_arg,
        navsat_node
    ])