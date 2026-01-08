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
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    config_file_name = LaunchConfiguration('config_file_name')
    configured_params = RewrittenYaml(source_file=vn300_conf,
                                      root_key='',
                                      param_rewrites=param_substitutions,
                                      convert_types=True)


    # Vectornav
    start_vectornav_cmd = Node(
        package='vectornav',
        executable='vectornav',
        name='vectornav',
        output='screen',
        parameters=[PathJoinSubstitution(
            [get_package_share_directory('phoenix_robot'), 'config', 'vectornav', config_file_name])
        ],
        remappings=[
            ('/odometry/filtered', '/odom'),
        ],
    )

    # Node that converts raw vectornav data to ros msgs
    start_vectornav_sensor_msgs_cmd = Node(
        package='vectornav',
        executable='vn_sensor_msgs',
        output='screen',
        remappings=[('/vectornav/imu', '/kohm/gps/imu'), ('vectornav/gnss',
                                                          '/kohm/navsat'), ('/vectornav/magnetic', '/kohm/mag')],
        parameters=[PathJoinSubstitution(
            [get_package_share_directory('phoenix_robot'), 'config', 'vn_sensor_msgs', config_file_name])],
    )

    return LaunchDescription([
        # Launch Arguments
        DeclareLaunchArgument('use_sim_time',
                              default_value='false',
                              description='Use simulation clock if true'),
        DeclareLaunchArgument('config_file_name',
                              default_value='robot_localization.yaml',
                              description='Name of the config file to load'),
        # Nodes
        start_vectornav_cmd,
        start_vectornav_sensor_msgs_cmd,
    ])
