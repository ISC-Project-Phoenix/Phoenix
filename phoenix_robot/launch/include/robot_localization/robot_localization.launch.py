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
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

def generate_launch_description():
    pkg_share = get_package_share_directory('phoenix_robot')
    
    # /home/redtoo/Documents/ws-phnx-gps/src/Phoenix/phoenix_robot/config/robot_localization/robot_localization_dual_ekf.yaml
    rl_config = PathJoinSubstitution([pkg_share, 'config', 'robot_localization', 'robot_localization_dual_ekf.yaml'])
    navsat_config = PathJoinSubstitution([pkg_share, 'config', 'navsat_transform', 'navsat_transform','navsat_transform.yaml'])

    # LOCAL EKF (odom -> base_link)
    # No gps instead a smooth transtion. 
    local_ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_odom',
        output='screen',
        parameters=[rl_config, {'use_sim_time': LaunchConfiguration('use_sim_time')}],
        remappings=[('/odometry/filtered', '/odom')]
    )

    # GLOBAL EKF (map -> odom)
    # This node provides the 'map' frame and is fed by the NavSat Transform Node
    global_ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_map',
        output='screen',
        parameters=[rl_config, {'use_sim_time': LaunchConfiguration('use_sim_time')}],
        remappings=[('/odometry/filtered', '/odometry/global')]
    )

    # NAVSAT TRANSFORM NODE
    # Converts Lat/Lon to X/Y and bridges the GPS into the Global EKF
    navsat_transform = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform',
        output='screen',
        parameters=[navsat_config, {'use_sim_time': LaunchConfiguration('use_sim_time')}],
        remappings=[
            ('imu', '/phoenix/imu'),           # Data from VectorNav
            ('gps/fix', '/phoenix/navsat'),    # Data from VectorNav
            ('odometry/filtered', '/odom'),    # Input from LOCAL EKF
            ('odometry/gps', '/odometry/gps')  # Output to GLOBAL EKF
        ]
    )
    return LaunchDescription([
        # Launch Arguments
        DeclareLaunchArgument('use_sim_time',
                              default_value='true',
                              description='Use simulation clock if true'),
        # Nodes
        local_ekf,
        global_ekf,
        navsat_transform,
    ])
