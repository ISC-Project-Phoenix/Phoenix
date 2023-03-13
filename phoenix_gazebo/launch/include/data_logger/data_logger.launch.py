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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    max_braking_speed = LaunchConfiguration('max_braking_speed', default='-10.0')
    max_throttle_speed = LaunchConfiguration('max_throttle_speed', default='10.0')
    max_steering_rad = LaunchConfiguration('max_steering_rad', default='0.34')

    data_path = LaunchConfiguration('data_path', default='./training_data')

    data = Node(
        package='data_logger',
        executable='data_logger',
        name='data_logger',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'max_braking_speed': max_braking_speed,
            'max_throttle_speed': max_throttle_speed,
            'max_steering_rad': max_steering_rad,
            'data_path': data_path
        }])

    return LaunchDescription([
        # Launch Arguments
        DeclareLaunchArgument('use_sim_time',
                              default_value='true',
                              description='Use simulation clock if true'),
        DeclareLaunchArgument('max_braking_speed',
                              default_value='-10.0',
                              description='Maximum braking speed'),
        DeclareLaunchArgument('max_throttle_speed',
                              default_value='10.0',
                              description='Maximum throttle speed'),
        DeclareLaunchArgument('max_steering_rad',
                              default_value='0.34',
                              description='Maximum wheel angle'),
        DeclareLaunchArgument('data_path',
                              default_value='./training_data',
                              description='Path the write data to'),

        # Nodes
        data,
    ])
