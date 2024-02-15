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
    min_look_ahead_distance = LaunchConfiguration('min_look_ahead_distance', default='3.85')
    max_look_ahead_distance = LaunchConfiguration('max_look_ahead_distance', default='10.0')
    k_dd = LaunchConfiguration('k_dd', default='1.5')
    max_speed = LaunchConfiguration('max_speed', default=6.7056)
    avoidance_radius = LaunchConfiguration('avoidance_radius', default=2.0)
    rear_axle_frame = LaunchConfiguration('rear_axle_frame', default='rear_axle')
    wheel_base = LaunchConfiguration('wheel_base', default='1.08')
    gravity_constant = LaunchConfiguration('gravity_constant', default='9.81')
    debug = LaunchConfiguration('debug', default='true')
    sonp = LaunchConfiguration('stop_on_no_path', default='false')

    pp = Node(
        package='hybrid_pp',
        executable='hybrid_pp',
        name='hybrid_pp',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'min_look_ahead_distance': min_look_ahead_distance,
            'max_look_ahead_distance': max_look_ahead_distance,
            'k_dd': k_dd,
            'max_speed': max_speed,
            'avoidance_radius': avoidance_radius,
            'rear_axle_frame': rear_axle_frame,
            'wheel_base': wheel_base,
            'gravity_constant': gravity_constant,
            'debug': debug,
            'stop_on_no_path': sonp
        }], )

    return LaunchDescription([
        # Launch Arguments
        DeclareLaunchArgument('use_sim_time',
                              default_value='true',
                              description='Use simulation clock if true'),
        DeclareLaunchArgument('min_look_ahead_distance',
                              default_value='3.85',
                              description='Minimum look ahead distance'),
        DeclareLaunchArgument('max_look_ahead_distance',
                              default_value='10.0',
                              description='Maximum look ahead distance'),
        DeclareLaunchArgument('k_dd',
                              default_value='1.3',
                              description='K_dd constant'),
        DeclareLaunchArgument('max_speed',
                              default_value='4.0',
                              description='Maximum speed'),
        DeclareLaunchArgument('avoidance_radius',
                              default_value='2.0',
                              description='Radius for avoiding obstacles'),
        DeclareLaunchArgument('rear_axle_frame',
                              default_value='rear_axle',
                              description='Rear axle frame'),
        DeclareLaunchArgument('wheel_base',
                              default_value='1.08',
                              description='Wheel base'),
        DeclareLaunchArgument('gravity_constant',
                              default_value='9.81',
                              description='Gravity constant'),
        DeclareLaunchArgument('debug',
                              default_value='true',
                              description='Debug mode'),
        DeclareLaunchArgument('stop_on_no_path',
                              default_value='false',
                              description='Stops the kart if no path is found'),
        # Nodes
        pp,
    ])
