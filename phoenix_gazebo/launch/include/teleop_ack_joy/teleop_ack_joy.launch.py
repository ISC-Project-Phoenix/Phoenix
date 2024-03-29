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
    throttle_axis = LaunchConfiguration('throttle_axis', default='5')
    steering_axis = LaunchConfiguration('steering_axis', default='0')
    min_axis_input = LaunchConfiguration('min_axis_input', default='-1.0')
    max_axis_input = LaunchConfiguration('max_axis_input', default='1.0')
    min_steering_angle = LaunchConfiguration('min_steering_angle', default='-0.2733')
    max_steering_angle = LaunchConfiguration('max_steering_angle', default='0.2733')
    max_speed = LaunchConfiguration('max_speed', default='4.0')

    ack_joy = Node(
        package='teleop_ack_joy',
        executable='teleop_ack_joy',
        name='teleop_ack_joy',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'throttle_axis': throttle_axis,
            'steering_axis': steering_axis,
            'min_axis_input': min_axis_input,
            'max_axis_input': max_axis_input,
            'min_steering_angle': min_steering_angle,
            'max_steering_angle': max_steering_angle,
            'max_speed': max_speed,
        }], )

    joy = Node(
        package='joy',
        executable='joy_node',
        name='joy',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
        }], )

    return LaunchDescription([
        # Launch Arguments
        DeclareLaunchArgument('use_sim_time',
                              default_value='true',
                              description='Use simulation clock if true'),
        DeclareLaunchArgument('throttle_axis',
                              default_value='5',
                              description=''),
        DeclareLaunchArgument('steering_axis',
                              default_value='0',
                              description=''),
        DeclareLaunchArgument('min_axis_input',
                              default_value='-1.0',
                              description=''),
        DeclareLaunchArgument('max_axis_input',
                              default_value='1.0',
                              description=''),
        DeclareLaunchArgument('min_steering_angle',
                              default_value='-0.272',
                              description=''),
        DeclareLaunchArgument('max_steering_angle',
                              default_value='0.272',
                              description=''),
        DeclareLaunchArgument('max_speed',
                              default_value='4.0',
                              description=''),
        # Nodes
        ack_joy,
        joy
    ])
