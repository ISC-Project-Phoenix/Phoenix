# Copyright (c) 2026 Intelligent Systems Club
# License: TBD. Contact for usage. 

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    port = LaunchConfiguration('port', default='/dev/ttyACM1')
    max_speed = LaunchConfiguration('max_speed', default='4.0')
    max_steering_angle = LaunchConfiguration('max_steering_angle', default='0.2733')

    ack_joy = Node(package='teleop_ack_rc',
               executable='teleop_ack_rc',
                name='teleop_ack_rc',
               parameters=[{
                    'use_sim_time': use_sim_time,
                    # TODO KEEP THESE HERE SINCE THE OTHER JOY USES THEM IDK IF WE WILL
                    # 'throttle_axis': throttle_axis,
                    # 'steering_axis': steering_axis,
                    # 'min_axis_input': min_axis_input,
                    # 'max_axis_input': max_axis_input,
                    # 'min_steering_angle': min_steering_angle,
                    'port': port,
                    'max_speed': max_speed,
                    'max_steering_angle': max_steering_angle,
               }],
               )
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
        DeclareLaunchArgument('port',
                              default_value='/dev/ttyACM1',
                              description='Serial port for RC receiver'),
        DeclareLaunchArgument('max_speed',
                              default_value='4.0',
                              description='Maximum speed in m/s'),
        DeclareLaunchArgument('max_steering_angle',
                              default_value='0.2733',
                              description='Maximum steering angle in radians'),
        ack_joy, 
        joy
    ])
