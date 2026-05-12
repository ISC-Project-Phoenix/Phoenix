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
from launch.substitutions import PythonExpression



def generate_launch_description():
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Hardcoded paths, so you should update these to use path join at some point
    sim_waypoints = '/home/redtoo/Documents/ws-gps-phoenix/src/gps_publisher/src/gps_waypoints_purdue_sim_mk1.txt'
    real_waypoints = '/home/isc/Documents/dev/phnx_ws_2026/data/gps_publisher/src/gps_waypoints_parking_lot_mk1.txt'

    # This expression picks the real path if use_sim_time is 'false', else sim path
    chosen_file = PythonExpression([
        "'", real_waypoints, "' if '", use_sim_time, "' == 'false' else '", sim_waypoints, "'"
    ])

    plan = Node(
        package='yet_another_gps_publisher',
        executable='yet_another_gps_publisher',
        name='yet_another_gps_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'waypoint_file_path': chosen_file
        }], )

    return LaunchDescription([
        # Launch Arguments
        DeclareLaunchArgument('use_sim_time',
                              default_value='true',
                              description='Use simulation clock if true'),
        # Nodes
        plan,

    ])
