import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_robot_model(pkg_description):
    urdf_dir = os.path.join(pkg_description, 'urdf')
    urdf_file = os.path.join(urdf_dir, 'phoenix.urdf')
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()
    return robot_desc, urdf_file


def generate_launch_description():
    # ROS packages
    pkg_tinykart_description = get_package_share_directory(
        'phoenix_description')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    robot_desc, urdf_file = generate_robot_model(pkg_tinykart_description)

    # Nodes
    robot_state_publisher = Node(package='robot_state_publisher',
                                 executable='robot_state_publisher',
                                 name='robot_state_publisher',
                                 output='screen',
                                 parameters=[{
                                     'use_sim_time': use_sim_time,
                                     'robot_description': robot_desc,
                                 }])

    joint_state_publisher = Node(package='joint_state_publisher',
                                 executable='joint_state_publisher',
                                 name='joint_state_publisher',
                                 output='screen',
                                 )
    return LaunchDescription([
        # Launch Arguments
        DeclareLaunchArgument('use_sim_time',
                              default_value='true',
                              description='Use simulation clock if true'),

        # Nodes
        robot_state_publisher,
        joint_state_publisher,
    ])
