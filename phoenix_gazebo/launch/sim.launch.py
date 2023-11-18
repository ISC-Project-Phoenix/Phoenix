import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap
from launch.conditions import IfCondition, UnlessCondition


# Launches a sim and core phnx nodes. This file does not assume any user display or input
def generate_launch_description():
    # ROS packages
    pkg_phoenix_gazebo = get_package_share_directory('phoenix_gazebo')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    use_webots = LaunchConfiguration('use_webots', default='true')
    gazebo_world = LaunchConfiguration(
        'gazebo_world', default='purdue_gp_track.sdf')

    max_braking_speed = LaunchConfiguration('max_braking_speed', default='-10.0')
    max_throttle_speed = LaunchConfiguration('max_throttle_speed', default='10.0')
    max_steering_rad = LaunchConfiguration('max_steering_rad', default='2.0')
    wheelbase = LaunchConfiguration('wheelbase', default='1.08')

    # Misc utility nodes
    state_publishers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_phoenix_gazebo, 'launch'),
            '/include/state_publishers/state_publishers.launch.py'
        ]),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    ekf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_phoenix_gazebo, 'launch'),
            '/include/robot_localization/robot_localization.launch.py'
        ]),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    # Nodes that run if using gazebo
    ign_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_phoenix_gazebo, 'launch'),
            '/include/gazebo/gazebo.launch.py'
        ]),
        launch_arguments={
            'gazebo_world': gazebo_world
        }.items(),
        condition=UnlessCondition(use_webots)
    )

    gz_io_ros = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_phoenix_gazebo, 'launch'),
            '/include/gz_io_ros/gz_io_ros.launch.py'
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'max_braking_speed': max_braking_speed,
            'max_throttle_speed': max_throttle_speed,
            'wheelbase': wheelbase
        }.items(),
        condition=UnlessCondition(use_webots)
    )

    # Nodes that run if using webots
    webots = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_phoenix_gazebo, 'launch'),
            '/include/webots/webots.launch.py'
        ]),
        condition=IfCondition(use_webots)
    )

    # Autonomy pipeline

    # We need two versions of obj_detector to remap correctly
    obj_detector_gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_phoenix_gazebo, 'launch'),
            '/include/obj_detector/obj_detector.launch.py'
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items(),
        condition=UnlessCondition(use_webots)
    )

    obj_detector_wb = GroupAction(actions=[
        SetRemap('camera/mid/rgb', 'camera/mid/rgb/image_color'),
        SetRemap('camera/mid/depth', 'camera/mid/depth/image'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(pkg_phoenix_gazebo, 'launch'),
                '/include/obj_detector/obj_detector.launch.py'
            ]),
            launch_arguments={
                'use_sim_time': use_sim_time,
            }.items(),
            condition=IfCondition(use_webots)
        )])

    obj_tracker = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_phoenix_gazebo, 'launch'),
            '/include/obj_tracker/obj_tracker.launch.py'
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items(),
    )

    return LaunchDescription([
        # Launch Arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument('gazebo_world',
                              default_value='purdue_gp_track.sdf',
                              description='gazebo world to load'),
        DeclareLaunchArgument('max_braking_speed',
                              default_value='-10.0',
                              description='Maximum braking speed'),
        DeclareLaunchArgument('max_throttle_speed',
                              default_value='10.0',
                              description='Maximum throttle speed'),
        DeclareLaunchArgument('wheelbase',
                              default_value='1.08',
                              description='Maximum throttle speed'),
        DeclareLaunchArgument('max_steering_rad',
                              default_value='2.0',
                              description='Maximum wheel angle'),
        DeclareLaunchArgument('use_webots',
                              default_value='true',
                              description='Opens webots if true, else opens gazebo'),

        # Nodes
        state_publishers,
        ekf,
        ign_gazebo,
        gz_io_ros,
        webots,
        obj_detector_gz,
        obj_detector_wb,
        obj_tracker
    ])
