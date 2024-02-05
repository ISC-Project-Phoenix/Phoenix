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

    webots = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_phoenix_gazebo, 'launch'),
            '/include/webots/webots.launch.py'
        ]),
    )

    # Autonomy pipeline

    obj_detector_wb = GroupAction(actions=[
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(pkg_phoenix_gazebo, 'launch'),
                '/include/obj_detector/dual_camera.launch.py'
            ]),
            launch_arguments={
                'use_sim_time': use_sim_time,
                # Webots has odd outputs
                'rgb_raw_postfix': "/rgb/image_color",
                'depth_raw_postfix': "/depth/image",
            }.items(),
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

    pp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_phoenix_gazebo, 'launch'),
            '/include/hybrid_pp/hybrid_pp.launch.py'
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items(),
    )

    planner = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_phoenix_gazebo, 'launch'),
            '/include/obj_planner/obj_planner.launch.py'
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

        # Nodes
        state_publishers,
        ekf,
        webots,
        obj_detector_wb,
        obj_tracker,
        pp,
        planner
    ])
