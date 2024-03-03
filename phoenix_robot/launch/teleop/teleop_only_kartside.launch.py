import os

from ament_index_python.packages import get_package_share_directory

from launch.conditions import IfCondition, UnlessCondition
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


# Launches just enough nodes on the kart to teleop
def generate_launch_description():
    # ROS packages
    pkg_phoenix_gazebo = get_package_share_directory('phoenix_gazebo')
    pkg_phoenix_robot = get_package_share_directory('phoenix_robot')
    pkg_robot_state_controller = get_package_share_directory(
        'robot_state_controller')

    encoder_only = LaunchConfiguration('encoder_only')

    # Launch arguments
    drive_mode_switch_button = LaunchConfiguration(
        'drive_mode_switch_button', default='7')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    state_publishers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_phoenix_gazebo, 'launch')
            , '/include/state_publishers/state_publishers.launch.py'
        ]),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    ekf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_phoenix_robot, 'launch')
            , '/include/robot_localization/robot_localization.launch.py'
        ]),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
        condition=UnlessCondition(encoder_only)
    )

    ekf_encoder_only = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_phoenix_robot, 'launch')
            , '/include/robot_localization/robot_localization.launch.py'
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'config_file_name': 'robot_localization_encoder_only.yaml'
        }.items(),
        condition=IfCondition(encoder_only)
    )

    robot_state_controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_robot_state_controller, 'launch'),
            '/robot_state_controller.launch.py'
        ]),
        launch_arguments={
            'switch_button': drive_mode_switch_button,
            'init_value': 'teleop',
            'use_sim_time': use_sim_time
        }.items(),
    )

    # Connect to two cameras
    cameras = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_phoenix_robot, 'launch'),
            '/include/oakd/dual_cameras.launch.py'
        ]),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
        condition=UnlessCondition(encoder_only)
    )

    pir = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_phoenix_robot, 'launch'),
            '/include/phnx_io_ros/phnx_io_ros.launch.py'
        ]),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    return LaunchDescription([
        # Launch Arguments
        DeclareLaunchArgument(
            'drive_mode_switch_button',
            default_value='10',
            description='Which button is used on the joystick to switch drive mode. (In joy message)'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(
            'encoder_only',
            default_value='false',
            description='Only use the encoder for odom, so no cameras are required.'),

        # Nodes
        robot_state_controller,
        state_publishers,
        ekf,
        ekf_encoder_only,
        cameras,
        pir,
    ])
