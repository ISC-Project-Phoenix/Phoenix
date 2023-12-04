import os

from ament_index_python.packages import get_package_share_directory

from launch.conditions import IfCondition, UnlessCondition
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # ROS packages
    pkg_phoenix_gazebo = get_package_share_directory('phoenix_gazebo')
    pkg_phoenix_robot = get_package_share_directory('phoenix_robot')
    pkg_robot_state_controller = get_package_share_directory(
        'robot_state_controller')

    # Launch arguments
    drive_mode_switch_button = LaunchConfiguration(
        'drive_mode_switch_button', default='7')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # TODO make these correct
    max_braking_speed = LaunchConfiguration('max_braking_speed', default='-10.0')
    max_throttle_speed = LaunchConfiguration('max_throttle_speed', default='10.0')
    max_steering_rad = LaunchConfiguration('max_steering_rad', default='2.0')
    wheelbase = LaunchConfiguration('wheelbase', default='1.08')

    state_publishers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_phoenix_gazebo, 'launch')
            , '/include/state_publishers/state_publishers.launch.py'
        ]),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    robot_state_controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_robot_state_controller, 'launch'),
            '/robot_state_controller.launch.py'
        ]),
        launch_arguments={
            'switch_button': drive_mode_switch_button,
            'use_sim_time': use_sim_time
        }.items(),
    )

    oakd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_phoenix_robot, 'launch'),
            '/include/oakd/oakd.launch.py'
        ])
    )

    pir = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_phoenix_robot, 'launch'),
            '/include/phnx_io_ros/phnx_io_ros.launch.py'
        ]),
        launch_arguments={
            'max_braking_speed': max_braking_speed,
            'max_throttle_speed': max_throttle_speed,
            'max_steering_rad': max_steering_rad,
            'wheelbase': wheelbase
        }.items()
    )

    obj_detect = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_phoenix_gazebo, 'launch'),
            'include/obj_detector/obj_detector.launch.py'
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items(),
    )

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
            'drive_mode_switch_button',
            default_value='10',
            description='Which button is used on the joystick to switch drive mode. (In joy message)'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
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

        # Nodes
        robot_state_controller,
        state_publishers,
        oakd,
        pir,
        obj_detect,
        obj_tracker
    ])
