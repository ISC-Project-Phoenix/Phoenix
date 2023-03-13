import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # ROS packages
    pkg_phoenix_gazebo = get_package_share_directory('phoenix_gazebo')
    pkg_teleop_twist_joy = get_package_share_directory('teleop_twist_joy')
    pkg_robot_state_controller = get_package_share_directory(
        'robot_state_controller')

    # Config
    joy_config = os.path.join(pkg_phoenix_gazebo, 'config/joystick',
                              'xbone.config.yaml')

    # Launch arguments
    drive_mode_switch_button = LaunchConfiguration(
        'drive_mode_switch_button', default='7')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    gazebo_world = LaunchConfiguration(
        'gazebo_world', default='purdue_gp_track.sdf')

    max_braking_speed = LaunchConfiguration('max_braking_speed', default='-10.0')
    max_throttle_speed = LaunchConfiguration('max_throttle_speed', default='10.0')
    max_steering_rad = LaunchConfiguration('max_steering_rad', default='0.34')
    wheelbase = LaunchConfiguration('wheelbase', default='1.8')

    state_publishers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_phoenix_gazebo, 'launch'),
            '/include/state_publishers/state_publishers.launch.py'
        ]),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    ign_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_phoenix_gazebo, 'launch'),
            '/include/gazebo/gazebo.launch.py'
        ]),
        launch_arguments={
            'gazebo_world': gazebo_world
        }.items()
    )

    joy_with_teleop_twist = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_teleop_twist_joy, 'launch', 'teleop-launch.py')),
        launch_arguments={
            'joy_dev': '/dev/input/js0',
            'config_filepath': joy_config
        }.items(),
    )

    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_phoenix_gazebo, 'launch'),
            '/include/rviz/rviz.launch.py'
        ]),
        launch_arguments={
            'use_rviz': use_rviz,
            'use_sim_time': use_sim_time
        }.items(),
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
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument('use_rviz',
                              default_value='true',
                              description='Open rviz if true'),
        DeclareLaunchArgument('follow_waypoints',
                              default_value='true',
                              description='follow way points if true'),
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
                              default_value='1.8',
                              description='Maximum throttle speed'),
        DeclareLaunchArgument('max_steering_rad',
                              default_value='0.34',
                              description='Maximum wheel angle'),

        # Nodes
        state_publishers,
        ign_gazebo,
        joy_with_teleop_twist,
        robot_state_controller,
        rviz,
        gz_io_ros
    ])
