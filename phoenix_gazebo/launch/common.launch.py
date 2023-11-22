import os

from ament_index_python.packages import get_package_share_directory

from launch.conditions import IfCondition, UnlessCondition
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


# Launches sim as well as rviz and joy stuff
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
    use_webots = LaunchConfiguration('use_webots', default='true')
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    gazebo_world = LaunchConfiguration(
        'gazebo_world', default='purdue_gp_track.sdf')

    use_wheel = LaunchConfiguration('use_wheel', default='false')

    # TODO make these correct
    max_braking_speed = LaunchConfiguration('max_braking_speed', default='-10.0')
    max_throttle_speed = LaunchConfiguration('max_throttle_speed', default='10.0')
    max_steering_rad = LaunchConfiguration('max_steering_rad', default='2.0')
    wheelbase = LaunchConfiguration('wheelbase', default='1.08')

    # If wheel is not used, we need to translate joystick commands to ackermann
    joy_with_teleop_twist = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_teleop_twist_joy, 'launch', 'teleop-launch.py')),
        launch_arguments={
            'joy_dev': '/dev/input/js0',
            'config_filepath': joy_config
        }.items(),
        condition=UnlessCondition(use_wheel)
    )

    twist_to_ackermann = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_phoenix_gazebo, 'launch', 'include', 'twist_to_ackermann/twist_to_ackermann.launch.py')),
        launch_arguments={
            'wheelbase': wheelbase
        }.items(),
        condition=UnlessCondition(use_wheel)
    )

    logi_g29 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_phoenix_gazebo, 'launch', 'include', 'logi_g29/logi_g29.launch.py')),
        launch_arguments={
            'max_braking_speed': max_braking_speed,
            'max_throttle_speed': max_throttle_speed,
            'max_steering_rad': max_steering_rad,
            'wheelbase': wheelbase
        }.items(),
        condition=IfCondition(use_wheel)
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

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_phoenix_gazebo, 'launch'),
            '/sim.launch.py'
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'max_braking_speed': max_braking_speed,
            'max_throttle_speed': max_throttle_speed,
            'max_steering_rad': max_steering_rad,
            'wheelbase': wheelbase,
            'gazebo_world': gazebo_world,
            'use_webots': use_webots
        }.items(),
    )

    # Launch here since this is only used for Joy stuff
    robot_state_controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_robot_state_controller, 'launch'),
            '/robot_state_controller.launch.py'
        ]),
        launch_arguments={
            'switch_button': drive_mode_switch_button,
            'use_sim_time': use_sim_time,
            'init_value': "auton"
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

        # Set true if using logitech wheel
        DeclareLaunchArgument('use_wheel',
                              default_value='false',
                              description='Use logitech wheel'),
        # Changes drive mode switch default state
        DeclareLaunchArgument('drive_mode_start_state',
                              default_value='auton',
                              description='Changes drive mode switch default state'),
        DeclareLaunchArgument('use_webots',
                              default_value='true',
                              description='Opens webots if true, else opens gazebo'),

        # Nodes
        sim,
        joy_with_teleop_twist,
        twist_to_ackermann,
        logi_g29,
        rviz,
        robot_state_controller
    ])
