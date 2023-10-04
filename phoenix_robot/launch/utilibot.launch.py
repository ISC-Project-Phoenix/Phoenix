import os

from ament_index_python.packages import get_package_share_directory

from launch.conditions import IfCondition, UnlessCondition
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


# stuff that needs to be launched on the remote utilibot. This contains all joystick handling nodes
def generate_launch_description():
    # ROS packages
    pkg_phoenix_gazebo = get_package_share_directory('phoenix_gazebo')
    pkg_phoenix_robot = get_package_share_directory('phoenix_robot')
    pkg_teleop_twist_joy = get_package_share_directory('teleop_twist_joy')

    # Config
    joy_config = os.path.join(pkg_phoenix_gazebo, 'config/joystick',
                              'xbone.config.yaml')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    use_rviz = LaunchConfiguration('use_rviz', default='true')

    use_wheel = LaunchConfiguration('use_wheel', default='true')

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
            'use_sim_time': use_sim_time,
            'wheelbase': wheelbase
        }.items(),
        condition=UnlessCondition(use_wheel)
    )

    logi_g29 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_phoenix_gazebo, 'launch', 'include', 'logi_g29/logi_g29.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
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

    return LaunchDescription([
        # Launch Arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument('use_rviz',
                              default_value='true',
                              description='Open rviz if true'),
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
                              default_value='true',
                              description='Use logitech wheel'),

        # Nodes
        joy_with_teleop_twist,
        twist_to_ackermann,
        logi_g29,
        rviz,
    ])