import os

from ament_index_python.packages import get_package_share_directory

from launch.conditions import IfCondition, UnlessCondition
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    # ROS packages
    pkg_phoenix_gazebo = get_package_share_directory('phoenix_gazebo')
    pkg_phoenix_robot = get_package_share_directory('phoenix_robot')
    pkg_robot_state_controller = get_package_share_directory(
        'robot_state_controller')
    pkg_isc_sick = get_package_share_directory('ros2_sick')

    # Launch arguments
    drive_mode_switch_button = LaunchConfiguration(
        'drive_mode_switch_button', default='7')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    use_ai = LaunchConfiguration('use_ai', default='false')

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
    )


    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_phoenix_robot, 'launch'),
            '/include/oakd/oakd.launch.py'
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
            'use_ai',
            default_value='false',
            description='Use ai stack if true'),

        # Nodes
        # robot_state_controller,
        # state_publishers,
        # ekf,
        camera,
        # pir,
        # pp,
        # obj_detector_ai,
        # obj_detector_cv,
        # poly_plan,
        # poly_plan_ai
    ])