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

    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_phoenix_robot, 'launch'),
            '/include/oakd/oakd.launch.py'
        ]),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    sick = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_isc_sick, 'launch')
            , '/sick.launch.py'
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'config_file': PathJoinSubstitution([pkg_phoenix_robot, 'config', 'ros2_sick', 'sick_lms111.yaml'])
        }.items(),
    )

    pir = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_phoenix_robot, 'launch'),
            '/include/phnx_io_ros/phnx_io_ros.launch.py'
        ]),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    pp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_phoenix_gazebo, 'launch'),
            '/include/hybrid_pp/hybrid_pp.launch.py'
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'max_speed': '8.0',
            'min_speed': '0.5'
        }.items(),
    )

    obj_detector_ai = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_phoenix_gazebo, 'launch'),
            '/include/road_detectors/obj_detector_ai.launch.py'
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items(),
        condition=IfCondition(use_ai)
    )

    obj_detector_cv = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_phoenix_gazebo, 'launch'),
            '/include/road_detectors/obj_detector_cv.launch.py'
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items(),
        condition=UnlessCondition(use_ai)
    )

    poly_plan_ai = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_phoenix_gazebo, 'launch'),
            '/include/polynomial_planner/polynomial_planner_ai.launch.py'
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items(),
        condition=IfCondition(use_ai)
    )

    poly_plan = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_phoenix_gazebo, 'launch'),
            '/include/polynomial_planner/polynomial_planner_ai.launch.py'
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items(),
        condition=UnlessCondition(use_ai)
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
        robot_state_controller,
        state_publishers,
        ekf,
        camera,
        sick,
        pir,
        pp,
        obj_detector_ai,
        obj_detector_cv,
        poly_plan,
        poly_plan_ai
    ])