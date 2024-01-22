import os

from ament_index_python.packages import get_package_share_directory

from launch.conditions import IfCondition, UnlessCondition
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node


# Launches two object detectors and a concat node to combine them. Effectively one big camera
def generate_launch_description():
    # Launch arguments
    pkg_phoenix_gazebo = get_package_share_directory('phoenix_gazebo')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    debug = LaunchConfiguration('debug', default='false')
    trans = LaunchConfiguration('transport')
    rgb_post = LaunchConfiguration('rgb_raw_postfix')
    depth_post = LaunchConfiguration('depth_raw_postfix')

    left = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_phoenix_gazebo, 'launch'),
            '/include/obj_detector/obj_detector.launch.py'
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'camera_topic_rgb': ["camera/left", rgb_post],
            'camera_topic_depth': ["camera/left", depth_post],
            'camera_info_topic': TextSubstitution(text="camera/left/rgb/camera_info"),
            'camera_frame': TextSubstitution(text="left_cam_link"),
            'detection_topic': TextSubstitution(text="/object_poses/left"),
            'name': TextSubstitution(text="left_obj_detector"),
            'debug': debug,
            'transport': trans
        }.items(),
    )

    right = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_phoenix_gazebo, 'launch'),
            '/include/obj_detector/obj_detector.launch.py'
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'camera_topic_rgb': ["camera/right", rgb_post],
            'camera_topic_depth': ["camera/right", depth_post],
            'camera_info_topic': TextSubstitution(text="camera/right/rgb/camera_info"),
            'camera_frame': TextSubstitution(text="right_cam_link"),
            'detection_topic': TextSubstitution(text="/object_poses/right"),
            'name': TextSubstitution(text="right_obj_detector"),
            'debug': debug,
            'transport': trans
        }.items(),
    )

    combine = Node(
        package='obj_detector',
        executable='detection_cat',
        name='detection_cat',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time
        }],
        remappings=[
            ('/object_detection1', '/object_poses/left'),
            ('/object_detection2', '/object_poses/right'),
            ('/object_detection_cat', '/object_poses'),
        ]
    )

    return LaunchDescription([
        # Launch Arguments
        DeclareLaunchArgument('use_sim_time',
                              default_value='true',
                              description='Use simulation clock if true'),
        DeclareLaunchArgument('debug',
                              default_value='false',
                              description='Displays debug'),
        DeclareLaunchArgument('transport',
                              default_value='raw',
                              description='transport type'),
        DeclareLaunchArgument('rgb_raw_postfix',
                              default_value='/rgb',
                              description='postfix of rgb base'),
        DeclareLaunchArgument('depth_raw_postfix',
                              default_value='/depth',
                              description='postfix of depth base'),

        # Nodes
        left,
        right,
        combine
    ])
