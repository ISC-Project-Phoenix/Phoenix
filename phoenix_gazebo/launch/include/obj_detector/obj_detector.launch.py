import os

from ament_index_python.packages import get_package_share_directory

from launch.conditions import IfCondition, UnlessCondition
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    rgb_topic = LaunchConfiguration('camera_topic_rgb')
    depth_topic = LaunchConfiguration('camera_topic_depth')
    camera_frame = LaunchConfiguration('camera_frame')
    detection_topic = LaunchConfiguration('detection_topic')
    camera_info_topic = LaunchConfiguration('camera_info_topic')
    name = LaunchConfiguration('name')
    debug = LaunchConfiguration('debug')
    trans = LaunchConfiguration('transport')
    hsv_lowerb = LaunchConfiguration('hsv_lowerb')
    hsv_upperb = LaunchConfiguration('hsv_upperb')
    area_threshold = LaunchConfiguration('area_threshold')

    remappings = [('/camera/mid/rgb/camera_info', camera_info_topic),
                  ('/camera/mid/depth', depth_topic),
                  ('/object_poses', detection_topic),
                  ('/camera/mid/rgb', rgb_topic),
                  ('/camera/mid/rgb/compressed', [rgb_topic, '/compressed']),
                  ]

    det = Node(
        package='obj_detector',
        executable='obj_detector',
        name=name,
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'camera_frame': camera_frame,
            'debug': debug,
            'transport_type': trans,
            'hsv_upperb': hsv_upperb,
            'hsv_lowerb': hsv_lowerb,
            'area_threshold' : area_threshold,
        }],
        remappings=remappings
    )

    return LaunchDescription([
        # Launch Arguments
        DeclareLaunchArgument('use_sim_time',
                              default_value='true',
                              description='Use simulation clock if true'),
        DeclareLaunchArgument('camera_topic_rgb',
                              default_value='/camera/mid/rgb',
                              description='rgb topic'),
        DeclareLaunchArgument('camera_topic_depth',
                              default_value='/camera/mid/depth',
                              description='depth topic'),
        DeclareLaunchArgument('camera_frame',
                              default_value='mid_cam_link',
                              description='camera frame'),
        DeclareLaunchArgument('detection_topic',
                              default_value='/object_poses',
                              description='camera frame'),
        DeclareLaunchArgument('camera_info_topic',
                              default_value='/camera/mid/rgb/camera_info',
                              description='rgb camera info topic'),
        DeclareLaunchArgument('name',
                              default_value='obj_detector',
                              description='node name'),
        DeclareLaunchArgument('debug',
                              default_value='true',
                              description='Show debug windows'),
        DeclareLaunchArgument('transport',
                              default_value='raw',
                              description='Transport type'),
        DeclareLaunchArgument('hsv_lowerb',
                              default_value="[0, 120, 120]",
                              description='Cone color filter lower bound in HSV'),
        DeclareLaunchArgument('hsv_upperb',
                              default_value="[30, 255, 255]",
                              description='Cone color filter upper bound in HSV'),
        DeclareLaunchArgument('gamma',
                              default_value="0.4",
                              description='Gamma value to filter images with'),
        DeclareLaunchArgument('area_threshold',
                              default_value='200.0',
                              description='Pixel threshold to filter small detections out'),


        # Nodes
        det,
    ])
