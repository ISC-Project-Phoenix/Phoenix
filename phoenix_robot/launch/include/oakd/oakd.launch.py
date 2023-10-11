import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import ComposableNodeContainer, Node, SetRemap
from launch_ros.descriptions import ComposableNode


# Launches the oak-d driver, as well as its tf publisher
def generate_launch_description():
    oakd_package = get_package_share_directory("depthai_ros_driver")
    phnx_package = get_package_share_directory("phoenix_robot")

    camera = GroupAction(
        actions=[
            # Topic remaps
            SetRemap('/oak/stereo/image_raw', '/camera/mid/depth'),
            SetRemap('/oak/stereo/camera_info', '/camera/mid/depth/camera_info'),
            SetRemap('/oak/stereo/image_raw/compressed', '/camera/mid/depth/compressed'),
            SetRemap('/oak/rgb/image_raw', '/camera/mid/rgb'),
            SetRemap('/oak/rgb/image_raw/compressed', '/camera/mid/rgb/compressed'),
            SetRemap('/oak/rgb/image_raw/theora', '/camera/mid/rgb/compressed/theora'),
            SetRemap('/oak/rgb/camera_info', '/camera/mid/rgb/camera_info'),
            SetRemap('/oak/imu/data', '/camera/mid/imu'),
            SetRemap('/robot_description', '/camera/mid/robot_description'),

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(oakd_package, 'launch', 'camera.launch.py')),
                launch_arguments={
                    'params_file': os.path.join(phnx_package, 'config', 'oakd', 'camera.yaml'),
                    'camera_model': 'OAK-D-S2'
                }.items()
            )]
    )

    declared_arguments = [
        DeclareLaunchArgument("name", default_value="oak"),
    ]

    return LaunchDescription([
        camera,
    ])
