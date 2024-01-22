import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import ComposableNodeContainer, Node, SetRemap
from launch_ros.descriptions import ComposableNode


# Launches two cameras. These cameras are discriminated using their exact IDs given in the cameral and camerar configs.
# These must be changed if not using the exact cameras on phoenix.
def generate_launch_description():
    oakd_package = get_package_share_directory("depthai_ros_driver")
    phnx_package = get_package_share_directory("phoenix_robot")

    cameral = GroupAction(
        actions=[
            # Topic remaps
            SetRemap('/oakl/stereo/image_raw', '/camera/left/depth'),
            SetRemap('/oakl/stereo/camera_info', '/camera/left/depth/camera_info'),
            SetRemap('/oakl/stereo/image_raw/compressed', '/camera/left/depth/compressed'),
            SetRemap('/oakl/rgb/image_raw', '/camera/left/rgb'),
            SetRemap('/oakl/rgb/image_raw/compressed', '/camera/left/rgb/compressed'),
            SetRemap('/oakl/rgb/image_raw/theora', '/camera/left/rgb/compressed/theora'),
            SetRemap('/oakl/rgb/camera_info', '/camera/left/rgb/camera_info'),
            SetRemap('/oakl/imu/data', '/camera/left/imu'),
            SetRemap('/robot_description', '/camera/left/robot_description'),

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(oakd_package, 'launch', 'camera.launch.py')),
                launch_arguments={
                    'params_file': os.path.join(phnx_package, 'config', 'oakd', 'cameral.yaml'),
                    'name': "oakl"
                }.items()
            )]
    )

    camerar = GroupAction(
        actions=[
            # Topic remaps
            SetRemap('/oakr/stereo/image_raw', '/camera/right/depth'),
            SetRemap('/oakr/stereo/camera_info', '/camera/right/depth/camera_info'),
            SetRemap('/oakr/stereo/image_raw/compressed', '/camera/right/depth/compressed'),
            SetRemap('/oakr/rgb/image_raw', '/camera/right/rgb'),
            SetRemap('/oakr/rgb/image_raw/compressed', '/camera/right/rgb/compressed'),
            SetRemap('/oakr/rgb/image_raw/theora', '/camera/right/rgb/compressed/theora'),
            SetRemap('/oakr/rgb/camera_info', '/camera/right/rgb/camera_info'),
            SetRemap('/oakr/imu/data', '/camera/right/imu'),
            SetRemap('/robot_description', '/camera/right/robot_description'),

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(oakd_package, 'launch', 'camera.launch.py')),
                launch_arguments={
                    'params_file': os.path.join(phnx_package, 'config', 'oakd', 'camerar.yaml'),
                    'name': "oakr"
                }.items()
            )]
    )

    return LaunchDescription([
        cameral,
        camerar
    ])
