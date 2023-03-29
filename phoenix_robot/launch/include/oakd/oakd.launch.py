import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def launch_setup(context, *args, **kwargs):
    log_level = 'info'
    if (context.environment.get('DEPTHAI_DEBUG') == '1'):
        log_level = 'debug'

    params_file = LaunchConfiguration("params_file")

    name = LaunchConfiguration('name').perform(context)

    return [
        ComposableNodeContainer(
            name=name + "_container",
            namespace="",
            package="rclcpp_components",
            executable="component_container",
            composable_node_descriptions=[
                ComposableNode(
                    package="depthai_ros_driver",
                    plugin="depthai_ros_driver::Camera",
                    name=name,
                    parameters=[params_file],
                    remappings=[
                        ('/oak/rgb/image_raw', '/camera/mid/rgb'),
                        ('/oak/rgb/image_raw/compressed', '/camera/mid/rgb/compressed'),
                        ('/oak/rgb/image_raw/theora', '/camera/mid/rgb/compressed/theora'),
                        ('/oak/rgb/camera_info', '/camera/mid/camera_info'),
                        ('/oak/imu/data', '/camera/mid/imu')]
                )
            ],
            arguments=['--ros-args', '--log-level', log_level],
            output="both",
        )
    ]


def generate_launch_description():
    pkg_phnx_robot = get_package_share_directory("phoenix_robot")

    declared_arguments = [
        DeclareLaunchArgument("name", default_value="oak"),
        DeclareLaunchArgument("params_file",
                              default_value=os.path.join(pkg_phnx_robot, 'config', 'oakd', 'camera.yaml')),
    ]

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
