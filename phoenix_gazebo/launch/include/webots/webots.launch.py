import os
import pathlib
import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from launch_ros.actions import Node
from webots_ros2_driver.urdf_spawner import URDFSpawner, get_webots_driver_node
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController


def generate_launch_description():
    world = LaunchConfiguration('world')
    phnx_desc = get_package_share_directory("phoenix_description")
    phnx_gazebo = get_package_share_directory("phoenix_gazebo")
    urdf_path = os.path.join(phnx_desc, 'urdf', 'phoenix.urdf')

    robot_driver = WebotsController(
        robot_name='Phoenix',
        parameters=[
            {'robot_description': urdf_path,
             'use_sim_time': True}
        ],
        respawn=True
    )

    # The WebotsLauncher is a Webots custom action that allows you to start a Webots simulation instance.
    # It searches for the Webots installation in the path specified by the `WEBOTS_HOME` environment variable and default installation paths.
    # The Ros2Supervisor node is mandatory to spawn an URDF robot.
    # The accepted arguments are:
    # - `world` (str): Path to the world to launch.
    # - `gui` (bool): Whether to display GUI or not.
    # - `mode` (str): Can be `pause`, `realtime`, or `fast`.
    # - `ros2_supervisor` (bool): Spawn the `Ros2Supervisor` custom node that communicates with a Supervisor robot in the simulation.
    webots = WebotsLauncher(
        world=PathJoinSubstitution([phnx_gazebo, 'webots_project', 'worlds', world]),
        ros2_supervisor=True
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value='purdue_track.wbt',
        ),
        # Starts Webots
        webots,

        # Starts the Ros2Supervisor node created with the WebotsLauncher
        webots._supervisor,
        robot_driver,

        # This action will kill all nodes once the Webots simulation has exited
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])
