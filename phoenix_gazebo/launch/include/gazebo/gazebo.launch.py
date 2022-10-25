import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import launch.substitutions
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


def generate_launch_description():
    # ROS packages
    pkg_phoenix_description = get_package_share_directory(
        'phoenix_description')
    pkg_phoenix_gazebo = get_package_share_directory('phoenix_gazebo')
    pkg_ros_ign_gazebo = get_package_share_directory('ros_ign_gazebo')

    gazebo_world = LaunchConfiguration(
        'gazebo_world', default='purdue_gp_track.sdf')

    # Nodes
    ign_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_ign_gazebo, 'launch',
                         'ign_gazebo.launch.py')),
        launch_arguments={
            'ign_args': launch.substitutions.PathJoinSubstitution([pkg_phoenix_gazebo + '/worlds/', gazebo_world])
        }.items()
    )

    ign_bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        arguments=[
            '/world/phoenix_world/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            '/model/phoenix/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
            '/model/phoenix/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
            '/model/phoenix/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
            '/model/phoenix/joint_state@sensor_msgs/msg/JointState[ignition.msgs.Model',
            '/rgbd_camera/image@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/rgbd_camera/depth_image@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/rgbd_camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
            '/rgbd_camera/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked',
        ],
        output='screen',
        remappings=[
            ('/world/phoenix_world/clock', '/clock'),
            ('/model/phoenix/tf', '/tf'),
            ('/model/phoenix/cmd_vel', '/robot/cmd_vel'),
            ('/model/phoenix/odometry', '/odom'),
            ('/model/phoenix/joint_state', 'joint_states')
        ])
        
    ign_spawn_robot = Node(package='ros_ign_gazebo',
                           executable='create',
                           arguments=[
                               '-name', 'phoenix', '-x', '0', '-z', '0', '-Y',
                               '0', '-topic', 'robot_description'
                           ],
                           output='screen')

    return LaunchDescription([
        # Nodes
        ign_gazebo,
        ign_bridge,
        ign_spawn_robot,

        DeclareLaunchArgument('gazebo_world',
                              default_value='purdue_gp_track.sdf',
                              description='gazebo world to load'),
    ])
