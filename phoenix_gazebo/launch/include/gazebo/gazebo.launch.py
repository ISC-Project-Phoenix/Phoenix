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
    pkg_ros_ign_gazebo = get_package_share_directory('ros_gz_sim')

    gazebo_world = LaunchConfiguration(
        'gazebo_world', default='purdue_gp_track.sdf')

    # Nodes
    ign_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_ign_gazebo, 'launch',
                         'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': [launch.substitutions.PathJoinSubstitution([pkg_phoenix_gazebo + '/worlds/', gazebo_world]),
                        launch.substitutions.TextSubstitution(text=" -r")]
        }.items()
    )

    ign_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/world/phoenix_world/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/model/phoenix/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/model/phoenix/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/model/phoenix/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/model/phoenix/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/mid_rgbd_camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/mid_rgbd_camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/mid_rgbd_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/sky_rgbd_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/sky_rgbd_camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/mid_rgbd_camera_imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
        ],
        output='screen',
        remappings=[
            ('/world/phoenix_world/clock', '/clock'),
            ('/model/phoenix/tf', '/tf'),
            ('/model/phoenix/cmd_vel', '/robot/cmd_vel'),
            ('/model/phoenix/odometry', '/odom_can'),
            ('/model/phoenix/joint_state', 'joint_states'),
            # Remap our cameras to match design docs
            ('/mid_rgbd_camera/image', '/camera/mid/rgb'),
            ('/mid_rgbd_camera/depth_image', '/camera/mid/depth'),
            ('/mid_rgbd_camera/camera_info', '/camera/mid/rgb/camera_info'),
            ('/mid_rgbd_camera_imu', '/camera/mid/imu'),
            ('/sky_rgbd_camera/image', '/camera/score/rgb'),
            ('/sky_rgbd_camera/camera_info', '/camera/score/camera_info'),
        ])

    ign_spawn_robot = Node(package='ros_gz_sim',
                           executable='create',
                           arguments=[
                               '-name', 'phoenix', '-x', '-24.6', '-y', '-45.1', '-z', '0', '-Y',
                               '-0.556156', '-topic', 'robot_description'
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
