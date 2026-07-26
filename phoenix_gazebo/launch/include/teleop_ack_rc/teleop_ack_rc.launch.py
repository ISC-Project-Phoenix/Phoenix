import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_teleop_ack_rc = get_package_share_directory('teleop_ack_rc')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    return LaunchDescription([
        # RC Joy Node
        Node(
            package='teleop_ack_rc',
            executable='rc_joy',
            name='rc_joy_node',
            output='screen',
            parameters=[os.path.join(pkg_teleop_ack_rc, 'config', 'params.yaml')]
        ),

        # Teleop Ack RC Node (logic bridge)
        Node(
            package='teleop_ack_rc',
            executable='teleop_ack_rc',
            name='teleop_ack_rc_node',
            output='screen',
            parameters=[os.path.join(pkg_teleop_ack_rc, 'config', 'params.yaml')]
        )
    ])