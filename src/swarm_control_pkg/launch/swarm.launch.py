from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='swarm_control_pkg',
            executable='swarm_api',
            name='swarm_api',
            output='screen',
        ),
    ])
