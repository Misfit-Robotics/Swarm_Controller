from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    agent_id_arg = DeclareLaunchArgument('agent_id', default_value='drone_001',
                                         description='Unique agent identifier')
    sim_speed_arg = DeclareLaunchArgument('sim_speed', default_value='1.0',
                                          description='Simulation speed multiplier (1.0=realtime, 2.0=2x, etc.)')

    agent_id = LaunchConfiguration('agent_id')
    sim_speed = LaunchConfiguration('sim_speed')

    return LaunchDescription([
        agent_id_arg,
        sim_speed_arg,

        Node(
            package='sensor_sim_pkg',
            executable='drone_sim',
            name=['drone_sim_', agent_id],
            parameters=[{'agent_id': agent_id, 'sim_speed': sim_speed}],
            output='screen',
        ),

        Node(
            package='comm_sim_pkg',
            executable='comm_node',
            name=['comm_node_', agent_id],
            parameters=[{'agent_id': agent_id}],
            output='screen',
        ),

        Node(
            package='decision_pkg',
            executable='decision_node',
            name=['decision_node_', agent_id],
            parameters=[{'agent_id': agent_id}],
            output='screen',
        ),
    ])
