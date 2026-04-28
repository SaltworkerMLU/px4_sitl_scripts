# launch/inner_loop_nodes.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    attitude_node = Node(
        package='px4_sitl_scripts',
        executable='inner_loop_nodes',
        name='attitude_controller',
        output='screen',
        arguments=['attitude']
    )

    rate_node = Node(
        package='px4_sitl_scripts',
        executable='inner_loop_nodes',
        name='rate_controller',
        output='screen',
        arguments=['rate']
    )

    mixer_node = Node(
        package='px4_sitl_scripts',
        executable='inner_loop_nodes',
        name='mixer',
        output='screen',
        arguments=['mixer']
    )

    return LaunchDescription([
        attitude_node,
        rate_node,
        mixer_node
    ])