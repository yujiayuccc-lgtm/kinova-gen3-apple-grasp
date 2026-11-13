from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='apple_grasp_scene',
            executable='detect_apple_service.py',
            name='detect_apple_service',
            output='screen',
        ),
        Node(
            package='apple_grasp_scene',
            executable='pick_apple_action_server.py',
            name='pick_apple_action_server',
            output='screen',
        ),
    ])
