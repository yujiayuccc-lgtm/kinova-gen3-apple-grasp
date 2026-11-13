from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    task_config = PathJoinSubstitution([
        FindPackageShare('apple_grasp_scene'),
        'config',
        'task_manager.yaml',
    ])
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
            parameters=[task_config],
        ),
    ])
