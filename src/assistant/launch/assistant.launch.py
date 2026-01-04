"""Launch file for the home assistant nodes."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description with all assistant nodes."""
    return LaunchDescription([
        Node(
            package='assistant',
            executable='speech_node',
            name='speech_recognition',
            output='screen',
        ),
        Node(
            package='assistant',
            executable='trigger_node',
            name='trigger_detection',
            output='screen',
        ),
        Node(
            package='assistant',
            executable='llm_node',
            name='llm',
            output='screen',
        ),
    ])
