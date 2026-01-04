"""Launch file for the home assistant nodes."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description with all assistant nodes."""

    # Declare launch arguments
    conversation_timeout_arg = DeclareLaunchArgument(
        'conversation_timeout',
        default_value='30.0',
        description='Seconds to wait for follow-up before ending conversation'
    )

    max_history_turns_arg = DeclareLaunchArgument(
        'max_history_turns',
        default_value='10',
        description='Maximum number of conversation turns to keep in history'
    )

    location_arg = DeclareLaunchArgument(
        'location',
        default_value='Seattle, WA',
        description='Location for context-aware responses'
    )

    timezone_arg = DeclareLaunchArgument(
        'timezone',
        default_value='America/Los_Angeles',
        description='Timezone for time-related queries'
    )

    tts_voice_arg = DeclareLaunchArgument(
        'tts_voice',
        default_value='en_US-lessac-medium',
        description='Piper TTS voice model name'
    )

    tts_rate_arg = DeclareLaunchArgument(
        'tts_rate',
        default_value='1.0',
        description='TTS speech rate (0.5-2.0, lower is faster)'
    )

    return LaunchDescription([
        # Launch arguments
        conversation_timeout_arg,
        max_history_turns_arg,
        location_arg,
        timezone_arg,
        tts_voice_arg,
        tts_rate_arg,

        # Nodes
        Node(
            package='assistant',
            executable='speech_node',
            name='speech_recognition',
            output='screen',
        ),
        Node(
            package='assistant',
            executable='wake_node',
            name='wake_detection',
            output='screen',
        ),
        Node(
            package='assistant',
            executable='llm_node',
            name='llm',
            output='screen',
            parameters=[{
                'conversation_timeout': LaunchConfiguration('conversation_timeout'),
                'max_history_turns': LaunchConfiguration('max_history_turns'),
                'location': LaunchConfiguration('location'),
                'timezone': LaunchConfiguration('timezone'),
            }],
        ),
        Node(
            package='assistant',
            executable='tts_node',
            name='tts',
            output='screen',
            parameters=[{
                'voice': LaunchConfiguration('tts_voice'),
                'length_scale': LaunchConfiguration('tts_rate'),
            }],
        ),
    ])
