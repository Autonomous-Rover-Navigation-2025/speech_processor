from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='speech_processor',
            executable='wake_audio_processor',
            name='wake_audio_processor',
            output='screen'
        ),
        Node(
            package='speech_processor',
            executable='intent_classifier',
            name='intent_classifier',
            output='screen'
        ),
        Node(
            package='speech_processor',
            executable='location_matcher',
            name='location_matcher',
            output='screen'
        ),
        Node(
            package='speech_processor',
            executable='llm_responder',
            name='llm_responder',
            output='screen' #parameters=[{'endpoint': 'http://localhost:11434/api/generate'}]
        ),
        # Node(
        #     package='speech_processor',
        #     executable='text_speech',
        #     name='text_speech',
        #     output='screen'
        # ),
        Node(
            package='speech_processor',
            executable='tts_publisher',
            name='tts_publisher',
            output='screen' 
        ),
    ])
