from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='audio_tools',
            executable='audio_capture_node',
            name='audio_capture',
            parameters=[{
                'sample_format': 'S16LE',
                'channels': 1,
                'sample_rate': 16000,
                'device': -1,  # Use default device
                'chunk_size': 1024,
            }],
            output='screen',
        ),
        
        Node(
            package='audio_tools',
            executable='vad_node',
            name='vad',
            parameters=[{
                'energy_threshold': 0.01,
                'hold_time': 0.5,
                'min_samples': 160,
            }], # Office, close mic
            output='screen',
        ),
    ])