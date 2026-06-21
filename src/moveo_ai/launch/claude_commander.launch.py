from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'anthropic_api_key',
            default_value='',
            description='Anthropic API key. Leave blank to use the ANTHROPIC_API_KEY env var.',
        ),
        Node(
            package='moveo_ai',
            executable='claude_commander',
            name='claude_commander',
            output='screen',
            parameters=[
                {
                    'use_sim_time': True,
                    'anthropic_api_key': LaunchConfiguration('anthropic_api_key'),
                }
            ],
        ),
    ])
