from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_first_package',
            executable='simple_publisher',
            name='simple_publisher',
            output='screen',
            respawn=True,
            parameters=[
                {'message_texte': 'Salut depuis le launch file'},
                {'frequence': 3.0},
            ],
        ),
        Node(
            package='my_first_package',
            executable='simple_subscriber',
            name='simple_subscriber',
            output='screen'
        ),
    ])
