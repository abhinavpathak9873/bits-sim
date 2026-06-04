from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='dynamic_nav_benchmark',
            executable='baseline_go_to_goal',
            output='screen',
            parameters=[{
                'max_linear_speed': 0.28,
                'max_angular_speed': 0.9,
            }],
        )
    ])
