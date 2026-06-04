from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='dynamic_nav_benchmark',
            executable='baseline_follow_gap',
            output='screen',
            parameters=[{
                'max_linear_speed': 0.26,
                'max_angular_speed': 1.0,
                'free_distance_m': 1.15,
                'bubble_radius_rad': 0.28,
                'heading_weight': 0.65,
            }],
        )
    ])
