from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='dynamic_nav_benchmark',
            executable='baseline_bug2',
            output='screen',
            parameters=[{
                'max_linear_speed': 0.24,
                'max_angular_speed': 0.9,
                'obstacle_enter_m': 0.85,
                'obstacle_exit_m': 1.25,
                'desired_wall_distance_m': 0.75,
            }],
        )
    ])
