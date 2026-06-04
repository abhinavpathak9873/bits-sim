from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='dynamic_nav_benchmark',
            executable='baseline_vfh',
            output='screen',
            parameters=[{
                'max_linear_speed': 0.26,
                'max_angular_speed': 1.0,
                'influence_distance_m': 2.1,
                'front_stop_m': 0.45,
                'repulsive_gain': 1.25,
            }],
        )
    ])
