from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='mall'),
        Node(
            package='dynamic_nav_benchmark',
            executable='baseline_astar',
            output='screen',
            parameters=[{
                'world': LaunchConfiguration('world'),
                'max_linear_speed': 0.24,
                'max_angular_speed': 0.9,
                'grid_resolution_m': 0.35,
                'obstacle_inflation_m': 0.55,
                'waypoint_tolerance_m': 0.45,
                'front_stop_m': 0.55,
            }],
        )
    ])
